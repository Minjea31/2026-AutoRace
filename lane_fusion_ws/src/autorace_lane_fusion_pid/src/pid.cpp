#include "autorace_lane_fusion_pid/pid.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace lane_pid {
const std::vector<const char *> &groups() {
  static const std::vector<const char *> g = {"오차 정의", "PID 게인", "조향 제한", "속도", "안전"};
  return g;
}
const std::vector<Spec> &specs() {
  // Defaults: simulator LIMO (wheelbase 0.20 m, centre steering limit ~0.41 rad).
  static const std::vector<Spec> s = {
      {"lateral_weight", "오차 정의", "-",
       "가까운 점 횡오차 가중치. 차량 바로 앞(BEV 맨 아래) 중앙 경로까지의 좌우 거리(m, 좌+)에 곱함. "
       "차로 중앙 유지의 주 오차", 0.5, 0, 3, .05},
      {"heading_weight", "오차 정의", "m/rad",
       "헤딩 오차 가중치. 가까운 구간의 중앙 경로 방향과 차량 진행 방향의 각도(rad, 좌+)에 곱함. "
       "올리면 경로와 평행하게 맞추려는 힘이 커져 흔들림이 줄고 커브 진입이 빨라짐", 0.10, 0, 1.5, .01},
      {"lookahead_weight", "오차 정의", "-",
       "목표점 횡오차 가중치. 전방 목표점(lookahead)의 좌우 거리(m)에 곱함. 커브를 미리 보지만 "
       "크면 커브 안쪽을 깎음", 1.0, 0, 2, .05},
      {"far_heading_weight", "오차 정의", "m/rad",
       "목표점 선 각도 가중치. 전방 목표점(lookahead)에서 중앙 경로 방향과 차량 진행 방향의 각도(rad, 좌+)에 "
       "곱함. 커브 안에서는 커브가 끝날 때까지 값이 남아 조향을 유지(커브 중간에 직진하며 바깥으로 나가는 것 방지). "
       "0=끔", 0.0, 0, 1.5, .01},
      {"lateral_rate_limit", "오차 정의", "m/s",
       "횡오차 측정값의 최대 변화 속도. 정지선·노면 표시 오검출로 한 프레임에 크게 튀는 값을 "
       "이 속도로만 따라감. 차량 횡이동 속도보다 크게",
       0.6, .05, 5, .05},
      {"heading_rate_limit", "오차 정의", "rad/s",
       "헤딩 측정값의 최대 변화 속도. 가로선을 경로로 착각해 헤딩이 순간 ±1.5 rad로 튀는 것을 억제",
       2.0, .1, 10, .1},
      {"error_deadband", "오차 정의", "m",
       "불감대. 결합 오차의 절댓값이 이 값보다 작으면 0으로 봄(직선 미세 떨림 억제)",
       0.0, 0, .05, .001},
      {"kp", "PID 게인", "rad/m", "비례 게인 P. 결합 오차 1 m당 조향각. 크면 빠르게 복귀하지만 진동·커브 안쪽 깎기", 0.40, 0, 6, .05},
      {"ki", "PID 게인", "rad/(m·s)",
       "적분 게인 I. 한쪽으로 계속 남는 오차(정상상태 오차) 제거. 크면 느린 출렁임. 기본 0",
       0.0, 0, 3, .01},
      {"kd", "PID 게인", "rad·s/m", "미분 게인 D. 오차 변화 속도에 반대로 작용해 진동·오버슈트 감쇠", 0.06, 0, 1.5, .01},
      {"d_filter_tau", "PID 게인", "s",
       "미분 저역통과 필터 시정수. 크면 D가 부드럽지만 늦게 반응. 0=필터 없음", 0.05, 0, .5, .005},
      {"integral_limit", "PID 게인", "m·s", "적분 누적값 상한(±). 적분 와인드업 방지", 0.2, 0, 2, .01},
      {"curvature_ff", "PID 게인", "-",
       "곡률 피드포워드 비율. atan(축간거리×경로 곡률)을 이 비율만큼 미리 더함. 0=끔, 1=기하 조향 전부",
       0.0, 0, 1.5, .05},
      {"wheelbase", "조향 제한", "m", "축간거리. 곡률 피드포워드 계산용", 0.20, .05, .6, .005},
      {"max_steering", "조향 제한", "rad", "조향각 절댓값 상한(가상 중심 조향각). 시뮬 LIMO 약 0.41",
       0.40, .05, .7, .01},
      {"steering_rate", "조향 제한", "rad/s", "조향각 최대 변화율. 너무 낮으면 커브에서 늦음", 2.0, .1, 8, .1},
      {"speed", "속도", "m/s", "기본(최대) 주행 속도", 0.25, 0, 1, .01},
      {"min_speed", "속도", "m/s", "주행 중 최저 속도(감속 후에도 이 값 이상, 단 정지 조건은 0)", 0.08, 0, .5, .01},
      {"steer_slowdown", "속도", "1/rad", "조향 감속. 속도 = speed / (1 + 값×|조향각|)", 1.0, 0, 6, .1},
      {"curvature_slowdown", "속도", "m", "곡률 감속. 속도를 추가로 1 / (1 + 값×|곡률|)배", 0.0, 0, 2, .05},
      {"acceleration", "속도", "m/s²", "가속 제한. 감속은 즉시 적용", 0.5, .05, 3, .05},
      {"confidence_stop", "안전", "-",
       "인식 신뢰도가 이 값 이하인 검출은 유실로 봄(아래 hold_time/recover_time 단계 적용)",
       0.10, 0, .9, .01},
      {"confidence_full", "안전", "-", "인식 신뢰도가 이 값 이상이면 최대 속도(사이 구간은 비례 감속)",
       0.50, .05, 1, .01},
      {"timeout", "안전", "s",
       "이 시간 동안 /lane_fusion/target 메시지 자체가 없으면(인식 노드 정지) 즉시 정지", 0.30, .05, 2, .01},
      {"hold_time", "안전", "s",
       "경로 유실(차선 미검출) 후 이 시간까지는 마지막 경로로 정상 속도 유지(HOLD)", 0.20, 0, 1, .01},
      {"recover_time", "안전", "s",
       "유실이 hold_time~이 시간이면 recover_speed 이하 저속 복구, 넘으면 정지. "
       "lane_fusion track_memory 이하로", 0.50, .05, 2, .01},
      {"recover_speed", "안전", "m/s", "저속 복구 구간의 최대 속도", 0.15, 0, 1, .01},
      {"active", "안전", "0/1", "주행 허용. 0이면 속도 0(조향 계산은 계속). 튜너의 정지/주행 버튼", 1, 0, 1, 1},
  };
  return s;
}
const Spec &spec(const std::string &name) {
  for (const auto &s : specs())
    if (name == s.name)
      return s;
  throw std::out_of_range("Unknown PID parameter: " + name);
}
Config::Config() {
  for (const auto &s : specs())
    values[s.name] = s.value;
}
void Config::validate() const {
  for (const auto &s : specs()) {
    const auto it = values.find(s.name);
    if (it == values.end())
      throw std::invalid_argument(std::string("Missing parameter: ") + s.name);
    const double v = it->second;
    if (!std::isfinite(v) || v < s.min || v > s.max || (s.step == 1 && v != std::floor(v)))
      throw std::invalid_argument(std::string("Out of range: ") + s.name);
  }
  if ((*this)["confidence_stop"] >= (*this)["confidence_full"])
    throw std::invalid_argument("confidence_stop must be < confidence_full");
  if ((*this)["min_speed"] > (*this)["speed"])
    throw std::invalid_argument("min_speed must be <= speed");
  if ((*this)["hold_time"] > (*this)["recover_time"])
    throw std::invalid_argument("hold_time must be <= recover_time");
}
Config Config::load(const std::string &path, const std::string &node) {
  Config c;
  const auto root = YAML::LoadFile(path);
  auto params = root[node] ? root[node]["ros__parameters"] : YAML::Node();
  if (!params || !params.IsMap())
    throw std::invalid_argument(path + ": needs " + node + ": ros__parameters: map");
  for (const auto &kv : params) {
    const auto key = kv.first.as<std::string>();
    if (key == "use_sim_time" || key == "enable_vehicle_output")
      continue; // node launch options, not gains
    if (!c.values.count(key)) {
      double number;
      if (kv.second.IsScalar() && !YAML::convert<double>::decode(kv.second, number)) {
        c.options[key] = kv.second.Scalar(); // bool / string node option
        continue;
      }
      throw std::invalid_argument("Unknown PID parameter: " + key);
    }
    c.values[key] = kv.second.as<double>();
  }
  c.validate();
  return c;
}
void Config::save(const std::string &path, const std::string &node) const {
  validate();
  std::ofstream f(path);
  f << "# Lane PID controller (config/sim = simulator, config/real = real car). Written by lane_fusion_pid_tuner.\n"
       "# e = lateral_weight*near_lateral + heading_weight*near_heading + lookahead_weight*lateral\n"
       "#     + far_heading_weight*heading (path angle at the lookahead point)\n# "
       "steer = kp*e + ki*int(e) + kd*de/dt + ff\n";
  YAML::Emitter e;
  e << YAML::BeginMap << YAML::Key << node << YAML::Value << YAML::BeginMap << YAML::Key
    << "ros__parameters" << YAML::Value << YAML::BeginMap;
  for (const auto &s : specs()) {
    const double v = values.at(s.name);
    e << YAML::Key << s.name << YAML::Value;
    if (s.step == 1)
      e << double(std::lround(v)); // keep double type for ROS parameters
    else
      e << v;
  }
  for (const auto &[key, text] : options)
    e << YAML::Key << key << YAML::Value << text;
  e << YAML::EndMap << YAML::EndMap << YAML::EndMap;
  // yaml-cpp prints integral doubles as "1"; ROS would then declare an
  // integer. Force a decimal point on every scalar value.
  std::string text = e.c_str(), out;
  size_t start = 0;
  while (start < text.size()) {
    size_t end = text.find('\n', start);
    if (end == std::string::npos)
      end = text.size();
    std::string line = text.substr(start, end - start);
    const auto colon = line.find(": ");
    if (colon != std::string::npos) {
      const auto value = line.substr(colon + 2);
      if (!value.empty() && value.find_first_not_of("-0123456789") == std::string::npos)
        line += ".0";
    }
    out += line + '\n';
    start = end + 1;
  }
  f << out;
  if (!f)
    throw std::runtime_error("Cannot save YAML: " + path);
}

void Controller::reset() {
  t_ = {};
  last_ = {};
  have_prev_ = false;
  prev_error_ = prev_time_ = d_filtered_ = integral_ = 0;
  near_lateral_ = near_heading_ = far_lateral_ = far_heading_ = 0;
  lost_ = 1e9;
  confidence_ = 0;
}
void Controller::measure(const Measurement &m, const Config &c) {
  if (m.valid && m.detected && m.confidence > c["confidence_stop"]) {
    lost_ = 0;
    confidence_ = m.confidence;
  }
  // No path in this frame: keep the last steering target; step() slows or
  // stops by the time since the last good detection. A held path (valid but
  // not detected) still steers: perception moved it with the car.
  if (!m.valid)
    return;
  last_ = m;
  const double dt = m.time - prev_time_;
  const bool timed = have_prev_ && dt > 1e-4 && dt <= .5;
  // Measurement slew limit: a real vehicle cannot jump sideways or rotate
  // within one frame, so single-frame perception glitches are clipped.
  auto follow = [&](double &state, double raw, double rate) {
    state = timed ? state + std::clamp(raw - state, -rate * dt, rate * dt) : raw;
  };
  follow(near_lateral_, m.near_lateral, c["lateral_rate_limit"]);
  follow(near_heading_, m.near_heading, c["heading_rate_limit"]);
  follow(far_lateral_, m.lateral, c["lateral_rate_limit"]);
  follow(far_heading_, m.heading, c["heading_rate_limit"]);
  double e = c["lateral_weight"] * near_lateral_ + c["heading_weight"] * near_heading_ +
             c["lookahead_weight"] * far_lateral_ + c["far_heading_weight"] * far_heading_;
  const double band = c["error_deadband"];
  e = std::copysign(std::max(0., std::abs(e) - band), e);

  if (timed) {
    const double raw = (e - prev_error_) / dt;
    const double tau = c["d_filter_tau"];
    d_filtered_ += (tau > 0 ? dt / (tau + dt) : 1.) * (raw - d_filtered_);
  } else
    d_filtered_ = 0;
  prev_error_ = e;
  prev_time_ = m.time;
  have_prev_ = true;

  const double limit = c["max_steering"];
  t_.error = e;
  t_.p = c["kp"] * e;
  t_.d = c["kd"] * d_filtered_;
  t_.ff = c["curvature_ff"] * std::atan(c["wheelbase"] * m.curvature);
  // Conditional integration (anti-windup): accumulate only while the output
  // is not saturated, or when the new error pulls it back out of saturation.
  if (c["ki"] > 0 && timed) {
    const double lim = c["integral_limit"];
    const double candidate = std::clamp(integral_ + e * dt, -lim, lim);
    const double u = t_.p + c["ki"] * candidate + t_.d + t_.ff;
    if (std::abs(u) <= limit || (u > 0) != (e > 0))
      integral_ = candidate;
  } else if (c["ki"] <= 0)
    integral_ = 0;
  t_.i = c["ki"] * integral_;
  t_.target_steering = std::clamp(t_.p + t_.i + t_.d + t_.ff, -limit, limit);
}
const Terms &Controller::step(double dt, bool fresh, const Config &c, double speed_limit) {
  dt = std::clamp(dt, 0., .1);
  lost_ += dt; // measure() zeroes it on every good detection
  t_.lost = lost_;
  const bool drive = fresh && lost_ <= c["recover_time"];
  t_.tracking = drive;
  if (!drive) {
    // Stopped: forget I and D history so reacquisition starts clean. The
    // output stage brings steering back to centre at the rate limit.
    have_prev_ = false;
    integral_ = d_filtered_ = 0;
  }
  const double desired = drive ? t_.target_steering : 0.;
  const double rate = c["steering_rate"] * dt;
  t_.steering += std::clamp(desired - t_.steering, -rate, rate);
  t_.steering = std::clamp(t_.steering, -c["max_steering"], c["max_steering"]);

  double v = 0;
  if (drive && c["active"] > .5) {
    const double q = std::clamp((confidence_ - c["confidence_stop"]) /
                                    (c["confidence_full"] - c["confidence_stop"]),
                                0., 1.);
    v = c["speed"] * q / (1 + c["steer_slowdown"] * std::abs(t_.steering)) /
        (1 + c["curvature_slowdown"] * std::abs(last_.curvature));
    if (q > 0)
      v = std::clamp(v, c["min_speed"], c["speed"]);
    if (lost_ > c["hold_time"])
      v = std::min(v, c["recover_speed"]);
  }
  // Braking (and every stop) is immediate; only acceleration is limited.
  v = std::min(v, std::max(0., speed_limit));
  t_.speed = v < t_.speed ? v : std::min(v, t_.speed + c["acceleration"] * dt);
  return t_;
}
} // namespace lane_pid
