// C++ 코어 단위 테스트(ROS 없이): 파라미터 동기화, 계획, PD 추종, 후진 탈출
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <unistd.h>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "autorace_objavoid_v3_cpp/mission.hpp"
#include "autorace_objavoid_v3_cpp/params.hpp"
#include "autorace_objavoid_v3_cpp/planner.hpp"
#include "autorace_objavoid_v3_cpp/start_gate.hpp"

using namespace objavoid;

// Python 공통 설정(objavoid_v3.yaml = Python Params 기본값)과 C++ 기본값이 이름·값 모두 같아야 한다
TEST(Params, MatchesPythonConfig)
{
  const YAML::Node y = YAML::LoadFile(PY_PARAMS_FILE)["objavoid_v3_node"]["ros__parameters"];
  ASSERT_TRUE(y);
  Params p;
  std::map<std::string, std::vector<double>> cpp;
#define OBJAVOID_COLLECT_D(n) cpp[#n] = {p.n};
#define OBJAVOID_COLLECT_I(n) cpp[#n] = {static_cast<double>(p.n)};
#define OBJAVOID_COLLECT_B(n) cpp[#n] = {p.n ? 1.0 : 0.0};
#define OBJAVOID_COLLECT_V(n) cpp[#n] = p.n;
#define OBJAVOID_COLLECT(t, n, def) OBJAVOID_COLLECT_##t(n)
  OBJAVOID_PARAMS(OBJAVOID_COLLECT)
#undef OBJAVOID_COLLECT
  // 노드 전용(코어 Params가 아닌) 파라미터
  const std::set<std::string> node_only{"command_topic", "scan_topic", "odom_topic", "auto_activate", "marker_rate"};
  size_t n_yaml = 0;
  for (auto it = y.begin(); it != y.end(); ++it) {
    const std::string k = it->first.as<std::string>();
    if (node_only.count(k)) {continue;}
    ++n_yaml;
    ASSERT_TRUE(cpp.count(k)) << "C++ Params에 없음: " << k;
    std::vector<double> v;
    if (it->second.IsSequence()) {
      for (const auto & e : it->second) {v.push_back(e.as<double>());}
    } else if (it->second.as<std::string>() == "true" || it->second.as<std::string>() == "false") {
      v = {it->second.as<bool>() ? 1.0 : 0.0};
    } else {
      v = {it->second.as<double>()};
    }
    ASSERT_EQ(v.size(), cpp[k].size()) << k;
    for (size_t q = 0; q < v.size(); ++q) {EXPECT_NEAR(v[q], cpp[k][q], 1e-9) << k;}
  }
  EXPECT_EQ(n_yaml, cpp.size()) << "yaml과 C++ 파라미터 개수가 다름";
}

TEST(Planner, AvoidsObstacleWithinKinematics)
{
  Params p;
  PriorBoundary bd({-1, 6}, {0.768, 0.768}, {-0.768, -0.768});
  Planner pl(p, &bd);
  const std::vector<Box> boxes{{1.2, 1.5, -0.15, 0.15}};  // 가운데 차로 정면
  const PathPtr path = pl.plan({0.0, 0.0, 0.0, 0.0}, boxes, nullptr, nullptr);
  ASSERT_TRUE(path);
  const auto obs = obstacle_array(boxes);
  for (size_t q = 0; q < path->s.size(); ++q) {
    EXPECT_GE(body_clearance(path->s[q], path->d[q], path->psi[q], obs, p), p.margin_obstacle - 1e-6);
    if (path->s[q] > p.start_relax) {EXPECT_LE(std::abs(path->kappa[q]), 1.0 / p.plan_min_radius + 1e-6);}
  }
  // 장애물 옆에서는 가운데에서 벗어나 있어야 함
  EXPECT_GT(std::abs(path->d_at(1.35)), 0.2);
}

TEST(Controller, ConvergesOnStraightPath)
{
  Params p;
  Path path;
  for (int k = 0; k <= 100; ++k) {
    path.s.push_back(k * 0.05);
    path.d.push_back(0.0);
    path.d1.push_back(0.0);
    path.d2.push_back(0.0);
    path.psi.push_back(0.0);
    path.kappa.push_back(0.0);
    path.clear.push_back(1.0);
    path.dsteer.push_back(0.0);
    path.bclear.push_back(1.0);
  }
  path.complete = true;
  PathTracker c(p);
  double s = 0, d = 0.1, psi = 0.0;
  for (int k = 0; k < 400; ++k) {
    double v, delta;
    CtrlInfo info;
    c.step(path, s, d, psi, 0.04, p.v_max, -1e9, v, delta, info);
    s += v * std::cos(psi) * 0.04;
    d += v * std::sin(psi) * 0.04;
    psi += v * std::tan(delta) / p.wheelbase * 0.04;
    if (s > 4.0) {break;}
  }
  EXPECT_LT(std::abs(d), 0.02);
  EXPECT_LT(std::abs(psi), 0.05);
}

// 앞이 도로 폭 전체로 막히면 경로 끝에서 서고, reverse_wait 뒤 후진해 reverse_dist 물러난다
static void run_blocked_escape(bool reverse_enabled)
{
  Params p;
  p.use_lane = false;
  p.reverse_enable = reverse_enabled;
  p.reverse_max = 1;
  ObjAvoidCore core(p);
  const int N = 400;
  std::vector<double> ang(N);
  for (int k = 0; k < N; ++k) {ang[k] = -1.9 + 3.8 * k / (N - 1);}
  const double wall_x = 0.9;
  auto scan = [&](double x) {
      std::vector<float> r(N);
      for (int k = 0; k < N; ++k) {
        const double a = ang[k];
        r[k] = (std::abs((wall_x - x) * std::tan(a)) < 1.0 && std::abs(a) < 1.4) ?
          static_cast<float>((wall_x - x) / std::cos(a)) : 8.0f;
      }
      return r;
    };
  double x = 0, t = 0, v = 0;
  std::vector<State> states;
  std::vector<double> xs;
  while (t < 12.0) {
    core.on_odom(t, x, 0.0, 0.0, v, 0.0);
    if (t == 0.0) {ASSERT_TRUE(core.activate(t));}
    if (std::abs(t * 10 - std::round(t * 10)) < 1e-6) {
      core.process_scan(t, scan(x), ang[0], ang[1] - ang[0], t);
    }
    double vc = 0, sc = 0;
    v = core.control(t, 0.04, vc, sc) ? vc : 0.0;
    x += v * 0.04;
    states.push_back(core.state);
    xs.push_back(x);
    t = std::round((t + 0.04) * 1e6) / 1e6;
  }
  size_t i = 0;
  while (i < states.size() && states[i] != State::REVERSE) {++i;}
  if(!reverse_enabled) {
    EXPECT_EQ(i,states.size());
    EXPECT_EQ(core.state,State::STOP);
    for(size_t k=1;k<xs.size();++k) EXPECT_GE(xs[k],xs[k-1]);
    EXPECT_EQ(YAML::Load(core.status_json())["recovery"]["policy"].as<std::string>(),"normal");
    return;
  }
  ASSERT_LT(i, states.size()) << "후진하지 않음";
  const double xmax = *std::max_element(xs.begin(), xs.begin() + i);
  const double xmin = *std::min_element(xs.begin() + i, xs.end());
  EXPECT_LT(xmax + p.body_front, wall_x + p.lidar_x);  // 벽에 닿기 전에 섬
  EXPECT_NEAR(xmax - xmin, p.reverse_dist, 0.03);
  EXPECT_EQ(core.state, State::STOP);
}

TEST(Mission, ReverseEscapeWhenBlocked) {run_blocked_escape(true);}
TEST(Mission, NormalModeStopsWhenBlockedWithoutReversing) {
  EXPECT_FALSE(Params().reverse_enable);
  run_blocked_escape(false);
}

// 갈림길: 표지판 방향으로 섬 좌/우 갈래에 들어가 DONE.
// known_island=true: 섬 위치 실측(fork_island_s=2.0) — 표지판이 없으면 섬 앞에서 기다렸다가 no_sign.
// known_island=false: 섬 위치 모름 — 표지판 전에는 계속 주행, 표지판을 본 위치 + fork_sign_to_island로 섬 추정.
struct ForkRun
{
  std::string done; double d_at_island = 0, min_island_gap = 1e9, island = 0, max_x = 0; bool waited = false;
};

static ForkRun run_fork(int sign, double sign_t0 = 0.5, bool known_island = true)
{
  Params p;
  p.use_lane = false;
  p.section_end = 2.0;
  p.fork_island_s = known_island ? 2.0 : 0.0;
  p.fork_sign_timeout = sign_t0 > 1.0 ? 20.0 : 1.0;
  p.boundary_s = {-1.0, 6.5};
  p.boundary_left = {0.75, 0.75};
  p.boundary_right = {-0.75, -0.75};
  p.lane_centers = {-0.5, 0.0, 0.5};
  ObjAvoidCore core(p);
  const int N = 400;
  std::vector<float> scan(N, 8.0f);
  const double amin = -1.9, ainc = 3.8 / (N - 1);
  double x = 0, y = 0, yaw = 0, v = 0, t = 0, delta = 0;
  ForkRun out;
  out.island = 2.0;
  while (t < 40.0) {
    core.on_odom(t, x, y, yaw, v, v * std::tan(delta) / p.wheelbase);
    if (t == 0.0) {core.activate(t);}
    if (std::abs(t * 10 - std::round(t * 10)) < 1e-6) {
      core.process_scan(t, scan, amin, ainc, t);
      if (sign != 0 && t > sign_t0 && t < sign_t0 + 0.5) {core.on_sign(t, sign < 0);}  // YOLO가 몇 번 발행
    }
    if (!known_island && core.island_known()) {out.island = core.island_s();}
    double vc = 0, sc = 0;
    const bool pub = core.control(t, 0.04, vc, sc);
    if (core.state == State::DONE) {out.done = core.done_reason; break;}
    if (core.reason == "wait_sign") {out.waited = true;}
    v = pub ? vc : 0.0;
    delta = sc;
    x += v * std::cos(yaw) * 0.04;
    y += v * std::sin(yaw) * 0.04;
    yaw += v * std::tan(delta) / p.wheelbase * 0.04;
    out.max_x = std::max(out.max_x, x);
    if (known_island || core.island_known()) {  // 회전한 차체와 섬 사이 실제 간격(SAT)
      const std::vector<Obs> isl = obstacle_array({Box{out.island, out.island + p.fork_island_len,
          -p.fork_island_half, p.fork_island_half}});
      out.min_island_gap = std::min(out.min_island_gap, body_clearance(x, y, yaw, isl, p));
    }
    if (x + p.body_front < out.island) {out.d_at_island = y;}
    t = std::round((t + 0.04) * 1e6) / 1e6;
  }
  std::printf("[fork sign=%d t0=%.0f known=%d] t=%.1f x=%.2f y=%.2f done=%s island=%.2f d_at_island=%.2f "
    "island_gap=%.3f waited=%d\n", sign, sign_t0, known_island, t, x, y, out.done.c_str(), out.island,
    out.d_at_island, out.min_island_gap, out.waited);
  return out;
}

TEST(Mission, ForkLeftSign)
{
  const ForkRun r = run_fork(1);
  EXPECT_EQ(r.done, "fork_left");
  EXPECT_GT(r.d_at_island, 0.3);
  EXPECT_GT(r.min_island_gap, 0.05);
  EXPECT_FALSE(r.waited);
}

TEST(Mission, ForkRightSign)
{
  const ForkRun r = run_fork(-1);
  EXPECT_EQ(r.done, "fork_right");
  EXPECT_LT(r.d_at_island, -0.3);
  EXPECT_GT(r.min_island_gap, 0.05);
}

TEST(Mission, ForkNoSignWaitsThenHandsOver)  // 섬 위치를 준 경우
{
  const ForkRun r = run_fork(0);
  EXPECT_TRUE(r.waited);
  EXPECT_EQ(r.done, "no_sign");
  EXPECT_LT(std::abs(r.d_at_island), 0.1);  // 방향을 모르면 가운데에서 대기
}

TEST(Mission, ForkLateSign)  // 섬 앞에서 기다리는 중에 표지판이 늦게 와도 갈래로 들어간다
{
  const ForkRun l = run_fork(1, 9.0);
  EXPECT_EQ(l.done, "fork_left");
  EXPECT_GT(l.min_island_gap, 0.05);
  const ForkRun r = run_fork(-1, 9.0);
  EXPECT_EQ(r.done, "fork_right");
  EXPECT_GT(r.min_island_gap, 0.05);
}

TEST(Mission, ForkUnknownIslandDrivesUntilSign)  // 섬 위치 모름: 표지판 전에는 멈추지 않고 계속 주행
{
  const ForkRun none = run_fork(0, 0.5, false);
  EXPECT_FALSE(none.waited);
  EXPECT_GT(none.max_x, 3.0);  // 예전 섬 위치(2.0)에서 서지 않음
  // 주행 중(약 4 s, 차 앞 약 0.6 m)에 표지판: 그 위치 + 2.0 m를 섬으로 보고 갈래 진입
  const ForkRun l = run_fork(1, 4.0, false);
  EXPECT_EQ(l.done, "fork_left");
  EXPECT_GT(l.min_island_gap, 0.05);
  const ForkRun r = run_fork(-1, 4.0, false);
  EXPECT_EQ(r.done, "fork_right");
  EXPECT_GT(r.min_island_gap, 0.05);
}

// 표지판 받침(섬 끝 가운데 기둥)을 LiDAR로 잡고, A면 받침 왼쪽·B면 오른쪽으로만 지나 갈래에 들어간다.
// block_side: 정한 쪽 갈래 입구에 장애물(작은 기둥)을 둬도 반대쪽으로 가지 않는지(0이면 없음)
struct PostRun {std::string done; double min_post_gap = 1e9, y_pass = 0, post_s0 = 0; bool found = false, crossed = false;};

static PostRun run_post(int sign, double block_d = 0.0)
{
  Params p;
  p.use_lane = false;
  p.section_end = 2.0;
  p.boundary_s = {-1.0, 6.5};
  p.boundary_left = {0.75, 0.75};
  p.boundary_right = {-0.75, -0.75};
  p.lane_centers = {-0.5, 0.0, 0.5};
  p.fork_sign_to_island = 5.0;  // 거리 추정이 틀려도(일부러 멀게) 받침 기준으로 가는지
  ObjAvoidCore core(p);
  struct Circle {double x, y, r;};
  std::vector<Circle> posts{{2.05, 0.05, 0.05}};
  if (block_d != 0.0) {posts.push_back({2.6, block_d, 0.05});}
  const int N = 400;
  const double amin = -1.9, ainc = 3.8 / (N - 1);
  auto scan_at = [&](double x, double y, double yaw) {
      std::vector<float> r(N, 8.0f);
      const double lx = x + std::cos(yaw) * p.lidar_x, ly = y + std::sin(yaw) * p.lidar_x;
      for (int k = 0; k < N; ++k) {
        const double a = yaw + amin + k * ainc, dx = std::cos(a), dy = std::sin(a);
        for (const Circle & c : posts) {
          const double ox = lx - c.x, oy = ly - c.y;
          const double b = ox * dx + oy * dy, cc = ox * ox + oy * oy - c.r * c.r, disc = b * b - cc;
          if (disc >= 0 && -b - std::sqrt(disc) > 0) {r[k] = std::min<float>(r[k], -b - std::sqrt(disc));}
        }
      }
      return r;
    };
  double x = 0, y = 0, yaw = 0, v = 0, t = 0, delta = 0;
  PostRun out;
  while (t < 40.0) {
    core.on_odom(t, x, y, yaw, v, v * std::tan(delta) / p.wheelbase);
    if (t == 0.0) {core.activate(t);}
    if (std::abs(t * 10 - std::round(t * 10)) < 1e-6) {
      core.process_scan(t, scan_at(x, y, yaw), amin, ainc, t);
      if (t > 2.0 && t < 2.6) {  // 표지판(받침 위)을 YOLO가 발행: 방향 + 영상 박스 가로 중심
        core.on_sign(t, sign < 0);
        const double cxw = x + std::cos(yaw) * p.fork_cam_x - std::sin(yaw) * p.fork_cam_y;
        const double cyw = y + std::sin(yaw) * p.fork_cam_x + std::cos(yaw) * p.fork_cam_y;
        const double bearing = std::atan2(posts[0].y - cyw, posts[0].x - cxw) - yaw;
        core.on_sign_detail(t, sign < 0, p.fork_cam_cx - p.fork_cam_fx * std::tan(bearing));
      }
    }
    double vc = 0, sc = 0;
    const bool pub = core.control(t, 0.04, vc, sc);
    if (core.sign_post()) {out.found = true; out.post_s0 = core.sign_post()->s0;}
    if (core.state == State::DONE) {out.done = core.done_reason; break;}
    v = pub ? vc : 0.0;
    delta = sc;
    x += v * std::cos(yaw) * 0.04;
    y += v * std::sin(yaw) * 0.04;
    yaw += v * std::tan(delta) / p.wheelbase * 0.04;
    const std::vector<Obs> po = obstacle_array({Box{posts[0].x - 0.05, posts[0].x + 0.05, posts[0].y - 0.05,
        posts[0].y + 0.05}});
    out.min_post_gap = std::min(out.min_post_gap, body_clearance(x, y, yaw, po, p));
    if (std::abs(x - posts[0].x) < 0.05) {out.y_pass = y;}
    if (sign * (y - posts[0].y) < -0.15 && x > posts[0].x - 0.3) {out.crossed = true;}  // 반대쪽으로 감
    t = std::round((t + 0.04) * 1e6) / 1e6;
  }
  std::printf("[post sign=%d block=%.2f] t=%.1f x=%.2f y=%.2f done=%s found=%d post_s0=%.2f y_pass=%.2f gap=%.3f "
    "crossed=%d\n", sign, block_d, t, x, y, out.done.c_str(), out.found, out.post_s0, out.y_pass, out.min_post_gap,
    out.crossed);
  return out;
}

TEST(Mission, ForkSignPostLeftRight)
{
  const PostRun l = run_post(1);
  EXPECT_TRUE(l.found);
  EXPECT_EQ(l.done, "fork_left");
  EXPECT_GT(l.y_pass, 0.25);
  EXPECT_GT(l.min_post_gap, 0.05);
  const PostRun r = run_post(-1);
  EXPECT_TRUE(r.found);
  EXPECT_EQ(r.done, "fork_right");
  EXPECT_LT(r.y_pass, -0.15);
  EXPECT_GT(r.min_post_gap, 0.05);
}

TEST(Mission, ForkSignPostNeverWrongSide)  // 정한 갈래 입구가 막혀도 반대쪽으로 가지 않음
{
  const PostRun l = run_post(1, 0.5);
  EXPECT_FALSE(l.crossed);
  EXPECT_NE(l.done, "fork_right");
  const PostRun r = run_post(-1, -0.5);
  EXPECT_FALSE(r.crossed);
  EXPECT_NE(r.done, "fork_left");
}

// ---------------------------------------------------------------- 자동 시작 게이트
namespace {
void add_rect(std::vector<XY> & w, double x0, double x1, double y0, double y1, double step = 0.01)
{
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    for (double y = y0; y <= y1 + 1e-9; y += step) {w.push_back({x, y});}
  }
}
std::vector<XY> stop_line(double x = 0.5) {std::vector<XY> w; add_rect(w, x, x + 0.04, -0.75, 0.75); return w;}
// 차로 경계 점선(차가 가운데서 shift 만큼 왼쪽에 있으면 점선은 오른쪽으로 보임)
void add_dashes(std::vector<XY> & w, double x0, double shift = 0.0)
{
  for (double x = x0; x < x0 + 1.4; x += 0.4) {
    for (double yc : {0.25 - shift, -0.25 - shift}) {add_rect(w, x, x + 0.2, yc - 0.01, yc + 0.01);}
  }
}
const std::vector<Box> kCar{{1.5, 1.7, -0.1, 0.1}};
}  // namespace

TEST(StartGate, ThreeLaneStart)
{
  Params p;
  auto w = stop_line();
  add_dashes(w, 0.65);
  const auto g = eval_start_gate(w, kCar, p);
  EXPECT_TRUE(g.line);
  EXPECT_NEAR(g.line_x, 0.52, 0.02);
  EXPECT_GE(g.dashes_l + g.dashes_r, 4);
  EXPECT_EQ(g.obstacles, 1);
  EXPECT_LE(g.stripes, p.start_stripe_max);
  EXPECT_TRUE(g.ok);
}

TEST(StartGate, ThreeLaneStartOffCenter)  // 실차 bag: 점선이 y +0.17 / -0.33 (차가 0.08 m 왼쪽)
{
  Params p;
  auto w = stop_line();
  add_dashes(w, 0.65, 0.08);
  EXPECT_TRUE(eval_start_gate(w, kCar, p).ok);
}

TEST(StartGate, RejectsCrosswalk)  // 정지선 + 횡단보도 줄무늬: 장애물이 있어도 시작 아님
{
  Params p;
  auto thin = stop_line();  // 얇은 줄무늬가 정지선과 떨어져 있음
  for (double y = -0.6; y < 0.6; y += 0.12) {add_rect(thin, 0.7, 1.0, y, y + 0.04);}
  const auto g = eval_start_gate(thin, kCar, p);
  EXPECT_GT(g.stripes, p.start_stripe_max);
  EXPECT_FALSE(g.ok);
  auto wide = stop_line();  // 굵은 줄무늬가 정지선 바로 뒤
  for (double y = -0.6; y < 0.6; y += 0.2) {add_rect(wide, 0.56, 0.9, y, y + 0.1);}
  EXPECT_FALSE(eval_start_gate(wide, kCar, p).ok);
}

TEST(StartGate, RejectsThickWhiteBlob)  // 라바콘·벽처럼 두꺼운 흰 영역은 정지선이 아님
{
  Params p;
  p.start_need_line = true;
  std::vector<XY> w;
  add_rect(w, 0.9, 1.4, -0.7, -0.1);
  add_dashes(w, 0.65);
  EXPECT_FALSE(eval_start_gate(w, kCar, p).line);
}

TEST(StartGate, NeedsDashesAndObstacle)
{
  Params p;
  EXPECT_FALSE(eval_start_gate(stop_line(), kCar, p).ok);  // 정지선만
  auto w = stop_line();
  add_dashes(w, 0.65);
  EXPECT_FALSE(eval_start_gate(w, {}, p).ok);  // 장애물 없음
  std::vector<XY> d;
  add_dashes(d, 0.65);
  EXPECT_TRUE(eval_start_gate(d, kCar, p).ok);   // 기본: 정지선 없이 점선 + 장애물
  p.start_need_line = true;
  EXPECT_FALSE(eval_start_gate(d, kCar, p).ok);  // 정지선 필수면 거부
  p.start_need_line = false;
  EXPECT_FALSE(eval_start_gate(d, {}, p).ok);    // 장애물 없음
  auto lng = stop_line();  // 길게 이어진 흰 실선은 점선이 아님
  add_rect(lng, 0.65, 2.4, 0.24, 0.26);
  add_rect(lng, 0.65, 2.4, -0.26, -0.24);
  EXPECT_FALSE(eval_start_gate(lng, kCar, p).ok);
}

TEST(StartGate, ObstacleMustBeAheadInHeading)  // 11-15 방지턱 표지판(길가) 오작동
{
  Params p;
  std::vector<XY> w;
  add_dashes(w, 0.65);
  EXPECT_TRUE(eval_start_gate(w, {{1.5, 1.7, -0.1, 0.1}}, p).ok);    // 정면
  EXPECT_FALSE(eval_start_gate(w, {{1.5, 1.7, 0.4, 0.6}}, p).ok);    // 옆 차로·길가
  EXPECT_FALSE(eval_start_gate(w, {{3.2, 3.5, -0.1, 0.1}}, p).ok);   // 너무 멂
  std::vector<XY> one;  // 점선 조각 1개뿐
  add_rect(one, 0.7, 0.9, 0.24, 0.26);
  EXPECT_FALSE(eval_start_gate(one, kCar, p).ok);
}

TEST(StartGate, DashPairOption)  // start_dash_both: 좌우 간격이 차로 폭과 다르면 거부
{
  Params p;
  p.start_dash_both = true;
  std::vector<XY> w;
  for (double x = 0.65; x < 1.5; x += 0.4) {
    add_rect(w, x, x + 0.2, 0.09, 0.11);
    add_rect(w, x, x + 0.2, -0.11, -0.09);
  }
  EXPECT_FALSE(eval_start_gate(w, kCar, p).ok);
  std::vector<XY> ok;
  add_dashes(ok, 0.65);
  EXPECT_TRUE(eval_start_gate(ok, kCar, p).ok);
}

// 단순 갈림길(fork_turn): 결정 후 fork_turn_after 직진, 정한 쪽으로 일정 조향 fork_turn_dist 뒤 DONE
static void run_fork_turn(int sign)
{
  Params p;
  p.use_lane = false;
  p.fork_turn = true;
  p.boundary_s = {-1.0, 8.0};
  p.boundary_left = {0.75, 0.75};
  p.boundary_right = {-0.75, -0.75};
  p.lane_centers = {-0.5, 0.0, 0.5};
  ObjAvoidCore core(p);
  const int N = 400;
  std::vector<float> scan(N, 8.0f);
  const double amin = -1.9, ainc = 3.8 / (N - 1);
  double x = 0, y = 0, yaw = 0, v = 0, t = 0, delta = 0, x_turn = -1, max_steer = 0;
  while (t < 30.0) {
    core.on_odom(t, x, y, yaw, v, v * std::tan(delta) / p.wheelbase);
    if (t == 0.0) {ASSERT_TRUE(core.activate(t));}
    if (std::abs(t * 10 - std::round(t * 10)) < 1e-6) {
      core.process_scan(t, scan, amin, ainc, t);
      if (t > 2.0 && t < 2.5) {core.on_sign(t, sign < 0);}
    }
    double vc = 0, sc = 0;
    const bool pub = core.control(t, 0.04, vc, sc);
    if (core.state == State::DONE) {break;}
    if (core.reason == "fork_turn" && x_turn < 0) {x_turn = x;}
    if (core.reason == "fork_turn") {max_steer = sc;}
    v = pub ? vc : 0.0;
    delta = sc;
    x += v * std::cos(yaw) * 0.04;
    y += v * std::sin(yaw) * 0.04;
    yaw += v * std::tan(delta) / p.wheelbase * 0.04;
    t = std::round((t + 0.04) * 1e6) / 1e6;
  }
  EXPECT_EQ(core.state, State::DONE);
  EXPECT_EQ(core.done_reason, sign > 0 ? "fork_left" : "fork_right");
  EXPECT_GT(x_turn, 0.5);                                   // 결정 후 바로 꺾지 않음
  EXPECT_NEAR(max_steer, sign * p.fork_turn_steer, 1e-9);  // 정한 쪽으로 일정 조향
  EXPECT_GT(sign * y, 0.05);                                // 실제로 그쪽으로 이동
}

TEST(Mission, ForkTurnLeft) {run_fork_turn(1);}
TEST(Mission, ForkTurnRight) {run_fork_turn(-1);}

TEST(Servo, EmulateCalibrationMatchesDriver)
{
  Params p;
  EXPECT_DOUBLE_EQ(emulate_calibration(0.3, p), 0.3);  // 기본 꺼짐: 그대로
  p.cmd_emulate_calibration = true;
  const double L = p.cmd_wheelbase, T = p.cmd_track;
  auto inner = [&](double c) {
      const double a = std::abs(c);
      return std::copysign(std::atan(2 * L * std::sin(a) / (2 * L * std::cos(a) - T * std::sin(a))), c);
    };
  for (double d : {-0.6, -0.3, -0.05, 0.02, 0.1, 0.25, 0.45, 0.6}) {
    const double cal = std::copysign(std::min(p.cmd_cal_limit, (std::abs(inner(d)) + p.cmd_cal_deadband) /
        p.cmd_cal_gain), d);                                     // 보정 켠 드라이버 서보 값
    const double legacy = inner(emulate_calibration(d, p)) / p.cmd_legacy_scale;  // 보정 끈 드라이버 + 변환
    EXPECT_NEAR(legacy, cal, 1e-9) << d;
  }
  EXPECT_EQ(emulate_calibration(0.0, p), 0.0);
}

TEST(Mission, ActivateShiftPutsOriginAtLine)
{
  Params p;
  ObjAvoidCore core(p);
  core.on_odom(0.0, 1.0, 2.0, M_PI / 2, 0.0, 0.0);
  ASSERT_TRUE(core.activate(0.0, 0.6));
  const auto v = core.vehicle(0.0);
  ASSERT_TRUE(v);
  EXPECT_NEAR(v->s, -0.6, 1e-9);
  EXPECT_NEAR(v->d, 0.0, 1e-9);
}

TEST(Perception, LimoAxisSizeDoesNotGrowWithHistory)
{
  Params p;
  p.obstacle_fixed_size = true;
  p.obstacle_min_length = 0.322;
  p.obstacle_min_width = 0.22;
  PriorBoundary boundary({-1, 6}, {1.0, 1.0}, {-1.0, -1.0});
  std::vector<Box> boxes;
  std::vector<Pt2> pts;
  detect(std::vector<float>(41, 1.0f), -0.1, 0.005, {}, RoadFrame(0, 0, 0), boundary, 0, p, boxes, pts);
  ASSERT_EQ(boxes.size(), 1u);
  EXPECT_NEAR(boxes[0].s1-boxes[0].s0,0.322,1e-9);
  EXPECT_NEAR(boxes[0].d1-boxes[0].d0,0.22,1e-9);
  for(const auto & pt:pts) {
    EXPECT_GE(pt.s,boxes[0].s0-1e-9);EXPECT_LE(pt.s,boxes[0].s1+1e-9);
    EXPECT_GE(pt.d,boxes[0].d0-1e-9);EXPECT_LE(pt.d,boxes[0].d1+1e-9);
  }
  Tracker tracker(p);
  tracker.update(boxes, 0, -1, nullptr, nullptr);
  auto shifted = boxes;
  shifted[0].s0 += 0.02; shifted[0].s1 += 0.02;
  shifted[0].d0 += 0.01; shifted[0].d1 += 0.01;
  tracker.update(shifted, 0.1, -1, nullptr, nullptr);
  ASSERT_EQ(tracker.tracks.size(), 1u);
  EXPECT_NEAR(tracker.tracks[0].box.s1 - tracker.tracks[0].box.s0, shifted[0].s1-shifted[0].s0, 1e-9);
  EXPECT_NEAR(tracker.tracks[0].box.d1 - tracker.tracks[0].box.d0, shifted[0].d1-shifted[0].d0, 1e-9);
}

TEST(Island, PartialFrontAndSideFitButNotSingleStraight) {
  IslandShape truth; truth.pose={4.6,0.06,0.12,0,0,true,true};
  const auto v=truth.vertices();std::vector<XY> points;
  for(int edge:{0,1,5}) for(int i=0;i<22;++i) {
    const double t=i/21.0; const auto a=v[edge],b=v[(edge+1)%6];
    points.push_back({a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t});
  }
  IslandMapper mapper;
  for(int i=0;i<3;++i) ASSERT_TRUE(mapper.update(points,4.25));
  ASSERT_TRUE(mapper.shape.pose.confirmed);
  EXPECT_NEAR(mapper.shape.pose.s,4.6,0.08);
  EXPECT_NEAR(mapper.shape.pose.d,0.06,0.06);
  EXPECT_NEAR(mapper.shape.pose.yaw,0.12,0.09);
  points.clear();for(int i=0;i<50;++i) points.push_back({4.0+i*0.02,0.25});
  mapper.reset();EXPECT_FALSE(mapper.update(points,4.25));
}
TEST(Island, EntireRotatedBodyAndBothBranches) {
  Params p;p.fork_shape=true;
  IslandShape sh;sh.pose={2,0,0,0,3,true,true};
  EXPECT_LT(sh.clearance(2.5,0,0,p),0);
  EXPECT_GT(sh.clearance(2.5,0.51,0,p),0.1);
  EXPECT_LT(sh.clearance(2.5,0.38,0.7,p),0.03);
  PriorBoundary bd({-1,7},{0.77,0.77},{-0.77,-0.77});Planner pl(p,&bd);pl.island=sh;
  for(int side:{-1,1}) {
    pl.island_side=side;p.home_d=side*0.51;
    auto path=pl.plan({0,0,0,0},{},nullptr,nullptr);
    ASSERT_TRUE(path);
    EXPECT_GT(path->end_s(),2.0);
    for(size_t i=0;i<path->s.size();++i) {
      EXPECT_GE(sh.clearance(path->s[i],path->d[i],path->psi[i],p),p.margin_obstacle+p.fork_shape_margin-1e-6);
      if(path->s[i]>=2) {EXPECT_GT(side*path->d[i],0);}
    }
  }
}
TEST(Island, ObservationTurnsTowardViewAndRejectsUnseenOrBlocked) {
  Params p;p.fork_shape=true;p.fork_search_speed=0.2;
  PriorBoundary bd({-1,8},{0.77,0.77},{-0.77,-0.77});Planner pl(p,&bd);
  VisibilityGrid grid(p,-1,8);grid.mark_free(-1,8,-0.77,0.77);
  for(double lateral:{-0.3,0.3}) {
    auto path=pl.observe(2,lateral,0,0,3.5,0,{},grid,1);
    ASSERT_TRUE(path);
    EXPECT_LT(lateral*path->psi.back(),0);
    EXPECT_GT(path->s.back(),path->s.front());
  }
  VisibilityGrid unseen(p,-1,8);
  EXPECT_FALSE(pl.observe(2,0.3,0,0,3.5,0,{},unseen,1));
  EXPECT_FALSE(pl.observe(2,0.3,0,0,3.5,0,{{2,3,-0.7,0.7}},grid,1));
}

TEST(Island, ClosedLoopBothBranchesAndFreshLaneHandoff) {
  for(int side:{-1,1}) {
    Params p;p.fork_shape=true;p.use_lane=false;p.section_end=2;p.v_max=0.2;
    p.boundary_s={-1,7};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
    ObjAvoidCore core(p);double x=0,y=0,yaw=0,v=0,steer=0;
    std::vector<float> scan(400,8);
    for(int step=0;step<1200;++step) {
      const double t=step*0.04;
      core.on_odom(t,x,y,yaw,v,v*std::tan(steer)/p.wheelbase);
      if(step==0) {
        ASSERT_TRUE(core.activate(t));
        core.island_section_reached_at=t; // Inject an already confirmed post-line mission.
        core.island_mapper.shape.pose={2,0,0,0,3,true,true};
        core.on_sign(t,side<0);core.on_sign(t,side<0);
      }
      core.island_frame_at=t;
      if(step%5==0) {
        core.grid->mark_free(-1,7,-0.77,0.77);
        core.process_scan(t,scan,-1.9,3.8/399,t);
      }
      // Goal arrival releases control even without a lane-command callback.
      double vc=0,sc=0;const bool publish=core.control(t,0.04,vc,sc);

      if(core.state==State::DONE) break;
      v=publish?vc:0;steer=sc;
      x+=v*std::cos(yaw)*0.04;y+=v*std::sin(yaw)*0.04;
      yaw+=v*std::tan(steer)/p.wheelbase*0.04;
      EXPECT_GT(core.island_mapper.shape.clearance(x,y,yaw,p),0);
    }
    EXPECT_EQ(core.done_reason,side>0?"fork_left":"fork_right");
  }
}

TEST(Island, OffCenterTargetStaysInsideActualCorridor) {
  Params p;p.fork_shape=true;
  PriorBoundary boundary({-1,8},{0.77,0.77},{-0.77,-0.77});Planner pl(p,&boundary);
  pl.island.pose={4,-0.15,0,0,3,true,true};pl.island_side=-1;
  EXPECT_NEAR(pl.desired_d(5),-0.585,1e-6);
  EXPECT_GT(pl.desired_d(5)-p.body_width/2,-0.77);
  pl.island_side=1;EXPECT_NEAR(pl.desired_d(5),0.435,1e-6);
}

TEST(StartGate, YellowEntranceEitherSideWithoutDashRequirement) {
  Params p;std::vector<XY> yellow,left;
  for(int i=0;i<80;++i) {
    const double x=0.2+i*0.018;
    const double y=0.25+std::clamp(x-0.4,0.0,0.52);
    yellow.push_back({x,y});yellow.push_back({x,-y});left.push_back({x,y});
  }
  auto both=eval_entrance_gate(yellow,false,p);
  EXPECT_TRUE(both.ok);EXPECT_TRUE(both.left);EXPECT_TRUE(both.right);
  EXPECT_TRUE(eval_entrance_gate(left,false,p).ok);
  EXPECT_TRUE(eval_entrance_gate(left,true,p).ok);
  yellow.clear();for(int i=0;i<80;++i) {yellow.push_back({0.2+i*0.018,0.77});yellow.push_back({0.2+i*0.018,-0.77});}
  EXPECT_FALSE(eval_entrance_gate(yellow,true,p).ok);
}
TEST(StartGate, RejectsNarrowingAndIsolatedDiagonal) {
  Params p;std::vector<XY> yellow;
  for(int i=0;i<80;++i) {
    const double x=0.2+i*0.018,y=0.77-std::clamp(x-0.4,0.0,0.52);
    yellow.push_back({x,y});yellow.push_back({x,-y});
  }
  EXPECT_FALSE(eval_entrance_gate(yellow,true,p).ok);
  yellow.clear();for(int i=0;i<40;++i) yellow.push_back({0.5+i*0.015,0.2+i*0.015});
  EXPECT_FALSE(eval_entrance_gate(yellow,true,p).ok);
}

TEST(Mission, EntranceAnchorIncludesHeadingAndLateralOffset) {
  Params p;p.use_lane=false;ObjAvoidCore core(p);
  core.on_odom(1,2,3,0,0,0);
  ASSERT_TRUE(core.activate(1,0.6,0.2,0.1));
  EXPECT_NEAR(core.frame->yaw0,0.2,1e-9);
  EXPECT_NEAR(core.frame->x0,2+0.6*std::cos(0.2)-0.1*std::sin(0.2),1e-9);
  EXPECT_NEAR(core.frame->y0,3+0.6*std::sin(0.2)+0.1*std::cos(0.2),1e-9);
}

TEST(StartGate, EntranceWindowIsAlternativeAndWaitsForTwoObstacles) {
  EntranceWindow w;
  EXPECT_FALSE(w.update(10,true,0));EXPECT_TRUE(w.armed(10));
  EXPECT_FALSE(w.update(10.1,false,2));EXPECT_TRUE(w.armed(10.1));
  EXPECT_FALSE(w.update(10.31,false,2));EXPECT_FALSE(w.armed(10.31));
  EXPECT_LT(w.deadline,10.31);
  EXPECT_TRUE(w.update(11,true,2));
  // It is an alternative path: no yellow sighting means old gate remains independently usable.
  w.reset();EXPECT_FALSE(w.update(20,false,2));
  Params p;p.start_need_line=false;p.start_dash_both=false;p.start_dash_min=0;
  EXPECT_TRUE(eval_start_gate({},{{0.7,1,-0.1,0.1},{1.2,1.5,-0.1,0.1}},p).ok);
}

TEST(StartGate, PassedEntranceAndStaleConfirmationCannotStart) {
  EntranceWindow w;
  EXPECT_FALSE(w.update(1,true,0));
  EXPECT_FALSE(w.update(1.1,true,2,5,2,false));
  EXPECT_FALSE(w.armed(1.1));
  EXPECT_FALSE(w.update(1.2,false,2));
  EXPECT_TRUE(entrance_ahead(0.5,0,0,0,0));
  EXPECT_FALSE(entrance_ahead(0.5,0,0.6,0,0));
  EXPECT_FALSE(entrance_ahead(1,0,0,0,3.141592653589793));
  EXPECT_TRUE(entrance_ahead(0,0.5,0,0,1.570796326794897));
}

TEST(StartGate, DisconnectedCornerFragmentsAndStraightLinesAreRejected) {
  Params p;std::vector<XY> yellow;
  for(int i=0;i<35;++i) {
    const double x=0.41+i*0.01;yellow.push_back({x,0.25+x-0.4});
  }
  for(int i=0;i<35;++i) yellow.push_back({1.08+i*0.01,0.77});
  EXPECT_FALSE(eval_entrance_gate(yellow,false,p).ok);
  yellow.clear();
  for(int i=0;i<100;++i) {
    const double x=0.2+i*0.015;
    yellow.push_back({x,-0.20-0.20*x});yellow.push_back({x,-0.60-0.20*x});
  }
  EXPECT_FALSE(eval_entrance_gate(yellow,false,p).ok);
}

TEST(StartGate, ConnectedNoisyEntranceIsRecognizedFromEitherSide) {
  Params p;
  for(int sign:{-1,1}) for(double yaw:{-0.2,0.0,0.2}) {
    std::vector<XY> yellow;const double c=std::cos(yaw),s=std::sin(yaw);
    for(int i=0;i<95;++i) {
      const double x=0.2+i*0.015;
      const double y=sign*(0.25+std::clamp(x-0.4,0.0,0.52))+0.004*std::sin(i*1.7);
      yellow.push_back({c*x-s*y,s*x+c*y});
    }
    EXPECT_TRUE(eval_entrance_gate(yellow,false,p).ok) << "side=" << sign << " yaw=" << yaw;
  }
}

TEST(StartGate, RecordedStraightParkingPaintDoesNotCountAsEntrance) {
  const auto data=YAML::LoadFile(std::string(TEST_DATA_DIR)+"/straight_parking_215435.yaml");
  Params p;
  for(const auto & frame:data["frames"]) {
    std::vector<XY> yellow;
    for(const auto & q:frame["yellow"]) yellow.push_back({q[0].as<double>(),q[1].as<double>()});
    EXPECT_FALSE(eval_entrance_gate(yellow,false,p).ok) << "bag time=" << frame["bag_time"];
  }
}
TEST(StartGate, ContinuousYellowDoesNotExtendFiveSecondDeadlineAndClockResetClears) {
  EntranceWindow w;EXPECT_FALSE(w.update(1,true,0));
  EXPECT_FALSE(w.update(4,true,1));EXPECT_DOUBLE_EQ(w.deadline,6);
  EXPECT_FALSE(w.update(6.01,true,2));
  EXPECT_FALSE(w.update(0,false,2));EXPECT_FALSE(w.armed(0));
  EXPECT_TRUE(w.update(1,true,2));
}

TEST(Island, FreshObservationsUnlockAndRecoverWrongModel) {
  IslandMapper mapper;IslandShape truth;truth.pose={4.6,0.05,0.1,0,3,true,true};
  const auto vertices=truth.vertices();std::vector<XY> points;
  for(int edge:{0,1,5}) for(int i=0;i<24;++i) {
    const auto a=vertices[edge],b=vertices[(edge+1)%6];const double t=i/23.0;
    points.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)});
  }
  mapper.shape.pose={3.6,0,0,0,3,true,true};
  for(int i=0;i<6;++i) mapper.update({},4.25);
  EXPECT_FALSE(mapper.shape.pose.valid);EXPECT_FALSE(mapper.shape.pose.confirmed);
  for(int i=0;i<4;++i) mapper.update(points,4.25);
  EXPECT_TRUE(mapper.shape.pose.confirmed);EXPECT_NEAR(mapper.shape.pose.s,4.6,0.08);
  // A model covering the measured vehicle must be discarded, not frozen as confirmed.
  Params p;mapper.shape.pose={3.7,0,0,0,3,true,true};
  for(int i=0;i<4;++i) mapper.update(points,4.25,&p,4.1,0,0);
  ASSERT_TRUE(mapper.shape.pose.confirmed);
  EXPECT_GT(mapper.shape.clearance(4.1,0,0,p),p.margin_obstacle);
}
TEST(Island, RoadExternalFlareIsNotIslandEvidence) {
  IslandMapper mapper;std::vector<XY> road;
  for(int i=0;i<100;++i) {
    const double x=4+i*0.02,y=0.25+std::clamp(x-4.4,0.0,0.52);
    road.push_back({x,y});road.push_back({x,-y});
  }
  for(int i=0;i<4;++i) EXPECT_FALSE(mapper.update(road,4.25));
  EXPECT_FALSE(mapper.shape.pose.confirmed);
}

TEST(Mission, ReplayTimeRewindClearsConfirmedIslandAndActivation) {
  Params p;p.use_lane=false;p.fork_shape=true;ObjAvoidCore core(p);
  core.on_odom(10,0,0,0,0,0);ASSERT_TRUE(core.activate(10));
  core.island_mapper.shape.pose={4.2,0,0,0,3,true,true};
  core.on_sign(10,false);core.on_sign(10,false);
  core.on_odom(20,1,0,0,0,0);
  core.on_odom(1,0,0,0,0,0);
  EXPECT_FALSE(core.active());EXPECT_EQ(core.state,State::IDLE);
  EXPECT_FALSE(core.island_mapper.shape.pose.valid);EXPECT_EQ(core.fork_votes_l,0);
  ASSERT_TRUE(core.activate(1));EXPECT_FALSE(core.island_known());
}

TEST(Outline, FullOutlineFitsWithoutIslandInfluence) {
  Params p;p.road_outline=true;RoadOutline outline(p);
  std::vector<XY> road;
  const double yaw=0.04,cs=std::cos(yaw),sn=std::sin(yaw);
  for(const auto & edge:outline.edges) for(int i=0;i<40;++i) {
    const double t=i/39.0,x=edge.first.x+t*(edge.second.x-edge.first.x),y=edge.first.y+t*(edge.second.y-edge.first.y);
    road.push_back({0.10+cs*x-sn*y,-0.025+sn*x+cs*y});
  }
  const auto clean=outline.fit(road);ASSERT_TRUE(clean.valid);EXPECT_TRUE(clean.longitudinal);
  EXPECT_NEAR(clean.s,0.10,0.03);EXPECT_NEAR(clean.d,-0.025,0.02);EXPECT_NEAR(clean.yaw,yaw,0.015);
  IslandShape sh;sh.pose={4.9,0.06,0.05,0,0,true,false};
  const auto vertices=sh.vertices();
  for(int k=0;k<6;++k) for(int i=0;i<30;++i) {
    const double t=i/29.0;const auto a=vertices[k],b=vertices[(k+1)%6];
    road.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)});
  }
  const auto mixed=outline.fit(road);ASSERT_TRUE(mixed.valid);
  EXPECT_NEAR(mixed.s,clean.s,0.005);EXPECT_NEAR(mixed.d,clean.d,0.005);EXPECT_NEAR(mixed.yaw,clean.yaw,0.005);
}
TEST(Outline, StraightOnlyDoesNotInventLongitudinalCorrection) {
  Params p;RoadOutline outline(p);std::vector<XY> pts;
  for(int i=0;i<60;++i) pts.push_back({2.0+i*0.02,0.77+0.03});
  const auto fit=outline.fit(pts);ASSERT_TRUE(fit.valid);EXPECT_FALSE(fit.longitudinal);
  EXPECT_DOUBLE_EQ(fit.s,0);EXPECT_NEAR(fit.d,0.03,0.015);
}
TEST(Outline, IslandAloneCannotAlignRoad) {
  Params p;RoadOutline outline(p);std::vector<XY> pts;
  for(int i=0;i<70;++i) pts.push_back({3+i*0.02,0.25});
  EXPECT_FALSE(outline.fit(pts).valid);
}
TEST(Island, ReferenceLineDoesNotDeterminePosition) {
  IslandShape truth;truth.pose={2.1,0.05,0.1,0,0,true,true};std::vector<XY> pts;
  const auto v=truth.vertices();for(int k:{0,1,5}) for(int i=0;i<30;++i) {
    const double t=i/29.0;pts.push_back({v[k].x+t*(v[(k+1)%6].x-v[k].x),v[k].y+t*(v[(k+1)%6].y-v[k].y)});
  }
  IslandMapper first,second;
  for(int i=0;i<3;++i) {ASSERT_TRUE(first.update(pts,0));ASSERT_TRUE(second.update(pts,20));}
  ASSERT_TRUE(first.shape.pose.confirmed);ASSERT_TRUE(second.shape.pose.confirmed);
  EXPECT_DOUBLE_EQ(first.shape.pose.s,second.shape.pose.s);EXPECT_NEAR(first.shape.pose.s,2.1,0.08);
}

TEST(Projection, FullImageSampledBothColorsAndTwoMeterLimit) {
  const std::string config="/tmp/objavoid_projection_"+std::to_string(getpid())+".yaml";
  {std::ofstream f(config);f<<"lane_lite: {width: 100, height: 100, roi_top: 0.55, bev_top_y: 0.1, bev_bottom_y: 0.9, bev_top_left: 0.1, bev_top_right: 0.9, bev_bottom_left: 0.1, bev_bottom_right: 0.9, lane_width_m: 0.5, lane_width_px: 50, bev_near_m: 0.2, bev_far_m: 2.0}\n";}
  // Ground x=0.1+0.02v, y=-1+0.02u. Bottom pixels are beyond 2m.
  const std::vector<double> h{0,0.02,0.1,0.02,0,-1,0,0,1};
  GroundProjection full(config,2,1.5,h,true),cropped(config,2,1.5,h,false);
  std::remove(config.c_str());
  cv::Mat mask(100,100,CV_8UC1);
  for(int v=0;v<100;++v) for(int u=0;u<100;++u) {
    mask.at<uchar>(v,u)=u<50?1:2;
  }
  const auto yellow=full.yellow_points(mask),white=full.white_points(mask);
  ASSERT_FALSE(yellow.empty());ASSERT_FALSE(white.empty());
  bool near=false,left=false,right=false;
  for(const auto & points:{yellow,white}) for(const auto & q:points) {
    EXPECT_LE(q.x,2);EXPECT_TRUE(std::isfinite(q.x));near|=q.x<0.5;left|=q.y>0.8;right|=q.y< -0.8;
  }
  EXPECT_TRUE(near);EXPECT_TRUE(left);EXPECT_TRUE(right);
  EXPECT_GT(yellow.size(),cropped.yellow_points(mask).size());
  const auto dense=full.class_points(mask,1,1);
  EXPECT_GT(double(yellow.size())/dense.size(),0.20);EXPECT_LT(double(yellow.size())/dense.size(),0.30);
  EXPECT_EQ(full.yellow_points(mask).size(),yellow.size());EXPECT_EQ(full.white_points(mask).size(),white.size());
}
TEST(Mission, OutlineReframePreservesCandidateWorldPoseAndIgnoresDuplicateFrame) {
  Params p;p.road_outline=true;p.fork_shape=true;p.lane_lidar_filter=false;
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
  core.island_mapper.shape.pose={3,0,0,0,2,true,false};
  RoadOutline outline(p);std::vector<XY> yellow,view;
  const double c=std::cos(0.035),sn=std::sin(0.035);
  for(const auto & e:outline.edges) for(int i=0;i<80;++i) {
    const double t=i/79.0,x=e.first.x+t*(e.second.x-e.first.x),y=e.first.y+t*(e.second.y-e.first.y);
    if(x>0.2 && x<1.9) yellow.push_back({c*x-sn*y,0.05+sn*x+c*y});
  }
  for(double x=0.2;x<2;x+=0.03) for(double y=-1;y<1;y+=0.03) view.push_back({x,y});
  for(int i=0;i<4;++i) {
    const double t=1.1+i*0.15;core.on_odom(t,0,0,0,0,0);
    ASSERT_TRUE(core.on_lane(t,yellow,view,nullptr));
    const auto size=core.yellow_history.size();EXPECT_FALSE(core.on_lane(t,yellow,view,nullptr));
    EXPECT_EQ(core.yellow_history.size(),size);
    core.process_scan(t,std::vector<float>(720,INFINITY),-M_PI,2*M_PI/720,t);
  }
  EXPECT_TRUE(core.aligned);ASSERT_TRUE(core.island_mapper.shape.pose.valid);
  const auto & ip=core.island_mapper.shape.pose;double x,y;core.frame->to_odom(ip.s,ip.d,x,y);
  EXPECT_NEAR(x,3,1e-6);EXPECT_NEAR(y,0,1e-6);
  EXPECT_NEAR(core.frame->yaw0,0.035,0.02);EXPECT_NEAR(core.frame->y0,0.05,0.025);
}
TEST(Island, ConfirmedPoseSurvivesSideOnlyObservation) {
  IslandMapper mapper;mapper.shape.pose={3.0,0,0,0,3,true,true};
  std::vector<XY> pts;for(int i=0;i<40;++i) pts.push_back({3.3+i*0.02,0.25});
  // Include tiny tape thickness so the collinearity guard can be bypassed only
  // for an already confirmed model, never for first localization.
  for(int i=0;i<40;++i) pts.push_back({3.3+i*0.02,0.265});
  for(int i=0;i<8;++i) mapper.update(pts,100);
  EXPECT_TRUE(mapper.shape.pose.confirmed);EXPECT_NEAR(mapper.shape.pose.s,3.0,0.001);
}

TEST(Outline, LargeInitialOffsetAndYawWithNoise) {
  Params p;RoadOutline outline(p);std::vector<XY> pts;
  const double yaw=0.14,c=std::cos(yaw),sn=std::sin(yaw);
  for(const auto & e:outline.edges) for(int i=0;i<100;++i) {
    const double t=i/99.0,x=e.first.x+t*(e.second.x-e.first.x),y=e.first.y+t*(e.second.y-e.first.y);
    // Simulate the entrance window rather than a full map in one image.
    if(x< -0.1 || x>1.8) continue;
    pts.push_back({0.3+c*x-sn*y,0.32+sn*x+c*y+0.008*std::sin(i*1.7)});
  }
  for(int i=0;i<40;++i) pts.push_back({0.2+i*0.039,-1.2+std::fmod(i*0.317,2.4)});
  const auto fit=outline.fit(pts,true);ASSERT_TRUE(fit.valid);
  EXPECT_TRUE(fit.longitudinal);EXPECT_NEAR(fit.s,0.3,0.07);
  EXPECT_NEAR(fit.d,0.32,0.04);EXPECT_NEAR(fit.yaw,yaw,0.04);
}
TEST(Island, WarmCandidateAvoidsRepeatingGlobalSearch) {
  IslandShape truth;truth.pose={4.6,0.06,0.1,0,0,true,true};
  const auto v=truth.vertices();std::vector<XY> pts;
  for(int k:{0,1,5}) for(int i=0;i<25;++i) {
    const double t=i/24.0;pts.push_back({v[k].x+t*(v[(k+1)%6].x-v[k].x),v[k].y+t*(v[(k+1)%6].y-v[k].y)});
  }
  IslandMapper mapper;ASSERT_TRUE(mapper.update(pts,0));const auto first=mapper.evaluations;
  ASSERT_TRUE(mapper.update(pts,0));EXPECT_LE(mapper.evaluations,27u);EXPECT_LT(mapper.evaluations,first);
  ASSERT_TRUE(mapper.update(pts,0));EXPECT_TRUE(mapper.shape.pose.confirmed);
}
TEST(Mission, FreshYoloEventAfterActivationSelectsRequestedLatticeSide) {
  for(bool right:{false,true}) {
    Params p;p.use_lane=false;p.fork_shape=true;p.fork_sign_min=1;
    p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
    ObjAvoidCore core(p);core.on_sign(0.95,right);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
    EXPECT_EQ(core.fork_side(),0);
    EXPECT_GT(core.island_sign_after,1e90);
    core.on_sign(1.01,right);
    EXPECT_EQ(core.fork_side(),right?-1:1);
    EXPECT_DOUBLE_EQ(core.island_sign_after,1.01);
    core.island_section_reached_at=1;
    core.island_mapper.shape.pose={2,0.02,0.06,0,3,true,true};
    core.on_sign(1,right);core.island_frame_at=1;
    // Trigger the pending branch update by changing the command before entering.
    core.on_sign(1,!right);core.on_sign(1,right);
    core.grid->mark_free(-1,8,-0.77,0.77);
    ASSERT_TRUE(core.process_scan(1,std::vector<float>(720,8),-M_PI,2*M_PI/720,1));
    ASSERT_TRUE(core.path_);bool entered=false;
    const int side=right?-1:1;
    for(size_t i=0;i<core.path_->s.size();++i) {
      const auto q=core.island_mapper.shape.local(core.path_->s[i],core.path_->d[i]);
      if(q.x>=0.3) {entered=true;EXPECT_GT(side*q.y,0.25);}
      EXPECT_GE(core.island_mapper.shape.clearance(core.path_->s[i],core.path_->d[i],core.path_->psi[i],p),p.margin_obstacle+p.fork_shape_margin-1e-6);
    }
    EXPECT_TRUE(entered);
  }
}

TEST(Mission, SavedMapPlansBeforeMatchingAndSurvivesObservationLoss) {
  Params p;p.road_outline=true;p.fork_shape=true;p.lane_lidar_filter=false;
  p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
  const std::vector<float> scan(720,8);
  auto step=[&](double t) {
    core.on_odom(t,0,0,0,0,0);core.grid->mark_free(-1,8,-0.77,0.77);
    ASSERT_TRUE(core.process_scan(t,scan,-M_PI,2*M_PI/720,t));ASSERT_TRUE(core.path_);
    double speed=0,steer=0;ASSERT_TRUE(core.control(t,0.05,speed,steer));EXPECT_GT(speed,0);
  };
  step(1);EXPECT_FALSE(core.aligned);
  EXPECT_EQ(YAML::Load(core.status_json())["road_mapping"]["source"].as<std::string>(),"prior");
  RoadOutline outline(p);std::vector<XY> yellow,view;
  const double c=std::cos(0.035),sn=std::sin(0.035);
  for(const auto & e:outline.edges) for(int i=0;i<80;++i) {
    const double t=i/79.0,x=e.first.x+t*(e.second.x-e.first.x),y=e.first.y+t*(e.second.y-e.first.y);
    if(x>0.2&&x<1.9) yellow.push_back({c*x-sn*y,0.05+sn*x+c*y});
  }
  for(double x=0.2;x<2;x+=0.03) for(double y=-1;y<1;y+=0.03) view.push_back({x,y});
  for(double t:{1.15,1.30,1.45}) {
    core.on_odom(t,0,0,0,0,0);ASSERT_TRUE(core.on_lane(t,yellow,view,nullptr));step(t);
  }
  ASSERT_TRUE(core.aligned);const RoadFrame saved=*core.frame;
  for(double t:{1.6,1.75,1.9,2.05,2.2}) {
    core.on_odom(t,0,0,0,0,0);ASSERT_TRUE(core.on_lane(t,{},view,nullptr));step(t);
  }
  EXPECT_TRUE(core.aligned);EXPECT_FALSE(core.outline_fit.valid);
  EXPECT_DOUBLE_EQ(core.frame->x0,saved.x0);EXPECT_DOUBLE_EQ(core.frame->y0,saved.y0);
  EXPECT_DOUBLE_EQ(core.frame->yaw0,saved.yaw0);
  const auto status=YAML::Load(core.status_json());
  EXPECT_EQ(status["road_mapping"]["source"].as<std::string>(),"matched");
  EXPECT_TRUE(status["road_mapping"]["using_saved_map"].as<bool>());
}

TEST(Mission, GoalArrivalEndsOutputEvenWithoutPathOrLaneCommand) {
  for(int side:{-1,1}) {
    Params p;p.use_lane=false;p.fork_shape=true;p.fork_sign_min=1;
    p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
    ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
    core.island_section_reached_at=1;
    core.island_mapper.shape.pose={2,0,0,0,3,true,true};core.on_sign(1,side<0);
    const auto goal=core.fork_goal();ASSERT_TRUE(goal);
    double v=0,steer=0;
    core.on_odom(1.1,goal->x-0.25,goal->y,0,0,0);core.control(1.1,0.04,v,steer);
    EXPECT_NE(core.state,State::DONE);
    core.on_odom(1.2,goal->x,goal->y,0,0,0);
    EXPECT_FALSE(core.control(1.2,0.04,v,steer));EXPECT_EQ(core.state,State::DONE);
    EXPECT_FALSE(core.path_);EXPECT_FALSE(core.last_info.valid);
    EXPECT_FALSE(core.control(1.24,0.04,v,steer));
    EXPECT_FALSE(core.process_scan(1.24,std::vector<float>(720,8),-M_PI,2*M_PI/720,1.24));
    EXPECT_EQ(core.mapping_restarts,0);
  }
}
TEST(Mission, SignObservedWellBeforeActivationIsRetained) {
  Params p;p.use_lane=false;p.fork_sign_min=1;
  ObjAvoidCore core(p);core.on_sign(1,false);core.on_odom(10,0,0,0,0,0);
  ASSERT_TRUE(core.activate(10));EXPECT_EQ(core.fork_side(),1);
  ObjAvoidCore expired(p);expired.on_sign(1,true);expired.on_odom(32,0,0,0,0,0);
  ASSERT_TRUE(expired.activate(32));EXPECT_EQ(expired.fork_side(),0);
}
TEST(Mission, ConfirmedObstacleReappearanceRestartsMapsButPreservesDirection) {
  Params p;p.use_lane=false;p.fork_shape=true;p.fork_sign_min=1;p.obstacle_reacquire_absence_sec=0.5;
  p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));core.on_sign(1,false);
  std::vector<float> clear(720,8),obstacle=clear;
  for(int i=335;i<=385;++i) obstacle[i]=1.0;
  auto scan=[&](double t,const std::vector<float> & ranges) {
    core.on_odom(t,0,0,0,0,0);core.process_scan(t,ranges,-M_PI,2*M_PI/720,t);
  };
  scan(1,obstacle);ASSERT_FALSE(core.last_boxes.empty());
  core.aligned=true;core.island_mapper.shape.pose={4.6,0,0,0,3,true,true};
  core.yellow_history.emplace_back(1,std::vector<XY>{{4.6,0}});
  for(double t:{1.1,1.2,1.3,1.4,1.5,1.6,1.7}) scan(t,clear);
  scan(1.8,obstacle);EXPECT_EQ(core.mapping_restarts,0);
  scan(1.9,obstacle);EXPECT_EQ(core.mapping_restarts,1);EXPECT_EQ(core.fork_side(),1);
  EXPECT_FALSE(core.aligned);EXPECT_FALSE(core.island_mapper.shape.pose.valid);
  EXPECT_TRUE(core.yellow_history.empty());EXPECT_FALSE(core.last_boxes.empty());
  scan(2.0,obstacle);EXPECT_EQ(core.mapping_restarts,1);
}
TEST(Mission, SensorDropoutOrShortGapDoesNotRestartMapping) {
  Params p;p.use_lane=false;p.obstacle_reacquire_absence_sec=0.5;
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
  std::vector<float> clear(720,8),obstacle=clear,invalid(720,0);
  for(int i=335;i<=385;++i) obstacle[i]=1;
  for(const auto & sample:std::vector<std::pair<double,std::vector<float>>>{{1,obstacle},{1.1,clear},{1.2,obstacle},{1.3,obstacle},
      {1.4,invalid},{2.4,invalid},{2.5,obstacle},{2.6,obstacle}}) {
    core.on_odom(sample.first,0,0,0,0,0);core.process_scan(sample.first,sample.second,-M_PI,2*M_PI/720,sample.first);
  }
  EXPECT_EQ(core.mapping_restarts,0);
}

TEST(Planner, PrefersTenCentimeterBodyClearanceInRoomyCorridor) {
  Params p;p.margin_obstacle=0.03;p.preferred_obstacle_clearance=0.10;
  PriorBoundary bd({-1,8},{0.77,0.77},{-0.77,-0.77});Planner planner(p,&bd);
  const std::vector<Box> boxes{{1,1.3,-0.1,0.1}};
  const auto path=planner.plan({0,0,0,0},boxes,nullptr,nullptr);
  ASSERT_TRUE(path);ASSERT_TRUE(path->complete);
  EXPECT_NEAR(planner.last.clearance_margin,0.10,1e-9);EXPECT_FALSE(planner.last.clearance_fallback);
  const auto obs=obstacle_array(boxes);
  for(size_t i=0;i<path->s.size();++i)
    EXPECT_GE(body_clearance(path->s[i],path->d[i],path->psi[i],obs,p),0.10-1e-6);
}
TEST(Planner, NarrowCorridorFallsBackWithoutLosingReachablePath) {
  Params p;p.margin_obstacle=0.03;p.margin_boundary=0.03;p.preferred_obstacle_clearance=0.10;
  PriorBoundary bd({-1,8},{0.4,0.4},{-0.4,-0.4});Planner planner(p,&bd);
  const std::vector<Box> boxes{{0.8,1.2,-0.1,0.1}};
  const auto path=planner.plan({0,0.25,0,0},boxes,nullptr,nullptr);
  ASSERT_TRUE(path);ASSERT_TRUE(path->complete);
  EXPECT_TRUE(planner.last.clearance_fallback);EXPECT_NEAR(planner.last.clearance_margin,0.03,1e-9);
  const auto obs=obstacle_array(boxes);double minimum=kInf;
  for(size_t i=0;i<path->s.size();++i) minimum=std::min(minimum,body_clearance(path->s[i],path->d[i],path->psi[i],obs,p));
  EXPECT_GE(minimum,0.03-1e-6);EXPECT_LT(minimum,0.10);
}
TEST(Mission, ConfirmedIslandMapSurvivesLostEvidenceAndPurpleLineThreshold) {
  Params p;p.use_lane=false;p.fork_shape=true;p.fork_sign_min=1;p.section_end=4.25;
  p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
  core.island_mapper.shape.pose={4.493,0.12,0.02,0,3,true,true};core.on_sign(1,true);
  auto step=[&](double t,double x) {
    core.on_odom(t,x,0,0,0,0);core.island_frame_at=t;core.grid->mark_free(-1,8,-0.77,0.77);
    core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t);
  };
  step(1.1,3.075);
  for(int i=0;i<8;++i)core.island_mapper.update({},4.25);
  ASSERT_FALSE(core.island_mapper.shape.pose.valid);
  core.search_started=0;core.search_distance=2.575;
  step(1.2,3.111);ASSERT_TRUE(core.path_);EXPECT_FALSE(core.searching);
  EXPECT_TRUE(core.navigation_island().pose.confirmed);EXPECT_NEAR(core.island_s(),4.493,1e-6);
  EXPECT_DOUBLE_EQ(core.search_distance,0);EXPECT_DOUBLE_EQ(core.search_started,-1);
  ASSERT_TRUE(core.reset_mapping(1.2));EXPECT_EQ(core.fork_side(),-1);
  EXPECT_FALSE(core.navigation_island().pose.valid);EXPECT_FALSE(core.path_);
}
TEST(StartGate, ReentryRequiresLeavingAndFreshStableEntrance) {
  EntranceReentry reentry;reentry.note_start(1);EntranceResult good{true,true,false,0.5,0,0,30};
  for(double t:{1.1,1.2,1.3,6.0,6.1}) EXPECT_FALSE(reentry.update(t,good,true));
  EXPECT_FALSE(reentry.update(6.2,{},false));EXPECT_FALSE(reentry.update(7.3,{},false));
  EXPECT_FALSE(reentry.update(7.4,good,true));EXPECT_FALSE(reentry.update(7.5,good,true));EXPECT_TRUE(reentry.update(7.6,good,true));
  EXPECT_FALSE(reentry.update(7.7,good,true));EXPECT_FALSE(reentry.update(8.0,good,true));
}

TEST(StartGate, ReentryRequiresWhiteLineInEveryConfirmationFrame) {
  EntranceReentry reentry;reentry.note_start(1);
  EntranceResult good{true,true,false,0.5,0,0,30};
  EXPECT_FALSE(reentry.update(6,{},false));
  EXPECT_FALSE(reentry.update(7.1,{},false));
  for(double t:{7.2,7.4,7.6,7.8}) EXPECT_FALSE(reentry.update(t,good,false));
  EXPECT_FALSE(reentry.update(8.0,good,true));
  EXPECT_FALSE(reentry.update(8.2,good,true));
  EXPECT_FALSE(reentry.update(8.4,good,false));
  EXPECT_FALSE(reentry.update(8.6,good,true));
  EXPECT_FALSE(reentry.update(8.8,good,true));
  EXPECT_TRUE(reentry.update(9.0,good,true));
}

TEST(Perception, LimoLongitudinalAndTransverseHaveExactDimensions) {
  Params p;p.obstacle_fixed_size=true;p.obstacle_min_length=0.322;p.obstacle_min_width=0.22;
  PriorBoundary boundary({-1,6},{1.,1.},{-1.,-1.});
  for(double yaw:{0.,M_PI/2}) {
    // Ray-cast a real rectangle, including occlusion of its far faces.
    std::vector<float> ranges(1440,8.f);
    const double c=std::cos(yaw),sn=std::sin(yaw);
    for(int i=0;i<1440;++i) {
      const double angle=-M_PI+i*2*M_PI/1440;
      const double origin[2]={-c,sn};
      const double dir[2]={c*std::cos(angle)+sn*std::sin(angle),-sn*std::cos(angle)+c*std::sin(angle)};
      const double half[2]={0.161,0.11};double lo=0,hi=8;
      for(int axis=0;axis<2;++axis) {
        if(std::abs(dir[axis])<1e-10) {if(std::abs(origin[axis])>half[axis])hi=-1;continue;}
        double t0=(-half[axis]-origin[axis])/dir[axis],t1=(half[axis]-origin[axis])/dir[axis];
        if(t0>t1) {std::swap(t0,t1);}
        lo=std::max(lo,t0);hi=std::min(hi,t1);
      }
      if(hi>=lo && lo>0)ranges[i]=lo;
    }
    std::vector<Box> boxes;std::vector<Pt2> pts;
    detect(ranges,-M_PI,2*M_PI/1440,{},RoadFrame(0,0,0),boundary,0,p,boxes,pts);
    ASSERT_EQ(boxes.size(),1u) << yaw;ASSERT_GT(pts.size(),10u);
    for(const auto & pt:pts) {
      EXPECT_GE(pt.s,boxes[0].s0-1e-8);EXPECT_LE(pt.s,boxes[0].s1+1e-8);
      EXPECT_GE(pt.d,boxes[0].d0-1e-8);EXPECT_LE(pt.d,boxes[0].d1+1e-8);
    }
    const bool transverse=std::abs(yaw-M_PI/2)<1e-8;
    EXPECT_NEAR(boxes[0].s1-boxes[0].s0,transverse?0.22:0.322,1e-9);
    EXPECT_NEAR(boxes[0].d1-boxes[0].d0,transverse?0.322:0.22,1e-9);
  }
}

TEST(Perception, LongWallKeepsMeasuredCollisionFootprintWithoutRobotCount) {
  Params p;p.obstacle_fixed_size=true;p.obstacle_min_length=.322;p.obstacle_min_width=.22;
  PriorBoundary boundary({-1,6},{.75,.75},{-.75,-.75});
  std::vector<float> ranges(1441,kInf);
  const double amin=-M_PI/2,inc=M_PI/1440;
  for(size_t i=0;i<ranges.size();++i) {
    const double a=amin+i*inc;
    if(std::abs(a)<1.1) ranges[i]=1/std::cos(a);
  }
  std::vector<Box> boxes;std::vector<Pt2> pts;std::vector<ClusterInfo> info;
  detect(ranges,amin,inc,{},RoadFrame(0,0,0),boundary,0,p,boxes,pts,&info);
  ASSERT_EQ(boxes.size(),1u);ASSERT_EQ(info.size(),1u);
  EXPECT_TRUE(info[0].wall);EXPECT_FALSE(boxes[0].robot_candidate);
  EXPECT_GT(boxes[0].d1-boxes[0].d0,1.5);  // never shrink to a 22/32cm robot
  EXPECT_LT(body_clearance(1,0,0,obstacle_array(boxes),p),0);  // still collidable
  EXPECT_GE(body_clearance(0,0,0,obstacle_array(boxes),p),0);
  EXPECT_EQ(eval_start_gate({},boxes,p).obstacles,0);
  Tracker tracker(p);
  tracker.update(boxes,0,-.2,nullptr,nullptr);tracker.update(boxes,.1,-.2,nullptr,nullptr);
  EXPECT_EQ(tracker.confirmed_count(),1);EXPECT_EQ(tracker.structure_count(),1);
  EXPECT_EQ(tracker.section_count(),0);ASSERT_EQ(tracker.confirmed_boxes().size(),1u);
  tracker.update({},.2,2.,nullptr,nullptr);EXPECT_EQ(tracker.passed,0);
}

TEST(Perception, WallClassificationUsesReturnsOutsideNarrowRoi) {
  Params p;p.obstacle_fixed_size=true;p.obstacle_min_length=.322;p.obstacle_min_width=.22;
  p.roi_margin=0;
  PriorBoundary boundary({-1,6},{.35,.35},{-.35,-.35});
  const double amin=-M_PI/2,inc=M_PI/1440;
  std::vector<float> ranges(1441,kInf);
  // Slanted wall y=0.1+0.8x. Its short ROI intersection resembles a robot.
  for(size_t i=0;i<ranges.size();++i) {
    const double a=amin+i*inc,den=std::sin(a)-.8*std::cos(a);
    const double r=.1/den;
    if(r>.05 && r<3.5 && r*std::cos(a)>0) ranges[i]=r;
  }
  std::vector<Box> boxes;std::vector<Pt2> pts;std::vector<ClusterInfo> info;
  detect(ranges,amin,inc,{},RoadFrame(0,0,0),boundary,0,p,boxes,pts,&info);
  ASSERT_EQ(boxes.size(),1u);ASSERT_EQ(info.size(),1u);
  EXPECT_LT(std::hypot(boxes[0].s1-boxes[0].s0,boxes[0].d1-boxes[0].d0),.55);
  EXPECT_GT(info[0].span,1.);EXPECT_TRUE(info[0].wall);EXPECT_FALSE(boxes[0].robot_candidate);
}

TEST(Perception, BagWallIsExcludedAndVisibleRobotFacesRemainCandidates) {
  const auto cases=YAML::LoadFile(std::string(TEST_DATA_DIR)+"/wall_bag_cases.json");
  Params p;p.obstacle_fixed_size=true;p.obstacle_min_length=.322;p.obstacle_min_width=.22;
  p.start_obs_y=.35;p.start_obs_min=2;p.start_dash_both=true;p.start_need_line=false;
  PriorBoundary boundary({-1,4},{.55,.55},{-.55,-.55});
  for(const auto & c:cases) {
    SCOPED_TRACE(c["bag"].as<std::string>()+" t="+c["bag_t"].as<std::string>());
    const auto ranges=c["ranges"].as<std::vector<float>>();
    std::vector<Box> boxes;std::vector<Pt2> pts;std::vector<ClusterInfo> info;
    detect(ranges,c["angle_min"].as<double>(),c["angle_inc"].as<double>(),{.103,0,0},
      RoadFrame(0,0,0),boundary,0,p,boxes,pts,&info);
    const double x=c["target"][0].as<double>(),y=c["target"][1].as<double>();
    double best=kInf;size_t index=0;
    for(size_t k=0;k<info.size();++k) {
      const double d=std::hypot(info[k].observed.cs()-x,info[k].observed.cd()-y);
      if(d<best){best=d;index=k;}
    }
    ASSERT_LT(best,.16);ASSERT_LT(index,boxes.size());
    EXPECT_EQ(boxes[index].robot_candidate,c["robot_candidate"].as<bool>());
    if(!c["robot_candidate"].as<bool>()) {
      EXPECT_TRUE(info[index].wall);EXPECT_GT(info[index].span,1.1);
      EXPECT_GT(boxes[index].s1-boxes[index].s0,1.1);
    }
  }
}

TEST(Tracker, RobotAndStructureRemainSeparateAndOnlyRobotCountsAsPassed) {
  Params p;Tracker tracker(p);
  const Box robot{1,1.322,0,.22},wall{1,1.322,.01,.23,false};
  for(double t:{0.,.1,.2}) tracker.update({robot,wall},t,-.2,nullptr,nullptr);
  ASSERT_EQ(tracker.confirmed_count(),2);EXPECT_EQ(tracker.structure_count(),1);
  EXPECT_EQ(tracker.section_count(),1);EXPECT_EQ(tracker.confirmed_boxes().size(),2u);
  tracker.update({},.3,2.,nullptr,nullptr);EXPECT_EQ(tracker.passed,1);
}

TEST(Mission, LostPathRetracesTwentyCentimetersAndExcludesFailedCorridor) {
  for(double radius:{100000.,1.2}) {
    Params p;p.use_lane=false;p.reverse_enable=true;p.reverse_wait=0.2;p.reverse_dist=0.20;p.reverse_max=1;
    ObjAvoidCore core(p);
    core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0));
    double t=0,x=0,y=0,psi=0,v=0;
    for(int i=1;i<=30;++i) {
      t=i*0.05;const double arc=i*0.015;psi=arc/radius;
      x=radius*std::sin(psi);y=radius*(1-std::cos(psi));
      core.on_odom(t,x,y,psi,0.3,0.3/radius);
    }
    const double stop_x=x,stop_y=y;
    bool backed=false,finished=false;double distance=0;
    PathPtr failed;
    for(int i=0;i<180;++i) {
      t+=0.04;core.on_odom(t,x,y,psi,v,v/radius);
      if(core.state!=State::REVERSE) {
        core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t);
        if(!failed && core.path_) failed=core.path_;
        core.clear_path();
      }
      // Once reversing, no rear scan or rear visibility is offered.
      double vc=0,steer=0;ASSERT_TRUE(core.control(t,0.04,vc,steer));
      if(core.state==State::REVERSE) backed=true;
      if(backed && core.reason=="reversed") {finished=true;break;}
      const double ds=vc*0.04;
      const double dpsi=ds*std::tan(steer)/p.wheelbase;
      x+=ds*std::cos(psi+dpsi/2);y+=ds*std::sin(psi+dpsi/2);psi+=dpsi;v=vc;
      if(vc<0) distance-=ds;
    }
    ASSERT_TRUE(backed);ASSERT_TRUE(finished) << core.reason;
    EXPECT_NEAR(distance,0.20,0.008);
    const double goal_arc=0.45-0.20;
    EXPECT_NEAR(x,radius*std::sin(goal_arc/radius),0.025);
    EXPECT_NEAR(y,radius*(1-std::cos(goal_arc/radius)),0.025);
    EXPECT_GT(std::hypot(stop_x-x,stop_y-y),0.18);
    const std::string status=core.status_json();
    EXPECT_EQ(status.find("\"excluded_regions\":0"),std::string::npos);
    ASSERT_TRUE(failed);
    // Nearby alternatives on the failed corridor must also be rejected.
    t+=0.1;core.on_odom(t,x,y,psi,0,0);
    core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t);
    if(core.path_) {
      for(size_t k=0;k<core.path_->s.size();++k) {
        const double sk=core.path_->s[k];
        if(sk>=0.65 && sk<=0.95) {
          EXPECT_GT(std::abs(core.path_->d[k]-failed->d_at(sk)),0.15);
        }
      }
    }
    const auto before=YAML::Load(core.status_json())["recovery"]["excluded_regions"].as<int>();
    ASSERT_GT(before,0);
    // Carry the vehicle back to another odometry pose and re-anchor the same course.
    t+=0.1;core.on_odom(t,3.0,-1.0,0.5,0,0);
    ASSERT_TRUE(core.reset_mapping(t));
    EXPECT_EQ(YAML::Load(core.status_json())["recovery"]["excluded_regions"].as<int>(),before);
    core.on_odom(t+0.1,3.0,-1.0,0.5,0,0);
    core.process_scan(t+0.1,std::vector<float>(720,8),-M_PI,2*M_PI/720,t+0.1);
    if(core.path_) for(size_t k=0;k<core.path_->s.size();++k) {
      const double sk=core.path_->s[k];
      if(sk>=0.65 && sk<=0.95) {EXPECT_GT(std::abs(core.path_->d[k]-failed->d_at(sk)),0.15);}
    }
  }
}
TEST(Mission, LostPathDoesNotBackWithoutDrivenHistory) {
  Params p;p.use_lane=false;p.reverse_enable=true;p.reverse_wait=0.1;ObjAvoidCore core(p);
  core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0));
  for(int i=1;i<=60;++i) {
    const double t=i*0.05;core.on_odom(t,0,0,0,0,0);
    core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t);core.clear_path();
    double v=0,steer=0;core.control(t,0.05,v,steer);
    EXPECT_GE(v,0);EXPECT_NE(core.state,State::REVERSE);
  }
}

TEST(Mission, RetraceStopsWhenOdometryIsStale) {
  Params p;p.use_lane=false;p.reverse_enable=true;p.reverse_wait=0.1;ObjAvoidCore core(p);
  core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0));
  for(int i=1;i<=30;++i) core.on_odom(i*0.05,i*0.015,0,0,0.3,0);
  double t=1.5;bool backing=false;
  for(int i=0;i<40;++i) {
    t+=0.05;core.on_odom(t,0.45,0,0,0,0);
    core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t);core.clear_path();
    double v=0,steer=0;core.control(t,0.05,v,steer);
    if(core.state==State::REVERSE) {backing=true;break;}
  }
  ASSERT_TRUE(backing);
  double v=-1,steer=0;core.control(t+p.odom_timeout+0.1,0.05,v,steer);
  EXPECT_DOUBLE_EQ(v,0);EXPECT_EQ(core.state,State::STOP);EXPECT_EQ(core.reason,"odom_stale");
}

TEST(Mission, NearForkWithoutSignOrIslandDoesNotEnterSearchLimit) {
  Params p;p.use_lane=false;p.fork_shape=true;p.section_end=0.5;
  p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
  ObjAvoidCore core(p);core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0));
  for(double t:{0.0,7.0}) {
    core.on_odom(t,0,0,0,0,0);
    core.search_started=0;core.search_distance=2;
    ASSERT_TRUE(core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t));
    EXPECT_FALSE(core.searching);EXPECT_EQ(core.reason=="search_limit",false);
    EXPECT_DOUBLE_EQ(core.search_distance,0);EXPECT_TRUE(core.path_);
  }
}


TEST(Mission, IslandFitWaitsForPurpleLineAndReusesEarlierObservations) {
  for(bool outline:{false,true}) {
    Params p;p.fork_shape=true;p.road_outline=outline;p.lane_align=false;
    p.lane_lidar_filter=false;p.fork_sign_min=1;
    p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
    ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
    core.on_sign(1.01,false);
    IslandShape truth;truth.pose={4.6,0.05,0.1,0,0,true,true};
    const auto vertices=truth.vertices();std::vector<XY> full,side;
    for(int edge:{0,1,5}) for(int i=0;i<30;++i) {
      const auto a=vertices[edge],b=vertices[(edge+1)%6];const double t=i/29.0;
      const XY q{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)};
      full.push_back(q);if(edge==1) side.push_back(q);
    }
    const std::vector<float> scan(720,8);
    auto image=[&](double t,double x,const std::vector<XY> & world) {
      core.on_odom(t,x,0,0,0,0);std::vector<XY> base;
      for(const auto & q:world) base.push_back({q.x-x,q.y});
      EXPECT_TRUE(core.on_lane(t,base,{},nullptr));
      EXPECT_TRUE(core.process_scan(t,scan,-M_PI,2*M_PI/720,t));
    };
    const double crossing=p.section_end-p.body_front;
    for(double t:{2.0,2.1,2.2}) image(t,crossing-0.02,full);
    EXPECT_FALSE(core.island_section_reached_at);
    EXPECT_FALSE(core.island_mapper.shape.pose.valid);
    EXPECT_EQ(core.island_mapper.evaluations,0u);
    EXPECT_STREQ(core.island_mapper.rejection,"waiting_section_line");
    EXPECT_FALSE(core.fork_goal());EXPECT_FALSE(core.planner.fork_target);
    if(core.path_) {EXPECT_LE(core.path_->end_s(),p.section_end+.10001);}
    // Arrival may use the latest image even when captured before the crossing.
    core.on_odom(2.3,crossing,0,0,0,0);
    ASSERT_TRUE(core.process_scan(2.3,scan,-M_PI,2*M_PI/720,2.3));
    ASSERT_TRUE(core.island_section_reached_at);
    EXPECT_DOUBLE_EQ(*core.island_section_reached_at,2.3);
    EXPECT_DOUBLE_EQ(core.island_fit_stamp,2.2);
    ASSERT_TRUE(core.island_mapper.shape.pose.valid);
    ASSERT_TRUE(core.island_mapper.shape.pose.confirmed);
    ASSERT_TRUE(core.fork_goal());ASSERT_TRUE(core.planner.fork_target);
    EXPECT_GE(core.island_mapper.input_count,full.size());
    // Earlier corners remain usable with fresh side-only observations.
    for(double t:{2.4,2.5}) image(t,crossing,side);
    ASSERT_TRUE(core.island_mapper.shape.pose.confirmed);
    EXPECT_NEAR(core.island_s(),4.6,0.08);
    ASSERT_TRUE(core.fork_goal());ASSERT_TRUE(core.planner.fork_target);
    image(2.7,crossing-0.1,full);
    EXPECT_DOUBLE_EQ(*core.island_section_reached_at,2.3);
    const auto status=YAML::Load(core.status_json());
    EXPECT_TRUE(status["island_mapping"]["section_reached"].as<bool>());
    ASSERT_TRUE(core.reset_mapping(2.7));
    EXPECT_FALSE(core.island_section_reached_at);
    EXPECT_FALSE(core.island_mapper.shape.pose.valid);
    core.on_odom(1,0,0,0,0,0);
    EXPECT_FALSE(core.island_section_reached_at);
  }
}

TEST(Mission, PurpleLineStillRequiresDirectionAndPostSignObservations) {
  Params p;p.fork_shape=true;p.lane_align=false;p.lane_lidar_filter=false;p.fork_sign_min=1;
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
  const std::vector<float> scan(720,8);
  core.on_odom(2,p.section_end-p.body_front,0,0,0,0);
  // The image precedes arrival, but processing occurs after arrival.
  ASSERT_TRUE(core.on_lane(1.9,{}, {},nullptr));
  ASSERT_TRUE(core.process_scan(2,scan,-M_PI,2*M_PI/720,2));
  ASSERT_TRUE(core.island_section_reached_at);
  EXPECT_STREQ(core.island_mapper.rejection,"waiting_lr_sign");
  core.on_sign(2,false);
  ASSERT_TRUE(core.on_lane(1.95,{}, {},nullptr));
  ASSERT_TRUE(core.process_scan(2,scan,-M_PI,2*M_PI/720,2));
  EXPECT_EQ(core.island_mapper.evaluations,0u);
  EXPECT_LT(core.island_fit_stamp,0);
  core.on_odom(2.1,p.section_end-p.body_front,0,0,0,0);
  ASSERT_TRUE(core.on_lane(2.1,{}, {},nullptr));
  ASSERT_TRUE(core.process_scan(2.1,scan,-M_PI,2*M_PI/720,2.1));
  EXPECT_DOUBLE_EQ(core.island_fit_stamp,2.1);
  core.on_odom(1,0,0,0,0,0);
  EXPECT_FALSE(core.island_section_reached_at);
}


TEST(Mission, IslandCorrectsRoadSidesWithoutMovingEgoFrame) {
  for(double turn:{-0.35,0.35}) {
    Params p;p.fork_shape=true;p.road_outline=true;p.lane_lidar_filter=false;p.fork_sign_min=1;
    p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
    ObjAvoidCore core(p);core.on_odom(1,1.2,-0.8,0.4,0,0);ASSERT_TRUE(core.activate(1));
    core.aligned=true;  // The entrance map has already been aligned.
    const RoadFrame initial=*core.frame;core.on_sign(1.01,false);
    IslandShape truth;truth.pose={4.6,0.10,0.08,0,0,true,true};
    const auto vertices=truth.vertices();std::vector<XY> world;
    for(int edge:{0,1,5}) for(int i=0;i<30;++i) {
      const auto a=vertices[edge],b=vertices[(edge+1)%6];const double u=i/29.0;
      XY q;initial.to_odom(a.x+u*(b.x-a.x),a.y+u*(b.y-a.y),q.x,q.y);world.push_back(q);
    }
    double car_x,car_y;initial.to_odom(4.1,-0.1,car_x,car_y);
    const double heading=initial.yaw0+turn,c=std::cos(heading),sn=std::sin(heading);
    const std::vector<float> scan(720,8);
    auto image=[&](double t,const std::vector<XY> & observed) {
      core.on_odom(t,car_x,car_y,heading,0,0);std::vector<XY> base;
      for(const auto & q:observed) base.push_back({c*(q.x-car_x)+sn*(q.y-car_y),-sn*(q.x-car_x)+c*(q.y-car_y)});
      XY tip;initial.to_odom(truth.pose.s,truth.pose.d,tip.x,tip.y);
      const std::vector<XY> view{{c*(tip.x-car_x)+sn*(tip.y-car_y),-sn*(tip.x-car_x)+c*(tip.y-car_y)}};
      EXPECT_TRUE(core.on_lane(t,base,view,nullptr));
      EXPECT_TRUE(core.process_scan(t,scan,-M_PI,2*M_PI/720,t));
      EXPECT_DOUBLE_EQ(core.frame->x0,initial.x0);EXPECT_DOUBLE_EQ(core.frame->y0,initial.y0);
      EXPECT_DOUBLE_EQ(core.frame->yaw0,initial.yaw0);
    };
    for(double t:{2.0,2.15,2.30}) image(t,world);
    ASSERT_TRUE(core.island_section_reached_at);
    ASSERT_TRUE(core.island_mapper.shape.pose.confirmed);
    const auto island=core.navigation_island();
    EXPECT_NEAR(island.pose.s,truth.pose.s,0.08);EXPECT_NEAR(island.pose.d,truth.pose.d,0.06);
    EXPECT_NEAR(island.pose.yaw,truth.pose.yaw,0.09);
    // The island retains its measured offset; it is not forced to d=yaw=0.
    EXPECT_GT(island.pose.d,0.04);
    const double sample_s=4.9;double saved_r,saved_l;core.boundary()->limits(sample_s,saved_r,saved_l);
    const double center=island.pose.d+std::tan(island.pose.yaw)*(sample_s-island.pose.s);
    EXPECT_NEAR((saved_l-center)*std::cos(island.pose.yaw)-island.width/2,0.50,1e-9);
    EXPECT_NEAR((center-saved_r)*std::cos(island.pose.yaw)-island.width/2,0.50,1e-9);
    const auto goal=core.fork_goal();ASSERT_TRUE(goal);
    // Contradictory outer paint cannot override the confirmed island reference.
    std::vector<XY> shifted_outer;RoadOutline outline(p);
    for(const auto & edge:outline.edges) for(int i=0;i<60;++i) {
      const double u=i/59.0,sk=edge.first.x+u*(edge.second.x-edge.first.x);
      const double d=edge.first.y+u*(edge.second.y-edge.first.y);
      XY q;initial.to_odom(std::cos(-0.04)*sk-std::sin(-0.04)*d,
        0.05+std::sin(-0.04)*sk+std::cos(-0.04)*d,q.x,q.y);shifted_outer.push_back(q);
    }
    for(double t:{2.45,2.60,2.90}) {
      image(t,shifted_outer);double r,l;core.boundary()->limits(sample_s,r,l);
      EXPECT_DOUBLE_EQ(r,saved_r);EXPECT_DOUBLE_EQ(l,saved_l);
    }
    const auto status=YAML::Load(core.status_json());
    EXPECT_EQ(status["road_mapping"]["center_source"].as<std::string>(),"island");
    EXPECT_FALSE(status["road_mapping"]["frozen"].as<bool>());
    EXPECT_TRUE(status["road_mapping"]["island_anchored"].as<bool>());
    EXPECT_DOUBLE_EQ(status["road_mapping"]["island_side_gap_m"].as<double>(),0.5);
    ASSERT_TRUE(core.fork_goal());EXPECT_DOUBLE_EQ(core.fork_goal()->x,goal->x);EXPECT_DOUBLE_EQ(core.fork_goal()->y,goal->y);
    core.on_sign(2.91,true);EXPECT_EQ(core.fork_side(),1);
    // Fresh island observations may correct the road again: there is no map freeze.
    std::vector<XY> shifted_island=world;
    for(auto & q:shifted_island) {q.x-=initial.s*0.05;q.y+=initial.c*0.05;}
    for(double t:{3.0,3.15,3.30,3.45,3.60,3.75}) image(t,shifted_island);
    const auto updated=core.navigation_island();EXPECT_GT(updated.pose.d,island.pose.d+0.015);
    double new_r,new_l;core.boundary()->limits(sample_s,new_r,new_l);
    EXPECT_GT(new_l,saved_l+0.015);
    const double axis=updated.pose.d+std::tan(updated.pose.yaw)*(sample_s-updated.pose.s);
    EXPECT_NEAR((new_l-axis)*std::cos(updated.pose.yaw)-updated.width/2,0.5,1e-9);
    EXPECT_NEAR((axis-new_r)*std::cos(updated.pose.yaw)-updated.width/2,0.5,1e-9);
    EXPECT_DOUBLE_EQ(core.fork_goal()->x,goal->x);EXPECT_DOUBLE_EQ(core.fork_goal()->y,goal->y);
    ASSERT_TRUE(core.reset_mapping(3.75));
    EXPECT_FALSE(YAML::Load(core.status_json())["road_mapping"]["island_anchored"].as<bool>());
    core.on_odom(1,0,0,0,0,0);EXPECT_FALSE(core.fork_goal());
  }
}

TEST(Outline, IslandRoadHasFiftyCentimeterNormalGapAndContinuousTapers) {
  Params p;
  for(double yaw:{-0.25,0.,0.25}) for(double offset:{-0.17,0.13}) {
    IslandShape island;island.pose={3.1,offset,yaw,0,3,true,true};RoadOutline road(p,island);
    for(double s:{3.3,3.8,4.3}) {
      double r,l;road.limits(s,r,l);const double center=offset+std::tan(yaw)*(s-3.1);
      EXPECT_NEAR((l-center)*std::cos(yaw)-island.width/2,0.5,1e-9);
      EXPECT_NEAR((center-r)*std::cos(yaw)-island.width/2,0.5,1e-9);
      EXPECT_NEAR((l-r)*std::cos(yaw),1.5,1e-9);
    }
    for(const auto & edge:road.edges) for(const auto q:{edge.first,edge.second}) {
      double r,l;road.limits(q.x,r,l);EXPECT_TRUE(std::isfinite(r));EXPECT_TRUE(std::isfinite(l));
      EXPECT_LT(std::min(std::abs(q.y-r),std::abs(q.y-l)),1e-8);
    }
  }
}

TEST(Mission, SelectedForkPathStaysAndOnlyYellowBlockingBacksOppositeTwentyCentimeters) {
  for(int side:{-1,1}) for(bool island_paint:{false,true}) {
    Params p;p.fork_shape=true;p.fork_sign_min=1;p.road_outline=false;p.lane_align=false;
    p.section_end=1.;p.reverse_wait=0.1;p.reverse_max=3;p.lane_lidar_filter=false;
    p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
    ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
    core.aligned=true;core.on_sign(1.01,side<0);core.island_mapper.shape.pose={3,0,0,0,3,true,true};
    core.island_section_reached_at=1.1;
    const std::vector<float> clear(720,8);
    double t=1.1,x=.7,y=0,yaw=0,v=0,delta=0;
    auto scan=[&] {
      core.on_odom(t,x,y,yaw,0,0);core.island_frame_at=core.island_fit_stamp=t;
      core.grid->mark_free(-1,8,-1,1);
      EXPECT_TRUE(core.process_scan(t,clear,-M_PI,2*M_PI/720,t));
    };
    scan();ASSERT_TRUE(core.path_) << core.status_json();
    const auto target=core.fork_goal();ASSERT_TRUE(target);
    ASSERT_GE(core.path_->end_s(),target->x);
    ASSERT_NEAR(core.path_->d_at(target->x),target->y,.06);
    const auto chosen=core.path_;
    for(int i=0;i<4;++i) {t+=.1;scan();EXPECT_EQ(core.path_,chosen);}
    // A new object stops the selected path, without choosing another side or reversing.
    Track object;object.id=70;object.confirmed=true;object.hits=10;object.last_seen=t;
    object.box={1.4,1.7,-.65,.65};core.tracker->tracks.push_back(object);
    for(int i=0;i<4;++i) {t+=.05;scan();core.control(t,.05,v,delta);EXPECT_GE(v,0);}
    EXPECT_EQ(core.path_,chosen);EXPECT_NE(core.state,State::REVERSE);
    EXPECT_EQ(core.reason,"path_obstacle_blocked");
    core.tracker->tracks.clear();
    // Move only the observed yellow boundary across the selected corridor.
    auto * lane=const_cast<LaneBoundary *>(core.lane_boundary());
    ASSERT_TRUE(lane);
    if(island_paint) core.island_mapper.shape.pose.d=side*0.25;
    else for(size_t i=0;i<lane->s_bins.size();++i) if(lane->s_bins[i]>1.0) {
      if(side>0) lane->left[i]=0.3;else lane->right[i]=-0.3;
    }
    bool reversing=false,finished=false;double moved=0,back_y=0;
    for(int i=0;i<160;++i) {
      t+=.04;scan();core.control(t,.04,v,delta);
      if(core.state==State::REVERSE) {
        reversing=true;EXPECT_EQ(core.path_,chosen);
        EXPECT_LE(side*delta,0.001);EXPECT_GE(v,-.08001);
      }
      if(v<0) {
        moved+=-v*.04;
        const double next_yaw=yaw+v*std::tan(delta+p.steer_trim)/p.wheelbase*.04;
        x+=v*std::cos((yaw+next_yaw)/2)*.04;y+=v*std::sin((yaw+next_yaw)/2)*.04;
        yaw=next_yaw;back_y=y;
      }
      if(reversing && core.reason=="reversed") {finished=true;break;}
    }
    EXPECT_TRUE(reversing);EXPECT_TRUE(finished);EXPECT_NEAR(moved,.20,.006);
    EXPECT_LT(side*back_y,0.); // left goal -> right rear; right goal -> left rear
    ASSERT_TRUE(core.fork_goal());EXPECT_DOUBLE_EQ(core.fork_goal()->x,target->x);
    EXPECT_DOUBLE_EQ(core.fork_goal()->y,target->y);
    // A persistent blocked line must not cause several blind 20cm reverses.
    for(int i=0;i<4;++i) {
      t+=.15;scan();core.control(t,.04,v,delta);
      EXPECT_GE(v,0);EXPECT_NE(core.state,State::REVERSE);
    }
    core.island_mapper.shape.pose.d=0;lane->rebuild({});t+=.04;scan();
    for(int i=0;i<8 && (!core.path_ || core.path_->end_s()<target->x);++i) {t+=.15;scan();}
    ASSERT_TRUE(core.path_);ASSERT_GE(core.path_->end_s(),target->x) << core.status_json();
    EXPECT_NEAR(core.path_->d_at(target->x),target->y,.06);
  }
}

TEST(Outline, RecoversLargeLateralOffsetFromVisibleBendAndKeepsVehicleInside) {
  Params p;RoadOutline outline(p);
  for(double offset:{-.9,-.75,-.55,.55,.75,.9}) {
    const double yaw=.06,c=std::cos(yaw),sn=std::sin(yaw);
    std::vector<XY> observed;
    // Only one outer boundary is visible, including its straight/ramp transition.
    for(int k=0;k<3;++k) for(int i=0;i<80;++i) {
      const auto edge=outline.edges[k];const double u=i/79.;
      const double s=edge.first.x+u*(edge.second.x-edge.first.x),d=edge.first.y+u*(edge.second.y-edge.first.y);
      if(s<-.4 || s>2.) continue;
      observed.push_back({c*s-sn*d,offset+sn*s+c*d});
    }
    const XY car{c,offset+sn};
    const auto fit=outline.fit(observed,true,car);ASSERT_TRUE(fit.valid) << offset;
    EXPECT_NEAR(fit.d,offset,.06);EXPECT_NEAR(fit.yaw,yaw,.04);
    double s,d;RoadFrame(fit.s,fit.d,fit.yaw).to_road(car.x,car.y,s,d);
    double r,l;outline.limits(s,r,l);EXPECT_GT(d,r);EXPECT_LT(d,l);
  }
}

TEST(Mission, ResetEpochClearsObservedPointsAndRejectsPacketsFromPreviousMap) {
  Params p;p.fork_shape=true;p.lane_lidar_filter=false;p.lane_align=false;
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
  std::vector<XY> points;for(int i=0;i<20;++i) points.push_back({.5+i*.03,.7});
  for(double t:{1.1,1.2,1.3,1.4}) {
    core.on_odom(t,0,0,0,0,0);ASSERT_TRUE(core.on_lane(t,points,points,&points));
  }
  ASSERT_FALSE(core.ymap->confirmed().empty());ASSERT_FALSE(core.yellow_observed.empty());
  const auto epoch=core.mapping_epoch;core.on_odom(2,0,0,0,0,0);
  ASSERT_TRUE(core.reset_mapping(2));EXPECT_EQ(core.mapping_epoch,epoch+1);
  EXPECT_TRUE(core.ymap->confirmed().empty());EXPECT_TRUE(core.wmap->confirmed().empty());
  EXPECT_TRUE(core.yellow_observed.empty());EXPECT_TRUE(core.white_observed.empty());
  EXPECT_TRUE(core.yellow_history.empty());EXPECT_FALSE(core.fork_goal());EXPECT_FALSE(core.path_);
  EXPECT_FALSE(core.on_lane(1.4,points,points,&points));
  EXPECT_FALSE(core.process_scan(1.4,std::vector<float>(720,8),-M_PI,2*M_PI/720,2));
  EXPECT_TRUE(core.yellow_observed.empty());
  core.on_odom(2.1,0,0,0,0,0);EXPECT_TRUE(core.on_lane(2.1,points,points,&points));
  EXPECT_FALSE(core.yellow_observed.empty());
  EXPECT_EQ(YAML::Load(core.status_json())["mapping_epoch"].as<unsigned long>(),core.mapping_epoch);
}

namespace {
Params startup_params() {
  Params p;p.use_lane=false;p.fork_enable=false;p.section_end=3.;
  p.boundary_s={-1,6};p.boundary_left={.8,.8};p.boundary_right={-.8,-.8};return p;
}
void startup_scan(ObjAvoidCore & core,double t,double speed=0) {
  core.on_odom(t,0,0,0,speed,0);core.grid->mark_free(-1,6,-.8,.8);
  ASSERT_TRUE(core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t));
}
}
TEST(Mission, StartupHoldsThreeSecondsWhileMappingThenTracksReadyPath) {
  ObjAvoidCore core(startup_params());core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0,0,0,0,true));
  double v=1,steer=0;
  for(double t:{0.,1.,2.999}) {
    startup_scan(core,t);ASSERT_TRUE(core.path_);EXPECT_TRUE(core.path_->complete);
    ASSERT_TRUE(core.control(t,.04,v,steer));EXPECT_DOUBLE_EQ(v,0);EXPECT_EQ(core.reason,"startup_hold");
  }
  startup_scan(core,3.);ASSERT_TRUE(core.control(3.,.04,v,steer));EXPECT_GT(v,0);
  EXPECT_FALSE(YAML::Load(core.status_json())["startup"]["waiting"].as<bool>());
}
TEST(Mission, StartupMeasuresThreeSecondsFromMeasuredStop) {
  ObjAvoidCore core(startup_params());core.on_odom(0,0,0,0,.15,0);ASSERT_TRUE(core.activate(0,0,0,0,true));
  double v=1,steer=0;startup_scan(core,0,.15);core.control(0,.04,v,steer);
  EXPECT_DOUBLE_EQ(v,0);EXPECT_EQ(core.reason,"startup_stopping");
  for(double t:{1.,3.999}) {startup_scan(core,t);core.control(t,.04,v,steer);EXPECT_DOUBLE_EQ(v,0);}
  startup_scan(core,4.);core.control(4.,.04,v,steer);EXPECT_GT(v,0);
}
TEST(Mission, StartupWaitsPastThreeSecondsForCompleteVerifiedUnblockedPath) {
  for(int defect=0;defect<4;++defect) {
    SCOPED_TRACE(defect);
    ObjAvoidCore core(startup_params());core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0,0,0,0,true));
    double v=1,steer=0;startup_scan(core,0);core.control(0,.04,v,steer);
    startup_scan(core,3.);ASSERT_TRUE(core.path_);const auto good=core.path_;
    if(defect==0) core.clear_path();
    if(defect==1) {auto partial=std::make_shared<Path>(*good);partial->complete=false;core.path_=partial;}
    if(defect==2) core.verified_s=.1;
    if(defect==3) {Track object;object.confirmed=true;object.box={.8,1.1,-.5,.5};core.tracker->tracks.push_back(object);}
    core.control(3.,.04,v,steer);EXPECT_DOUBLE_EQ(v,0);EXPECT_EQ(core.reason,"startup_path_wait");
    core.path_=good;core.tracker->tracks.clear();startup_scan(core,3.1);
    core.control(3.1,.04,v,steer);EXPECT_GT(v,0);
  }
}
TEST(Mission, StartupRestartsStopConfirmationWhenOdometryIsStale) {
  ObjAvoidCore core(startup_params());core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0,0,0,0,true));
  double v=1,steer=0;startup_scan(core,0);core.control(0,.04,v,steer);
  core.control(3.,.04,v,steer);EXPECT_DOUBLE_EQ(v,0);EXPECT_EQ(core.reason,"odom_stale");
  for(double t:{4.,6.999}) {startup_scan(core,t);core.control(t,.04,v,steer);EXPECT_DOUBLE_EQ(v,0);}
  startup_scan(core,7.);core.control(7.,.04,v,steer);EXPECT_GT(v,0);
}
TEST(Mission, StartupHoldSurvivesMapResetWithoutEarlyReleaseOrRestart) {
  ObjAvoidCore core(startup_params());core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0,0,0,0,true));
  double v=1,steer=0;startup_scan(core,0);core.control(0,.04,v,steer);
  startup_scan(core,1.);ASSERT_TRUE(core.reset_mapping(1.));
  startup_scan(core,2.999);core.control(2.999,.04,v,steer);EXPECT_DOUBLE_EQ(v,0);
  startup_scan(core,3.);core.control(3.,.04,v,steer);EXPECT_GT(v,0);
}

TEST(Mission, CrossingYellowPaintKeepsInsidePathAndSteersBackInsteadOfStopping) {
  for(double side:{-1.,1.}) for(bool have_path:{false,true}) {
    Params p=startup_params();p.boundary_left={.5,.5};p.boundary_right={-.5,-.5};p.deviation_stop=.05;
    ObjAvoidCore core(p);core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0));
    if(have_path) startup_scan(core,0);
    const auto old=core.path_;
    core.on_odom(1,0,side*.7,0,0,0);core.grid->mark_free(-1,6,-1,1);
    ASSERT_TRUE(core.process_scan(1,std::vector<float>(720,8),-M_PI,2*M_PI/720,1));
    ASSERT_TRUE(core.path_);if(have_path) {EXPECT_EQ(core.path_,old);}
    double v=0,steer=0;ASSERT_TRUE(core.control(1,.1,v,steer));EXPECT_GT(v,0);
    EXPECT_LT(side*(steer+p.steer_trim),0);EXPECT_EQ(core.reason,"tracking");
    for(size_t i=0;i<core.path_->s.size();++i)
      EXPECT_GE(boundary_clearance(core.path_->s[i],core.path_->d[i],core.path_->psi[i],*core.boundary(),p),-1e-6);
    Track object;object.confirmed=true;object.box={-.1,.1,side*.7-.1,side*.7+.1};core.tracker->tracks.push_back(object);
    core.control(1,.1,v,steer);EXPECT_DOUBLE_EQ(v,0);EXPECT_EQ(core.reason,"recovery_obstacle");
  }
}
TEST(Outline, VehicleOutsidePaintDoesNotDragTheMatchedRoadAroundIt) {
  Params p;RoadOutline road(p);std::vector<XY> points;
  for(const auto & edge:road.edges) for(int i=0;i<50;++i) {
    const double u=i/49.;points.push_back({edge.first.x+u*(edge.second.x-edge.first.x),edge.first.y+u*(edge.second.y-edge.first.y)});
  }
  const auto fit=road.fit(points,true,XY{1,1.2});ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(fit.s,0,.03);EXPECT_NEAR(fit.d,0,.03);EXPECT_NEAR(fit.yaw,0,.02);
}
TEST(Mission, IslandUsesAccumulatedCornersOlderThanRecentImageWindow) {
  Params p;p.fork_shape=true;p.road_outline=true;p.lane_lidar_filter=false;p.fork_sign_min=1;
  p.boundary_s={-1,8};p.boundary_left={.77,.77};p.boundary_right={-.77,-.77};
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));core.aligned=true;core.on_sign(1.01,false);
  IslandShape island;island.pose={4.6,.04,.05,0,0,true,true};const auto v=island.vertices();
  std::vector<XY> full,side;
  for(int edge:{0,1,5}) for(int i=0;i<30;++i) {
    const auto a=v[edge],b=v[(edge+1)%6];const double u=i/29.;XY q{a.x+u*(b.x-a.x),a.y+u*(b.y-a.y)};
    full.push_back(q);if(edge==1)side.push_back(q);
  }
  const double crossing=p.section_end-p.body_front;
  auto image=[&](double t,double x,const std::vector<XY> & world) {
    core.on_odom(t,x,0,0,0,0);std::vector<XY> base;for(const auto & q:world)base.push_back({q.x-x,q.y});
    ASSERT_TRUE(core.on_lane(t,base,{},nullptr));core.grid->mark_free(-1,8,-.8,.8);
    ASSERT_TRUE(core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t));
  };
  for(double t:{2.,2.1,2.2}) image(t,crossing-.1,full);
  image(5.,crossing,side);
  ASSERT_EQ(core.yellow_history.size(),1u); // The old corners are no longer in the recent frame cache.
  ASSERT_TRUE(core.island_mapper.shape.pose.valid);EXPECT_NEAR(core.island_s(),4.6,.08);
  EXPECT_GT(core.island_mapper.input_count,side.size());EXPECT_FALSE(core.island_mapper.shape.pose.confirmed);
  for(double t:{5.1,5.2}) image(t,crossing,side);
  EXPECT_TRUE(core.navigation_island().pose.confirmed);EXPECT_TRUE(core.fork_goal());
  EXPECT_EQ(YAML::Load(core.status_json())["island_mapping"]["observation_source"].as<std::string>(),"accumulated_map");
}
TEST(Mission, OneCachedIslandFrameCannotBeCountedAsThreeIndependentObservations) {
  Params p;p.fork_shape=true;p.road_outline=true;p.lane_lidar_filter=false;p.fork_sign_min=1;
  p.boundary_s={-1,8};p.boundary_left={.77,.77};p.boundary_right={-.77,-.77};
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));core.aligned=true;core.on_sign(1.01,false);
  IslandShape island;island.pose={4.6,.04,.05,0,0,true,true};const auto v=island.vertices();
  std::vector<XY> full;for(int edge:{0,1,5})for(int i=0;i<30;++i){const auto a=v[edge],b=v[(edge+1)%6];const double u=i/29.;full.push_back({a.x+u*(b.x-a.x),a.y+u*(b.y-a.y)});}
  const double crossing=p.section_end-p.body_front;
  core.on_odom(2,crossing-.1,0,0,0,0);std::vector<XY> base;for(const auto & q:full)base.push_back({q.x-(crossing-.1),q.y});
  ASSERT_TRUE(core.on_lane(2,base,{},nullptr));
  for(double t:{2.1,2.2,2.3}) {core.on_odom(t,crossing,0,0,0,0);core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t);}
  ASSERT_TRUE(core.island_mapper.shape.pose.valid);EXPECT_EQ(core.island_mapper.shape.pose.stable,1);
  EXPECT_FALSE(core.navigation_island().pose.confirmed);EXPECT_FALSE(core.fork_goal());
}
