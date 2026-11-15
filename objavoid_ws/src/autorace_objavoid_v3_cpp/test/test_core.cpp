// C++ 코어 단위 테스트(ROS 없이): 파라미터 동기화, 계획, PD 추종, 후진 탈출
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
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
TEST(Mission, ReverseEscapeWhenBlocked)
{
  Params p;
  p.use_lane = false;
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
  ASSERT_LT(i, states.size()) << "후진하지 않음";
  const double xmax = *std::max_element(xs.begin(), xs.begin() + i);
  const double xmin = *std::min_element(xs.begin() + i, xs.end());
  EXPECT_LT(xmax + p.body_front, wall_x + p.lidar_x);  // 벽에 닿기 전에 섬
  EXPECT_NEAR(xmax - xmin, p.reverse_dist, 0.03);
  EXPECT_EQ(core.state, State::STOP);
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
