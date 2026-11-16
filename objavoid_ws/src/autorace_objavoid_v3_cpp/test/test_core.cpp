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
  cv::Mat mask(100,100,CV_8UC1),bgr(100,100,CV_8UC3);
  for(int v=0;v<100;++v) for(int u=0;u<100;++u) {
    mask.at<uchar>(v,u)=u<50?1:2;
    bgr.at<cv::Vec3b>(v,u)=u<50?cv::Vec3b(0,200,200):cv::Vec3b(255,255,255);
  }
  const auto yellow=full.yellow_points(mask,bgr),white=full.white_points(mask,bgr);
  ASSERT_FALSE(yellow.empty());ASSERT_FALSE(white.empty());
  bool near=false,left=false,right=false;
  for(const auto & points:{yellow,white}) for(const auto & q:points) {
    EXPECT_LE(q.x,2);EXPECT_TRUE(std::isfinite(q.x));near|=q.x<0.5;left|=q.y>0.8;right|=q.y< -0.8;
  }
  EXPECT_TRUE(near);EXPECT_TRUE(left);EXPECT_TRUE(right);
  EXPECT_GT(yellow.size(),cropped.yellow_points(mask,bgr).size());
  const auto dense=full.class_points(mask,bgr,1,false,1);
  EXPECT_GT(double(yellow.size())/dense.size(),0.20);EXPECT_LT(double(yellow.size())/dense.size(),0.30);
  bgr.setTo(cv::Scalar(0,0,0));
  EXPECT_TRUE(full.yellow_points(mask,bgr).empty());EXPECT_TRUE(full.white_points(mask,bgr).empty());
}
TEST(Mission, OutlineReframePreservesIslandWorldPoseAndIgnoresDuplicateFrame) {
  Params p;p.road_outline=true;p.fork_shape=true;p.lane_lidar_filter=false;
  ObjAvoidCore core(p);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
  core.island_mapper.shape.pose={3,0,0,0,3,true,true};
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
  EXPECT_TRUE(core.aligned);ASSERT_TRUE(core.island_mapper.shape.pose.confirmed);
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
TEST(Mission, ConfirmedYoloEventIsCachedAndSelectsRequestedLatticeSide) {
  for(bool right:{false,true}) {
    Params p;p.use_lane=false;p.fork_shape=true;p.fork_sign_min=1;
    p.boundary_s={-1,8};p.boundary_left={0.77,0.77};p.boundary_right={-0.77,-0.77};
    ObjAvoidCore core(p);core.on_sign(0.95,right);core.on_odom(1,0,0,0,0,0);ASSERT_TRUE(core.activate(1));
    EXPECT_EQ(core.fork_side(),right?-1:1);
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

TEST(Mission, LostPathRetracesTwentyCentimetersAndExcludesFailedCorridor) {
  for(double radius:{100000.,1.2}) {
    Params p;p.use_lane=false;p.reverse_wait=0.2;p.reverse_dist=0.20;p.reverse_max=1;
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
  Params p;p.use_lane=false;p.reverse_wait=0.1;ObjAvoidCore core(p);
  core.on_odom(0,0,0,0,0,0);ASSERT_TRUE(core.activate(0));
  for(int i=1;i<=60;++i) {
    const double t=i*0.05;core.on_odom(t,0,0,0,0,0);
    core.process_scan(t,std::vector<float>(720,8),-M_PI,2*M_PI/720,t);core.clear_path();
    double v=0,steer=0;core.control(t,0.05,v,steer);
    EXPECT_GE(v,0);EXPECT_NE(core.state,State::REVERSE);
  }
}

TEST(Mission, RetraceStopsWhenOdometryIsStale) {
  Params p;p.use_lane=false;p.reverse_wait=0.1;ObjAvoidCore core(p);
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
