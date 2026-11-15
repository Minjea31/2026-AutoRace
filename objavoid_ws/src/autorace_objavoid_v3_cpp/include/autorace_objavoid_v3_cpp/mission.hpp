// ROS 독립 미션 코어(Python mission.py): 인지 -> 재계획 정책 -> 경로 유지, 안전 정지, 후진 탈출, DONE 판정.
// 스레드: on_odom(센서), process_scan/on_lane(작업 스레드), control(제어 타이머). 모든 공개 함수는 mtx_로 보호.
#pragma once
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "autorace_objavoid_v3_cpp/controller.hpp"
#include "autorace_objavoid_v3_cpp/lane.hpp"
#include "autorace_objavoid_v3_cpp/params.hpp"
#include "autorace_objavoid_v3_cpp/perception.hpp"
#include "autorace_objavoid_v3_cpp/planner.hpp"
#include "autorace_objavoid_v3_cpp/road.hpp"

namespace objavoid {

enum class State {IDLE, RUN, STOP, DONE, REVERSE};
const char * state_name(State s);

struct Stats
{
  std::vector<double> v;
  void add(double x) {v.push_back(x);}
  std::string json() const;
};

struct Pose {double x, y, yaw, v, w;};
struct Veh {double s, d, psi, v;};

class ObjAvoidCore
{
public:
  explicit ObjAvoidCore(const Params & p);
  std::recursive_mutex & mutex() {return mtx_;}

  void on_odom(double t, double x, double y, double yaw, double v, double w);
  std::optional<Pose> pose_at(double t, double extrapolate = -1.0) const;
  // s_shift: 도로 원점을 현재 자세에서 전방으로 옮김(자동 시작 시 정지선 위치)
  bool activate(double t, double s_shift = 0.0);
  double start_shift = 0.0;
  void deactivate();
  bool on_lane(double stamp, const std::vector<XY> & yellow, const std::vector<XY> & view,
    const std::vector<XY> * white);
  bool process_scan(double stamp, const std::vector<float> & ranges, double angle_min, double angle_inc,
    double t_now);
  // -> 발행 안 함이면 false
  bool control(double t, double dt, double & v_cmd, double & steer_cmd);
  std::optional<Veh> vehicle(double t) const;
  // 갈림길 표지판(/yolo/LR_sign): right=false 왼쪽 A, true 오른쪽 B
  void on_sign(double t, bool right);
  // 섬 시작 s: 실측값(fork_island_s) > 표지판을 본 위치로 추정한 값 > 모름(inf)
  double island_s() const
  {
    if (p.fork_island_s > 0) {return p.fork_island_s;}
    if (const auto pb = sign_post()) {return pb->s0;}  // 표지판 받침(갈림길 섬 끝)
    return island_est_ ? *island_est_ : kInf;
  }
  // YOLO 표지판 박스(영상 가로 중심 u, 촬영 시각) -> 그 방향의 LiDAR 박스를 표지판 받침으로
  void on_sign_detail(double stamp, bool right, double u);
  std::optional<Box> sign_post() const;
  bool island_known() const {return std::isfinite(island_s());}
  Box island_box() const {return {island_s(), island_s() + p.fork_island_len, -p.fork_island_half, p.fork_island_half};}
  int fork_side() const {return fork_side_;}  // +1 왼쪽(A), -1 오른쪽(B), 0 미정
  int fork_votes_l = 0, fork_votes_r = 0;
  bool fork_locked = false;
  std::string status_json() const;
  void clear_path() {std::lock_guard<std::recursive_mutex> g(mtx_); path_.reset();}
  bool active() const {std::lock_guard<std::recursive_mutex> g(mtx_); return frame.has_value();}
  State get_state() const {std::lock_guard<std::recursive_mutex> g(mtx_); return state;}
  std::string get_done_reason() const {std::lock_guard<std::recursive_mutex> g(mtx_); return done_reason;}
  double scan_timeout() const {return p.scan_timeout;}

  // 상태 조회(마커·노드용, mutex 잡고 호출)
  Params p;
  State state = State::IDLE;
  std::string reason = "not_activated";
  std::optional<RoadFrame> frame;
  PathPtr path_;
  std::unique_ptr<Tracker> tracker;
  std::unique_ptr<VisibilityGrid> grid;
  std::unique_ptr<YellowMap> ymap, wmap;
  const Boundary * boundary() const;
  const LaneBoundary * lane_boundary() const {return lane_.get();}
  Planner planner;
  CtrlInfo last_info;
  double verified_s = kNaN;
  std::vector<Pt2> last_points;
  std::vector<Box> last_boxes;
  std::vector<XY> lane_cells, white_cells;
  bool has_lane_cells = false;
  int replans = 0;
  bool aligned = false;
  std::string done_reason;
  Stats scan_queue_ms;
  PathTracker ctrl, proj;

private:
  double wall() const;
  void reset_metrics();
  void set_state(State s, const std::string & r, double t);
  std::vector<XY> lidar_ground_filter(const std::vector<XY> & pts, const Pose & pose);
  std::optional<Veh> lane_update(Veh veh, double t);
  bool lane_blocking(double t) const;
  void reframe(double a, double dc, double t);
  bool emergency(const std::vector<float> & ranges, double angle_min, double angle_inc, double v) const;
  PlanStart start_state(const Veh & veh, std::optional<double> w, double & ey) const;
  void maybe_replan(const Veh & veh, double t);
  void update_clear_ahead(const Veh & veh);
  bool maybe_start_reverse(double t, const std::optional<Veh> & veh);
  std::pair<double, double> reverse_clear(double s, double d, double psi, const std::vector<Box> & boxes) const;
  void reverse_step(double t, double dt, const Veh & veh, double & v_cmd, double & steer_cmd);
  void end_reverse(double t, const std::string & why);
  void turn_diag(double t);
  bool check_done(const Veh & veh, double t);
  std::vector<Box> plan_boxes() const;
  void apply_fork(int side);

  mutable std::recursive_mutex mtx_;
  PriorBoundary prior_;
  std::unique_ptr<LaneBoundary> lane_;
  double section_end0_;
  std::deque<std::array<double, 6>> odom_;
  static constexpr size_t kOdomMax = 200;

  int path_version_ = -1;
  double last_plan_t_ = -1e9;
  std::optional<double> last_scan_t_;
  double emergency_t_ = -1e9;
  bool clear_ahead_ = false;
  std::optional<double> center_since_;
  int route_changes_ = 0, plan_failures_ = 0;
  std::map<std::string, int> plan_reasons_;
  std::vector<std::pair<double, std::string>> stop_events_;
  Stats plan_ms_, percep_ms_, scan_age_ms_, edges_, ey_, epsi_, lane_ms_;
  std::string last_plan_reason_;
  std::pair<double, double> cmd_{0, 0};
  std::string last_fail_json_ = "null";
  std::map<std::string, int> fail_checks_;
  int lane_frames_ = 0;
  long lane_lidar_dropped_ = 0;
  struct Rev {double t0, x0, y0; int sigma; double target;};
  std::optional<Rev> rev_;
  int rev_count_ = 0;
  std::optional<double> rev_anchor_s_;
  int reverses_ = 0;
  std::optional<double> stuck_since_;
  int lane_version_ = -1;
  int blocked_count_ = 0;
  std::optional<double> turn_ratio_;
  std::deque<std::pair<double, double>> cmd_hist_;
  int align_count_ = 0;
  double last_align_t_ = -1e9;
  std::string align_info_json_ = "null";
  std::optional<std::pair<double, double>> align_prev_;
  std::optional<double> island_;
  double activated_t_ = 0;
  struct OccScan {std::vector<double> r; double amin, ainc; LidarPose lp;};
  std::optional<OccScan> occ_scan_;
  int fork_side_ = 0;
  bool fork_replan_ = false;
  std::optional<double> wait_since_;
  double home_d0_ = 0.0, plan_beyond0_ = 0.0, roi_beyond0_ = 0.0;
  std::optional<double> island_est_;
  // 단순 갈림길(fork_turn): 결정 위치, 꺾기 시작 위치(odom)
  std::optional<std::pair<double, double>> turn_from_, turn_start_;
  int post_id_ = -1;
  std::optional<Box> post_box_;
  int post_align_ = -1;
  bool match_post(double stamp, double u);
  std::chrono::steady_clock::time_point t0_wall_ = std::chrono::steady_clock::now();
};

// JSON 보조
std::string jnum(double v, int prec = 3);
std::string jstr(const std::string & s);

}  // namespace objavoid
