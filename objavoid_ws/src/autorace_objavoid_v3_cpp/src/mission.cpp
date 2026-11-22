#include "autorace_objavoid_v3_cpp/mission.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <set>
#include <sstream>

namespace objavoid {

using Lock = std::lock_guard<std::recursive_mutex>;

const char * state_name(State s)
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::RUN: return "RUN";
    case State::STOP: return "STOP";
    case State::DONE: return "DONE";
    case State::REVERSE: return "REVERSE";
  }
  return "?";
}

std::string jnum(double v, int prec)
{
  if (!std::isfinite(v)) {return "null";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
  std::string s(buf);
  if (s.find('.') != std::string::npos) {
    while (!s.empty() && s.back() == '0') {s.pop_back();}
    if (!s.empty() && s.back() == '.') {s += '0';}
  }
  if (s == "-0.0") {s = "0.0";}
  return s;
}

std::string jstr(const std::string & s)
{
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') {o += '\\';}
    o += c;
  }
  return o + "\"";
}

std::string Stats::json() const
{
  if (v.empty()) {return "{\"n\":0}";}
  std::vector<double> a = v;
  std::sort(a.begin(), a.end());
  const double mean = std::accumulate(a.begin(), a.end(), 0.0) / a.size();
  // numpy percentile(linear)
  const double pos = 0.95 * (a.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(pos));
  const size_t hi = std::min(lo + 1, a.size() - 1);
  const double p95 = a[lo] + (a[hi] - a[lo]) * (pos - lo);
  return "{\"n\":" + std::to_string(a.size()) + ",\"mean\":" + jnum(mean) + ",\"p95\":" + jnum(p95) +
         ",\"max\":" + jnum(a.back()) + "}";
}

// ------------------------------------------------------------ 생성
static Params adjust(Params p)
{
  if (p.observe_mode) {  // 관찰 모드: 실차 한계로 계획
    p.plan_min_radius = p.observe_min_radius;
    p.max_heading = p.observe_max_heading;
    p.plan_min_radius_left = p.plan_min_radius_right = 0.0;
  }
  return p;
}

ObjAvoidCore::ObjAvoidCore(const Params & pin)
: p(adjust(pin)), planner(p, nullptr), ctrl(p), proj(p),
  prior_(p.boundary_s, p.boundary_left, p.boundary_right), section_end0_(p.section_end)
{
  if (p.use_lane) {lane_ = std::make_unique<LaneBoundary>(prior_, p);}
  road_boundary_=std::make_unique<IslandRoadBoundary>(boundary());
  planner.set_boundary(boundary());
  home_d0_ = p.home_d;
  plan_beyond0_ = p.plan_beyond_end;
  roi_beyond0_ = p.roi_beyond_end;
  reset_metrics();
}

const Boundary * ObjAvoidCore::boundary() const
{
  if(road_boundary_) return road_boundary_.get();
  return lane_ ? static_cast<const Boundary *>(lane_.get()) : static_cast<const Boundary *>(&prior_);
}

double ObjAvoidCore::wall() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_wall_).count();
}

void ObjAvoidCore::reset_metrics()
{
  startup_wait_=false;startup_stopped_at_.reset();
  path_.reset();planner.awaiting_island=p.fork_shape;
  path_version_ = -1;
  last_plan_t_ = -1e9;
  last_scan_t_.reset();
  emergency_t_ = -1e9;
  clear_ahead_ = false;
  center_since_.reset();
  replans = 0;
  route_changes_ = 0;
  plan_failures_ = 0;
  plan_reasons_.clear();
  stop_events_.clear();
  plan_ms_ = percep_ms_ = scan_age_ms_ = scan_queue_ms = edges_ = ey_ = epsi_ = lane_ms_ = Stats();
  last_info = CtrlInfo();
  last_plan_reason_.clear();
  cmd_ = {0, 0};
  done_reason.clear();
  verified_s = kNaN;
  last_fail_json_ = "null";
  fail_checks_.clear();
  lane_frames_ = 0;
  lane_lidar_dropped_ = 0;
  rev_.reset();
  driven_trail_.clear();last_route_.reset();last_route_points_.clear();
  rev_count_ = 0;
  rev_anchor_s_.reset();
  reverses_ = 0;
  stuck_since_.reset();
  lane_version_ = -1;
  blocked_count_ = 0;
  turn_ratio_.reset();
  cmd_hist_.clear();
  aligned = false;
  align_count_ = 0;
  last_align_t_ = -1e9;
  align_info_json_ = "null";
  align_prev_.reset();
  island_.reset();
  has_lane_cells = false;
}

void ObjAvoidCore::set_state(State s, const std::string & r, double t)
{
  if (s == State::STOP && r != "first_plan" && (state != State::STOP || reason != r)) {
    stop_events_.push_back({std::round(t * 100) / 100, r});
  }
  state = s;
  reason = r;
}

// ------------------------------------------------------------ 입력
void ObjAvoidCore::on_odom(double t, double x, double y, double yaw, double v, double w)
{
  Lock g(mtx_);
  // Delayed or rewound sensor packets never start a new mapping session.
  if (!odom_.empty() && t <= odom_.back()[0]) {return;}
  if(active() && !rev_ && std::isfinite(x) && std::isfinite(y) && v>=-0.01) {
    if(driven_trail_.empty()) driven_trail_.push_back({x,y});
    else {
      const auto & prev=driven_trail_.back();
      const double step=std::hypot(x-prev.x,y-prev.y);
      if(step>0.30) driven_trail_.clear(); // discontinuous odometry cannot define a safe reverse trace
      if(step>=0.005) driven_trail_.push_back({x,y});
    }
    while(driven_trail_.size()>2000) driven_trail_.pop_front();
  }
  odom_.push_back({t, x, y, yaw, v, w});
  while (odom_.size() > kOdomMax) {odom_.pop_front();}
}

std::optional<Pose> ObjAvoidCore::pose_at(double t, double extrapolate) const
{
  Lock g(mtx_);
  const double lim = extrapolate < 0 ? p.odom_extrapolate_max : extrapolate;
  if (odom_.empty()) {return std::nullopt;}
  const auto it = std::lower_bound(odom_.begin(), odom_.end(), t,
      [](const std::array<double, 6> & a, double tt) {return a[0] < tt;});
  const size_t k = static_cast<size_t>(it - odom_.begin());
  if (k == 0) {
    if (odom_[0][0] - t > 0.05) {return std::nullopt;}
    const auto & o = odom_[0];
    return Pose{o[1], o[2], o[3], o[4], o[5]};
  }
  if (k >= odom_.size()) {
    const auto & o = odom_.back();
    const double dt = t - o[0];
    if (dt > lim) {return std::nullopt;}
    const double yaw2 = o[3] + o[5] * dt;
    return Pose{o[1] + o[4] * dt * std::cos(o[3] + o[5] * dt / 2), o[2] + o[4] * dt * std::sin(o[3] + o[5] * dt / 2),
      yaw2, o[4], o[5]};
  }
  const auto & a = odom_[k - 1];
  const auto & b = odom_[k];
  const double u = (t - a[0]) / std::max(b[0] - a[0], 1e-9);
  const double dyaw = wrap(b[3] - a[3]);
  return Pose{a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2]), a[3] + u * dyaw, b[4], b[5]};
}

// ------------------------------------------------------------ 활성화
bool ObjAvoidCore::activate(double t, double s_shift, double yaw_offset, double d_shift, bool start_hold)
{
  Lock g(mtx_);
  if (frame && state != State::IDLE && state != State::DONE) {return false;}
  const auto pose = pose_at(t, 1.0);
  if (!pose) {return false;}
  const double road_yaw=pose->yaw+yaw_offset;
  const double c = std::cos(road_yaw), sn = std::sin(road_yaw);
  frame = RoadFrame(pose->x + c * s_shift-sn*d_shift, pose->y + sn * s_shift+c*d_shift, road_yaw);
  ++mapping_epoch;
  start_shift = s_shift;
  road_boundary_->reset();
  locked_fork_goal_odom_.reset();planner.fork_target.reset();fork_path_committed_=yellow_route_blocked_=obstacle_route_blocked_=false;
  tracker = std::make_unique<Tracker>(p);
  grid = std::make_unique<VisibilityGrid>(p, boundary()->s_min, boundary()->s_max);
  grid->mark_free(-1.0 - s_shift, p.body_front + 0.05 - s_shift, -0.5 * p.body_width - 0.05 - d_shift,
    0.5 * p.body_width + 0.05 - d_shift);
  ctrl.reset();
  reset_metrics();planner.last=PlanDiag();planner.has_ego_clear=false;planner.last_ego_clear={0,0};
  p.section_end = section_end0_;
  if (p.use_lane) {
    ymap = std::make_unique<YellowMap>(p, pose->x, pose->y);
    wmap = std::make_unique<YellowMap>(p, pose->x, pose->y);  // 흰 점선: 정렬·노란 노이즈 제거용
    lane_->rebuild({});
  } else {
    ymap.reset();
    wmap.reset();
  }
  activated_t_ = t;
  occ_scan_.reset();
  // 갈림길: 매 시도 새로(표지판은 코드 실행 중 교체·설치됨)
  island_sign_after=1e100;island_section_reached_at.reset();
  fork_side_ = 0;
  retained_island_.pose={};island_mapper.reset(); planner.island.pose.valid=false; planner.island_side=0;
  yellow_observed.clear();white_observed.clear();
  lane_cells.clear();white_cells.clear();last_boxes.clear();last_points.clear();fresh_view_odom.clear();
  yellow_history.clear();road_match_points.clear();island_match_points.clear();unmatched_points.clear();
  outline_fit={};outline_stable=0;outline_frame_at=-1e9;mapping_ms=Stats();road_boundary_->reset();

  searching=false;search_started=-1;search_distance=0;search_last.reset();entry_stable.reset();
  island_frame_at=-1e9;island_fit_stamp=-1e9;fresh_island_odom.clear();search_plan_at=-1e9;lane_command_at=-1e9;
  island_mapper.shape.length=p.fork_shape_length;island_mapper.shape.width=p.fork_shape_width;
  fork_votes_l = fork_votes_r = 0;
  fork_flip_streak = 0;
  fork_locked = false;
  fork_replan_ = false;
  wait_since_.reset();
  p.home_d = home_d0_;
  p.plan_beyond_end = plan_beyond0_;
  p.roi_beyond_end = roi_beyond0_;
  island_est_.reset();
  turn_from_.reset();
  turn_start_.reset();
  post_id_ = -1;
  post_box_.reset();
  if (p.fork_enable && p.fork_island_s <= 0) {
    // 섬 위치를 모름: 표지판을 볼 때까지 section_end에 묶이지 않고 도로 경계 끝까지 계획·장애물 추적
    const double far = boundary()->s_max - p.section_end;
    p.plan_beyond_end = std::max(p.plan_beyond_end, far);
    p.roi_beyond_end = std::max(p.roi_beyond_end, far);
  }
  state = State::STOP;
  reason = "first_plan";
  if(pending_sign_) {
    const auto cached=*pending_sign_;pending_sign_.reset();
    if(!p.fork_shape && t>=cached.first && t-cached.first<=p.fork_sign_cache_sec) on_sign(t,cached.second);
  }
  startup_wait_=start_hold;startup_stopped_at_.reset();
  if(start_hold) reason="startup_stopping";
  return true;
}

void ObjAvoidCore::deactivate()
{
  Lock g(mtx_);
  state = State::IDLE;
  reason = "not_activated";
  startup_wait_=false;startup_stopped_at_.reset();
  frame.reset();road_boundary_->reset();locked_fork_goal_odom_.reset();planner.fork_target.reset();fork_path_committed_=yellow_route_blocked_=obstacle_route_blocked_=false;
}

std::optional<Veh> ObjAvoidCore::vehicle(double t) const
{
  Lock g(mtx_);
  const auto pose = pose_at(t);
  if (!pose || !frame) {return std::nullopt;}
  Veh v;
  frame->pose_to_road(pose->x, pose->y, pose->yaw, v.s, v.d, v.psi);
  v.v = pose->v;
  return v;
}

bool ObjAvoidCore::on_lane(double stamp, const std::vector<XY> & yellow, const std::vector<XY> & view,
  const std::vector<XY> * white)
{
  Lock g(mtx_);
  if (stamp<activated_t_ || !ymap || !frame || state == State::IDLE || state == State::DONE) {return false;}
  const auto pose = pose_at(stamp);
  if (!pose) {return false;}
  if(stamp<=island_frame_at) return false;
  island_frame_at=stamp;
  const double w0 = wall();
  const std::vector<XY> yb = lidar_ground_filter(yellow, *pose);
  yellow_raw_count=yellow.size();yellow_filtered_count=yb.size();
  yellow_observed.clear();white_observed.clear();
  for(const auto & q:yellow) yellow_observed.push_back(YellowMap::to_odom(q,pose->x,pose->y,pose->yaw));
  if(white) for(const auto & q:*white) white_observed.push_back(YellowMap::to_odom(q,pose->x,pose->y,pose->yaw));
  if(p.fork_shape) {
    fresh_view_odom.clear();for(const auto & q:view) fresh_view_odom.push_back(YellowMap::to_odom(q,pose->x,pose->y,pose->yaw));
    fresh_island_odom.clear();fresh_island_odom.reserve(yb.size());
    for(const auto & q:yb) fresh_island_odom.push_back(YellowMap::to_odom(q,pose->x,pose->y,pose->yaw));
    yellow_history.emplace_back(stamp,fresh_island_odom);
    while(!yellow_history.empty() && yellow_history.front().first<stamp-0.5) yellow_history.pop_front();
    fresh_island_odom.clear();std::set<std::pair<int,int>> occupied;
    for(const auto & batch:yellow_history) for(const auto & q:batch.second)
      if(occupied.emplace(std::lround(q.x/0.02),std::lround(q.y/0.02)).second) fresh_island_odom.push_back(q);

  }
  ymap->update(yb, view, pose->x, pose->y, pose->yaw);
  if (white != nullptr) {
    const std::vector<XY> wb = lidar_ground_filter(*white, *pose);
    wmap->update(wb, view, pose->x, pose->y, pose->yaw);
  }
  ++lane_frames_;
  lane_ms_.add((wall() - w0) * 1000);
  return true;
}

std::vector<XY> ObjAvoidCore::lidar_ground_filter(const std::vector<XY> & pts, const Pose & pose)
{
  // 바닥 투영점이 같은 방향 LiDAR 거리보다 멀면 서 있는 물체(노란·연두 라바콘)로 보고 버림
  if (!p.lane_lidar_filter || !occ_scan_ || pts.empty()) {return pts;}
  const OccScan & sc = *occ_scan_;
  const long n = static_cast<long>(sc.r.size());
  std::vector<XY> out;
  for (const XY & q : pts) {
    const XY o = YellowMap::to_odom(q, pose.x, pose.y, pose.yaw);
    const double dx = o.x - sc.lp.x, dy = o.y - sc.lp.y;
    const double rng = std::hypot(dx, dy);
    // 빔 인덱스 = ((방향 - lyaw - amin + pi) mod 2pi - pi) / ainc
    double a = std::atan2(dy, dx) - sc.lp.yaw - sc.amin + M_PI;
    a = std::fmod(a, 2 * M_PI);
    if (a < 0) {a += 2 * M_PI;}
    a -= M_PI;
    const long idx = std::lround(a / sc.ainc);
    double nearest = kInf;
    for (long o2 = -p.lane_lidar_beams; o2 <= p.lane_lidar_beams; ++o2) {
      const long j = idx + o2;
      if (j >= 0 && j < n) {nearest = std::min(nearest, sc.r[j]);}
    }
    if (rng < nearest - p.lane_lidar_margin) {out.push_back(q);} else {++lane_lidar_dropped_;}
  }
  return out;
}

RoadOutline ObjAvoidCore::road_outline() const
{
  return road_boundary_->model ? *road_boundary_->model : RoadOutline(p);
}

std::optional<Veh> ObjAvoidCore::lane_update(Veh veh, double t)
{
  if (!ymap || (ymap->version + wmap->version == lane_version_ && island_frame_at<=island_fit_stamp)) {return veh;}
  const double mapping_start=wall();
  lane_version_ = ymap->version + wmap->version;
  std::vector<XY> cells = ymap->confirmed();
  const std::vector<XY> wcells = wmap->confirmed();
  if (!cells.empty() && !wcells.empty()) {
    // 흰 점선 근처(lane_white_exclusion)에 잡힌 노란 칸은 노이즈로 버림(np.roll과 같이 격자 가장자리는 순환)
    const int r = static_cast<int>(std::ceil(p.lane_white_exclusion / wmap->res));
    const int n = wmap->n;
    std::vector<XY> keep;
    for (const XY & c : cells) {
      const int ii = ymap->idx_i(c.x), jj = ymap->idx_j(c.y);
      bool near = false;
      for (int di = -r; di <= r && !near; ++di) {
        for (int dj = -r; dj <= r && !near; ++dj) {
          const int a = ((ii - di) % n + n) % n, b = ((jj - dj) % n + n) % n;
          near = wmap->hit_confirmed(a, b);
        }
      }
      if (!near) {keep.push_back(c);}
    }
    cells.swap(keep);
  }
  white_cells = wcells;
  lane_cells = cells;
  has_lane_cells = true;
  std::vector<Pt2> sd, wsd;
  for (const XY & c : cells) {Pt2 q; frame->to_road(c.x, c.y, q.s, q.d); sd.push_back(q);}
  for (const XY & c : wcells) {Pt2 q; frame->to_road(c.x, c.y, q.s, q.d); wsd.push_back(q);}
  // Island observations are fitted independently. They never translate or
  // rotate the ego/observation coordinate system after island confirmation.
  if (p.fork_shape && !p.road_outline) {
    std::vector<Pt2> outer;
    for (const auto & q : sd) {
      double right, left;
      prior_.limits(q.s, right, left);
      if (std::min(std::abs(q.d-right), std::abs(q.d-left)) < 0.20)
        outer.push_back(q);
    }
    sd.swap(outer);
  }
  if(p.road_outline && island_frame_at>outline_frame_at) {
    outline_frame_at=island_frame_at;
    if(!navigation_island().pose.confirmed) {
      RoadOutline outline(p);std::vector<XY> observed;
      for(const auto & q:fresh_island_odom) {double rs,rd;frame->to_road(q.x,q.y,rs,rd);observed.push_back({rs,rd});}
      int fresh_outer=0;
      if(!yellow_history.empty()) for(const auto & q:yellow_history.back().second) {
        double rs,rd;frame->to_road(q.x,q.y,rs,rd);
        if(!outline.island_region({rs,rd}) && outline.distance({rs,rd})<0.20) ++fresh_outer;
      }
      const auto previous=outline_fit;
      outline_fit=outline.fit(observed,!aligned,XY{veh.s,veh.d});
      // A legacy start may not be anchored at the entrance. Bootstrap heading/centre
      // from the outer straight pair, never from central island points.
      if(!outline_fit.valid && !aligned) {
        std::vector<Pt2> exterior;
        for(const auto & q:observed) if(std::abs(q.y)>0.48) exterior.push_back({q.x,q.y});
        const std::vector<double> widths{p.fork_road_width};
        Params bootstrap_params=p;bootstrap_params.lane_max_d=1.5;
        const auto bootstrap=fit_road(exterior,bootstrap_params,veh.s-0.5,veh.s+2.2,nullptr,&widths,wsd);
        if(bootstrap && bootstrap->center) {
          outline_fit={0,*bootstrap->center,bootstrap->a,kInf,static_cast<int>(exterior.size()),true,false};
        }
      }
      if(fresh_outer<6 && outline_fit.valid) {
        // Permit a displaced outer pair to initialize, but never confirm from an empty frame.
        if(yellow_history.empty() || yellow_history.back().second.size()<12) outline_fit.valid=false;
      }
      const bool consistent=previous.valid && outline_fit.valid &&
        std::abs(previous.d-outline_fit.d)<0.06 && std::abs(previous.yaw-outline_fit.yaw)<0.06 &&
        (!outline_fit.longitudinal || std::abs(previous.s-outline_fit.s)<0.10);
      outline_stable=outline_fit.valid?(consistent?outline_stable+1:1):0;
      if(outline_fit.valid && outline_stable>=2 && t-last_align_t_>=0.2) {
        const double a=outline_fit.yaw,dc=outline_fit.d,ds=outline_fit.longitudinal?outline_fit.s:0;
        if(!aligned || std::abs(a)>0.015 || std::abs(dc)>0.02 || std::abs(ds)>0.03) {
          reframe(a,dc,t,ds);last_align_t_=t;outline_stable=0;
          align_info_json_="{\"source\":\"road_outline\",\"ds\":"+jnum(ds)+",\"dcenter\":"+jnum(dc)+",\"dpsi\":"+jnum(a)+"}";
          if(auto v2=vehicle(t)) veh=*v2;
        }
        aligned=true;
      }
    }
    classify_outline_points(sd);
  } else if(p.road_outline) {
    sd.clear();for(const auto & q:road_match_points) sd.push_back({q.x,q.y});
  }
  // 재정렬은 구간 입구에서만(활성화 후 lane_align_s_max 이내). 처음 정렬은 위치와 무관하게 시도.
  const bool allow = !aligned || veh.s < p.lane_align_s_max;
  if (!p.road_outline && p.lane_align && allow && t - last_align_t_ >= p.lane_align_period && !sd.empty()) {
    const double near_end = std::min(veh.s + 2.2, p.section_end - 0.1);
    std::optional<RoadFit> fit;
    if (!aligned) {
      fit = fit_road(sd, p, veh.s - 0.5, near_end, &prior_, nullptr, wsd);
    } else {
      const std::vector<double> widths{p.lane_pair_widths.back()};
      fit = fit_road(sd, p, veh.s - 0.5, near_end, nullptr, &widths, wsd);
    }
    const bool waited = t - activated_t_ >= p.lane_align_wait;
    if (fit) {
      const double a = fit->a;
      std::optional<double> center = fit->center;
      if (!center && (aligned || waited)) {center = 0.0;}  // 방향만 보정
      const double dc = center ? *center : 0.0;
      const bool big = std::abs(a) > p.lane_align_tol || std::abs(dc) > p.lane_center_tol;
      const auto prev = align_prev_;
      align_prev_ = std::make_pair(a, dc);
      const bool consistent = prev && std::abs(prev->first - a) < 0.02 && std::abs(prev->second - dc) < 0.03;
      if (center && ((big && consistent) || !aligned)) {
        reframe(a, dc, t);
        last_align_t_ = t + (p.lane_realign_period - p.lane_align_period);
        align_prev_.reset();
        align_info_json_ = "{\"t\":" + jnum(t, 2) + ",\"dpsi\":" + jnum(a) + ",\"dcenter\":" + jnum(dc) +
          ",\"pair\":true}";
        sd.clear();
        for (const XY & c : cells) {
          Pt2 q; frame->to_road(c.x, c.y, q.s, q.d);
          double right, left; prior_.limits(q.s, right, left);
          if (!p.fork_shape || std::min(std::abs(q.d-right),std::abs(q.d-left))<0.20)
            sd.push_back(q);
        }
        if (auto v2 = vehicle(t)) {veh = *v2;}
      }
      if (center) {aligned = true;}
    }
  }
  // 갈림길 섬 끝: 가운데 근처(|d|<lane_island_d)에 나타나는 노란선 -> 구간 끝
  const double lo = section_end0_ - p.lane_island_window, hi = section_end0_ + p.lane_island_window;
  if (!p.fork_shape && !sd.empty() && aligned && p.lane_island) {
    std::vector<double> ss;
    for (const Pt2 & q : sd) {
      if (std::abs(q.d) < p.lane_island_d && q.s > std::max(lo, veh.s + 0.3) && q.s < hi) {ss.push_back(q.s);}
    }
    if (ss.size() >= 3) {
      std::sort(ss.begin(), ss.end());
      for (size_t i = 0; i + 2 < ss.size(); ++i) {
        if (ss[i + 2] - ss[i] < 0.15) {
          island_ = island_ ? std::min(*island_, ss[i]) : ss[i];
          p.section_end = std::clamp(*island_ - 0.02, lo, hi);
          break;
        }
      }
    }
  }
  // Start island fitting when the front of the measured vehicle reaches the
  // purple section_end line. Latch the crossing through stops and reversing.
  const double front_s=veh.s+std::max(p.body_front*std::cos(veh.psi),-p.body_rear*std::cos(veh.psi))+
    0.5*p.body_width*std::abs(std::sin(veh.psi));
  if(p.fork_shape && !island_section_reached_at && front_s>=p.section_end)
    island_section_reached_at=t;
  if(p.fork_shape && !island_section_reached_at) island_mapper.rejection="waiting_section_line";
  else if(p.fork_shape && !fork_side_) island_mapper.rejection="waiting_lr_sign";
  if(p.fork_shape && island_section_reached_at && fork_side_ &&
      island_frame_at>island_sign_after && island_frame_at>island_fit_stamp) {
    const double previous_fit_stamp=island_fit_stamp;island_fit_stamp=island_frame_at;
    std::vector<XY> points;points.reserve(cells.size()+fresh_island_odom.size());
    for(const auto & q:cells) {double rs,rd;frame->to_road(q.x,q.y,rs,rd);points.push_back({rs,rd});}
    // Keep the existing observation history, including images before arrival.
    for(const auto & batch:yellow_history) if(batch.first>island_sign_after)
      for(const auto & q:batch.second) {double s,d;frame->to_road(q.x,q.y,s,d);points.push_back({s,d});}
    const auto previous=planner.island.pose;
    if(p.road_outline) {const auto outline=road_outline();points.erase(std::remove_if(points.begin(),points.end(),[&](const XY & q){return !outline.island_region(q);}),points.end());}
    int fresh_central=0;
    if(!yellow_history.empty()) for(const auto & q:yellow_history.back().second) {
      double rs,rd;frame->to_road(q.x,q.y,rs,rd);
      if(std::abs(rd)<0.48 && (!p.road_outline || (rs>0.5*(p.road_outline_length-p.road_outline_straight) && rs<0.5*(p.road_outline_length+p.road_outline_straight)))) ++fresh_central;
    }
    if(fresh_central<6) points.clear();
    // A confirmed static island remains in odom when its front corner leaves the camera.
    // Contradictions in the observed front region still go through the normal rejection path.
    const auto visible_model=navigation_island();
    const auto & known=visible_model.pose;
    bool front_visible=true;
    if(known.confirmed) {
      double wx,wy;frame->to_odom(known.s,known.d,wx,wy);
      front_visible=std::any_of(fresh_view_odom.begin(),fresh_view_odom.end(),[&](const XY & q) {
        return std::hypot(q.x-wx,q.y-wy)<0.08;
      });
    }
    const bool matched=front_visible && island_mapper.update(points,section_end0_,&p,veh.s,veh.d,veh.psi);
    // A fitted accumulated map may already have three independent supporting
    // camera frames at arrival. Verify those frames without refitting duplicates.
    if(matched && !island_mapper.shape.pose.confirmed) {
      const auto polygon=island_mapper.shape.vertices();int extra=0;
      for(auto it=yellow_history.rbegin();it!=yellow_history.rend() && extra<2;++it) {
        if(it->first>=island_frame_at || it->first<=previous_fit_stamp || it->first<=island_sign_after) continue;
        int central=0,hits=0;
        for(const auto & world:it->second) {
          XY q;frame->to_road(world.x,world.y,q.x,q.y);
          if(std::abs(q.y)>=.48) continue;
          ++central;double best=kInf;
          for(size_t k=0;k<polygon.size();++k) {
            const auto a=polygon[k],b=polygon[(k+1)%polygon.size()];
            const double dx=b.x-a.x,dy=b.y-a.y;
            const double u=std::clamp(((q.x-a.x)*dx+(q.y-a.y)*dy)/(dx*dx+dy*dy),0.,1.);
            best=std::min(best,std::hypot(q.x-a.x-u*dx,q.y-a.y-u*dy));
          }
          hits+=best<.05;
        }
        if(hits>=6 && hits>=.5*central) ++extra;
      }
      island_mapper.shape.pose.stable+=extra;
      island_mapper.shape.pose.confirmed=island_mapper.shape.pose.stable>=3;
    }
    if(island_mapper.shape.pose.confirmed) retained_island_=island_mapper.shape;
    road_boundary_->update(p,navigation_island());
    planner.island=navigation_island();
    if(matched) {
      planner.island=navigation_island(); planner.island_side=fork_side_;
      if(fork_side_ && planner.island.pose.confirmed && (!previous.confirmed ||
        std::hypot(previous.s-planner.island.pose.s,previous.d-planner.island.pose.d)>0.03) && !fork_path_committed_) apply_fork(fork_side_);
    }
  }
  // 구간 끝 너머와 입구 이후 가운데 근처(섬) 노란선은 이 미션의 외곽 경계에서 제외
  std::vector<Pt2> keep;
  for (const Pt2 & q : sd) {
    bool island = q.s > 1.0 && std::abs(q.d) < p.lane_island_d;
    if(p.fork_shape) {
      if(navigation_island().pose.valid) {
        const auto local=navigation_island().local(q.s,q.d);
        island=local.x>-0.08 && local.x<island_mapper.shape.length+0.08 &&
          std::abs(local.y)<island_mapper.shape.width/2+0.08;
      } else {
        island=q.s>section_end0_-0.8 && std::abs(q.d)<p.lane_island_d;
      }
    }
    if ((p.fork_shape || q.s < p.section_end) && !island) {keep.push_back(q);}
  }
  lane_->rebuild(keep);
  mapping_ms.add((wall()-mapping_start)*1000);
  return veh;
}

void ObjAvoidCore::classify_outline_points(std::vector<Pt2> & sd)
{
  const auto outline=road_outline();
  road_match_points.clear();island_match_points.clear();unmatched_points.clear();sd.clear();
  for(const auto & q:fresh_island_odom) {
    double rs,rd;frame->to_road(q.x,q.y,rs,rd);const XY local{rs,rd};
    if(outline.island_region(local)) island_match_points.push_back(local);
    else if(outline.distance(local)<0.12) {road_match_points.push_back(local);sd.push_back({rs,rd});}
    else unmatched_points.push_back(local);
  }
}

static bool crosses_yellow(const Veh & v,const Boundary & boundary,const IslandShape & island,const Params & p)
{
  return boundary_clearance(v.s,v.d,v.psi,boundary,p)<0 ||
    (p.fork_shape && island.pose.valid && island.clearance(v.s,v.d,v.psi,p)<0);
}
static bool path_inside_yellow(const Path & path,const Boundary & boundary,const IslandShape & island,const Params & p,double from)
{
  for(size_t i=0;i<path.s.size();++i) if(path.s[i]>=from) {
    if(boundary_clearance(path.s[i],path.d[i],path.psi[i],boundary,p)<-1e-6) return false;
    if(p.fork_shape && island.pose.valid && island.clearance(path.s[i],path.d[i],path.psi[i],p)<-1e-6) return false;
  }
  return true;
}

bool ObjAvoidCore::lane_blocking(double t) const
{
  // The outline is an online correction to an already usable saved map.
  // Lack of camera registration must not gate either planning or control.
  if (p.road_outline || !p.lane_align || !ymap || aligned) {return false;}
  return t - activated_t_ < p.lane_align_wait;
}

void ObjAvoidCore::reframe(double a, double dc, double t, double ds)
{
  (void)t;
    const RoadFrame old = *frame;
  const double yaw = old.yaw0 + a;
  const RoadFrame nf(old.x0+old.c*ds-old.s*dc,old.y0+old.s*ds+old.c*dc,yaw);
  if (std::abs(a) > p.lane_reset_tracks_angle) {
    tracker->tracks.clear();  // 큰 재정렬: 박스를 버리고 새 좌표계에서 다시 검출
    ++tracker->version;
  }
  for (Track & tr : tracker->tracks) {
    const Box & b = tr.box;
    double s0 = kInf, s1 = -kInf, d0 = kInf, d1 = -kInf;
    const double cs[4] = {b.s0, b.s1, b.s1, b.s0}, cd[4] = {b.d0, b.d0, b.d1, b.d1};
    for (int k = 0; k < 4; ++k) {
      double x, y, ns, nd;
      old.to_odom(cs[k], cd[k], x, y);
      nf.to_road(x, y, ns, nd);
      s0 = std::min(s0, ns); s1 = std::max(s1, ns); d0 = std::min(d0, nd); d1 = std::max(d1, nd);
    }
    tr.box = {s0, s1, d0, d1, b.robot_candidate};
  }
  auto point=[&](double & s,double & d) {
    double x,y;old.to_odom(s,d,x,y);nf.to_road(x,y,s,d);
  };
  for(auto & q:last_points) point(q.s,q.d);
  for(auto * points:{&road_match_points,&island_match_points,&unmatched_points,&failed_routes_})
    for(auto & q:*points) point(q.x,q.y);
  for(auto & b:last_boxes) {
    double s0=kInf,s1=-kInf,d0=kInf,d1=-kInf;
    for(auto q:std::array<XY,4>{{{b.s0,b.d0},{b.s1,b.d0},{b.s1,b.d1},{b.s0,b.d1}}}) {
      point(q.x,q.y);s0=std::min(s0,q.x);s1=std::max(s1,q.x);
      d0=std::min(d0,q.y);d1=std::max(d1,q.y);
    }
    b={s0,s1,d0,d1,b.robot_candidate};
  }
  *grid = grid->reframed(old, nf);
  frame = nf;
  planner.has_ego_clear=false;
  auto & ip=island_mapper.shape.pose;
  if(ip.valid) {
    double x,y;old.to_odom(ip.s,ip.d,x,y);nf.to_road(x,y,ip.s,ip.d);ip.yaw=wrap(ip.yaw-a);
  }
  if(retained_island_.pose.valid) {
    auto & cached=retained_island_.pose;double x,y;old.to_odom(cached.s,cached.d,x,y);
    nf.to_road(x,y,cached.s,cached.d);cached.yaw=wrap(cached.yaw-a);
  }
  planner.island=navigation_island();search_plan_at=-1e9;

  path_.reset();
  ++align_count_;
  last_plan_t_ = -1e9;
}

IslandShape ObjAvoidCore::navigation_island() const
{
  Lock lock(mtx_);
  if(island_mapper.shape.pose.confirmed) return island_mapper.shape;
  if(retained_island_.pose.valid) return retained_island_;
  return island_mapper.shape;
}
// ------------------------------------------------------------ 작업 스레드
bool ObjAvoidCore::process_scan(double stamp, const std::vector<float> & ranges, double angle_min,
  double angle_inc, double t_now)
{
  Lock g(mtx_);
  if (stamp<activated_t_ || !frame || state == State::IDLE || state == State::DONE) {return false;}
  const double w0 = wall();
  scan_age_ms_.add((t_now - stamp) * 1000);
  const auto pose = pose_at(stamp);
  if (!pose) {return false;}
  const double c = std::cos(pose->yaw), sn = std::sin(pose->yaw);
  const LidarPose lp{pose->x + c * p.lidar_x - sn * p.lidar_y, pose->y + sn * p.lidar_x + c * p.lidar_y,
    pose->yaw + p.lidar_yaw};
  double sv, dv, psiv;
  frame->pose_to_road(pose->x, pose->y, pose->yaw, sv, dv, psiv);
  // 가장 뒤 모서리: 지난 장애물을 이보다 먼저 지우지 않음
  double s_rear = sv - p.body_rear * std::abs(std::cos(psiv)) - 0.5 * p.body_width * std::abs(std::sin(psiv));
  std::vector<Box> boxes;
  std::vector<Pt2> pts;
  detect(ranges, angle_min, angle_inc, lp, *frame, *boundary(), sv, p, boxes, pts);
  Beam beam{*frame, lp, angle_min, angle_inc, last_scan_t_ ? std::max(0.0, stamp - *last_scan_t_) : 0.1};
  tracker->update(boxes, stamp, s_rear, &ranges, &beam);
  if (post_id_ >= 0) {
    for (const Track & tr : tracker->tracks) {
      if (tr.id == post_id_) {post_box_ = tr.box; post_align_ = align_count_;}
    }
  }
  grid->update(ranges, angle_min, angle_inc, lp, *frame);
  if (emergency(ranges, angle_min, angle_inc, pose->v)) {emergency_t_ = stamp;}
  last_scan_t_ = stamp;
  last_points = pts;
  last_boxes = boxes;
  OccScan oc;
  oc.r.resize(ranges.size());
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    oc.r[i] = (std::isfinite(r) && r > 0.02) ? r : kInf;
  }
  oc.amin = angle_min;
  oc.ainc = angle_inc;
  oc.lp = lp;
  occ_scan_ = std::move(oc);
  percep_ms_.add((wall() - w0) * 1000);

  auto veh = vehicle(t_now);
  if (!veh) {return false;}
  veh = lane_update(*veh, t_now);
  if (lane_blocking(t_now)) {
    path_.reset();reason="lane_align";
    return true;
  }  // Legacy lane alignment only; outline matching uses the saved map meanwhile.
  if (rev_) {return true;}                   // 후진 중에는 인지만 갱신
  if(island_mapper.shape.pose.confirmed) retained_island_=island_mapper.shape;
  const auto mapped_island=navigation_island();
  planner.island=mapped_island;planner.island_side=fork_side_;
  road_boundary_->update(p,mapped_island);
  if(p.fork_shape && mapped_island.pose.confirmed && fork_side_ && island_section_reached_at) {
    if(!locked_fork_goal_odom_) if(const auto goal=fork_goal()) {
      XY q;frame->to_odom(goal->x,goal->y,q.x,q.y);locked_fork_goal_odom_=q;
      locked_fork_yaw_odom_=frame->yaw0+mapped_island.pose.yaw;fork_locked=true;
    }
    if(locked_fork_goal_odom_) {
      XY goal;frame->to_road(locked_fork_goal_odom_->x,locked_fork_goal_odom_->y,goal.x,goal.y);
      planner.fork_target=goal;planner.fork_target_yaw=wrap(locked_fork_yaw_odom_-frame->yaw0);
    }
  }
  searching=false;search_started=-1;search_distance=0;search_last.reset();
  planner.awaiting_island=p.fork_shape && !locked_fork_goal_odom_;
  if(planner.awaiting_island) {
    planner.island_side=0;
    if(island_section_reached_at) {
      path_.reset();planner.last=PlanDiag();last_info=CtrlInfo();verified_s=kNaN;
      set_state(State::STOP,fork_side_?"waiting_island_confirmation":"waiting_lr_sign",t_now);
      return true;
    }
  }
  maybe_replan(*veh,t_now);

  if (path_) {
    verified_s = planner.verified_until(*path_, veh->s, *grid, ymap ? boundary() : nullptr);
  }
  update_clear_ahead(*veh);
  return true;
}

bool ObjAvoidCore::emergency(const std::vector<float> & ranges, double angle_min, double angle_inc, double v) const
{
  const double reach = p.body_front + p.emergency_distance + std::max(v, 0.0) * p.emergency_time;
  int hits = 0;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r <= p.scan_range_min) {continue;}
    const double th = angle_min + i * angle_inc + p.lidar_yaw;
    const double x = p.lidar_x + r * std::cos(th), y = p.lidar_y + r * std::sin(th);
    if (x > p.body_front - 0.02 && x < reach && std::abs(y) < 0.5 * p.body_width + 0.02) {++hits;}
  }
  return hits >= p.emergency_points;
}

PlanStart ObjAvoidCore::start_state(const Veh & veh, std::optional<double> w, double & ey) const
{
  ey = 0.0;
  const auto island=navigation_island();
  const bool over_paint=crosses_yellow(veh,*boundary(),island,p);
  if (path_) {
    // proj는 작업 스레드 전용 투영 상태
    ey = const_cast<PathTracker &>(proj).errors(*path_, veh.s, veh.d, veh.psi).ey;
    if ((std::abs(ey)<p.stitch_error || over_paint) && path_->s.front()<=veh.s && veh.s<=path_->end_s()-.05 &&
        path_inside_yellow(*path_,*boundary(),island,p,veh.s-p.body_rear)) {
      double d0, v0, a0;
      path_->state_at(veh.s, d0, v0, a0);
      return {veh.s, d0, v0, a0};
    }
  }
  if(over_paint) {
    std::vector<std::pair<double,double>> free,gaps;
    planner.free_intervals(veh.s,obstacle_array(plan_boxes()),free,gaps);
    double best=kInf,inside=veh.d;
    for(const auto & interval:free) {
      const double candidate=std::clamp(veh.d,interval.first,interval.second);
      if(std::abs(candidate-veh.d)<best) {best=std::abs(candidate-veh.d);inside=candidate;}
    }
    if(std::isfinite(best)) return {veh.s,inside,0,0};
  }
  const double v0 = std::tan(veh.psi);
  // 시작 곡률: 실제 운동(odom 각속도/속도). 정지 중이면 마지막 명령 조향(RUN일 때)만, 아니면 0.
  double k = 0.0;
  if (w && std::abs(veh.v) > 0.05) {
    k = *w / veh.v;
  } else if (state == State::RUN) {
    k = std::tan(ctrl.delta) / p.wheelbase;
  }
  const auto [rl, rr] = plan_radius(p);
  k = std::clamp(k, -1.0 / rr, 1.0 / rl);
  return {veh.s, veh.d, v0, k * std::pow(1 + v0 * v0, 1.5)};
}

static PathPtr splice_path(const Path & old, double s_c, const Path & tail)
{
  // old 경로의 s_c까지 + tail(s_c에서 시작)
  const size_t k = static_cast<size_t>(std::lower_bound(old.s.begin(), old.s.end(), s_c) - old.s.begin());
  const double prev_s = k > 0 ? old.s[k - 1] : -1e9;
  const size_t j = (tail.s.size() > 1 && std::abs(tail.s[0] - prev_s) < 1e-6) ? 1 : 0;
  auto out = std::make_shared<Path>();
  auto cat = [&](std::vector<double> & dst, const std::vector<double> & a, const std::vector<double> & b) {
      if (a.empty() || b.empty()) {return;}
      dst.assign(a.begin(), a.begin() + k);
      dst.insert(dst.end(), b.begin() + j, b.end());
    };
  cat(out->s, old.s, tail.s); cat(out->d, old.d, tail.d); cat(out->d1, old.d1, tail.d1);
  cat(out->d2, old.d2, tail.d2); cat(out->psi, old.psi, tail.psi); cat(out->kappa, old.kappa, tail.kappa);
  cat(out->clear, old.clear, tail.clear); cat(out->dsteer, old.dsteer, tail.dsteer);
  cat(out->bclear, old.bclear, tail.bclear);
  out->cost = tail.cost;
  out->complete = tail.complete;
  out->nodes = tail.nodes;
  out->stamp = tail.stamp;
  return out;
}

void ObjAvoidCore::maybe_replan(const Veh & veh, double t)
{
  const double s = veh.s;
  const std::vector<Box> boxes = plan_boxes();
  if(path_ && crosses_yellow(veh,*boundary(),navigation_island(),p) &&
      path_inside_yellow(*path_,*boundary(),navigation_island(),p,s-p.body_rear)) {
    const auto [ob,bd]=planner.path_clearance(*path_,s-p.body_rear,boxes);
    if(ob<=p.blocked_tol && bd<=p.blocked_tol && path_->end_s()>s+std::max(.4,p.stop_buffer+.2)) {
      yellow_route_blocked_=obstacle_route_blocked_=false;blocked_count_=0;return;
    }
  }
  if(p.fork_shape && fork_path_committed_ && path_) {
    const auto [ob,bd]=planner.path_clearance(*path_,s-p.body_rear,boxes);
    bool object=false;
    const auto actual_objects=obstacle_array(tracker->confirmed_boxes());
    for(size_t q=0;q<path_->s.size();++q) if(path_->s[q]>=s-p.body_rear) {
      const double required=path_->clear.empty()?p.margin_obstacle:std::min(p.margin_obstacle,path_->clear[q]);
      if(required-body_clearance(path_->s[q],path_->d[q],path_->psi[q],actual_objects,p)>p.blocked_tol) {
        object=true;break;
      }
    }
    const bool yellow=bd>p.blocked_tol || (ob>p.blocked_tol && !object);
    blocked_count_=(object||yellow)?blocked_count_+1:0;
    obstacle_route_blocked_=object && blocked_count_>=p.blocked_scans;
    yellow_route_blocked_=yellow && !object && blocked_count_>=p.blocked_scans;
    fork_replan_=false;
    return; // Keep the selected path. A blocked path stops before recovery.
  }
  const double s_lim = planner.plan_limit(boxes) - 0.05;
  std::string why;
  bool hard = false;
  if (!path_) {
    why = "no_path";
    hard = true;
  } else if (fork_replan_) {
    why = "fork";  // 갈림길 방향이 정해짐: 목표 차로·계획 범위가 바뀌었으니 기존 경로에 매이지 않고 새로
    hard = true;
  } else {
    const double ey = proj.errors(*path_, veh.s, veh.d, veh.psi).ey;
    const auto [c_obs, c_b] = planner.path_clearance(*path_, s - p.body_rear, boxes);
    const bool hit = c_obs > p.blocked_tol || c_b > p.blocked_tol;
    blocked_count_ = hit ? blocked_count_ + 1 : 0;
    if (blocked_count_ >= p.blocked_scans) {
      why = "blocked"; hard = true;
    } else if (std::abs(ey) > p.deviation_replan) {
      why = "deviation"; hard = true;
    } else if (path_->end_s() - s < p.replan_remaining && path_->end_s() < s_lim - 0.05 &&
      t - last_plan_t_ >= (path_->complete ? 0.3 : 0.5))
    {
      why = "extend";
    } else if (t - last_plan_t_ >= 0.3 && tracker->version != path_version_) {
      why = "new_obstacle";
    } else if (!path_->complete && t - last_plan_t_ >= 0.5) {
      why = "incomplete";
    } else if (std::isfinite(verified_s) && verified_s - s < 1.0 && path_->end_s() - verified_s > 0.1 &&
      t - last_plan_t_ >= p.replan_period)
    {
      why = "unverified";
    }
  }
  if (why.empty()) {return;}
  fork_replan_ = false;
  const auto pose = pose_at(t);
  double ey0;
  const PlanStart start = start_state(veh, pose ? std::optional<double>(pose->w) : std::nullopt, ey0);
  const double w0 = wall();
  PathPtr nw;
  const PathPtr old = path_;
  const double s_c = s + p.commit_len;
  if (!hard && p.commit_len > 0 && old && std::abs(ey0) < p.stitch_error && old->s.front() <= s &&
    old->end_s() >= s_c + 0.1)
  {
    // 차 앞 commit_len은 기존 경로 그대로, 그 끝 상태에서 새로 계획해 이어 붙임
    double dc, d1c, d2c;
    old->state_at(s_c, dc, d1c, d2c);
    PathPtr tail = planner.plan({s_c, dc, d1c, d2c}, boxes, grid.get(), old.get(), -1.0, t);
    if (tail) {nw = splice_path(*old, s_c, *tail);}
  }
  if (!nw) {
    nw = planner.plan(start, boxes, grid.get(), hard ? nullptr : path_.get(), hard ? 0.0 : -1.0, t);
  }
  if (!nw && (hard || std::abs(start.a) > 1.0 || std::abs(start.v) > 0.8)) {
    // 차 자세가 계획 한계 밖: 시작 곡률 0, 운동학 완화 구간을 늘려 재시도
    const double relax0 = p.start_relax;
    p.start_relax = p.start_relax_retry;
    nw = planner.plan({start.s, start.d, start.v, 0.0}, boxes, grid.get(), nullptr, 0.0, t);
    p.start_relax = relax0;
    if (nw) {++plan_reasons_["relaxed_start"];}
  }
  if (!nw || (!nw->complete && nw->end_s() - start.s < p.escape_use_len)) {
    // 5차 노드 경로가 없거나 짧음: 꺾고-유지-풀기 조향 후보
    double delta0 = ctrl.delta;
    if (pose && std::abs(pose->v) > 0.05) {delta0 = std::atan(p.wheelbase * pose->w / pose->v);}
    const bool over_paint=crosses_yellow(veh,*boundary(),navigation_island(),p);
    PathPtr esc = planner.escape(veh.s,over_paint?start.d:veh.d,
      over_paint?std::atan(start.v):veh.psi,delta0,boxes,t);
    if (esc && (!nw || esc->end_s() > nw->end_s() + 0.2)) {
      nw = esc;
      ++plan_reasons_["escape"];
    }
  }
  // Escape candidates also have to pass the selected point; they cannot
  // replace it with a different fork destination.
  if(nw && planner.fork_target && nw->end_s()>=planner.fork_target->x &&
      std::abs(nw->d_at(planner.fork_target->x)-planner.fork_target->y)>=0.06) nw.reset();
  if(nw && !path_inside_yellow(*nw,*boundary(),navigation_island(),p,start.s)) nw.reset();
  plan_ms_.add((wall() - w0) * 1000);
  edges_.add(static_cast<double>(planner.last.edges));
  last_plan_t_ = t;
  last_plan_reason_ = why;
  path_version_ = tracker->version;
  if (!nw) {
    if(p.fork_shape && locked_fork_goal_odom_) yellow_route_blocked_=yellow_blocks_route(veh,t);
    ++plan_failures_;
    const PlanDiag & L = planner.last;
    std::ostringstream o;
    o << "{\"t\":" << jnum(t, 2) << ",\"reason\":" << jstr(why) << ",\"start\":[" << jnum(start.s) << ","
      << jnum(start.d) << "," << jnum(start.v) << "," << jnum(start.a) << "],\"stations\":[";
    for (size_t q = 0; q < L.stations.size(); ++q) {o << (q ? "," : "") << jnum(L.stations[q], 2);}
    o << "],\"nodes\":[";
    for (size_t q = 0; q < L.layers.size(); ++q) {o << (q ? "," : "") << L.layers[q].second.size();}
    o << "],\"edges\":" << L.edges << ",\"rejected\":{";
    bool first = true;
    for (auto & [k, v] : L.rejected) {o << (first ? "" : ",") << jstr(k) << ":" << v; first = false;}
    o << "},\"ego_clear\":[" << jnum(planner.last_ego_clear.first) << "," << jnum(planner.last_ego_clear.second)
      << "],\"boxes\":[";
    for (size_t q = 0; q < boxes.size() && q < 8; ++q) {
      o << (q ? "," : "") << "[" << jnum(boxes[q].s0, 2) << "," << jnum(boxes[q].s1, 2) << "," << jnum(boxes[q].d0, 2)
        << "," << jnum(boxes[q].d1, 2) << "]";
    }
    o << "]}";
    last_fail_json_ = o.str();
    for (auto & [k, v] : L.rejected) {fail_checks_[k] += v;}
    if (hard) {path_.reset();}
    return;
  }
  if (path_) {
    double mx = -kInf;
    for (size_t q = 0; q < nw->s.size(); ++q) {
      if (nw->s[q] >= s && nw->s[q] <= std::min(path_->end_s(), s + 1.5)) {
        const double dp = path_->d_at(nw->s[q]);
        if (std::isfinite(dp)) {mx = std::max(mx, std::abs(nw->d[q] - dp));}
      }
    }
    if (mx > 0.05) {++route_changes_;}
  }
  ++replans;
  ++plan_reasons_[why];
  path_ = nw;
  yellow_route_blocked_=obstacle_route_blocked_=false;
  if(p.fork_shape && planner.fork_target && !nw->complete && nw->end_s()<planner.fork_target->x)
    yellow_route_blocked_=yellow_blocks_route(veh,t);
  if(p.fork_shape && planner.fork_target && nw->end_s()>=planner.fork_target->x &&
     std::abs(nw->d_at(planner.fork_target->x)-planner.fork_target->y)<0.06) fork_path_committed_=true;
  if(frame && nw->end_s()>veh.s+0.15) {
    const double at=std::min(nw->end_s(),veh.s+0.40);
    double x,y;frame->to_odom(at,nw->d_at(at),x,y);last_route_=XY{x,y};
    last_route_points_.clear();
    for(double sk=veh.s+0.15;sk<=std::min(nw->end_s(),veh.s+0.80);sk+=0.08) {
      frame->to_odom(sk,nw->d_at(sk),x,y);last_route_points_.push_back({x,y});
    }
  }
}

bool ObjAvoidCore::yellow_blocks_route(const Veh & veh,double t)
{
  if(!planner.fork_target || !island_section_reached_at) return false;
  // Distinguish yellow road/island paint constraints from actual objects.
  // This relaxed-paint path is diagnostic only and is never driven.
  const auto diag=planner.last;const auto ego=planner.last_ego_clear;const bool has=planner.has_ego_clear;
  PriorBoundary relaxed({-100,100},{3,3},{-3,-3});
  planner.set_boundary(&relaxed);
  const auto saved_island=planner.island;planner.island.pose.valid=planner.island.pose.confirmed=false;
  const auto pose=pose_at(t);double ey;
  const auto start=start_state(veh,pose?std::optional<double>(pose->w):std::nullopt,ey);
  const auto probe=planner.plan(start,plan_boxes(),grid.get(),nullptr,0,t);
  planner.set_boundary(boundary());planner.island=saved_island;planner.last=diag;planner.last_ego_clear=ego;planner.has_ego_clear=has;
  return probe && probe->end_s()>=planner.fork_target->x &&
    std::abs(probe->d_at(planner.fork_target->x)-planner.fork_target->y)<0.06;
}

void ObjAvoidCore::update_clear_ahead(const Veh & veh)
{
  const double a = veh.s + p.body_front;
  const double b = std::min(a + p.done_clear_ahead, p.section_end);
  if (b <= a + 0.05) {clear_ahead_ = true; return;}
  const double wh = 0.5 * p.body_width;
  bool ok = true;
  for (double s = a; s < b && ok; s += 0.05) {
    ok = grid->is_seen(s, veh.d) && grid->is_seen(s, veh.d + wh) && grid->is_seen(s, veh.d - wh);
  }
  clear_ahead_ = ok;
}

bool ObjAvoidCore::startup_path_ready(const Veh & veh) const
{
  if(!path_ || !path_->complete || path_->s.size()<2 || path_->d.size()!=path_->s.size() ||
     path_->psi.size()!=path_->s.size()) return false;
  const double ahead=std::max(0.40,p.stop_buffer+0.20);
  if(path_->s.front()>veh.s+0.05 || path_->end_s()<veh.s+ahead ||
     !std::isfinite(verified_s) || verified_s<veh.s+ahead) return false;
  for(size_t i=0;i<path_->s.size();++i)
    if(!std::isfinite(path_->s[i]) || !std::isfinite(path_->d[i]) || !std::isfinite(path_->psi[i])) return false;
  const auto [ob,bd]=planner.path_clearance(*path_,veh.s-p.body_rear,plan_boxes());
  return ob<=p.blocked_tol && bd<=p.blocked_tol;
}

// ------------------------------------------------------------ 제어 주기
bool ObjAvoidCore::control(double t, double dt, double & v_out, double & steer_out)
{
  Lock g(mtx_);
  if (state == State::IDLE || state == State::DONE || !frame) {return false;}
  const std::optional<double> to = odom_.empty() ? std::nullopt : std::optional<double>(odom_.back()[0]);
  const auto veh = vehicle(t);
  const double pose_age=odom_.empty()?kInf:t-odom_.back()[0];
  if(startup_wait_) {
    std::string hold;
    if(!to || !veh || pose_age<-.05 || pose_age>p.odom_timeout) {
      startup_stopped_at_.reset();hold="odom_stale";
    } else if(std::abs(veh->v)>0.03) {
      startup_stopped_at_.reset();hold="startup_stopping";
    } else {
      if(!startup_stopped_at_) startup_stopped_at_=t;
      if(t-*startup_stopped_at_<3.0) hold="startup_hold";
      else if(!last_scan_t_ || t-*last_scan_t_>p.scan_timeout) hold="scan_stale";
      else if(t-emergency_t_<0.3) hold="emergency";
      else if(lane_blocking(t) || !startup_path_ready(*veh)) hold="startup_path_wait";
    }
    if(!hold.empty()) {
      set_state(State::STOP,hold,t);ctrl.v=0;v_out=0;steer_out=ctrl.delta-p.steer_trim;
      cmd_={v_out,steer_out};return true;
    }
    startup_wait_=false;startup_stopped_at_.reset();
  }
  if(p.fork_shape && !p.observe_mode && veh && pose_age>=0 && pose_age<=p.odom_timeout && check_done(*veh,t)) {
    set_state(State::DONE,done_reason,t);path_.reset();last_info={};verified_s=kNaN;
    return false;  // No zero-speed command: yield the mission output to lane.
  }
  const PathPtr path = path_;
  if (rev_) {
    if (!to || t - *to > p.odom_timeout || !veh) {
      end_reverse(t, "odom_stale");
    } else {
      reverse_step(t, dt, *veh, v_out, steer_out);
      cmd_ = {v_out, steer_out};
      return true;
    }
  }
  if (veh && rev_anchor_s_ && veh->s > *rev_anchor_s_ + 0.5) {
    rev_count_ = 0;  // 충분히 전진: 다음 막힘에서 다시 후진 허용
    rev_anchor_s_.reset();
  }
  std::string why;
  if (!to || t - *to > p.odom_timeout || !veh) {
    why = "odom_stale";
  } else if (!last_scan_t_ || t - *last_scan_t_ > p.scan_timeout) {
    why = "scan_stale";
  } else if (t - emergency_t_ < 0.3) {
    why = "emergency";
  } else if (lane_blocking(t)) {
    why = "first_plan";
  } else if (p.fork_shape && veh->s+p.body_front>=section_end0_-1.0 && t-island_frame_at>0.5) {
    why="island_image_stale";
  } else if (p.fork_shape && island_section_reached_at && !locked_fork_goal_odom_) {
    why=fork_side_?"waiting_island_confirmation":"waiting_lr_sign";
  } else if (obstacle_route_blocked_) {
    why="path_obstacle_blocked";
  } else if (yellow_route_blocked_) {
    why="yellow_line_blocked";
  } else if (!path) {
    why = "no_path";
  }
  if (why.empty()) {
    const double ey = ctrl.errors(*path, veh->s, veh->d, veh->psi).ey;
    const bool paint_recovery=crosses_yellow(*veh,*boundary(),navigation_island(),p) &&
      path_inside_yellow(*path,*boundary(),navigation_island(),p,veh->s-p.body_rear);
    if(paint_recovery && body_clearance(veh->s,veh->d,veh->psi,obstacle_array(tracker->confirmed_boxes()),p)<0)
      why="recovery_obstacle";
    else if (std::abs(ey)>p.deviation_stop && !p.observe_mode && !paint_recovery) {
      // 갈림길 구간에서는 이탈로 서지 않음(서면 조향만 유지돼 ey가 줄 수 없음)
      //   섬 매칭: 구간 끝 통과 또는 목표 잠김 / 표지판 받침: 방향 결정 후 구간 끝 통과 또는 결정 고정
      const bool in_fork=p.fork_shape ? (island_section_reached_at || locked_fork_goal_odom_) :
        (p.fork_enable && fork_side_!=0 && (fork_locked || veh->s+p.body_front>=p.section_end));
      if(!in_fork) why="path_deviation";
    }
  }
  // 단순 갈림길: 결정 후 fork_turn_after 지나면 정한 쪽으로 일정 조향, fork_turn_dist 꺾으면 DONE.
  // 꺾는 중에는 경로 없음·이탈은 무시하고 센서 끊김·비상만 정지
  if (p.fork_turn && fork_side_ != 0 && turn_from_ && !p.observe_mode && veh && why != "odom_stale" &&
    why != "scan_stale" && why != "emergency")
  {
    if (const auto pose = pose_at(t, 0.3)) {
      if (!turn_start_ &&
        std::hypot(pose->x - turn_from_->first, pose->y - turn_from_->second) >= p.fork_turn_after)
      {
        turn_start_ = std::make_pair(pose->x, pose->y);
        fork_locked = true;
      }
      if (turn_start_) {
        if (std::hypot(pose->x - turn_start_->first, pose->y - turn_start_->second) >= p.fork_turn_dist) {
          done_reason = fork_side_ > 0 ? "fork_left" : "fork_right";
          set_state(State::DONE, done_reason, t);
          cmd_ = {0, 0};
          return false;
        }
        set_state(State::RUN, "fork_turn", t);
        ctrl.v = p.fork_turn_speed;
        ctrl.delta = fork_side_ * p.fork_turn_steer;
        v_out = p.fork_turn_speed;
        steer_out = ctrl.delta - p.steer_trim;
        cmd_ = {v_out, steer_out};
        return true;
      }
    }
  }
  if (!why.empty()) {
    if (reason == "first_plan" && (why == "no_path" || why == "scan_stale") && replans == 0 &&
      t - activated_t_ < 1.0 + p.lane_align_wait)
    {
      why = "first_plan";
    }
    set_state(State::STOP, why, t);
    if (maybe_start_reverse(t, veh)) {
      reverse_step(t, dt, *veh, v_out, steer_out);
      cmd_ = {v_out, steer_out};
      return true;
    }
    ctrl.v = 0.0;  // hold_stop: 조향 유지, 속도 0
    v_out = 0.0;
    steer_out = ctrl.delta - p.steer_trim;
    cmd_ = {v_out, steer_out};
    return true;
  }
  if (!p.fork_shape && p.fork_enable && !p.observe_mode && fork_side_ == 0 && p.fork_island_s > 0 &&
    veh->s + p.body_front >= island_s() - p.fork_wait_dist)
  {
    // 표지판을 아직 못 봄: 섬 앞에서 서서 기다림. 너무 오래면 차선 주행에 넘김
    if (!wait_since_) {wait_since_ = t;}
    if (t - *wait_since_ > p.fork_sign_timeout) {
      done_reason = "no_sign";
      set_state(State::DONE, done_reason, t);
      cmd_ = {0, 0};
      return false;
    }
    set_state(State::STOP, "wait_sign", t);
    ctrl.v = 0.0;
    v_out = 0.0;
    steer_out = ctrl.delta - p.steer_trim;
    cmd_ = {v_out, steer_out};
    return true;
  }
  if (!p.fork_shape && !p.observe_mode && check_done(*veh, t)) {
    set_state(State::DONE, done_reason, t);
    cmd_ = {0, 0};
    return false;
  }
  double v_cmd, delta;
  CtrlInfo info;
  ctrl.step(*path, veh->s, veh->d, veh->psi, dt, p.v_max, std::isfinite(verified_s) ? verified_s : -1e9, v_cmd,
    delta, info);
  turn_diag(t);
  ey_.add(std::abs(info.ey));
  epsi_.add(std::abs(info.epsi));
  last_info = info;
  if (v_cmd > 0) {
    set_state(State::RUN, "tracking", t);
    stuck_since_.reset();
  } else if (!path->complete) {
    set_state(State::STOP, "path_blocked_ahead", t);  // 안전 경로가 앞에서 끊김 -> 끝에서 정지
    if (maybe_start_reverse(t, veh)) {
      reverse_step(t, dt, *veh, v_out, steer_out);
      cmd_ = {v_out, steer_out};
      return true;
    }
  } else {
    set_state(State::STOP, "unverified_ahead", t);
  }
  v_out = v_cmd;
  steer_out = delta - p.steer_trim;
  cmd_ = {v_out, steer_out};
  return true;
}

// ------------------------------------------------------------ 후진 탈출
static bool is_stuck(const std::string & r)
{
  return r == "no_path" || r == "path_blocked_ahead";
}

bool ObjAvoidCore::maybe_start_reverse(double t, const std::optional<Veh> & veh)
{
  if(p.fork_shape) {
    if(reason!="yellow_line_blocked" || !yellow_route_blocked_ || !island_section_reached_at ||
       !locked_fork_goal_odom_ || !fork_side_ || p.observe_mode || !veh) {stuck_since_.reset();return false;}
    if(!stuck_since_) stuck_since_=t;
    if(t-*stuck_since_<p.reverse_wait || std::abs(veh->v)>0.03 || rev_anchor_s_ || rev_count_>=p.reverse_max ||
       !last_scan_t_ || t-*last_scan_t_>p.scan_timeout) return false;
    const auto pose=pose_at(t);if(!pose) return false;
    Rev r;r.fork_lane=true;r.t0=t;r.x0=r.last_x=pose->x;r.y0=r.last_y=pose->y;
    rev_=r;++rev_count_;++reverses_;rev_anchor_s_=veh->s;
    set_state(State::REVERSE,"reverse_yellow_line",t);return true;
  }
  if (!is_stuck(reason) && !(reason=="emergency" && !path_)) {stuck_since_.reset(); return false;}
  if (!stuck_since_) {stuck_since_ = t;}
  if (!p.reverse_enable || p.observe_mode || !veh || rev_count_ >= p.reverse_max ||
    t - *stuck_since_ < p.reverse_wait || std::abs(veh->v) > 0.03)
  {
    return false;
  }
  if (!last_scan_t_ || t - *last_scan_t_ > p.scan_timeout) {return false;}
  const auto pose=pose_at(t);
  if(!pose || driven_trail_.size()<2) return false;
  Rev r;r.t0=t;r.x0=r.last_x=pose->x;r.y0=r.last_y=pose->y;
  XY previous{pose->x,pose->y};double distance=0;
  for(auto it=driven_trail_.rbegin();it!=driven_trail_.rend();++it) {
    const double step=std::hypot(it->x-previous.x,it->y-previous.y);
    if(step>0.30) return false;
    if(step<1e-6) continue;
    if(distance+step>=p.reverse_dist) {
      const double u=(p.reverse_dist-distance)/step;
      r.trail.push_back({previous.x+u*(it->x-previous.x),previous.y+u*(it->y-previous.y)});
      distance=p.reverse_dist;break;
    }
    distance+=step;r.trail.push_back(*it);previous=*it;
  }
  if(distance<p.reverse_dist-0.005 || r.trail.empty()) return false;
  bool saved=false;
  for(const auto & point:last_route_points_) {
    double s,d;frame->to_road(point.x,point.y,s,d);
    if(s>=veh->s+0.15 && s<=veh->s+0.85) {failed_routes_.push_back({s,d});saved=true;}
  }
  if(!saved) {
    failed_routes_.push_back({veh->s+0.35,veh->d});
  }
  rev_=std::move(r);++rev_count_;++reverses_;
  rev_anchor_s_=veh->s;
  path_.reset();
  set_state(State::REVERSE,"reverse_retrace",t);
  return true;
}

std::pair<double, double> ObjAvoidCore::reverse_clear(double s, double d, double psi,
  const std::vector<Box> & boxes) const
{
  // 차체 네 모서리의 (추적 박스 여유, 노란 경계 여유). 노란선은 카메라로 본 곳만
  const double c = std::cos(psi), sn = std::sin(psi), wh = 0.5 * p.body_width;
  double cb = kInf, cbox = kInf;
  for (double ex : {p.body_front, -p.body_rear}) {
    for (double ey : {wh, -wh}) {
      const double cs = s + ex * c - ey * sn, cd = d + ex * sn + ey * c;
      double lo, hi;
      boundary()->limits(cs, lo, hi);
      double rg = cd - lo, lf = hi - cd;
      if (boundary()->has_known()) {
        if (!boundary()->known(cs, -1)) {rg = kInf;}
        if (!boundary()->known(cs, 1)) {lf = kInf;}
      }
      cb = std::min({cb, rg, lf});
      for (const Box & bx : boxes) {
        const double gap = std::max(std::max(bx.s0 - cs, cs - bx.s1), std::max(bx.d0 - cd, cd - bx.d1));
        cbox = std::min(cbox, gap);
      }
    }
  }
  cbox=body_clearance(s,d,psi,obstacle_array(boxes),p);
  return {cbox, cb};
}

void ObjAvoidCore::reverse_step(double t, double dt, const Veh & veh, double & v_out, double & steer_out)
{
  Rev & r=*rev_;
  const auto pose=pose_at(t);
  if(!pose) {end_reverse(t,"odom_stale");v_out=0;steer_out=ctrl.delta-p.steer_trim;return;}
  const double step=std::hypot(pose->x-r.last_x,pose->y-r.last_y);
  if(step>0.1) {end_reverse(t,"reverse_odom_jump");v_out=0;steer_out=ctrl.delta-p.steer_trim;return;}
  r.travelled+=step;r.last_x=pose->x;r.last_y=pose->y;
  if(r.fork_lane) {
    const double limit=std::min(0.20,p.reverse_dist);
    if(r.travelled>=limit || t-r.t0>std::min(6.0,p.reverse_timeout)) {
      end_reverse(t,r.travelled>=limit?"reversed":"reverse_timeout");
      v_out=0;steer_out=ctrl.delta-p.steer_trim;return;
    }
    const double want=-fork_side_*p.max_steering;
    // Unknown rear space is permitted for this explicitly requested manoeuvre;
    // mapped objects, pose discontinuities, timeout and distance still stop it.
    const double ds=0.03,psi=veh.psi-ds*std::tan(want)/p.wheelbase;
    const double mid=0.5*(veh.psi+psi);
    if(body_clearance(veh.s-ds*std::cos(mid),veh.d-ds*std::sin(mid),psi,
       obstacle_array(tracker->confirmed_boxes()),p)<0) {
      end_reverse(t,"reverse_blocked");v_out=0;steer_out=ctrl.delta-p.steer_trim;return;
    }
    const double step_steer=p.max_steering_rate*std::clamp(dt,1e-3,0.2);
    ctrl.delta+=std::clamp(want-ctrl.delta,-step_steer,step_steer);
    v_out=std::abs(want-ctrl.delta)<0.1 ? -std::min({0.08,p.reverse_speed,(limit-r.travelled)/std::max(dt,0.01)}) : 0;
    steer_out=ctrl.delta-p.steer_trim;return;
  }
  while(r.cursor+1<r.trail.size() && std::hypot(r.trail[r.cursor].x-pose->x,r.trail[r.cursor].y-pose->y)<0.06) ++r.cursor;
  size_t look=r.cursor;
  while(look+1<r.trail.size() && std::hypot(r.trail[look].x-pose->x,r.trail[look].y-pose->y)<0.08) ++look;
  const double dx=r.trail[look].x-pose->x,dy=r.trail[look].y-pose->y;
  const double lateral=-std::sin(pose->yaw)*dx+std::cos(pose->yaw)*dy;
  const double want=std::clamp(std::atan2(2*p.wheelbase*lateral,std::max(0.0025,dx*dx+dy*dy)),-p.max_steering,p.max_steering);
  const double moved=r.travelled;
  const std::vector<Box> boxes = plan_boxes();
  const auto c_now = reverse_clear(veh.s, veh.d, veh.psi, boxes);
  std::optional<double> target;
  std::string end;
  if (moved >= p.reverse_dist) {
    end = "reversed";
  } else if (t - r.t0 > p.reverse_timeout) {
    end = "reverse_timeout";
  } else {
    const double cand=want;
    bool safe=true;
    for(double ds=0.02;ds<=0.0801 && safe;ds+=0.02) {
      const double psi_n=veh.psi-ds*std::tan(cand)/p.wheelbase;
      const double mid=0.5*(veh.psi+psi_n);
      const double ns=veh.s-ds*std::cos(mid),nd=veh.d-ds*std::sin(mid);
      const auto [ob,bd]=reverse_clear(ns,nd,psi_n,boxes);
      safe=(ob>=p.reverse_margin || (ob>=0 && ob>=c_now.first-0.003)) &&
        (bd>=0 || bd>=c_now.second-0.003);

    }
    if(safe) target=cand;
    if (!target) {end = "reverse_blocked";}
  }
  if (!end.empty()) {
    end_reverse(t, end);
    v_out = 0.0;
    steer_out = ctrl.delta - p.steer_trim;
    return;
  }
  const double stp = p.max_steering_rate * std::clamp(dt, 1e-3, 0.2);
  ctrl.delta += std::clamp(*target - ctrl.delta, -stp, stp);
  v_out = std::abs(*target - ctrl.delta) < 0.1 ? -std::min(p.reverse_speed,std::max(0.0,p.reverse_dist-moved)/std::max(dt,0.01)) : 0.0;  // 바퀴를 먼저 꺾고 움직임
  steer_out = ctrl.delta - p.steer_trim;
}

void ObjAvoidCore::end_reverse(double t, const std::string & why)
{
  if(const auto pose=pose_at(t)) {
    while(!driven_trail_.empty() && std::hypot(driven_trail_.back().x-pose->x,driven_trail_.back().y-pose->y)>0.03) driven_trail_.pop_back();
    driven_trail_.push_back({pose->x,pose->y});
  }
  const bool fork_recovery=rev_ && rev_->fork_lane;
  rev_.reset();
  if(fork_recovery) {fork_path_committed_=false;yellow_route_blocked_=obstacle_route_blocked_=false;}
  ctrl.v = 0.0;
  blocked_count_ = 0;
  stuck_since_ = t;
  path_.reset();
  last_plan_t_ = -1e9;
  set_state(State::STOP, why, t);
}

void ObjAvoidCore::turn_diag(double t)
{
  cmd_hist_.push_back({t, ctrl.delta});
  while (cmd_hist_.size() > 50) {cmd_hist_.pop_front();}
  const auto pose = pose_at(t);
  double sum = 0;
  int n = 0;
  for (auto & [tt, dd] : cmd_hist_) {
    if (t - 0.3 <= tt && tt <= t - 0.2) {sum += dd; ++n;}
  }
  if (!pose || n == 0 || std::abs(pose->v) < 0.08) {return;}
  const double dc = sum / n;
  if (std::abs(dc) < 0.12) {return;}
  const double r = (pose->w / pose->v) / (std::tan(dc) / p.wheelbase);
  if (r > 0.0 && r < 3.0) {turn_ratio_ = turn_ratio_ ? 0.95 * *turn_ratio_ + 0.05 * r : r;}
}

std::vector<Box> ObjAvoidCore::plan_boxes() const
{
  std::vector<Box> b = tracker->confirmed_boxes();
  if(!rev_ && frame) for(const auto & failed:failed_routes_) {
    const double s=failed.x,d=failed.y;
    b.push_back({s-0.10,s+0.10,d-0.12,d+0.12});
  }
  // 섬(평평한 노면 표시라 LiDAR로 안 보임)은 방향이 정해진 뒤에만 가상 장애물로: 그 전에는 가운데에서 대기해
  // 어느 표지판이 와도 같은 거리로 갈래에 들어갈 수 있게
  if (!p.fork_shape && p.fork_enable && fork_side_ != 0) {
    if (const auto pb = sign_post()) {
      // 표지판 받침 기준: A면 받침 오른쪽 전체, B면 왼쪽 전체를 받침부터 섬 길이만큼 막아 정한 쪽으로만 지나가게
      const double s0 = pb->s0, s1 = pb->s0 + p.fork_island_len;
      const ForkGeom fg = fork_geometry(fork_side_, *pb, p);  // 11-20: 섬 폭 기준(fork_island_half_width)
      b.push_back(Box{s0, s1, fg.wall_d0, fg.wall_d1});
    } else if (island_known()) {
      b.push_back(island_box());
    }
  }
  return b;
}

std::optional<Box> ObjAvoidCore::sign_post() const
{
  if (!p.fork_enable || !p.fork_use_sign_post || !tracker) {return std::nullopt;}
  if (post_id_ >= 0) {
    for (const Track & tr : tracker->tracks) {
      if (tr.id == post_id_) {return tr.box;}
    }
  }
  if (post_box_ && post_align_ == align_count_) {return post_box_;}  // 추적이 끝나도 마지막 박스(같은 좌표계일 때)
  return std::nullopt;
}

bool ObjAvoidCore::match_post(double stamp, double u, double cam_dist)
{
  const auto pose = pose_at(stamp, 0.5);
  if (!pose || !frame) {return false;}
  const double c = std::cos(pose->yaw), sn = std::sin(pose->yaw);
  const double cx = pose->x + c * p.fork_cam_x - sn * p.fork_cam_y;
  const double cy = pose->y + sn * p.fork_cam_x + c * p.fork_cam_y;
  const double ray = pose->yaw + std::atan((p.fork_cam_cx - u) / p.fork_cam_fx);  // 왼쪽 +
  const bool cam = std::isfinite(cam_dist) && cam_dist > 0;
  // 1) 그 방향(± bearing_tol)의 LiDAR 박스 중 카메라 추정 거리와 맞는 가장 가까운 것
  int best = -1;
  double best_d = kInf;
  for (const Track & tr : tracker->tracks) {
    if (!tr.box.robot_candidate) {continue;}
    double x, y;
    frame->to_odom(tr.box.cs(), tr.box.cd(), x, y);
    const double dist = std::hypot(x - cx, y - cy);
    const double diff = std::abs(wrap(std::atan2(y - cy, x - cx) - ray));
    if (dist < p.fork_post_max_dist && diff < p.fork_bearing_tol && std::abs(tr.box.cd()) < p.fork_post_center_d &&
      (!cam || std::abs(dist - cam_dist) < p.fork_post_match) && dist < best_d)
    {
      best = tr.id;
      best_d = dist;
    }
  }
  if (best >= 0) {
    for (const Track & tr : tracker->tracks) {
      if (tr.id == best) {post_id_ = best; post_box_ = tr.box; post_align_ = align_count_;}
    }
    return true;
  }
  // 2) LiDAR에 없으면 카메라 추정 위치(방향 + 판 크기로 구한 거리)를 받침으로(이전 추정과 평균)
  if (!cam || cam_dist > p.fork_post_max_dist || post_id_ >= 0) {return post_id_ >= 0;}
  double s, d;
  frame->to_road(cx + cam_dist * std::cos(ray), cy + cam_dist * std::sin(ray), s, d);
  if (post_box_ && post_align_ == align_count_) {
    s = 0.5 * (s + post_box_->cs());
    d = 0.5 * (d + post_box_->cd());
  }
  post_box_ = Box{s - 0.06, s + 0.06, d - 0.06, d + 0.06};
  post_align_ = align_count_;
  return true;
}

ForkGeom fork_geometry(int side, const Box & pb, const Params & p)
{
  ForkGeom g;
  if (p.fork_island_half_width > 0) {
    // 11-20(bag 114541): 받침 끝 + 0.3이면 차 옆면이 섬 노란 선 위(목표 0.11, 섬 선 0.0) -> 섬 선 기준
    const double c = pb.cd(), h = p.fork_island_half_width;
    const double off = h + 0.5 * p.body_width + p.fork_island_clear;
    g.wall_d0 = side > 0 ? -3.0 : c - h;
    g.wall_d1 = side > 0 ? c + h : 3.0;
    g.target = side > 0 ? c + off : c - off;
  } else {
    g.wall_d0 = side > 0 ? -3.0 : pb.d0;
    g.wall_d1 = side > 0 ? pb.d1 : 3.0;
    g.target = side > 0 ? pb.d1 + p.fork_pass_offset : pb.d0 - p.fork_pass_offset;
  }
  if (p.fork_target_limit > 0) {
    g.target = std::clamp(g.target, -p.fork_target_limit, p.fork_target_limit);
  } else if (p.fork_target_clamp && !p.lane_centers.empty()) {
    const auto [lo, hi] = std::minmax_element(p.lane_centers.begin(), p.lane_centers.end());
    g.target = std::clamp(g.target, *lo, *hi);
  }
  return g;
}

// 목표 차로: 받침을 알면 받침 기준(fork_geometry), 모르면 바깥 차로
void ObjAvoidCore::fork_target()
{
  if (fork_side_ == 0) {return;}
  if (const auto pb = sign_post()) {
    p.home_d = fork_geometry(fork_side_, *pb, p).target;  // 받침이 밀려도 목표는 도로 안(11-19 bag 114208: -0.98 -> 경로 없음)
  } else {
    const auto [mn, mx] = std::minmax_element(p.lane_centers.begin(), p.lane_centers.end());
    p.home_d = fork_side_ > 0 ? *mx : *mn;
  }
  p.plan_beyond_end = island_s() + p.fork_plan_beyond - p.section_end;
  p.roi_beyond_end = std::max(roi_beyond0_, p.plan_beyond_end);
  fork_replan_ = true;
}

void ObjAvoidCore::on_sign_detail(double stamp, bool right, double u, double h)
{
  Lock g(mtx_);
  if (p.fork_shape || !p.fork_enable || p.fork_turn || !p.fork_use_sign_post || !frame || !tracker || fork_locked) {return;}
  if (fork_side_ != 0 && (right ? -1 : 1) != fork_side_) {return;}  // 정한 방향과 다른 표지판 박스는 무시
  const double cam_dist = h > 1.0 ? p.fork_cam_fx * p.fork_sign_size / h : kNaN;
  const auto before = sign_post();
  if (match_post(stamp, u, cam_dist) && fork_side_ != 0) {
    const auto now = sign_post();
    // 받침을 처음 찾았거나 위치가 5 cm 넘게 바뀌면 목표·계획을 받침 기준으로 다시
    if (!before || !now || std::abs(now->cd() - before->cd()) > 0.05 || std::abs(now->cs() - before->cs()) > 0.05) {
      fork_target();
    }
  }
}

void ObjAvoidCore::on_sign(double t, bool right)
{
  Lock g(mtx_);
  if(!p.fork_enable) return;
  if(!frame || state==State::IDLE) {pending_sign_=std::make_pair(t,right);return;}
  if(state==State::DONE || fork_locked) return;
  (right ? fork_votes_r : fork_votes_l) += 1;
  int side = 0;
  if(p.fork_sign_min<=1) side=right?-1:1;
  else if (fork_votes_l >= p.fork_sign_min && fork_votes_l > fork_votes_r) {side = 1;}
  else if (fork_votes_r >= p.fork_sign_min && fork_votes_r > fork_votes_l) {side = -1;}
  // 11-19: once decided, an opposite sign must repeat fork_flip_min times in a row and outvote the
  // current side (11-18/19 bags: one stray LEFT flipped RIGHT with 22-50 votes -> wrong branch).
  if (p.fork_flip_min > 0 && fork_side_ != 0) {
    const int det = right ? -1 : 1;
    if (det != fork_side_) {
      ++fork_flip_streak;
      const int opp = det > 0 ? fork_votes_l : fork_votes_r, cur = fork_side_ > 0 ? fork_votes_l : fork_votes_r;
      if (fork_flip_streak < p.fork_flip_min || opp <= cur) {side = fork_side_;}
    } else {
      fork_flip_streak = 0;
    }
    if (side != fork_side_) {fork_flip_streak = 0;}
  }
  if (side != 0 && side != fork_side_ && p.fork_turn) {
    // 단순 갈림길: 방향과 결정 위치만 기록(꺾기 시작 전이면 방향은 다수결로 바뀔 수 있음)
    fork_side_ = side;
    if (!turn_from_) {
      if (const auto pose = pose_at(t, 0.5)) {turn_from_ = std::make_pair(pose->x, pose->y);}
    }
    return;
  }
  if (side != 0 && side != fork_side_) {
    if (!p.fork_shape && !island_est_ && p.fork_island_s <= 0) {
      // 섬 위치 추정: 표지판을 결정한 순간 차 앞끝 + fork_sign_to_island
      if (const auto veh = vehicle(t)) {island_est_ = veh->s + p.body_front + p.fork_sign_to_island;}
    }
    if(p.fork_shape) {if(!fork_side_) island_sign_after=t;fork_side_=side; planner.island_side=side;path_.reset();fork_replan_=true;
      if(navigation_island().pose.confirmed) apply_fork(side);
    } else if (island_known()) {apply_fork(side);}
  }
}

void ObjAvoidCore::apply_fork(int side)
{
  fork_side_ = side;
  if(p.fork_shape) {
    const auto model=navigation_island();const auto & ip=model.pose;
    planner.island_side=side;
    p.home_d=home_d0_;
    p.plan_beyond_end=ip.s+island_mapper.shape.length+0.5-p.section_end;
    p.roi_beyond_end=std::max(roi_beyond0_,p.plan_beyond_end);
    planner.island_side=side;fork_replan_=true;return;
  }
  fork_target();  // 받침 기준(없으면 바깥 차로)으로 목표·계획 범위, 즉시 재계획
  wait_since_.reset();
}

std::optional<XY> ObjAvoidCore::fork_goal() const
{
  Lock lock(mtx_);
  if(locked_fork_goal_odom_ && frame) {
    XY q;frame->to_road(locked_fork_goal_odom_->x,locked_fork_goal_odom_->y,q.x,q.y);return q;
  }
  const auto sh=navigation_island();
  if(!fork_side_ || !sh.pose.confirmed || !island_section_reached_at) return std::nullopt;
  const double s=sh.pose.s+sh.width/2+0.5;
  double right,left;boundary()->limits(s,right,left);
  const double center=sh.pose.d+std::tan(sh.pose.yaw)*(s-sh.pose.s);
  const double half=sh.width/(2*std::cos(sh.pose.yaw));
  return XY{s,fork_side_>0?(left+center+half)/2:(right+center-half)/2};
}

bool ObjAvoidCore::check_done(const Veh & veh, double t)
{
  if(p.fork_shape) {
    const auto sh=navigation_island();
    if(!fork_side_ || !sh.pose.confirmed) {entry_stable.reset();return false;}
    const auto q=sh.local(veh.s,veh.d);
    if(q.x+p.body_front>=-p.fork_lock_dist) fork_locked=true;
    const auto goal=fork_goal();
    if(!goal || std::hypot(veh.s-goal->x,veh.d-goal->y)>p.fork_goal_tolerance || fork_side_*q.y<=0) return false;
    done_reason=fork_side_>0?"fork_left":"fork_right";return true;
  }
  if (p.fork_enable && p.fork_turn) {return false;}  // 단순 갈림길: 꺾기를 마치면 DONE
  if (p.fork_enable) {
    // 갈림길: 정한 갈래로 섬 시작을 지나야 완료(그 전에는 '가운데 복귀'·'구간 끝' 완료 없음)
    const double front = veh.s + p.body_front;
    if (fork_side_ != 0 && island_known() && front >= island_s() - p.fork_lock_dist) {fork_locked = true;}
    const auto pb = sign_post();
    const double side_d = pb ? veh.d - pb->cd() : veh.d;  // 받침(없으면 도로 가운데) 기준 정한 쪽에 있는지
    if (fork_side_ != 0 && front >= island_s() + p.fork_enter_len && fork_side_ * side_d > (pb ? 0.15 : 0.2)) {
      done_reason = fork_side_ > 0 ? "fork_left" : "fork_right";
      return true;
    }
    return false;
  }
  if (tracker->passed < p.done_min_passed || tracker->section_count() > 0 || !clear_ahead_) {
    center_since_.reset();
    return false;
  }
  if (veh.s + p.body_front >= p.section_end - 0.1) {
    done_reason = "section_end";
    return true;
  }
  if (std::abs(veh.d - p.home_d) <= p.done_center_tol && std::abs(veh.psi) <= p.done_heading_tol) {
    if (!center_since_) {center_since_ = t;}
    if (t - *center_since_ >= p.done_hold) {
      done_reason = "centered";
      return true;
    }
  } else {
    center_since_.reset();
  }
  return false;
}

// ------------------------------------------------------------ 보고
std::string ObjAvoidCore::status_json() const
{
  Lock g(mtx_);
  std::ostringstream o;
  const char * stage=!p.fork_shape?"normal":state==State::DONE?"done":fork_path_committed_?"path_locked":locked_fork_goal_odom_?"target_selected":island_section_reached_at?"await_island":"approach";
  o << "{\"startup\":{\"waiting\":" << (startup_wait_?"true":"false")
    << ",\"hold_sec\":3.0,\"stopped_at\":" << (startup_stopped_at_?jnum(*startup_stopped_at_):"null") << "},\"fork_stage\":" << jstr(stage) << ",\"mapping_epoch\":" << mapping_epoch << ",\"mapping_started_at\":" << jnum(activated_t_,6) << ",\"state\":" << jstr(state_name(state)) << ",\"reason\":" << jstr(reason) << ",\"replans\":" << replans
    << ",\"recovery\":{\"policy\":" << jstr(p.fork_shape?"yellow_line_only":p.reverse_enable?"reverse":"normal")
    << ",\"mode\":" << jstr(rev_?(rev_->fork_lane?"fork_side":"retrace"):"idle")
    << ",\"distance\":" << jnum(rev_?rev_->travelled:0)
    << ",\"target\":" << jnum(p.fork_shape?std::min(0.20,p.reverse_dist):p.reverse_dist)
    << ",\"goal_locked\":" << (locked_fork_goal_odom_?"true":"false")
    << ",\"path_committed\":" << (fork_path_committed_?"true":"false")
    << ",\"yellow_blocked\":" << (yellow_route_blocked_?"true":"false")
    << ",\"excluded_regions\":" << failed_routes_.size() << "}"
    << ",\"reverses\":" << reverses_ << ",\"route_changes\":" << route_changes_ << ",\"plan_failures\":"
    << plan_failures_ << ",\"plan_reasons\":{";
  bool first = true;
  for (auto & [k, v] : plan_reasons_) {o << (first ? "" : ",") << jstr(k) << ":" << v; first = false;}
  o << "},\"last_plan_reason\":" << jstr(last_plan_reason_) << ",\"stop_events\":[";
  const size_t st = stop_events_.size() > 20 ? stop_events_.size() - 20 : 0;
  for (size_t q = st; q < stop_events_.size(); ++q) {
    o << (q > st ? "," : "") << "[" << jnum(stop_events_[q].first, 2) << "," << jstr(stop_events_[q].second) << "]";
  }
  o << "],\"n_stop_events\":" << stop_events_.size() << ",\"plan_ms\":" << plan_ms_.json() << ",\"percep_ms\":"
    << percep_ms_.json() << ",\"scan_age_ms\":" << scan_age_ms_.json() << ",\"scan_queue_ms\":" <<
    scan_queue_ms.json() << ",\"edges\":" << edges_.json() << ",\"ey\":" << ey_.json() << ",\"epsi\":" <<
    epsi_.json() << ",\"cmd\":[" << jnum(cmd_.first) << "," << jnum(cmd_.second, 4) << "],\"done_reason\":" <<
    jstr(done_reason) << ",\"last_fail\":" << last_fail_json_ << ",\"fail_checks\":{";
  first = true;
  for (auto & [k, v] : fail_checks_) {o << (first ? "" : ",") << jstr(k) << ":" << v; first = false;}
  o << "},\"turn_ratio\":" << (turn_ratio_ ? jnum(*turn_ratio_) : "null") << ",\"lane\":{\"frames\":" <<
    lane_frames_ << ",\"lidar_dropped\":" << lane_lidar_dropped_ << ",\"ms\":" << lane_ms_.json() <<
    ",\"aligned\":" << (aligned ? "true" : "false") << ",\"aligns\":" << align_count_ << ",\"last_align\":" <<
    align_info_json_ << ",\"island_s\":" << (island_ ? jnum(*island_) : "null") << ",\"section_end\":" <<
    jnum(p.section_end) << "}";
  o << ",\"road_mapping\":{\"enabled\":" << (p.road_outline?"true":"false")
    << ",\"source\":" << jstr((aligned || road_boundary_->model)?"matched":"prior")
    << ",\"center_source\":" << jstr(road_boundary_->model?"island":aligned?"outer_road":"prior")
    << ",\"island_anchored\":" << (road_boundary_->model?"true":"false")
    << ",\"frozen\":false,\"island_side_gap_m\":0.5"
    << ",\"using_saved_map\":" << ((!road_boundary_->model && (!aligned || !outline_fit.valid))?"true":"false")
    << ",\"valid\":" << ((road_boundary_->model || outline_fit.valid)?"true":"false")
    << ",\"longitudinal_observable\":" << (outline_fit.longitudinal?"true":"false")
    << ",\"error\":" << jnum(outline_fit.error) << ",\"hits\":" << outline_fit.hits << ",\"inlier_ratio\":" << jnum(outline_fit.inlier_ratio)
    << ",\"road_points\":" << road_match_points.size() << ",\"island_points\":" << island_match_points.size()
    << ",\"unmatched_points\":" << unmatched_points.size() << ",\"raw_points\":" << yellow_raw_count
    << ",\"filtered_points\":" << yellow_filtered_count << ",\"ms\":" << mapping_ms.json() << "}";
  o << ",\"clearance_policy\":{\"preferred\":" << jnum(p.preferred_obstacle_clearance)
    << ",\"selected\":" << jnum(planner.last.clearance_margin)
    << ",\"fallback\":" << (planner.last.clearance_fallback?"true":"false") << "}";
  const auto mapped=navigation_island();const auto & ip=mapped.pose;
  o << ",\"island_mapping\":{\"valid\":" << (ip.valid?"true":"false")
    << ",\"source\":" << jstr(island_mapper.shape.pose.confirmed?"observed":retained_island_.pose.valid?"saved":"candidate")
    << ",\"observation_confirmed\":" << (island_mapper.shape.pose.confirmed?"true":"false")
    << ",\"road_aligned\":" << (aligned?"true":"false")
    << ",\"confirmed\":" << (ip.confirmed?"true":"false") << ",\"s\":" << jnum(ip.s)
    << ",\"section_offset_cm\":" << (ip.valid?jnum(100*(ip.s-p.section_end)):"null")
    << ",\"section_reached\":" << (island_section_reached_at?"true":"false")
    << ",\"section_reached_at\":" << (island_section_reached_at?jnum(*island_section_reached_at):"null")
    << ",\"rejection\":" << jstr(island_mapper.rejection)
    << ",\"observation_source\":\"accumulated_map\",\"map_points\":" << lane_cells.size()
    << ",\"fit_input_points\":" << island_mapper.input_count
    << ",\"evaluations\":" << island_mapper.evaluations
    << ",\"candidates\":" << island_mapper.candidate_count
    << ",\"misses\":" << island_mapper.misses << ",\"fresh_frames\":" << ip.stable
    << ",\"d\":" << jnum(ip.d) << ",\"yaw\":" << jnum(ip.yaw) << ",\"error\":" << jnum(ip.error)
    << ",\"searching\":" << (searching?"true":"false") << ",\"search_distance\":" << jnum(search_distance) << "}";
 o << ",\"fork\":{\"enable\":" << (p.fork_enable ? "true" : "false") << ",\"side\":" <<
    jstr(fork_side_ > 0 ? "left" : (fork_side_ < 0 ? "right" : "none")) << ",\"votes\":[" << fork_votes_l << "," <<
    fork_votes_r << "],\"locked\":" << (fork_locked ? "true" : "false") << ",\"island_s\":" << jnum(island_s()) <<
    ",\"home_d\":" << jnum(p.home_d) << ",\"turn\":" <<
    jstr(!p.fork_turn ? "off" : (turn_start_ ? "turning" : (turn_from_ ? "decided" : "wait_sign")));
  if (const auto pb = sign_post()) {
    o << ",\"post\":[" << jnum(pb->s0, 2) << "," << jnum(pb->s1, 2) << "," << jnum(pb->d0, 2) << "," << jnum(pb->d1, 2)
      << "]";
  }
  o << "}";
  if (tracker) {
    o << ",\"tracks\":" << tracker->confirmed_count() << ",\"passed\":" << tracker->passed <<
      ",\"structure_tracks\":" << tracker->structure_count() << ",\"robot_tracks\":" <<
      tracker->confirmed_count() - tracker->structure_count();
  }
  if (last_info.valid) {o << ",\"ey_now\":" << jnum(last_info.ey, 4) << ",\"epsi_now\":" << jnum(last_info.epsi, 4);}
  o << ",\"impl\":\"cpp\"}";
  return o.str();
}

}  // namespace objavoid
