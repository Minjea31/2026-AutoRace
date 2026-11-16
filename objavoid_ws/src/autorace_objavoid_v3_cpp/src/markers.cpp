#include "autorace_objavoid_v3_cpp/markers.hpp"

#include <cmath>
#include <cstdio>

#include <geometry_msgs/msg/point.hpp>

namespace objavoid {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using geometry_msgs::msg::Point;

namespace {

Marker mk(const std::string & ns, int id, int type, const builtin_interfaces::msg::Time & stamp,
  std::array<double, 4> col, std::array<double, 3> scale = {0.02, 0.02, 0.02})
{
  Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = scale[0];
  m.scale.y = scale[1];
  m.scale.z = scale[2];
  m.color.r = col[0];
  m.color.g = col[1];
  m.color.b = col[2];
  m.color.a = col[3];
  return m;
}

Point pt(const RoadFrame & f, double s, double d, double z = 0.02)
{
  Point q;
  f.to_odom(s, d, q.x, q.y);
  q.z = z;
  return q;
}

void rect(Marker & m, const RoadFrame & f, double s0, double s1, double d0, double d1, double z = 0.03)
{
  const double c[5][2] = {{s0, d0}, {s1, d0}, {s1, d1}, {s0, d1}, {s0, d0}};
  for (auto & q : c) {m.points.push_back(pt(f, q[0], q[1], z));}
}

std::string fmt(const char * f, double a, double b)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, a, b);
  return buf;
}

}  // namespace

MarkerArray build_markers(const ObjAvoidCore & core, const builtin_interfaces::msg::Time & stamp, double t)
{
  MarkerArray arr;
  if (!core.frame) {return arr;}
  const RoadFrame & f = *core.frame;
  const Params & p = core.p;
  Marker clear;
  clear.action = Marker::DELETEALL;
  arr.markers.push_back(clear);

  // 장애물 감지: 이번 스캔 도로 안 LiDAR 점(분홍), 검출 박스(초록), 추적 박스(빨강/주황)
  if (!core.last_points.empty()) {
    Marker m = mk("lidar_roi_points", 0, Marker::POINTS, stamp, {1.0, 0.3, 0.8, 1.0}, {0.025, 0.025, 0});
    for (const Pt2 & q : core.last_points) {m.points.push_back(pt(f, q.s, q.d, 0.06));}
    arr.markers.push_back(m);
  }
  for (size_t k = 0; k < core.last_boxes.size(); ++k) {
    const Box & b = core.last_boxes[k];
    Marker m = mk("scan_detections", static_cast<int>(k), Marker::LINE_STRIP, stamp, {0.2, 1.0, 0.3, 0.9},
        {0.01, 0, 0});
    rect(m, f, b.s0, b.s1, b.d0, b.d1, 0.05);
    arr.markers.push_back(m);
  }
  if (core.tracker) {
    for (const Track & tr : core.tracker->tracks) {
      const Box & b = tr.box;
      std::array<double, 4> col = tr.confirmed ? std::array<double, 4>{1.0, 0.1, 0.1, 1.0} :
        std::array<double, 4>{1.0, 0.6, 0.0, 0.8};
      Marker m = mk("obstacles", tr.id, Marker::LINE_STRIP, stamp, col, {0.03, 0, 0});
      rect(m, f, b.s0, b.s1, b.d0, b.d1);
      arr.markers.push_back(m);
      Marker txt = mk("obstacle_label", tr.id, Marker::TEXT_VIEW_FACING, stamp, {1, 1, 1, 1}, {0, 0, 0.06});
      txt.pose.position = pt(f, b.cs(), b.cd(), 0.25);
      txt.text = "#" + std::to_string(tr.id) + " " + fmt("%.2fx%.2f", b.s1 - b.s0, b.d1 - b.d0);
      arr.markers.push_back(txt);
    }
  }

  // Gap 후보·노드
  const PlanDiag & last = core.planner.last;
  Marker gm = mk("gaps", 0, Marker::LINE_LIST, stamp, {0.1, 0.9, 0.2, 0.9}, {0.02, 0, 0});
  for (auto & [sk, gaps] : last.gaps) {
    for (auto & [a, b] : gaps) {
      gm.points.push_back(pt(f, sk, a, 0.01));
      gm.points.push_back(pt(f, sk, b, 0.01));
    }
  }
  arr.markers.push_back(gm);
  Marker nm = mk("gap_nodes", 0, Marker::POINTS, stamp, {0.2, 0.4, 1.0, 1.0}, {0.03, 0.03, 0});
  for (auto & [sk, nodes] : last.layers) {
    for (double d : nodes) {nm.points.push_back(pt(f, sk, d, 0.015));}
  }
  arr.markers.push_back(nm);

  // 후보 경로(통과한 간선, 회색)
  Marker cm = mk("candidates", 0, Marker::LINE_LIST, stamp, {0.6, 0.6, 0.6, 0.5}, {0.008, 0, 0});
  for (size_t c = 0; c < last.cand.size() && c < 40; ++c) {
    const auto & pl = last.cand[c];
    for (size_t q = 0; q + 1 < pl.size(); ++q) {
      cm.points.push_back(pt(f, pl[q].s, pl[q].d, 0.02));
      cm.points.push_back(pt(f, pl[q + 1].s, pl[q + 1].d, 0.02));
    }
  }
  arr.markers.push_back(cm);

  // 선택 경로(진한 분홍)와 검증 끝
  const PathPtr path = core.path_;
  if (path) {
    Marker pm = mk("path", 0, Marker::LINE_STRIP, stamp, {1.0, 0.2, 0.6, 1.0}, {0.035, 0, 0});
    for (size_t q = 0; q < path->s.size(); ++q) {pm.points.push_back(pt(f, path->s[q], path->d[q], 0.04));}
    arr.markers.push_back(pm);
    if (std::isfinite(core.verified_s)) {
      Marker vm = mk("verified_end", 0, Marker::SPHERE, stamp, {1.0, 1.0, 0.0, 0.9}, {0.05, 0.05, 0.05});
      const double dv = core.verified_s <= path->end_s() ? path->d_at(core.verified_s) : path->d.back();
      vm.pose.position = pt(f, core.verified_s, dv, 0.05);
      arr.markers.push_back(vm);
    }
  }

  // 차체, 목표점(추종 기준점), LFD(곡률 미리보기 점)
  const auto veh = core.vehicle(t);
  if (veh) {
    Marker em = mk("ego", 0, Marker::LINE_STRIP, stamp, {0.3, 0.6, 1.0, 1.0}, {0.015, 0, 0});
    const double c = std::cos(veh->psi), sn = std::sin(veh->psi), wh = p.body_width / 2;
    const double corners[5][2] = {{p.body_front, wh}, {p.body_front, -wh}, {-p.body_rear, -wh}, {-p.body_rear, wh},
      {p.body_front, wh}};
    for (auto & q : corners) {
      em.points.push_back(pt(f, veh->s + q[0] * c - q[1] * sn, veh->d + q[0] * sn + q[1] * c, 0.05));
    }
    arr.markers.push_back(em);
    const CtrlInfo & info = core.last_info;
    if (info.valid) {
      Marker tm = mk("target", 0, Marker::SPHERE, stamp, {0.1, 0.3, 1.0, 1.0}, {0.05, 0.05, 0.05});
      tm.pose.position = pt(f, info.ref_s, info.ref_d, 0.05);
      arr.markers.push_back(tm);
      if (path) {
        const double sf = std::min(std::max(info.s_ff, path->s.front()), path->s.back());
        const double df = interp(sf, path->s, path->d);
        Marker lm = mk("lookahead", 0, Marker::LINE_STRIP, stamp, {1.0, 0.6, 0.0, 1.0}, {0.012, 0, 0});
        lm.points = {pt(f, veh->s, veh->d, 0.06), pt(f, sf, df, 0.06)};
        arr.markers.push_back(lm);
        Marker pm2 = mk("lookahead", 1, Marker::SPHERE, stamp, {1.0, 0.6, 0.0, 1.0}, {0.045, 0.045, 0.045});
        pm2.pose.position = pt(f, sf, df, 0.06);
        arr.markers.push_back(pm2);
        Marker lt = mk("lookahead", 2, Marker::TEXT_VIEW_FACING, stamp, {1.0, 0.7, 0.2, 1.0}, {0, 0, 0.06});
        lt.pose.position = pt(f, sf, df, 0.15);
        lt.text = fmt("LFD %.2f m (k_ff %+.2f)", std::hypot(sf - veh->s, df - veh->d), info.k_ff);
        arr.markers.push_back(lt);
      }
    }
  }

  // 도로 경계: 관측된 노란 실선(노랑) / 사전 경계(회색)
  const Boundary * bd = core.boundary();
  const int NS = 120;
  std::vector<double> ss(NS), rr(NS), ll(NS);
  for (int k = 0; k < NS; ++k) {
    ss[k] = bd->s_min + (bd->s_max - bd->s_min) * k / (NS - 1);
    bd->limits(ss[k], rr[k], ll[k]);
  }
  for (int side : {-1, 1}) {
    const auto & dd = side < 0 ? rr : ll;
    Marker ym = mk("lane_boundary_observed", side + 1, Marker::LINE_LIST, stamp, {1.0, 0.85, 0.0, 1.0}, {0.02, 0, 0});
    Marker pm = mk("lane_boundary_prior", side + 1, Marker::LINE_LIST, stamp, {0.55, 0.55, 0.55, 0.7},
        {0.01, 0, 0});
    for (int k = 0; k + 1 < NS; ++k) {
      const bool kn = bd->has_known() && bd->known(ss[k], side) && bd->known(ss[k + 1], side);
      Marker & m = kn ? ym : pm;
      m.points.push_back(pt(f, ss[k], dd[k], 0.01));
      m.points.push_back(pt(f, ss[k + 1], dd[k + 1], 0.01));
    }
    arr.markers.push_back(ym);
    arr.markers.push_back(pm);
  }
  for(int color=0;color<2;++color) {
    auto raw=mk(color?"white_observed":"yellow_observed",0,Marker::POINTS,stamp,
      color?std::array<double,4>{0.95,0.95,1,0.7}:std::array<double,4>{1,0.8,0,0.7},{0.015,0.015,0});
    const auto & points=color?core.white_observed:core.yellow_observed;
    for(const auto & q:points) {Point a;a.x=q.x;a.y=q.y;a.z=0.01;raw.points.push_back(a);}
    arr.markers.push_back(raw);
  }
  if (!core.white_cells.empty()) {
    Marker wm = mk("lane_white_cells", 0, Marker::POINTS, stamp, {0.9, 0.9, 1.0, 0.8}, {0.02, 0.02, 0});
    for (size_t q = 0; q < core.white_cells.size(); ++q) {
      Point a; a.x = core.white_cells[q].x; a.y = core.white_cells[q].y; a.z = 0.005;
      wm.points.push_back(a);
    }
    arr.markers.push_back(wm);
  }
  if (core.ymap) {
    const std::vector<XY> cells = core.has_lane_cells ? core.lane_cells : core.ymap->confirmed();
    Marker lm = mk("lane_yellow_cells", 0, Marker::POINTS, stamp, {1.0, 0.5, 0.0, 0.9}, {0.03, 0.03, 0});
    for (size_t q = 0; q < cells.size(); ++q) {
      Point a; a.x = cells[q].x; a.y = cells[q].y; a.z = 0.005;
      lm.points.push_back(a);
    }
    arr.markers.push_back(lm);
  }
  // 갈림길: 섬(가상 장애물)과 정해진 방향
  if (p.fork_enable) {  // 표지판 받침(LiDAR)과 막은 쪽
    if (const auto pb = core.sign_post()) {
      Marker sp = mk("fork_sign", 0, Marker::LINE_STRIP, stamp, {0.0, 0.8, 1.0, 1.0}, {0.03, 0, 0});
      rect(sp, f, pb->s0, pb->s1, pb->d0, pb->d1, 0.06);
      arr.markers.push_back(sp);
      Marker st2 = mk("fork_sign", 1, Marker::TEXT_VIEW_FACING, stamp, {0.0, 0.8, 1.0, 1.0}, {0, 0, 0.07});
      st2.pose.position = pt(f, pb->cs(), pb->cd(), 0.35);
      st2.text = "표지판 받침";
      arr.markers.push_back(st2);
    }
  }
  if (!p.fork_shape && p.fork_enable && core.island_known() && core.fork_side() != 0 && core.sign_post()) {
    const auto pb = core.sign_post();
    const int side = core.fork_side();
    Marker wm2 = mk("fork_island", 0, Marker::LINE_STRIP, stamp, {0.6, 0.3, 1.0, 1.0}, {0.025, 0, 0});
    const double s0 = pb->s0, s1 = pb->s0 + p.fork_island_len;
    rect(wm2, f, s0, s1, side > 0 ? -0.8 : pb->d0, side > 0 ? pb->d1 : 0.8, 0.03);
    arr.markers.push_back(wm2);
    Marker it2 = mk("fork_island", 1, Marker::TEXT_VIEW_FACING, stamp, {0.6, 0.3, 1.0, 1.0}, {0, 0, 0.08});
    it2.pose.position = pt(f, s0, side > 0 ? 0.5 : -0.5, 0.3);
    it2.text = std::string(side > 0 ? "A (왼쪽)" : "B (오른쪽)") + " 받침 기준 L" + std::to_string(core.fork_votes_l) +
      " R" + std::to_string(core.fork_votes_r) + (core.fork_locked ? " 고정" : "");
    arr.markers.push_back(it2);
  } else if (!p.fork_shape && p.fork_enable && core.island_known()) {
    const Box ib = core.island_box();
    const int side = core.fork_side();
    std::array<double, 4> col = side == 0 ? std::array<double, 4>{0.8, 0.8, 0.8, 0.9} :
      std::array<double, 4>{0.6, 0.3, 1.0, 1.0};
    Marker im = mk("fork_island", 0, Marker::LINE_STRIP, stamp, col, {0.025, 0, 0});
    rect(im, f, ib.s0, ib.s1, ib.d0, ib.d1, 0.03);
    arr.markers.push_back(im);
    Marker it = mk("fork_island", 1, Marker::TEXT_VIEW_FACING, stamp, col, {0, 0, 0.08});
    it.pose.position = pt(f, ib.s0, side > 0 ? 0.5 : (side < 0 ? -0.5 : 0.0), 0.3);
    it.text = side > 0 ? "A (왼쪽)" : (side < 0 ? "B (오른쪽)" : "표지판 대기");
    it.text += " L" + std::to_string(core.fork_votes_l) + " R" + std::to_string(core.fork_votes_r) +
      (core.fork_locked ? " 고정" : "");
    arr.markers.push_back(it);
  }
  if(p.road_outline) {
    RoadOutline outline(p);
    auto model=mk("road_outline",0,Marker::LINE_LIST,stamp,{0.2,0.8,1,0.8});
    for(const auto & e:outline.edges) {model.points.push_back(pt(f,e.first.x,e.first.y));model.points.push_back(pt(f,e.second.x,e.second.y));}
    arr.markers.push_back(model);
    const std::array<std::string,3> names{"road_match_points","island_match_points","unmatched_points"};
    const std::array<std::array<double,4>,3> colors{{{0.2,0.8,1,1},{0.6,0.3,1,1},{0.5,0.5,0.5,0.7}}};
    const std::vector<XY> * sets[]={&core.road_match_points,&core.island_match_points,&core.unmatched_points};
    for(int k=0;k<3;++k) {
      auto evidence=mk(names[k],0,Marker::POINTS,stamp,colors[k],{0.025,0.025,0});
      for(const auto & q:*sets[k]) evidence.points.push_back(pt(f,q.x,q.y,0.04));
      arr.markers.push_back(evidence);
    }
  }
  if(p.fork_shape && core.navigation_island().pose.valid) {
    const auto sh=core.navigation_island();
    auto polygon=mk("fork_island",0,Marker::LINE_STRIP,stamp,
      sh.pose.confirmed?std::array<double,4>{0.6,0.3,1,1}:std::array<double,4>{0.8,0.8,0.8,1});
    for(const auto & q:sh.vertices()) polygon.points.push_back(pt(f,q.x,q.y));
    polygon.points.push_back(polygon.points.front());arr.markers.push_back(polygon);
    if(core.searching && core.path_) {
      auto search=mk("fork_observation",0,Marker::LINE_STRIP,stamp,{1,0.65,0,1},{0.045,0,0});
      for(size_t i=0;i<core.path_->s.size();++i) search.points.push_back(pt(f,core.path_->s[i],core.path_->d[i]));
      arr.markers.push_back(search);
    }
    const int side=core.fork_side();
    if(side) {
      auto goal=mk("fork_goal",0,Marker::SPHERE,stamp,{0.3,1,0.5,1},{0.1,0.1,0.1});
      const auto target=core.fork_goal();
      if(target) {goal.pose.position=pt(f,target->x,target->y,0.05);arr.markers.push_back(goal);}
    }
  }
  Marker sm = mk("section_end", 0, Marker::LINE_LIST, stamp, {1.0, 0.3, 1.0, 0.8}, {0.01, 0, 0});
  sm.points = {pt(f, p.section_end, -0.8, 0.01), pt(f, p.section_end, 0.8, 0.01)};
  arr.markers.push_back(sm);

  // 상태·정지 원인
  Marker st = mk("state", 0, Marker::TEXT_VIEW_FACING, stamp, {1, 1, 1, 1}, {0, 0, 0.09});
  if (veh) {st.pose.position = pt(f, veh->s, veh->d + 0.35, 0.4);}
  st.text = std::string(state_name(core.state)) + ": " + core.reason + "  replan=" + std::to_string(core.replans) +
    " passed=" + std::to_string(core.tracker ? core.tracker->passed : 0);
  if (core.ymap) {st.text += std::string("  lane=") + (core.aligned ? "aligned" : "unaligned");}
  if (!core.done_reason.empty()) {st.text += "  DONE(" + core.done_reason + ")";}
  if (p.fork_enable) {
    const int side = core.fork_side();
    st.text += std::string("  fork=") + (side > 0 ? "A" : (side < 0 ? "B" : "?"));
  }
  st.text += "  [cpp]";
  arr.markers.push_back(st);
  return arr;
}

MarkerArray preview_markers(const ObjAvoidCore & core, const std::vector<float> & ranges, double angle_min,
  double angle_inc, double x, double y, double yaw, const builtin_interfaces::msg::Time & stamp)
{
  const Params & p = core.p;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const LidarPose lp{x + c * p.lidar_x - s * p.lidar_y, y + s * p.lidar_x + c * p.lidar_y, yaw + p.lidar_yaw};
  const RoadFrame f(x, y, yaw);
  const PriorBoundary prior(p.boundary_s, p.boundary_left, p.boundary_right);
  std::vector<Box> boxes;
  std::vector<Pt2> pts;
  detect(ranges, angle_min, angle_inc, lp, f, prior, 0.0, p, boxes, pts);
  MarkerArray arr;
  Marker clear;
  clear.action = Marker::DELETEALL;
  arr.markers.push_back(clear);
  Marker pm = mk("lidar_roi_points", 0, Marker::POINTS, stamp, {1.0, 0.3, 0.8, 1.0}, {0.025, 0.025, 0});
  for (const Pt2 & q : pts) {pm.points.push_back(pt(f, q.s, q.d, 0.06));}
  arr.markers.push_back(pm);
  for (size_t k = 0; k < boxes.size(); ++k) {
    const Box & b = boxes[k];
    Marker dm = mk("scan_detections", static_cast<int>(k), Marker::LINE_STRIP, stamp, {0.2, 1.0, 0.3, 0.9},
        {0.015, 0, 0});
    rect(dm, f, b.s0, b.s1, b.d0, b.d1, 0.05);
    arr.markers.push_back(dm);
    Marker txt = mk("obstacle_label", static_cast<int>(k), Marker::TEXT_VIEW_FACING, stamp, {1, 1, 1, 1},
        {0, 0, 0.06});
    txt.pose.position = pt(f, b.cs(), b.cd(), 0.25);
    txt.text = fmt("%.2fx%.2f", b.s1 - b.s0, b.d1 - b.d0);
    arr.markers.push_back(txt);
  }
  Marker st = mk("state", 0, Marker::TEXT_VIEW_FACING, stamp, {1, 1, 1, 1}, {0, 0, 0.09});
  st.pose.position = pt(f, 0.0, 0.35, 0.4);
  st.text = "IDLE (미리보기): 검출 " + std::to_string(boxes.size()) + "개 — 활성화하면 추적·경로 계획 [cpp]";
  arr.markers.push_back(st);
  return arr;
}

void to_base(MarkerArray & arr, double x0, double y0, double yaw, const std::string & frame)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  auto tf = [&](Point & q) {
      const double dx = q.x - x0, dy = q.y - y0;
      q.x = c * dx + s * dy;
      q.y = -s * dx + c * dy;
    };
  for (Marker & m : arr.markers) {
    m.header.frame_id = frame;
    for (Point & q : m.points) {tf(q);}
    if (m.type == Marker::SPHERE || m.type == Marker::TEXT_VIEW_FACING) {tf(m.pose.position);}
  }
}

}  // namespace objavoid
