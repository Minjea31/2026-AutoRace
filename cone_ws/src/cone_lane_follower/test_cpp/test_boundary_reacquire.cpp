#include "cone_lane_follower/boundary_tracker.hpp"
#include <iostream>
using namespace cone;
namespace {
void require(bool ok, const char*message) {
  if (!ok) throw std::runtime_error(message);
}
const auto identity = [](const Point&q) { return q; };
Params parameters() {
  auto p = BOUNDARY_DEFAULTS;
  p["boundary_connect_distance"] = .35;
  return p;
}
Path other_side(double sign) {
  return {{.1,-sign*.3,0},{.3,-sign*.3,0},{.5,-sign*.3,0},
          {.7,-sign*.3,0},{.9,-sign*.3,0}};
}
const Path& side_line(const std::pair<Path,Path>&b, double sign) {
  return sign > 0?b.first:b.second;
}
void missing_and_recover(double sign) {
  auto p = parameters();
  BoundaryTracker t;
  auto opposite = other_side(sign);
  auto first = opposite;
  first.push_back({.5,sign*.3,0});
  t.update(first,first,identity,1,p,.6);
  auto held = t.update(opposite,opposite,identity,2,p,.6);
  require(side_line(held,sign)==Path({{.5,sign*.3,0}}),
          "Missing side must keep its last endpoint");
  require(t.tracks.size()==opposite.size(),"A held endpoint must not become a detection");
  // Held singleton + opposite boundary must form ray-midpoint sections.
  auto path=unified_path(held.first,held.second,{},{},.6,CENTER_DEFAULTS);
  require(path.size()>=2,"Held endpoint should remain usable by the path planner");
  for (const auto&q:path) require(std::abs(q[1])<1e-8,"Unexpected midpoint path");
  auto fresh=opposite;
  fresh.push_back({.24,sign*.3,0}); // Closer to vehicle, but farther from saved endpoint.
  fresh.push_back({.6,sign*.3,0});
  auto b=t.update(fresh,fresh,identity,3,p,.6);
  require(side_line(b,sign).front()==Point({.6,sign*.3,0}),
          "Reacquisition must start nearest the saved endpoint on either side");
}
void hold_until_valid_candidate() {
  auto p=parameters();
  BoundaryTracker t;
  auto opposite=other_side(-1);
  auto initial=opposite;initial.push_back({.5,-.3,0});
  t.update(initial,initial,identity,1,p,.6);
  auto far=opposite;far.push_back({1.,-.3,0});
  auto b=t.update(far,far,identity,2,p,.6);
  require(b.second==Path({{.5,-.3,0}}),"Out-of-gate candidate must not replace held point");
  auto bad=opposite;bad.push_back({.51,-.05,0});
  b=t.update(bad,bad,identity,30,p,.6);
  require(b.second==Path({{.5,-.3,0}}),"Wrong-width candidate must not replace held point");
  auto fresh=opposite;fresh.push_back({.51,-.05,0});fresh.push_back({.8,-.3,0});
  b=t.update(fresh,fresh,identity,31,p,.6);
  require(b.second.front()==Point({.8,-.3,0}),"Nearest eligible candidate must win");
  require(t.last_endpoint[1]==std::optional<Point>({{.8,-.3,0}}),
          "Recovered endpoint must become the new reference");
}
void ego_motion_and_empty_scans() {
  auto p=parameters();
  BoundaryTracker t;
  Path initial{{.5,.3,0},{.5,-.3,0}};
  t.update(initial,initial,identity,1,p,.6);
  const double yaw=.45,c=std::cos(yaw),s=std::sin(yaw);
  auto to_local=[=](const Point&q) {
    double x=q[0]-.25,y=q[1]-.04;
    return Point{c*x+s*y,-s*x+c*y,q[2]};
  };
  for (int n=0;n<4;++n) {
    auto held=t.update({},{},to_local,2+n,p,.6);
    require(held.first==Path({to_local(initial[0])})&&
            held.second==Path({to_local(initial[1])}),
            "Held points must transform from odom on every scan without drifting");
    require(t.tracks.empty(),"Empty scans must not create tracked detections");
    require(unified_path(held.first,held.second,{},{},.6,CENTER_DEFAULTS).empty(),
            "Two held singletons must not invent a full path");
  }
  Path world{{.24,-.3,0},{.6,-.3,0}},local;
  for(const auto&q:world)local.push_back(to_local(q));
  auto b=t.update(local,world,to_local,6,p,.6);
  require(b.second.front()==to_local(world[1]),"Ego motion must not change nearest world seed");
  require(t.last_endpoint[1]==std::optional<Point>(world[1]),"Reference must stay in odom");
}
void reverse_scan_order() {
  auto p=parameters();
  BoundaryTracker t;
  auto left=other_side(-1);
  std::reverse(left.begin(),left.end()); // LaserScan sees the left side far-to-near.
  auto initial=left;initial.push_back({.5,-.3,0});
  t.update(initial,initial,identity,1,p,.6);
  t.update(left,left,identity,2,p,.6);
  auto fresh=left;fresh.push_back({.24,-.3,0});fresh.push_back({.6,-.3,0});
  auto b=t.update(fresh,fresh,identity,3,p,.6);
  require(b.second.front()==Point({.6,-.3,0}),"Raw scan order must not reverse width gating");
}
void singleton_midpoints() {
  Path left{{.1,.3,0},{.3,.32,0},{.5,.35,0},{.7,.39,0},{.9,.44,0}};
  Path right{{.4,-.3,0}};
  Diag d;
  auto path=unified_path(left,right,{},{},std::nullopt,CENTER_DEFAULTS,&d);
  require(path.size()>=2&&d.count("single_point_midpoints"),
          "One-point midpoint branch must run before the learned-width gate");
  require(path.front()==lerp(left.front(),right.front(),.5)&&
          xy(path.back(),lerp(left.back(),right.front(),.5))<1e-9,
          "Midpoint path endpoints must use the actual singleton, not a normal offset");
  require(path_length(path)<.5,"Singleton midpoint path should span half the opposite boundary");
  Path mirrored_left,mirrored_right;
  for(const auto&q:right)mirrored_left.push_back({q[0],-q[1],q[2]});
  for(const auto&q:left)mirrored_right.push_back({q[0],-q[1],q[2]});
  auto mirrored=unified_path(mirrored_left,mirrored_right,{},{},std::nullopt,CENTER_DEFAULTS);
  require(mirrored.size()==path.size(),"Left/right singleton logic must be symmetric");
  for(size_t i=0;i<path.size();++i)
    require(std::abs(path[i][0]-mirrored[i][0])<1e-9&&std::abs(path[i][1]+mirrored[i][1])<1e-9,
            "Mirrored singleton path mismatch");
  require(unified_path({{.1,.3,0}},right,{},{},std::nullopt,CENTER_DEFAULTS).empty(),
          "Two isolated points cannot form a path");
  require(unified_path(left,{{.4,-3.,0}},{},{},.6,CENTER_DEFAULTS).empty(),
          "Out-of-range singleton sections must fail explicitly");
}
void reset_and_fresh_bootstrap() {
  auto p=parameters();
  BoundaryTracker t;
  Path initial{{.5,.3,0},{.5,-.3,0}};
  auto empty=t.update({},{},identity,1,p,.6);
  require(empty.first.empty()&&empty.second.empty(),"No history must mean no held geometry");
  t.update(initial,initial,identity,2,p,.6);
  t.clear();
  require(!t.last_endpoint[0]&&!t.last_endpoint[1]&&t.tracks.empty()&&!t.stamp,
          "Mission reset must clear every boundary memory");
  empty=t.update({},{},identity,3,p,.6);
  require(empty.first.empty()&&empty.second.empty(),"Cleared endpoints must not reappear");
  Path fresh{{.1,1.,0},{.1,-1.,0}};
  auto b=t.update(fresh,fresh,identity,4,p);
  require(b.first.size()==1&&b.second.size()==1,"New mission must bootstrap at fresh points");
  empty=t.update({},{},identity,1,p);
  require(empty.first.empty()&&empty.second.empty(),"Backward time must reset held endpoints");
}
}
int main() {
  missing_and_recover(1);
  missing_and_recover(-1);
  hold_until_valid_candidate();
  ego_motion_and_empty_scans();
  reset_and_fresh_bootstrap();
  reverse_scan_order();
  singleton_midpoints();
  std::cout<<"Boundary reacquisition, held geometry, odom motion and resets passed\n";
}
