#include "cone_lane_follower/boundary_tracker.hpp"
#include <iostream>
using namespace cone;
void check(bool ok, const char*why) { if (!ok) { std::cerr << why << '\n'; std::exit(1); } }
Path shifted(const Path&p, double dx, double dy) { Path out=p; for(auto&q:out){q[0]+=dx;q[1]+=dy;} return out; }
int main() {
  Params p=BOUNDARY_DEFAULTS; p["boundary_connect_distance"]=.35; p["boundary_max_turn"]=100;
  Path world{{.1,.3,0},{.1,-.3,0},{.3,-.3,0},{.5,-.3,0}};
  BoundaryTracker tracker;
  auto first=tracker.update(world,world,10,p);
  check(first.first.size()==1&&first.second.size()==3,"initial boundaries");
  auto moved=shifted(world,-.05,.5);
  auto next=tracker.update(moved,world,10.1,p);
  check(next.first.size()==1&&next.second.size()==3,"preserve sides across x/y sign changes");
  for(auto&q:next.second) check(q[1]>0,"test must cross y sign");
  Path right_world(world.begin()+1,world.end());
  auto missing=tracker.update(shifted(right_world,-.05,.5),right_world,10.2,p);
  check(missing.first.empty()&&missing.second.size()==3,"missing left must not steal right");
  auto absent=tracker.update({}, {},10.5,p);
  check(absent.first.empty()&&absent.second.empty(),"missing tracks never become output");
  auto returned=tracker.update(moved,world,11.1,p);
  check(returned.first.size()==1&&returned.second.size()==3,"identity survives exactly one second");
  tracker.update({}, {},12.101,p);
  check(tracker.tracks.empty(),"expire after one second without observation");
  tracker.update(world,world,20,p);
  tracker.update({}, {},19,p);
  check(tracker.tracks.empty(),"clock reversal clears stale identities");
  // A newly visible cone can extend its established boundary.
  tracker.clear();tracker.update(world,world,30,p);
  world.push_back({.7,-.3,0});
  auto extended=tracker.update(world,world,30.1,p);
  check(extended.second.size()==4,"new cone must extend known right boundary");
  // Once only the left identity expires, an unseen left cone may initialize
  // that side without stealing any currently matched right cone.
  tracker.clear();
  tracker.update({{.1,.3,0},{.1,-.3,0}},{{.1,.3,0},{.1,-.3,0}},40,p);
  tracker.update({{.1,-.3,0}},{{.1,-.3,0}},40.9,p);
  auto reacquired=tracker.update({{.2,.3,0},{.1,-.3,0}},{{.2,.3,0},{.1,-.3,0}},41.1,p);
  check(reacquired.first.size()==1&&reacquired.second.size()==1,"expired side can reinitialize independently");
  std::cout<<"Boundary tracking tests passed\n";
}
