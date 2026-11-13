#include "cone_lane_follower/boundary_tracker.hpp"
#include <cassert>
using namespace cone;
int main(){
 Params p=BOUNDARY_DEFAULTS;p["boundary_connect_distance"]=.35;
 BoundaryTracker t;
 Path x{{.1,.3,0},{.3,.3,0},{.1,-.3,0},{.3,-.3,0}};
 t.update(x,x,1,p,.6);
 Path right{{.1,-.3,0},{.3,-.3,0}};
 auto a=t.update(right,right,1.1,p,.6);
 assert(a.first.empty()&&a.second.size()==2);
 Path fresh{{.2,.3,0},{.1,-.3,0},{.3,-.3,0}};
 auto b=t.update(fresh,fresh,1.2,p,.6);
 assert(b.first.size()==1&&b.second.size()==2);
 assert(!strict_seed_matches({.2,-.28,0},right,1,.6,.25));
 assert(!strict_seed_matches({2,.3,0},right,1,.6,.25));
 t.update({}, {},1.3,p,.6);assert(t.tracks.empty());
}
