#include "cone_lane_follower/boundary_tracker.hpp"
#include <cassert>
using namespace cone;
int main(){
 Params p=BOUNDARY_DEFAULTS;p["boundary_connect_distance"]=.35;
 BoundaryTracker t;
 auto identity=[](const Point&q){return q;};
 Path x{{.1,.3,0},{.3,.3,0},{.1,-.3,0},{.3,-.3,0}};
 t.update(x,x,identity,1,p,.6);
 Path right{{.1,-.3,0},{.3,-.3,0}};
 auto a=t.update(right,right,identity,1.1,p,.6);
 assert(a.first==Path({{.3,.3,0}})&&a.second.size()==2);
 Path fresh{{.2,.3,0},{.1,-.3,0},{.3,-.3,0}};
 auto b=t.update(fresh,fresh,identity,1.2,p,.6);
 assert(b.first.size()==1&&b.second.size()==2);
 assert(!strict_seed_matches({.2,-.28,0},right,1,.6,.25));
 assert(!strict_seed_matches({2,.3,0},right,1,.6,.25));
 auto held=t.update({}, {},identity,1.3,p,.6);
 assert(t.tracks.empty());
 assert(held.first==Path({{.2,.3,0}})&&held.second==Path({{.3,-.3,0}}));
}
