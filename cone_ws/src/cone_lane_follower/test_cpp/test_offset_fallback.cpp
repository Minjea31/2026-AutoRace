#include "cone_lane_follower/core.hpp"
#include <cassert>
using namespace cone;
int main(){
 Path left{{.1,.3,0}},right{{.1,-.3,0},{.4,-.3,0},{.7,-.3,0}};
 Pairs pairs{{left[0],right[0]}};
 Diag d;
 auto a=unified_path(left,right,pairs,{},.6,CENTER_DEFAULTS,&d);
 assert(a.size()>=2);assert(d.count("offset_fallback"));
 for(auto&q:a)assert(std::abs(q[1])<1e-9);
 auto b=unified_path(left,right,pairs,{{.1,0,0}},.6,CENTER_DEFAULTS);
 assert(b.size()>=2);
 assert(unified_path(left,right,pairs,{},std::nullopt,CENTER_DEFAULTS).empty());
 Path conflict{{.1,1.5,0}};
 assert(unified_path(conflict,right,{{conflict[0],right[0]}},{},.6,CENTER_DEFAULTS).empty());
}
