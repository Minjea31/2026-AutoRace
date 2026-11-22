#include "cone_lane_follower/core.hpp"
#include <cassert>
using namespace cone;
int main(){
 Path l{{.1,.3,0},{.7,.3,0}},r{{.1,-.3,0},{.7,-.3,0}};
 auto pairs=match_cross_sections(l,r,.05,.1,1.5);Path centers;
 for(auto&p:pairs)centers.push_back(lerp(p.first,p.second,.5));
 auto path=smooth_center_path(centers,l,r,.25,.3,75);
 assert(path.size()>=2);
 assert(unified_path(l,{}, {},{},.6,CENTER_DEFAULTS).size()>=2);
 assert(unified_path(l,{}, {},{},std::nullopt,CENTER_DEFAULTS).empty());
 assert(unified_path({l[0]}, {},{},{},.6,CENTER_DEFAULTS).empty());
}
