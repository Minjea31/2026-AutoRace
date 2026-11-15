#include "cone_lane_follower/core.hpp"
#include <cassert>
using namespace cone;
int main(){
 // Bag cone_lane_1114_0018 52.3 s: one cone beside the car (0.20,+0.35) -> none ahead.
 std::vector<Path> beside{{{.19,.35,0},{.21,.35,0}}};
 assert(count_front_clusters(beside,.2)==0);
 // Mid-corridor (45.1 s): cones still ahead.
 std::vector<Path> corridor{{{.12,-.40,0},{.12,-.40,0}},{{.29,-.27,0},{.29,-.27,0}},{{.47,-.03,0},{.47,-.03,0}}};
 assert(count_front_clusters(corridor,.2)==2);
 assert(count_front_clusters({},.2)==0);
 std::vector<Path> withEmpty{{}, {{.5,0,0}}};
 assert(count_front_clusters(withEmpty,.2)==1);
}
