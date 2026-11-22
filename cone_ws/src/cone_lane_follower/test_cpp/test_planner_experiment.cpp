#include "cone_lane_follower/core.hpp"
#include <iostream>
using namespace cone;
void require(bool ok,const char*why){if(!ok)throw std::runtime_error(why);}
int main(){
  auto p=CENTER_DEFAULTS;
  Path left={{0.24859951805752076,0.39033198881150988,-0.034000000000000016}};
  Path right={{0.038994930312418873,-0.2970513350968314,-0.034000000000000016},{0.2106007316537645,-0.32977447842010443,-0.034000000000000023},{0.45343451545482605,-0.33636903041752703,-0.034000000000000016},{0.69632188716757504,-0.28516185951711709,-0.034000000000000016},{0.92305827716448774,-0.10837513137507114,-0.034000000000000002},{0.99444800709406189,0.066345589425810023,-0.034000000000000002},{1.0495267005771889,0.29679346254716626,-0.034000000000000002},{1.0415458460619502,0.54420164544502314,-0.034000000000000002},{0.96173184037687331,0.73427555354936236,-0.034000000000000002},{0.84609834697717579,0.90166027182019892,-0.034000000000000002},{0.71131494022519914,1.0354289621300408,-0.034000000000000002}};
  p["center_turn_radius_check"]=1;
  Diag old_diag;
  require(single_point_midpath(left,right,p,&old_diag).empty(),"14.9s fixture must fail with old radius rejection");
  p["center_turn_radius_check"]=0;
  Diag d;
  auto path=single_point_midpath(left,right,p,&d);
  require(path.size()>=2&&d.count("single_point_midpoints"),"14.9s midpoint path must survive disabled radius rejection");
  require(peak_curvature(path)>1/p.at("center_min_turn_radius"),"Fixture must actually exceed nominal minimum radius");
  require(valid_center_geometry(path,p),"Geometry checks must still apply");
  require(offset_safe(path,{{right,1}}),"Boundary checks must still apply");
  // Synthetic outer boundary with a tighter-than-nominal inward offset.
  Path boundary;
  for(int i=0;i<=30;++i){double t=.04*i;boundary.push_back({.5*std::sin(t),.5-.5*std::cos(t),0});}
  auto tight=corner_offset(boundary,.3,.05,.2630871384,p);
  require(tight.size()>=2&&peak_curvature(tight)>1/.2630871384,"Offset path radius check must also be disabled");
  p["center_turn_radius_check"]=1;
  auto checked=corner_offset(boundary,.3,.05,.2630871384,p);
  require(checked.empty()||peak_curvature(checked)<=1/.2630871384,"Radius enforcement must be restorable");
  // 20.9s branch: the 24cm rightward candidate must beat the 25.2cm leftward branch.
  Path chain{{.0558,.4903,0},{.2387,.5607,0},{.4407,.6064,0}};
  Point farther{.6474,.7506,0},nearer{.6710,.5404,0};
  Path cones=chain;cones.push_back(farther);cones.push_back(nearer);
  auto bp=BOUNDARY_DEFAULTS;bp["boundary_connect_distance"]=.35;bp["boundary_max_turn"]=100;
  bp["boundary_direction_weight"]=2;
  auto biased=select_boundary_chain(cones,bp);
  require(biased.size()>3&&biased[3]==farther,"Original direction preference fixture");
  bp["boundary_direction_weight"]=0;
  auto nearest=select_boundary_chain(cones,bp);
  require(nearest.size()>3&&nearest[3]==nearer,"Zero direction score must choose nearer admissible point");
  // Existing turn/distance rejection remains independent of scoring.
  require(boundary_options(chain,{{1,0,0}},{Point{2,2,0}},bp,{}).empty(),"Distance gate must remain active");
  auto all=CENTER_DEFAULTS;
  for(const auto&group:{CLUSTER_DEFAULTS,BOUNDARY_DEFAULTS,CONTROL_DEFAULTS})all.insert(group.begin(),group.end());
  require(validate_parameters(all).empty(),"Zero score and disabled radius parameter must validate");
  all["boundary_direction_weight"]=-1;
  require(!validate_parameters(all).empty(),"Negative score must still be rejected");
  std::cout<<"Recorded 14.9s radius bypass, 20.9s nearest branch, restored checks and validation passed\n";
}
