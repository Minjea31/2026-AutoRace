#pragma once
#include <array>
#include <vector>
#include "autorace_objavoid_v3_cpp/params.hpp"
#include "autorace_objavoid_v3_cpp/lane.hpp"
namespace objavoid {
struct IslandPose {double s=0, d=0, yaw=0, error=kInf; int stable=0; bool valid=false, confirmed=false;};
// Convex conservative envelope: rounded R40 corners remain inside this polygon.
struct IslandShape {
  IslandPose pose;
  double length=1.687, width=0.5;
  std::array<XY, 6> vertices() const;
  XY local(double s, double d) const;
  double clearance(double s, double d, double yaw, const Params & p) const;
};
class IslandMapper {
public:
  IslandShape shape;
  int misses=0;
  size_t input_count=0, candidate_count=0, evaluations=0;
  const char * rejection="no_observation";
  void reset() {shape.pose=IslandPose{};misses=0;}
  bool update(const std::vector<XY> & road_points, double reference_s,
    const Params * vehicle=nullptr, double car_s=0, double car_d=0, double car_yaw=0);
};
}  // namespace objavoid
