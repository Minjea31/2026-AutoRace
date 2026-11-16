#pragma once
#include "autorace_objavoid_v3_cpp/lane.hpp"
namespace objavoid {
// Pose of the drawing in the current road frame. The island is deliberately absent.
struct OutlineFit {
  double s=0,d=0,yaw=0,error=kInf;
  int hits=0; bool valid=false,longitudinal=false;
  double inlier_ratio=0;
};
class RoadOutline {
public:
  explicit RoadOutline(const Params & p);
  std::vector<std::pair<XY,XY>> edges;
  double distance(const XY & q) const;
  // Independently classify evidence, including before an island has been localized.
  bool island_region(const XY & q) const;
  OutlineFit fit(const std::vector<XY> & observations, bool acquire=true) const;
};
}  // namespace objavoid
