#pragma once
#include "autorace_objavoid_v3_cpp/island.hpp"
#include <optional>
namespace objavoid {
// Pose of the drawing in the current road frame. The island is deliberately absent.
struct OutlineFit {
  double s=0,d=0,yaw=0,error=kInf;
  int hits=0; bool valid=false,longitudinal=false;
  double inlier_ratio=0;
};
class RoadOutline : public Boundary {
public:
  explicit RoadOutline(const Params & p);
  RoadOutline(const Params & p, const IslandShape & island);
  void limits(double s, double & right, double & left) const override;
  bool has_known() const override {return true;}
  bool known(double, int) const override {return true;}
  std::vector<std::pair<XY,XY>> edges;
  double distance(const XY & q) const;
  // Independently classify evidence, including before an island has been localized.
  bool island_region(const XY & q) const;
  OutlineFit fit(const std::vector<XY> & observations, bool acquire=true, std::optional<XY> ego=std::nullopt) const;
private:
  std::optional<IslandShape> island_;
  double ramp_=0, straight_end_=0;
};
// Stable boundary address shared by perception and the planner. Confirmed island
// geometry replaces the road model, never the odometry/observation coordinates.
class IslandRoadBoundary : public Boundary {
public:
  explicit IslandRoadBoundary(const Boundary * base):base_(base) {reset();}
  std::optional<RoadOutline> model;
  void reset() {model.reset();s_min=base_->s_min;s_max=base_->s_max;}
  void update(const Params & p,const IslandShape & island) {
    if(!p.fork_shape || !p.road_outline || !island.pose.confirmed) return;
    model.emplace(p,island);s_min=model->s_min;s_max=model->s_max;
  }
  void limits(double s,double & r,double & l) const override {
    if(model) model->limits(s,r,l);else base_->limits(s,r,l);
  }
  bool has_known() const override {return model || base_->has_known();}
  bool known(double s,int side) const override {return model || base_->known(s,side);}
private:
  const Boundary * base_;
};
}  // namespace objavoid
