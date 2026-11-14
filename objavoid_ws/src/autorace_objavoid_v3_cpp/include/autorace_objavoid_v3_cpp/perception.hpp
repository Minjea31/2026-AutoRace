// LiDAR 장애물 검출·추적과 관측 격자(Python perception.py).
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include "autorace_objavoid_v3_cpp/params.hpp"
#include "autorace_objavoid_v3_cpp/road.hpp"

namespace objavoid {

struct Box
{
  double s0 = 0, s1 = 0, d0 = 0, d1 = 0;
  double cs() const {return 0.5 * (s0 + s1);}
  double cd() const {return 0.5 * (d0 + d1);}
  Box unite(const Box & o) const
  {
    return {std::min(s0, o.s0), std::max(s1, o.s1), std::min(d0, o.d0), std::max(d1, o.d1)};
  }
  double gap_to(const Box & o, double shrink = 0.02) const
  {
    const double ds = std::max({o.s0 - s1 + shrink, s0 - o.s1 + shrink, 0.0});
    const double dd = std::max({o.d0 - d1 + shrink, d0 - o.d1 + shrink, 0.0});
    return std::hypot(ds, dd);
  }
};

struct Track
{
  int id = 0;
  Box box;
  int hits = 0;
  double first_seen = 0, last_seen = 0;
  bool confirmed = false;
  double ghost_time = 0;
  std::vector<Box> hist;
};

struct LidarPose {double x = 0, y = 0, yaw = 0;};

struct Beam
{
  RoadFrame frame;
  LidarPose lidar;
  double angle_min = 0, angle_inc = 0, dt = 0.1;
};

struct Pt2 {double s, d;};

// 관측 범위 [lo, hi]를 min_size 이상으로. 가려진 쪽(또는 센서 반대쪽)으로 확장.
void correct_extent(double & lo, double & hi, double sensor, double min_size, bool occ_lo, bool occ_hi);

// 스캔 -> 클러스터 박스, ROI 점(도로 좌표)
void detect(const std::vector<float> & ranges, double angle_min, double angle_inc, const LidarPose & lp,
  const RoadFrame & frame, const Boundary & boundary, double s_vehicle, const Params & p,
  std::vector<Box> & boxes, std::vector<Pt2> & pts);

class Tracker
{
public:
  explicit Tracker(const Params & p) : p_(p) {}
  void update(const std::vector<Box> & boxes, double t, double s_rear, const std::vector<float> * ranges,
    const Beam * beam);
  std::vector<Box> confirmed_boxes() const;
  int confirmed_count() const;
  int section_count() const;
  std::vector<Track> tracks;
  int next_id = 1;
  int passed = 0;
  std::vector<int> passed_ids;
  int version = 0;

private:
  bool see_through(const Track & tr, const std::vector<float> & ranges, const Beam & beam) const;
  const Params & p_;
};

class VisibilityGrid
{
public:
  VisibilityGrid(const Params & p, double s_min, double s_max, double d_half = 1.0);
  void update(const std::vector<float> & ranges, double angle_min, double angle_inc, const LidarPose & lp,
    const RoadFrame & frame);
  VisibilityGrid reframed(const RoadFrame & old_f, const RoadFrame & new_f) const;
  void mark_free(double s0, double s1, double d0, double d1);
  bool is_seen(double s, double d) const;

  double res, s_min, d_min;
  int ns, nd;
  std::vector<unsigned char> seen;

private:
  const Params * p_;
  std::vector<double> steps_;
};

}  // namespace objavoid
