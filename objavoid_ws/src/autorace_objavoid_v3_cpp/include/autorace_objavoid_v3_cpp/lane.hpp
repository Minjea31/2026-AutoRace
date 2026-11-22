// 카메라 노란 실선 경계(Python lane_boundary.py): 바닥 투영, odom 노란선 격자, 도로 좌표 경계, 직선 정렬.
#pragma once
#include <array>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "autorace_objavoid_v3_cpp/params.hpp"
#include "autorace_objavoid_v3_cpp/perception.hpp"
#include "autorace_objavoid_v3_cpp/road.hpp"

namespace objavoid {

struct XY {double x, y;};

// 퓨전 lane.yaml(lane_lite 섹션) BEV 정의 -> 이미지 픽셀 -> base_link 바닥 좌표
class GroundProjection
{
public:
  // cfg_path: lane.yaml. homography(9개, 모두 0이면 사용 안 함)
  GroundProjection(const std::string & cfg_path, double max_range, double max_lateral,
    const std::vector<double> & homography, bool full_image=false);
  void set_scale(double sx, double sy);
  // Deep class mask only: yellow=1, white=2. No raw image is required.
  std::vector<XY> class_points(const cv::Mat & mask, int cls, int step = 2) const;
  std::vector<XY> yellow_points(const cv::Mat & mask) const { return class_points(mask, 1); }
  std::vector<XY> white_points(const cv::Mat & mask) const { return class_points(mask, 2); }
  // Fraction of the lane BEV image (lane.yaml bev_* quad -> w x h) covered by deep class cls (2 = white).
  double class_ratio_bev(const cv::Mat & mask, int cls) const;
  int w = 0, h = 0;
  std::vector<XY> view_points;
  double near = 0, max_range = 2.5, max_lateral = 1.5;

private:
  void to_ground(double u, double v, double & x, double & y) const;
  bool to_image(double x, double y, double & u, double & v) const;
  cv::Matx33d G_, Ginv_, Hbev_;
  std::array<double, 4> roi_{0, 1, 0, 1};
  double sx_ = 1.0, sy_ = 1.0;
};

class YellowMap
{
public:
  YellowMap(const Params & p, double x0, double y0, double half = 7.0);
  static XY to_odom(const XY & pt, double x, double y, double yaw);
  void update(const std::vector<XY> & yellow_base, const std::vector<XY> & view_base, double x, double y,
    double yaw);
  std::vector<XY> confirmed() const;
  bool hit_confirmed(int i, int j) const;
  int idx_i(double x) const {return static_cast<int>((x - x0) / res);}
  int idx_j(double y) const {return static_cast<int>((y - y0) / res);}

  double res, x0, y0;
  int n;
  std::vector<int16_t> views, hits;
  int frames = 0;
  int version = 0;

private:
  const Params * p_;
};

class LaneBoundary : public Boundary
{
public:
  LaneBoundary(const PriorBoundary & prior, const Params & p);
  void limits(double s, double & r, double & l) const override;
  bool has_known() const override {return true;}
  bool known(double s, int side) const override;
  void rebuild(const std::vector<Pt2> & cells_sd);

  PriorBoundary prior;
  double bin = 0.05;
  std::vector<double> s_bins, right, left;
  std::vector<char> known_r, known_l;
  std::vector<Pt2> cells_sd;

private:
  const Params * p_;
};

struct LineSeg {double ang, len, nrm, sm, thick; int count; char kind;};
std::vector<LineSeg> line_segments(const std::vector<Pt2> & pts, int max_lines = 5, double tol = 0.02,
  int iters = 150, unsigned seed = 0, double max_gap = 0.10);

struct RoadFit {double a; std::optional<double> center; std::vector<double> lines;};
// 노란·흰 점선 칸(도로 좌표) -> 방향 보정각, 중심 보정
std::optional<RoadFit> fit_road(const std::vector<Pt2> & yellow_sd, const Params & p, double s_from, double s_to,
  const Boundary * prior, const std::vector<double> * widths, const std::vector<Pt2> & white_sd);

}  // namespace objavoid
