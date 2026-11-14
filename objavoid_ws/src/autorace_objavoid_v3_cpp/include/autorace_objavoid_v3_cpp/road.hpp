// 도로 좌표계와 주행 가능 경계(Python road.py).
// 도로 좌표 (s, d): 활성화 순간의 odom 자세를 원점으로, s = 전방, d = 왼쪽.
#pragma once
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace objavoid {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

inline double wrap(double a)
{
  double r = std::fmod(a + M_PI, 2 * M_PI);
  if (r < 0) {r += 2 * M_PI;}
  return r - M_PI;
}

// numpy.interp(x, xp, fp): 범위 밖은 양 끝값(left/right 지정 시 그 값)
inline double interp(double x, const std::vector<double> & xp, const std::vector<double> & fp,
  double left = kNaN, double right = kNaN)
{
  const size_t n = xp.size();
  if (n == 0) {return kNaN;}
  if (x < xp[0]) {return std::isnan(left) ? fp[0] : left;}
  if (x > xp[n - 1]) {return std::isnan(right) ? fp[n - 1] : right;}
  if (n == 1) {return fp[0];}
  size_t lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    size_t mid = (lo + hi) / 2;
    if (xp[mid] <= x) {lo = mid;} else {hi = mid;}
  }
  const double dx = xp[hi] - xp[lo];
  if (dx <= 0) {return fp[hi];}
  return fp[lo] + (fp[hi] - fp[lo]) * (x - xp[lo]) / dx;
}

struct RoadFrame
{
  double x0 = 0, y0 = 0, yaw0 = 0, c = 1, s = 0;
  RoadFrame() = default;
  RoadFrame(double x, double y, double yaw)
  : x0(x), y0(y), yaw0(yaw), c(std::cos(yaw)), s(std::sin(yaw)) {}
  void to_road(double x, double y, double & rs, double & rd) const
  {
    const double dx = x - x0, dy = y - y0;
    rs = c * dx + s * dy;
    rd = -s * dx + c * dy;
  }
  void to_odom(double rs, double rd, double & x, double & y) const
  {
    x = x0 + c * rs - s * rd;
    y = y0 + s * rs + c * rd;
  }
  void pose_to_road(double x, double y, double yaw, double & rs, double & rd, double & psi) const
  {
    to_road(x, y, rs, rd);
    psi = wrap(yaw - yaw0);
  }
};

// 경계 인터페이스: limits(s) -> (right, left), 관측 여부 known(s, side)
class Boundary
{
public:
  virtual ~Boundary() = default;
  virtual void limits(double s, double & right, double & left) const = 0;
  virtual bool has_known() const {return false;}
  virtual bool known(double, int) const {return false;}
  double s_min = 0, s_max = 0;
};

class PriorBoundary : public Boundary
{
public:
  PriorBoundary(std::vector<double> s, std::vector<double> l, std::vector<double> r)
  : s_pts(std::move(s)), left(std::move(l)), right(std::move(r))
  {
    if (!(s_pts.size() == left.size() && left.size() == right.size() && s_pts.size() >= 2)) {
      throw std::invalid_argument("boundary arrays must have equal length >= 2");
    }
    for (size_t i = 0; i < s_pts.size(); ++i) {
      if ((i > 0 && s_pts[i] <= s_pts[i - 1]) || left[i] <= right[i]) {
        throw std::invalid_argument("boundary s must increase and left > right");
      }
    }
    s_min = s_pts.front();
    s_max = s_pts.back();
  }
  void limits(double s, double & r, double & l) const override
  {
    r = interp(s, s_pts, right);
    l = interp(s, s_pts, left);
  }
  std::vector<double> s_pts, left, right;
};

}  // namespace objavoid
