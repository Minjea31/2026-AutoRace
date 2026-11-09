#include "autorace_lane_fusion/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace lane_lite::geo {
float length(cv::Point2f v) { return std::sqrt(v.dot(v)); }
cv::Point2f unit(cv::Point2f v, cv::Point2f fallback) {
  const float n = length(v);
  return n > 1e-6f ? v * (1.f / n) : fallback;
}
cv::Point2f tangent(const Path &p, size_t i) {
  return unit(p[std::min(i + 1, p.size() - 1)] - p[i ? i - 1 : 0]);
}
size_t nearest(const Path &p, cv::Point2f q, float *distance) {
  size_t best = 0;
  float d = 1e9f;
  for (size_t i = 0; i < p.size(); ++i)
    if (const float e = length(p[i] - q); e < d)
      d = e, best = i;
  if (distance)
    *distance = d;
  return best;
}
float sideOf(const Path &p, cv::Point2f q) {
  const size_t i = nearest(p, q);
  return (q - p[i]).dot(rightOf(tangent(p, i)));
}
float pathLength(const Path &p) {
  float s = 0;
  for (size_t i = 1; i < p.size(); ++i)
    s += length(p[i] - p[i - 1]);
  return s;
}
Path offset(const Path &p, float distance) {
  Path out;
  for (size_t i = 0; i < p.size(); ++i)
    out.push_back(p[i] + rightOf(tangent(p, i)) * distance);
  return out;
}
std::vector<float> separation(const Path &a, const Path &b) {
  std::vector<float> out;
  for (size_t i = 0; i < a.size(); ++i) {
    float d;
    const size_t j = nearest(b, a[i], &d);
    if (interior(b, j))
      out.push_back((b[j] - a[i]).dot(rightOf(tangent(a, i))) >= 0 ? d : -d);
  }
  return out;
}
float median(std::vector<float> v) {
  if (v.empty())
    return NAN;
  std::nth_element(v.begin(), v.begin() + long(v.size() / 2), v.end());
  return v[v.size() / 2];
}
float distanceTo(const Path &a, const Path &b) {
  auto s = separation(a, b);
  for (auto &v : s)
    v = std::abs(v);
  if (!s.empty())
    return median(s);
  float d;
  nearest(b, a.front(), &d);
  return d;
}
static cv::Point2f nearDirection(const Path &p) {
  return unit(p[std::max<size_t>(1, p.size() / 3)] - p.front());
}
float angleBetween(const Path &a, const Path &b) {
  std::vector<float> out;
  for (size_t i = 0; i < a.size(); ++i) {
    const size_t j = nearest(b, a[i]);
    if (interior(b, j))
      out.push_back(std::acos(std::clamp(tangent(a, i).dot(tangent(b, j)), -1.f, 1.f)));
  }
  if (out.size() < 3)
    out = {std::acos(std::clamp(nearDirection(a).dot(nearDirection(b)), -1.f, 1.f))};
  return median(out) * float(180 / CV_PI);
}
Path blend(const Path &fresh, const Path &old, double alpha, float gate) {
  Path out = fresh;
  for (auto &p : out) {
    float d;
    const size_t j = nearest(old, p, &d);
    if (d < gate && interior(old, j))
      p += (old[j] - p) * float(1 - alpha);
  }
  return out;
}
cv::Point2f closestPoint(const Path &p, cv::Point2f q) {
  cv::Point2f best = p.front();
  float d = length(q - best);
  for (size_t j = 1; j < p.size(); ++j) {
    const cv::Point2f a = p[j - 1], e = p[j] - a;
    const float u = std::clamp((q - a).dot(e) / std::max(1e-6f, e.dot(e)), 0.f, 1.f);
    const cv::Point2f x = a + e * u;
    if (length(q - x) < d)
      d = length(q - x), best = x;
  }
  return best;
}
bool transverse(const Path &p) { return p.size() > 1 && std::abs(nearDirection(p).y) < .5f; }

// One point per base vertex: the midpoint where the base normal meets the
// other boundary (corresponding points across the lane, also in a bend); else
// the midpoint to the closest other point at a plausible width; else the
// normal offset by half the nearest measured (or learnt) width, capped at 80 %
// of the local radius on the inside of a bend so it never folds.
Path centreLine(const Path &base, const Path &other, float sign, float width, float tol) {
  const size_t n = base.size();
  std::vector<float> across(n, NAN);
  std::vector<cv::Point2f> mid(n);
  std::vector<int> kind(n, 3);
  for (size_t i = 0; i < n; ++i) {
    const cv::Point2f p = base[i], d = rightOf(tangent(base, i)) * sign;
    for (size_t j = 1; j < other.size(); ++j) {
      const cv::Point2f a = other[j - 1], e = other[j] - a, q = a - p;
      const float den = d.x * -e.y - d.y * -e.x;
      if (std::abs(den) < 1e-6f)
        continue;
      const float t = (q.x * -e.y - q.y * -e.x) / den, u = (d.x * q.y - d.y * q.x) / den;
      if (u >= 0 && u <= 1 && t > width * (1 - tol) && t < width * (1 + tol) &&
          (std::isnan(across[i]) || t < across[i]))
        across[i] = t;
    }
    if (!std::isnan(across[i])) {
      kind[i] = 1, mid[i] = p + d * (.5f * across[i]);
    } else if (!other.empty()) {
      const cv::Point2f q = closestPoint(other, p);
      const float w = length(q - p);
      if (w > width * (1 - tol) && w < width * (1 + tol) && (q - p).dot(d) > 0)
        kind[i] = 2, mid[i] = (p + q) * .5f, across[i] = w;
    }
  }
  for (size_t i = 0; i < n; ++i) {
    if (kind[i] != 3)
      continue;
    float w = width;
    for (size_t k = 1; k < n; ++k)
      if ((i >= k && !std::isnan(across[i - k])) || (i + k < n && !std::isnan(across[i + k]))) {
        w = i >= k && !std::isnan(across[i - k]) ? across[i - k] : across[i + k];
        break;
      }
    float half = .5f * w;
    const size_t a = i > 3 ? i - 3 : 0, b = std::min(i + 3, n - 1);
    const cv::Point2f ta = tangent(base, a), tb = tangent(base, b);
    const float turn = std::atan2(ta.x * tb.y - ta.y * tb.x, ta.dot(tb)) * sign;
    float arc = 0;
    for (size_t k = a; k < b; ++k)
      arc += length(base[k + 1] - base[k]);
    if (turn > 1e-3f)
      half = std::min(half, .8f * arc / turn);
    mid[i] = base[i] + rightOf(tangent(base, i)) * (sign * half);
  }
  Path out;
  for (size_t i = 0; i < n; ++i) {
    cv::Point2f sum;
    int count = 0;
    for (size_t k = i > 2 ? i - 2 : 0; k < std::min(n, i + 3); ++k)
      sum += mid[k], ++count;
    const cv::Point2f c = sum * (1.f / count);
    if (out.empty() || (c - out.back()).dot(tangent(base, i)) > 0)
      out.push_back(c);
  }
  return out;
}

bool fitPath(const Path &points, const std::vector<float> &weights_in, double max_residual, Path &out,
             double &rms) {
  Path pts = points;
  auto weights = weights_in;
  auto power = [](double t, int degree, double *row) {
    for (int k = 0; k <= degree; ++k)
      row[k] = k ? row[k - 1] * t : 1;
  };
  cv::Mat cx, cy;
  int degree = 0;
  double arc = 0;
  rms = 1e9;
  for (int attempt = 0; pts.size() >= 3; ++attempt) {
    const int n = int(pts.size());
    std::vector<double> tau(n, 0);
    for (int i = 1; i < n; ++i)
      tau[i] = tau[i - 1] + length(pts[i] - pts[i - 1]);
    arc = tau.back();
    if (arc < 8)
      break;
    for (auto &t : tau)
      t = 2 * t / arc - 1;
    degree = std::min(n - 1, n <= 6 ? 2 : 3);
    cv::Mat a(n, degree + 1, CV_64F), bx(n, 1, CV_64F), by(n, 1, CV_64F);
    for (int i = 0; i < n; ++i) {
      const double w = std::sqrt(weights[i]);
      double *row = a.ptr<double>(i);
      power(tau[i], degree, row);
      for (int k = 0; k <= degree; ++k)
        row[k] *= w;
      bx.at<double>(i) = w * pts[i].x;
      by.at<double>(i) = w * pts[i].y;
    }
    if (!cv::solve(a, bx, cx, cv::DECOMP_QR) || !cv::solve(a, by, cy, cv::DECOMP_QR))
      return false;
    double sum = 0, worst_e = -1, row[6];
    size_t worst = 0;
    for (int i = 0; i < n; ++i) {
      power(tau[i], degree, row);
      double x = 0, y = 0;
      for (int k = 0; k <= degree; ++k)
        x += cx.at<double>(k) * row[k], y += cy.at<double>(k) * row[k];
      const double e = std::hypot(pts[i].x - x, pts[i].y - y);
      sum += e * e;
      if (e > worst_e)
        worst_e = e, worst = size_t(i);
    }
    rms = std::sqrt(sum / n);
    if (rms <= max_residual)
      break;
    const size_t drop = attempt == 0 ? worst : pts.size() - 1;
    pts.erase(pts.begin() + long(drop));
    weights.erase(weights.begin() + long(drop));
  }
  if (rms > max_residual)
    return false;
  const int samples = std::max(8, std::min(64, int(arc / 15) + 1));
  double row[6];
  out.clear();
  for (int k = 0; k <= samples; ++k) {
    power(2. * k / samples - 1, degree, row);
    double x = 0, y = 0;
    for (int j = 0; j <= degree; ++j)
      x += cx.at<double>(j) * row[j], y += cy.at<double>(j) * row[j];
    out.emplace_back(float(x), float(y));
  }
  return true;
}

double circleCurvature(const std::vector<cv::Point2d> &p) {
  if (p.size() < 5)
    return 0;
  const auto a = p.front(), b = p.back();
  const double chord = cv::norm(b - a);
  if (chord < .1)
    return 0;
  double bulge = 0, side = 0;
  for (const auto &q : p) {
    const double c = ((b - a).x * (q - a).y - (b - a).y * (q - a).x) / chord;
    if (std::abs(c) > std::abs(bulge))
      bulge = c;
    side += c;
  }
  if (std::abs(bulge) < .003)
    return 0;
  cv::Mat A(int(p.size()), 3, CV_64F), r(int(p.size()), 1, CV_64F), s;
  for (int i = 0; i < int(p.size()); ++i) {
    A.at<double>(i, 0) = p[i].x, A.at<double>(i, 1) = p[i].y, A.at<double>(i, 2) = 1;
    r.at<double>(i) = -(p[i].x * p[i].x + p[i].y * p[i].y);
  }
  if (!cv::solve(A, r, s, cv::DECOMP_SVD))
    return 0;
  const double D = s.at<double>(0), E = s.at<double>(1), F = s.at<double>(2);
  const double r2 = D * D / 4 + E * E / 4 - F;
  if (!(r2 > 0))
    return 0;
  // Points left of the chord (positive cross) bulge left: a right turn.
  return (side > 0 ? -1 : 1) / std::sqrt(r2);
}
void drawPath(cv::Mat &image, const Path &p, cv::Scalar color, int thickness, bool dashed) {
  for (size_t i = 1; i < p.size(); ++i)
    if (!dashed || (i / 2) % 2 == 0)
      cv::line(image, p[i - 1], p[i], color, thickness, cv::LINE_AA);
}
} // namespace lane_lite::geo
