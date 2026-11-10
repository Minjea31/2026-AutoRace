#include "autorace_lane_fusion/candidates.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace lane_lite {
using namespace geo;
namespace {
// Keep pixels on a thin run along a row OR a column. Painted lines of any
// direction are thin across; broad glare and floor patches are thin in neither.
cv::Mat thinBands(const cv::Mat &mask, int max_width) {
  auto rows = [max_width](const cv::Mat &src) {
    cv::Mat dst = cv::Mat::zeros(src.size(), CV_8U);
    for (int y = 0; y < src.rows; ++y) {
      const uchar *s = src.ptr<uchar>(y);
      uchar *d = dst.ptr<uchar>(y);
      for (int x = 0; x < src.cols;) {
        int end = x;
        while (end < src.cols && s[end])
          ++end;
        if (end > x && end - x <= max_width)
          std::fill(d + x, d + end, 255);
        x = std::max(end, x + 1);
      }
    }
    return dst;
  };
  cv::Mat transposed, columns;
  cv::transpose(mask, transposed);
  cv::transpose(rows(transposed), columns);
  return rows(mask) | columns;
}

struct Peak {
  int x;
  bool yellow;
};
// Column histogram peaks of the lower BEV, strongest first.
std::vector<Peak> findPeaks(const cv::Mat &mask, const cv::Mat &yellow, int y0, int minimum, int separation_px) {
  cv::Mat hist, yhist;
  cv::reduce(mask.rowRange(y0, mask.rows) / 255, hist, 0, cv::REDUCE_SUM, CV_32F);
  cv::reduce(yellow.rowRange(y0, mask.rows) / 255, yhist, 0, cv::REDUCE_SUM, CV_32F);
  cv::blur(hist, hist, {9, 1});
  cv::blur(yhist, yhist, {9, 1});
  std::vector<Peak> out;
  for (int n = 0; n < 6; ++n) {
    double best;
    cv::Point at;
    cv::minMaxLoc(hist, nullptr, &best, nullptr, &at);
    if (best < minimum)
      break;
    out.push_back({at.x, yhist.at<float>(at.x) * 2 > hist.at<float>(at.x)});
    hist.colRange(std::max(0, at.x - separation_px), std::min(hist.cols, at.x + separation_px + 1)).setTo(0);
  }
  return out;
}

struct Search {
  Path centers;
  std::vector<double> weights;
  std::vector<cv::Point> pixels;
};
// Windows that step along the marking's own direction, so a boundary that
// bends from vertical to transverse is followed without a second model.
// Only observed window centres are kept; missed windows are never fitted.
// (lane_ws autorace_lane_lite, unchanged apart from the display trace.)
Search slide(const cv::Mat &mask, cv::Point2f start, cv::Point2f dir, const Path &prior, const Config &c) {
  const int w = mask.cols, h = mask.rows;
  const float step = std::max(4, h / int(c["windows"])), margin = c["margin"];
  const float band = c["line_width_max"];
  const size_t min_pixels = c["min_pixels"];
  Search s;
  cv::Point2f p = start - dir * (step * .5f);
  int misses = 0;
  std::vector<cv::Point> found, inside, chosen;
  std::vector<float> offsets;
  for (int i = 0; i < 2 * int(c["windows"]); ++i) {
    if (!prior.empty())
      dir = unit(dir + tangent(prior, nearest(prior, p)), dir);
    const cv::Point2f q = p + dir * step, across = rightOf(dir);
    if (q.x < -margin || q.x > w - 1 + margin || q.y < -step || q.y > h - 1 + step)
      break; // the boundary left the BEV
    const cv::RotatedRect box(q, {2 * margin, step}, float(std::atan2(across.y, across.x) * 180 / CV_PI));
    const cv::Rect bounds = box.boundingRect() & cv::Rect(0, 0, w, h);
    found.clear(), inside.clear(), offsets.clear(), chosen.clear();
    if (!bounds.empty())
      cv::findNonZero(mask(bounds), found);
    cv::Mat hist = cv::Mat::zeros(1, 2 * int(margin) + 1, CV_32F), smooth;
    for (auto pt : found) {
      pt += bounds.tl();
      const cv::Point2f v = cv::Point2f(pt) - q;
      const float o = v.dot(across);
      if (std::abs(v.dot(dir)) <= step * .5f && std::abs(o) <= margin) {
        inside.push_back(pt);
        offsets.push_back(o);
        hist.at<float>(cvRound(o + margin)) += 1;
      }
    }
    // The cross-track peak closest to the prediction picks one marking; a
    // stop line or a second marking in the window is not averaged in.
    cv::blur(hist, smooth, {5, 1});
    double largest;
    cv::minMaxLoc(smooth, nullptr, &largest);
    int peak = -1;
    for (int b = 0; b < smooth.cols && largest > 0; ++b)
      if (smooth.at<float>(b) * 3 >= largest && (peak < 0 || std::abs(b - margin) < std::abs(peak - margin)))
        peak = b;
    bool hit = false;
    if (peak >= 0) {
      cv::Point2d sum;
      for (size_t k = 0; k < inside.size(); ++k)
        if (std::abs(offsets[k] - (peak - margin)) <= band) {
          chosen.push_back(inside[k]);
          sum += cv::Point2d(inside[k]);
        }
      if (chosen.size() >= min_pixels) {
        const cv::Point2f center(sum * (1. / chosen.size()));
        const cv::Point2f turn = s.centers.empty() ? dir : unit(center - s.centers.back(), dir);
        if (turn.dot(dir) > .5f) { // at most 60 deg per window: no jump to another line
          dir = unit(dir + turn, dir);
          s.centers.push_back(center);
          s.weights.push_back(double(chosen.size()));
          s.pixels.insert(s.pixels.end(), chosen.begin(), chosen.end());
          p = center;
          hit = true;
        }
      }
    }
    if (!hit) {
      p = q;
      // With a track, keep looking for a boundary that starts later (inner
      // side of a bend, dashed line); otherwise stop after a gap.
      if (++misses >= c["max_misses"] && !(!prior.empty() && s.centers.empty()))
        break;
    } else {
      misses = 0;
    }
  }
  return s;
}
// Cubic s(t) across the chord first->last centre (any orientation), with a
// ridge on the bend terms that is strong only for short observations. If it
// still does not fit, one outlier window is dropped, then the far end is
// trimmed: the near part matters most for control. Never extrapolates.
bool fitLine(const Search &s, const Config &c, int h, Candidate &out, std::string &note) {
  out.observed = int(s.centers.size());
  if (out.observed < c["min_windows"]) {
    note = "관측 윈도우 " + std::to_string(out.observed) + "개 < min_windows";
    return false;
  }
  Path pts = s.centers;
  auto weights = s.weights;
  cv::Mat coef;
  cv::Point2f origin, along, across;
  float chord = 0;
  double rms = 1e9;
  for (int attempt = 0; pts.size() >= size_t(c["min_windows"]); ++attempt) {
    origin = pts.front();
    chord = length(pts.back() - origin);
    if (chord < 8)
      break;
    along = unit(pts.back() - origin);
    across = rightOf(along);
    const int n = int(pts.size());
    double total = 0;
    for (double wgt : weights)
      total += wgt;
    const double ridge = std::sqrt(.005 * total * std::pow(h / chord, 4));
    cv::Mat a = cv::Mat::zeros(n + 2, 4, CV_64F), b = cv::Mat::zeros(n + 2, 1, CV_64F);
    for (int i = 0; i < n; ++i) {
      const double t = (pts[i] - origin).dot(along) / chord, wgt = std::sqrt(weights[i]);
      for (int k = 0; k < 4; ++k)
        a.at<double>(i, k) = wgt * std::pow(t, 3 - k);
      b.at<double>(i) = wgt * (pts[i] - origin).dot(across);
    }
    a.at<double>(n, 0) = a.at<double>(n + 1, 1) = ridge;
    if (!cv::solve(a, b, coef, cv::DECOMP_SVD) || !cv::checkRange(coef))
      break;
    size_t worst = 0;
    double worst_error = -1, sum = 0;
    for (int i = 0; i < n; ++i) {
      const double t = (pts[i] - origin).dot(along) / chord;
      double model = 0;
      for (int k = 0; k < 4; ++k)
        model += coef.at<double>(k) * std::pow(t, 3 - k);
      const double e = std::abs((pts[i] - origin).dot(across) - model);
      sum += e * e;
      if (e > worst_error)
        worst_error = e, worst = size_t(i);
    }
    rms = std::sqrt(sum / n);
    if (rms <= c["max_residual"])
      break;
    const size_t drop = attempt == 0 ? worst : pts.size() - 1;
    pts.erase(pts.begin() + long(drop));
    weights.erase(weights.begin() + long(drop));
  }
  out.residual = rms;
  if (rms > c["max_residual"] || coef.empty()) {
    note = chord < 8 ? "관측 길이 부족" : "윈도우 중심이 곡선에 맞지 않음";
    return false;
  }
  out.path.clear();
  for (int k = 0; k <= 32; ++k) {
    const double t = k / 32.;
    double model = 0;
    for (int j = 0; j < 4; ++j)
      model += coef.at<double>(j) * std::pow(t, 3 - j);
    out.path.push_back(origin + along * float(t * chord) + across * float(model));
  }
  out.observed = int(pts.size());
  out.points = pts;
  // Full quality at twice the minimum: a short, real bend is not a weak one.
  out.quality = std::min(1., out.observed / (2 * c["min_windows"]));
  return true;
}
} // namespace

cv::Mat hsvClasses(const cv::Mat &bev, const cv::Mat &valid, const Config &c, cv::Mat *colours) {
  cv::Mat hsv, blurred, tophat, value, yellow, white, white_raw;
  cv::GaussianBlur(bev, blurred, {5, 5}, 0);
  cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(c["yellow_h_min"], c["yellow_s_min"], c["yellow_v_min"]),
              cv::Scalar(c["yellow_h_max"], 255, 255), yellow);
  cv::inRange(hsv, cv::Scalar(0, 0, c["white_v_min"]), cv::Scalar(179, c["white_s_max"], 255), white_raw);
  cv::extractChannel(hsv, value, 2);
  const int k = 2 * int(c["line_width_max"]) + 1;
  cv::morphologyEx(value, tophat, cv::MORPH_TOPHAT, cv::getStructuringElement(cv::MORPH_RECT, {k, k}));
  white = white_raw & (tophat >= c["white_tophat"]);
  // Crosswalk blocks / road text: white but much thicker than a lane line.
  if (const int t = c["white_max_width"]; t > 0) {
    cv::Mat thick;
    cv::morphologyEx(white, thick, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {t, t}));
    cv::dilate(thick, thick, cv::getStructuringElement(cv::MORPH_RECT, {7, 7}));
    white &= ~thick;
  }
  // Only white DASHED lines are boundaries: long continuous white is removed.
  if (const double limit = c["white_max_length"]; limit > 0) {
    std::vector<std::vector<cv::Point>> parts;
    cv::findContours(white.clone(), parts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < parts.size(); ++i)
      if (parts[i].size() >= 2) {
        const auto box = cv::minAreaRect(parts[i]).size;
        if (std::max(box.width, box.height) > limit)
          cv::drawContours(white, parts, int(i), 0, cv::FILLED);
      }
  }
  if (!c["yellow_enabled"])
    yellow.setTo(0);
  if (!c["white_enabled"])
    white.setTo(0), white_raw.setTo(0);
  yellow &= valid;
  white &= valid;
  if (colours)
    *colours = (yellow | white_raw) & valid;
  const auto open3 = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
  cv::morphologyEx(yellow, yellow, cv::MORPH_OPEN, open3);
  cv::morphologyEx(white, white, cv::MORPH_OPEN, open3);
  // A line of thickness t has min(row, column) run <= t*sqrt(2) at any angle;
  // per colour, so glare touching a yellow line does not widen it.
  const int run = cvRound(c["line_width_max"] * 1.45);
  yellow = thinBands(yellow, run);
  white = thinBands(white, run);
  cv::Mat classes = cv::Mat::zeros(bev.size(), CV_8U);
  classes.setTo(2, white);
  classes.setTo(1, yellow);
  if (c["min_area"] > 0) {
    cv::Mat labels, stats, centroids;
    const int count = cv::connectedComponentsWithStats(classes > 0, labels, stats, centroids);
    std::vector<uchar> keep(count, 0);
    for (int i = 1; i < count; ++i)
      keep[i] = stats.at<int>(i, cv::CC_STAT_AREA) >= c["min_area"];
    for (int y = 0; y < classes.rows; ++y) {
      const int *label = labels.ptr<int>(y);
      uchar *m = classes.ptr<uchar>(y);
      for (int x = 0; x < classes.cols; ++x)
        if (!keep[label[x]])
          m[x] = 0;
    }
  }
  return classes;
}

Detection detect(const cv::Mat &classes, int source, const Path prior_in[2], double width, const Config &c,
                 bool keep_roles) {
  Detection out;
  out.source = source;
  const int w = classes.cols, h = classes.rows;
  const cv::Mat mask = classes > 0, yellow = classes == 1;
  const double cu = c["vehicle_center_u"] * (w - 1), tol = c["width_tolerance"];
  const float band = c["line_width_max"];
  const cv::Point2f vehicle(float(cu), float(h - 1));
  Path prior[2] = {prior_in[0], prior_in[1]};

  // 1. Seeds: a recent track starts from its own near end; otherwise the
  // histogram peaks that best form one corridor of the expected width.
  std::vector<Peak> peaks;
  bool peaks_found = false;
  auto histogram = [&]() -> const std::vector<Peak> & {
    if (!peaks_found) {
      peaks_found = true;
      const int min_peak = std::max(5, int(c["min_pixels"]) / 2);
      peaks = findPeaks(mask, yellow, int(c["histogram_start"] * (h - 1)), min_peak, int(c["margin"]));
      if (peaks.empty())
        peaks = findPeaks(mask, yellow, 0, min_peak, int(c["margin"]));
    }
    return peaks;
  };
  auto nearestPeak = [&](double expected, double radius, double exclude) {
    double best = NAN, distance = radius;
    for (const auto &p : histogram())
      if (std::abs(p.x - expected) < distance && p.x != exclude)
        distance = std::abs(p.x - expected), best = p.x;
    return best;
  };
  double seed[2] = {NAN, NAN};
  for (int side = 0; side < 2; ++side)
    if (!prior[side].empty())
      seed[side] = prior[side].front().x;
  if (prior[0].empty() && prior[1].empty()) {
    double best = 1e9;
    for (const auto &a : histogram())
      for (const auto &b : histogram()) {
        const double sep = b.x - a.x;
        if (sep <= 0 || std::abs(sep - width) > tol * width)
          continue;
        const double score = std::abs(sep - width) / width + std::abs((a.x + b.x) / 2. - cu) / width;
        if (score < best)
          best = score, seed[0] = a.x, seed[1] = b.x;
      }
  }
  for (int side = 0; side < 2; ++side)
    if (std::isnan(seed[side])) {
      const double sign = side == 0 ? -1 : 1, other = seed[1 - side];
      const double expected = std::isnan(other) ? cu + sign * width / 2 : other + sign * width;
      seed[side] = nearestPeak(expected, std::isnan(other) ? width * .75 : tol * width * 1.5, other);
      // A lone peak must lie on its own side of the car.
      if (!std::isnan(seed[side]) && std::isnan(other) && (seed[side] - cu) * sign < 0)
        seed[side] = NAN;
    }

  // 2. Direction-following sliding windows and per-line fit. A track whose
  // own windows find nothing (the line moved more than `margin`) gets one
  // more try from the histogram peak nearest to it; the tracker's jump
  // confirmation still guards the switch.
  auto run = [&](int side, cv::Point2f start, cv::Point2f dir, const Path &guide) {
    // Both ways from the start: a seed in the middle of a bend must still
    // collect the part nearer the vehicle. Near -> far order is kept.
    auto search = slide(mask, start, dir, guide, c);
    const auto back = slide(mask, start, -dir, {}, c);
    search.centers.insert(search.centers.begin(), back.centers.rbegin(), back.centers.rend());
    search.weights.insert(search.weights.begin(), back.weights.rbegin(), back.weights.rend());
    search.pixels.insert(search.pixels.end(), back.pixels.begin(), back.pixels.end());
    auto &cand = out.side[side];
    if (!fitLine(search, c, h, cand, out.note[side])) {
      cand = {};
      return false;
    }
    // Paths run near -> far; the search order can invert that (a seed in a
    // bend, a dash gap at the start), which would flip heading and side.
    if (length(cand.path.front() - vehicle) > length(cand.path.back() - vehicle) + .05f * h)
      std::reverse(cand.path.begin(), cand.path.end());
    size_t yellow_pixels = 0, near = 0, near_yellow = 0;
    for (const auto &p : search.pixels) {
      const bool y = yellow.at<uchar>(p) != 0;
      yellow_pixels += y;
      if (p.y >= h / 2)
        ++near, near_yellow += y;
    }
    cand.cls = 2 * yellow_pixels > search.pixels.size() ? 1 : 2;
    cand.near_cls = near ? (2 * near_yellow > near ? 1 : 2) : cand.cls;
    cand.source = source;
    cand.length = pathLength(cand.path);
    out.note[side].clear();
    return true;
  };
  // Lowest marking pixel under a histogram seed: a bend entry often begins
  // mid-BEV and must not end the search by bottom misses.
  auto lowest = [&](double x) {
    const int x0 = std::clamp(cvRound(x - band), 0, w - 1), x1 = std::clamp(cvRound(x + band), x0 + 1, w);
    cv::Mat column;
    cv::reduce(mask.colRange(x0, x1), column, 1, cv::REDUCE_MAX);
    int y = h - 1;
    while (y > 0 && !column.at<uchar>(y))
      --y;
    return cv::Point2f(float(x), float(y));
  };
  for (int side = 0; side < 2; ++side) {
    if (std::isnan(seed[side])) {
      out.note[side] = "시작점 없음";
      continue;
    }
    if (prior[side].empty()) {
      run(side, lowest(seed[side]), {0, -1}, {});
      continue;
    }
    // Start a little nearer than the old near end: the car moved forward.
    const cv::Point2f dir = tangent(prior[side], 0);
    if (run(side, prior[side].front() - dir * float(h / c["windows"]), dir, prior[side]))
      continue;
    const double x = nearestPeak(prior[side].front().x, tol * width * 1.5, seed[1 - side]);
    if (!std::isnan(x) && std::abs(x - prior[side].front().x) > .5 * band)
      run(side, lowest(x), {0, -1}, {});
  }

  // 3. A white boundary whose near part lies across the road is a stop line
  // or crosswalk edge, unless the other boundary turns the same way (a bend).
  // Colour of the near third only: a search that starts on a yellow bend and
  // runs onto a stop line is mostly yellow but its path is the line.
  auto crossing = [&](int side) {
    const auto &p = out.side[side].path;
    if (p.empty() || !transverse(p))
      return false;
    int yellow_count = 0, all = 0;
    for (size_t i = 0; i <= p.size() / 3; ++i) {
      const cv::Rect box = cv::Rect(cvRound(p[i].x) - 4, cvRound(p[i].y) - 4, 9, 9) & cv::Rect(0, 0, w, h);
      yellow_count += cv::countNonZero(yellow(box));
      all += cv::countNonZero(mask(box));
    }
    return 2 * yellow_count <= all;
  };
  for (int side = 0; side < 2; ++side) {
    const auto &other = !out.side[1 - side].path.empty() ? out.side[1 - side].path : prior[1 - side];
    if (crossing(side) && !(other.size() > 1 && transverse(other))) {
      out.side[side] = {};
      out.note[side] = "가로 흰 선(정지선·횡단보도)으로 보고 제외";
      out.forget[side] = true;
      prior[side].clear();
    }
  }
  // 4. A boundary well past the vehicle centre cannot be that side's
  // boundary (e.g. a track locked onto the wrong line): it changes side.
  for (int side = 0; side < 2 && !keep_roles; ++side) {
    const double sign = side == 0 ? 1 : -1;
    auto &cand = out.side[side];
    if (cand.path.empty() || !out.side[1 - side].path.empty() || sign * (cand.path.front().x - cu) < .25 * width)
      continue;
    out.side[1 - side] = cand;
    out.note[1 - side] = std::string(side ? "오른쪽" : "왼쪽") + " 후보가 차량 반대편에 있어 역할 교정";
    cand = {};
    out.note[side] = "역할 교정으로 비움";
    out.forget[side] = true;
    break;
  }
  return out;
}
} // namespace lane_lite
