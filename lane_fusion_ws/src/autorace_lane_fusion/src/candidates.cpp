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

struct Point {
  cv::Point2f p;
  float weight;
  int cls;
  bool used = false;
};
std::vector<Point> cellCentroids(const cv::Mat &classes, int cell, int min_pixels) {
  const int gx = (classes.cols + cell - 1) / cell, gy = (classes.rows + cell - 1) / cell;
  std::vector<cv::Vec3f> acc(size_t(gx) * gy * 2, cv::Vec3f(0, 0, 0));
  for (int y = 0; y < classes.rows; ++y) {
    const uchar *row = classes.ptr<uchar>(y);
    const int cy = y / cell;
    for (int x = 0; x < classes.cols; ++x)
      if (const uchar k = row[x]; k == 1 || k == 2) {
        auto &a = acc[(size_t(cy) * gx + x / cell) * 2 + (k - 1)];
        a[0] += x, a[1] += y, a[2] += 1;
      }
  }
  std::vector<Point> out;
  for (size_t i = 0; i < acc.size(); ++i)
    if (acc[i][2] >= min_pixels)
      out.push_back({{acc[i][0] / acc[i][2], acc[i][1] / acc[i][2]}, acc[i][2], int(i % 2) + 1});
  return out;
}
// From the unused centroid nearest the vehicle, step to the nearest same-class
// centroid within max_gap that keeps the direction (|turn| <= max_turn), both
// ways. Neighbours within 1.25 cells (a thick band is two cells wide) are
// absorbed into the same chain.
std::vector<std::vector<int>> chainPoints(std::vector<Point> &pts, cv::Point2f vehicle, float cell,
                                          float max_gap, float max_turn_deg) {
  const float cos_turn = std::cos(max_turn_deg * float(CV_PI / 180));
  std::vector<int> order(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    order[i] = int(i);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return length(pts[a].p - vehicle) < length(pts[b].p - vehicle);
  });
  auto absorb = [&](int k) {
    for (auto &q : pts)
      if (!q.used && q.cls == pts[k].cls && length(q.p - pts[k].p) < 1.25f * cell)
        q.used = true;
  };
  auto extend = [&](std::vector<int> &seq) {
    while (true) {
      const int n = int(seq.size());
      const cv::Point2f last = pts[seq.back()].p;
      cv::Point2f dir(0, 0); // over the last ~2 cells (single steps jitter by a cell)
      for (int k = n - 2; k >= 0; --k)
        if (length(last - pts[seq[k]].p) >= 1.5f * cell || k == 0) {
          dir = unit(last - pts[seq[k]].p, {0, 0});
          break;
        }
      int best = -1;
      float best_score = 1e9f;
      for (size_t j = 0; j < pts.size(); ++j) {
        const auto &q = pts[j];
        if (q.used || q.cls != pts[seq.back()].cls)
          continue;
        const float d = length(q.p - last);
        if (d > max_gap || d < 1e-3f)
          continue;
        float score = d;
        if (dir.x != 0 || dir.y != 0) {
          const float c = unit(q.p - last).dot(dir);
          if (c < cos_turn)
            continue;
          score = d * (1 + 3 * (1 - c));
        }
        if (score < best_score)
          best_score = score, best = int(j);
      }
      if (best < 0)
        return;
      pts[best].used = true;
      absorb(best);
      seq.push_back(best);
    }
  };
  std::vector<std::vector<int>> chains;
  for (int start : order) {
    if (pts[start].used)
      continue;
    pts[start].used = true;
    absorb(start);
    std::vector<int> seq = {start};
    extend(seq);
    std::reverse(seq.begin(), seq.end());
    extend(seq);
    chains.push_back(seq);
  }
  return chains;
}
// Windows that step along the marking's own direction from a chain seed.
// The window is wider than a zigzag's teeth, so its centre follows the mean
// line; only observed centres are kept, missed windows are never fitted.
struct Search {
  Path centers;
  std::vector<float> weights;
};
Search slide(const cv::Mat &mask, cv::Point2f start, cv::Point2f dir, const Path &guide, const Config &c) {
  const int w = mask.cols, h = mask.rows;
  const float step = std::max(4, h / int(c["windows"])), margin = c["margin"];
  const float band = c["line_width_max"];
  const size_t min_pixels = c["min_pixels"];
  const float max_turn = float(std::cos(c["max_turn_deg"] * CV_PI / 180));
  Search s;
  cv::Point2f p = start - dir * (step * .5f);
  int misses = 0;
  std::vector<cv::Point> found, inside;
  std::vector<float> offsets;
  for (int i = 0; i < 2 * int(c["windows"]); ++i) {
    if (!guide.empty())
      dir = unit(dir + tangent(guide, nearest(guide, p)), dir);
    const cv::Point2f q = p + dir * step, across = rightOf(dir);
    if (q.x < -margin || q.x > w - 1 + margin || q.y < -step || q.y > h - 1 + step)
      break; // the boundary left the BEV
    const cv::RotatedRect box(q, {2 * margin, step}, float(std::atan2(across.y, across.x) * 180 / CV_PI));
    const cv::Rect bounds = box.boundingRect() & cv::Rect(0, 0, w, h);
    found.clear(), inside.clear(), offsets.clear();
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
    // The cross-track peak closest to the prediction picks one marking.
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
      size_t chosen = 0;
      for (size_t k = 0; k < inside.size(); ++k)
        if (std::abs(offsets[k] - (peak - margin)) <= band)
          sum += cv::Point2d(inside[k]), ++chosen;
      const cv::Point2f center(chosen ? sum * (1. / chosen) : cv::Point2d());
      const cv::Point2f turn = s.centers.empty() ? dir : unit(center - s.centers.back(), dir);
      const size_t n = s.centers.size();
      const cv::Point2f before = n >= 2 ? unit(s.centers[n - 1] - s.centers[n - 2], dir) : dir;
      if (chosen >= min_pixels && turn.dot(dir) > .5f && turn.dot(before) >= max_turn) {
        dir = unit(dir + turn, dir);
        s.centers.push_back(center);
        s.weights.push_back(float(chosen));
        p = center;
        hit = true;
      }
    }
    if (!hit) {
      p = q;
      if (++misses >= c["max_misses"])
        break;
    } else {
      misses = 0;
    }
  }
  return s;
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

std::vector<Candidate> buildCandidates(const cv::Mat &classes, int source, cv::Point2f vehicle,
                                       const Config &c) {
  const float cell = float(c["cell"]);
  auto pts = cellCentroids(classes, int(cell), int(c["min_cell_pixels"]));
  struct Seq {
    Path p;
    std::vector<float> w;
    int cls;
  };
  std::vector<Seq> seqs;
  for (const auto &ids : chainPoints(pts, vehicle, cell, float(c["max_gap"]), float(c["max_turn_deg"]))) {
    Seq q{{}, {}, pts[ids.front()].cls};
    for (int k : ids)
      q.p.push_back(pts[k].p), q.w.push_back(pts[k].weight);
    if (length(q.p.back() - vehicle) < length(q.p.front() - vehicle)) { // near -> far
      std::reverse(q.p.begin(), q.p.end());
      std::reverse(q.w.begin(), q.w.end());
    }
    seqs.push_back(q);
  }
  // A ragged spot or a sharp track corner can split one line: join a far end
  // to another chain's near end. Ends that touch (< 2 cells) may meet at up to
  // 75 deg; across a real gap both directions must agree within max_turn.
  const float join_cos = std::cos(c["max_turn_deg"] * CV_PI / 180);
  const float corner_cos = std::cos(75 * CV_PI / 180);
  auto endDir = [](const Path &p, bool far) {
    const size_t n = p.size(), k = std::min<size_t>(3, n - 1);
    return far ? unit(p[n - 1] - p[n - 1 - k]) : unit(p[k] - p[0]);
  };
  for (bool merged = true; merged;) {
    merged = false;
    for (size_t a = 0; a < seqs.size() && !merged; ++a)
      for (size_t b = 0; b < seqs.size() && !merged; ++b) {
        if (a == b || seqs[a].cls != seqs[b].cls || seqs[a].p.size() < 2 || seqs[b].p.size() < 2)
          continue;
        const cv::Point2f gap = seqs[b].p.front() - seqs[a].p.back();
        const cv::Point2f da = endDir(seqs[a].p, true), db = endDir(seqs[b].p, false);
        const bool touching = length(gap) < 2 * cell;
        if (length(gap) > c["max_gap"] || da.dot(db) < (touching ? corner_cos : join_cos) ||
            (!touching && (unit(gap).dot(da) < join_cos || unit(gap).dot(db) < join_cos)))
          continue;
        seqs[a].p.insert(seqs[a].p.end(), seqs[b].p.begin(), seqs[b].p.end());
        seqs[a].w.insert(seqs[a].w.end(), seqs[b].w.begin(), seqs[b].w.end());
        seqs.erase(seqs.begin() + long(b));
        merged = true;
      }
  }
  // Each chain (also a short one: a zigzag breaks into teeth) seeds sliding
  // windows on its own class, forward along the chain and back toward the
  // vehicle; the window centres are fitted. A chain the windows cannot follow
  // keeps its own fit if it is long enough.
  cv::Mat class_mask[3];
  for (int k = 1; k <= 2; ++k)
    class_mask[k] = classes == k;
  std::vector<Candidate> out;
  for (const auto &q : seqs) {
    if (q.p.size() < 2)
      continue;
    Candidate cand;
    cand.cls = q.cls;
    cand.source = source;
    const cv::Point2f dir = unit(q.p[std::min<size_t>(2, q.p.size() - 1)] - q.p[0]);
    auto ahead = slide(class_mask[q.cls], q.p.front(), dir, q.p, c);
    const auto back = slide(class_mask[q.cls], q.p.front(), -dir, {}, c);
    Path centers(back.centers.rbegin(), back.centers.rend());
    std::vector<float> weights(back.weights.rbegin(), back.weights.rend());
    centers.insert(centers.end(), ahead.centers.begin(), ahead.centers.end());
    weights.insert(weights.end(), ahead.weights.begin(), ahead.weights.end());
    bool ok = centers.size() >= size_t(c["min_windows"]) &&
              fitPath(centers, weights, c["max_residual"], cand.path, cand.residual);
    if (ok) {
      cand.points = centers;
    } else if (pathLength(q.p) >= c["min_length"] && q.p.size() >= 3 &&
               fitPath(q.p, q.w, c["max_residual"], cand.path, cand.residual)) {
      cand.points = q.p;
      ok = true;
    }
    if (!ok)
      continue;
    if (length(cand.path.back() - vehicle) < length(cand.path.front() - vehicle))
      std::reverse(cand.path.begin(), cand.path.end()); // near -> far
    cand.length = pathLength(cand.path);
    if (cand.length < c["min_length"])
      continue;
    cand.quality = std::min(1., cand.length / (.5 * classes.rows));
    out.push_back(cand);
  }
  // A second chain along the same band (thick paint) is a duplicate.
  std::sort(out.begin(), out.end(), [](const Candidate &a, const Candidate &b) { return a.length > b.length; });
  std::vector<Candidate> kept;
  for (const auto &cand : out) {
    bool duplicate = false;
    for (const auto &k : kept)
      duplicate |= distanceTo(cand.path, k.path) < 1.5f * cell;
    if (!duplicate)
      kept.push_back(cand);
  }
  return kept;
}
} // namespace lane_lite
