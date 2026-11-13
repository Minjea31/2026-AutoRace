#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
namespace cone {
using Point = std::array<double, 3>;
using Path = std::vector<Point>;
using Pairs = std::vector<std::pair<Point, Point> >;
using Params = std::map<std::string, double>;
using Diag = std::map<std::string, std::string>;
using Boundaries = std::vector<std::pair<Path, double> >;
constexpr double pi = 3.14159265358979323846;
inline double steady()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline double clamp(double x, double lo = 0., double hi = 1.)
{
  return std::max(lo, std::min(hi, x));
}

inline double xy(const Point&a, const Point&b)
{
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}

inline double dist(const Point&a, const Point&b)
{
  return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2) + std::pow(a[2] - b[2], 2));
}

inline Point lerp(const Point&a, const Point&b, double t)
{
  Point q;
  for (int k = 0; k < 3; k++) q[k] = a[k] + t * (b[k] - a[k]);
  return q;
}

inline Path slice(const Path&p, size_t first, size_t last = SIZE_MAX)
{
  first = std::min(first, p.size());
  last = std::min(last, p.size());
  return Path(p.begin() + first, p.begin() + std::max(first, last));
}

inline void append(Path&a, const Path&b)
{
  a.insert(a.end(), b.begin(), b.end());
}

inline bool finite(const Path&p)
{
  for (auto&q: p) for (auto v: q) if (!std::isfinite(v)) return false;
  return true;
}

inline const Params ROI_DEFAULTS = {{"roi_x_min", -.3}, {"roi_x_max", 1.0}, {"roi_y_min", -.8}, {"roi_y_max",
                                                                                                 .8}};
inline const Params CLUSTER_DEFAULTS = {{"cluster_threshold", .10}, {"min_cluster_points", 2},
  {"max_cluster_width", .28}, {"cluster_point_size", .05}, {"cone_center_size", .08}};
inline const Params CENTER_DEFAULTS = {{"center_min_turn_radius", .2630871384}, {"center_sample_step", .05},
  {"center_min_width", .10}, {"center_max_width", 1.50}, {"center_max_gap", .30}, {"center_max_turn", 75.},
  {"center_smoothing_weight", .25}, {"center_width_relative_tolerance", .25}};
inline const Params BOUNDARY_DEFAULTS = {{"boundary_connect_distance", .30}, {"boundary_max_turn", 80.},
  {"boundary_initial_max_turn", 110.}, {"boundary_direction_window", 3}, {"boundary_distance_weight", 1.},
  {"boundary_direction_weight", 2.}};
inline const Params CONTROL_DEFAULTS = {{"control_speed", .15}, {"control_reference_x", .25},
  {"control_lateral_gain", 2.}, {"control_heading_gain", 1.}, {"control_max_steering", 80. * pi / 180},
  {"control_max_steering_rate", 1.}, {"control_speed_reference_steering", .48}, {"control_timeout", .5},
  {"control_memory_tf_timeout", .2}, {"control_max_path_distance", .5}, {"control_memory_speed", .15},
  {"control_memory_goal_tolerance", .08}, {"control_memory_recovery_distance", .15},
  {"control_exit_timeout", 1.5}};
inline void reject(Diag*d, const std::string&key, const std::string&value)
{
  if (d) (*d)[key] = value;
}

inline void count(Diag*d, const std::string&key)
{
  if (d) {
    auto&i = (*d)[key];
    i = std::to_string(i.empty()?1:std::stoi(i) + 1);
  }
}

struct BudgetExceeded : std::exception {};
inline void check_budget(const Params&p)
{
  auto i = p.find("_planning_deadline");
  if (i != p.end()&&steady() >= i->second) throw BudgetExceeded();
}

inline double angle(double ax, double ay, double bx, double by)
{
  return std::acos(clamp(ax * bx + ay * by, -1, 1)) * 180 / pi;
}

inline double cross(double ax, double ay, double bx, double by)
{
  return ax * by - ay * bx;
}

inline bool segments_cross(const Point&a, const Point&b, const Point&c, const Point&d)
{
  if (std::max(a[0], b[0]) < std::min(c[0], d[0])||std::max(c[0], d[0]) < std::min(a[0], b[0])||std::max(a[1],
                                                                                                         b[1])
      < std::min(c[1], d[1])||std::max(c[1], d[1]) < std::min(a[1], b[1]))
    return false;
  auto cr = [](const Point&p, const Point&q, const Point&r){
              return cross(q[0] - p[0], q[1] - p[1], r[0] - p[0], r[1] - p[1]);
            };
  return cr(a, b, c) * cr(a, b, d) <= 0 && cr(c, d, a) * cr(c, d, b) <= 0;
}

struct Projection {
  double distance, t;
  Point q;
};
inline Projection segment_projection(const Point&p, const Point&a, const Point&b)
{
  double dx = b[0] - a[0], dy = b[1] - a[1], norm = dx * dx + dy * dy;
  double t = norm?clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / norm):0;
  auto q = lerp(a, b, t);
  return {xy(p, q), t, q};
}

inline Path sample_boundary(const Path&line, double step)
{
  if (line.size() < 2)
    return {};
  Path out{line.front()};
  double target = step, total = 0;
  for (size_t i = 1; i < line.size(); i++) {
    auto&a = line[i - 1];
    auto&b = line[i];
    double len = xy(a, b);
    while (len > 1e-9&&target < total + len) {
      out.push_back(lerp(a, b, (target - total) / len));
      target += step;
    } total += len;
  }
  if (xy(out.back(), line.back()) > 1e-8)
    out.push_back(line.back());
  return out;
}

inline double path_length(const Path&p)
{
  double s = 0;
  for (size_t i = 1; i < p.size(); i++) s += xy(p[i - 1], p[i]);
  return s;
}

inline std::vector<double> curvature(const Path&p)
{
  std::vector<double> out;
  for (size_t i = 2; i < p.size(); i++) {
    auto&a = p[i - 2];
    auto&b = p[i - 1];
    auto&c = p[i];
    double den = xy(a, b) * xy(b, c) * xy(a, c);
    out.push_back(den > 1e-12?2 * std::abs(cross(b[0] - a[0], b[1] - a[1], c[0] - b[0], c[1] - b[1])) / den:
                  INFINITY);
  } return out;
}

inline double peak_curvature(const Path&p)
{
  auto v = curvature(p);
  return v.empty()?0:*std::max_element(v.begin(), v.end());
}

struct Option {
  double score;
  Point q, direction;
  bool operator<(const Option&o)const
  {
    return std::tie(score, q, direction) < std::tie(o.score, o.q, o.direction);
  }

};
inline std::vector<Option> boundary_options(const Path&chain, const Path&directions, const Path&remaining,
                                            const Params&p, const Path&opposite)
{
  double hx = 0, hy = 0;
  if (directions.empty())
    hx = 1;
  else
    for (size_t i = directions.size() - std::min(directions.size(), size_t(p.at(
                                                                             "boundary_direction_window")));
         i < directions.size(); i++) {
      hx += directions[i][0];
      hy += directions[i][1];
    }
  double len = std::hypot(hx, hy);
  Point heading = len > 1e-9?Point{hx / len, hy / len, 0}:directions.back();
  double limit = p.at(directions.empty()?"boundary_initial_max_turn":"boundary_max_turn");
  std::vector<Option> out;
  for (auto&q: remaining) {
    double dx = q[0] - chain.back()[0], dy = q[1] - chain.back()[1], gap = std::hypot(dx, dy);
    if (!(gap > 1e-9&&gap <= p.at("boundary_connect_distance")))
      continue;
    Point dir{dx / gap, dy / gap, 0};
    double deviation = angle(heading[0], heading[1], dir[0], dir[1]);
    if (deviation > limit||(!directions.empty()&&angle(directions.back()[0], directions.back()[1], dir[0],
                                                       dir[1]) > p.at("boundary_max_turn")))
      continue;
    bool crossing = false;
    for (size_t i = 1; i + 1 < chain.size(); i++)
      crossing |= segments_cross(chain.back(), q, chain[i - 1], chain[i]);
    for (size_t i = 1; i < opposite.size(); i++)
      crossing |= segments_cross(chain.back(), q, opposite[i - 1], opposite[i]);
    if (crossing)
      continue;
    out.push_back({p.at("boundary_distance_weight") * gap / p.at("boundary_connect_distance") + p.at(
                     "boundary_direction_weight") * deviation / limit, q, dir});
  } return out;
}

inline Path unique_nearest(const Path&cones)
{
  std::set<Point>s(cones.begin(), cones.end());
  Path r(s.begin(), s.end());
  std::sort(r.begin(), r.end(), [](auto&a, auto&b){
      double x = std::hypot(a[0], a[1]), y = std::hypot(b[0], b[1]);
      return std::tie(x, a) < std::tie(y, b);
    });
  return r;
}

inline Path select_boundary_chain(const Path&cones, const Params&p = BOUNDARY_DEFAULTS, const
                                  Path&opposite = {})
{
  auto remaining = unique_nearest(cones);
  if (remaining.empty()) return {};
  auto it = std::find_if(remaining.begin(), remaining.end(), [](auto&q){
      return std::abs(std::atan2(q[1], q[0])) <= pi / 2;
    });
  if (it == remaining.end()) it = remaining.begin();
  Path chain{*it}, dirs;
  remaining.erase(it);
  while (!remaining.empty()) {
    auto options = boundary_options(chain, dirs, remaining, p, opposite);
    if (options.empty()) break;
    auto o = *std::min_element(options.begin(), options.end());
    chain.push_back(o.q);
    dirs.push_back(o.direction);
    remaining.erase(std::find(remaining.begin(), remaining.end(), o.q));
  } return chain;
}

inline bool boundary_width_matches(const Point&q, const Path&opposite, double side, double width, double tol)
{
  std::vector<std::pair<double, double> > comp;
  for (size_t i = 1; i < opposite.size(); i++) {
    auto&a = opposite[i - 1];
    auto&b = opposite[i];
    double dx = b[0] - a[0], dy = b[1] - a[1], norm = dx * dx + dy * dy;
    if (norm < 1e-12) continue;
    double t = ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / norm;
    if (t >= 0&&t <= 1) {
      double sign = cross(dx, dy, q[0] - a[0], q[1] - a[1]) / std::sqrt(norm);
      comp.emplace_back(std::abs(sign), side * sign);
    }
  } if (comp.empty()) return true;
  auto w = std::min_element(comp.begin(), comp.end())->second;
  return w > 0&&std::abs(w - width) <= width * tol;
}

inline std::pair<Path, Path> select_boundaries(const Path&cones, const Params&p,
                                               std::optional<double>width = {}, double tol = .25)
{
  auto rem = unique_nearest(cones);
  std::array<Path, 2> chains, dirs;
  for (int i = 0; i < 2; i++) {
    double side = i == 0?1:-1;
    auto it = rem.end();
    for (auto j = rem.begin(); j != rem.end(); j++) if (side * (*j)[1] > 0) {
        if (it == rem.end()) it = j;
        if (std::abs(std::atan2((*j)[1], (*j)[0])) <= pi / 2) {
          it = j;
          break;
        }
      }if (it != rem.end()) {
      chains[i].push_back(*it);
      rem.erase(it);
    }
  }
  while (!rem.empty()) {
    using T = std::tuple<double, int, Point, Point>;
    std::vector<T> proposals;
    for (int i = 0; i < 2; i++) {
      if (chains[i].empty()) continue;
      for (auto&o: boundary_options(chains[i], dirs[i], rem, p, chains[1 - i])) {
        if (width&&!boundary_width_matches(o.q, chains[1 - i], i == 0?1:-1, *width, tol)) continue;
        proposals.emplace_back(o.score, i, o.q, o.direction);
      }
    } if (proposals.empty()) break;
    auto [score, i, q, dir] = *std::min_element(proposals.begin(), proposals.end());
    chains[i].push_back(q);
    dirs[i].push_back(dir);
    rem.erase(std::find(rem.begin(), rem.end(), q));
  } return {chains[0], chains[1]};
}

inline Pairs match_cross_sections(const Path&left, const Path&right, double step, double minimum, double
                                  maximum)
{
  auto refs = sample_boundary(left, step);
  if (refs.size() < 2||right.size() < 2)
    return {};
  Pairs pairs;
  double previous = -1;
  for (size_t i = 0; i < refs.size(); i++) {
    auto&q = refs[i];
    auto&a = refs[i?i - 1:0];
    auto&b = refs[std::min(refs.size() - 1, i + 1)];
    double dx = b[0] - a[0], dy = b[1] - a[1], len = std::hypot(dx, dy);
    if (len < 1e-9)
      continue;
    double nx = dy / len, ny = -dx / len, arc = 0;
    std::vector<std::pair<double, Point> > hits;
    for (size_t j = 1; j < right.size(); j++) {
      auto&a = right[j - 1];
      auto&b = right[j];
      double vx = b[0] - a[0], vy = b[1] - a[1], sl = std::hypot(vx, vy), den = cross(nx, ny, vx, vy);
      if (std::abs(den) > 1e-9&&dx * vx + dy * vy > 0) {
        double ax = a[0] - q[0], ay = a[1] - q[1], w = cross(ax, ay, vx, vy) / den, t = cross(ax, ay, nx,
                                                                                              ny) / den,
               progress = arc + clamp(t) * sl;
        if (w >= minimum&&w <= maximum&&t >= -1e-8&&t <= 1 + 1e-8&&progress > previous + 1e-8) {
          auto pt = lerp(a, b, clamp(t));
          if (std::none_of(hits.begin(), hits.end(), [&](auto&h){
              return xy(pt, h.second) < 1e-8;
            })) hits.emplace_back(progress, pt);
        }
      } arc += sl;
    }
    if (hits.size() == 1) {
      previous = hits[0].first;
      pairs.emplace_back(q, hits[0].second);
    }
  } return pairs;
}

inline Pairs match_projected_sections(const Path&left, const Path&right, double step, double minimum, double
                                      maximum, Diag*d = nullptr)
{
  if (left.size() < 2||right.size() < 2) {
    count(d, "insufficient_boundary_points");
    return {};
  } if (!finite(left)||!finite(right)) {
    count(d, "nonfinite_boundary");
    return {};
  } auto refs = sample_boundary(left, step);
  if (refs.size() < 2)
    count(d, "insufficient_boundary_samples");
  Pairs pairs;
  double previous = -1;
  for (size_t i = 0; i < refs.size(); i++) {
    auto&q = refs[i];
    auto&a = refs[i?i - 1:0];
    auto&b = refs[std::min(refs.size() - 1, i + 1)];
    double dx = b[0] - a[0], dy = b[1] - a[1], norm = std::hypot(dx, dy);
    if (norm < 1e-9) {
      count(d, "degenerate_left_tangent");
      continue;
    } using T = std::tuple<bool, double, double, Point>;
    std::vector<T>opts;
    double arc = 0;
    for (size_t j = 1; j < right.size(); j++) {
      auto&a = right[j - 1];
      auto&b = right[j];
      double vx = b[0] - a[0], vy = b[1] - a[1], len = std::hypot(vx, vy);
      if (len < 1e-9) {
        count(d, "degenerate_right_segment");
        continue;
      } double raw = ((q[0] - a[0]) * vx + (q[1] - a[1]) * vy) / (len * len), t = clamp(raw);
      auto pt = lerp(a, b, t);
      double rx = pt[0] - q[0], ry = pt[1] - q[1], width = std::hypot(rx, ry), progress = arc + t * len;
      arc += len;
      bool interior = raw > 1e-8&&raw < 1 - 1e-8;
      if (!(minimum <= width&&width <= maximum)) {
        count(d, "width_out_of_range");
        continue;
      } if (progress < previous - 1e-8) {
        count(d, "backward_correspondence");
        continue;
      } if (dx * vx + dy * vy <= 0) {
        count(d, "opposing_tangents");
        continue;
      } if ((dy * rx - dx * ry) / norm <= 1e-8||(vy * rx - vx * ry) / len <= 1e-8) {
        count(d, "connector_not_inward");
        continue;
      } if (!pairs.empty()&&xy(pairs.back().second, pt) > 1e-8&&segments_cross(pairs.back().first,
                                                                               pairs.back().second, q, pt)) {
        count(d, "crossing_sections");
        continue;
      } if (std::none_of(opts.begin(), opts.end(), [&](auto&h){
          return xy(pt, std::get<3>(h)) < 1e-8;
        })) opts.emplace_back(!interior, width, progress, pt);
    }
    if (opts.empty())
      continue;
    std::sort(opts.begin(), opts.end());
    if (opts.size() > 1&&std::get<0>(opts[0]) == std::get<0>(opts[1])&&std::abs(std::get<1>(opts[0]) -
                                                                                std::get<1>(opts[1])) <
        1e-8) {
      count(d, "ambiguous_projection");
      continue;
    } previous = std::get<2>(opts[0]);
    pairs.emplace_back(q, std::get<3>(opts[0]));
  } return pairs;
}

inline Path smooth_center_path(const Path&centers, const Path&left, const Path&right, double weight, double
                               maxgap, double maxturn, Diag*d = nullptr)
{
  auto fail = [&](const std::string&reason)->Path {
                count(d, reason);
                return {};
              };
  if (centers.size() < 2)
    return fail("insufficient_center_points");
  Path poly = left;
  poly.insert(poly.end(), right.rbegin(), right.rend());
  size_t n = poly.size();
  for (size_t i = 0; i < n; i++)
    for (size_t j = i + 2; j < n; j++)
      if (!(i == 0&&j == n - 1)&&segments_cross(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) %
                                                                                          n])) return fail(
          "lane_polygon_crosses");
  auto inside = [&](const Point&p){
                  int count = 0;
                  for (size_t i = 0; i < n; i++) {
                    auto&a = poly[i];
                    auto&b = poly[(i + 1) % n];
                    if ((a[1] > p[1]) != (b[1] > p[1])) {
                      double x = a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
                      count += x > p[0];
                    }
                  } return count % 2 == 1;
                };
  auto safe = [&](const Point&a, const Point&b){
                if (!inside(a)||!inside(b)) return false;
                for (size_t i = 0; i < n; i++) if (segments_cross(a, b, poly[i], poly[(i + 1) %
                                                                                      n])) return false;
                return true;
              };
  std::vector<Path>runs;
  Path run;
  for (auto&p: centers) {
    if (!inside(p)) {
      runs.push_back(run);
      run.clear();
      continue;
    } if (!run.empty()&&(xy(run.back(), p) > maxgap||!safe(run.back(), p))) {
      runs.push_back(run);
      run.clear();
    } run.push_back(p);
  } runs.push_back(run);
  Path path = *std::max_element(runs.begin(), runs.end(), [](auto&a, auto&b){
      return a.size() < b.size();
    });
  if (path.size() < 2)
    return fail("no_continuous_inside_path");
  for (size_t i = 1; i + 1 < path.size(); i++) {
    Point p;
    for (int k = 0; k < 3; k++) p[k] = (1 - weight) * path[i][k] + weight * (path[i - 1][k] + path[i +
                                                                                                   1][k]) / 2;
    if (safe(path[i - 1], p)&&safe(p, path[i + 1])) path[i] = p;
  }
  for (size_t i = 2; i < path.size(); i++) {
    auto&a = path[i - 2];
    auto&b = path[i - 1];
    auto&c = path[i];
    double ux = b[0] - a[0], uy = b[1] - a[1], vx = c[0] - b[0], vy = c[1] - b[1];
    double co = (ux * vx + uy * vy) / std::max(1e-12, std::hypot(ux, uy) * std::hypot(vx, vy));
    if (std::acos(clamp(co, -1, 1)) * 180 / pi > maxturn) return fail("path_turn_limit");
  }
  for (size_t i = 1; i < path.size(); i++)
    for (size_t j = i + 2; j < path.size(); j++)
      if (segments_cross(path[i - 1], path[i], path[j - 1], path[j])) return fail("path_self_intersection");
  return path;
}

inline Path offset_boundary(const Path&boundary, double distance, double step)
{
  if (!finite(boundary)) return {};
  auto refs = sample_boundary(boundary, step);
  Path out;
  for (size_t i = 0; i < refs.size(); i++) {
    auto&a = refs[i?i - 1:0];
    auto&b = refs[std::min(refs.size() - 1, i + 1)];
    double dx = b[0] - a[0], dy = b[1] - a[1], len = std::hypot(dx, dy);
    if (len < 1e-9) return {};
    out.push_back({refs[i][0] - distance * dy / len, refs[i][1] + distance * dx / len, refs[i][2]});
  } return out;
}

inline bool point_near_boundary(const Point&p, const Path&line, double tolerance)
{
  for (size_t i = 1; i < line.size(); i++) {
    auto&a = line[i - 1];
    auto&b = line[i];
    double dx = b[0] - a[0], dy = b[1] - a[1], norm = dx * dx + dy * dy;
    if (norm < 1e-12) continue;
    double t = clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / norm);
    if (std::hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy) <= tolerance) return true;
  } return false;
}

inline bool offset_safe(const Path&path, const Boundaries&boundaries, Diag*d = nullptr)
{
  for (auto&[line, side]: boundaries) {
    for (auto&point: path) {
      double best = INFINITY;
      size_t at = 0;
      Projection hit{};
      for (size_t i = 1; i < line.size(); i++) {
        auto h = segment_projection(point, line[i - 1], line[i]);
        if (h.distance < best) {
          best = h.distance;
          hit = h;
          at = i;
        }
      } if (at == 0) throw std::invalid_argument("empty boundary segments");
      auto&a = line[at - 1];
      auto&b = line[at];
      double dx = b[0] - a[0], dy = b[1] - a[1], raw = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) /
                                                       (dx * dx + dy * dy);
      if (raw >= 0&&raw <= 1&&side * cross(dx, dy, point[0] - hit.q[0], point[1] - hit.q[1]) <= 0) {
        reject(d, "outside_boundary", "point outside observed boundary side=" + std::to_string(side));
        return false;
      }
    }
    for (size_t i = 1; i < path.size(); i++) for (size_t j = 1; j < line.size(); j++) if (segments_cross(
                                                                                            path[i - 1],
                                                                                            path[i], line[j -
                                                                                                          1],
                                                                                            line[j])) {
          reject(d, "boundary_crossing", "boundary_side=" + std::to_string(side));
          return false;
        }
  } return true;
}

inline bool valid_center_geometry(const Path&path, const Params&p, Diag*d = nullptr)
{
  if (path.size() < 2||!finite(path))
    return false;
  for (size_t i = 1; i < path.size(); i++)
    if (xy(path[i - 1], path[i]) > p.at("center_max_gap")) {
      reject(d, "gap", "segment=" + std::to_string(i - 1) + "->" + std::to_string(i) + " distance=" +
             std::to_string(xy(path[i - 1], path[i])) + "m limit=" + std::to_string(p.at("center_max_gap")) +
             "m");
      return false;
    }
  for (size_t i = 2; i < path.size(); i++) {
    auto&a = path[i - 2];
    auto&b = path[i - 1];
    auto&c = path[i];
    double ux = b[0] - a[0], uy = b[1] - a[1], vx = c[0] - b[0], vy = c[1] - b[1], norm = std::hypot(ux, uy) *
                                                                                          std::hypot(vx, vy);
    double deg = norm >= 1e-12?std::acos(clamp((ux * vx + uy * vy) / norm, -1, 1)) * 180 / pi:NAN;
    if (norm < 1e-12||deg > p.at("center_max_turn")) {
      reject(d, "turn", "point=" + std::to_string(i - 1) + " angle=" + std::to_string(deg) + "deg limit=" +
             std::to_string(p.at("center_max_turn")) + "deg");
      return false;
    }
  }
  for (size_t i = 1; i < path.size(); i++)
    for (size_t j = i + 2; j < path.size(); j++)
      if (segments_cross(path[i - 1], path[i], path[j - 1], path[j])) {
        reject(d, "self_crossing", "segments=" + std::to_string(i - 1) + ", " + std::to_string(j - 1));
        return false;
      }return true;
}

inline Path quintic_bridge(const Path&first, int i, const Path&second, int j, double step)
{
  auto derivatives = [](const Path&path, int idx)->std::optional<std::pair<Point, Point> > {
                       int lo = std::max(0, idx - 1), hi = std::min(int(path.size()) - 1, idx + 1);
                       double len = dist(path[lo], path[hi]);
                       if (len < 1e-9) return {};
                       Point tan{}, cur{};
                       for (int k = 0; k < 3; k++) tan[k] = (path[hi][k] - path[lo][k]) / len;
                       if (idx > 0&&idx < int(path.size()) - 1) {
                         auto&a = path[idx - 1];
                         auto&b = path[idx];
                         auto&c = path[idx + 1];
                         double u = dist(a, b), v = dist(b, c);
                         if (std::min(u, v) > 1e-9) for (int k = 0; k < 3; k++) cur[k] = 2 * ((c[k] - b[k]) /
                                                                                              v - (b[k] -
                                                                                                   a[k]) /
                                                                                              u) / (u + v);
                       } return std::make_pair(tan, cur);
                     };
  auto l = derivatives(first, i), r = derivatives(second, j);
  double len = dist(first[i], second[j]);
  if (!l||!r||len < 1e-9)
    return {};
  std::array<std::array<double, 6>, 3>coef;
  for (int k = 0; k < 3; k++) {
    double a = first[i][k], b = l->first[k] * len, c = l->second[k] * len * len / 2, d = second[j][k] - a -
                                                                                         b - c, e =
      r->first[k] * len - b - 2 * c, f = r->second[k] * len * len - 2 * c;
    coef[k] = {a, b, c, 10 * d - 4 * e + f / 2, -15 * d + 7 * e - f, 6 * d - 3 * e + f / 2};
  } int samples = std::max(2, int(std::ceil(len / step)) * 4);
  Path curve;
  for (int m = 0; m <= samples; m++) {
    Point q{};
    for (int k = 0; k < 3; k++) for (int n = 0; n < 6; n++) q[k] += coef[k][n] * std::pow(double(m) / samples,
                                                                                          n);
    curve.push_back(q);
  } return sample_boundary(curve, step);
}

inline Path corner_offset(const Path&boundary, double distance, double step, double radius, const Params&p,
                          Diag*d = nullptr)
{
  struct Segment {
    Point q, end, t;
  };
  std::vector<Segment>segments;
  for (size_t i = 1; i < boundary.size(); i++) {
    auto&a = boundary[i - 1];
    auto&b = boundary[i];
    double dx = b[0] - a[0], dy = b[1] - a[1], len = std::hypot(dx, dy);
    if (len < 1e-9) continue;
    Point t{dx / len, dy / len, 0};
    segments.push_back({{a[0] - distance * t[1], a[1] + distance * t[0], a[2]}, {b[0] - distance * t[1],
                                                                                 b[1] + distance * t[0],
                                                                                 b[2]}, t});
  }
  if (segments.empty()) {
    reject(d, "corner_failure", "no nonzero boundary segments");
    return {};
  } Path vertices{segments[0].q};
  int corners = 0;
  for (size_t i = 1; i < segments.size(); i++) {
    auto&a = segments[i - 1].q;
    auto&end = segments[i - 1].end;
    auto&u = segments[i - 1].t;
    auto&b = segments[i].q;
    auto&v = segments[i].t;
    double cosine = u[0] * v[0] + u[1] * v[1];
    if (cosine <= -.95) {
      reject(d, "corner_failure", "boundary reversal; check cone association");
      return {};
    } double den = cross(u[0], u[1], v[0], v[1]);
    if (std::abs(den) < 1e-8) {
      vertices.push_back(b);
      continue;
    } corners += std::acos(clamp(cosine, -1, 1)) > 15 * pi / 180;
    double t = cross(b[0] - a[0], b[1] - a[1], v[0], v[1]) / den;
    Point point{a[0] + t * u[0], a[1] + t * u[1], b[2]};
    if (xy(point, end) > std::max(std::abs(distance) * 4, step)) {
      reject(d, "corner_failure", "unbounded miter; check cone association");
      return {};
    } vertices.push_back(point);
  }
  vertices.push_back(segments.back().end);
  int trimmed = 0;
  while (true) {
    std::optional<std::tuple<size_t, size_t, Point> >hit;
    for (size_t i = 0; i + 1 < vertices.size()&&!hit; i++) for (size_t j = i + 2; j + 1 < vertices.size();
                                                                j++) {
        auto&a = vertices[i];
        auto&b = vertices[i + 1];
        auto&c = vertices[j];
        auto&dd = vertices[j + 1];
        double ux = b[0] - a[0], uy = b[1] - a[1], vx = dd[0] - c[0], vy = dd[1] - c[1], den = cross(ux, uy,
                                                                                                     vx, vy);
        if (std::abs(den) < 1e-10) continue;
        double dx = c[0] - a[0], dy = c[1] - a[1], t = cross(dx, dy, vx, vy) / den, z = cross(dx, dy, ux,
                                                                                              uy) / den;
        if (t >= 0&&t <= 1&&z >= 0&&z <= 1) {
          hit = {{i, j, {a[0] + t * ux, a[1] + t * uy, a[2]}}};
          break;
        }
      }if (!hit) break;
    auto[i, j, q] = *hit;
    Path v = slice(vertices, 0, i + 1);
    v.push_back(q);
    append(v, slice(vertices, j + 1));
    vertices = std::move(v);
    trimmed++;
  }
  Path clean;
  for (auto&q: vertices)
    if (clean.empty()||dist(q, clean.back()) > 1e-8) clean.push_back(q);
  Path path = sample_boundary(clean, step);
  reject(d, "corners", std::to_string(corners));
  reject(d, "trimmed_loops", std::to_string(trimmed));
  double side = distance > 0?1:-1;
  auto acceptable = [&](const Path&candidate){
                      if (!offset_safe(candidate, {{boundary, side}})||!valid_center_geometry(candidate,
                                                                                              p)) return false;
                      double progress = -INFINITY;
                      for (auto&point: candidate) {
                        std::vector<std::pair<double, double> >hits;
                        double arc = 0;
                        for (size_t i = 1; i < boundary.size(); i++) {
                          auto&a = boundary[i - 1];
                          auto&b = boundary[i];
                          double len = xy(a, b);
                          if (len < 1e-9) continue;
                          auto h = segment_projection(point, a, b);
                          hits.emplace_back(h.distance, arc + h.t * len);
                          arc += len;
                        } if (hits.empty()) return false;
                        double current = std::min_element(hits.begin(), hits.end())->second;
                        if (current < progress - step) return false;
                        progress = std::max(progress, current);
                      } return true;
                    };
  double limit = 1 / radius;
  for (int iteration = 0; iteration < 12; iteration++) {
    check_budget(p);
    auto values = curvature(path);
    double peak = values.empty()?0:*std::max_element(values.begin(), values.end());
    if (peak <= limit&&acceptable(path)) {
      reject(d, "max_curvature_per_m", std::to_string(peak));
      reject(d, "min_radius_m", std::to_string(radius));
      reject(d, "corner_smoothing", "success");
      return path;
    } if (path.size() < 4)
      break;
    int index = values.empty()?path.size() / 2:std::max_element(values.begin(), values.end()) -
                values.begin() + 1;
    std::optional<std::pair<double, Path> >best;
    for (int span = 3; span < int(path.size()); span++) {
      check_budget(p);
      int lo = std::max(0, index - span), hi = std::min(int(path.size()) - 1, index + span);
      if (hi - lo < 3) continue;
      auto bridge = quintic_bridge(path, lo, path, hi, step);
      Path candidate = slice(path, 0, lo);
      append(candidate, bridge);
      append(candidate, slice(path, hi + 1));
      if (bridge.empty()||!acceptable(candidate)) continue;
      double newpeak = peak_curvature(candidate);
      if (newpeak < peak - 1e-6&&(!best||newpeak < best->first)) best = {{newpeak, candidate}};
      if (newpeak <= limit) break;
    } if (!best)
      break;
    path = best->second;
  }
  reject(d, "corner_failure", "no safe corner within radius >= " + std::to_string(radius) + "m");
  reject(d, "max_curvature_per_m", std::to_string(peak_curvature(path)));
  return {};
}

inline Path unified_path(const Path&left, const Path&right, const Pairs&pairs, const Path&measured,
                         std::optional<double>width, const Params&p, Diag*d = nullptr)
{
  Boundaries valid;
  if (left.size() >= 3)
    valid.emplace_back(left, -1);
  if (right.size() >= 3)
    valid.emplace_back(right, 1);
  if (valid.empty()||!width) {
    reject(d, "offset_unavailable", "requires boundary >=3 points and learned width");
    return measured;
  }
  auto distance = [](const Path&line){
                    double best = INFINITY;
                    for (size_t i = 1; i < line.size(); i++) best = std::min(best, segment_projection({0, 0,
                                                                                                       0},
                                                                                                      line[i -
                                                                                                           1],
                                                                                                      line[i])
                                                                             .distance);
                    return best;
                  };
  if (pairs.empty()) {
    auto it = std::min_element(valid.begin(), valid.end(), [&](auto&a, auto&b){
        return distance(a.first) < distance(b.first);
      });
    auto&line = it->first;
    double side = it->second;
    auto&opposite = side == -1?right:left;
    if (opposite.size() < 3)
      for (auto&point: opposite) {
        std::vector<std::pair<double, double> >candidates;
        for (size_t i = 1; i < line.size(); i++) {
          auto&a = line[i - 1];
          auto&b = line[i];
          auto hit = segment_projection(point, a, b);
          double dx = b[0] - a[0], dy = b[1] - a[1], len = std::hypot(dx, dy);
          if (len > 1e-9) candidates.emplace_back(hit.distance, side * cross(dx, dy, point[0] - hit.q[0],
                                                                             point[1] - hit.q[1]) / len);
        } if (!candidates.empty()) {
          auto[dist, inward] = *std::min_element(candidates.begin(), candidates.end());
          if (inward <= 0||std::abs(dist - *width) > *width * p.at("center_width_relative_tolerance")) {
            reject(d, "width_conflict", "point distance=" + std::to_string(dist) + " width=" + std::to_string(
                     *width) + " inward=" + std::to_string(inward));
            return {};
          }
        }
      }
    reject(d, "offset_side", side == -1?"left boundary -> right normal":"right boundary -> left normal");
    auto path = corner_offset(line, side * (*width) / 2, p.at("center_sample_step"), p.at(
                                "center_min_turn_radius"), p, d);
    double len = 0;
    for (size_t i = 1; i < path.size(); i++)
      len += dist(path[i - 1], path[i]);
    reject(d, "offset_length_m", std::to_string(len));
    return offset_safe(path, valid, d)&&valid_center_geometry(path, p, d)?path:Path{};
  }
  if (measured.empty())
    return {};
  struct Tail {
    double remaining;
    Path line;
    double side;
    size_t i;
    double t;
  };
  std::vector<Tail>tails;
  for (int idx = 0; idx < 2; idx++) {
    auto&line = idx == 0?left:right;
    if (line.size() < 3) continue;
    double side = idx == 0?-1:1;
    auto refs = sample_boundary(line, p.at("center_sample_step"));
    auto point = idx == 0?pairs.back().first:pairs.back().second;
    double best = INFINITY;
    Projection hit{};
    size_t at = 0;
    for (size_t i = 1; i < refs.size(); i++) {
      auto h = segment_projection(point, refs[i - 1], refs[i]);
      if (h.distance < best) {
        best = h.distance;
        hit = h;
        at = i - 1;
      }
    } double remaining = dist(hit.q, refs[at + 1]);
    for (size_t i = at + 2; i < refs.size(); i++) remaining += dist(refs[i - 1], refs[i]);
    tails.push_back({remaining, line, side, at, hit.t});
  }
  if (tails.size() != 2)
    return measured;
  std::stable_sort(tails.begin(), tails.end(), [](auto&a, auto&b){
      return a.remaining < b.remaining;
    });
  if (tails[1].remaining - tails[0].remaining <= p.at("center_sample_step"))
    return measured;
  auto&longer = tails[1];
  auto offset = corner_offset(longer.line, longer.side * (*width) / 2, p.at("center_sample_step"), p.at(
                                "center_min_turn_radius"), p, d);
  if (offset.empty())
    return measured;
  auto target = offset_boundary(longer.line, longer.side * (*width) / 2, p.at("center_sample_step")).at(
    longer.i);
  auto cut = std::min_element(offset.begin(), offset.end(), [&](auto&a, auto&b){
      return dist(a, target) < dist(b, target);
    }) - offset.begin();
  auto tail = slice(offset, cut + 1);
  reject(d, "offset_side", longer.side == -1?"left boundary -> right normal":"right boundary -> left normal");
  double len = 0;
  for (size_t i = 1; i < tail.size(); i++)
    len += dist(tail[i - 1], tail[i]);
  reject(d, "offset_length_m", std::to_string(len));
  if (tail.size() < 2)
    return measured;
  for (int count = 3; count <= int(std::max(measured.size(), tail.size())); count++) {
    check_budget(p);
    int start = std::max(0, int(measured.size()) - count), end = std::min(int(tail.size()) - 1, count - 1);
    auto bridge = quintic_bridge(measured, start, tail, end, p.at("center_sample_step"));
    auto combined = slice(measured, 0, start);
    append(combined, bridge);
    append(combined, slice(tail, end + 1));
    if (!bridge.empty()&&offset_safe(combined, valid, d)&&valid_center_geometry(combined, p, d)&&
        peak_curvature(combined) <= 1 / p.at("center_min_turn_radius")) {
      reject(d, "join", "success window_samples=" + std::to_string(count));
      return combined;
    }
  }
  reject(d, "join", "failed; retained measured sections");
  return measured;
}

struct ControlGeometry {
  double lateral, heading, x, y;
  Point a, b;
};
inline std::optional<ControlGeometry> path_control_geometry(const Path&path, double ref, double maximum = .5)
{
  if (path.size() < 2||!finite(path)) return {};
  using T = std::tuple<double, double, double, double, double, Point, Point>;
  std::vector<T>choices;
  for (size_t i = 1; i < path.size(); i++) {
    auto&a = path[i - 1];
    auto&b = path[i];
    double dx = b[0] - a[0], dy = b[1] - a[1], len = std::hypot(dx, dy);
    if (len < 1e-8||dx / len <= .1) continue;
    double t = clamp(((ref - a[0]) * dx - a[1] * dy) / (len * len)), x = a[0] + t * dx, y = a[1] + t * dy;
    if (x <= 0) continue;
    double lat = (dx * y - dy * (x - ref)) / len;
    choices.emplace_back(std::hypot(x - ref, y), lat, std::atan2(dy, dx), x, y, a, b);
  } if (choices.empty()) return {};
  auto s = *std::min_element(choices.begin(), choices.end(), [](auto&a, auto&b){
      return std::tie(std::get<0>(a), std::get<1>(a), std::get<2>(a)) < std::tie(std::get<0>(b), std::get<1>(
                                                                                   b), std::get<2>(b));
    });
  if (std::get<0>(s) > maximum) return {};
  return ControlGeometry{std::get<1>(s), std::get<2>(s), std::get<3>(s), std::get<4>(s), std::get<5>(s),
                         std::get<6>(s)};
}

inline std::string path_stop_reason(const Path&p, double ref)
{
  if (p.empty()) return "empty_path";
  if (p.size() < 2) return "path_too_few_points";
  if (!finite(p)) return "path_nonfinite_coordinates";
  if (!path_control_geometry(p, ref, INFINITY)) return "path_no_usable_forward_segment";
  return "path_too_far_from_reference";
}

inline std::string validate_roi(const Params&p)
{
  for (auto&[k, v]: p) if (!std::isfinite(v)) return "ROI bounds must be finite numbers";
  if (p.at("roi_x_min") >= p.at("roi_x_max")||p.at("roi_y_min") >= p.at(
        "roi_y_max")) return "ROI minima must be smaller than maxima";
  return "";
}

inline std::string validate_parameters(const Params&p)
{
  for (auto&[name, value]: p) {
    if (name == "min_cluster_points"||name == "boundary_direction_window") {
      if (value < 1||std::floor(value) != value) return name + " must be an integer >= 1";
    }else if (!std::isfinite(value)||value <= 0) return name + " must be finite and > 0";
  } if (p.at("center_min_width") >= p.at(
          "center_max_width")) return "center_min_width must be smaller than center_max_width";
  if (p.at("center_smoothing_weight") > .5||p.at("center_max_turn") >
      180) return "smoothing weight must be <= 0.5 and center_max_turn <= 180";
  if (p.at("boundary_max_turn") >= 180||p.at("boundary_initial_max_turn") >
      180) return "boundary_max_turn must be < 180 and initial turn <= 180 degrees";
  if (p.at("control_max_steering") >= pi / 2) return "control_max_steering must be < pi/2 radians";
  return "";
}

using Clusters = std::vector<Path>;
inline Clusters cluster_points(const Path&points, const std::vector<size_t>&indices, double threshold)
{
  if (points.size() != indices.size()) throw std::invalid_argument("Point and scan index counts differ");
  Clusters out;
  for (size_t i = 0; i < points.size(); i++) {
    if (i == 0||indices[i] != indices[i - 1] + 1||xy(points[i], points[i - 1]) >=
        threshold) out.emplace_back();
    out.back().push_back(points[i]);
  } return out;
}

inline Clusters filter_clusters(const Clusters&clusters, size_t minimum, double maximum)
{
  Clusters out;
  for (auto&c: clusters) {
    if (c.size() < minimum) continue;
    double xlo = INFINITY, xhi = -INFINITY, ylo = INFINITY, yhi = -INFINITY;
    for (auto&q: c) {
      xlo = std::min(xlo, q[0]);
      xhi = std::max(xhi, q[0]);
      ylo = std::min(ylo, q[1]);
      yhi = std::max(yhi, q[1]);
    } if (xhi - xlo <= maximum&&yhi - ylo <= maximum) out.push_back(c);
  } return out;
}

inline Point cluster_center(const Path&p)
{
  if (p.empty()) throw std::invalid_argument("Cannot calculate an empty cluster center");
  Point q{};
  for (auto&a: p) for (int k = 0; k < 3; k++) q[k] += a[k];
  for (auto&v: q) v /= p.size();
  return q;
}

} // namespace cone
