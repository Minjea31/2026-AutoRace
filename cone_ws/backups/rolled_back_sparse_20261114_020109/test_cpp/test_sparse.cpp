#include "cone_lane_follower/core.hpp"
#include <iostream>
using namespace cone;
int main()
{
  auto check = [](bool ok, const char* message) {
    if (!ok) { std::cerr << message << '\n'; std::exit(1); }
  };
  Path left{{.1, .3, 0}}, right{{.1, -.3, 0}};
  auto pairs = match_cross_sections(left, right, .05, .1, 1.5);
  check(pairs.size() == 1, "one cone per side must yield a section");
  auto center = lerp(pairs[0].first, pairs[0].second, .5);
  check(xy(center, {.1, 0, 0}) < 1e-9, "incorrect singleton midpoint");
  check(smooth_center_path({center}, left, right, .25, .3, 75).empty(),
        "a single center must not become a drive path");
  check(match_cross_sections(left, {{.1, -2, 0}}, .05, .1, 1.5).empty(), "reject excessive width");
  check(match_cross_sections(left, {}, .05, .1, 1.5).empty(), "missing side has no measured center");
  right.push_back({.7, -.3, 0});
  check(match_cross_sections(left, right, .05, .1, 1.5).size() == 1, "singleton to segment");
  left.push_back({.7, .3, 0});
  check(match_cross_sections(left, {right.front()}, .05, .1, 1.5).size() == 1, "segment to singleton");
  pairs = match_cross_sections(left, right, .05, .1, 1.5);
  Path centers;
  for (auto& pair: pairs) centers.push_back(lerp(pair.first, pair.second, .5));
  auto path = smooth_center_path(centers, left, right, .25, .3, 75);
  check(path.size() >= 2, "two cones per side must produce a path");
  for (auto& point: path) check(std::abs(point[1]) < 1e-9, "path must follow lane center");
  check(!unified_path(left, {}, {}, {}, .6, CENTER_DEFAULTS).empty(), "two-point offset with learned width");
  check(unified_path(left, {}, {}, {}, {}, CENTER_DEFAULTS).empty(), "offset requires learned width");
  check(!unified_path(left, right, pairs, path, {}, CENTER_DEFAULTS).empty(), "measured path needs no learned width");
}
