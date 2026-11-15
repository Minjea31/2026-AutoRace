#pragma once
#include "cone_lane_follower/core.hpp"
namespace cone {
// Visible identities are associated between adjacent scans. Each side keeps its
// last observed endpoint in odom for reacquisition and a one-point boundary hold.
class BoundaryTracker {
public:
  struct Track { Point world; int side; double seen; };
  std::vector<Track> tracks;
  std::optional<double> stamp;
  std::array<std::optional<Point>, 2> last_endpoint;
  void clear() {
    tracks.clear();
    stamp.reset();
    last_endpoint = {};
  }
  std::pair<Path, Path> update(const Path&local, const Path&world,
                              const std::function<Point(const Point&)>&to_local, double now,
                              const Params&p, std::optional<double>width = {}, double tol = .25) {
    if (local.size() != world.size()||!finite(local)||!finite(world)||!std::isfinite(now))
      throw std::invalid_argument("invalid boundary tracking input");
    if (stamp&&now <= *stamp) clear();
    stamp = now;

    // World coordinates remove ego motion. A 12 cm gate is below typical cone spacing.
    using Match = std::tuple<double, size_t, size_t>;
    std::vector<Match> options;
    for (size_t i = 0; i < world.size(); ++i)
      for (size_t j = 0; j < tracks.size(); ++j) {
        double distance = xy(world[i], tracks[j].world);
        if (distance <= .12) options.emplace_back(distance, i, j);
      }
    std::sort(options.begin(), options.end());
    std::vector<int> owner(local.size(), -1);
    std::vector<bool> used(tracks.size(), false);
    std::array<Path, 2> anchors;
    for (auto&[distance, i, j]: options) {
      if (owner[i] != -1||used[j]) continue;
      // Near-ties between opposite sides are ambiguous: do not transfer identity.
      bool ambiguous = false;
      for (size_t k = 0; k < tracks.size(); ++k)
        if (tracks[k].side != tracks[j].side&&
            xy(world[i], tracks[k].world) <= distance + .025) ambiguous = true;
      if (ambiguous) continue;
      owner[i] = int(j);
      used[j] = true;
      anchors[tracks[j].side].push_back(local[i]);
      tracks[j].world = world[i];
      tracks[j].seen = now;
    }
    // LaserScan order runs in opposite directions on the two sides. Width
    // comparisons need boundary travel order, not raw angular scan order.
    for (auto&line: anchors) line = select_boundary_chain(line, p);
    // Without a live match, recover from the missing side's last endpoint rather
    // than choosing a new origin-nearest seed. Use the normal cone connection gate.
    std::array<bool, 2> bootstrap{!last_endpoint[0], !last_endpoint[1]};
    std::vector<Match> recovery;
    for (int side = 0; side < 2; ++side) {
      if (!anchors[side].empty()||!last_endpoint[side]) continue;
      for (size_t i = 0; i < world.size(); ++i) {
        if (owner[i] >= 0) continue;
        double distance = xy(world[i], *last_endpoint[side]);
        if (distance > p.at("boundary_connect_distance")) continue;
        const auto&opposite = anchors[1 - side];
        double sign = side == 0?1:-1;
        if (opposite.size() >= 2&&width?
            !strict_seed_matches(local[i], opposite, sign, *width, tol):
            sign * local[i][1] <= 0) continue;
        recovery.emplace_back(distance, i, size_t(side));
      }
    }
    std::sort(recovery.begin(), recovery.end());
    std::vector<bool> recovered(local.size(), false);
    for (const auto&[distance, i, side]: recovery) {
      if (!anchors[side].empty()||recovered[i]) continue;
      // A point almost equally close to both lost sides has no reliable identity.
      bool ambiguous = false;
      for (const auto&[other_distance, other_i, other_side]: recovery)
        if (other_i == i&&other_side != side&&other_distance <= distance + .025)
          ambiguous = true;
      if (ambiguous) continue;
      anchors[side].push_back(local[i]);
      recovered[i] = true;
    }
    auto boundaries = select_boundaries_tracked(local, p, width, tol, anchors, bootstrap);
    for (int side = 0; side < 2; ++side) {
      const auto&line = side == 0?boundaries.first:boundaries.second;
      if (!line.empty()) {
        // One remaining cone becomes the reference; otherwise keep the chain end.
        auto it = std::find(local.begin(), local.end(), line.back());
        last_endpoint[side] = world[size_t(it - local.begin())];
      }
      for (const auto&q: line) {
        auto it = std::find(local.begin(), local.end(), q);
        size_t i = size_t(it - local.begin());
        if (owner[i] == -1) {
          tracks.push_back({world[i], side, now});
          owner[i] = int(tracks.size() - 1);
        }
      }
    }
    // Scan identities contain only actual detections; a held endpoint must not
    // become a fake detection or renew its world position through round-off.
    std::vector<Track> current;
    for (size_t i=0;i<world.size();++i)
      if (owner[i]>=0) current.push_back({world[i], tracks[size_t(owner[i])].side, now});
    tracks = std::move(current);
    for (int side = 0; side < 2; ++side) {
      auto&line = side == 0?boundaries.first:boundaries.second;
      if (line.empty()&&last_endpoint[side]) {
        auto q = to_local(*last_endpoint[side]);
        if (!finite(Path{q})) throw std::invalid_argument("invalid held boundary transform");
        line.push_back(q);
      }
    }
    return boundaries;
  }
};
}
