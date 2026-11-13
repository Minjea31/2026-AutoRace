#pragma once
#include "cone_lane_follower/core.hpp"
namespace cone {
// Only adjacent scans are associated; no invisible detections are retained.
class BoundaryTracker {
public:
  struct Track { Point world; int side; double seen; };
  std::vector<Track> tracks;
  std::optional<double> stamp;
  void clear() { tracks.clear(); stamp.reset(); }
  std::pair<Path, Path> update(const Path&local, const Path&world, double now,
                              const Params&p, std::optional<double>width = {}, double tol = .25) {
    if (local.size() != world.size()||!finite(local)||!finite(world)||!std::isfinite(now))
      throw std::invalid_argument("invalid boundary tracking input");
    if (stamp&&now <= *stamp) clear();
    stamp = now;

    std::array<bool, 2> bootstrap{true, true};
    for (const auto&t: tracks) bootstrap[t.side] = false;
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
    auto boundaries = select_boundaries_tracked(local, p, width, tol, anchors, bootstrap);
    for (int side = 0; side < 2; ++side) {
      const auto&line = side == 0?boundaries.first:boundaries.second;
      for (const auto&q: line) {
        auto it = std::find(local.begin(), local.end(), q);
        size_t i = size_t(it - local.begin());
        if (owner[i] == -1) {
          tracks.push_back({world[i], side, now});
          owner[i] = int(tracks.size() - 1);
        }
      }
    }
    // Retain only this scan: no grace-period identities or invisible geometry.
    std::vector<Track> current;
    for (size_t i=0;i<world.size();++i)
      if (owner[i]>=0) current.push_back({world[i], tracks[size_t(owner[i])].side, now});
    tracks = std::move(current);
    return boundaries;
  }
};
}
