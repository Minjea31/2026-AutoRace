#pragma once
#include "core.hpp"
namespace cone {
class PathMemory {
public:
Path points;
size_t index = 0;
bool finished = false;
std::string update_reason = "empty";
void clear()
{
  points.clear();
  index = 0;
  finished = false;
  update_reason = "empty";
}

bool refresh(const Path&observation, const std::function<bool(const Path&)>&validate)
{
  if (!validate(observation))
    return false;
  auto old = finished?Path{}:slice(points, index);
  std::vector<std::pair<double, Path> >candidates;
  for (size_t i = 1; i < old.size(); i++) {
    auto&a = old[i - 1];
    auto&b = old[i];
    if (xy(a, b) * xy(a, b) < 1e-12) continue;
    auto hit = segment_projection(observation.back(), a, b);
    double dx = observation.back()[0] - observation[observation.size() - 2][0], dy = observation.back()[1] -
                                                                                     observation[observation.
                                                                                                 size() -
                                                                                                 2][1], vx =
      b[0] - a[0], vy = b[1] - a[1];
    if (hit.distance <= .10&&dx * vx + dy * vy > 0) {
      auto tail = slice(old, i);
      while (!tail.empty()&&dist(observation.back(), tail[0]) < 1e-8) tail.erase(tail.begin());
      Path combined = observation;
      append(combined, tail);
      if (!tail.empty()&&validate(combined)) candidates.emplace_back(hit.distance, combined);
    }
  }
  points = candidates.empty()?observation:std::min_element(candidates.begin(), candidates.end(), [](auto&a,
                                                                                                    auto&b){
        return a.first < b.first;
      })->second;
  index = 0;
  finished = false;
  update_reason = candidates.empty()?"refreshed":"refreshed_with_tail";
  return true;
}

bool store(const Path&observation)
{
  if (observation.size() < 2||!finite(observation)) {
    update_reason = "invalid_observation";
    return false;
  } if (points.empty()) {
    points = observation;
    index = 0;
    finished = false;
    update_reason = "stored";
    return true;
  } auto old = slice(points, index);
  double dx = old.back()[0] - old[old.size() - 2][0], dy = old.back()[1] - old[old.size() - 2][1];
  using T = std::tuple<double, size_t, double, Point>;
  std::vector<T>candidates;
  for (size_t i = 1; i < observation.size(); i++) {
    auto&a = observation[i - 1];
    auto&b = observation[i];
    if (xy(a, b) * xy(a, b) < 1e-12) continue;
    auto hit = segment_projection(old.back(), a, b);
    double vx = b[0] - a[0], vy = b[1] - a[1], norm = std::hypot(dx, dy) * std::hypot(vx, vy);
    if (hit.distance <= .10&&norm > 1e-12&&(dx * vx + dy * vy) / norm >= -1e-12) candidates.emplace_back(
        hit.distance, i - 1, hit.t, hit.q);
  }
  if (candidates.empty()) {
    update_reason = "kept_no_aligned_extension";
    return false;
  } auto[distance, i, t, q] = *std::min_element(candidates.begin(), candidates.end());
  auto&a = observation[i];
  auto&b = observation[i + 1];
  double vx = b[0] - a[0], vy = b[1] - a[1], raw = ((old.back()[0] - a[0]) * vx + (old.back()[1] - a[1]) *
                                                    vy) / (vx * vx + vy * vy);
  if (!(raw >= -1e-8&&raw <= 1 + 1e-8)) {
    update_reason = "kept_shorter_path";
    return false;
  } auto tail = slice(observation, i + 1);
  double extension = xy(q, tail[0]) + path_length(tail);
  if (extension <= 1e-8) {
    update_reason = "kept_shorter_path";
    return false;
  } while (!tail.empty()&&xy(old.back(), tail[0]) <= 1e-8)
    tail.erase(tail.begin());
  if (tail.empty()) {
    update_reason = "kept_capacity_limit";
    return false;
  } double tx = tail[0][0] - old.back()[0], ty = tail[0][1] - old.back()[1], norm = std::hypot(dx, dy) *
                                                                                    std::hypot(tx, ty);
  if (norm < 1e-12||(dx * tx + dy * ty) / norm < -1e-12) {
    update_reason = "kept_discontinuous_join";
    return false;
  } points = old;
  append(points, tail);
  index = 0;
  finished = false;
  update_reason = "extended";
  return true;
}

bool replace_from_vehicle(const Path&observation, const Path&local, double maximum)
{
  if (observation.size() != local.size()||observation.size() < 2||!finite(observation)||!finite(local)) {
    update_reason = "recovery_invalid_path";
    return false;
  } using T = std::tuple<double, size_t, double>;
  std::vector<T>candidates;
  for (size_t i = 1; i < local.size(); i++) {
    auto&a = local[i - 1];
    auto&b = local[i];
    double len = xy(a, b);
    if (len * len < 1e-12) continue;
    auto h = segment_projection({0, 0, 0}, a, b);
    if (len > 1e-9&&(b[0] - a[0]) / len >= std::cos(pi / 4)) candidates.emplace_back(h.distance, i - 1, h.t);
  } if (candidates.empty()) {
    update_reason = "recovery_direction_mismatch";
    return false;
  } auto[distance, i, t] = *std::min_element(candidates.begin(), candidates.end());
  if (distance > maximum) {
    update_reason = "recovery_too_far_from_vehicle";
    return false;
  } auto q = lerp(observation[i], observation[i + 1], t);
  auto tail = slice(observation, i + 1);
  while (!tail.empty()&&xy(q, tail[0]) < 1e-8) tail.erase(tail.begin());
  if (tail.empty()) {
    update_reason = "recovery_insufficient_forward_path";
    return false;
  } points = {q};
  append(points, tail);
  index = 0;
  finished = false;
  update_reason = "replaced_unusable_memory";
  return true;
}

std::pair<Path, std::string> remaining(const Path&local, double goal, double maximum)
{
  if (finished) return {{}, "memory_goal_reached"};
  if (local.size() != points.size()||local.size() < 2) return {{}, "memory_path_unavailable"};
  using T = std::tuple<double, size_t, double, Point>;
  std::vector<T>choices;
  double arc = 0;
  for (size_t i = index; i + 1 < local.size(); i++) {
    auto&a = local[i];
    auto&b = local[i + 1];
    if (xy(a, b) * xy(a, b) >= 1e-12) {
      auto h = segment_projection({0, 0, 0}, a, b);
      choices.emplace_back(h.distance, i, h.t, h.q);
    } arc += xy(a, b);
    if (arc >= maximum) break;
  } if (choices.empty()) return {{}, "memory_path_unusable"};
  auto[distance, i, t, q] = *std::min_element(choices.begin(), choices.end());
  if (distance > maximum) return {{}, "memory_path_too_far"};
  auto&end = local.back();
  if (i == local.size() - 2&&(std::hypot(end[0], end[1]) <= goal||t >= 1 - 1e-8)) {
    finished = true;
    return {{}, "memory_goal_reached"};
  } index = std::max(index, i);
  Path r{q};
  append(r, slice(local, i + 1));
  return {r, ""};
}

};
} // namespace cone
