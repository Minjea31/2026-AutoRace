#include "cone_lane_follower/async_planner.hpp"
#include <iostream>
#include <thread>
using namespace cone;
int main()
{
  AsyncPlanner<int> planner;
  AsyncPlanner<int>::Payload p{{}, {}, {}, {{0, 0, 0}, {.1, 0, 0}}, std::nullopt, CENTER_DEFAULTS, -1};
  planner.request(p, 1);
  // Budget expiry must preserve the measured result.
  std::optional<AsyncPlanner<int>::Result> result;
  for (int i = 0; i < 1000&&!result; i++) {
    result = planner.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!result||result->metadata != 1||result->path != p.measured||!result->diagnostics.count(
        "planning_budget"))
    return 1;
  planner.request(p, 2);
  planner.request(p, 3);
  planner.reset();
  planner.request(p, 4);
  result.reset();
  for (int i = 0; i < 1000&&!result; i++) {
    result = planner.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!result||result->metadata != 4)
    return 2;
  p.left = {{.1,.3,0},{.3,.32,0},{.5,.35,0},{.7,.39,0}};
  p.right = {{.4,-.3,0}};
  p.pairs = single_point_sections(p.left, p.right, .1, 1.5);
  p.measured = single_point_midpath(p.left, p.right, CENTER_DEFAULTS);
  p.width.reset();
  p.budget = .5;
  planner.request(p, 5);
  result.reset();
  for (int i = 0; i < 1000&&!result; i++) {
    result = planner.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!result||result->metadata != 5||result->path.size() < 2||
      !result->diagnostics.count("single_point_midpoints"))
    return 3;
  std::cout << "Async generation, budget fallback and singleton midpoint path passed\n";
}
