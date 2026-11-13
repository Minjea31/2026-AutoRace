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
  std::cout << "Async generation reset and measured-path budget fallback passed\n";
}
