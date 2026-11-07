// DeepPairing: frame/mask pairing policy of lane_fusion_node (no ROS graph).
#include "autorace_lane_fusion/deep_pairing.hpp"
#include <gtest/gtest.h>

namespace {
using lane_lite::DeepPairing;
// Camera stamps 33 ms apart.
builtin_interfaces::msg::Time stamp(int i) {
  builtin_interfaces::msg::Time t;
  t.sec = i * 33 / 1000;
  t.nanosec = (i * 33 % 1000) * 1000000;
  return t;
}
int index(const builtin_interfaces::msg::Time &t) { return (t.sec * 1000 + t.nanosec / 1000000) / 33; }
DeepPairing::Frame frame(int i) {
  auto f = std::make_shared<sensor_msgs::msg::CompressedImage>();
  f->header.stamp = stamp(i);
  return f;
}
DeepPairing::Mask mask(int i) {
  auto m = std::make_shared<sensor_msgs::msg::Image>();
  m->header.stamp = stamp(i);
  m->encoding = "mono8";
  m->width = m->height = m->step = 2;
  m->data.assign(4, 0);
  return m;
}
struct Run {
  int own = 0, previous = 0, none = 0;
  std::vector<int> order;
};
// Camera at 30 Hz; the network takes `latency` s and handles every
// `every`-th frame (it skips frames while busy).
Run simulate(double latency, int every, int frames = 60) {
  DeepPairing p;
  Run run;
  std::vector<std::pair<double, int>> due; // mask arrival time, frame index
  for (int tick = 0; tick < frames * 33 + 600; ++tick) {
    const double now = tick * 1e-3;
    if (tick % 33 == 0 && tick / 33 < frames) {
      const int i = tick / 33;
      p.addFrame(frame(i), now);
      if (i % every == 0)
        due.push_back({now + latency, i});
    }
    for (auto it = due.begin(); it != due.end();)
      if (it->first <= now) {
        p.addMask(mask(it->second), now);
        it = due.erase(it);
      } else
        ++it;
    DeepPairing::Frame f;
    DeepPairing::Mask m;
    bool own;
    if (p.next(true, now, .3, .5, f, m, own)) {
      run.order.push_back(index(f->header.stamp));
      (own ? run.own : m ? run.previous : run.none)++;
    }
  }
  return run;
}
} // namespace

TEST(DeepPairing, EveryFrameGetsItsOwnMaskWhenTheNetworkKeepsUp) {
  const auto r = simulate(.05, 1);
  EXPECT_GE(r.own, 58);
  EXPECT_LE(r.previous + r.none, 2);
}
TEST(DeepPairing, SkippedFramesRunWithThePreviousMaskInsteadOfBeingDropped) {
  // Network at 15 Hz: every other frame has no mask of its own.
  const auto r = simulate(.06, 2);
  EXPECT_GE(r.own + r.previous, 57); // ~all 60 frames processed
  EXPECT_GE(r.own, 27);
  EXPECT_GE(r.previous, 27);
  for (size_t i = 1; i < r.order.size(); ++i)
    EXPECT_GT(r.order[i], r.order[i - 1]); // in order, never backwards
}
TEST(DeepPairing, NoMasksMeansHsvOnlyAtOnce) {
  DeepPairing p;
  DeepPairing::Frame f;
  DeepPairing::Mask m;
  bool own;
  p.addFrame(frame(1), 0);
  ASSERT_TRUE(p.next(true, 0, .3, .5, f, m, own));
  EXPECT_FALSE(m);
}
