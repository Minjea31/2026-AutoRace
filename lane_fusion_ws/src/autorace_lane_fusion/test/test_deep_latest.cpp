// Mask choice for a frame: own mask, else the newest earlier one within max_age.
#include "autorace_lane_fusion/deep_latest.hpp"
#include <gtest/gtest.h>

namespace {
sensor_msgs::msg::Image::ConstSharedPtr mask(int64_t ns) {
  auto m = std::make_shared<sensor_msgs::msg::Image>();
  m->header.stamp.sec = int32_t(ns / 1000000000);
  m->header.stamp.nanosec = uint32_t(ns % 1000000000);
  m->encoding = "mono8";
  m->width = m->height = m->step = 4;
  m->data.assign(16, 0);
  return m;
}
constexpr int64_t kMs = 1000000;
} // namespace

TEST(DeepLatest, OwnMaskPreferred) {
  lane_lite::DeepLatest d;
  d.addMask(mask(1000 * kMs), 0);
  d.addMask(mask(1033 * kMs), 0);
  double age;
  bool own;
  auto m = d.pick(1033 * kMs, .12, age, own);
  ASSERT_TRUE(m);
  EXPECT_TRUE(own);
  EXPECT_DOUBLE_EQ(age, 0);
}
TEST(DeepLatest, NewestEarlierMaskWithinAge) {
  lane_lite::DeepLatest d;
  d.addMask(mask(1000 * kMs), 0);
  d.addMask(mask(1033 * kMs), 0);
  double age;
  bool own;
  auto m = d.pick(1100 * kMs, .12, age, own); // frame newer than both masks
  ASSERT_TRUE(m);
  EXPECT_FALSE(own);
  EXPECT_NEAR(age, .067, 1e-6);
}
TEST(DeepLatest, StaleMaskIsNotUsed) {
  lane_lite::DeepLatest d;
  d.addMask(mask(1000 * kMs), 0);
  double age;
  bool own;
  EXPECT_FALSE(d.pick(1200 * kMs, .12, age, own));
  EXPECT_NEAR(age, .2, 1e-6);
}
TEST(DeepLatest, FutureMaskIsNotUsed) {
  lane_lite::DeepLatest d;
  d.addMask(mask(1100 * kMs), 0);
  double age;
  bool own;
  EXPECT_FALSE(d.pick(1000 * kMs, .12, age, own));
}
TEST(DeepLatest, MalformedMaskIgnored) {
  lane_lite::DeepLatest d;
  auto m = std::make_shared<sensor_msgs::msg::Image>(*mask(0));
  m->encoding = "rgb8";
  EXPECT_FALSE(d.addMask(m, 0));
}
