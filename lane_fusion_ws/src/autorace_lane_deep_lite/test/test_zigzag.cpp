// Zigzag detector on real deep masks (car 06, bag sensor_20261119_184015, lane_640_mask model).
// Expected levels were checked with the Python prototype (lane_fusion_ws/tools/zigzag_review.py).
#include "autorace_lane_deep_lite/zigzag.hpp"
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {
using deep_lite::Config;
Config real() { return Config::load(std::string(DEEP_LITE_SOURCE_DIR) + "/config/real/lane.yaml"); }
cv::Mat mask(const std::string &name) {
  const auto m = cv::imread(std::string(DEEP_LITE_SOURCE_DIR) + "/test/data/" + name, cv::IMREAD_UNCHANGED);
  EXPECT_FALSE(m.empty()) << name;
  return m;
}
int level(const std::string &name) { return deep_lite::detectZigzag(mask(name), real()).level; }

TEST(Zigzag, NormalLinesAreNotZigzag) {
  EXPECT_EQ(level("zigzag_30_0.png"), 0);  // straight yellow lines
  EXPECT_EQ(level("zigzag_76_0.png"), 0);  // curve
  EXPECT_EQ(level("zigzag_126_0.png"), 0); // single sharp kink
}

TEST(Zigzag, CurveExitShowsOneSideFirst) { EXPECT_EQ(level("zigzag_114_6.png"), 1); }

TEST(Zigzag, ZoneShowsBothSides) {
  EXPECT_EQ(level("zigzag_115_3.png"), 2);
  EXPECT_EQ(level("zigzag_116_0.png"), 2);
}

TEST(Zigzag, EmptyAndInvalidMasks) {
  EXPECT_EQ(deep_lite::detectZigzag(cv::Mat::zeros(480, 640, CV_8U), real()).level, 0);
  EXPECT_THROW(deep_lite::detectZigzag(cv::Mat(), real()), std::invalid_argument);
  EXPECT_THROW(deep_lite::detectZigzag(cv::Mat::zeros(480, 640, CV_32F), real()), std::invalid_argument);
}

TEST(Zigzag, LowerResolutionMaskIsResized) {
  cv::Mat small;
  cv::resize(mask("zigzag_115_3.png"), small, {320, 240}, 0, 0, cv::INTER_NEAREST);
  EXPECT_EQ(deep_lite::detectZigzag(small, real()).level, 2);
}
} // namespace
