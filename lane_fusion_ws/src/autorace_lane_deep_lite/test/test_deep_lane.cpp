// Synthetic class masks drawn in BEV and warped back to the camera view.
// Real-car config (lane_ws values): 277 px lane = 0.52 m.
#include "autorace_lane_deep_lite/deep_lane.hpp"
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

namespace {
using deep_lite::Config;
Config real() { return Config::load(std::string(DEEP_LITE_SOURCE_DIR) + "/config/real/lane.yaml"); }
using deep_lite::LineState;
using deep_lite::Pipeline;

cv::Mat toCamera(const cv::Mat &bev, const Config &c) {
  const int w = c["width"], h = c["height"];
  const std::vector<cv::Point2f> src = {
      {float(c["bev_top_left"] * (w - 1)), float(c["bev_top_y"] * (h - 1))},
      {float(c["bev_top_right"] * (w - 1)), float(c["bev_top_y"] * (h - 1))},
      {float(c["bev_bottom_right"] * (w - 1)), float(c["bev_bottom_y"] * (h - 1))},
      {float(c["bev_bottom_left"] * (w - 1)), float(c["bev_bottom_y"] * (h - 1))}};
  const std::vector<cv::Point2f> dst = {
      {0, 0}, {float(w - 1), 0}, {float(w - 1), float(h - 1)}, {0, float(h - 1)}};
  cv::Mat cam;
  cv::warpPerspective(bev, cam, cv::getPerspectiveTransform(dst, src), {w, h}, cv::INTER_NEAREST,
                      cv::BORDER_CONSTANT, 0);
  return cam;
}
// Vertical line at x, solid or dashed (dash/gap in px).
void vertical(cv::Mat &bev, int x, int cls, int dash = 0, int gap = 0) {
  for (int y = bev.rows - 1; y >= 0; y -= dash ? dash + gap : bev.rows)
    cv::line(bev, {x, y}, {x, dash ? std::max(0, y - dash) : 0}, cls, 11);
}
deep_lite::Result run(Pipeline &p, const cv::Mat &cam, int frames = 6) {
  deep_lite::Result r;
  for (int i = 0; i < frames; ++i)
    r = p.process(cam, i * .05, true);
  return r;
}
} // namespace

TEST(DeepLite, StraightYellowAndDashedWhite) {
  const Config c = real();
  Pipeline p(c);
  cv::Mat bev = cv::Mat::zeros(480, 640, CV_8U);
  vertical(bev, 320 - 138, 1);
  vertical(bev, 320 + 138, 2, 107, 80);
  const auto r = run(p, toCamera(bev, c));
  EXPECT_EQ(r.state, "both");
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.lateral_m, 0, .02);
  EXPECT_NEAR(r.curvature, 0, .1);
  EXPECT_NEAR(r.lane_width_px, 277, 15);
}

TEST(DeepLite, SingleLineOffsetsByHalfLane) {
  const Config c = real();
  Pipeline p(c);
  cv::Mat bev = cv::Mat::zeros(480, 640, CV_8U);
  vertical(bev, 320 + 138, 2, 107, 80); // only the right dashed line
  const auto r = run(p, toCamera(bev, c));
  EXPECT_EQ(r.state, "right only");
  EXPECT_NEAR(r.lateral_m, 0, .03);
}

TEST(DeepLite, PicksOwnLaneBesideAnotherLine) {
  const Config c = real();
  Pipeline p(c);
  cv::Mat bev = cv::Mat::zeros(480, 640, CV_8U);
  vertical(bev, 320 - 138, 1);
  vertical(bev, 320 + 138, 1);
  vertical(bev, 320 - 138 - 277, 2, 107, 80); // neighbouring lane (mostly off-screen)
  const auto r = run(p, toCamera(bev, c));
  EXPECT_EQ(r.state, "both");
  EXPECT_NEAR(r.lateral_m, 0, .02);
}

TEST(DeepLite, LeftBendHasPositiveCurvature) {
  const Config c = real();
  Pipeline p(c);
  cv::Mat bev = cv::Mat::zeros(480, 640, CV_8U);
  // Bend to the left: centre of the circle to the left of the vehicle.
  const cv::Point centre(320 - 700, 480);
  cv::ellipse(bev, centre, {700 - 138, 700 - 138}, 0, -60, 0, 1, 11);
  cv::ellipse(bev, centre, {700 + 138, 700 + 138}, 0, -60, 0, 1, 11);
  const auto r = run(p, toCamera(bev, c));
  ASSERT_TRUE(r.valid);
  EXPECT_GT(r.curvature, .3); // radius 700 px * 1.88 mm/px ~ 1.3 m -> 0.76 1/m
  EXPECT_LT(r.curvature, 1.5);
}

TEST(DeepLite, EmptyMaskIsLost) {
  const Config c = real();
  Pipeline p(c);
  const auto r = run(p, cv::Mat::zeros(480, 640, CV_8U));
  EXPECT_FALSE(r.valid);
  EXPECT_EQ(r.left.state, LineState::Lost);
  EXPECT_EQ(r.right.state, LineState::Lost);
}

TEST(DeepLite, ShortDropoutIsHeldThenLost) {
  // lane_ws tracking: a lost mask keeps the last path for track_memory.
  const Config c = real();
  Pipeline p(c);
  cv::Mat bev = cv::Mat::zeros(480, 640, CV_8U);
  vertical(bev, 320 - 138, 1);
  vertical(bev, 320 + 138, 2, 107, 80);
  run(p, toCamera(bev, c));
  const cv::Mat empty = cv::Mat::zeros(480, 640, CV_8U);
  EXPECT_EQ(p.process(empty, .35).state, "held");
  EXPECT_FALSE(p.process(empty, .35 + c["track_memory"] + .1).valid);
}

TEST(DeepLite, ConfigKeepsOnlyMaskKeysAndRoundTrips) {
  for (const auto &s : deep_lite::specs())
    EXPECT_NE(std::string(s.name).rfind("yellow_h", 0), 0u) << s.name;
  const std::string path = testing::TempDir() + "deep_lite_roundtrip.yaml";
  real().save(path);
  EXPECT_EQ(Config::load(path).values, real().values);
}
