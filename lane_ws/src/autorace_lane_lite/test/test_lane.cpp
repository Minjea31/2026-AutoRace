// Synthetic BEV-space checks with an identity-like BEV (full-frame trapezoid).
#include "autorace_lane_lite/lane.hpp"
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

namespace {
lane_lite::Config identity() {
  lane_lite::Config c;
  for (auto [k, v] : std::map<std::string, double>{
           {"roi_top", 0}, {"bev_top_y", 0}, {"bev_top_left", 0}, {"bev_top_right", 1},
           {"bev_bottom_left", 0}, {"bev_bottom_right", 1}, {"lane_width_px", 300},
           {"bev_near_m", .2}, {"bev_far_m", 1.2}})
    c.values[k] = v;
  return c;
}
// Dark floor, yellow left at xl, white right at xl + width; optional glare.
cv::Mat scene(int xl, int width, bool left = true, bool right = true, bool glare = false) {
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  if (glare)
    cv::circle(img, {330, 250}, 110, cv::Scalar(235, 235, 235), cv::FILLED);
  if (left)
    cv::line(img, {xl, 479}, {xl + 20, 0}, cv::Scalar(0, 200, 230), 14);
  if (right)
    cv::line(img, {xl + width, 479}, {xl + width + 20, 0}, cv::Scalar(240, 240, 240), 14);
  return img;
}
} // namespace

TEST(LaneLite, BothBoundariesGiveCentredTarget) {
  lane_lite::Pipeline p(identity());
  auto r = p.process(scene(170, 300), 0);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "both");
  EXPECT_NEAR(r.target_x, 330, 8);
}
TEST(LaneLite, GlareBlobIsNotAMarking) {
  lane_lite::Pipeline p(identity());
  auto r = p.process(scene(170, 300, true, true, true), 0);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "both");
  EXPECT_NEAR(r.target_x, 330, 10);
}
TEST(LaneLite, SingleBoundaryOffsetsByLaneWidth) {
  lane_lite::Pipeline p(identity());
  auto r = p.process(scene(170, 300, true, false), 0);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "left only");
  EXPECT_NEAR(r.target_x, 330, 12);
  EXPECT_GT(r.lateral_m, -0.05);
}
TEST(LaneLite, ShortDropoutIsHeldThenLost) {
  lane_lite::Pipeline p(identity());
  p.process(scene(170, 300), 0);
  cv::Mat empty(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  EXPECT_EQ(p.process(empty, .1).state, "held");
  EXPECT_FALSE(p.process(empty, .7).valid);
}
TEST(LaneLite, OneFrameJumpIsRejected) {
  lane_lite::Pipeline p(identity());
  for (int i = 0; i < 5; ++i)
    p.process(scene(170, 300), i * .05);
  auto r = p.process(scene(260, 300), .25); // both lines jump 90 px at once
  EXPECT_LT(std::abs(r.target_x - 330), 25);
}
TEST(LaneLite, TransverseBendIsFollowed) {
  // Only the outer boundary of a right-hand hairpin: it starts vertical on
  // the left and turns horizontal ahead (the x(y)-only failure case).
  lane_lite::Pipeline p(identity());
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  cv::ellipse(img, {560, 479}, {400, 360}, 0, 180, 270, cv::Scalar(0, 200, 230), 14);
  auto r = p.process(img, 0);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "left only");
  EXPECT_LT(r.heading, -0.3); // the path turns right
  EXPECT_LT(r.lateral_m, 0);
}
TEST(LaneLite, DashedBoundaryIsTrackedNearToFar) {
  // 3-lane section: only a white dashed line on the right (dash 80 px, gap
  // 60 px = 1.5 windows), its phase scrolling as the car drives forward.
  lane_lite::Pipeline p(identity());
  for (int i = 0; i < 20; ++i) {
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
    for (int y = 479 + (i * 23) % 140; y > -80; y -= 140)
      cv::line(img, {470, y}, {470, y - 80}, cv::Scalar(240, 240, 240), 14);
    auto r = p.process(img, i * .05);
    ASSERT_TRUE(r.valid) << i;
    EXPECT_EQ(r.state, "right only") << i;
    EXPECT_LT(std::abs(r.heading), .1) << i; // a reversed path gives ~pi
    EXPECT_NEAR(r.target_x, 320, 15) << i;
  }
}
TEST(LaneLite, StopLineIsNeverABoundary) {
  // Yellow left boundary; a white stop line approaches across the lane. The
  // right side must never lock onto it, also when it creeps in over frames.
  lane_lite::Pipeline p(identity());
  for (int i = 0; i < 20; ++i) {
    cv::Mat img = scene(170, 300, true, false);
    const int y = 100 + i * 18;
    cv::line(img, {150, y}, {640, y + 10}, cv::Scalar(240, 240, 240), 14);
    auto r = p.process(img, i * .05);
    ASSERT_TRUE(r.valid) << i;
    EXPECT_NE(r.right.state, lane_lite::LineState::Detected) << i;
    EXPECT_LT(std::abs(r.heading), .2) << i;
    EXPECT_LT(std::abs(r.near_heading), .2) << i;
  }
}
TEST(LaneLite, CrosswalkStripesAreRemoved) {
  // Thick white stripes inside the lane; their long edges must not pair with
  // a boundary or bend the centre path.
  auto c = identity();
  c.values["white_max_width"] = 44;
  lane_lite::Pipeline p(c);
  cv::Mat img = scene(170, 300);
  for (int x = 200; x < 450; x += 90)
    cv::rectangle(img, {x, 150}, {x + 55, 400}, cv::Scalar(240, 240, 240), cv::FILLED);
  lane_lite::Result r;
  for (int i = 0; i < 5; ++i)
    r = p.process(img, i * .05);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "both");
  EXPECT_NEAR(r.target_x, 330, 10);
  EXPECT_LT(std::abs(r.heading), .15);
}
TEST(LaneLite, ThickDashesSurviveWhiteMaxWidth) {
  // Real-car 3-lane dashes are ~30 px thick in BEV; the crosswalk filter
  // must keep them (white_max_width 24 erased them on the bag).
  auto c = identity();
  c.values["white_max_width"] = 44;
  lane_lite::Pipeline p(c);
  for (int i = 0; i < 10; ++i) {
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
    for (int y = 479 + (i * 23) % 140; y > -80; y -= 140)
      cv::line(img, {470, y}, {470, y - 80}, cv::Scalar(240, 240, 240), 30);
    auto r = p.process(img, i * .05);
    ASSERT_TRUE(r.valid) << i;
    EXPECT_EQ(r.state, "right only") << i;
    EXPECT_NEAR(r.target_x, 320, 15) << i;
  }
}
TEST(LaneLite, SolidWhiteIsIgnoredDashedWhiteKept) {
  // Only white DASHED lines are lane boundaries: a continuous white line
  // (outer road edge) is removed by white_max_length, dashes stay.
  auto c = identity();
  c.values["white_max_length"] = 320;
  lane_lite::Pipeline solid(c), dashed(c);
  lane_lite::Result rs, rd;
  for (int i = 0; i < 5; ++i) {
    rs = solid.process(scene(170, 300), i * .05); // full-height white line
    cv::Mat img = scene(170, 300, true, false);
    for (int y = 479 + (i * 23) % 140; y > -80; y -= 140)
      cv::line(img, {470, y}, {470 + 4, y - 80}, cv::Scalar(240, 240, 240), 14);
    rd = dashed.process(img, i * .05);
  }
  ASSERT_TRUE(rs.valid);
  EXPECT_EQ(rs.state, "left only");
  EXPECT_NEAR(rs.target_x, 330, 12); // still centred from the yellow line
  ASSERT_TRUE(rd.valid);
  EXPECT_EQ(rd.state, "both");
}
TEST(LaneLite, YellowPairSkipsAngleLimitWhitePairDoesNot) {
  // In a tight bend the two yellow boundaries are not parallel in BEV; the
  // angle test (meant for white crosswalk / stop-line fragments) must not
  // drop the inner one. A white right line with the same shape is dropped.
  auto c = identity();
  c.values["max_angle_deg"] = 5;
  for (const bool white : {false, true}) {
    lane_lite::Pipeline p(c);
    cv::Mat img = scene(170, 300, true, false);
    cv::line(img, {470, 479}, {560, 0},
             white ? cv::Scalar(240, 240, 240) : cv::Scalar(0, 200, 230), 14);
    lane_lite::Result r;
    for (int i = 0; i < 5; ++i)
      r = p.process(img, i * .05);
    ASSERT_TRUE(r.valid) << white;
    EXPECT_EQ(r.state, white ? "left only" : "both") << white;
  }
}
TEST(LaneLite, NoViewsGivesSameResult) {
  // The driving node skips display images; the measurement must not change.
  lane_lite::Pipeline a(identity()), b(identity());
  for (int i = 0; i < 5; ++i) {
    const auto img = scene(170 + 4 * i, 300);
    const auto ra = a.process(img, i * .05), rb = b.process(img, i * .05, false);
    ASSERT_EQ(ra.valid, rb.valid) << i;
    EXPECT_EQ(ra.state, rb.state) << i;
    EXPECT_DOUBLE_EQ(ra.lateral_m, rb.lateral_m) << i;
    EXPECT_DOUBLE_EQ(ra.heading, rb.heading) << i;
    EXPECT_DOUBLE_EQ(ra.near_lateral_m, rb.near_lateral_m) << i;
    EXPECT_TRUE(rb.path_view.empty() && rb.overlay.empty()) << i;
    EXPECT_FALSE(ra.path_view.empty()) << i;
  }
}
TEST(LaneLite, ConfigRoundTrip) {
  auto c = identity();
  c.values["margin"] = 61;
  const auto path = testing::TempDir() + "/lane_lite.yaml";
  c.save(path);
  EXPECT_EQ(lane_lite::Config::load(path)["margin"], 61);
}
