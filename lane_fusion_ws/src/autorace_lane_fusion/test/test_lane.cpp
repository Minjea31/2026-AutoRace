// Synthetic BEV-space checks with an identity-like BEV (full-frame trapezoid).
#include "autorace_lane_fusion/lane.hpp"
#include <cmath>
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
TEST(LaneLite, HeldPathMovesWithOdometry) {
  lane_lite::Pipeline p(identity());
  const auto a = p.process(scene(170, 300), 0);
  ASSERT_TRUE(a.valid);
  cv::Mat empty(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  p.move(.1, 0); // 10 cm forward along a straight lane: same lateral offset
  auto r = p.process(empty, .1);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "held");
  EXPECT_NEAR(r.lateral_m, a.lateral_m, .02);
  EXPECT_GT(r.center_path.back().y, a.center_path.back().y + 40); // far end slid toward the car
  p.move(0, .1); // turned left: the held path now lies to the right
  r = p.process(empty, .2);
  ASSERT_TRUE(r.valid);
  EXPECT_LT(r.lateral_m, a.lateral_m - .04);
  // Reacquisition blends with the held path instead of jumping.
  const auto back = p.process(scene(170, 300), .25);
  ASSERT_TRUE(back.valid);
  EXPECT_LT(back.lateral_m, a.lateral_m - .005);
}
TEST(LaneLite, RightBendWithOnlyTheLeftLineUsesTheWiderCurveLane) {
  // Right-hand bend: only the outer (left) line, turned right, is in view.
  // With curve_lane_width_m the centre lies further inside (right) than half
  // the straight width; a straight left line and a left bend are unchanged.
  auto run = [](double curve_m, int far_x) {
    auto c = identity();
    c.values["curve_lane_width_m"] = curve_m;
    lane_lite::Pipeline p(c);
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
    cv::line(img, {150, 479}, {far_x, 0}, cv::Scalar(0, 200, 230), 14);
    lane_lite::Result r;
    for (int i = 0; i < 30; ++i)
      r = p.process(img, i * .05);
    return r;
  };
  const double base = identity()["lane_width_m"];
  const auto off = run(0, 450), on = run(1.5 * base, 450);
  ASSERT_TRUE(off.valid && on.valid);
  EXPECT_EQ(on.state, "left only");
  EXPECT_GT(on.target_x, off.target_x + 20);
  EXPECT_LT(on.lateral_m, off.lateral_m);
  EXPECT_NEAR(run(1.5 * base, 170).target_x, run(0, 170).target_x, 1); // straight
}
TEST(LaneLite, AfterARightBendTheLaneJustDrivenIsNotTakenAsTheLeftLine) {
  // Right-hand bend: only the left line (turned right) is seen, the lane just
  // driven runs parallel further left. The left line then drops out: with
  // curve_memory the outer line is not taken as this lane's left boundary.
  auto run = [](double memory) {
    auto c = identity();
    c.values["curve_lane_width_m"] = 1.3 * c["lane_width_m"];
    c.values["curve_memory"] = memory;
    lane_lite::Pipeline p(c);
    auto frame = [](bool left) {
      cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
      if (left)
        cv::line(img, {200, 479}, {560, 0}, cv::Scalar(0, 200, 230), 14);
      cv::line(img, {0, 300}, {330, 0}, cv::Scalar(0, 200, 230), 14); // lane just driven
      return img;
    };
    double t = 0;
    lane_lite::Result r;
    for (; t < 1; t += .05) {
      p.move(.02, 0);
      r = p.process(frame(true), t);
    }
    EXPECT_EQ(r.state, "left only");
    const double lane = r.target_x;
    double worst = 0;
    for (; t < 2.5; t += .05) {
      p.move(1e-4, 0); // odometry running, car (nearly) still: the memory stays on the line
      r = p.process(frame(false), t);
      if (r.valid)
        worst = std::max(worst, lane - r.target_x); // > 0: pulled toward the outer line
    }
    return worst;
  };
  EXPECT_GT(run(0), 100);  // without the memory the outer line becomes the left boundary
  EXPECT_LT(run(2.), 20); // with it: no centre on the outer line
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
TEST(LaneLite, TrackedBoundaryKeepsItsRoleAcrossTheCarCentre) {
  // Zigzag / bend: the yellow LEFT boundary drifts to the right of the image
  // centre. Continuity keeps it the left boundary; screen x must not swap it.
  lane_lite::Pipeline p(identity());
  for (int i = 0; i < 18; ++i) {
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
    const int x = 200 + 15 * i; // up to 455 px, well right of 320
    cv::line(img, {x, 479}, {x + 20, 0}, cv::Scalar(0, 200, 230), 14);
    auto r = p.process(img, i * .05);
    ASSERT_TRUE(r.valid) << i;
    EXPECT_EQ(r.state, "left only") << i;
    EXPECT_NEAR(r.target_x, x + 10 + 150, 35) << i; // smoothing lags a moving line
  }
}
TEST(LaneLite, EitherSourceAloneGivesItsBoundary) {
  // HSV sees only the yellow left line (the white one is too dim for it);
  // the network sees only the white right line. Each detector's correct
  // boundary is used: both sides, and the source is never forced.
  cv::Mat img = scene(170, 300, true, false);
  cv::line(img, {470, 479}, {490, 0}, cv::Scalar(90, 90, 90), 14); // below white_v_min
  cv::Mat deep = cv::Mat::zeros(480, 640, CV_8U);
  cv::line(deep, {470, 479}, {490, 0}, 2, 14);
  lane_lite::Pipeline p(identity());
  lane_lite::Result r;
  for (int i = 0; i < 4; ++i)
    r = p.process(img, i * .05, true, deep);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "both");
  EXPECT_EQ(r.left.source, lane_lite::kHsv);
  EXPECT_EQ(r.right.source, lane_lite::kDeep);
  EXPECT_NEAR(r.target_x, 330, 12);
}
TEST(LaneLite, DeepAloneKeepsTheLaneWhenHsvSeesNothing) {
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  cv::Mat deep = cv::Mat::zeros(480, 640, CV_8U);
  cv::line(deep, {170, 479}, {190, 0}, 1, 14);
  cv::line(deep, {470, 479}, {490, 0}, 2, 14);
  lane_lite::Pipeline p(identity());
  lane_lite::Result r;
  for (int i = 0; i < 4; ++i)
    r = p.process(img, i * .05, false, deep);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "both");
  EXPECT_EQ(r.source, "deep");
  EXPECT_TRUE(r.hsv_used); // HSV always runs; it just found nothing
}
TEST(LaneLite, AgreeingSourcesConfirmEachOther) {
  cv::Mat deep = cv::Mat::zeros(480, 640, CV_8U);
  cv::line(deep, {170, 479}, {190, 0}, 1, 14);
  cv::line(deep, {470, 479}, {490, 0}, 2, 14);
  lane_lite::Pipeline p(identity());
  const auto r = p.process(scene(170, 300), 0, true, deep);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.left.source, lane_lite::kDeep | lane_lite::kHsv);
  EXPECT_EQ(r.right.source, lane_lite::kDeep | lane_lite::kHsv);
  EXPECT_EQ(r.candidates.size(), 2u);
}
TEST(LaneLite, OlderDeepMaskIsAlignedWithOdometryOrDropped) {
  // The mask was captured 0.1 s earlier; since then the car moved 5 cm left,
  // so the lines lay 27 px further left in the mask (0.05 m / (0.55/300)).
  // With the motion the mask lines up with this frame (both sources confirm
  // each line); without it a mask that old is not used at all.
  cv::Mat deep = cv::Mat::zeros(480, 640, CV_8U);
  cv::line(deep, {143, 479}, {163, 0}, 1, 14);
  cv::line(deep, {443, 479}, {463, 0}, 2, 14);
  for (const bool moved : {true, false}) {
    lane_lite::Pipeline p(identity());
    lane_lite::DeepInput in(deep);
    in.age = .1;
    in.moved = moved;
    in.dy = .05;
    const auto r = p.process(scene(170, 300), 0, false, in);
    ASSERT_TRUE(r.valid) << moved;
    EXPECT_EQ(r.deep_used, moved);
    EXPECT_EQ(r.source, moved ? "deep+hsv" : "hsv") << moved;
    EXPECT_EQ(r.left.source, moved ? lane_lite::kDeep | lane_lite::kHsv : lane_lite::kHsv) << moved;
  }
  lane_lite::Pipeline p(identity());
  lane_lite::DeepInput stale(deep);
  stale.age = .5; // beyond deep_max_age even with odometry
  stale.moved = true;
  EXPECT_FALSE(p.process(scene(170, 300), 0, false, stale).deep_used);
}
TEST(LaneLite, HsvSolidWhiteDoesNotReplaceDeepBoundary) {
  // Deep sees the yellow line and only a short piece of the white right one
  // (weak side -> HSV runs); HSV also sees a solid white line just outside
  // (outer road edge). The right boundary is the lane line, not the edge.
  lane_lite::Pipeline p(identity());
  cv::Mat img = scene(170, 300);
  cv::line(img, {560, 479}, {580, 0}, cv::Scalar(240, 240, 240), 14);
  cv::Mat deep = cv::Mat::zeros(480, 640, CV_8U);
  cv::line(deep, {170, 479}, {190, 0}, 1, 14);
  cv::line(deep, {470, 479}, {473, 380}, 2, 14);
  lane_lite::Result r;
  for (int i = 0; i < 4; ++i)
    r = p.process(img, i * .05, true, deep);
  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(r.hsv_used);
  EXPECT_EQ(r.state, "both");
  EXPECT_NEAR(r.target_x, 330, 12);
}
TEST(LaneLite, UncertainSingleSideGivesNoNewCentre) {
  // A single fresh line right under the vehicle centre: its role (left or
  // right) is not certain, so no centre path is produced from it.
  lane_lite::Pipeline p(identity());
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  cv::line(img, {325, 479}, {335, 0}, cv::Scalar(0, 200, 230), 14);
  const auto r = p.process(img, 0);
  EXPECT_FALSE(r.valid);
}
TEST(LaneLite, LaneShiftReachesTheOutputWithoutAnExtraGate) {
  // Accepted boundaries go straight to the centre path (no output hold):
  // with tracking unfiltered, a confirmed sideways shift shows up at once.
  auto c = identity();
  c.values["track_jump_px"] = 400;
  c.values["smoothing"] = 1;
  lane_lite::Pipeline p(c);
  lane_lite::Result r;
  for (int i = 0; i < 4; ++i)
    r = p.process(scene(170, 300), i * .05);
  const double before = r.lateral_m;
  r = p.process(scene(280, 300), .2);
  EXPECT_LT(r.lateral_m, before - .15);
}
TEST(LaneLite, CentreFollowsTheMiddleOfABend) {
  // Concentric arcs (right-hand bend), radii 650 / 350 around (820, 479):
  // the centre path must stay on radius 500, also at the lookahead point.
  lane_lite::Pipeline p(identity());
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  cv::ellipse(img, {820, 479}, {650, 650}, 0, 180, 250, cv::Scalar(0, 200, 230), 14);
  cv::ellipse(img, {820, 479}, {350, 350}, 0, 180, 270, cv::Scalar(240, 240, 240), 14);
  lane_lite::Result r;
  for (int i = 0; i < 3; ++i)
    r = p.process(img, i * .05);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "both");
  for (const auto &q : r.center_path)
    EXPECT_NEAR(std::hypot(q.x - 820, q.y - 479), 500, 15) << q;
  EXPECT_LT(r.heading, -.1); // turns right
}
// 3-lane road, lane 200 px: yellow road edges and white lane lines at x (BEV
// bottom), each leaning 20 px to the right at the top like scene().
cv::Mat road(std::vector<std::pair<int, bool>> lines, bool island = false) {
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  for (auto [x, yellow] : lines)
    cv::line(img, {x, 479}, {x + 20, 0}, yellow ? cv::Scalar(0, 200, 230) : cv::Scalar(240, 240, 240), 14);
  if (island) // the island tip straight ahead
    cv::fillConvexPoly(img, std::vector<cv::Point>{{330, 200}, {390, 120}, {390, 0}, {270, 0}, {270, 120}},
                       cv::Scalar(0, 200, 230));
  return img;
}
lane_lite::Config threeLanes() {
  auto c = identity();
  c.values["lane_width_px"] = 200;
  c.values["fork_shift"] = 1; // geometry tests: aim exactly at the target lane centre
  return c;
}
lane_lite::Result runFork(lane_lite::Pipeline &p, const cv::Mat &img, int sign, int frames = 10) {
  lane_lite::Result r;
  for (int i = 0; i < 4; ++i)
    r = p.process(img, i * .05);
  if (sign)
    p.forkSign(sign);
  for (int i = 4; i < 4 + frames; ++i)
    r = p.process(img, i * .05);
  return r;
}
TEST(LaneLite, ForkFromTheMiddleLaneMovesOneLaneToTheSign) {
  const auto img = road({{20, true}, {220, false}, {420, false}, {620, true}});
  for (const int sign : {-1, 1}) {
    lane_lite::Pipeline p(threeLanes());
    const auto r = runFork(p, img, sign);
    ASSERT_TRUE(r.valid) << sign;
    EXPECT_EQ(r.fork, sign);
    // A: left lane centre (x 130 at mid height), B: right lane centre (530).
    EXPECT_NEAR(r.target_x, sign < 0 ? 130 : 530, 30) << sign << " " << r.fork_state;
  }
}
TEST(LaneLite, ForkSteeringDrivesHalfALaneInsideTheTargetLaneLine) {
  // A from the middle lane; the target lane's left line is 40 px further in
  // (x 60) than the 1.5 lanes the odometry pose expects (x 20): the steer centre follows
  // the seen line (half a lane inside it), not the synthetic target.
  const auto img = road({{60, true}, {220, false}, {420, false}, {620, true}});
  lane_lite::Pipeline p(threeLanes());
  const auto r = runFork(p, img, -1, 6);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.state, "fork steer") << r.fork_state;
  EXPECT_NEAR(r.target_x, 60 + 100 + 10, 15) << r.fork_state; // line + half lane (+ lean at mid height)
}
TEST(LaneLite, ForkInTheOuterLaneOfTheSignKeepsTheLane) {
  // Already in the left lane (yellow road edge half a lane to the left): A keeps it.
  const auto img = road({{220, true}, {420, false}, {620, false}});
  lane_lite::Pipeline p(threeLanes());
  const auto r = runFork(p, img, -1);
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.target_x, 330, 25) << r.fork_state;
}
TEST(LaneLite, ForkWithoutSignTakesTheDefaultAtTheIsland) {
  lane_lite::Pipeline p(threeLanes()); // fork_default 1 = B
  const auto lanes = road({{20, true}, {220, false}, {420, false}, {620, true}});
  auto r = runFork(p, lanes, 0, 2);
  EXPECT_EQ(r.fork, 0); // no island, no sign: plain lane keeping
  EXPECT_NEAR(r.target_x, 330, 25);
  const auto island = road({{20, true}, {220, false}, {420, false}, {620, true}}, true);
  for (int i = 6; i < 18; ++i) // island seen: waits fork_default_wait (1.2 s) for a sign
    r = p.process(island, i * .05);
  EXPECT_EQ(r.fork, 0);
  for (int i = 18; i < 55; ++i)
    r = p.process(island, i * .05);
  EXPECT_EQ(r.fork, 1);
  EXPECT_NEAR(r.target_x, 530, 30) << r.fork_state;
}
TEST(LaneLite, ASignDuringTheIslandWaitWins) {
  lane_lite::Pipeline p(threeLanes());
  const auto island = road({{20, true}, {220, false}, {420, false}, {620, true}}, true);
  auto r = runFork(p, island, 0, 6); // island seen, default not yet taken
  EXPECT_EQ(r.fork, 0);
  p.forkSign(-1);
  for (int i = 10; i < 55; ++i)
    r = p.process(island, i * .05);
  EXPECT_EQ(r.fork, -1);
}
TEST(LaneLite, AFarSignSeenBeforeTheIslandReplacesTheDefault) {
  // The A sign is in view but still too small to act on (node: fork_sign_area)
  // when the island shows up: the island wait ends in A, not fork_default B.
  lane_lite::Pipeline p(threeLanes());
  p.forkSignSeen(-1);
  const auto island = road({{20, true}, {220, false}, {420, false}, {620, true}}, true);
  lane_lite::Result r;
  for (int i = 0; i < 55; ++i)
    r = p.process(island, i * .05);
  EXPECT_EQ(r.fork, -1) << r.fork_state;
}
TEST(LaneLite, ForkLateSignAtTheIslandStillChangesLanes) {
  // Middle lane; the B sign comes only when the right dashes have ended and
  // the island tip (yellow) sits half a lane to the right: the right boundary
  // was tracked as WHITE, so this is not the B lane and the car must change.
  lane_lite::Pipeline p(threeLanes());
  const auto lanes = road({{20, true}, {220, false}, {420, false}, {620, true}});
  auto r = runFork(p, lanes, 0, 2);
  auto late = road({{20, true}, {220, false}, {620, true}});
  cv::line(late, {400, 479}, {430, 0}, cv::Scalar(0, 200, 230), 14); // island edge where the dash was
  p.forkSign(1);
  for (int i = 6; i < 18; ++i)
    r = p.process(late, i * .05);
  EXPECT_EQ(r.fork, 1);
  EXPECT_EQ(r.fork_state.find("바깥 차로 유지"), std::string::npos) << r.fork_state;
}
TEST(LaneLite, ForkMiddleLaneWithTheIslandEdgeAsRightBoundaryStillGoesB) {
  // 11-10 car: middle lane, road edges outside the view, the right dash had
  // already turned into the yellow island edge when the B sign came; the old
  // logic took that for the B lane and steered to A. The right boundary was
  // white a moment ago, so B means: put that line on the car's LEFT.
  lane_lite::Pipeline p(threeLanes());
  auto r = runFork(p, road({{220, false}, {420, false}}), 0, 2);
  const auto island = road({{220, false}, {420, true}});
  for (int i = 6; i < 12; ++i) // the dash becomes the island edge (tracked as right)
    r = p.process(island, i * .05);
  p.forkSign(1);
  for (int i = 12; i < 24; ++i)
    r = p.process(island, i * .05);
  EXPECT_EQ(r.fork, 1);
  EXPECT_NEAR(r.target_x, 530, 30) << r.fork_state;
}
TEST(LaneLite, ForkSteeringHandsOverToLaneKeepingAfterTheSideStep) {
  // B from the middle lane: a synthetic target one lane to the right, held on
  // the ground by odometry; once the car has moved most of a lane sideways,
  // lane keeping in the branch takes over.
  lane_lite::Pipeline p(threeLanes());
  const auto img = road({{20, true}, {220, false}, {420, false}, {620, true}});
  auto r = runFork(p, img, 1, 2);
  EXPECT_EQ(r.state, "fork steer");
  EXPECT_NE(r.fork_state.find("조향"), std::string::npos) << r.fork_state;
  EXPECT_LT(r.lateral_m, -.4); // one lane (0.55 m) to the right
  p.move(.2, -.25, 0.); // half way: the target is closer now
  r = p.process(img, .35);
  EXPECT_EQ(r.state, "fork steer");
  EXPECT_NEAR(r.lateral_m, -.3, .06);
  p.move(.2, -.25, 0.); // 0.5 m sideways >= 85 % of a lane
  r = p.process(img, .4);
  EXPECT_NE(r.fork_state.find("분기 주행"), std::string::npos) << r.fork_state;
  EXPECT_NE(r.state, "fork steer");
}
TEST(LaneLite, ForkSteeringAimsBeyondTheLaneOnlyWithoutTheTargetLine) {
  // fork_shift 1.8 sets the synthetic aim (1.8 lanes over) used while the
  // target lane's line is not seen; the steering still ends after 85 % of ONE
  // lane sideways. With the line in view the centre is half a lane inside it.
  auto c = threeLanes();
  c.values["fork_shift"] = 1.8;
  {
    lane_lite::Pipeline p(c);
    const auto img = road({{20, true}, {220, false}, {420, false}}); // B lane's right line not seen
    auto r = runFork(p, img, 1, 2);
    EXPECT_NEAR(r.lateral_m, -1.8 * .55, .06);
    p.move(.4, -.5, 0.); // 0.5 m >= 85 % of 0.55 m
    r = p.process(img, .35);
    EXPECT_NE(r.fork_state.find("분기 주행"), std::string::npos) << r.fork_state;
  }
  lane_lite::Pipeline p(c);
  const auto r = runFork(p, road({{20, true}, {220, false}, {420, false}, {620, true}}), 1, 6);
  EXPECT_NEAR(r.lateral_m, -.55, .06) << r.state;
}
TEST(LaneLite, ForkFromTheFarLaneSteersTwoLanes) {
  // Left lane (yellow road edge on the left), sign B: two lanes to the right.
  lane_lite::Pipeline p(threeLanes());
  const auto r = runFork(p, road({{220, true}, {420, false}, {620, false}}), 1, 3);
  EXPECT_NE(r.fork_state.find("2차로"), std::string::npos) << r.fork_state;
  EXPECT_LT(r.lateral_m, -.9);
}
TEST(LaneLite, ForkDashRunningIntoTheYellowIslandIsStillALaneLine) {
  // 11-10 car snapshot: middle lane, the left dash is followed on into the
  // island's yellow edge far ahead, so most of its pixels are yellow. Its
  // NEAR half is white, so sign A still means "steer", not "outer lane".
  lane_lite::Pipeline p(threeLanes());
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  cv::line(img, {220, 479}, {226, 330}, cv::Scalar(240, 240, 240), 14); // near: white dash
  cv::line(img, {226, 330}, {240, 0}, cv::Scalar(0, 200, 230), 14);     // far: yellow island edge
  cv::line(img, {420, 479}, {440, 0}, cv::Scalar(240, 240, 240), 14);
  auto r = runFork(p, img, -1, 4);
  EXPECT_EQ(r.fork, -1);
  EXPECT_EQ(r.fork_state.find("바깥 차로 유지"), std::string::npos) << r.fork_state;
  EXPECT_GT(r.lateral_m, .4) << r.fork_state; // one lane to the left
}
TEST(LaneLite, ForkEndsAfterTheConfiguredDistance) {
  const auto img = road({{20, true}, {220, false}, {420, false}, {620, true}});
  lane_lite::Pipeline p(threeLanes());
  auto r = runFork(p, img, 1, 3);
  EXPECT_EQ(r.fork, 1);
  p.move(3.5, 0.); // beyond fork_length (3 m)
  r = p.process(img, 1.);
  EXPECT_EQ(r.fork, 0);
}
TEST(LaneLite, ConfigRoundTrip) {
  auto c = identity();
  c.values["merge_px"] = 12;
  const auto path = testing::TempDir() + "/lane_lite.yaml";
  c.save(path);
  EXPECT_EQ(lane_lite::Config::load(path)["merge_px"], 12);
}
