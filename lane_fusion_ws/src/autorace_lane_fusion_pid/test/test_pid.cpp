#include "autorace_lane_fusion_pid/pid.hpp"
#include "autorace_lane_fusion_pid/parking_approach.hpp"
#include <cmath>
#include <cstdio>
#include <gtest/gtest.h>

using lane_pid::Config;
using lane_pid::Controller;
using lane_pid::Measurement;
using lane_pid::ParkingApproach;

TEST(ParkingApproach, DefaultsLimitToPointThreeForTenSecondsThenFiveSecondCooldown) {
  ParkingApproach gate;
  EXPECT_FALSE(gate.receive(true, 100.));
  ASSERT_TRUE(gate.receive(true, 100.1));
  EXPECT_DOUBLE_EQ(gate.limit(110.099, .8), .3);
  EXPECT_DOUBLE_EQ(gate.limit(110.1, .8), .8);
  for (int i = 0; i < 149; ++i)
    EXPECT_FALSE(gate.receive(true, 100.2 + i * .1));
  EXPECT_FALSE(gate.receive(true, 115.09));
  EXPECT_FALSE(gate.receive(true, 115.1));
  EXPECT_TRUE(gate.receive(true, 115.2));
  EXPECT_DOUBLE_EQ(gate.limit(115.3, .8), .3);
  EXPECT_DOUBLE_EQ(gate.limit(115.3, .2), .2);
}

TEST(ParkingApproach, TwoMessagesStartExactlyThreeSeconds) {
  ParkingApproach gate(2, .4, 3.);
  EXPECT_FALSE(gate.receive(true, 10.));
  EXPECT_DOUBLE_EQ(gate.limit(10., .8), .8);
  EXPECT_TRUE(gate.receive(true, 10.1));
  EXPECT_DOUBLE_EQ(gate.limit(13.099, .8), .4);
  EXPECT_DOUBLE_EQ(gate.limit(13.1, .8), .8);
}

TEST(ParkingApproach, FalseAndLongGapsResetConfirmation) {
  ParkingApproach gate(2, .4, 3.);
  EXPECT_FALSE(gate.receive(true, 1.));
  EXPECT_FALSE(gate.receive(false, 1.1));
  EXPECT_FALSE(gate.receive(true, 1.2));
  EXPECT_FALSE(gate.receive(true, 2.));
  EXPECT_TRUE(gate.receive(true, 2.1));
}

TEST(ParkingApproach, CooldownIgnoresOldMessagesAndRequiresTwoNewOnes) {
  ParkingApproach gate(2, .4, 3.);
  gate.receive(true, 1.);
  ASSERT_TRUE(gate.receive(true, 1.1));
  for (int i = 0; i < 79; ++i)
    EXPECT_FALSE(gate.receive(true, 1.2 + i * .1));
  EXPECT_FALSE(gate.active(4.1));
  EXPECT_FALSE(gate.receive(true, 9.09)); // still inside three-second slowdown plus five-second cooldown
  EXPECT_FALSE(gate.receive(true, 9.1));  // first NEW confirmation
  EXPECT_FALSE(gate.active(9.1));
  EXPECT_TRUE(gate.receive(true, 9.2));
  EXPECT_TRUE(gate.active(12.19));
  EXPECT_FALSE(gate.active(12.2));
}

TEST(ParkingApproach, FalseDoesNotCancelActiveSlowdown) {
  ParkingApproach gate(2, .4, 3.);
  gate.receive(true, 1.);
  gate.receive(true, 1.1);
  gate.receive(false, 1.2);
  EXPECT_TRUE(gate.active(4.));
  EXPECT_DOUBLE_EQ(gate.limit(2., .2), .2); // never accelerate a slower lane
  EXPECT_DOUBLE_EQ(gate.limit(2., 0.), 0.);
  EXPECT_FALSE(gate.receive(false, 10.));
  EXPECT_FALSE(gate.receive(true, 10.1));
  EXPECT_TRUE(gate.receive(true, 10.2));
}

TEST(ParkingApproach, InvalidSettingsAreRejected) {
  EXPECT_THROW(ParkingApproach(0), std::invalid_argument);
  EXPECT_THROW(ParkingApproach(2, -.1), std::invalid_argument);
  EXPECT_THROW(ParkingApproach(2, .4, 0.), std::invalid_argument);
  EXPECT_THROW(ParkingApproach(2, .4, 3., 0.), std::invalid_argument);
  EXPECT_THROW(ParkingApproach(2, .4, 3., .5, -1.), std::invalid_argument);
}

namespace {
Measurement sample(double lateral, double t, double heading = 0, double confidence = 1) {
  Measurement m;
  m.valid = true;
  m.near_lateral = lateral;
  m.near_heading = heading;
  m.confidence = confidence;
  m.time = t;
  return m;
}
Config plain(double kp, double ki = 0, double kd = 0) {
  Config c;
  c.values["kp"] = kp;
  c.values["ki"] = ki;
  c.values["kd"] = kd;
  c.values["lateral_weight"] = 1;
  c.values["heading_weight"] = 0;
  c.values["lookahead_weight"] = 0;
  c.values["d_filter_tau"] = 0;
  return c;
}
} // namespace

TEST(Pid, ParkingLimitBrakesImmediatelyAndRestoresWithAccelerationLimit) {
  Controller pid;
  auto c = plain(0.);
  c.values["speed"] = .8;
  c.values["min_speed"] = .5; // temporary cap must also override min_speed
  for (int i = 0; i < 30; ++i) { // fresh detections: no path-loss slowdown
    pid.measure(sample(0., i * .1), c);
    pid.step(.1, true, c);
  }
  EXPECT_DOUBLE_EQ(pid.terms().speed, .8);
  EXPECT_DOUBLE_EQ(pid.step(.02, true, c, .4).speed, .4);
  EXPECT_NEAR(pid.step(.02, true, c).speed, .41, 1e-9);
  EXPECT_DOUBLE_EQ(pid.step(.02, false, c, .4).speed, 0.);
  c.values["active"] = 0.;
  EXPECT_DOUBLE_EQ(pid.step(.02, true, c, .4).speed, 0.);
}

TEST(Pid, DefaultsAreValidAndEveryParameterHasHelp) {
  EXPECT_NO_THROW(Config().validate());
  for (const auto &s : lane_pid::specs()) {
    EXPECT_GT(std::string(s.help).size(), 5u) << s.name;
    EXPECT_LE(s.min, s.value);
    EXPECT_LE(s.value, s.max);
  }
}

TEST(Pid, LeftErrorSteersLeftProportionally) {
  Controller pid;
  const auto c = plain(2.0);
  pid.measure(sample(.05, 0), c);
  EXPECT_NEAR(pid.terms().p, .10, 1e-9);
  EXPECT_NEAR(pid.terms().target_steering, .10, 1e-9);
  Controller right;
  right.measure(sample(-.05, 0), c);
  EXPECT_LT(right.terms().target_steering, 0);
}

TEST(Pid, ErrorCombinesNearHeadingAndLookahead) {
  Controller pid;
  auto c = plain(1.0);
  c.values["heading_weight"] = .5;
  c.values["lookahead_weight"] = .25;
  auto m = sample(.02, 0, .2);
  m.lateral = .4;
  pid.measure(m, c);
  EXPECT_NEAR(pid.terms().error, .02 + .1 + .1, 1e-9);
}

TEST(Pid, DerivativeUsesMeasurementTime) {
  Controller pid;
  const auto c = plain(0, 0, 1.0);
  pid.measure(sample(0, 0), c);
  EXPECT_EQ(pid.terms().d, 0);
  pid.measure(sample(.01, .1), c); // 0.1 m/s
  EXPECT_NEAR(pid.terms().d, .1, 1e-9);
}

TEST(Pid, IntegralAccumulatesAndIsClamped) {
  Controller pid;
  auto c = plain(0, 1.0);
  c.values["integral_limit"] = .05;
  for (int i = 0; i <= 100; ++i)
    pid.measure(sample(.1, i * .05), c);
  EXPECT_NEAR(pid.terms().i, .05, 1e-9);
}

TEST(Pid, NoWindupWhileSaturated) {
  Controller pid;
  auto c = plain(10, 1.0); // kp alone saturates
  c.values["integral_limit"] = 2;
  for (int i = 0; i <= 50; ++i)
    pid.measure(sample(.2, i * .05), c);
  EXPECT_NEAR(pid.terms().target_steering, c["max_steering"], 1e-9);
  EXPECT_NEAR(pid.terms().i, 0, 1e-9);
}

TEST(Pid, PathLossIsHeldThenSlowedThenStopped) {
  Controller pid;
  auto c = plain(1, 1, 1);
  c.values["speed"] = .5;
  c.values["recover_speed"] = .15;
  for (int i = 0; i < 40; ++i) {
    pid.measure(sample(.1, i * .05), c);
    pid.step(.05, true, c);
  }
  const double cruise = pid.terms().speed, steer = pid.terms().target_steering;
  ASSERT_GT(cruise, .3);
  Measurement lost; // perception: no centre path this frame
  lost.time = 2.0;
  pid.measure(lost, c);
  pid.step(.05, true, c); // one lost frame: no stop, steering kept
  EXPECT_NEAR(pid.terms().speed, cruise, 1e-9);
  EXPECT_NEAR(pid.terms().target_steering, steer, 1e-9);
  for (int i = 0; i < 4; ++i) // 0.25 s: recovery speed
    pid.step(.05, true, c);
  EXPECT_LE(pid.terms().speed, .15 + 1e-9);
  EXPECT_GT(pid.terms().speed, 0);
  for (int i = 0; i < 6; ++i) // 0.55 s: stop, I/D history cleared
    pid.step(.05, true, c);
  EXPECT_EQ(pid.terms().speed, 0);
  pid.measure(sample(.1, 2.6), c);
  EXPECT_EQ(pid.terms().i, 0);
  EXPECT_GT(pid.step(.05, true, c).speed, 0); // reacquired: drives again
}

TEST(Pid, HeldPathSteersButCountsAsLoss) {
  Controller pid;
  auto c = plain(1);
  pid.measure(sample(0, 0), c);
  pid.step(.05, true, c);
  auto held = sample(.2, .05);
  held.detected = false; // perception holding its last path
  for (int i = 0; i < 8; ++i) {
    held.time = .05 * (i + 1);
    pid.measure(held, c);
    pid.step(.05, true, c);
  }
  EXPECT_GT(pid.terms().target_steering, 0); // follows the held path
  EXPECT_LE(pid.terms().speed, c["recover_speed"] + 1e-9);
  for (int i = 0; i < 3; ++i) {
    held.time += .05;
    pid.measure(held, c);
    pid.step(.05, true, c);
  }
  EXPECT_EQ(pid.terms().speed, 0); // > recover_time without a detection
}

TEST(Pid, TimeoutStopsImmediatelyAndSteeringIsRateLimited) {
  Controller pid;
  auto c = plain(10);
  c.values["steering_rate"] = 1.0;
  pid.measure(sample(.2, 0), c);
  EXPECT_NEAR(pid.step(.05, true, c).steering, .05, 1e-9);
  EXPECT_GT(pid.step(.05, true, c).speed, 0);
  EXPECT_EQ(pid.step(.05, false, c).speed, 0);
}

TEST(Pid, AccelerationLimitedAndActiveSwitch) {
  Controller pid;
  auto c = plain(0);
  c.values["acceleration"] = 1.0;
  pid.measure(sample(0, 0), c);
  EXPECT_NEAR(pid.step(.05, true, c).speed, .05, 1e-9);
  c.values["active"] = 0;
  EXPECT_EQ(pid.step(.05, true, c).speed, 0);
}

TEST(Pid, YamlRoundTripKeepsDoubles) {
  Config c;
  c.values["kp"] = 2.25;
  const std::string path = ::testing::TempDir() + "pid_roundtrip.yaml";
  c.save(path);
  EXPECT_EQ(Config::load(path).values, c.values);
  std::FILE *f = std::fopen(path.c_str(), "r");
  char buffer[4096] = {};
  const size_t n = std::fread(buffer, 1, sizeof(buffer) - 1, f);
  std::fclose(f);
  EXPECT_NE(std::string(buffer, n).find("active: 1.0"), std::string::npos);
}

TEST(Pid, RealYamlNodeOptionsAreKept) {
  // config/real/pid.yaml carries parking_calibration_stop (a node option, not
  // a gain): loading must not fail and saving must write it back unchanged.
  const auto path = testing::TempDir() + "/pid_options.yaml";
  {
    lane_pid::Config c;
    c.options["parking_calibration_stop"] = "true";
    c.save(path);
  }
  const auto c = lane_pid::Config::load(path);
  ASSERT_EQ(c.options.count("parking_calibration_stop"), 1u);
  EXPECT_EQ(c.options.at("parking_calibration_stop"), "true");
  EXPECT_DOUBLE_EQ(c["kp"], lane_pid::Config()["kp"]);
}
TEST(Pid, RejectsInconsistentConfig) {
  Config c;
  c.values["confidence_stop"] = .6;
  c.values["confidence_full"] = .5;
  EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST(Pid, SingleFrameGlitchIsSlewLimited) {
  Controller pid;
  auto c = plain(1.0);
  c.values["heading_weight"] = 1.0;
  c.values["heading_rate_limit"] = 2.0;
  pid.measure(sample(0, 0, 0), c);
  pid.measure(sample(0, .05, -1.6), c); // impossible jump in 50 ms
  EXPECT_NEAR(pid.terms().error, -.1, 1e-9);
}

TEST(Pid, FarHeadingKeepsSteeringInsideACurve) {
  // Centred on the path, but the path ahead turns left: only the far heading
  // term (0 = off, the previous behaviour) produces a left steering target.
  Controller off, on;
  auto c = plain(1);
  auto m = sample(0, 0);
  m.heading = .3;
  off.measure(m, c);
  EXPECT_NEAR(off.terms().target_steering, 0, 1e-9);
  c.values["far_heading_weight"] = .5;
  on.measure(m, c);
  EXPECT_NEAR(on.terms().error, .15, 1e-9);
  EXPECT_GT(on.terms().target_steering, 0);
}

TEST(ParkingApproach, CalibrationStopLatchesUntilExplicitMatchingReady) {
  ParkingApproach gate(2,.3,10.,.5,5.,true);
  EXPECT_FALSE(gate.receive(true,1.)); EXPECT_TRUE(gate.receive(true,1.1));
  for(int i=0;i<100;++i) {
    EXPECT_FALSE(gate.receive(true,2.+i));
    EXPECT_DOUBLE_EQ(gate.limit(2.+i,.8),0.);
  }
  EXPECT_TRUE(gate.ready(102.)); EXPECT_FALSE(gate.ready(103.));
  EXPECT_DOUBLE_EQ(gate.limit(111.9,.8),.3);
  EXPECT_DOUBLE_EQ(gate.limit(112.,.8),.3);
  gate.release();
  EXPECT_DOUBLE_EQ(gate.limit(112.1,.8),.8);
}

TEST(ParkingApproach, SamePersistentSignCannotRestartCalibrationAfterCooldown) {
  ParkingApproach gate(2,.3,10.,.5,5.,true);
  EXPECT_FALSE(gate.receive(true,1.)); EXPECT_TRUE(gate.receive(true,1.1));
  EXPECT_TRUE(gate.ready(4.));
  for (int i=0;i<200;++i) EXPECT_FALSE(gate.receive(true,5.+i));
  gate.release();
  EXPECT_FALSE(gate.receive(false,205.));
  EXPECT_FALSE(gate.receive(true,205.1)); // One false frame is insufficient.
  EXPECT_FALSE(gate.receive(false,206.));
  EXPECT_FALSE(gate.receive(false,206.6));
  EXPECT_FALSE(gate.receive(true,207.));
  EXPECT_TRUE(gate.receive(true,207.1));
}
