#include "autorace_lane_fusion_pid/pid.hpp"
#include <cmath>
#include <cstdio>
#include <gtest/gtest.h>

using lane_pid::Config;
using lane_pid::Controller;
using lane_pid::Measurement;

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

TEST(Pid, LostLaneStopsAndResetsHistory) {
  Controller pid;
  auto c = plain(1, 1, 1);
  for (int i = 0; i < 20; ++i) {
    pid.measure(sample(.1, i * .05), c);
    pid.step(.05, true, c);
  }
  EXPECT_GT(pid.terms().speed, 0);
  Measurement lost;
  lost.time = 1.0;
  pid.measure(lost, c);
  EXPECT_EQ(pid.terms().i, 0);
  EXPECT_EQ(pid.step(.02, true, c).speed, 0);
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
