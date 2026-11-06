#pragma once
// Error-based PID lane keeping for the simulator.
//
//   e      = lateral_weight   * near lateral offset (m)
//          + heading_weight   * near path heading (rad)
//          + lookahead_weight * lookahead-point lateral offset (m)       (left +)
//   delta  = kp*e + ki*integral(e) + kd*filtered(de/dt) + curvature_ff*atan(L*kappa)
//
// Inputs come from /lane_fusion/target (lane perception). One parameter table is
// shared by the ROS node and the separate PID tuner window, so names, ranges
// and Korean descriptions never drift apart.
#include <map>
#include <string>
#include <vector>

namespace lane_pid {
struct Spec {
  const char *name, *group, *unit, *help;
  double value, min, max, step;
};
const std::vector<Spec> &specs();
const std::vector<const char *> &groups();
const Spec &spec(const std::string &name);

struct Config {
  std::map<std::string, double> values;
  Config(); // defaults from specs()
  double operator[](const std::string &key) const { return values.at(key); }
  // Throws std::invalid_argument for out-of-range or inconsistent values.
  void validate() const;
  // ROS 2 parameter file: <node>: ros__parameters: {...}. Unknown keys throw.
  static Config load(const std::string &path, const std::string &node = "lane_fusion_pid_controller");
  void save(const std::string &path, const std::string &node = "lane_fusion_pid_controller") const;
};

// One sample of /lane_fusion/target.
struct Measurement {
  bool valid = false;
  double lateral = 0, heading = 0, lookahead = 0, curvature = 0, confidence = 0; // target point
  double near_lateral = 0, near_heading = 0; // nearest centre-path point
  double time = 0; // acquisition/receipt time (s), monotonic per source
};

struct Terms {
  double error = 0, p = 0, i = 0, d = 0, ff = 0;
  double target_steering = 0; // saturated PID output
  double steering = 0, speed = 0; // after rate / acceleration limits
  bool tracking = false;          // last measurement usable
};

class Controller {
public:
  void reset();
  // Call exactly once per NEW measurement. dt comes from measurement times,
  // never from the output timer, so the D and I terms see real sampling.
  void measure(const Measurement &m, const Config &c);
  // Output stage at a fixed rate. fresh=false (timeout) stops immediately.
  const Terms &step(double dt, bool fresh, const Config &c);
  const Terms &terms() const { return t_; }

private:
  Terms t_;
  Measurement last_;
  bool have_prev_ = false;
  double prev_error_ = 0, prev_time_ = 0, d_filtered_ = 0, integral_ = 0;
  double near_lateral_ = 0, near_heading_ = 0, far_lateral_ = 0; // slew-limited inputs
};
} // namespace lane_pid
