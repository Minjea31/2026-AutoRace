#pragma once
// Error-based PID lane keeping for the simulator.
//
//   e      = lateral_weight   * near lateral offset (m)
//          + heading_weight   * near path heading (rad)
//          + lookahead_weight * lookahead-point lateral offset (m)       (left +)
//          + far_heading_weight * path angle at the lookahead point (rad) (left +)
//   delta  = kp*e + ki*integral(e) + kd*filtered(de/dt) + curvature_ff*atan(L*kappa)
//
// Inputs come from /lane_fusion/target (lane perception). One parameter table is
// shared by the ROS node and the separate PID tuner window, so names, ranges
// and Korean descriptions never drift apart.
#include <map>
#include <limits>
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
  // Node options in the same file that are not tuner gains (e.g.
  // parking_calibration_stop): kept as written and saved back unchanged.
  std::map<std::string, std::string> options;
  Config(); // defaults from specs()
  double operator[](const std::string &key) const { return values.at(key); }
  // Throws std::invalid_argument for out-of-range or inconsistent values.
  void validate() const;
  // ROS 2 parameter file: <node>: ros__parameters: {...}. Numeric keys must be
  // specs; other scalars (bool / string node options) go to `options`.
  static Config load(const std::string &path, const std::string &node = "lane_fusion_pid_controller");
  void save(const std::string &path, const std::string &node = "lane_fusion_pid_controller") const;
};

// One sample of /lane_fusion/target.
struct Measurement {
  bool valid = false;
  double lateral = 0, heading = 0, lookahead = 0, curvature = 0, confidence = 0; // target point
  double near_lateral = 0, near_heading = 0; // nearest centre-path point
  double time = 0; // acquisition/receipt time (s), monotonic per source
  // At least one boundary DETECTED this frame (false: perception is holding
  // its last centre path). Older perception output without states: = valid.
  bool detected = true;
};

struct Terms {
  double error = 0, p = 0, i = 0, d = 0, ff = 0;
  double target_steering = 0; // saturated PID output
  double steering = 0, speed = 0; // after rate / acceleration limits
  bool tracking = false;          // driving on a path (detected or held/recovering)
  double lost = 0;                // s since the last good detection
};

class Controller {
public:
  void reset();
  // Call exactly once per NEW measurement. dt comes from measurement times,
  // never from the output timer, so the D and I terms see real sampling.
  void measure(const Measurement &m, const Config &c);
  // Output stage at a fixed rate. Path loss is staged by time since the last
  // good detection: <= hold_time normal, <= recover_time recover_speed, then
  // stop. fresh=false (no perception message within timeout) stops immediately.
  const Terms &step(double dt, bool fresh, const Config &c,
                    double speed_limit = std::numeric_limits<double>::infinity());
  const Terms &terms() const { return t_; }

private:
  Terms t_;
  Measurement last_;
  bool have_prev_ = false;
  double prev_error_ = 0, prev_time_ = 0, d_filtered_ = 0, integral_ = 0;
  double near_lateral_ = 0, near_heading_ = 0, far_lateral_ = 0, far_heading_ = 0; // slew-limited inputs
  double lost_ = 1e9, confidence_ = 0; // since / of the last good detection
};
} // namespace lane_pid
