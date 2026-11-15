#include "cone_lane_follower/core.hpp"
#include "cone_lane_follower/boundary_tracker.hpp"
#include "cone_lane_follower/path_memory.hpp"
#include "cone_lane_follower/async_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <iomanip>
#include <sstream>
namespace cone {
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Transform = geometry_msgs::msg::TransformStamped;
using Stamp = builtin_interfaces::msg::Time;
using Scan = sensor_msgs::msg::LaserScan;
inline int64_t ns(const Stamp&t)
{
  return int64_t(t.sec) * 1000000000 + t.nanosec;
}

inline geometry_msgs::msg::Point message_point(const Point&p)
{
  geometry_msgs::msg::Point q;
  q.x = p[0];
  q.y = p[1];
  q.z = p[2];
  return q;
}

inline Path transform_points(const Path&points, const Transform&tf)
{
  auto&t = tf.transform.translation;
  auto&q = tf.transform.rotation;
  double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm)||norm < 1e-12||!std::isfinite(t.x)||!std::isfinite(t.y)||!std::isfinite(
        t.z)) throw std::invalid_argument("Invalid TF translation or quaternion");
  double qx = q.x / norm, qy = q.y / norm, qz = q.z / norm, qw = q.w / norm;
  Path out;
  for (auto&v: points) {
    auto[x, y, z] = v;
    double ux = 2 * (qy * z - qz * y), uy = 2 * (qz * x - qx * z), uz = 2 * (qx * y - qy * x);
    out.push_back({x + qw * ux + qy * uz - qz * uy + t.x, y + qw * uy + qz * ux - qx * uz + t.y, z + qw * uz +
                   qx * uy - qy * ux + t.z});
  } return out;
}

inline std::pair<Path, std::vector<size_t> > scan_to_points(const Scan&msg)
{
  Path points;
  std::vector<size_t>indices;
  if (!std::isfinite(msg.angle_min)||!std::isfinite(msg.angle_increment)||!std::isfinite(msg.range_min)||
      std::isnan(msg.range_max)||msg.range_min < 0||msg.range_max < msg.range_min) return {points, indices};
  for (size_t i = 0; i < msg.ranges.size(); i++) {
    double distance = msg.ranges[i];
    if (!std::isfinite(distance)||distance < msg.range_min||distance > msg.range_max) continue;
    double angle = double(msg.angle_min) + i * double(msg.angle_increment);
    if (std::isfinite(angle)) {
      points.push_back({distance*std::cos(angle), distance * std::sin(angle), 0});
      indices.push_back(i);
    }
  } return {points, indices};
}

struct Metadata {
  int64_t stamp;
  std::optional<Transform>to_odom;
  Params parameters;
  double received;
};
struct Snapshot {
  Path odom_path;
  int64_t stamp;
  Params parameters;
  double received;
};
class RoiNode : public rclcpp::Node {
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
rclcpp::Subscription<Scan>::SharedPtr scan_sub_;
rclcpp::Publisher<Marker>::SharedPtr roi_pub_, status_pub_, points_pub_;
rclcpp::Publisher<MarkerArray>::SharedPtr clusters_pub_, centers_pub_, boundaries_pub_, control_points_pub_,
                                          center_pub_, center_spot_pub_, backup_pub_;
rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_pub_;
std::vector<rclcpp::TimerBase::SharedPtr>timers_;
std::unique_ptr<AsyncPlanner<Metadata> >async_planner_;
Path measured_center_path_, center_points_, center_path_, left_boundary_, right_boundary_, memory_local_path_;
Pairs center_pairs_;
Clusters all_clusters_, candidate_clusters_, boundary_candidate_clusters_;
std::optional<Snapshot>planned_snapshot_;
std::string center_path_mode_ = "Invalid", projection_status_ = "not_attempted", control_mode_ = "Idle",
            control_stop_reason_, memory_status_ = "empty", memory_update_reason_ = "not_attempted",
            last_logged_state_;
bool control_engaged_ = false, control_fault_ = false, memory_following_ = false, mission_cone_ = false,
     width_mission_active_ = false;
std::optional<int64_t>control_scan_stamp_, memory_scan_stamp_;
std::optional<double>control_received_, last_control_tick_, lane_width_mean_;
std::optional<std::pair<double, int64_t> >mission_exit_since_, lane_width_updated_;
double last_steering_ = 0, command_speed_ = 0, command_steering_ = 0, control_lateral_error_ = 0,
       control_heading_error_ = 0, last_warning_ = -INFINITY;
size_t lane_width_samples_ = 0;
// Cone exit: follow the lane command's steering at a capped speed (see follow_lane_on_exit).
rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr exit_lane_sub_;
std::optional<ackermann_msgs::msg::AckermannDriveStamped> exit_lane_cmd_;
std::optional<double> exit_lane_received_, exit_lane_until_;
// Consecutive scans with no candidate cone ahead of exit_front_x (front-clear exit).
size_t front_clear_scans_ = 0;
// Cone exit context for the lane node (/cone_lane/exit_lane_active): true while
// the exit wait / lane hold asks for lane steering, published every control tick.
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exit_active_pub_;
bool exit_bias_requested_ = false;
Diag path_diagnostics_;
PathMemory path_memory_;
BoundaryTracker boundary_tracker_;
double value(const std::string&name)const
{
  auto p = get_parameter(name);
  return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER?double(p.as_int()):p.as_double();
}

Params values(const Params&defaults)const
{
  Params p;
  for (auto&[name, _]: defaults) p[name] = value(name);
  return p;
}

Params non_roi_values()const
{
  Params p;
  for (auto defaults: {CLUSTER_DEFAULTS, CENTER_DEFAULTS, BOUNDARY_DEFAULTS,
                       CONTROL_DEFAULTS}) for (auto&[name, _]: defaults) p[name] = value(name);
  return p;
}

rclcpp::Time time_at(int64_t stamp)const
{
  return rclcpp::Time(stamp, get_clock()->get_clock_type());
}

public:
explicit RoiNode(const rclcpp::NodeOptions&options = rclcpp::NodeOptions()) : Node("cone_lane_roi", options)
{
  for (auto defaults: {ROI_DEFAULTS, CLUSTER_DEFAULTS, CENTER_DEFAULTS, BOUNDARY_DEFAULTS, CONTROL_DEFAULTS})
    for (auto&[name, v]: defaults) {
      if (name == "min_cluster_points"||name == "boundary_direction_window") declare_parameter<int64_t>(name,
                                                                                                        int64_t(
                                                                                                          v));
      else declare_parameter<double>(name, v);
    }
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  declare_parameter<bool>("planner_async", false, descriptor);
  declare_parameter<double>("planner_budget_sec", .06, descriptor);
  double budget = value("planner_budget_sec");
  if (!std::isfinite(budget)||budget <= 0)
    throw std::invalid_argument("planner_budget_sec must be finite and > 0");
  if (get_parameter("planner_async").as_bool())
    async_planner_ = std::make_unique<AsyncPlanner<Metadata> >();
  // Cone exit lane follow: 0 speed disables it (exit wait then just yields).
  declare_parameter<double>("exit_lane_speed", .3, descriptor);
  declare_parameter<double>("exit_lane_hold_sec", 1., descriptor);
  declare_parameter<double>("exit_lane_timeout", .25, descriptor);
  declare_parameter<std::string>("exit_lane_topic", "/mission/lane", descriptor);
  // Front-clear exit: while following the memory path with no live path, if no
  // candidate cone is ahead of exit_front_x for exit_front_scans scans and a fresh
  // lane command exists, skip the rest of the memory path and enter the exit wait
  // (lane steering at exit_lane_speed) at once. exit_front_scans 0 = off.
  declare_parameter<double>("exit_front_x", .2, descriptor);
  declare_parameter<double>("exit_front_scans", 2., descriptor);
  for (auto name: {"exit_lane_speed", "exit_lane_hold_sec", "exit_lane_timeout", "exit_front_scans"}) {
    double v = value(name);
    if (!std::isfinite(v)||v < 0) throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
  }
  if (!std::isfinite(value("exit_front_x"))) throw std::invalid_argument("exit_front_x must be finite");
  auto reason = validate_roi(values(ROI_DEFAULTS));
  if (reason.empty())
    reason = validate_parameters(non_roi_values());
  if (!reason.empty())
    throw std::invalid_argument(reason);
  parameter_callback_ = add_on_set_parameters_callback([this](const
                                                              std::vector<rclcpp::Parameter>&parameters){
        rcl_interfaces::msg::SetParametersResult result;
        auto bounds = values(ROI_DEFAULTS);
        auto p = non_roi_values();
        try{for (auto&v: parameters) {if (bounds.count(v.get_name())||p.count(v.get_name())) {double x;
                                                                                              if (v.get_type()
                                                                                                  == rclcpp::
                                                                                                  ParameterType
                                                                                                  ::
                                                                                                  PARAMETER_INTEGER)
                                                                                                x =
                                                                                                  v.as_int();
                                                                                              else if (v.
                                                                                                       get_type()
                                                                                                       ==
                                                                                                       rclcpp
                                                                                                       ::
                                                                                                       ParameterType
                                                                                                       ::
                                                                                                       PARAMETER_DOUBLE)
                                                                                                x =
                                                                                                  v.as_double();
                                                                                              else throw std::
                                                                                                invalid_argument(
                                                                                                  "Parameter must be numeric");
                                                                                              if ((v.get_name()
                                                                                                   ==
                                                                                                   "min_cluster_points"
                                                                                                   ||v.
                                                                                                   get_name()
                                                                                                   ==
                                                                                                   "boundary_direction_window")
                                                                                                  &&v.get_type()
                                                                                                  !=
                                                                                                  rclcpp::
                                                                                                  ParameterType
                                                                                                  ::
                                                                                                  PARAMETER_INTEGER)
                                                                                                throw std::
                                                                                                invalid_argument(
                                                                                                  v.get_name()
                                                                                                  +
                                                                                                  " must be an integer >= 1");
                                                                                              if (bounds.count(
                                                                                                    v.get_name()))
                                                                                                bounds[v.
                                                                                                       get_name()
                                                                                                ] = x;
                                                                                              else p[v.
                                                                                                     get_name()
                                                                                                ] = x;
                                      }} result.reason = validate_roi(bounds);
            if (result.reason.empty()) result.reason = validate_parameters(p);
        }catch (const std::exception&e) {result.reason = e.what();
        } result.successful = result.reason.empty();
        return result;
      });
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  auto outline = rclcpp::QoS(1).transient_local();
  roi_pub_ = create_publisher<Marker>("/cone_lane/roi_marker", outline);
  status_pub_ = create_publisher<Marker>("/cone_lane/control_status", outline);
  points_pub_ = create_publisher<Marker>("/cone_lane/roi_points", 1);
  clusters_pub_ = create_publisher<MarkerArray>("/cone_lane/clusters", 1);
  centers_pub_ = create_publisher<MarkerArray>("/cone_lane/cone_centers", 1);
  boundaries_pub_ = create_publisher<MarkerArray>("/cone_lane/boundaries", 1);
  control_points_pub_ = create_publisher<MarkerArray>("/cone_lane/control_points", 1);
  center_pub_ = create_publisher<MarkerArray>("/cone_lane/center_path", 1);
  center_spot_pub_ = create_publisher<MarkerArray>("/cone_lane/center_spot", 1);
  backup_pub_ = create_publisher<MarkerArray>("/cone_lane/backup_path", 1);
  command_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/mission/cone/command_05", 1);
  exit_active_pub_ = create_publisher<std_msgs::msg::Bool>("/cone_lane/exit_lane_active", 1);
  scan_sub_ = create_subscription<Scan>("/scan", rclcpp::QoS(1).best_effort().durability_volatile(), [this](
                                          Scan::ConstSharedPtr msg){
        scan_callback(*msg);
      });
  exit_lane_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    get_parameter("exit_lane_topic").as_string(), 1,
    [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg){
        exit_lane_cmd_ = *msg;
        exit_lane_received_ = steady();
      });
  // ROS-clock timers preserve rclpy create_timer behavior under /clock.
  timers_.push_back(rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(200), [this] {
        publish_roi_marker();
      }));
  timers_.push_back(rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(50), [this] {
        control_tick();
        std_msgs::msg::Bool exit;
        exit.data = exit_bias_requested_;
        exit_active_pub_->publish(exit);
      }));
  timers_.push_back(rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(200), [this] {
        publish_control_status();
      }));
  timers_.push_back(rclcpp::create_timer(this, get_clock(), std::chrono::seconds(1), [this] {
        publish_control_log();
      }));
  publish_roi_marker();
  publish_control_status();
}

private:
void scan_callback(const Scan&msg)
{
  int64_t stamp = ns(msg.header.stamp);
  double age = (now().nanoseconds() - stamp) / 1e9;
  if (stamp > 0&&age > value("control_timeout"))
    return;
  if (!control_received_||steady() - *control_received_ > value("control_timeout"))
    mission_exit_since_.reset();
  control_received_ = steady();
  control_scan_stamp_ = stamp;
  control_fault_ = false;
  auto[points, indices] = scan_to_points(msg);
  Path base;
  try{
    if (msg.header.frame_id.empty()) throw std::invalid_argument("LaserScan frame_id is empty");
    base = transform_points(points, tf_buffer_->lookupTransform("base_link", msg.header.frame_id, time_at(
                                                                  stamp)));
  }catch (const std::exception&e) {
    if (mission_cone_&&steady() - last_warning_ >= 1) {
      RCLCPP_DEBUG(get_logger(), "[라바콘] 스캔 좌표변환 실패: %s", e.what());
      last_warning_ = steady();
    } control_fault_ = true;
    mission_exit_since_.reset();
    front_clear_scans_ = 0;
    all_clusters_.clear();
    candidate_clusters_.clear();
    boundary_candidate_clusters_.clear();
    publish_roi_points({}, msg.header.stamp);
    update_boundary_markers({}, msg.header.stamp);
    publish_cluster_markers({}, msg.header.stamp);
    publish_cone_center_markers({}, msg.header.stamp);
    return;
  }
  Path roi;
  std::vector<size_t>roi_indices;
  auto b = values(ROI_DEFAULTS);
  for (size_t i = 0; i < base.size(); i++) {
    auto&q = base[i];
    if (b["roi_x_min"] < q[0]&&q[0] < b["roi_x_max"]&&b["roi_y_min"] < q[1]&&q[1] < b["roi_y_max"]) {
      roi.push_back(q);
      roi_indices.push_back(indices[i]);
    }
  }
  candidate_clusters_ = filter_clusters(cluster_points(roi, roi_indices, value("cluster_threshold")),
                                        size_t(value("min_cluster_points")), value("max_cluster_width"));
  all_clusters_ = cluster_points(base, indices, value("cluster_threshold"));
  boundary_candidate_clusters_ = filter_clusters(all_clusters_, size_t(value("min_cluster_points")), value(
                                                   "max_cluster_width"));
  if (count_front_clusters(candidate_clusters_, value("exit_front_x")) == 0) front_clear_scans_++;
  else front_clear_scans_ = 0;
  if (candidate_clusters_.size() >= 8)
    mission_cone_ = true;
  Path centers;
  for (auto&c: boundary_candidate_clusters_)
    centers.push_back(cluster_center(c));
  update_boundary_markers(centers, msg.header.stamp);
  publish_roi_points(roi, msg.header.stamp);
  publish_roi_marker();
  publish_cluster_markers(all_clusters_, msg.header.stamp);
  publish_cone_center_markers(centers, msg.header.stamp);
}

void reset_lane_width()
{
  lane_width_mean_.reset();
  lane_width_samples_ = 0;
  lane_width_updated_.reset();
  width_mission_active_ = false;
}

bool lane_width_is_fresh(const Params&p)const
{
  return lane_width_mean_&&lane_width_updated_&&p.at("center_min_width") <= *lane_width_mean_&&
         *lane_width_mean_ <= p.at("center_max_width");
}

void update_boundary_markers(const Path&centers, const Stamp&stamp)
{
  auto p = values(BOUNDARY_DEFAULTS), cp = values(CENTER_DEFAULTS);
  std::optional<double> width;
  if (lane_width_is_fresh(cp))
    width = lane_width_mean_.value();
  std::pair<Path, Path> bounds;
  if (!mission_cone_||control_fault_) {
    boundary_tracker_.clear();
  }else{
    try{
      // Both detections and saved identities use odom at the exact scan timestamp.
      auto tf = memory_transform("odom", "base_link", time_at(ns(stamp)));
      auto world = transform_points(centers, tf);
      auto from_odom = memory_transform("base_link", "odom", time_at(ns(stamp)));
      auto to_local = [&from_odom](const Point&q) {
        return transform_points(Path{q}, from_odom).front();
      };
      bounds = boundary_tracker_.update(centers, world, to_local, ns(stamp) / 1e9, p, width,
                                        cp["center_width_relative_tolerance"]);
    }catch (const std::exception&e) {
      // No valid motion transform: never compare coordinates from different frames.
      boundary_tracker_.clear();
      if (steady() - last_warning_ >= 1) {
        RCLCPP_WARN(get_logger(), "[라바콘] 경계 추적 실패: 좌표변환 불가/지연 (%s)", e.what());
        last_warning_ = steady();
      }
    }
  }
  left_boundary_ = std::move(bounds.first);
  right_boundary_ = std::move(bounds.second);
  auto markers = marker_array(stamp);
  for (int i = 0; i < 2; i++) {
    auto&boundary = i == 0?left_boundary_:right_boundary_;
    if (boundary.empty()) continue;
    auto marker = new_marker(i == 0?"left_boundary":"right_boundary", boundary.size() >= 2?Marker::LINE_STRIP:
                             Marker::POINTS, stamp);
    marker.scale.x = boundary.size() >= 2?.03:.10;
    marker.scale.y = .10;
    if (i == 0) marker.color.r = 1;
    else marker.color.b = 1;
    set_points(marker, boundary);
    marker.lifetime = rclcpp::Duration::from_seconds(.5);
    markers.markers.push_back(marker);
  } boundaries_pub_->publish(markers);
  update_center_path(stamp);
  update_control_activation();
}

void update_center_path(const Stamp&stamp)
{
  path_diagnostics_.clear();
  projection_status_ = "not_attempted";
  if (!mission_cone_||!width_mission_active_)
    reset_lane_width();
  width_mission_active_ = mission_cone_;
  auto p = values(CENTER_DEFAULTS);
  // Only the one-point-side case uses the requested fan-midpoint path.
  // Do not submit it to the shared worker or reuse an older normal-path snapshot.
  if (use_single_point_midpath(left_boundary_, right_boundary_)) {
    if (async_planner_) {
      async_planner_->reset();
      async_planner_->poll(); // Drain any completed job from the previous mode.
    }
    planned_snapshot_.reset();
    center_pairs_ = single_point_sections(left_boundary_, right_boundary_,
                                          p["center_min_width"], p["center_max_width"]);
    center_points_.clear();
    for (auto&[a,b]: center_pairs_) center_points_.push_back(lerp(a,b,.5));
    center_path_ = single_point_midpath(left_boundary_, right_boundary_, p, &path_diagnostics_);
    measured_center_path_ = center_path_;
    center_path_mode_ = center_path_.empty()?"Invalid":"Single-point Midpoints";
    publish_center_path_markers(stamp);
    return;
  }
  // All other cases keep the original cross-section/offset/planner flow.
  bool both = left_boundary_.size() >= 2&&right_boundary_.size() >= 2;
  center_pairs_ = both?match_cross_sections(left_boundary_, right_boundary_, p["center_sample_step"],
                                            p["center_min_width"], p["center_max_width"]):Pairs{};
  center_points_.clear();
  for (auto&[a, b]: center_pairs_)
    center_points_.push_back(lerp(a, b, .5));
  center_path_ = smooth_center_path(center_points_, left_boundary_, right_boundary_,
                                    p["center_smoothing_weight"], p["center_max_gap"], p["center_max_turn"],
                                    &path_diagnostics_);
  center_path_mode_ = center_path_.empty()?"Invalid":"Normal Sections";
  if (mission_cone_&&!center_path_.empty()&&path_control_geometry(center_path_, value("control_reference_x"),
                                                                  value("control_max_path_distance"))) {
    std::vector<double>widths;
    double sum = 0;
    for (auto&[a, b]: center_pairs_) {
      widths.push_back(xy(a, b));
      sum += widths.back();
    } double mean = sum / widths.size();
    auto mm = std::minmax_element(widths.begin(), widths.end());
    if (*mm.second - *mm.first <= mean * p["center_width_relative_tolerance"]) {
      if (!lane_width_is_fresh(p)) {
        lane_width_mean_.reset();
        lane_width_samples_ = 0;
      } double total = lane_width_mean_.value_or(0) * lane_width_samples_ + sum;
      lane_width_samples_ += widths.size();
      lane_width_mean_ = total / lane_width_samples_;
      lane_width_updated_ = {{steady(), now().nanoseconds()}};
    }
  }
  measured_center_path_ = center_path_;
  if (!async_planner_) {
    center_path_ = unified_path(left_boundary_, right_boundary_, center_pairs_, center_path_,
                                lane_width_mean_, p, &path_diagnostics_);
    center_path_mode_ = center_path_.empty()?"Invalid":"Unified";
  }else if (!mission_cone_||!lane_width_mean_) {
    async_planner_->reset();
    planned_snapshot_.reset();
  }else{
    std::optional<Transform>tf;
    try{
      tf = memory_transform("odom", "base_link", time_at(ns(stamp)));
    }catch (const std::exception&) {} typename AsyncPlanner<Metadata>::Payload payload{left_boundary_,
                                                                                       right_boundary_,
                                                                                       center_pairs_,
                                                                                       center_path_,
                                                                                       lane_width_mean_, p,
                                                                                       value(
                                                                                         "planner_budget_sec")};
    async_planner_->request(std::move(payload), {ns(stamp), tf, p, steady()});
    use_async_path();
  }
  if (center_path_.empty()&&lane_width_mean_&&(left_boundary_.size() >= 2) != (right_boundary_.size() >= 2)&&
      !left_boundary_.empty()&&!right_boundary_.empty()&&path_diagnostics_.count("width_conflict"))
    center_path_mode_ = "Boundary Width Conflict";
  publish_center_path_markers(stamp);
}

void clear_path_memory()
{
  if (async_planner_) {
    async_planner_->reset();
    planned_snapshot_.reset();
  } path_memory_.clear();
  memory_scan_stamp_.reset();
  memory_local_path_.clear();
  memory_status_ = "empty";
  memory_update_reason_ = "not_attempted";
  memory_following_ = false;
}

Transform memory_transform(const std::string&target, const std::string&source, const rclcpp::Time&stamp)
{
  auto tf = tf_buffer_->lookupTransform(target, source, stamp);
  double age = (now().nanoseconds() - ns(tf.header.stamp)) / 1e9;
  if (tf.header.frame_id != target||tf.child_frame_id != source||!(age >= 0&&age <= value(
                                                                     "control_memory_tf_timeout"))) throw std
          ::invalid_argument("Missing or stale odom transform");
  return tf;
}

std::pair<Path, std::string> remaining_memory_path(const Params&p)
{
  if (path_memory_.points.empty()) return {{}, "memory_path_unavailable"};
  if (path_memory_.finished) return {{}, "memory_goal_reached"};
  try{
    auto tf = memory_transform("base_link", "odom", time_at(0));
    auto local = transform_points(path_memory_.points, tf);
    return path_memory_.remaining(local, p.at("control_memory_goal_tolerance"), p.at(
                                    "control_max_path_distance"));
  }catch (const std::exception&) {
    return {{}, "memory_tf_unavailable_or_stale"};
  }
}

void remember_live_path(bool replace = false)
{
  if (memory_scan_stamp_ == control_scan_stamp_&&!replace) return;
  Path points, local;
  try{
    auto tf = memory_transform("odom", "base_link", time_at(control_scan_stamp_.value_or(0)));
    points = transform_points(center_path_, tf);
    if (replace) {
      tf = memory_transform("base_link", "odom", time_at(0));
      local = transform_points(points, tf);
    }
  }catch (const std::exception&) {
    memory_status_ = memory_update_reason_ = "save_tf_unavailable_or_stale";
    return;
  } if (replace) path_memory_.replace_from_vehicle(points, local, value("control_memory_recovery_distance"));
  else path_memory_.refresh(points, [this](const Path&path){
          return valid_center_geometry(path, values(CENTER_DEFAULTS));
        });
  memory_scan_stamp_ = control_scan_stamp_;
  memory_status_ = memory_update_reason_ = path_memory_.update_reason;
}

void stop_control(const std::string&reason = "unspecified")
{
  if (reason == "tf_error"||reason == "mission_inactive") clear_path_memory();
  publish_control_points({});
  publish_drive(0, 0);
  last_steering_ = 0;
  control_mode_ = "Stopped";
  control_stop_reason_ = reason;
}

// Exit wait / mission end: publish nothing so the mux drops this input after its
// timeout and the lower-priority lane command takes over without a stop. A zero
// command here would be an active stop that outranks lane for the whole wait.
void yield_control(const std::string&reason)
{
  publish_control_points({});
  last_steering_ = 0;
  control_mode_ = "Yielding";
  control_stop_reason_ = reason;
}

// Cone exit (exit wait, then exit_lane_hold_sec after the mission ends): keep
// cone priority and pass the fresh lane command through unchanged (lane speed and
// steering, no extra speed cap). exit_lane_speed > 0 enables it, 0 = off (yield).
// Without a fresh lane command this yields, so the mux falls through as before.
bool exit_lane_fresh()
{
  double timeout = value("exit_lane_timeout");
  if (value("exit_lane_speed") <= 0||!exit_lane_cmd_||!exit_lane_received_) return false;
  auto&d = exit_lane_cmd_->drive;
  double wall_age = steady() - *exit_lane_received_;
  double ros_age = (now().nanoseconds() - ns(exit_lane_cmd_->header.stamp)) / 1e9;
  return wall_age >= 0&&wall_age <= timeout&&ros_age >= -.05&&ros_age <= timeout&&std::isfinite(d.speed)&&
         std::isfinite(d.steering_angle)&&d.speed >= 0;
}

bool follow_lane_on_exit(const std::string&reason)
{
  exit_bias_requested_ = value("exit_lane_speed") > 0;
  if (exit_lane_fresh()) {
    auto&d = exit_lane_cmd_->drive;
    publish_control_points({});
    last_steering_ = 0;
    publish_drive(d.speed, d.steering_angle);
    control_mode_ = "Exit Lane Follow";
    control_stop_reason_ = reason;
    return true;
  }
  yield_control(reason);
  return false;
}

void update_control_activation()
{
  if (!mission_cone_||control_fault_||candidate_clusters_.size() > 4||path_control_geometry(center_path_,
                                                                                            value(
                                                                                              "control_reference_x"),
                                                                                            value(
                                                                                              "control_max_path_distance")))
    mission_exit_since_.reset();
  if (mission_cone_) control_engaged_ = true;
  else if (control_engaged_&&!control_fault_) {
    stop_control("mission_inactive");
    control_engaged_ = false;
  } if (control_engaged_&&control_fault_) stop_control("tf_error");
  else if (control_engaged_&&center_path_.empty()) control_tick();
}

void control_tick()
{
  exit_bias_requested_ = false;
  double wall = steady();
  int64_t current = now().nanoseconds();
  double dt = last_control_tick_?clamp(wall - *last_control_tick_, 0, .1):.05;
  last_control_tick_ = wall;
  if (!control_engaged_) {
    publish_control_points({});
    clear_path_memory();
    mission_exit_since_.reset();
    last_steering_ = 0;
    if (exit_lane_until_&&!mission_cone_&&wall < *exit_lane_until_) {
      follow_lane_on_exit("exit_lane_hold");
      return;
    }
    exit_lane_until_.reset();
    control_mode_ = "Idle";
    return;
  } auto p = values(CONTROL_DEFAULTS);
  double age = control_scan_stamp_?(current - *control_scan_stamp_) / 1e9:INFINITY;
  bool fresh = control_received_&&control_scan_stamp_&&*control_scan_stamp_ > 0&&wall - *control_received_ >=
               0&&wall - *control_received_ <= p["control_timeout"]&&age >= 0&&age <= p["control_timeout"];
  if (!fresh||control_fault_||!mission_cone_) {
    mission_exit_since_.reset();
    std::string reason;
    if (control_fault_) reason = "tf_error";
    else if (!mission_cone_) reason = "mission_inactive";
    else if (!control_received_||!control_scan_stamp_) reason = "scan_not_received";
    else if (*control_scan_stamp_ <= 0) reason = "scan_stamp_zero_or_negative";
    else if (wall < *control_received_) reason = "scan_receive_clock_reversed";
    else if (wall - *control_received_ > p["control_timeout"]) reason = "scan_receive_timeout";
    else if (current < *control_scan_stamp_) reason = "scan_stamp_in_future";
    else reason = "scan_stamp_timeout";
    if (reason == "scan_receive_timeout"||reason == "scan_stamp_timeout") {
      memory_following_ = false;
      memory_local_path_.clear();
      memory_status_ = path_memory_.points.empty()?"empty":"paused_scan_timeout";
      publish_center_path_markers(now());
    }else clear_path_memory();
    stop_control(reason);
    return;
  }
  if (async_planner_)
    use_async_path();
  auto errors = path_control_geometry(center_path_, p["control_reference_x"], p["control_max_path_distance"]);
  auto live_errors = errors;
  std::string memory_reason;
  memory_local_path_.clear();
  if (!path_memory_.points.empty()) {
    auto[previous, reason] = remaining_memory_path(p);
    memory_reason = reason;
    if (memory_reason.empty()&&!path_control_geometry(previous, p["control_reference_x"],
                                                      p["control_max_path_distance"])) memory_reason =
        "memory_" + path_stop_reason(previous,
                                     p
                                     ["control_reference_x"]);
  }
  // Preserve the original branch ordering, including its unreachable recovery elif.
  if (live_errors) {
    remember_live_path();
    if (path_memory_.finished) remember_live_path(true);
  }else if (live_errors&&memory_reason != "memory_tf_unavailable_or_stale")
    remember_live_path(true);
  else if (!live_errors)
    memory_update_reason_ = "not_attempted: " + path_stop_reason(center_path_, p["control_reference_x"]);
  else
    memory_update_reason_ = "not_attempted: " + memory_reason;
  memory_following_ = false;
  if (!path_memory_.points.empty()) {
    auto[remaining, reason] = remaining_memory_path(p);
    memory_reason = reason;
    errors = path_control_geometry(remaining, p["control_reference_x"], p["control_max_path_distance"]);
    if (errors) {
      memory_local_path_ = remaining;
      memory_following_ = true;
      if (!live_errors) memory_status_ = "following";
    }else{
      if (memory_reason.empty()) memory_reason = "memory_" + path_stop_reason(remaining,
                                                                              p["control_reference_x"]);
      memory_status_ = memory_reason;
    }
  } if (live_errors) {
    errors = live_errors;
    memory_following_ = false;
  }
  // Front-clear exit: the remaining memory path is the last (often straight) cone
  // corridor; with no cone ahead any more, hand the steering to the lane now.
  bool front_clear = memory_following_&&!live_errors&&value("exit_front_scans") > 0&&
                     double(front_clear_scans_) >= value("exit_front_scans")&&exit_lane_fresh();
  if (front_clear) {
    memory_following_ = false;
    memory_local_path_.clear();
    memory_status_ = "skipped_front_clear";
    errors.reset();
  }
  publish_center_path_markers(now());
  if (!errors&&candidate_clusters_.size() <= 4) {
    if (!mission_exit_since_) mission_exit_since_ = {{wall, current}};
    auto[saved_wall, saved_ros] = *mission_exit_since_;
    if (wall < saved_wall||current < saved_ros) mission_exit_since_ = {{wall, current}};
    else if (wall - saved_wall >= p["control_exit_timeout"]&&(current - saved_ros) / 1e9 >=
             p["control_exit_timeout"]) {
      mission_cone_ = false;
      boundary_tracker_.clear();
      clear_path_memory();
      reset_lane_width();
      mission_exit_since_.reset();
      if (value("exit_lane_speed") > 0) exit_lane_until_ = wall + value("exit_lane_hold_sec");
      follow_lane_on_exit("mission_exit_detection_loss");
      control_engaged_ = false;
      return;
    }
    // Waiting for exit with no path and few cones: never a zero stop. Follow the
    // lane steering at a capped speed (or yield without a fresh lane command).
    // Live cones (a path) re-take control at once.
    follow_lane_on_exit(front_clear?"mission_exit_front_clear":"mission_exit_wait");
    return;
  }else
    mission_exit_since_.reset();
  if (!errors) {
    std::string reason;
    if (center_path_mode_ == "Width Unavailable or Stale") reason = "lane_width_unavailable_or_stale";
    else if (center_path_mode_ == "Boundary Width Conflict") reason = "boundary_width_conflict";
    stop_control(!memory_reason.empty()?memory_reason:!reason.empty()?reason:path_stop_reason(center_path_,
                                                                                              p[
                                                                                                "control_reference_x"
                                                                                              ]));
    return;
  }
  control_lateral_error_ = errors->lateral;
  control_heading_error_ = errors->heading;
  double target = p["control_lateral_gain"] * errors->lateral + p["control_heading_gain"] * errors->heading;
  double limit = p["control_max_steering"];
  target = clamp(target, -limit, limit);
  double step = p["control_max_steering_rate"] * dt;
  last_steering_ += clamp(target - last_steering_, -step, step);
  last_steering_ = clamp(last_steering_, -limit, limit);
  double slowdown = std::min(1., std::abs(target) / p["control_speed_reference_steering"]), speed =
    p["control_speed"] * (1 - p["steering_gain"] * slowdown);
  auto&active = memory_following_?memory_local_path_:center_path_;
  publish_control_points(active);
  double peak = peak_curvature(active);
  if (peak > 1e-9) {
    double ratio = std::min(1., peak * value("center_min_turn_radius"));
    speed = std::min(speed, p["control_speed"] * (1 - p["curvature_gain"] * ratio));
  } if (memory_following_)
    speed = std::min(speed, p["control_memory_speed"]);
  control_mode_ = memory_following_?"Memory Path Feedback":"Path Feedback";
  control_stop_reason_.clear();
  publish_drive(speed, last_steering_);
}

void use_async_path()
{
  center_path_ = measured_center_path_;
  if (use_single_point_midpath(left_boundary_, right_boundary_)) {
    // The control timer also polls here: a late normal-mode result must never
    // overwrite the current singleton path, including when that path is invalid.
    async_planner_->poll();
    planned_snapshot_.reset();
    center_path_mode_ = center_path_.empty()?"Invalid":"Single-point Midpoints";
    return;
  }
  center_path_mode_ = center_path_.empty()?"Invalid":"Measured (planner pending)";
  auto result = async_planner_->poll();
  if (result) {
    for (auto&[k, v]: result->diagnostics) path_diagnostics_[k] = v;
    auto&m = result->metadata;
    if (!result->path.empty()&&m.to_odom) planned_snapshot_ = Snapshot{transform_points(result->path,
                                                                                        *m.to_odom), m.stamp,
                                                                       m.parameters, m.received};
    else planned_snapshot_.reset();
  } if (!planned_snapshot_) return;
  auto&s = *planned_snapshot_;
  double timeout = value("control_timeout"), age = (now().nanoseconds() - s.stamp) / 1e9, wall_age =
    steady() - s.received;
  if (!(age >= 0&&age <= timeout&&wall_age >= 0&&wall_age <= timeout)||s.parameters != values(
        CENTER_DEFAULTS)) {
    planned_snapshot_.reset();
    return;
  } try{
    auto tf = memory_transform("base_link", "odom", time_at(control_scan_stamp_.value_or(0)));
    center_path_ = transform_points(s.odom_path, tf);
    center_path_mode_ = "Unified (worker)";
  }catch (const std::exception&) {
    path_diagnostics_["planner_tf"] = "unavailable; retained measured path";
  }
}

void publish_drive(double speed, double steering)
{
  command_speed_ = speed;
  command_steering_ = steering;
  ackermann_msgs::msg::AckermannDriveStamped msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp = now();
  msg.drive.speed = speed;
  msg.drive.steering_angle = steering;
  command_pub_->publish(msg);
}

Marker new_marker(const std::string&name, int kind, std::optional<Stamp>stamp = {})
{
  Marker m;
  m.header.frame_id = "base_link";
  m.header.stamp = stamp.value_or(Stamp(now()));
  m.ns = name;
  m.id = 0;
  m.type = kind;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1;
  m.color.a = 1;
  m.frame_locked = true;
  return m;
}

MarkerArray marker_array(std::optional<Stamp>stamp = {})
{
  MarkerArray a;
  auto m = new_marker("clear", Marker::POINTS, stamp);
  m.action = Marker::DELETEALL;
  a.markers.push_back(m);
  return a;
}

void set_points(Marker&m, const Path&path)
{
  for (auto&q: path) m.points.push_back(message_point(q));
}

void color(Marker&m, double r, double g, double b)
{
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
}

void publish_center_path_markers(const Stamp&stamp)
{
  auto markers = marker_array(stamp), backup = marker_array(stamp), spots = marker_array(stamp);
  Path sections;
  for (auto&[a, b]: center_pairs_) {
    sections.push_back(a);
    sections.push_back(b);
  } struct Spec {
    std::string name;
    Path points;
    int kind;
    Point color;
    double size;
  };
  for (auto&s: std::vector<Spec>{{"cross_sections", sections, Marker::LINE_LIST, {1, 1, 0}, .01},
         {"center_points", center_points_, Marker::POINTS, {1, 0, 1}, .05}, {"center_path", center_path_,
                                                                             Marker::LINE_STRIP, {0, 1, 0},
                                                                             .03}, {"remembered_path",
                                                                                    memory_local_path_,
                                                                                    Marker::LINE_STRIP, {1,
                                                                                                         .6,
                                                                                                         0},
                                                                                    .05}}) {
    if (s.points.empty()) continue;
    auto m = new_marker(s.name, s.kind, stamp);
    set_points(m, s.points);
    m.scale.x = m.scale.y = s.size;
    color(m, s.color[0], s.color[1], s.color[2]);
    m.lifetime = rclcpp::Duration::from_seconds(.5);
    if (s.name == "remembered_path") backup.markers.push_back(m);
    else if (s.name == "cross_sections"||s.name == "center_points") spots.markers.push_back(m);
    else markers.markers.push_back(m);
  } center_pub_->publish(markers);
  backup_pub_->publish(backup);
  center_spot_pub_->publish(spots);
}

void publish_control_points(const Path&path)
{
  auto markers = marker_array();
  auto g = path_control_geometry(path, value("control_reference_x"), value("control_max_path_distance"));
  if (g) {
    double ref = value("control_reference_x");
    for (int i = 0; i < 2; i++) {
      auto m = new_marker(i == 0?"reference":"projection", Marker::SPHERE);
      m.pose.position = message_point(i == 0?Point{ref, 0, .06}:Point{g->x, g->y, .06});
      m.scale.x = m.scale.y = m.scale.z = .08;
      color(m, i == 0?1:0, 1, 1);
      m.lifetime = rclcpp::Duration::from_seconds(.2);
      markers.markers.push_back(m);
    } struct Spec {
      std::string name;
      Path points;
      Point color;
    };
    for (auto&s: std::vector<Spec>{{"selected_segment", {g->a, g->b}, {1, .5, 0}}, {"projection_error", {{ref,
                                                                                      0, 0}, {g->x, g->y, 0}},
                                                                                    {1, 1, 1}},
           {"path_heading", {{g->x, g->y, 0}, {g->x + .2 * std::cos(
                                                 g
                                                 ->
                                                 heading), g->y + .2 * std::sin(g->heading), 0}}, {0, 1,
                                                                                                   1}}}) {
      auto m = new_marker(s.name, Marker::LINE_STRIP);
      for (auto q: s.points) {
        q[2] = .06;
        m.points.push_back(message_point(q));
      } m.scale.x = .025;
      color(m, s.color[0], s.color[1], s.color[2]);
      m.lifetime = rclcpp::Duration::from_seconds(.2);
      markers.markers.push_back(m);
    }
  } control_points_pub_->publish(markers);
}

void publish_cluster_markers(const Clusters&clusters, std::optional<Stamp>stamp = {})
{
  auto markers = marker_array(stamp);
  for (size_t i = 0; i < clusters.size(); i++) {
    auto m = new_marker("scan_clusters", Marker::POINTS, stamp);
    m.id = i;
    m.scale.x = m.scale.y = value("cluster_point_size");
    double h = std::fmod(i * .618034, 1.) * 6, f = h - std::floor(h), v = 1, s = .8, p = v * (1 - s), q = v *
                                                                                                          (1 -
                                                                                                           s *
                                                                                                           f),
           t = v * (1 - s * (1 - f));
    switch (int(h) % 6) {
    case 0: color(m, v, t, p);
      break;
    case 1: color(m, q, v, p);
      break;
    case 2: color(m, p, v, t);
      break;
    case 3: color(m, p, q, v);
      break;
    case 4: color(m, t, p, v);
      break;
    default: color(m, v, p, q);
    } set_points(m, clusters[i]);
    m.lifetime = rclcpp::Duration::from_seconds(.5);
    markers.markers.push_back(m);
  } clusters_pub_->publish(markers);
}

void publish_cone_center_markers(const Path&centers, std::optional<Stamp>stamp = {})
{
  auto markers = marker_array(stamp);
  for (size_t i = 0; i < centers.size(); i++) {
    auto m = new_marker("cone_candidate_centers", Marker::SPHERE, stamp);
    m.id = 2 * i;
    m.scale.x = m.scale.y = m.scale.z = value("cone_center_size");
    m.color.r = 1;
    m.pose.position = message_point(centers[i]);
    m.lifetime = rclcpp::Duration::from_seconds(.5);
    markers.markers.push_back(m);
    auto label = new_marker("cone_candidate_labels", Marker::TEXT_VIEW_FACING, stamp);
    label.id = 2 * i + 1;
    label.text = "Cone Candidate " + std::to_string(i);
    auto q = centers[i];
    q[2] += .12;
    label.pose.position = message_point(q);
    label.scale.z = .10;
    color(label, 1, 1, 1);
    label.lifetime = rclcpp::Duration::from_seconds(.5);
    markers.markers.push_back(label);
  } centers_pub_->publish(markers);
}

std::string log_reason_ko(const std::string&reason)const
{
  static const std::map<std::string, std::string> labels = {
    {"tf_error", "라이다 좌표변환 실패"},
    {"mission_inactive", "미션 비활성"},
    {"scan_not_received", "라이다 미수신"},
    {"scan_stamp_zero_or_negative", "라이다 시각 오류"},
    {"scan_receive_clock_reversed", "라이다 수신 시계 역행"},
    {"scan_receive_timeout", "라이다 수신 지연"},
    {"scan_stamp_in_future", "라이다 시각이 현재보다 미래"},
    {"scan_stamp_timeout", "라이다 데이터 오래됨"},
    {"empty_path", "경로 없음"},
    {"path_too_few_points", "경로 점 부족"},
    {"path_nonfinite_coordinates", "경로 좌표 이상"},
    {"path_no_usable_forward_segment", "사용 가능한 전방 경로 없음"},
    {"path_too_far_from_reference", "경로가 차량 기준점에서 너무 멂"},
    {"memory_path_unavailable", "메모리 경로 없음"},
    {"memory_goal_reached", "메모리 경로 끝 도달"},
    {"memory_path_unusable", "메모리 경로 사용 불가"},
    {"memory_path_too_far", "메모리 경로가 차량에서 너무 멂"},
    {"memory_tf_unavailable_or_stale", "메모리 좌표변환 불가/지연"},
    {"lane_width_unavailable_or_stale", "차선 폭 정보 없음/지연"},
    {"boundary_width_conflict", "좌우 경계 폭 불일치"},
    {"mission_exit_wait", "탈출 대기"},
    {"mission_exit_front_clear", "전방 라바콘 소실로 차선 전환"},
    {"mission_exit_detection_loss", "라바콘 감지 종료"},
    {"exit_lane_hold", "탈출 차선 추종"},
    {"insufficient_center_points", "중심점 부족"},
    {"offset_unavailable", "경계점 또는 차선 폭 정보 부족"},
    {"width_conflict", "추정 폭과 경계 불일치"},
    {"outside_boundary", "경계 밖 경로"},
    {"boundary_crossing", "경계 침범"},
    {"gap", "경로 점 간격 초과"},
    {"turn", "경로 꺾임 각도 초과"},
    {"self_crossing", "경로 자체 교차"},
    {"corner_failure", "코너 경로 계산 실패"},
    {"single_point_unavailable", "단일점 중점 구간 부족"},
    {"single_point_turn_radius", "단일점 경로 회전반경 부족"},
    {"single_point_invalid", "단일점 경로 형상 부적합"},
    {"planning_budget", "경로 연산 시간 초과"},
    {"planner_error", "경로 계산 예외"},
    {"planner_tf", "계산 경로 좌표변환 실패"},
    {"join", "계산 경로 연결 실패"}
  };
  if (reason.empty()||reason == "none"||reason == "None") return "없음";
  auto it = labels.find(reason);
  if (it != labels.end()) return it->second;
  if (reason.rfind("memory_", 0) == 0) return "메모리 " + log_reason_ko(reason.substr(7));
  return reason; // Preserve new/unclassified reason codes instead of hiding them.
}

std::string calculation_failure_summary()const
{
  // Diagnostics may contain rejected alternatives even when the selected live
  // path is valid. Only label the live path a failure if it cannot be used.
  double ref = value("control_reference_x");
  if (path_control_geometry(center_path_, ref, value("control_max_path_distance"))) return "없음";
  std::vector<std::string> reasons;
  for (const auto&key: {"planner_error", "planner_tf", "planning_budget", "outside_boundary",
                        "boundary_crossing", "self_crossing", "width_conflict", "gap", "turn",
                        "single_point_turn_radius", "single_point_invalid", "single_point_unavailable",
                        "corner_failure", "offset_unavailable", "insufficient_center_points", "join"}) {
    auto it = path_diagnostics_.find(key);
    if (it == path_diagnostics_.end()) continue;
    if (it->first == "join"&&it->second.rfind("failed", 0) != 0) continue;
    reasons.push_back(log_reason_ko(key));
  }
  if (reasons.empty()) return log_reason_ko(path_stop_reason(center_path_, ref));
  std::string out;
  for (size_t i = 0; i < reasons.size()&&i < 3; ++i) {
    if (!out.empty()) out += ", ";
    out += reasons[i];
  }
  if (reasons.size() > 3) out += " 외 " + std::to_string(reasons.size() - 3) + "건";
  return out;
}

std::string control_log_summary()const
{
  // Use the last command's control mode, not the mere presence of a cached path.
  bool live = control_mode_ == "Path Feedback";
  bool memory = control_mode_ == "Memory Path Feedback";
  bool lane = control_mode_ == "Exit Lane Follow";
  bool command = live||memory||lane||control_mode_ == "Stopped";
  bool stopped = control_mode_ == "Stopped"||(command&&std::abs(command_speed_) < 1e-6);
  std::string state = stopped?"정지":live||memory?"주행":lane?"차선 전환":
                      control_mode_ == "Yielding"?"제어 양보":"대기";
  std::string source = live?"계산":memory?"메모리":lane?"차선":"없음";
  std::string stop_reason = "없음";
  if (stopped) {
    stop_reason = lane?"차선 명령 속도 0":control_stop_reason_.empty()?
                  "경로 제어 출력 속도 0":log_reason_ko(control_stop_reason_);
  }
  std::ostringstream out;
  out << "[라바콘] 상태=" << state << " | 경로=" << source << " | 속도=";
  if (command) out << std::fixed << std::setprecision(2) << command_speed_ << "m/s";
  else out << "미발행";
  out << " | 조향=";
  if (command) out << std::showpos << std::setprecision(1) << command_steering_ * 180 / pi << "도" << std::noshowpos;
  else out << "미발행";
  out << " | 정지이유=" << stop_reason << " | 계산실패=" << calculation_failure_summary();
  if (control_mode_ == "Yielding"||lane) out << " | 전환이유=" << log_reason_ko(control_stop_reason_);
  return out.str();
}

void publish_control_log()
{
  if (!mission_cone_) return;
  auto summary = control_log_summary();
  if (control_mode_ == "Stopped")
    RCLCPP_WARN(get_logger(), "%s", summary.c_str());
  else
    RCLCPP_INFO(get_logger(), "%s", summary.c_str());
}

void publish_control_status()
{
  if (!mission_cone_) {
    last_logged_state_.clear();
  }else{
    std::string state = control_mode_ + "|" + control_stop_reason_ + "|" + calculation_failure_summary();
    if (state != last_logged_state_) {
      last_logged_state_ = state;
      publish_control_log();
    }
  } auto m = new_marker("control_status", Marker::TEXT_VIEW_FACING);
  auto b = values(ROI_DEFAULTS);
  m.pose.position = message_point({(b["roi_x_min"] + b["roi_x_max"]) / 2, b["roi_y_max"] + .4, .3});
  m.scale.z = .12;
  m.text = "Control: " + control_mode_ + " | mission: " + (mission_cone_?"True":"False") + "\nstop_reason: " +
           (control_stop_reason_.empty()?"None":control_stop_reason_);
  if (!control_stop_reason_.empty()) color(m, 1, .25, .25);
  else if (control_mode_ == "Path Feedback"||control_mode_ == "Memory Path Feedback") color(m, .25, 1, .25);
  else color(m, .8, .8, .8);
  m.lifetime = rclcpp::Duration::from_seconds(.6);
  status_pub_->publish(m);
}

void publish_roi_marker()
{
  auto b = values(ROI_DEFAULTS);
  auto m = new_marker("roi_boundary", Marker::LINE_STRIP);
  m.scale.x = .03;
  color(m, 1, 1, 0);
  double x0 = b["roi_x_min"], x1 = b["roi_x_max"], y0 = b["roi_y_min"], y1 = b["roi_y_max"];
  set_points(m, {{x0, y0, 0}, {x1, y0, 0}, {x1, y1, 0}, {x0, y1, 0}, {x0, y0, 0}});
  roi_pub_->publish(m);
}

void publish_roi_points(const Path&points, std::optional<Stamp>stamp = {})
{
  auto m = new_marker("roi_points", Marker::POINTS, stamp);
  m.scale.x = m.scale.y = .04;
  m.color.g = 1;
  set_points(m, points);
  if (points.empty()) m.action = Marker::DELETE;
  m.lifetime = rclcpp::Duration::from_seconds(.5);
  points_pub_->publish(m);
}

};
} // namespace cone
#ifndef CONE_NO_MAIN
int main(int argc, char**argv)
{
  rclcpp::init(argc, argv);
  try{
    rclcpp::spin(std::make_shared<cone::RoiNode>());
  }catch (const std::exception&e) {
    if (rclcpp::ok()) {
      RCLCPP_FATAL(rclcpp::get_logger("cone_lane_roi"), "%s", e.what());
      rclcpp::shutdown();
      return 1;
    }
  } if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}

#endif
