// /lane_fusion/target -> error-based PID -> /lane_fusion_pid/command (+ /mission/lane).
// Gains are ROS parameters; the separate lane_fusion_pid_tuner window edits them live.
#include "autorace_lane_fusion_pid/pid.hpp"
#include "autorace_lane_fusion_pid/parking_approach.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

namespace {
double steady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace

// /lane_fusion_pid/state layout (published every output tick):
// [tracking, error, p, i, d, ff, target_steering, steering, speed,
//  near_lateral_m, near_heading_rad, curvature, confidence, data_age_s, active,
//  lookahead_lateral_m, lost_s]  (lost_s: time since the last good detection)
class LanePidController : public rclcpp::Node {
public:
  LanePidController() : Node("lane_fusion_pid_controller") {
    for (const auto &s : lane_pid::specs()) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = std::string(s.help) + " [" + s.unit + "]";
      rcl_interfaces::msg::FloatingPointRange range;
      range.from_value = s.min;
      range.to_value = s.max;
      d.floating_point_range.push_back(range);
      config_.values[s.name] = declare_parameter<double>(s.name, s.value, d);
    }
    config_.validate();
    const bool vehicle = declare_parameter("enable_vehicle_output", false);
    const auto input = declare_parameter<std::string>("target_topic", "/lane_fusion/target");
    const auto parking_topic = declare_parameter<std::string>("parking_sign_topic", "/yolo/parking_sign");
    const int confirmations = declare_parameter<int>("parking_slowdown_confirmations", 2);
    const double parking_speed = declare_parameter<double>("parking_slowdown_speed", .3);
    const double parking_duration = declare_parameter<double>("parking_slowdown_duration", 10.);
    parking_ = lane_pid::ParkingApproach(confirmations, parking_speed, parking_duration,
                                       declare_parameter<double>("parking_slowdown_max_gap", .5),
                                       declare_parameter<double>("parking_slowdown_cooldown", 5.),
                                       declare_parameter<bool>("parking_calibration_stop", false));

    parameters_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &ps) {
      rcl_interfaces::msg::SetParametersResult result;
      auto next = config_;
      try {
        for (const auto &p : ps) {
          if (next.values.count(p.get_name()))
            next.values[p.get_name()] = p.as_double();
          else if (p.get_name() != "use_sim_time")
            throw std::invalid_argument("Startup-only parameter: " + p.get_name());
        }
        next.validate();
        config_ = next;
        result.successful = true;
      } catch (const std::exception &e) {
        result.reason = e.what();
      }
      return result;
    });

    command_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/lane_fusion_pid/command",
                                                                            rclcpp::QoS(1));
    if (vehicle)
      vehicle_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/mission/lane",
                                                                              rclcpp::QoS(1));
    state_ = create_publisher<std_msgs::msg::Float32MultiArray>("/lane_fusion_pid/state", rclcpp::QoS(1));
    approach_start_ = create_publisher<std_msgs::msg::Header>("/parking/approach_start", rclcpp::QoS(10));
    target_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        input, rclcpp::QoS(1), [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
          if (msg->data.size() < 6)
            return;
          lane_pid::Measurement m;
          m.valid = msg->data[0] > .5;
          m.lateral = msg->data[1];
          m.heading = msg->data[2];
          m.lookahead = msg->data[3];
          m.curvature = msg->data[4];
          m.confidence = msg->data[5];
          if (msg->data.size() >= 8) // boundary states: 2 = detected this frame
            m.detected = msg->data[6] > 1.5f || msg->data[7] > 1.5f;
          if (msg->data.size() >= 11) {
            m.near_lateral = msg->data[8];
            m.near_heading = msg->data[9];
          } else { // older perception output: only the lookahead point exists
            m.near_lateral = m.lateral;
            m.near_heading = m.heading;
          }
          m.time = now().seconds();
          if (m.time < last_measure_) // /clock jumped back (simulator restart)
            controller_.reset();
          last_measure_ = m.time;
          controller_.measure(m, config_);
          measurement_ = m;
          received_ = steady();
        });
    parking_sign_ = create_subscription<std_msgs::msg::Bool>(
        parking_topic, rclcpp::QoS(10),
        [this, parking_speed, parking_duration](std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!parking_mission_active_ && parking_.receive(msg->data, steady())) {
            std_msgs::msg::Header event;
            event.stamp = now();
            parking_event_stamp_ = event.stamp;
            approach_start_->publish(event);
            RCLCPP_INFO(get_logger(), "주차 표지 확인: lane 속도 최대 %.2f m/s, %.1f초간 감속",
                        parking_speed, parking_duration);
          }
        });
    parking_ready_ = create_subscription<std_msgs::msg::Header>(
        "/parking/approach_ready", rclcpp::QoS(10),
        [this](std_msgs::msg::Header::ConstSharedPtr msg) {
          if (msg->stamp == parking_event_stamp_ && parking_.ready(steady()))
            RCLCPP_INFO(get_logger(), "주차 정지 보정 완료: 0.3 m/s 접근 제한을 시작합니다.");
        });
    parking_state_ = create_subscription<std_msgs::msg::String>(
        "/parking/control_state",rclcpp::QoS(1),[this](std_msgs::msg::String::ConstSharedPtr m){
          // Compact state is produced by ParkingNode; mux validates timestamps independently.
          if(m->data.find("\"phase\":\"RELEASE\"")!=std::string::npos ||
             m->data.find("\"phase\":\"RESET\"")!=std::string::npos)parking_.release();
        });
    parking_active_ = create_subscription<std_msgs::msg::Bool>(
        "/parking/active", rclcpp::QoS(1),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) { parking_mission_active_ = msg->data; });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "lane_fusion_pid_controller: %s -> /lane_fusion_pid/command%s", input.c_str(),
                vehicle ? " + /mission/lane" : " (vehicle output OFF)");
  }

private:
  void tick() {
    const double ros_now = now().seconds();
    const double dt = last_tick_ < 0 || ros_now < last_tick_ ? 0 : ros_now - last_tick_;
    last_tick_ = ros_now;
    const double age = received_ > 0 ? steady() - received_ : 1e9;
    const auto &t = controller_.step(dt, age <= config_["timeout"], config_,
                                     parking_.limit(steady(), config_["speed"]));

    ackermann_msgs::msg::AckermannDriveStamped out;
    out.header.stamp = now();
    out.header.frame_id = "base_link";
    out.drive.steering_angle = float(t.steering);
    out.drive.speed = float(t.speed);
    command_->publish(out);
    if (vehicle_)
      vehicle_->publish(out);

    std_msgs::msg::Float32MultiArray s;
    const auto &m = measurement_;
    s.data = {float(t.tracking), float(t.error), float(t.p), float(t.i), float(t.d), float(t.ff),
              float(t.target_steering), float(t.steering), float(t.speed), float(m.near_lateral),
              float(m.near_heading), float(m.curvature), float(m.confidence), float(std::min(age, 99.)),
              float(config_["active"]), float(m.lateral), float(std::min(t.lost, 99.))};
    state_->publish(s);
  }
  lane_pid::Config config_;
  lane_pid::Controller controller_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parking_state_;
  lane_pid::ParkingApproach parking_;
  builtin_interfaces::msg::Time parking_event_stamp_;
  bool parking_mission_active_ = false;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr parking_ready_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parking_active_;
  lane_pid::Measurement measurement_;
  double received_ = 0, last_tick_ = -1, last_measure_ = -1e18;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_, vehicle_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr approach_start_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parking_sign_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr parameters_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<LanePidController>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
