// Pure pursuit on /lane_lite/target. Gains are ROS parameters (tune live with
// `ros2 param set /lane_lite_controller ...`), deliberately not in the lane GUI.
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class LaneLiteController : public rclcpp::Node {
public:
  LaneLiteController() : Node("lane_lite_controller") {
    declare_parameter("wheelbase", .20);        // m
    declare_parameter("axle_offset", .10);      // base_link -> rear axle (m)
    declare_parameter("speed", .25);            // m/s at full confidence
    declare_parameter("max_steering", .45);     // rad
    declare_parameter("steering_rate", 2.0);    // rad/s
    declare_parameter("heading_gain", 0.0);     // optional extra on path heading
    declare_parameter("steering_slowdown", 1.0); // v /= 1 + k|delta|
    declare_parameter("confidence_stop", .10);
    declare_parameter("confidence_full", .5);
    declare_parameter("acceleration", .5);      // m/s^2; braking is immediate
    declare_parameter("timeout", .3);           // s without fresh target -> stop
    const bool vehicle = declare_parameter("enable_vehicle_output", false);
    preview_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/lane_lite/command", rclcpp::QoS(1));
    if (vehicle)
      vehicle_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
          "/mission/lane", rclcpp::QoS(1));
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/lane_lite/target", rclcpp::QoS(1),
        [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {
          if (m->data.size() >= 6) {
            target_ = m->data;
            received_ = steady();
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
  }

private:
  static double steady() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  double p(const char *name) { return get_parameter(name).as_double(); }
  void tick() {
    const double now = steady(), dt = last_ < 0 ? 0 : std::min(.1, now - last_);
    last_ = now;
    double steer = 0, speed = 0;
    const bool fresh = !target_.empty() && now - received_ <= p("timeout") &&
                       target_[0] > .5;
    if (fresh) {
      const double y = target_[1], heading = target_[2], conf = target_[5];
      const double x = std::max(.05, target_[3] + p("axle_offset"));
      // Pure pursuit: curvature through the target point from the rear axle.
      steer = std::atan(2 * p("wheelbase") * y / (x * x + y * y)) +
              p("heading_gain") * heading;
      steer = std::clamp(steer, -p("max_steering"), p("max_steering"));
      const double quality = std::clamp((conf - p("confidence_stop")) /
                                            (p("confidence_full") - p("confidence_stop")),
                                        0., 1.);
      speed = p("speed") * quality / (1 + p("steering_slowdown") * std::abs(steer));
    }
    const double rate = p("steering_rate") * dt;
    steering_ += std::clamp(steer - steering_, -rate, rate);
    speed_ = speed < speed_ ? speed : std::min(speed, speed_ + p("acceleration") * dt);
    ackermann_msgs::msg::AckermannDriveStamped out;
    out.header.stamp = get_clock()->now();
    out.header.frame_id = "base_link";
    out.drive.steering_angle = float(steering_);
    out.drive.speed = float(speed_);
    preview_->publish(out);
    if (vehicle_)
      vehicle_->publish(out);
  }
  std::vector<float> target_;
  double received_ = 0, last_ = -1, steering_ = 0, speed_ = 0;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr preview_, vehicle_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneLiteController>());
  rclcpp::shutdown();
  return 0;
}
