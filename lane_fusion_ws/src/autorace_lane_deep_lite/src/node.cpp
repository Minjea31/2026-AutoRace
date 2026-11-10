// Deep-only lane perception: /lane_fusion/deep_mask -> /lane_deep/target.
// No camera image is decoded here; the mask carries the camera header.
//
// /lane_deep/target layout = /lane_fusion/target (metres/radians, left +):
// [valid, lateral_m, heading_rad, lookahead_m, curvature_1pm, confidence,
//  left_state, right_state, near_lateral_m, near_heading_rad, near_m]
// state: 0=lost 1=held 2=detected.
#include "autorace_lane_deep_lite/deep_lane.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class DeepLiteNode : public rclcpp::Node {
public:
  DeepLiteNode() : Node("lane_deep_lite_node") {
    const auto file = declare_parameter<std::string>("config_file", "");
    deep_lite::Config config;
    if (!file.empty())
      config = deep_lite::Config::load(file);
    for (const auto &s : deep_lite::specs())
      config.values[s.name] = declare_parameter<double>(s.name, config[s.name]);
    pipeline_.configure(config);
    timeout_ = declare_parameter<double>("mask_timeout", .3);
    const auto mask_topic = declare_parameter<std::string>("mask_topic", "/lane_fusion/deep_mask");
    const auto target_topic = declare_parameter<std::string>("target_topic", "/lane_deep/target");
    target_ = create_publisher<std_msgs::msg::Float32MultiArray>(target_topic, rclcpp::QoS(1));
    if (declare_parameter<bool>("debug", false))
      debug_ = create_publisher<sensor_msgs::msg::Image>("/lane_deep/debug",
                                                         rclcpp::SensorDataQoS().keep_last(1));
    mask_ = create_subscription<sensor_msgs::msg::Image>(
        mask_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { onMask(*m); });
    parameters_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
      rcl_interfaces::msg::SetParametersResult result;
      std::lock_guard<std::mutex> lock(mutex_);
      auto next = pipeline_.config();
      try {
        for (const auto &p : params)
          if (next.values.count(p.get_name()))
            next.values[p.get_name()] = p.as_double();
          else if (p.get_name() != "use_sim_time")
            throw std::invalid_argument("Startup-only parameter: " + p.get_name());
        next.validate();
        pipeline_.configure(next);
        result.successful = true;
      } catch (const std::exception &e) {
        result.reason = e.what();
      }
      return result;
    });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      std::lock_guard<std::mutex> lock(mutex_);
      if (steady() - processed_ > timeout_)
        publish(nullptr);
    });
  }

private:
  static double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void publish(const deep_lite::Result *r) {
    std_msgs::msg::Float32MultiArray msg;
    if (r && r->valid)
      msg.data = {1.f, float(r->lateral_m), float(r->heading), float(r->lookahead_m),
                  float(r->curvature), float(r->confidence), float(int(r->left.state)),
                  float(int(r->right.state)), float(r->near_lateral_m), float(r->near_heading),
                  float(r->near_m)};
    else
      msg.data = {0, 0, 0, 0, 0, 0, r ? float(int(r->left.state)) : 0.f,
                  r ? float(int(r->right.state)) : 0.f, 0, 0, 0};
    target_->publish(msg);
  }
  void onMask(const sensor_msgs::msg::Image &m) {
    if (m.encoding != "mono8" || m.step < m.width || m.data.size() < size_t(m.step) * m.height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "lane_deep_lite: mask must be mono8");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      double stamp = rclcpp::Time(m.header.stamp).seconds();
      if (stamp <= 0)
        stamp = now().seconds();
      const cv::Mat classes(int(m.height), int(m.width), CV_8U, const_cast<uint8_t *>(m.data.data()),
                            m.step);
      const bool views = debug_ && debug_->get_subscription_count() > 0;
      const auto r = pipeline_.process(classes, stamp, views);
      publish(&r);
      processed_ = steady();
      ++count_;
      ms_ += r.processing_ms;
      if (reported_ == 0)
        reported_ = processed_, count_ = 0, ms_ = 0;
      else if (processed_ - reported_ >= 5) {
        RCLCPP_INFO(get_logger(), "lane_deep_lite: %.1f Hz, %.2f ms/mask, %s",
                    count_ / (processed_ - reported_), ms_ / std::max<size_t>(1, count_),
                    r.state.c_str());
        reported_ = processed_;
        count_ = 0;
        ms_ = 0;
      }
      if (views) {
        auto h = m.header;
        h.frame_id = "lane_deep_bev";
        debug_->publish(*cv_bridge::CvImage(h, "bgr8", r.view).toImageMsg());
      }
    } catch (const std::exception &e) {
      pipeline_.reset();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "lane_deep_lite: %s", e.what());
    }
  }
  deep_lite::Pipeline pipeline_;
  std::mutex mutex_;
  double timeout_ = .3, processed_ = 0, reported_ = 0, ms_ = 0;
  size_t count_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  OnSetParametersCallbackHandle::SharedPtr parameters_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  cv::setNumThreads(1);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<DeepLiteNode>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
