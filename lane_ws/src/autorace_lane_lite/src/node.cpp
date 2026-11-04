// Perception only: camera -> /lane_lite/target (+ debug image). No drive output.
#include "autorace_lane_lite/lane.hpp"
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <thread>

// /lane_lite/target data layout (metres/radians, left positive):
// [valid, lateral_m, heading_rad, lookahead_m, curvature_1pm, confidence,
//  left_state, right_state]   state: 0=lost 1=held 2=detected
class LaneLiteNode : public rclcpp::Node {
public:
  LaneLiteNode() : Node("lane_lite_node") {
    const auto file = declare_parameter<std::string>("config_file", "");
    if (!file.empty())
      config_ = lane_lite::Config::load(file);
    for (const auto &s : lane_lite::specs())
      config_.values[s.name] = declare_parameter<double>(s.name, config_[s.name]);
    config_.validate();
    const auto topic = declare_parameter<std::string>(
        "image_topic", "/camera/color/image_raw/compressed");
    const auto transport = declare_parameter<std::string>("transport", "compressed");
    timeout_ = declare_parameter<double>("image_timeout", .3);
    target_ = create_publisher<std_msgs::msg::Float32MultiArray>("/lane_lite/target",
                                                                rclcpp::QoS(1));
    debug_ = create_publisher<sensor_msgs::msg::Image>(
        "/lane_lite/debug", rclcpp::SensorDataQoS().keep_last(1));
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    if (transport == "compressed")
      compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
          topic, qos, [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(mutex_);
            compressed_image_ = m;
            cv_.notify_one();
          });
    else if (transport == "raw")
      raw_ = create_subscription<sensor_msgs::msg::Image>(
          topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(mutex_);
            raw_image_ = m;
            cv_.notify_one();
          });
    else
      throw std::invalid_argument("transport must be compressed or raw");
    // The tuner's "Apply" sends every lane_lite key atomically.
    parameters_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          rcl_interfaces::msg::SetParametersResult result;
          std::lock_guard<std::mutex> lock(mutex_);
          auto next = config_;
          try {
            for (const auto &p : params)
              if (next.values.count(p.get_name()))
                next.values[p.get_name()] = p.as_double();
              else if (p.get_name() != "use_sim_time")
                throw std::invalid_argument("Startup-only parameter: " + p.get_name());
            next.validate();
            config_ = next;
            ++revision_;
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
    worker_ = std::thread([this] { work(); });
  }
  ~LaneLiteNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_one();
    worker_.join();
  }

private:
  static double steady() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void publish(const lane_lite::Result *r) {
    std_msgs::msg::Float32MultiArray msg;
    if (r && r->valid)
      msg.data = {1.f, float(r->lateral_m), float(r->heading), float(r->lookahead_m),
                  float(r->curvature), float(r->confidence),
                  float(int(r->left.state)), float(int(r->right.state))};
    else
      msg.data = {0, 0, 0, 0, 0, 0, r ? float(int(r->left.state)) : 0.f,
                  r ? float(int(r->right.state)) : 0.f};
    target_->publish(msg);
  }
  void work() {
    lane_lite::Pipeline pipeline;
    size_t applied = ~size_t(0);
    while (true) {
      sensor_msgs::msg::Image::ConstSharedPtr raw;
      sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed;
      lane_lite::Config config;
      size_t revision;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stop_ || raw_image_ || compressed_image_; });
        if (stop_)
          return;
        raw = std::move(raw_image_);
        compressed = std::move(compressed_image_);
        config = config_;
        revision = revision_;
      }
      try {
        if (revision != applied) {
          pipeline.configure(config);
          applied = revision;
        }
        const auto header = raw ? raw->header : compressed->header;
        double stamp = rclcpp::Time(header.stamp).seconds();
        if (stamp <= 0)
          stamp = now().seconds();
        const cv::Mat bgr = raw ? cv_bridge::toCvCopy(raw, "bgr8")->image
                                : cv::imdecode(compressed->data, cv::IMREAD_COLOR);
        if (bgr.empty())
          throw std::runtime_error("cannot decode image");
        const auto r = pipeline.process(bgr, stamp);
        std::lock_guard<std::mutex> lock(mutex_);
        publish(&r);
        processed_ = steady();
        if (debug_->get_subscription_count()) {
          auto h = header;
          h.frame_id = "lane_lite_bev";
          debug_->publish(*cv_bridge::CvImage(h, "bgr8", r.path_view).toImageMsg());
        }
      } catch (const std::exception &e) {
        pipeline.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "lane_lite: %s", e.what());
      }
    }
  }
  lane_lite::Config config_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool stop_ = false;
  size_t revision_ = 0;
  double timeout_ = .3, processed_ = 0;
  sensor_msgs::msg::Image::ConstSharedPtr raw_image_;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_;
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
    rclcpp::spin(std::make_shared<LaneLiteNode>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
