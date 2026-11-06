// Perception only: camera -> /lane_fusion/target (+ debug image). No drive output.
#include "autorace_lane_fusion/lane.hpp"
#include <condition_variable>
#include <deque>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <thread>

// /lane_fusion/target data layout (metres/radians, left positive):
// [valid, lateral_m, heading_rad, lookahead_m, curvature_1pm, confidence,
//  left_state, right_state, near_lateral_m, near_heading_rad, near_m]
// state: 0=lost 1=held 2=detected. lateral/heading = lookahead target point,
// near_* = nearest centre-path point.
//
// Deep fusion (deep_mode > 0): /lane_fusion/deep_mask (mono8 class ids, camera header
// copied) is paired with the camera frame of the SAME stamp. While masks keep
// arriving, a frame waits up to deep_wait for its mask; frames the network
// skipped are dropped. If no mask came for deep_timeout, frames run HSV-only
// at once until masks resume.
class LaneLiteNode : public rclcpp::Node {
public:
  LaneLiteNode() : Node("lane_fusion_node") {
    const auto file = declare_parameter<std::string>("config_file", "");
    if (!file.empty())
      config_ = lane_lite::Config::load(file);
    for (const auto &s : lane_lite::specs())
      config_.values[s.name] = declare_parameter<double>(s.name, config_[s.name]);
    config_.validate();
    const auto topic = declare_parameter<std::string>(
        "image_topic", "/camera/color/image_raw/compressed");
    timeout_ = declare_parameter<double>("image_timeout", .3);
    deep_wait_ = declare_parameter<double>("deep_wait", .15);
    deep_timeout_ = declare_parameter<double>("deep_timeout", .5);
    const auto deep_topic = declare_parameter<std::string>("deep_topic", "/lane_fusion/deep_mask");
    target_ = create_publisher<std_msgs::msg::Float32MultiArray>("/lane_fusion/target",
                                                                rclcpp::QoS(1));
    // Debug picture (/lane_fusion/debug) only on request: off, the node neither
    // advertises the topic nor builds any display image (saves CPU on the car).
    if (declare_parameter<bool>("debug", false))
      debug_ = create_publisher<sensor_msgs::msg::Image>(
          "/lane_fusion/debug", rclcpp::SensorDataQoS().keep_last(1));
    // Team camera input is JPEG CompressedImage only (real Orbbec and simulator).
    compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          frames_.push_back({m, steady()});
          while (frames_.size() > 8)
            frames_.pop_front();
          cv_.notify_one();
        });
    deep_ = create_subscription<sensor_msgs::msg::Image>(
        deep_topic, rclcpp::SensorDataQoS().keep_last(4),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          if (m->encoding != "mono8" || m->step < m->width ||
              m->data.size() < size_t(m->step) * m->height)
            return;
          std::lock_guard<std::mutex> lock(mutex_);
          masks_.push_back(m);
          while (masks_.size() > 8)
            masks_.pop_front();
          mask_seen_ = steady();
          cv_.notify_one();
        });
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
                  float(int(r->left.state)), float(int(r->right.state)),
                  float(r->near_lateral_m), float(r->near_heading), float(r->near_m)};
    else
      msg.data = {0, 0, 0, 0, 0, 0, r ? float(int(r->left.state)) : 0.f,
                  r ? float(int(r->right.state)) : 0.f, 0, 0, 0};
    target_->publish(msg);
  }
  static int64_t nanos(const builtin_interfaces::msg::Time &t) {
    return int64_t(t.sec) * 1000000000 + t.nanosec;
  }
  // Under mutex_. Picks the next frame to process (and its mask, if any);
  // returns false when the worker should keep waiting.
  bool next(sensor_msgs::msg::CompressedImage::ConstSharedPtr &frame,
            sensor_msgs::msg::Image::ConstSharedPtr &mask) {
    if (frames_.empty())
      return false;
    const bool deep = config_["deep_mode"] > 0 && steady() - mask_seen_ < deep_timeout_;
    if (deep) {
      // Newest frame that has its mask.
      for (auto f = frames_.rbegin(); f != frames_.rend(); ++f)
        for (const auto &m : masks_)
          if (nanos(m->header.stamp) == nanos(f->msg->header.stamp)) {
            frame = f->msg;
            mask = m;
            const auto t = nanos(frame->header.stamp);
            while (!frames_.empty() && nanos(frames_.front().msg->header.stamp) <= t)
              frames_.pop_front();
            while (!masks_.empty() && nanos(masks_.front()->header.stamp) <= t)
              masks_.pop_front();
            return true;
          }
      if (steady() - frames_.back().arrival < deep_wait_)
        return false; // its mask may still come
    }
    frame = frames_.back().msg; // HSV only
    frames_.clear();
    ++hsv_only_;
    return true;
  }
  void work() {
    lane_lite::Pipeline pipeline;
    size_t applied = ~size_t(0);
    while (true) {
      sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed;
      sensor_msgs::msg::Image::ConstSharedPtr mask_msg;
      lane_lite::Config config;
      size_t revision;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stop_ && !next(compressed, mask_msg))
          cv_.wait_for(lock, std::chrono::milliseconds(10));
        if (stop_)
          return;
        config = config_;
        revision = revision_;
      }
      try {
        if (revision != applied) {
          pipeline.configure(config);
          applied = revision;
        }
        const auto header = compressed->header;
        double stamp = rclcpp::Time(header.stamp).seconds();
        if (stamp <= 0)
          stamp = now().seconds();
        const cv::Mat bgr = cv::imdecode(compressed->data, cv::IMREAD_COLOR);
        if (bgr.empty())
          throw std::runtime_error("cannot decode image");
        const bool views = debug_ && debug_->get_subscription_count() > 0;
        cv::Mat deep;
        if (mask_msg)
          deep = cv::Mat(mask_msg->height, mask_msg->width, CV_8U,
                         const_cast<uint8_t *>(mask_msg->data.data()), mask_msg->step);
        const auto r = pipeline.process(bgr, stamp, views, deep);
        std::lock_guard<std::mutex> lock(mutex_);
        publish(&r);
        processed_ = steady();
        ++processed_count_;
        fused_ += r.deep_used;
        if (processed_ - reported_ >= 5) {
          if (config["deep_mode"] > 0)
            RCLCPP_INFO(get_logger(), "lane_fusion: %.1f Hz, deep-fused %zu / HSV-only %zu frames (5 s)",
                        processed_count_ / (processed_ - reported_), fused_, hsv_only_);
          reported_ = processed_;
          processed_count_ = fused_ = hsv_only_ = 0;
        }
        if (views) {
          auto h = header;
          h.frame_id = "lane_fusion_bev";
          debug_->publish(*cv_bridge::CvImage(h, "bgr8", r.path_view).toImageMsg());
        }
      } catch (const std::exception &e) {
        pipeline.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "lane_fusion: %s", e.what());
      }
    }
  }
  lane_lite::Config config_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool stop_ = false;
  size_t revision_ = 0;
  double timeout_ = .3, processed_ = 0, deep_wait_ = .15, deep_timeout_ = .5;
  double mask_seen_ = -1e9, reported_ = 0;
  size_t processed_count_ = 0, fused_ = 0, hsv_only_ = 0;
  struct Pending {
    sensor_msgs::msg::CompressedImage::ConstSharedPtr msg;
    double arrival;
  };
  std::deque<Pending> frames_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> masks_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr deep_;
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
