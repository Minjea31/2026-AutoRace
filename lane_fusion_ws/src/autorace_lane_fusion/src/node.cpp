// Perception only: camera (+ /lane_fusion/deep_mask) -> /lane_fusion/target (+ debug image).
#include "autorace_lane_fusion/deep_latest.hpp"
#include "autorace_lane_fusion/lane.hpp"
#include <condition_variable>
#include <deque>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <thread>
#include <atomic>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>

// /lane_fusion/target data layout (metres/radians, left positive):
// [valid, lateral_m, heading_rad, lookahead_m, curvature_1pm, confidence,
//  left_state, right_state, near_lateral_m, near_heading_rad, near_m]
// state: 0=lost 1=held 2=detected.
//
// Mask-synchronised fusion (sync_mask, default true): a frame is processed when
// ITS OWN deep mask arrives (paired by the copied camera stamp), so deep and
// HSV candidates always describe the same instant. Frames without a mask are
// skipped. While no mask has arrived for mask_wait seconds (deep node off or
// stalled) the newest frame runs alone with the newest mask no older than
// deep_max_age, else HSV only (the previous asynchronous behaviour, also used
// with sync_mask:=false). The JPEG is decoded only when HSV or the debug
// picture or the 10 Hz parking approach observation needs it.
class LaneFusionNode : public rclcpp::Node {
public:
  LaneFusionNode() : Node("lane_fusion_node") {
    const auto file = declare_parameter<std::string>("config_file", "");
    if (!file.empty())
      config_ = lane_lite::Config::load(file);
    for (const auto &s : lane_lite::specs())
      config_.values[s.name] = declare_parameter<double>(s.name, config_[s.name]);
    config_.validate();
    const auto topic = declare_parameter<std::string>("image_topic", "/camera/color/image_raw/compressed");
    timeout_ = declare_parameter<double>("image_timeout", .3);
    const auto deep_topic = declare_parameter<std::string>("deep_topic", "/lane_fusion/deep_mask");
    sync_ = declare_parameter<bool>("sync_mask", true);
    mask_wait_ = declare_parameter<double>("mask_wait", .3);
    // Wheel odometry moves a held centre path with the car during a short
    // dropout. Empty topic or no messages: the path is held in place.
    const auto odom = declare_parameter<std::string>("odom_topic", "/odom");
    if (!odom.empty())
      odom_ = create_subscription<nav_msgs::msg::Odometry>(
          odom, rclcpp::QoS(10), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
            const double t = steady();
            std::lock_guard<std::mutex> lock(mutex_);
            const double dt = odom_time_ > 0 ? std::clamp(t - odom_time_, 0., .25) : 0.; // /odom ~9 Hz
            odom_time_ = t;
            forward_ += m->twist.twist.linear.x * dt;
            yaw_ += m->twist.twist.angular.z * dt;
          });
    target_ = create_publisher<std_msgs::msg::Float32MultiArray>("/lane_fusion/target", rclcpp::QoS(1));
    floor_ = create_publisher<std_msgs::msg::Float64MultiArray>("/parking/floor_observation",rclcpp::QoS(1));
    floor_start_ = create_subscription<std_msgs::msg::Header>("/parking/approach_start",rclcpp::QoS(1),
        [this](std_msgs::msg::Header::ConstSharedPtr){floor_active_=true;});
    floor_active_sub_ = create_subscription<std_msgs::msg::Bool>("/parking/active",rclcpp::QoS(1),
        [this](std_msgs::msg::Bool::ConstSharedPtr m){if(m->data)floor_active_=false;});
    // Debug picture only on request: off, nothing is advertised or drawn.
    if (declare_parameter<bool>("debug", false))
      debug_ = create_publisher<sensor_msgs::msg::Image>("/lane_fusion/debug", rclcpp::SensorDataQoS().keep_last(1));
    compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        topic, rclcpp::SensorDataQoS().keep_last(1), [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          frame_ = m;
          recent_.push_back(m);
          while (recent_.size() > 30)
            recent_.pop_front();
          deep_.addFrame(lane_lite::DeepLatest::nanos(m->header.stamp), steady());
          cv_.notify_one();
        });
    mask_ = create_subscription<sensor_msgs::msg::Image>(
        deep_topic, rclcpp::SensorDataQoS().keep_last(4), [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!deep_.addMask(m, steady()) || !sync_)
            return;
          const auto stamp = lane_lite::DeepLatest::nanos(m->header.stamp);
          if (stamp <= done_stamp_)
            return;
          for (auto f = recent_.rbegin(); f != recent_.rend(); ++f)
            if (lane_lite::DeepLatest::nanos((*f)->header.stamp) == stamp) {
              pair_frame_ = *f, pair_mask_ = m; // newest pair wins; older pending one dropped
              cv_.notify_one();
              break;
            }
        });
    // The tuner's "Apply" sends every lane_lite key atomically.
    parameters_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
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
  ~LaneFusionNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_one();
    worker_.join();
  }

private:
  static double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void publish(const lane_lite::Result *r) {
    std_msgs::msg::Float32MultiArray msg;
    if (r && r->valid)
      msg.data = {1.f, float(r->lateral_m), float(r->heading), float(r->lookahead_m), float(r->curvature),
                  float(r->confidence), float(int(r->left.state)), float(int(r->right.state)),
                  float(r->near_lateral_m), float(r->near_heading), float(r->near_m)};
    else
      msg.data = {0, 0, 0, 0, 0, 0, r ? float(int(r->left.state)) : 0.f, r ? float(int(r->right.state)) : 0.f, 0, 0, 0};
    target_->publish(msg);
  }
  void work() {
    lane_lite::Pipeline pipeline;
    size_t applied = ~size_t(0);
    while (true) {
      sensor_msgs::msg::CompressedImage::ConstSharedPtr frame;
      sensor_msgs::msg::Image::ConstSharedPtr mask;
      double age = -1;
      bool own = false;
      lane_lite::Config config;
      size_t revision;
      double forward, yaw;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        // Masks flowing: wait for the next (frame, own mask) pair. Mask stream
        // silent for mask_wait: run the newest frame alone (wakes per frame).
        cv_.wait(lock, [this] {
          return stop_ || pair_frame_ || (frame_ && (!sync_ || !deep_.alive(steady(), mask_wait_)));
        });
        if (stop_)
          return;
        if (pair_frame_) {
          frame = pair_frame_;
          mask = pair_mask_;
          pair_frame_.reset();
          pair_mask_.reset();
          age = 0;
          own = true;
        } else {
          frame = frame_;
          frame_.reset();
        }
        const auto stamp_ns = lane_lite::DeepLatest::nanos(frame->header.stamp);
        if (stamp_ns <= done_stamp_) // never step the tracker back in time
          continue;
        done_stamp_ = stamp_ns;
        config = config_;
        revision = revision_;
        forward = forward_;
        yaw = yaw_;
        forward_ = yaw_ = 0;
        if (!mask)
          mask = deep_.pick(stamp_ns, config["deep_max_age"], age, own);
      }
      try {
        if (revision != applied) {
          pipeline.configure(config);
          applied = revision;
        }
        double stamp = rclcpp::Time(frame->header.stamp).seconds();
        if (stamp <= 0)
          stamp = now().seconds();
        const auto begin = steady();
        bool decoded = false;
        cv::Mat cached_image;
        auto image = [&]() {
          if (!cached_image.empty()) return cached_image;
          decoded = true;
          cv::Mat bgr = cv::imdecode(frame->data, cv::IMREAD_COLOR);
          if (bgr.empty())
            throw std::runtime_error("cannot decode image");
          cached_image=bgr;
          return cached_image;
        };
        const bool views = debug_ && debug_->get_subscription_count() > 0;
        cv::Mat classes;
        if (mask)
          classes = cv::Mat(int(mask->height), int(mask->width), CV_8U, const_cast<uint8_t *>(mask->data.data()),
                            mask->step);
        pipeline.move(forward, yaw);
        const auto r = pipeline.process(lane_lite::Pipeline::Image(image), stamp, views, classes);
        if (floor_active_ && steady()-floor_last_ >= .1) {
          const auto bgr=image();
          const int x0=int(bgr.cols*.20),x1=int(std::ceil(bgr.cols*.80));
          const int y0=int(bgr.rows*.86),y1=int(std::ceil(bgr.rows*.92));
          cv::Mat blur,hsv,white,cols;
          cv::GaussianBlur(bgr(cv::Rect(x0,y0,x1-x0,y1-y0)),blur,cv::Size(5,5),0);
          cv::cvtColor(blur,hsv,cv::COLOR_BGR2HSV);
          cv::inRange(hsv,cv::Scalar(0,0,config["white_v_min"]),cv::Scalar(179,config["white_s_max"],255),white);
          cv::reduce(white,cols,0,cv::REDUCE_MAX);
          std::vector<cv::Point> points;cv::findNonZero(white,points);
          double angle=0.;
          if(points.size()>1){cv::Vec4f line;cv::fitLine(points,line,cv::DIST_L2,0,.01,.01);
            angle=std::atan2(line[1],line[0]);
            if(angle>M_PI/2)angle-=M_PI;
            if(angle < -M_PI/2)angle+=M_PI;
          }
          std_msgs::msg::Float64MultiArray obs;
          // [capture sec, capture nanosec, white ratio, horizontal coverage, angle rad]
          obs.data={double(frame->header.stamp.sec),double(frame->header.stamp.nanosec),
            double(cv::countNonZero(white))/white.total(),double(cv::countNonZero(cols))/white.cols,angle};
          floor_->publish(obs);floor_last_=steady();
        }
        const double total = (steady() - begin) * 1000;
        std::lock_guard<std::mutex> lock(mutex_);
        publish(&r);
        processed_ = steady();
        stats_.add(r, mask != nullptr, own, age, decoded, total);
        if (processed_ - reported_ >= 5) {
          if (reported_ > 0)
            stats_.log(get_logger(), processed_ - reported_, deep_.latency());
          reported_ = processed_;
          stats_ = {};
        }
        if (views) {
          auto h = frame->header;
          h.frame_id = "lane_fusion_bev";
          debug_->publish(*cv_bridge::CvImage(h, "bgr8", r.path_view).toImageMsg());
        }
      } catch (const std::exception &e) {
        pipeline.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "lane_fusion: %s", e.what());
      }
    }
  }
  struct Stats {
    size_t frames = 0, own = 0, older = 0, none = 0, hsv = 0, decoded = 0, valid = 0;
    double age = 0, ms = 0, worst = 0;
    void add(const lane_lite::Result &r, bool mask, bool own_mask, double mask_age, bool decode, double total) {
      ++frames;
      own += mask && own_mask, older += mask && !own_mask, none += !mask;
      hsv += r.hsv_used, decoded += decode, valid += r.valid;
      age += mask ? mask_age : 0, ms += total, worst = std::max(worst, total);
    }
    void log(const rclcpp::Logger &logger, double seconds, double latency) const {
      if (!frames)
        return;
      RCLCPP_INFO(logger,
                  "lane_fusion: %.1f Hz | valid %zu%% | mask own %zu / older %zu (mean age %.0f ms) / none %zu | "
                  "HSV %zu%% | JPEG decoded %zu%% | %.1f ms (max %.1f) | network latency %.0f ms",
                  frames / seconds, 100 * valid / frames, own, older,
                  own + older ? 1000 * age / double(own + older) : 0., none, 100 * hsv / frames,
                  100 * decoded / frames, ms / frames, worst, latency * 1000);
    }
  };
  lane_lite::Config config_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool stop_ = false;
  size_t revision_ = 0;
  double timeout_ = .3, processed_ = 0, reported_ = 0;
  std::atomic<bool> floor_active_{false};
  double floor_last_=0.;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr floor_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr floor_start_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr floor_active_sub_;
  bool sync_ = true;
  double mask_wait_ = .3;
  int64_t done_stamp_ = 0; // capture stamp of the last processed frame
  std::deque<sensor_msgs::msg::CompressedImage::ConstSharedPtr> recent_; // for mask pairing
  sensor_msgs::msg::CompressedImage::ConstSharedPtr pair_frame_;
  sensor_msgs::msg::Image::ConstSharedPtr pair_mask_;
  double odom_time_ = 0, forward_ = 0, yaw_ = 0; // odometry since the last frame
  Stats stats_;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr frame_;
  lane_lite::DeepLatest deep_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
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
    rclcpp::spin(std::make_shared<LaneFusionNode>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
