// Perception only: camera (+ /lane_fusion/deep_mask, /odom) -> /lane_fusion/target (+ debug image).
#include "autorace_lane_fusion/deep_latest.hpp"
#include "autorace_lane_fusion/lane.hpp"
#include "autorace_lane_fusion/odometry.hpp"
#include "autorace_lane_fusion/cone_exit_prior.hpp"
#include <condition_variable>
#include <limits>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <thread>
#include <atomic>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

// /lane_fusion/target data layout (metres/radians, left positive):
// [valid, lateral_m, heading_rad, lookahead_m, curvature_1pm, confidence,
//  left_state, right_state, near_lateral_m, near_heading_rad, near_m]
// state: 0=lost 1=held 2=detected.
//
// Asynchronous fusion: the worker always takes the NEWEST camera frame (the
// keep-last-1 queue drops older ones) and never waits for the network. HSV
// runs on every processed frame; the newest deep mask captured at most
// deep_max_age before the frame is moved into it with the odometry between
// the two captures (all times = receive time on this node's steady clock, so
// camera and odometry clocks need not agree). max_rate (Hz, default 0 = every
// frame the worker keeps up with) can cap the processing rate to leave CPU for
// the network and other nodes; on the car bags 20 Hz instead of 30 Hz cost
// ~10 % of the detections in bends (same for lane_ws), so cap only if needed.
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
    max_rate_ = declare_parameter<double>("max_rate", 0.);
    const auto deep_topic = declare_parameter<std::string>("deep_topic", "/lane_fusion/deep_mask");
    // Wheel odometry: aligns older deep masks and moves the tracks / a held
    // centre path with the car. Empty topic or no messages: masks older than
    // one frame are not used and remembered paths stay in place.
    // Yaw rate from the IMU gyro when it is there: on car 06 bags the /odom
    // angular.z read about 2x the gyro, which rotated older masks twice too far.
    const auto odom = declare_parameter<std::string>("odom_topic", "/odom");
    const auto imu = declare_parameter<std::string>("imu_topic", "/imu");
    if (!imu.empty())
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          imu, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(mutex_);
            gyro_ = m->angular_velocity.z;
            gyro_time_ = steady();
          });
    if (!odom.empty())
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom, rclcpp::QoS(20), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
            const double t = steady();
            std::lock_guard<std::mutex> lock(mutex_);
            const bool fresh_gyro = t - gyro_time_ < .1;
            const double w = fresh_gyro ? gyro_ : m->twist.twist.angular.z;
            odom_.add(t, m->twist.twist.linear.x, w);
            cone_exit_prior_.add(t, m->twist.twist.linear.x, w);
          });
    // Cone exit: the cone node marks its exit wait + lane hold; with a coherent
    // recent turn this supplies a role prior for an ambiguous single boundary.
    cone_exit_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cone_lane/exit_lane_active", rclcpp::QoS(1), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          const double t = steady();
          const int before = cone_exit_prior_.side(t);
          cone_exit_prior_.signal(m->data, t);
          const int after = cone_exit_prior_.side(t);
          if (before != after)
            RCLCPP_INFO(get_logger(), "Cone exit boundary prior: %s",
                        after == 0 ? "left (recent right turn)" : after == 1 ? "right (recent left turn)" : "off");
        });
    yellow_ = create_publisher<nav_msgs::msg::Path>("/lane_fusion/yellow_boundary", rclcpp::QoS(1));
    target_ = create_publisher<std_msgs::msg::Float32MultiArray>("/lane_fusion/target", rclcpp::QoS(1));
    // Debug picture only on request: off, nothing is advertised or drawn.
    if (declare_parameter<bool>("debug", false))
      debug_ = create_publisher<sensor_msgs::msg::Image>("/lane_fusion/debug", rclcpp::SensorDataQoS().keep_last(1));
    compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        topic, rclcpp::SensorDataQoS().keep_last(1), [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
          const double t = steady();
          std::lock_guard<std::mutex> lock(mutex_);
          frame_ = m;
          frame_rx_ = t;
          deep_.addFrame(lane_lite::DeepLatest::nanos(m->header.stamp), t);
          cv_.notify_one();
        });
    mask_ = create_subscription<sensor_msgs::msg::Image>(
        deep_topic, rclcpp::SensorDataQoS().keep_last(4), [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          const double t = steady();
          std::lock_guard<std::mutex> lock(mutex_);
          deep_.addMask(m, t);
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
    double credit = 1, last_rx = -1, previous_rx = -1;
    while (true) {
      sensor_msgs::msg::CompressedImage::ConstSharedPtr frame;
      sensor_msgs::msg::Image::ConstSharedPtr mask;
      lane_lite::DeepInput deep;
      lane_lite::Config config;
      size_t revision;
      double rx, step[3] = {0, 0, 0};
      bool stepped = false;
      int cone_exit_side = -1;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stop_ || frame_; });
        if (stop_)
          return;
        frame = std::move(frame_);
        rx = frame_rx_;
        // Token bucket on the frame arrivals: 30 fps camera, 20 Hz cap ->
        // two of every three frames, each processed as soon as it arrives.
        if (max_rate_ > 0) {
          credit = std::min(1.5, credit + (last_rx < 0 ? 1 : (rx - last_rx) * max_rate_));
          last_rx = rx;
          if (credit < 1)
            continue;
          credit -= 1;
        }
        const auto stamp_ns = lane_lite::DeepLatest::nanos(frame->header.stamp);
        if (stamp_ns <= done_stamp_) // never step the tracker back in time
          continue;
        done_stamp_ = stamp_ns;
        config = config_;
        if (config["cone_exit_role_bias"] > 0)
          cone_exit_side = cone_exit_prior_.side(steady());
        revision = revision_;
        bool own = false;
        double age = -1;
        mask = deep_.pick(stamp_ns, 1., age, own); // the pipeline judges age and alignment
        if (mask) {
          deep.mask = cv::Mat(int(mask->height), int(mask->width), CV_8U, const_cast<uint8_t *>(mask->data.data()),
                              mask->step);
          deep.age = age;
          deep.moved = odom_.between(rx - age, rx, deep.dx, deep.dy, deep.dyaw);
        }
        if (previous_rx > 0)
          stepped = odom_.between(previous_rx, rx, step[0], step[1], step[2]);
      }
      previous_rx = rx;
      try {
        if (revision != applied) {
          pipeline.configure(config);
          applied = revision;
        }
        double stamp = rclcpp::Time(frame->header.stamp).seconds();
        if (stamp <= 0)
          stamp = now().seconds();
        const double begin = steady();
        const cv::Mat bgr = cv::imdecode(frame->data, cv::IMREAD_COLOR);
        if (bgr.empty())
          throw std::runtime_error("cannot decode image");
        const double decoded = steady();
        if (stepped)
          pipeline.move(step[0], step[1], step[2]);
        const bool debug_wanted = debug_ && debug_->get_subscription_count() > 0;
        const auto r = pipeline.process(bgr, stamp, debug_wanted, deep, cone_exit_side);
        nav_msgs::msg::Path boundary;
        boundary.header = frame->header;
        boundary.header.frame_id = "base_link";
        const lane_lite::Line *yellow = nullptr;
        for (const auto *line : {&r.left, &r.right})
          if (line->yellow && line->state == lane_lite::LineState::Detected && line->path.size() >= 2 &&
              (!yellow || line->quality > yellow->quality)) yellow = line;
        if (yellow) {
          const double w = config["width"], h = config["height"];
          const double cu = config["vehicle_center_u"] * (w - 1);
          const double mpx = config["lane_width_m"] / config["lane_width_px"];
          const double mrow = (config["bev_far_m"] - config["bev_near_m"]) / (h - 1);
          for (const auto &point : yellow->path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = boundary.header;
            pose.pose.position.x = config["bev_near_m"] + (h - 1 - point.y) * mrow;
            pose.pose.position.y = (cu - point.x) * mpx;
            pose.pose.orientation.w = 1.;
            boundary.poses.push_back(pose);
          }
        }
        yellow_->publish(boundary);
        std::lock_guard<std::mutex> lock(mutex_);
        publish(&r);
        processed_ = steady();
        // Capture -> publish, when the camera stamps the system clock.
        const double capture = now().seconds() - stamp;
        stats_.add(r, mask != nullptr, (decoded - begin) * 1e3, (processed_ - rx) * 1e3,
                   capture >= 0 && capture < 1 ? capture * 1e3 : -1);
        if (processed_ - reported_ >= 5) {
          if (reported_ > 0)
            stats_.log(get_logger(), processed_ - reported_, deep_.latency());
          reported_ = processed_;
          stats_ = {};
        }
        if (debug_wanted) {
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
    size_t frames = 0, valid = 0, offered = 0, used = 0, captured = 0;
    double age = 0, decode = 0, cv = 0, latency = 0, worst = 0, capture = 0;
    void add(const lane_lite::Result &r, bool mask, double decode_ms, double latency_ms, double capture_ms) {
      ++frames;
      valid += r.valid, offered += mask, used += r.deep_used;
      age += r.deep_used ? r.deep_age : 0;
      decode += decode_ms, cv += r.processing_ms, latency += latency_ms;
      worst = std::max(worst, latency_ms);
      if (capture_ms >= 0)
        ++captured, capture += capture_ms;
    }
    void log(const rclcpp::Logger &logger, double seconds, double network) const {
      if (!frames || seconds <= 0 || frames / seconds > 20.0)
        return;
      RCLCPP_WARN(logger,
                  "lane_fusion: %.1f Hz | valid %zu%% | deep used %zu%% (mean age %.0f ms, offered %zu%%) | "
                  "decode %.1f + CV %.1f ms | receive->publish %.1f ms (max %.1f) | capture->publish %s | "
                  "network latency %.0f ms",
                  frames / seconds, 100 * valid / frames, 100 * used / frames, used ? 1000 * age / used : 0.,
                  100 * offered / frames, decode / frames, cv / frames, latency / frames, worst,
                  captured ? (std::to_string(int(capture / captured)) + " ms").c_str() : "n/a", network * 1000);
    }
  };
  lane_lite::Config config_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool stop_ = false;
  size_t revision_ = 0;
  double gyro_ = 0, gyro_time_ = -1e9; // newest IMU yaw rate
  double timeout_ = .3, max_rate_ = 0, processed_ = 0, reported_ = 0, frame_rx_ = 0;
  int64_t done_stamp_ = 0; // capture stamp of the last processed frame
  Stats stats_;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr frame_;
  lane_lite::DeepLatest deep_;
  lane_lite::Odometry odom_;
  lane_lite::ConeExitPrior cone_exit_prior_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cone_exit_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr yellow_;
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
