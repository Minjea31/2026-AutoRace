// Perception only: camera (+ /lane_fusion/deep_mask, /odom) -> /lane_fusion/target (+ debug image).
#include "autorace_lane_fusion/deep_latest.hpp"
#include "autorace_lane_fusion/lane.hpp"
#include "autorace_lane_fusion/odometry.hpp"
#include <condition_variable>
#include <limits>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <thread>
#include <atomic>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
            odom_.add(t, m->twist.twist.linear.x, fresh_gyro ? gyro_ : m->twist.twist.angular.z);
          });
    target_ = create_publisher<std_msgs::msg::Float32MultiArray>("/lane_fusion/target", rclcpp::QoS(1));
    // Fork sign from YOLO: false = A (left), true = B (right); published only
    // while a sign is confirmed. Acted on after fork_sign_count messages of the
    // same direction, each within fork_sign_gap s of the previous one (an
    // opposite one restarts the count); the pipeline then latches it until
    // the fork is passed.
    const auto fork_topic = declare_parameter<std::string>("fork_topic", "/yolo/LR_sign");
    sign_count_needed_ = std::max<int64_t>(1, declare_parameter<int64_t>("fork_sign_count", 2));
    sign_gap_ = declare_parameter<double>("fork_sign_gap", 1.0);
    // Only a sign box of at least fork_sign_area px^2 (from /yolo/LR_detail)
    // counts. 1000 = the yolo_detect left/right minimum (act on the first
    // confirmation, user choice 2026-11-10). 2000 delayed the lane change to
    // ~45 px; at ~32 px (2-3 m before the island) it once drove the car past the
    // target lane into the wall. 3400 px^2 was too late (~0.7 m).
    // Live: ros2 param set /lane_fusion_node fork_sign_area N.
    declare_parameter<double>("fork_sign_area", 1000.);
    if (!fork_topic.empty())
      fork_sub_ = create_subscription<std_msgs::msg::Bool>(
          fork_topic, rclcpp::QoS(10), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
            const double t = steady();
            std::lock_guard<std::mutex> lock(mutex_);
            if (t - fork_detail_time_ > 2.) // no box details (older yolo_detect): count the flag alone
              countSign(m->data ? 1 : -1, std::numeric_limits<double>::infinity(), t);
          });
    // Fork snapshot: at every fork state change one frame is rendered with the
    // debug views and published latched on /lane_fusion/fork_debug (camera with
    // the followed boundaries / centre and the sign box from /yolo/LR_detail,
    // BEV candidates, BEV followed path), and saved as JPEG. Nothing is drawn
    // between changes.
    fork_detail_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        declare_parameter<std::string>("fork_detail_topic", "/yolo/LR_detail"), rclcpp::QoS(10),
        [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {
          const double area_min = get_parameter("fork_sign_area").as_double();
          std::lock_guard<std::mutex> lock(mutex_);
          fork_detail_ = m->data;
          fork_detail_time_ = steady();
          if (m->data.size() >= 2)
            countSign(m->data[0] > .5f ? 1 : -1, m->data[1] >= area_min ? 1. : 0., fork_detail_time_);
        });
    fork_debug_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "/lane_fusion/fork_debug", rclcpp::QoS(1).reliable().transient_local());
    snapshot_dir_ = declare_parameter<std::string>("fork_snapshot_dir", "");
    if (snapshot_dir_.empty() && std::getenv("HOME"))
      snapshot_dir_ = std::string(std::getenv("HOME")) + "/fork_snapshots";
    // Parking approach (main a95746e): after /parking/approach_start, a 10 Hz
    // white-floor observation of the near image band until /parking/active.
    floor_ = create_publisher<std_msgs::msg::Float64MultiArray>("/parking/floor_observation", rclcpp::QoS(1));
    floor_start_ = create_subscription<std_msgs::msg::Header>(
        "/parking/approach_start", rclcpp::QoS(1), [this](std_msgs::msg::Header::ConstSharedPtr) { floor_active_ = true; });
    floor_active_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/parking/active", rclcpp::QoS(1), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data)
            floor_active_ = false;
        });
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
      int sign = 0, seen = 0;
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
        sign = fork_sign_;
        fork_sign_ = 0;
        seen = !sign_blocked_ && steady() - sign_seen_time_ <= sign_gap_ ? sign_seen_dir_ : 0;
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
        pipeline.forkSignSeen(seen);
        if (sign) {
          pipeline.forkSign(sign);
          sign_time_ = steady();
        }
        const bool debug_wanted = debug_ && debug_->get_subscription_count() > 0;
        const bool snapshot = snapshot_pending_;
        const bool views = debug_wanted || snapshot;
        const auto r = pipeline.process(bgr, stamp, views, deep);
        if (floor_active_ && steady() - floor_last_ >= .1)
          observeFloor(bgr, frame->header.stamp, config);
        if (r.fork_state != fork_state_) {
          RCLCPP_INFO(get_logger(), "fork: %s", r.fork_state.empty() ? "off" : r.fork_state.c_str());
          fork_state_ = r.fork_state;
          snapshot_pending_ = true; // drawn on the next frame
        } else if (snapshot) {
          snapshot_pending_ = false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (fork_active_ && !r.fork) { // fork passed: the sign still in view must not restart it
          sign_blocked_ = true;
          fork_sign_ = 0;
          sign_seen_count_ = 0;
        }
        fork_active_ = r.fork != 0;
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
        if (snapshot && !r.overlay.empty())
          publishSnapshot(r, frame->header);
      } catch (const std::exception &e) {
        pipeline.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "lane_fusion: %s", e.what());
      }
    }
  }
  void observeFloor(const cv::Mat &bgr, const builtin_interfaces::msg::Time &stamp, const lane_lite::Config &config) {
    const int x0 = int(bgr.cols * .20), x1 = int(std::ceil(bgr.cols * .80));
    const int y0 = int(bgr.rows * .86), y1 = int(std::ceil(bgr.rows * .92));
    cv::Mat blur, hsv, white, cols;
    cv::GaussianBlur(bgr(cv::Rect(x0, y0, x1 - x0, y1 - y0)), blur, cv::Size(5, 5), 0);
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, config["white_v_min"]), cv::Scalar(179, config["white_s_max"], 255), white);
    cv::reduce(white, cols, 0, cv::REDUCE_MAX);
    std::vector<cv::Point> points;
    cv::findNonZero(white, points);
    double angle = 0.;
    if (points.size() > 1) {
      cv::Vec4f line;
      cv::fitLine(points, line, cv::DIST_L2, 0, .01, .01);
      angle = std::atan2(line[1], line[0]);
      if (angle > M_PI / 2)
        angle -= M_PI;
      if (angle < -M_PI / 2)
        angle += M_PI;
    }
    std_msgs::msg::Float64MultiArray obs;
    // [capture sec, capture nanosec, white ratio, horizontal coverage, angle rad]
    obs.data = {double(stamp.sec), double(stamp.nanosec), double(cv::countNonZero(white)) / white.total(),
                double(cv::countNonZero(cols)) / white.cols, angle};
    floor_->publish(obs);
    floor_last_ = steady();
  }
  // ASCII only (cv::putText has no Hangul).
  static std::string phaseName(const std::string &state) {
    if (state.empty())
      return "OFF";
    if (state.find("표지판 없음") != std::string::npos)
      return "DEFAULT (island seen, no sign)";
    if (state.find("조향") != std::string::npos)
      return state.find("2차로") != std::string::npos ? "STEER 2 lanes" : "STEER 1 lane";
    if (state.find("분기") != std::string::npos)
      return "BRANCH lane keeping";
    if (state.find("바깥") != std::string::npos)
      return "KEEP outer lane";
    return "?";
  }
  // One /yolo/LR_sign event (mutex_ held). `near` < 1: the box is still too
  // small (far), seen but not counted. After a fork ends, the sign that kept
  // being seen is ignored until it has been gone for fork_sign_gap s, so the
  // same sign cannot start the manoeuvre again.
  void countSign(int direction, double near, double t) {
    const bool continued = direction == sign_seen_dir_ && t - sign_seen_time_ <= sign_gap_;
    sign_seen_dir_ = direction;
    sign_seen_time_ = t;
    if (sign_blocked_ && continued)
      return;
    sign_blocked_ = false;
    if (!continued || near < 1)
      sign_seen_count_ = 0;
    if (near >= 1 && ++sign_seen_count_ >= sign_count_needed_)
      fork_sign_ = direction;
  }
  // Called with mutex_ held.
  void publishSnapshot(const lane_lite::Result &r, const std_msgs::msg::Header &header) {
    const double now_s = steady(), detail_age = now_s - fork_detail_time_;
    const bool detail = fork_detail_.size() >= 9 && detail_age < 10;
    cv::Mat camera = r.overlay.clone();
    std::string sign_text = "sign: none received";
    if (detail) {
      const auto &d = fork_detail_;
      const std::string letter = d[0] > .5f ? "B (right)" : "A (left)";
      const cv::Rect box{cv::Point{int(d[5]), int(d[6])}, cv::Point{int(d[7]), int(d[8])}};
      cv::rectangle(camera, box, {0, 255, 255}, 2);
      char label[64];
      std::snprintf(label, sizeof label, "%s %.0fpx2", d[0] > .5f ? "B" : "A", d[1]);
      cv::putText(camera, label, {box.x, std::max(14, box.y - 6)}, cv::FONT_HERSHEY_SIMPLEX, .55, {0, 0, 0}, 4);
      cv::putText(camera, label, {box.x, std::max(14, box.y - 6)}, cv::FONT_HERSHEY_SIMPLEX, .55, {0, 255, 255}, 1);
      char text[200];
      std::snprintf(text, sizeof text, "sign: %s  bbox %.0f px2 (%.0fx%.0f)  YOLO %.2f  (%.1f s ago)",
                    letter.c_str(), d[1], d[2], d[3], d[4], detail_age);
      sign_text = text;
    }
    std::vector<cv::Mat> panels = {camera, r.search_view, r.path_view};
    for (auto &p : panels)
      if (p.size() != camera.size())
        cv::resize(p, p, camera.size());
    const char *titles[] = {"camera + followed path + sign box", "BEV detected candidates (purple deep, orange HSV, green both)",
                            "BEV followed boundaries / centre / target"};
    for (size_t i = 0; i < panels.size(); ++i) {
      panels[i] = panels[i].clone();
      cv::putText(panels[i], titles[i], {8, panels[i].rows - 10}, cv::FONT_HERSHEY_SIMPLEX, .5, {0, 0, 0}, 3);
      cv::putText(panels[i], titles[i], {8, panels[i].rows - 10}, cv::FONT_HERSHEY_SIMPLEX, .5, {255, 255, 255}, 1);
    }
    cv::Mat row, out;
    cv::hconcat(panels, row);
    cv::Mat bar(64, row.cols, CV_8UC3, cv::Scalar(30, 30, 30));
    const std::string dir = r.fork < 0 ? "A (left)" : r.fork > 0 ? "B (right)" : "-";
    const bool by_sign = sign_time_ > 0 && now_s - sign_time_ < 15;
    const std::string line1 = "FORK " + dir + "   phase: " + phaseName(r.fork_state) + "   source: " +
                              (r.fork ? (by_sign ? "sign (/yolo/LR_sign)" : "default (no sign)") : "-") +
                              "   lane: " + r.state + " " + r.source;
    cv::putText(bar, line1, {10, 26}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 255, 0}, 2);
    cv::putText(bar, sign_text, {10, 54}, cv::FONT_HERSHEY_SIMPLEX, .6, {0, 255, 255}, 1);
    cv::vconcat(bar, row, out);
    std::vector<uchar> jpeg;
    cv::imencode(".jpg", out, jpeg, {cv::IMWRITE_JPEG_QUALITY, 85});
    sensor_msgs::msg::CompressedImage msg;
    msg.header = header;
    msg.format = "jpeg";
    msg.data = std::move(jpeg);
    fork_debug_->publish(msg);
    if (!snapshot_dir_.empty()) {
      std::error_code error;
      std::filesystem::create_directories(snapshot_dir_, error);
      char name[96];
      const auto t = std::time(nullptr);
      std::strftime(name, sizeof name, "%Y%m%d_%H%M%S", std::localtime(&t));
      std::string phase = phaseName(r.fork_state);
      for (auto &ch : phase)
        if (!std::isalnum(static_cast<unsigned char>(ch)))
          ch = '_';
      std::ofstream(snapshot_dir_ + "/fork_" + name + "_" + phase.substr(0, 20) + ".jpg", std::ios::binary)
          .write(reinterpret_cast<const char *>(msg.data.data()), std::streamsize(msg.data.size()));
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
      if (!frames)
        return;
      RCLCPP_INFO(logger,
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
  std::atomic<bool> floor_active_{false};
  double floor_last_ = 0.;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr floor_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr floor_start_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr floor_active_sub_;
  double timeout_ = .3, max_rate_ = 0, processed_ = 0, reported_ = 0, frame_rx_ = 0;
  int64_t done_stamp_ = 0; // capture stamp of the last processed frame
  Stats stats_;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr frame_;
  lane_lite::DeepLatest deep_;
  lane_lite::Odometry odom_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fork_sub_;
  int fork_sign_ = 0;      // confirmed /yolo/LR_sign not yet given to the pipeline
  int sign_seen_dir_ = 0, sign_seen_count_ = 0, sign_count_needed_ = 2;
  double sign_seen_time_ = -1e9, sign_gap_ = 1.0;
  bool sign_blocked_ = false, fork_active_ = false;
  bool snapshot_pending_ = false; // fork state changed: render the next frame
  double sign_time_ = -1, fork_detail_time_ = -1e9;
  std::vector<float> fork_detail_; // newest /yolo/LR_detail
  std::string snapshot_dir_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr fork_detail_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr fork_debug_;
  std::string fork_state_; // worker thread only
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
