// ROS 2 노드 objavoid_v3_node(C++). Python autorace_objavoid_v3/node.py와 같은 토픽·파라미터.
//  - executor 스레드 2개: 센서 콜백 그룹(scan/odom/imu/카메라)과 제어 타이머 그룹 분리
//  - 작업 스레드 1개: 최신 스캔만 꺼내 인지 + (필요할 때만) 경로 계획. 밀린 스캔은 버린다.
// 입력  /scan, /odom, /objavoid_v3/activate (Bool)  출력 command_topic(AckermannDriveStamped),
//       /objavoid_v3/state(String JSON), /objavoid_v3/done(Bool latched), /objavoid_v3/markers, /objavoid_v3/path
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "autorace_objavoid_v3_cpp/start_gate.hpp"
#include "autorace_objavoid_v3_cpp/lane.hpp"
#include "autorace_objavoid_v3_cpp/markers.hpp"
#include "autorace_objavoid_v3_cpp/mission.hpp"
#include "autorace_objavoid_v3_cpp/params.hpp"

using namespace std::chrono_literals;
using objavoid::Params;
using objavoid::State;

namespace {

double stamp_sec(const builtin_interfaces::msg::Time & st) {return st.sec + st.nanosec * 1e-9;}

double steady() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}

}  // namespace

class ObjAvoidV3Node : public rclcpp::Node
{
public:
  ObjAvoidV3Node()
  : Node("objavoid_v3_node")
  {
    Params p;
    declare_all(p);
    command_topic_ = declare_parameter<std::string>("command_topic", "/objavoid_v3/cmd");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    auto_activate_ = declare_parameter<bool>("auto_activate", false);
    marker_rate_ = dparam("marker_rate", 5.0);
    const auto lane_cfg = declare_parameter<std::string>("lane_config", "");
    const auto mask_topic = declare_parameter<std::string>("lane_mask_topic", "/lane_fusion/deep_mask");
    const auto image_topic = declare_parameter<std::string>("camera_topic", "/camera/color/image_raw/compressed");
    lane_rate_ = dparam("lane_rate", 5.0);
    lane_hsv_check_ = declare_parameter<bool>("lane_hsv_check", true);
    const auto ground_h = vparam("ground_homography", std::vector<double>(9, 0.0));
    const auto hsv = vparam("lane_hsv", std::vector<double>(4, 0.0));
    const double lane_range = dparam("lane_max_range", 2.5);
    const auto ground_scale = vparam("ground_scale", {1.0, 1.0});
    yaw_source_ = declare_parameter<std::string>("yaw_source", "odom");
    odom_scale_ = dparam("odom_scale", 1.0);
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
    marker_frame_ = declare_parameter<std::string>("marker_frame", "odom");
    // 갈림길 표지판(yolo_ws): false=왼쪽 A, true=오른쪽 B. 인식했을 때만 발행됨
    const auto sign_topic = declare_parameter<std::string>("fork_sign_topic", "/yolo/LR_sign");
    const auto detail_topic = declare_parameter<std::string>("fork_detail_topic", "/yolo/LR_detail");

    if (p.use_lane) {
      try {
        if (lane_cfg.empty()) {throw std::runtime_error("lane_config is empty");}
        gp_ = std::make_unique<objavoid::GroundProjection>(lane_cfg, lane_range, 1.5, ground_h);
        if (std::any_of(hsv.begin(), hsv.end(), [](double v) {return v != 0.0;}) && hsv.size() == 4) {
          gp_->hsv = {hsv[0], hsv[1], hsv[2], hsv[3]};
        }
        if (ground_scale.size() == 2 && (ground_scale[0] != 1.0 || ground_scale[1] != 1.0)) {
          gp_->set_scale(ground_scale[0], ground_scale[1]);
        }
      } catch (const std::exception & e) {  // 보정 파일이 없으면 차선 없이(사전 경계) 동작
        RCLCPP_ERROR(get_logger(), "lane_config load failed (%s): %s -> prior boundary only", lane_cfg.c_str(),
          e.what());
        p.use_lane = false;
        gp_.reset();
      }
    }
    core_ = std::make_unique<objavoid::ObjAvoidCore>(p);

    sensors_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions so;
    so.callback_group = sensors_;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr m) {on_scan(m);}, so);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(20),
        [this](nav_msgs::msg::Odometry::SharedPtr m) {on_odom(*m);}, so);
    if (yaw_source_ == "imu") {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(50),
          [this](sensor_msgs::msg::Imu::SharedPtr m) {on_imu(*m);}, so);
    }
    sign_sub_ = create_subscription<std_msgs::msg::Bool>(sign_topic, 10,
        [this](std_msgs::msg::Bool::SharedPtr m) {
          const int before = core_->fork_side();
          core_->on_sign(now_s(), m->data);
          const int after = core_->fork_side();
          if (after != before) {
            RCLCPP_INFO(get_logger(), "fork sign -> %s (votes L%d R%d)", after > 0 ? "LEFT/A" : "RIGHT/B",
              core_->fork_votes_l, core_->fork_votes_r);
          }
        }, so);
    // [right, area, w, h, conf, x0, y0, x1, y1, stamp_sec, stamp_nanosec] (원본 영상 px)
    detail_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(detail_topic, 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr m) {
          if (m->data.size() < 11) {return;}
          const double st = m->data[9] + m->data[10] * 1e-9;
          core_->on_sign_detail(st > 0 ? st : now_s(), m->data[0] > 0.5, 0.5 * (m->data[5] + m->data[7]));
        }, so);
    // 자동 시작 순서 조건: 보행자 횡단보도 미션(crosswalk_ws) 정지 -> 재출발 뒤에만 감시
    if (p.start_after_crosswalk) {
      const auto cw_qos = rclcpp::QoS(1).reliable();
      cw_start_sub_ = create_subscription<std_msgs::msg::Bool>(
        declare_parameter<std::string>("crosswalk_start_topic", "/start_crosswalk"), cw_qos,
        [this](std_msgs::msg::Bool::SharedPtr m) {
          if (m->data) {cw_seen_ = true; cw_last_cmd_ = steady(); cw_reset_ = true;}
        }, so);
      cw_cmd_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        declare_parameter<std::string>("crosswalk_command_topic", "/mission/crosswalk/command_01"), cw_qos,
        [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr) {
          cw_seen_ = true;
          cw_last_cmd_ = steady();
          cw_reset_ = true;
        }, so);
    }
    act_sub_ = create_subscription<std_msgs::msg::Bool>("/objavoid_v3/activate", 10,
        [this](std_msgs::msg::Bool::SharedPtr m) {on_activate(m->data);}, so);
    if (gp_) {
      mask_sub_ = create_subscription<sensor_msgs::msg::Image>(mask_topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::Image::SharedPtr m) {std::lock_guard<std::mutex> g(lane_mtx_); latest_mask_ = m;},
          so);
      if (lane_hsv_check_ || p.start_auto) {
        img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(image_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CompressedImage::SharedPtr m) {
              std::lock_guard<std::mutex> g(lane_mtx_);
              images_.push_back(m);
              while (images_.size() > 15) {images_.pop_front();}  // 최근 0.5 s만
            }, so);
      }
    }
    cmd_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(command_topic_, rclcpp::QoS(1));
    state_pub_ = create_publisher<std_msgs::msg::String>("/objavoid_v3/state", 10);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("/objavoid_v3/done", rclcpp::QoS(1).transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/objavoid_v3/markers", 2);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/objavoid_v3/path", 2);
    publish_done(false);

    worker_ = std::thread([this] {work_loop();});
    ctrl_period_ = 1.0 / p.control_rate;
    ctrl_timer_ = create_wall_timer(std::chrono::duration<double>(ctrl_period_), [this] {on_control();}, control_);
    status_timer_ = create_wall_timer(200ms, [this] {publish_status();}, control_);
    conflict_timer_ = create_wall_timer(1s, [this] {check_conflict();}, control_);
    pending_activate_ = auto_activate_;
    RCLCPP_INFO(get_logger(), "objavoid_v3 (C++) ready: cmd=%s scan=%s odom=%s auto_activate=%d",
      command_topic_.c_str(), scan_topic.c_str(), odom_topic.c_str(), auto_activate_);
  }

  ~ObjAvoidV3Node() override
  {
    running_ = false;
    scan_cv_.notify_all();
    if (worker_.joinable()) {worker_.join();}
  }

private:
  // ---------------------------------------------------------------- 파라미터
  rcl_interfaces::msg::ParameterDescriptor dyn() const
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.dynamic_typing = true;  // YAML 정수·실수 표기 차이 허용
    return d;
  }
  double dparam(const std::string & n, double def)
  {
    const auto v = declare_parameter(n, rclcpp::ParameterValue(def), dyn());
    return v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ? static_cast<double>(v.get<int64_t>()) :
           v.get<double>();
  }
  int iparam(const std::string & n, int def)
  {
    const auto v = declare_parameter(n, rclcpp::ParameterValue(static_cast<int64_t>(def)), dyn());
    return v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ? static_cast<int>(std::lround(v.get<double>())) :
           static_cast<int>(v.get<int64_t>());
  }
  bool bparam(const std::string & n, bool def) {return declare_parameter<bool>(n, def);}
  std::vector<double> vparam(const std::string & n, const std::vector<double> & def)
  {
    const auto v = declare_parameter(n, rclcpp::ParameterValue(def), dyn());
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto iv = v.get<std::vector<int64_t>>();
      return std::vector<double>(iv.begin(), iv.end());
    }
    return v.get<std::vector<double>>();
  }
  void declare_all(Params & p)
  {
#define OBJAVOID_DECL_D(n, def) p.n = dparam(#n, p.n);
#define OBJAVOID_DECL_I(n, def) p.n = iparam(#n, p.n);
#define OBJAVOID_DECL_B(n, def) p.n = bparam(#n, p.n);
#define OBJAVOID_DECL_V(n, def) p.n = vparam(#n, p.n);
#define OBJAVOID_DECL(t, n, def) OBJAVOID_DECL_##t(n, def)
    OBJAVOID_PARAMS(OBJAVOID_DECL)
#undef OBJAVOID_DECL
  }

  // ---------------------------------------------------------------- 입력
  double now_s() {return now().nanoseconds() * 1e-9;}

  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    const double t = stamp_sec(m.header.stamp), wz = m.angular_velocity.z;
    if (!has_imu_ || t <= imu_t_ || t - imu_t_ > 0.5) {
      if (!has_imu_) {imu_yaw_ = 0.0;}
      has_imu_ = true;
      imu_t_ = t;
      imu_w_ = wz;
      return;
    }
    imu_yaw_ += 0.5 * (imu_w_ + wz) * (t - imu_t_);
    imu_t_ = t;
    imu_w_ = wz;
  }

  void on_odom(const nav_msgs::msg::Odometry & m)
  {
    const double t = stamp_sec(m.header.stamp);
    const double v = m.twist.twist.linear.x * odom_scale_;
    if (yaw_source_ == "imu") {
      if (!has_imu_) {return;}
      // 방향은 IMU 자이로 적분, 위치는 odom 전진 속도 x odom_scale 적분
      if (!has_dead_ || t <= dead_t_ || t - dead_t_ > 0.5) {
        has_dead_ = true;
        dead_t_ = t; dead_x_ = 0.0; dead_y_ = 0.0;
      } else {
        dead_x_ += v * std::cos(imu_yaw_) * (t - dead_t_);
        dead_y_ += v * std::sin(imu_yaw_) * (t - dead_t_);
        dead_t_ = t;
      }
      core_->on_odom(t, dead_x_, dead_y_, imu_yaw_, v, imu_w_);
    } else {
      const auto & q = m.pose.pose.orientation;
      const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      core_->on_odom(t, m.pose.pose.position.x * odom_scale_, m.pose.pose.position.y * odom_scale_, yaw, v,
        m.twist.twist.angular.z);
    }
    if (pending_activate_) {try_activate();}
  }

  void on_scan(sensor_msgs::msg::LaserScan::SharedPtr m)
  {
    {
      std::lock_guard<std::mutex> g(scan_mtx_);
      if (latest_scan_) {++dropped_scans_;}  // 처리 못한 이전 스캔은 버림
      latest_scan_ = m;
      scan_recv_wall_ = steady();
    }
    scan_cv_.notify_one();
  }

  void on_activate(bool on)
  {
    if (on) {
      const State st = core_->get_state();
      if (st != State::IDLE && st != State::DONE) {return;}  // 재시작은 false 후 true
      pending_activate_ = true;
      try_activate();
    } else {
      pending_activate_ = false;
      core_->deactivate();
      RCLCPP_INFO(get_logger(), "deactivated");
    }
  }

  void try_activate()
  {
    if (core_->activate(now_s())) {
      pending_activate_ = false;
      done_sent_ = false;
      {
        std::lock_guard<std::mutex> g(stat_mtx_);
        cmd_periods_.clear();
        ctrl_ms_.clear();
      }
      publish_done(false);
      RCLCPP_INFO(get_logger(), "ACTIVATED: road frame fixed at current odom pose");
    }
  }

  void process_lane()
  {
    // 최신 마스크 1장(+ 같은 촬영 시각 원본) -> 차 기준 노란·흰 점 -> 코어. lane_rate 이하로.
    if (!gp_ || steady() - last_lane_wall_ < 1.0 / std::max(lane_rate_, 0.1)) {return;}
    sensor_msgs::msg::Image::SharedPtr m;
    std::deque<sensor_msgs::msg::CompressedImage::SharedPtr> imgs;
    {
      std::lock_guard<std::mutex> g(lane_mtx_);
      m.swap(latest_mask_);
      imgs = images_;
    }
    if (!m || m->encoding != "mono8" || m->height == 0) {return;}
    last_lane_wall_ = steady();
    const double st = stamp_sec(m->header.stamp);
    cv::Mat bgr;
    if (lane_hsv_check_) {
      sensor_msgs::msg::CompressedImage::SharedPtr near;
      for (auto & im : imgs) {
        if (std::abs(stamp_sec(im->header.stamp) - st) < 1e-3) {near = im;}
      }
      if (!near) {return;}  // 같은 프레임 원본이 없으면 이번 마스크는 건너뜀
      bgr = cv::imdecode(cv::Mat(1, static_cast<int>(near->data.size()), CV_8UC1, near->data.data()),
          cv::IMREAD_REDUCED_COLOR_2);
      if (bgr.empty()) {return;}
    }
    const cv::Mat mask(static_cast<int>(m->height), static_cast<int>(m->width), CV_8UC1, m->data.data(), m->step);
    const auto pts = gp_->yellow_points(mask, bgr);
    const auto wpts = gp_->white_points(mask, bgr);
    core_->on_lane(st, pts, gp_->view_points, &wpts);
  }

  // ---------------------------------------------------------------- 작업 스레드
  void work_loop()
  {
    while (running_ && rclcpp::ok()) {
      sensor_msgs::msg::LaserScan::SharedPtr msg;
      double recv = 0;
      {
        std::unique_lock<std::mutex> lk(scan_mtx_);
        scan_cv_.wait_for(lk, 200ms, [this] {return latest_scan_ != nullptr || !running_;});
        if (!running_) {break;}
        msg.swap(latest_scan_);
        recv = scan_recv_wall_;
      }
      try {
        process_lane();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "lane processing failed: %s", e.what());
      }
      if (!msg) {continue;}
      {
        std::lock_guard<std::recursive_mutex> g(core_->mutex());
        core_->scan_queue_ms.add((steady() - recv) * 1000);
      }
      if (!core_->active()) {
        try {
          start_gate(*msg);
        } catch (const std::exception & e) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "start gate failed: %s", e.what());
        }
        if (core_->active()) {continue;}
        preview(*msg);  // 활성화 전: 장애물 검출만 미리보기
        continue;
      }
      last_active_wall_ = steady();
      gate_hits_ = 0;
      const double t_now = now_s();
      const double st = stamp_sec(msg->header.stamp);
      if (t_now - st > core_->scan_timeout()) {continue;}  // 너무 오래된 스캔
      try {
        core_->process_scan(st, msg->ranges, msg->angle_min, msg->angle_increment, t_now);
      } catch (const std::exception & e) {  // 인지/계획 예외가 노드를 죽이지 않도록(경로 없음 -> 정지)
        RCLCPP_ERROR(get_logger(), "process_scan failed: %s", e.what());
        core_->clear_path();
      }
      const double w = steady();
      if (marker_rate_ > 0 && w - last_marker_wall_ >= 1.0 / marker_rate_) {
        last_marker_wall_ = w;
        publish_viz();
      }
    }
  }

  // ---------------------------------------------------------------- 자동 시작
  // 흰 정지선 + 그 너머 흰 점선 + 전방 장애물이 start_confirm 회 연속 -> 정지선을 원점으로 활성화
  void set_gate_status(const char * st)
  {
    std::lock_guard<std::mutex> lk(stat_mtx_);
    gate_status_ = st;
  }

  void start_gate(const sensor_msgs::msg::LaserScan & msg)
  {
    const objavoid::Params & p = core_->p;
    if (!p.start_auto || !gp_ || pending_activate_) {return;}
    const double w = steady();
    if (w - last_active_wall_ < p.start_rearm) {set_gate_status("rearm"); return;}
    if (p.start_after_crosswalk) {
      // 횡단보도 정지가 끝나고(명령 끊김) start_arm_dist 이상 간 뒤에만
      if (!cw_seen_) {set_gate_status("wait_crosswalk"); return;}
      if (w - cw_last_cmd_ < p.start_cw_quiet) {set_gate_status("crosswalk_active"); return;}
      if (cw_reset_.exchange(false)) {cw_armed_at_.reset();}
      const auto pose = core_->pose_at(now_s(), 0.3);
      if (!pose) {return;}
      if (!cw_armed_at_) {cw_armed_at_ = std::make_pair(pose->x, pose->y);}
      if (std::hypot(pose->x - cw_armed_at_->first, pose->y - cw_armed_at_->second) < p.start_arm_dist) {
        set_gate_status("after_crosswalk");
        return;
      }
    }
    if (w - last_gate_wall_ < 1.0 / std::max(lane_rate_, 0.1)) {return;}
    sensor_msgs::msg::CompressedImage::SharedPtr im;
    {
      std::lock_guard<std::mutex> g(lane_mtx_);
      if (!images_.empty()) {im = images_.back();}
    }
    if (!im) {set_gate_status("no_image"); return;}
    const double ist = stamp_sec(im->header.stamp);
    if (ist == last_gate_img_) {return;}
    last_gate_img_ = ist;
    last_gate_wall_ = w;
    const cv::Mat bgr = cv::imdecode(cv::Mat(1, static_cast<int>(im->data.size()), CV_8UC1, im->data.data()),
        cv::IMREAD_REDUCED_COLOR_2);
    if (bgr.empty()) {return;}
    const auto white = gp_->hsv_white_points(bgr, 2);
    // LiDAR 박스를 차 기준(base_link)으로
    const objavoid::LidarPose lp{p.lidar_x, p.lidar_y, p.lidar_yaw};
    const objavoid::RoadFrame f(0, 0, 0);
    const double hw = p.start_obs_y + 0.2;
    const objavoid::PriorBoundary wide({-1.0, p.start_obs_x_max + 1.0}, {hw, hw}, {-hw, -hw});
    std::vector<objavoid::Box> boxes;
    std::vector<objavoid::Pt2> pts;
    objavoid::detect(msg.ranges, msg.angle_min, msg.angle_increment, lp, f, wide, 0.0, p, boxes, pts);
    // 박스는 LIMO 크기로 보정돼 있음 -> 박스 안 LiDAR 점의 실제 좌우·앞뒤 범위로 기둥·벽 제외
    std::vector<objavoid::Box> solid;
    for (const auto & b : boxes) {
      double d0 = objavoid::kInf, d1 = -objavoid::kInf, s0 = objavoid::kInf, s1 = -objavoid::kInf;
      for (const auto & q : pts) {
        if (q.s >= b.s0 - 0.02 && q.s <= b.s1 + 0.02 && q.d >= b.d0 - 0.02 && q.d <= b.d1 + 0.02) {
          d0 = std::min(d0, q.d); d1 = std::max(d1, q.d); s0 = std::min(s0, q.s); s1 = std::max(s1, q.s);
        }
      }
      // 가는 기둥(표지판)과 넓은 벽·트랙 경계는 제외: LIMO 크기 물체만
      const double big = std::max(d1 - d0, s1 - s0);
      if (big >= p.start_obs_min_width && big <= p.start_obs_max_size) {solid.push_back(b);}
    }
    const auto g = objavoid::eval_start_gate(white, solid, p);
    gate_hits_ = g.ok ? gate_hits_ + 1 : 0;
    {
      std::ostringstream o;
      o << "{\"ok\":" << (g.ok ? "true" : "false") << ",\"hits\":" << gate_hits_ << ",\"line\":" <<
        (g.line ? objavoid::jnum(g.line_x) : "null") << ",\"span\":" << objavoid::jnum(g.line_span) <<
        ",\"line_pts\":" << g.line_pts << ",\"dash_l\":" << g.dashes_l << ",\"dash_r\":" << g.dashes_r <<
        ",\"obs\":" << g.obstacles << ",\"pair\":" << (g.pair ? "true" : "false") << ",\"obs_all\":" << boxes.size() << ",\"stripes\":" << g.stripes << ",\"after\":" << g.line_after << ",\"white\":" << white.size() << "}";
      std::lock_guard<std::mutex> lk(stat_mtx_);
      gate_json_ = o.str();
      gate_status_ = "watching";
    }
    if (gate_hits_ < p.start_confirm) {return;}
    gate_hits_ = 0;
    const double shift = p.start_origin_at_line && g.line ? g.line_x : 0.0;
    if (core_->activate(now_s(), shift)) {
      cw_seen_ = false;  // 다음 자동 시작은 다시 횡단보도 뒤에
      cw_armed_at_.reset();
      done_sent_ = false;
      {
        std::lock_guard<std::mutex> lk(stat_mtx_);
        cmd_periods_.clear();
        ctrl_ms_.clear();
      }
      publish_done(false);
      RCLCPP_INFO(get_logger(), "AUTO START: stop line %.2f m (0 = not used), dashes L%d R%d, obstacles %d, stripes %d -> "
        "road origin at %s", g.line ? g.line_x : 0.0, g.dashes_l, g.dashes_r, g.obstacles, g.stripes,
        p.start_origin_at_line && g.line ? "stop line" : "vehicle");
    }
  }

  void preview(const sensor_msgs::msg::LaserScan & msg)
  {
    if (marker_rate_ <= 0 || steady() - last_marker_wall_ < 1.0 / marker_rate_) {return;}
    last_marker_wall_ = steady();
    const auto pose = core_->pose_at(stamp_sec(msg.header.stamp), 0.3);
    if (!pose) {return;}
    auto arr = objavoid::preview_markers(*core_, msg.ranges, msg.angle_min, msg.angle_increment, pose->x, pose->y,
        pose->yaw, now());
    if (core_->p.start_auto && gp_) {
      std::string gs, gj;
      {
        std::lock_guard<std::mutex> lk(stat_mtx_);
        gs = gate_status_;
        gj = gate_json_;
      }
      for (auto & mk : arr.markers) {
        if (mk.ns == "state") {mk.text += "\n자동 시작(" + gs + "): " + (gj.empty() ? "-" : gj);}
      }
    }
    if (marker_frame_ != "odom") {objavoid::to_base(arr, pose->x, pose->y, pose->yaw, marker_frame_);}
    marker_pub_->publish(arr);
  }

  // ---------------------------------------------------------------- 제어
  void on_control()
  {
    const double w0 = steady();
    const double t = now_s();
    double dt = ctrl_period_;
    if (last_ctrl_wall_ > 0) {dt = w0 - last_ctrl_wall_;}
    last_ctrl_wall_ = w0;
    double v = 0, delta = 0;
    const bool out = core_->control(t, dt, v, delta);
    bool conflict;
    {
      std::lock_guard<std::mutex> g(stat_mtx_);
      conflict = !conflict_.empty();
    }
    if (out && !conflict) {
      ackermann_msgs::msg::AckermannDriveStamped msg;
      msg.header.stamp = now();
      msg.header.frame_id = "base_link";
      msg.drive.speed = static_cast<float>(v);
      msg.drive.steering_angle = static_cast<float>(objavoid::emulate_calibration(delta, core_->p));
      cmd_pub_->publish(msg);
      std::lock_guard<std::mutex> g(stat_mtx_);
      cmd_periods_.push_back(dt);
    }
    if (core_->get_state() == State::DONE && !done_sent_) {
      done_sent_ = true;
      publish_done(true);
      RCLCPP_INFO(get_logger(), "DONE (%s): stop publishing commands", core_->get_done_reason().c_str());
      publish_status();
    }
    std::lock_guard<std::mutex> g(stat_mtx_);
    ctrl_ms_.push_back((steady() - w0) * 1000);
  }

  void check_conflict()
  {
    // 같은 명령 토픽에 다른 발행자(예: Python V3, 기존 objavoid_node)가 있으면 발행 중단
    std::vector<std::string> others;
    for (const auto & info : get_publishers_info_by_topic(command_topic_)) {
      if (info.node_name() != get_name()) {others.push_back(info.node_name());}
    }
    std::sort(others.begin(), others.end());
    others.erase(std::unique(others.begin(), others.end()), others.end());
    std::lock_guard<std::mutex> g(stat_mtx_);
    if (others != conflict_) {
      if (!others.empty()) {
        std::string s;
        for (auto & o : others) {s += o + " ";}
        RCLCPP_ERROR(get_logger(), "other publishers on %s: %s-> commands paused", command_topic_.c_str(), s.c_str());
      }
      conflict_ = others;
    }
  }

  // ---------------------------------------------------------------- 보고
  void publish_done(bool v)
  {
    std_msgs::msg::Bool b;
    b.data = v;
    done_pub_->publish(b);
  }

  static std::string summary(std::vector<double> v, bool p99)
  {
    if (v.empty()) {return "";}
    if (v.size() > 5000) {v.erase(v.begin(), v.end() - 5000);}
    std::sort(v.begin(), v.end());
    double sum = 0;
    for (double x : v) {sum += x;}
    std::ostringstream o;
    if (p99) {
      o << "{\"n\":" << v.size() << ",\"mean\":" << objavoid::jnum(1000 * sum / v.size(), 2) << ",\"p99\":" <<
        objavoid::jnum(1000 * v[static_cast<size_t>(0.99 * (v.size() - 1))], 2) << ",\"max\":" <<
        objavoid::jnum(1000 * v.back(), 2) << "}";
    } else {
      o << "{\"mean\":" << objavoid::jnum(sum / v.size(), 3) << ",\"max\":" << objavoid::jnum(v.back(), 3) << "}";
    }
    return o.str();
  }

  void publish_status()
  {
    std::string js = core_->status_json();
    js.pop_back();  // 마지막 '}' 뒤에 노드 정보 추가
    std::ostringstream o;
    o << js;
    {
      std::lock_guard<std::mutex> g(stat_mtx_);
      const auto per = summary(cmd_periods_, true);
      if (!per.empty()) {o << ",\"cmd_period_ms\":" << per;}
      const auto cm = summary(ctrl_ms_, false);
      if (!cm.empty()) {o << ",\"ctrl_ms\":" << cm;}
      o << ",\"dropped_scans\":" << dropped_scans_ << ",\"conflict\":[";
      for (size_t k = 0; k < conflict_.size(); ++k) {o << (k ? "," : "") << objavoid::jstr(conflict_[k]);}
      o << "]";
    }
    const double t = now_s();
    o << ",\"t\":" << objavoid::jnum(t, 3);
    {
      std::lock_guard<std::mutex> g(stat_mtx_);
      o << ",\"start_gate\":{\"auto\":" << (core_->p.start_auto && gp_ ? "true" : "false") << ",\"status\":" <<
        objavoid::jstr(core_->active() ? "active" : gate_status_) << ",\"last\":" <<
        (gate_json_.empty() ? "null" : gate_json_) << ",\"shift\":" << objavoid::jnum(core_->start_shift) << "}";
    }
    if (core_->active()) {
      if (const auto veh = core_->vehicle(t)) {
        o << ",\"s\":" << objavoid::jnum(veh->s) << ",\"d\":" << objavoid::jnum(veh->d) << ",\"psi\":" <<
          objavoid::jnum(veh->psi);
      }
    }
    o << "}";
    std_msgs::msg::String m;
    m.data = o.str();
    state_pub_->publish(m);
  }

  void publish_viz()
  {
    const auto stamp = now();
    const double t = now_s();
    visualization_msgs::msg::MarkerArray arr;
    nav_msgs::msg::Path pmsg;
    pmsg.header.stamp = stamp;
    pmsg.header.frame_id = "odom";
    {
      std::lock_guard<std::recursive_mutex> g(core_->mutex());
      if (!core_->frame) {return;}
      arr = objavoid::build_markers(*core_, stamp, t);
      if (marker_frame_ != "odom") {
        const auto pose = core_->pose_at(t, 0.3);
        if (!pose) {return;}
        objavoid::to_base(arr, pose->x, pose->y, pose->yaw, marker_frame_);
      } else if (core_->path_) {
        const auto & path = *core_->path_;
        for (size_t k = 0; k < path.s.size(); k += 2) {
          geometry_msgs::msg::PoseStamped ps;
          ps.header = pmsg.header;
          core_->frame->to_odom(path.s[k], path.d[k], ps.pose.position.x, ps.pose.position.y);
          const double ya = path.psi[k] + core_->frame->yaw0;
          ps.pose.orientation.z = std::sin(ya / 2);
          ps.pose.orientation.w = std::cos(ya / 2);
          pmsg.poses.push_back(ps);
        }
      }
    }
    marker_pub_->publish(arr);
    if (marker_frame_ == "odom") {path_pub_->publish(pmsg);}  // Path는 odom 좌표만
  }

  std::unique_ptr<objavoid::ObjAvoidCore> core_;
  std::unique_ptr<objavoid::GroundProjection> gp_;
  std::string command_topic_, yaw_source_, marker_frame_;
  bool auto_activate_ = false, lane_hsv_check_ = true;
  double marker_rate_ = 5.0, lane_rate_ = 5.0, odom_scale_ = 1.0, ctrl_period_ = 0.04;
  rclcpp::CallbackGroup::SharedPtr sensors_, control_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr act_sub_, sign_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detail_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_, status_timer_, conflict_timer_;
  // IMU 방향 적분·추측 항법(센서 콜백 그룹 안에서만 사용)
  bool has_imu_ = false, has_dead_ = false;
  double imu_t_ = 0, imu_yaw_ = 0, imu_w_ = 0, dead_t_ = 0, dead_x_ = 0, dead_y_ = 0;
  std::mutex lane_mtx_;
  sensor_msgs::msg::Image::SharedPtr latest_mask_;
  std::deque<sensor_msgs::msg::CompressedImage::SharedPtr> images_;
  double last_lane_wall_ = 0;
  std::mutex scan_mtx_;
  std::condition_variable scan_cv_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  double scan_recv_wall_ = 0;
  long dropped_scans_ = 0;
  std::atomic<bool> running_{true};
  std::thread worker_;
  std::mutex stat_mtx_;
  std::vector<double> cmd_periods_, ctrl_ms_;
  std::vector<std::string> conflict_;
  double last_ctrl_wall_ = -1, last_marker_wall_ = 0;
  // 자동 시작(작업 스레드에서만 갱신, gate_json_/gate_status_는 stat_mtx_)
  double last_active_wall_ = -1e9, last_gate_wall_ = 0, last_gate_img_ = -1;
  int gate_hits_ = 0;
  // 횡단보도 순서 조건(센서 콜백 그룹·작업 스레드에서 사용)
  std::atomic<bool> cw_seen_{false};
  std::atomic<double> cw_last_cmd_{-1e9};
  std::atomic<bool> cw_reset_{false};
  std::optional<std::pair<double, double>> cw_armed_at_;  // 작업 스레드만
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cw_start_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cw_cmd_sub_;
  std::string gate_json_, gate_status_ = "idle";
  std::atomic<bool> done_sent_{false}, pending_activate_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjAvoidV3Node>();
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 2);
  ex.add_node(node);
  ex.spin();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
