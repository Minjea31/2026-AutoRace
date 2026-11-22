// Qt tuner for lane_deep_lite: live (/lane_fusion/deep_mask + camera for display)
// or bag + per-frame masks (tools/bag_masks.py --masks DIR).
//   lane_deep_lite_tuner [--profile real|sim] [--config YAML] [--bag BAG [--masks DIR]]
//                        [--mask-topic T] [--camera-topic T] [--frame N]
// Sliders re-run the pipeline at once; "Apply" sends every value to the running
// lane_deep_lite_node; "Save YAML" defaults to the package source config.
#include "autorace_lane_deep_lite/deep_lane.hpp"
#include <QApplication>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <map>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

#ifndef DEEP_LITE_SOURCE_DIR
#define DEEP_LITE_SOURCE_DIR ""
#endif

namespace {
double wall() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
QImage qimage(const cv::Mat &bgr) {
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return QImage(rgb.data, rgb.cols, rgb.rows, int(rgb.step), QImage::Format_RGB888).copy();
}
// Installed configs are build copies: save to the package source tree instead.
std::string editablePath(const std::string &path) {
  const std::string marker = "/share/autorace_lane_deep_lite/config/", source = DEEP_LITE_SOURCE_DIR;
  const auto at = path.find(marker);
  if (at == std::string::npos || source.empty() || !std::filesystem::is_directory(source + "/config"))
    return path;
  return source + "/config/" + path.substr(at + marker.size());
}

// Random access to the camera frames of a bag (timestamps indexed once).
class BagFrames {
public:
  void open(const std::string &path) {
    reader_ = std::make_unique<rosbag2_cpp::Reader>();
    reader_->open(path);
    for (const auto &t : reader_->get_all_topics_and_types()) {
      const bool compressed = t.type == "sensor_msgs/msg/CompressedImage";
      if ((compressed || t.type == "sensor_msgs/msg/Image") &&
          t.name.rfind("/camera/color/image_raw", 0) == 0 && (topic_.empty() || compressed))
        topic_ = t.name, raw_ = !compressed;
    }
    if (topic_.empty())
      throw std::runtime_error("no /camera/color/image_raw[/compressed] in " + path);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {topic_};
    reader_->set_filter(filter);
    while (reader_->has_next())
      stamps_.push_back(reader_->read_next()->time_stamp);
    if (stamps_.empty())
      throw std::runtime_error("empty camera topic " + topic_);
  }
  size_t size() const { return stamps_.size(); }
  double time(size_t i) const { return (stamps_[i] - stamps_[0]) * 1e-9; }
  const std::string &topic() const { return topic_; }
  cv::Mat image(size_t i) {
    if (i != next_)
      reader_->seek(stamps_[i]);
    const auto m = reader_->read_next();
    next_ = i + 1;
    rclcpp::SerializedMessage s(*m->serialized_data);
    if (raw_) {
      sensor_msgs::msg::Image msg;
      rclcpp::Serialization<sensor_msgs::msg::Image>().deserialize_message(&s, &msg);
      return cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    sensor_msgs::msg::CompressedImage msg;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage>().deserialize_message(&s, &msg);
    return cv::imdecode(msg.data, cv::IMREAD_COLOR);
  }

private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::vector<int64_t> stamps_;
  std::string topic_;
  bool raw_ = false;
  size_t next_ = ~size_t(0);
};

struct Live {
  std::mutex mutex;
  sensor_msgs::msg::Image::ConstSharedPtr mask;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr camera;
  size_t masks = 0;
};
} // namespace

class Window : public QMainWindow {
public:
  Window(const rclcpp::Node::SharedPtr &node, Live &live, deep_lite::Config config,
         std::string config_path, std::string bag, std::string masks, size_t start)
      : live_(live), config_(config), config_path_(config_path), mask_dir_(masks) {
    client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, "lane_deep_lite_node");
    pipeline_.configure(config_);
    resize(1500, 900);
    auto central = new QWidget;
    auto root = new QHBoxLayout(central);
    setCentralWidget(central);

    // Left: pictures, status, playback.
    auto left = new QVBoxLayout;
    auto pictures = new QHBoxLayout;
    camera_view_ = new QLabel;
    bev_view_ = new QLabel;
    for (auto *l : {camera_view_, bev_view_}) {
      l->setMinimumSize(480, 360);
      l->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored); // never grow to the pixmap
      l->setAlignment(Qt::AlignCenter);
      l->setStyleSheet("background:#202020");
      pictures->addWidget(l, 1);
    }
    left->addLayout(pictures, 1);
    status_ = new QLabel;
    status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    status_->setWordWrap(true);
    status_->setStyleSheet("font-family:monospace");
    left->addWidget(status_);
    auto legend = new QLabel(
        "BEV: 노랑=딥 노란 차선, 회색=딥 흰 점선, 보라=후보 체인, 초록=왼쪽, 하늘=오른쪽, "
        "빨강=중앙 경로·목표점, 점선=유지(HELD). 원본: 딥 마스크를 색으로 덮어 표시");
    legend->setWordWrap(true);
    left->addWidget(legend);
    if (!bag.empty()) {
      auto row = new QHBoxLayout;
      auto play = new QPushButton("▶/❚❚");
      auto prev = new QPushButton("◀ 이전");
      auto next = new QPushButton("다음 ▶");
      seek_ = new QSlider(Qt::Horizontal);
      frame_label_ = new QLabel;
      row->addWidget(play);
      row->addWidget(prev);
      row->addWidget(next);
      row->addWidget(seek_, 1);
      row->addWidget(frame_label_);
      left->addLayout(row);
      connect(play, &QPushButton::clicked, this, [this] { playing_ = !playing_; });
      connect(prev, &QPushButton::clicked, this, [this] { showFrame(index_ ? index_ - 1 : 0, true); });
      connect(next, &QPushButton::clicked, this, [this] { showFrame(index_ + 1, false); });
      connect(seek_, &QSlider::sliderReleased, this, [this] { showFrame(size_t(seek_->value()), true); });
    }
    root->addLayout(left, 3);

    // Right: buttons and parameters.
    auto right = new QVBoxLayout;
    auto buttons = new QHBoxLayout;
    auto load = new QPushButton("Load YAML");
    auto save = new QPushButton("Save YAML");
    auto apply = new QPushButton("Apply to lane_deep_lite_node");
    buttons->addWidget(load);
    buttons->addWidget(save);
    right->addLayout(buttons);
    right->addWidget(apply);
    auto form = new QWidget;
    auto params = new QVBoxLayout(form);
    std::map<std::string, QVBoxLayout *> groups;
    for (const auto &p : deep_lite::specs()) {
      if (!groups.count(p.group)) {
        auto box = new QGroupBox(QString::fromUtf8(p.group));
        groups[p.group] = new QVBoxLayout(box);
        params->addWidget(box);
      }
      auto label = new QLabel(QString::fromUtf8(p.name));
      label->setToolTip(QString::fromUtf8(p.help));
      auto help = new QLabel(QString::fromUtf8(p.help));
      help->setWordWrap(true);
      help->setStyleSheet("color:gray; font-size:11px");
      auto line = new QHBoxLayout;
      auto slider = new QSlider(Qt::Horizontal);
      slider->setRange(0, int(std::round((p.max - p.min) / p.step)));
      auto spin = new QDoubleSpinBox;
      spin->setRange(p.min, p.max);
      spin->setSingleStep(p.step);
      spin->setDecimals(p.step >= 1 ? 0 : 3);
      spin->setKeyboardTracking(false);
      spin->setMinimumWidth(80);
      line->addWidget(label);
      line->addWidget(slider, 1);
      line->addWidget(spin);
      groups[p.group]->addLayout(line);
      groups[p.group]->addWidget(help);
      spins_[p.name] = spin;
      sliders_[p.name] = slider;
      const std::string key = p.name;
      const double mn = p.min, st = p.step;
      connect(slider, &QSlider::valueChanged, this, [spin, mn, st](int i) { spin->setValue(mn + i * st); });
      connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this, key, slider, mn, st](double v) {
        QSignalBlocker block(slider);
        slider->setValue(int(std::round((v - mn) / st)));
        auto next = config_;
        next.values[key] = v;
        try {
          next.validate();
          config_ = next;
          pipeline_.configure(config_);
          note_ = "";
          rerun();
        } catch (const std::exception &e) {
          note_ = std::string("값 거부: ") + e.what();
        }
      });
    }
    params->addStretch();
    auto scroll = new QScrollArea;
    scroll->setWidget(form);
    scroll->setWidgetResizable(true);
    right->addWidget(scroll, 1);
    root->addLayout(right, 2);
    syncWidgets();

    connect(load, &QPushButton::clicked, this, [this] {
      const auto path = QFileDialog::getOpenFileName(this, "Load YAML", QString::fromStdString(editablePath(config_path_)),
                                                     "YAML (*.yaml *.yml)");
      if (path.isEmpty())
        return;
      try {
        config_ = deep_lite::Config::load(path.toStdString());
        config_path_ = path.toStdString();
        pipeline_.configure(config_);
        syncWidgets();
        rerun();
      } catch (const std::exception &e) {
        QMessageBox::warning(this, "Load YAML", e.what());
      }
    });
    connect(save, &QPushButton::clicked, this, [this] {
      const auto path = QFileDialog::getSaveFileName(this, "Save YAML", QString::fromStdString(editablePath(config_path_)),
                                                     "YAML (*.yaml *.yml)");
      if (path.isEmpty())
        return;
      try {
        config_.save(path.toStdString());
        config_path_ = path.toStdString();
        note_ = "저장: " + config_path_ + " (launch에 반영하려면 다시 빌드)";
        updateTitle();
      } catch (const std::exception &e) {
        QMessageBox::warning(this, "Save YAML", e.what());
      }
    });
    connect(apply, &QPushButton::clicked, this, [this] {
      if (!client_->service_is_ready()) {
        note_ = "lane_deep_lite_node가 보이지 않음 (ROS_DOMAIN_ID 확인)";
        return;
      }
      std::vector<rclcpp::Parameter> values;
      for (const auto &s : deep_lite::specs())
        values.emplace_back(s.name, config_[s.name]);
      pending_ = client_->set_parameters_atomically(values);
      note_ = "적용 중...";
    });

    if (!bag.empty()) {
      bag_.open(bag);
      if (mask_dir_.empty()) {
        const std::filesystem::path p(bag);
        mask_dir_ = (p.parent_path() / "deep_masks" / p.filename()).string();
      }
      if (!std::filesystem::is_directory(mask_dir_))
        throw std::runtime_error("deep mask folder not found: " + mask_dir_ +
                                 " (make it with tools/bag_masks.py --masks, or pass --masks DIR)");
      seek_->setRange(0, int(bag_.size()) - 1);
      showFrame(std::min(start, bag_.size() - 1), true);
    }
    updateTitle();
    auto timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { tick(); });
    timer->start(15);
  }

private:
  void updateTitle() {
    setWindowTitle(QString::fromStdString("Lane Deep Lite Tuner — " + editablePath(config_path_)));
  }
  void syncWidgets() {
    for (auto &[key, spin] : spins_) {
      QSignalBlocker a(spin), b(sliders_.at(key));
      spin->setValue(config_[key]);
      const auto &s = *std::find_if(deep_lite::specs().begin(), deep_lite::specs().end(),
                                    [&](const deep_lite::Spec &x) { return key == x.name; });
      sliders_.at(key)->setValue(int(std::round((config_[key] - s.min) / s.step)));
    }
  }
  // Bag: load frame i (reset tracking when not consecutive) and process it.
  void showFrame(size_t i, bool jump) {
    if (!bag_.size())
      return;
    i = std::min(i, bag_.size() - 1);
    if (jump || i != index_ + 1)
      pipeline_.reset();
    index_ = i;
    camera_ = bag_.image(i);
    char name[32];
    std::snprintf(name, sizeof name, "/%06zu.png", i);
    mask_ = cv::imread(mask_dir_ + name, cv::IMREAD_UNCHANGED);
    time_ = bag_.time(i);
    due_ = wall() + (i + 1 < bag_.size() ? bag_.time(i + 1) - bag_.time(i) : .05);
    {
      QSignalBlocker block(seek_);
      seek_->setValue(int(i));
    }
    frame_label_->setText(QString("%1 / %2").arg(i).arg(bag_.size() - 1));
    process();
  }
  // Re-run the current frame after a parameter change.
  void rerun() {
    pipeline_.reset();
    if (!mask_.empty())
      process();
  }
  void process() {
    if (mask_.empty()) {
      note_ = "이 프레임의 딥 마스크 없음";
      return;
    }
    result_ = pipeline_.process(mask_, time_, true);
    render();
  }
  void tick() {
    if (pending_.valid() && pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto r = pending_.get();
        note_ = r.successful ? "lane_deep_lite_node에 적용됨" : "적용 실패: " + r.reason;
      } catch (const std::exception &e) {
        note_ = std::string("적용 실패: ") + e.what();
      }
      pending_ = {};
      render();
    }
    if (bag_.size()) {
      if (playing_ && wall() >= due_ && index_ + 1 < bag_.size())
        showFrame(index_ + 1, false);
      return;
    }
    sensor_msgs::msg::Image::ConstSharedPtr mask;
    sensor_msgs::msg::CompressedImage::ConstSharedPtr camera;
    {
      std::lock_guard<std::mutex> lock(live_.mutex);
      mask = live_.mask;
      live_.mask.reset();
      camera = live_.camera;
    }
    if (!mask) {
      if (wall() - last_mask_ > 1 && wall() - last_wait_ > 1) {
        last_wait_ = wall();
        status_->setText("딥 마스크 기다리는 중 (/lane_fusion/deep_mask). 차에서 lane_deep_lite.launch.py "
                         "또는 deep_mask.launch.py가 떠 있는지, ROS_DOMAIN_ID가 같은지 확인");
      }
      return;
    }
    if (mask->encoding != "mono8")
      return;
    mask_ = cv::Mat(int(mask->height), int(mask->width), CV_8U, const_cast<uint8_t *>(mask->data.data()),
                    mask->step).clone();
    time_ = rclcpp::Time(mask->header.stamp).seconds();
    if (camera && camera != shown_camera_) {
      camera_ = cv::imdecode(camera->data, cv::IMREAD_COLOR);
      shown_camera_ = camera;
    }
    const double now = wall();
    if (last_mask_ > 0)
      mask_hz_ = .8 * mask_hz_ + .2 / std::max(1e-3, now - last_mask_);
    last_mask_ = now;
    process();
  }
  void render() {
    const auto &r = result_;
    const int w = config_["width"], h = config_["height"];
    cv::Mat cam = camera_.empty() ? cv::Mat(h, w, CV_8UC3, cv::Scalar(40, 40, 40)) : camera_.clone();
    cv::resize(cam, cam, {w, h});
    if (!mask_.empty()) {
      cv::Mat m;
      cv::resize(mask_, m, {w, h}, 0, 0, cv::INTER_NEAREST);
      cam.setTo(cv::Scalar(0, 220, 255), m == 1);
      cam.setTo(cv::Scalar(255, 255, 255), m == 2);
    }
    // BEV trapezoid in the camera picture.
    const std::vector<cv::Point> poly = {
        {cvRound(config_["bev_top_left"] * (w - 1)), cvRound(config_["bev_top_y"] * (h - 1))},
        {cvRound(config_["bev_top_right"] * (w - 1)), cvRound(config_["bev_top_y"] * (h - 1))},
        {cvRound(config_["bev_bottom_right"] * (w - 1)), cvRound(config_["bev_bottom_y"] * (h - 1))},
        {cvRound(config_["bev_bottom_left"] * (w - 1)), cvRound(config_["bev_bottom_y"] * (h - 1))}};
    cv::polylines(cam, poly, true, cv::Scalar(255, 120, 0), 2);
    auto put = [](QLabel *label, const cv::Mat &m) {
      if (m.empty())
        return;
      label->setPixmap(QPixmap::fromImage(qimage(m)).scaled(label->size(), Qt::KeepAspectRatio,
                                                             Qt::SmoothTransformation));
    };
    put(camera_view_, cam);
    put(bev_view_, r.view);
    char text[512];
    std::snprintf(text, sizeof text,
                  "상태 %-10s 왼쪽 %-8s 오른쪽 %-8s 후보 체인 %zu | 처리 %.2f ms%s\n"
                  "목표점: 횡오차 %+.3f m  heading %+.3f rad  곡률 %+.2f 1/m  거리 %.2f m  신뢰도 %.2f\n"
                  "가까운 점: 횡오차 %+.3f m  heading %+.3f rad  거리 %.2f m | 차로 폭 학습값 %.0f px",
                  r.state.c_str(), deep_lite::name(r.left.state), deep_lite::name(r.right.state),
                  r.chains.size(), r.processing_ms,
                  bag_.size() ? "" : (" | 마스크 " + std::to_string(int(std::round(mask_hz_))) + " Hz").c_str(),
                  r.lateral_m, r.heading, r.curvature, r.lookahead_m, r.confidence, r.near_lateral_m,
                  r.near_heading, r.near_m, r.lane_width_px);
    QString s = QString::fromUtf8(text);
    for (const auto &n : r.notes)
      s += "\n" + QString::fromStdString(n);
    if (!note_.empty())
      s += "\n" + QString::fromStdString(note_);
    status_->setText(s);
  }

  Live &live_;
  deep_lite::Config config_;
  deep_lite::Pipeline pipeline_;
  deep_lite::Result result_;
  std::string config_path_, mask_dir_, note_;
  BagFrames bag_;
  cv::Mat camera_, mask_;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr shown_camera_;
  size_t index_ = 0;
  double time_ = 0, due_ = 0, last_mask_ = 0, last_wait_ = 0, mask_hz_ = 0;
  bool playing_ = false;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  std::shared_future<rcl_interfaces::msg::SetParametersResult> pending_;
  QLabel *camera_view_, *bev_view_, *status_, *frame_label_ = nullptr;
  QSlider *seek_ = nullptr;
  std::map<std::string, QDoubleSpinBox *> spins_;
  std::map<std::string, QSlider *> sliders_;
};

int main(int argc, char **argv) {
  std::string profile = "real", config, bag, masks, mask_topic = "/lane_fusion/deep_mask",
              camera_topic = "/camera/color/image_raw/compressed", screenshot;
  size_t start = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto value = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--profile") profile = value();
    else if (a == "--config") config = value();
    else if (a == "--bag") bag = value();
    else if (a == "--masks") masks = value();
    else if (a == "--mask-topic") mask_topic = value();
    else if (a == "--camera-topic") camera_topic = value();
    else if (a == "--frame") start = std::stoul(value());
    else if (a == "--screenshot") screenshot = value(); // save the window after 3 s and quit
    else if (a == "-h" || a == "--help") {
      std::cout << "lane_deep_lite_tuner [--profile real|sim] [--config YAML] [--bag BAG [--masks DIR]]\n"
                   "                     [--frame N] [--mask-topic T] [--camera-topic T]\n"
                   "  live (no --bag): /lane_fusion/deep_mask + camera picture, Apply -> lane_deep_lite_node\n";
      return 0;
    }
  }
  if (config.empty()) {
    const std::string source = std::string(DEEP_LITE_SOURCE_DIR) + "/config/" + profile + "/lane.yaml";
    config = std::filesystem::exists(source) ? source : "";
    if (config.empty())
      try {
        config = ament_index_cpp::get_package_share_directory("autorace_lane_deep_lite") + "/config/" +
                 profile + "/lane.yaml";
      } catch (...) {
      }
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lane_deep_lite_tuner");
  Live live;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub;
  if (bag.empty()) {
    mask_sub = node->create_subscription<sensor_msgs::msg::Image>(
        mask_topic, rclcpp::SensorDataQoS().keep_last(1), [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(live.mutex);
          live.mask = m;
          ++live.masks;
        });
    camera_sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        camera_topic, rclcpp::SensorDataQoS().keep_last(1),
        [&](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(live.mutex);
          live.camera = m;
        });
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros([&] { executor.spin(); });
  QApplication app(argc, argv);
  int status = 0;
  try {
    deep_lite::Config c = config.empty() ? deep_lite::Config() : deep_lite::Config::load(config);
    Window window(node, live, c, config, bag, masks, start);
    window.show();
    if (!screenshot.empty())
      QTimer::singleShot(3000, [&] {
        window.grab().save(QString::fromStdString(screenshot));
        app.quit();
      });
    status = app.exec();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    status = 1;
  }
  executor.cancel();
  ros.join();
  rclcpp::shutdown();
  return status;
}
