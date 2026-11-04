// Qt tuner for lane_lite. Same layout and workflow as autorace_lane_single's
// lane_tuner; the ROI/BEV drag view and the rosbag reader are reused as-is.
// Perception only: no control gains and no drive output in this window.
#include "autorace_lane_lite/lane.hpp"
#include "autorace_lane_single/bag.hpp"
#include "autorace_lane_single/roi_view.hpp"
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSplitter>
#include <QTimer>
#include <atomic>
#include <filesystem>
#include <condition_variable>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace {
double wall() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
QImage qimage(const cv::Mat &mat) {
  cv::Mat rgb;
  cv::cvtColor(mat, rgb, mat.channels() == 1 ? cv::COLOR_GRAY2RGB : cv::COLOR_BGR2RGB);
  return QImage(rgb.data, rgb.cols, rgb.rows, int(rgb.step), QImage::Format_RGB888).copy();
}
// RoiView edits the legacy Config type; only ROI/BEV keys are exchanged.
const char *kGeometry[] = {"roi_left", "roi_top", "roi_right", "roi_bottom",
                           "bev_top_y", "bev_bottom_y", "bev_top_left", "bev_top_right",
                           "bev_bottom_left", "bev_bottom_right"};
lane_single::Config toView(const lane_lite::Config &c) {
  lane_single::Config v;
  for (const char *k : kGeometry)
    v.values[k] = c[k];
  return v;
}
const lane_lite::Spec &spec(const std::string &key) {
  for (const auto &s : lane_lite::specs())
    if (key == s.name)
      return s;
  throw std::out_of_range(key);
}

enum Image { Overlay, Colour, Feature, Used, Compare, Bev, BevMask, Search, PathView, Count };
struct Display {
  std::array<QImage, Count> images;
  lane_lite::Result result;
  size_t frame = 0, count = 0, serial = 0;
  double time = 0, ready = 0, received = 0, total_ms = 0;
};
#ifndef LANE_LITE_SOURCE_DIR
#define LANE_LITE_SOURCE_DIR ""
#endif
// Installed configs are build copies; Save goes back to the package source
// tree (config/sim, config/real) so tuning survives the next build.
std::string editablePath(const std::string &path) {
  const std::string marker = "/share/autorace_lane_lite/config/", source = LANE_LITE_SOURCE_DIR;
  const auto at = path.find(marker);
  if (at == std::string::npos || source.empty() || !std::filesystem::is_directory(source + "/config"))
    return path;
  return source + "/config/" + path.substr(at + marker.size());
}

struct Shared {
  std::mutex mutex;
  std::condition_variable cv;
  std::atomic<bool> stop{false};
  lane_lite::Config config;
  size_t revision = 0, source = 0, incoming = 0;
  std::string path, topic, error, status = "Open a bag or press Live";
  std::string config_path; // YAML loaded / saved last (GUI thread only)
  bool live = false, playing = false;
  double rate = 1, received = 0;
  int64_t seek = -1;
  int step = 0;
  sensor_msgs::msg::Image::ConstSharedPtr raw;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed;
  std::shared_ptr<Display> display;
};

void worker(Shared &s) {
  lane_lite::Pipeline pipeline;
  lane_single::Bag bag;
  lane_single::Frame frame;
  size_t source = ~size_t(0), revision = ~size_t(0), incoming = 0, index = 0, next = 0,
         serial = 0;
  bool loaded = false;
  double due = 0;
  while (!s.stop) {
    try {
      std::string path, topic;
      bool live, playing;
      double rate, received;
      int64_t seek;
      int step;
      size_t new_source, new_revision, new_incoming;
      lane_lite::Config config;
      sensor_msgs::msg::Image::ConstSharedPtr raw;
      sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed;
      {
        std::unique_lock<std::mutex> lock(s.mutex);
        s.cv.wait_for(lock, std::chrono::milliseconds(4));
        if (s.stop)
          break;
        path = s.path, topic = s.topic, live = s.live, playing = s.playing, rate = s.rate;
        new_source = s.source, new_revision = s.revision, config = s.config;
        seek = s.seek, s.seek = -1, step = s.step, s.step = 0;
        raw = s.raw, compressed = s.compressed, new_incoming = s.incoming;
        received = s.received;
      }
      const bool changed = revision != new_revision;
      if (changed) {
        pipeline.configure(config); // also resets tracking
        revision = new_revision;
      }
      if (source != new_source) {
        source = new_source;
        loaded = false;
        frame = {};
        pipeline.reset();
        index = next = incoming = 0;
        if (!live && !path.empty()) {
          {
            std::lock_guard<std::mutex> lock(s.mutex);
            s.status = "Indexing bag...";
            s.display.reset();
          }
          bag.open(path, topic, &s.stop);
          loaded = true;
          seek = std::max<int64_t>(seek, 0);
          std::lock_guard<std::mutex> lock(s.mutex);
          s.status = "Bag: " + bag.topic() + " | " + std::to_string(bag.size()) + " frames";
        }
      }
      const double begin = wall();
      bool process = false;
      if (live) {
        if (new_incoming != incoming && (raw || compressed)) {
          frame.image = raw ? lane_single::decode(*raw) : lane_single::decode(*compressed);
          frame.time = received;
          incoming = index = new_incoming;
          process = true;
        } else
          process = changed && !frame.image.empty();
      } else if (loaded) {
        size_t wanted = index;
        bool jump = false;
        if (seek >= 0) {
          wanted = std::min<size_t>(seek, bag.size() - 1);
          jump = wanted != index + 1;
          process = true;
        } else if (step) {
          wanted = size_t(std::clamp<int64_t>(int64_t(index) + step, 0, bag.size() - 1));
          jump = step != 1;
          process = true;
        } else if (playing && wall() >= due && next < bag.size()) {
          wanted = next;
          process = true;
        } else
          process = changed && !frame.image.empty();
        if (process && (seek >= 0 || step || wanted == next)) {
          if (jump) // tracking is only valid for consecutive frames
            pipeline.reset();
          index = wanted;
          frame = bag.frame(index);
          next = index + 1;
          due = wall() + (next < bag.size() ? std::max(.001, bag.time(next) - bag.time(index)) : .05) / rate;
          received = begin;
          if (next == bag.size()) {
            std::lock_guard<std::mutex> lock(s.mutex);
            s.playing = false;
          }
        } else if (process) // re-run the same frame after a parameter edit
          pipeline.reset();
        if (!playing && !process)
          due = wall();
      }
      if (!process || frame.image.empty())
        continue;
      auto d = std::make_shared<Display>();
      d->result = pipeline.process(frame.image, frame.time);
      const auto &r = d->result;
      cv::Mat compare;
      cv::hconcat(r.color_mask, r.feature_mask, compare);
      cv::putText(compare, "COLOR (HSV)", {12, 26}, cv::FONT_HERSHEY_SIMPLEX, .7, 150, 2);
      cv::putText(compare, "FEATURE (top-hat, thin, area)", {compare.cols / 2 + 12, 26},
                  cv::FONT_HERSHEY_SIMPLEX, .7, 150, 2);
      d->images = {qimage(r.overlay), qimage(r.color_mask), qimage(r.feature_mask),
                   qimage(r.line_mask), qimage(compare), qimage(r.bev),
                   qimage(r.bev_mask), qimage(r.search_view), qimage(r.path_view)};
      d->frame = index;
      d->count = live ? 0 : bag.size();
      d->time = live ? 0 : frame.time;
      d->ready = wall();
      d->received = received;
      d->total_ms = (d->ready - begin) * 1000;
      d->serial = ++serial;
      std::lock_guard<std::mutex> lock(s.mutex);
      if (source == s.source && revision == s.revision) {
        s.display = d;
        s.error.clear();
      }
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(s.mutex);
      s.error = e.what();
      s.playing = false;
    }
  }
}

class Window : public QMainWindow {
public:
  void updateTitle() {
    setWindowTitle(QString::fromStdString(
        "AutoRace Lane Lite Tuner — " +
        (s.config_path.empty() ? std::string("코드 기본값 (YAML 없음)") : editablePath(s.config_path))));
  }
  Window(Shared &shared, const rclcpp::Node::SharedPtr &node) : s(shared) {
    client = std::make_shared<rclcpp::AsyncParametersClient>(node, "lane_lite_node");
    updateTitle();
    resize(1450, 900);
    auto central = new QWidget;
    auto outer = new QVBoxLayout(central);
    setCentralWidget(central);
    auto bar = new QHBoxLayout;
    outer->addLayout(bar);
    auto open = new QPushButton("Open Bag");
    topic = new QLineEdit;
    topic->setPlaceholderText("Bag image topic (empty = automatic)");
    auto live = new QPushButton("Live");
    auto load = new QPushButton("Load YAML");
    auto save = new QPushButton("Save YAML");
    auto apply = new QPushButton("Apply to running lane_lite_node");
    apply->setToolTip("현재 인식 설정 전체를 실행 중인 /lane_lite_node에 원자적으로 적용");
    for (QWidget *widget : std::initializer_list<QWidget *>{open, topic, live, load, save, apply})
      bar->addWidget(widget);

    auto split = new QSplitter;
    outer->addWidget(split, 1);
    auto views = new QWidget;
    auto grid = new QGridLayout(views);
    split->addWidget(views);
    const char *titles[] = {"Original (ROI 노랑 / BEV 파랑)", "Binary", "BEV", "경계·중앙 경로·목표점"};
    for (int i = 0; i < 4; ++i) {
      auto box = new QGroupBox(titles[i]);
      auto layout = new QVBoxLayout(box);
      if (i == 0) {
        original = new lane_single::RoiView;
        original->setConfig(toView(s.config));
        auto mode = new QComboBox;
        mode->addItems({"Edit ROI (yellow)", "Edit BEV (blue corners)"});
        connect(mode, qOverload<int>(&QComboBox::currentIndexChanged), this,
                [this](int m) { original->setBevEditing(m == 1); });
        original->onRoiChanged = [this](double l, double t, double r, double b) {
          auto c = current();
          c.values["roi_left"] = l, c.values["roi_top"] = t;
          c.values["roi_right"] = r, c.values["roi_bottom"] = b;
          applyConfig(c);
        };
        original->onBevChanged = [this](const lane_single::Config &v) {
          auto c = current();
          for (const char *k : kGeometry)
            c.values[k] = v[k];
          applyConfig(c);
        };
        layout->addWidget(mode);
        layout->addWidget(original);
      } else {
        if (i == 1) {
          binary_mode = new QComboBox;
          binary_mode->addItems({"색상 후보 (HSV, ROI∩BEV)", "최종 특징 (Top-hat·두께·면적)",
                                 "차선으로 사용된 픽셀", "비교: 색상 | 최종 특징"});
          binary_mode->setCurrentIndex(3);
          layout->addWidget(binary_mode);
          connect(binary_mode, qOverload<int>(&QComboBox::currentIndexChanged), this,
                  [this](int) { refresh(); });
        }
        if (i == 2) {
          bev_mode = new QComboBox;
          bev_mode->addItems({"BEV 원본", "BEV 특징 마스크", "Sliding Window 탐색"});
          bev_mode->setCurrentIndex(2);
          bev_mode->setToolTip("탐색: 초록/청록=왼쪽/오른쪽 관측 윈도우, 회색=미관측, "
                               "원=시작점, 자홍 점선=이전 추적 경계");
          layout->addWidget(bev_mode);
          connect(bev_mode, qOverload<int>(&QComboBox::currentIndexChanged), this,
                  [this](int) { refresh(); });
        }
        labels[i] = new QLabel("Waiting for image");
        labels[i]->setAlignment(Qt::AlignCenter);
        labels[i]->setMinimumSize(240, 180);
        labels[i]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        layout->addWidget(labels[i]);
      }
      grid->addWidget(box, i / 2, i % 2);
    }
    details = new QPlainTextEdit;
    details->setReadOnly(true);
    details->setMaximumHeight(170);
    details->setLineWrapMode(QPlainTextEdit::NoWrap);
    grid->addWidget(details, 2, 0, 1, 2);

    // Parameter panel: one group per processing stage, slider + exact value.
    auto panel = new QWidget;
    auto panel_layout = new QVBoxLayout(panel);
    panel_layout->setContentsMargins(0, 0, 0, 0);
    auto jump = new QComboBox;
    auto scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setMinimumWidth(370);
    panel_layout->addWidget(jump);
    panel_layout->addWidget(scroll, 1);
    auto params = new QWidget;
    auto grouped = new QVBoxLayout(params);
    std::map<std::string, QVBoxLayout *> layouts;
    std::vector<QGroupBox *> boxes;
    for (const char *group : lane_lite::groups()) {
      const QString title = QString("%1. %2").arg(boxes.size() + 1, 2, 10, QChar('0'))
                                .arg(QString::fromUtf8(group));
      auto box = new QGroupBox(title);
      boxes.push_back(box);
      jump->addItem(title);
      layouts[group] = new QVBoxLayout(box);
      grouped->addWidget(box);
    }
    grouped->addStretch();
    scroll->setWidget(params);
    connect(jump, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [scroll, boxes](int i) { scroll->ensureWidgetVisible(boxes.at(i)); });
    split->addWidget(panel);
    split->setStretchFactor(0, 4);
    split->setStretchFactor(1, 1);
    for (const auto &p : lane_lite::specs()) {
      auto row = new QWidget;
      auto stack = new QVBoxLayout(row);
      stack->setContentsMargins(2, 3, 2, 3);
      auto text = new QLabel(QString(p.name) + " — " + QString::fromUtf8(p.help));
      text->setWordWrap(true);
      stack->addWidget(text);
      auto line = new QHBoxLayout;
      auto slider = new QSlider(Qt::Horizontal);
      slider->setRange(0, qRound((p.max - p.min) / p.step));
      auto spin = new QDoubleSpinBox;
      spin->setObjectName(p.name);
      spin->setRange(p.min, p.max);
      spin->setSingleStep(p.step);
      spin->setDecimals(p.step == 1 ? 0 : 3);
      spin->setKeyboardTracking(false);
      spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
      spin->setMinimumWidth(80);
      line->addWidget(slider, 1);
      line->addWidget(spin);
      stack->addLayout(line);
      layouts.at(p.group)->addWidget(row);
      spins[p.name] = spin;
      sliders[p.name] = slider;
      connect(slider, &QSlider::valueChanged, this, [spin, p](int i) {
        spin->setValue(std::clamp(p.min + i * p.step, p.min, p.max));
      });
      connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
              [this, key = std::string(p.name)](double value) {
                auto c = current();
                c.values[key] = value;
                applyConfig(c);
              });
    }
    syncWidgets(s.config);

    outer->addWidget(new QLabel(
        "<span style='color:#e02020'>빨강: 관측 경계</span> · "
        "<span style='color:#c050c0'>자홍 점선: 유지(HELD) 경계</span> · "
        "<span style='color:#e6a000'>주황 점선: 폭으로 추정한 반대 경계 / 탈락 후보</span> · "
        "<span style='color:#2060e0'>파랑: 중앙 경로(실선=양쪽 관측)</span> · "
        "<span style='color:#20b020'>초록 점: 목표점</span>"));
    auto transport = new QHBoxLayout;
    outer->addLayout(transport);
    play = new QPushButton("Play");
    auto back = new QPushButton("Previous frame");
    auto forward = new QPushButton("Next frame");
    auto speed = new QDoubleSpinBox;
    speed->setRange(.1, 4);
    speed->setSingleStep(.1);
    speed->setValue(1);
    speed->setSuffix(" ×");
    seek_bar = new QSlider(Qt::Horizontal);
    for (QWidget *widget : std::initializer_list<QWidget *>{play, back, forward, speed})
      transport->addWidget(widget);
    transport->addWidget(seek_bar, 1);
    status = new QLabel;
    status->setWordWrap(true);
    outer->addWidget(status);

    connect(open, &QPushButton::clicked, this, [this] {
      const auto path = QFileDialog::getExistingDirectory(this, "Select ROS 2 bag directory");
      if (!path.isEmpty())
        openBag(path.toStdString());
    });
    connect(live, &QPushButton::clicked, this, [this] {
      std::lock_guard<std::mutex> lock(s.mutex);
      s.live = true, s.path.clear(), s.playing = false, ++s.source;
      s.status = "Live ROS camera | preview only";
      s.cv.notify_one();
    });
    connect(play, &QPushButton::clicked, this, [this] {
      std::lock_guard<std::mutex> lock(s.mutex);
      s.playing = !s.playing;
      s.cv.notify_one();
    });
    connect(back, &QPushButton::clicked, this, [this] { stepBy(-1); });
    connect(forward, &QPushButton::clicked, this, [this] { stepBy(1); });
    connect(speed, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double v) {
      std::lock_guard<std::mutex> lock(s.mutex);
      s.rate = v;
    });
    connect(seek_bar, &QSlider::sliderReleased, this, [this] { seek(seek_bar->value()); });
    connect(load, &QPushButton::clicked, this, [this] {
      const auto path = QFileDialog::getOpenFileName(this, "Load YAML", {}, "YAML (*.yaml *.yml)");
      if (path.isEmpty())
        return;
      try {
        applyConfig(lane_lite::Config::load(path.toStdString()));
        s.config_path = path.toStdString();
        updateTitle();
      } catch (const std::exception &e) {
        QMessageBox::warning(this, "YAML", e.what());
      }
    });
    connect(save, &QPushButton::clicked, this, [this] {
      const auto suggested = s.config_path.empty() ? std::string("lane_lite.yaml") : editablePath(s.config_path);
      const auto path = QFileDialog::getSaveFileName(this, "Save YAML", QString::fromStdString(suggested),
                                                     "YAML (*.yaml)");
      if (path.isEmpty())
        return;
      try {
        current().save(path.toStdString());
        s.config_path = path.toStdString();
        updateTitle();
      } catch (const std::exception &e) {
        QMessageBox::warning(this, "YAML", e.what());
      }
    });
    connect(apply, &QPushButton::clicked, this, [this] {
      if (pending.valid()) {
        note = "Previous request is pending";
        return;
      }
      if (!client->service_is_ready()) {
        note = "lane_lite_node parameter service unavailable";
        return;
      }
      std::vector<rclcpp::Parameter> parameters;
      for (const auto &[k, v] : current().values)
        parameters.emplace_back(k, v);
      pending = client->set_parameters_atomically(parameters);
      note = "Applying...";
    });
    auto timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { render(); });
    timer->start(16);
  }
  void openBag(const std::string &path, int64_t frame = 0) {
    std::lock_guard<std::mutex> lock(s.mutex);
    s.path = path, s.topic = topic->text().toStdString(), s.live = false;
    s.playing = false, s.seek = frame, ++s.source;
    s.cv.notify_one();
  }

private:
  lane_lite::Config current() {
    std::lock_guard<std::mutex> lock(s.mutex);
    return s.config;
  }
  void stepBy(int delta) {
    std::lock_guard<std::mutex> lock(s.mutex);
    if (s.display && s.display->count)
      s.seek = std::clamp<int64_t>(int64_t(s.display->frame) + delta, 0, s.display->count - 1);
    s.playing = false;
    s.cv.notify_one();
  }
  void seek(int64_t frame) {
    std::lock_guard<std::mutex> lock(s.mutex);
    s.seek = frame, s.playing = false;
    s.cv.notify_one();
  }
  // Invalid edits (e.g. an inverted ROI) are rejected; widgets show the
  // configuration actually in use.
  void applyConfig(const lane_lite::Config &requested) {
    lane_lite::Config effective;
    {
      std::lock_guard<std::mutex> lock(s.mutex);
      try {
        requested.validate();
        if (requested.values != s.config.values) {
          s.config = requested;
          ++s.revision;
        }
        s.error.clear();
      } catch (const std::exception &e) {
        s.error = e.what();
      }
      effective = s.config;
      s.cv.notify_one();
    }
    syncWidgets(effective);
  }
  void syncWidgets(const lane_lite::Config &c) {
    for (auto &[key, spin] : spins) {
      QSignalBlocker a(spin), b(sliders.at(key));
      spin->setValue(c[key]);
      sliders.at(key)->setValue(qRound((c[key] - spec(key).min) / spec(key).step));
    }
    original->setConfig(toView(c));
  }
  void showImage(QLabel *label, const QImage &image) {
    label->setPixmap(QPixmap::fromImage(image).scaled(label->size(), Qt::KeepAspectRatio,
                                                      Qt::FastTransformation));
  }
  void refresh() {
    std::shared_ptr<Display> d;
    {
      std::lock_guard<std::mutex> lock(s.mutex);
      d = s.display;
    }
    if (!d)
      return;
    showImage(labels[1], d->images[Colour + binary_mode->currentIndex()]);
    showImage(labels[2], d->images[Bev + bev_mode->currentIndex()]);
    showImage(labels[3], d->images[PathView]);
  }
  void render() {
    std::shared_ptr<Display> d;
    std::string text, error;
    bool playing;
    {
      std::lock_guard<std::mutex> lock(s.mutex);
      d = s.display, text = s.status, error = s.error, playing = s.playing;
    }
    play->setText(playing ? "Pause" : "Play");
    const double now = wall();
    if (d && d->serial != last_serial) {
      last_serial = d->serial;
      ++frames;
      original->setImage(d->images[Overlay]);
      refresh();
      if (!seek_bar->isSliderDown()) {
        QSignalBlocker blocker(seek_bar);
        seek_bar->setRange(0, d->count ? int(d->count - 1) : 0);
        seek_bar->setValue(int(d->frame));
      }
      const auto &r = d->result;
      QStringList lines;
      lines << QString("상태 %1 | %2 | 차로 폭 학습값 %3 px")
                   .arg(QString::fromStdString(r.state))
                   .arg(r.valid ? "유효" : "무효")
                   .arg(r.lane_width_px, 0, 'f', 0);
      lines << QString("제어 입력(읽기 전용, /lane_lite/target): 횡오차 %1 m (좌+) | 헤딩 %2 rad | "
                       "목표 거리 %3 m | 곡률 %4 1/m | 신뢰도 %5")
                   .arg(r.lateral_m, 0, 'f', 3).arg(r.heading, 0, 'f', 3)
                   .arg(r.lookahead_m, 0, 'f', 2).arg(r.curvature, 0, 'f', 2)
                   .arg(r.confidence, 0, 'f', 2);
      for (const auto *line : {&r.left, &r.right})
        lines << QString("%1 %2 | 관측 윈도우 %3 | RMS %4 px | 품질 %5 | %6 | %7")
                     .arg(line == &r.left ? "왼쪽" : "오른쪽")
                     .arg(lane_lite::name(line->state))
                     .arg(line->observed).arg(line->residual, 0, 'f', 1)
                     .arg(line->quality, 0, 'f', 2).arg(line->yellow ? "노랑" : "흰색")
                     .arg(QString::fromStdString(line->note));
      for (const auto &n : r.notes)
        lines << QString::fromStdString(n);
      details->setPlainText(lines.join('\n'));
    }
    if (now - fps_start >= 1) {
      fps = frames / (now - fps_start);
      frames = 0;
      fps_start = now;
    }
    QString info = QString::fromStdString(text);
    if (d)
      info += QString("\nFrame %1 / %2 | %3 s | display %4 FPS | CV %5 ms | decode+CV %6 ms")
                  .arg(d->frame).arg(d->count).arg(d->time, 0, 'f', 2).arg(fps, 0, 'f', 1)
                  .arg(d->result.processing_ms, 0, 'f', 1).arg(d->total_ms, 0, 'f', 1);
    if (pending.valid() && pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto response = pending.get();
      note = response.successful ? "Applied to lane_lite_node" : response.reason;
      pending = {};
    }
    if (!note.empty())
      info += "\n" + QString::fromStdString(note);
    if (!error.empty())
      info += "\nError: " + QString::fromStdString(error);
    status->setText(info);
  }
  Shared &s;
  std::shared_ptr<rclcpp::AsyncParametersClient> client;
  std::shared_future<rcl_interfaces::msg::SetParametersResult> pending;
  std::string note;
  lane_single::RoiView *original = nullptr;
  QLabel *labels[4]{}, *status = nullptr;
  QComboBox *binary_mode = nullptr, *bev_mode = nullptr;
  QPlainTextEdit *details = nullptr;
  QPushButton *play = nullptr;
  QSlider *seek_bar = nullptr;
  QLineEdit *topic = nullptr;
  std::map<std::string, QDoubleSpinBox *> spins;
  std::map<std::string, QSlider *> sliders;
  size_t last_serial = 0, frames = 0;
  double fps = 0, fps_start = wall();
};
} // namespace

int main(int argc, char **argv) {
  std::string bag, config, topic, transport = "auto", screenshot, profile;
  int64_t start_frame = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help") {
      std::cout << "lane_lite_tuner [--profile sim|real] [--bag URI] [--frame N] [--topic NAME] "
                   "[--transport auto|raw|compressed] [--config YAML] [--screenshot PNG]\n"
                   "  --profile NAME loads config/NAME/lane.yaml from the package source tree\n";
      return 0;
    }
    if (i + 1 >= argc)
      continue;
    if (a == "--bag") bag = argv[++i];
    else if (a == "--frame") start_frame = std::stoll(argv[++i]);
    else if (a == "--topic") topic = argv[++i];
    else if (a == "--transport") transport = argv[++i];
    else if (a == "--config") config = argv[++i];
    else if (a == "--profile") profile = argv[++i];
    else if (a == "--screenshot") screenshot = argv[++i];
  }
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  cv::setNumThreads(1);
  if (config.empty() && !profile.empty())
    config = std::string(LANE_LITE_SOURCE_DIR) + "/config/" + profile + "/lane.yaml";
  Shared s;
  s.config_path = config;
  try {
    if (!config.empty())
      s.config = lane_lite::Config::load(config);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  s.live = bag.empty();
  if (s.live)
    s.status = "Live ROS camera | preview only";
  auto node = std::make_shared<rclcpp::Node>("lane_lite_tuner");
  // Default: the team's compressed colour topic; raw is explicit or inferred
  // from a custom topic name without "compressed".
  const bool raw = transport == "raw" ||
                   (transport == "auto" && !topic.empty() && topic.find("compressed") == std::string::npos);
  const auto qos = rclcpp::SensorDataQoS().keep_last(1);
  rclcpp::SubscriptionBase::SharedPtr subscription;
  if (raw)
    subscription = node->create_subscription<sensor_msgs::msg::Image>(
        topic.empty() ? "/camera/color/image_raw" : topic, qos,
        [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(s.mutex);
          s.raw = m, s.compressed.reset(), s.received = wall(), ++s.incoming;
          s.cv.notify_one();
        });
  else
    subscription = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic.empty() ? "/camera/color/image_raw/compressed" : topic, qos,
        [&](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(s.mutex);
          s.compressed = m, s.raw.reset(), s.received = wall(), ++s.incoming;
          s.cv.notify_one();
        });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros([&] { executor.spin(); });
  std::thread processing([&] { worker(s); });
  Window window(s, node);
  window.show();
  if (!bag.empty())
    window.openBag(bag, start_frame);
  if (!screenshot.empty())
    QTimer::singleShot(6000, [&] {
      window.grab().save(QString::fromStdString(screenshot));
      app.quit();
    });
  const int code = app.exec();
  s.stop = true;
  s.cv.notify_all();
  executor.cancel();
  ros.join();
  processing.join();
  rclcpp::shutdown();
  return code;
}
