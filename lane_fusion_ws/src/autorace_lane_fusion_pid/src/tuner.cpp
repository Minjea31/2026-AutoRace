// Separate PID tuner window. It only edits /lane_fusion_pid_controller parameters
// (live, via the ROS parameter service) and shows the controller's own state;
// lane perception parameters stay in lane_fusion_tuner.
#include "autorace_lane_fusion_pid/pid.hpp"
#include <QApplication>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <filesystem>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <QCheckBox>
#include <set>
#include <thread>

#ifndef LANE_PID_SOURCE_DIR
#define LANE_PID_SOURCE_DIR ""
#endif

namespace {
// Parameters shown by default (PID): the ones tuned in practice. The rest
// (calibration, rarely changed limits, unused ones) stay in the YAML and are
// still loaded, saved and applied; "고급 파라미터 표시" shows them.
const std::set<std::string> kBasic = {"lateral_weight", "heading_weight", "lookahead_weight", "kp", "kd", "curvature_ff", "max_steering", "steering_rate", "speed", "steer_slowdown", "curvature_slowdown"};
const char *kNode = "lane_fusion_pid_controller";

struct Live {
  std::mutex mutex;
  std::vector<float> state;
  double received = 0;
};
double wall() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
// Installed configs are build copies; saving goes to the package source tree.
std::string editablePath(const std::string &path) {
  const std::string marker = "/share/autorace_lane_fusion_pid/config/", source = LANE_PID_SOURCE_DIR;
  const auto at = path.find(marker);
  if (at == std::string::npos || source.empty() || !std::filesystem::is_directory(source + "/config"))
    return path;
  return source + "/config/" + path.substr(at + marker.size());
}

class Window : public QMainWindow {
public:
  Window(const rclcpp::Node::SharedPtr &node, Live &live, std::string path)
      : node_(node), live_(live), path_(std::move(path)) {
    client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, kNode);
    resize(1180, 860);
    auto central = new QWidget;
    setCentralWidget(central);
    auto outer = new QVBoxLayout(central);

    // Top bar: connection + file + drive toggle.
    auto bar = new QHBoxLayout;
    outer->addLayout(bar);
    connection_ = new QLabel;
    drive_ = new QPushButton;
    drive_->setMinimumHeight(38);
    drive_->setMinimumWidth(150);
    auto fetch = new QPushButton("노드 값 불러오기");
    fetch->setToolTip("실행 중인 /lane_fusion_pid_controller의 현재 파라미터를 창에 표시");
    auto load = new QPushButton("Load YAML");
    auto save = new QPushButton("Save YAML");
    auto defaults = new QPushButton("기본값");
    defaults->setToolTip("코드 기본값으로 되돌리고 노드에 바로 적용");
    bar->addWidget(drive_);
    bar->addWidget(connection_, 1);
    for (QWidget *w : std::initializer_list<QWidget *>{fetch, load, save, defaults})
      bar->addWidget(w);
    auto advanced = new QCheckBox("고급 파라미터 표시");
    advanced->setToolTip("안전·필터·제한값 등 평소엔 바꾸지 않는 파라미터");
    bar->addWidget(advanced);

    auto split = new QSplitter;
    outer->addWidget(split, 1);

    // Left: one group per stage, each row = name [unit] / description / slider + value.
    auto scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    auto params = new QWidget;
    auto column = new QVBoxLayout(params);
    std::map<std::string, QVBoxLayout *> boxes;
    std::map<std::string, QGroupBox *> group_boxes;
    std::vector<QWidget *> hidden;
    std::set<std::string> basic_groups;
    for (const char *group : lane_pid::groups()) {
      auto box = new QGroupBox(QString::fromUtf8(group));
      box->setStyleSheet("QGroupBox{font-weight:bold;}");
      boxes[group] = new QVBoxLayout(box);
      group_boxes[group] = box;
      column->addWidget(box);
    }
    column->addStretch();
    scroll->setWidget(params);
    split->addWidget(scroll);
    for (const auto &p : lane_pid::specs()) {
      if (std::string(p.name) == "active")
        continue; // shown as the drive toggle button
      auto row = new QWidget;
      auto stack = new QVBoxLayout(row);
      stack->setContentsMargins(4, 4, 4, 6);
      auto title = new QLabel(QString("<b><code>%1</code></b> <span style='color:#888'>[%2]</span>")
                                  .arg(p.name).arg(QString::fromUtf8(p.unit)));
      auto help = new QLabel(QString::fromUtf8(p.help));
      help->setWordWrap(true);
      help->setStyleSheet("color:#555; font-weight:normal;");
      auto line = new QHBoxLayout;
      auto slider = new QSlider(Qt::Horizontal);
      slider->setRange(0, qRound((p.max - p.min) / p.step));
      auto spin = new QDoubleSpinBox;
      spin->setRange(p.min, p.max);
      spin->setSingleStep(p.step);
      spin->setDecimals(p.step >= 1 ? 0 : p.step >= .01 ? 2 : 3);
      spin->setKeyboardTracking(false);
      spin->setMinimumWidth(90);
      auto range = new QLabel(QString("%1 ~ %2").arg(p.min).arg(p.max));
      range->setStyleSheet("color:#888;");
      line->addWidget(slider, 1);
      line->addWidget(spin);
      line->addWidget(range);
      stack->addWidget(title);
      stack->addWidget(help);
      stack->addLayout(line);
      boxes.at(p.group)->addWidget(row);
      if (kBasic.count(p.name))
        basic_groups.insert(p.group);
      else
        hidden.push_back(row);
      spins_[p.name] = spin;
      sliders_[p.name] = slider;
      connect(slider, &QSlider::valueChanged, this,
              [spin, p](int i) { spin->setValue(std::clamp(p.min + i * p.step, p.min, p.max)); });
      connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
              [this, key = std::string(p.name)](double v) { edit(key, v); });
    }

    // Right: live controller state (read from /lane_fusion_pid/state).
    auto monitor = new QWidget;
    auto mon = new QVBoxLayout(monitor);
    auto formula = new QLabel(
        "<b>오차</b> e = lateral_weight·가까운 횡오차 + heading_weight·가까운 헤딩 + lookahead_weight·목표점 횡오차 + far_heading_weight·목표점 선 각도<br>"
        "<b>조향</b> δ = kp·e + ki·∫e dt + kd·de/dt + curvature_ff·atan(L·κ)<br>"
        "<span style='color:#888'>좌측 +, 조향은 ±max_steering·steering_rate로 제한</span>");
    formula->setWordWrap(true);
    mon->addWidget(formula);
    auto show_all = [hidden, group_boxes, basic_groups](bool all) {
      for (auto *row : hidden)
        row->setVisible(all);
      for (const auto &[name, box] : group_boxes)
        box->setVisible(all || basic_groups.count(name));
    };
    connect(advanced, &QCheckBox::toggled, this, show_all);
    show_all(false);

    auto values = new QGroupBox("실시간 제어 상태 (/lane_fusion_pid/state)");
    auto form = new QFormLayout(values);
    const char *names[] = {"추종 상태", "결합 오차 e", "P 항", "I 항", "D 항", "곡률 FF",
                           "목표 조향 (포화 후)", "출력 조향", "출력 속도", "가까운 횡오차 (인식)",
                           "가까운 헤딩 (인식)", "곡률 (인식)", "인식 신뢰도", "데이터 나이",
                           "목표점 횡오차 (인식)"};
    for (int i = 0; i < 15; ++i) {
      readouts_[i] = new QLabel("-");
      readouts_[i]->setStyleSheet("font-family:monospace; font-size:13px;");
      form->addRow(QString::fromUtf8(names[i]), readouts_[i]);
    }
    mon->addWidget(values);
    auto bars = new QGroupBox("조향 기여도 (±max_steering 기준)");
    auto grid = new QGridLayout(bars);
    const char *bar_names[] = {"P", "I", "D", "FF", "출력"};
    for (int i = 0; i < 5; ++i) {
      bars_[i] = new QProgressBar;
      bars_[i]->setRange(-1000, 1000);
      bars_[i]->setTextVisible(false);
      grid->addWidget(new QLabel(bar_names[i]), i, 0);
      grid->addWidget(bars_[i], i, 1);
    }
    mon->addWidget(bars);
    mon->addStretch();
    split->addWidget(monitor);
    split->setStretchFactor(0, 3);
    split->setStretchFactor(1, 2);

    status_ = new QLabel;
    status_->setWordWrap(true);
    outer->addWidget(status_);

    connect(drive_, &QPushButton::clicked, this, [this] {
      edit("active", config_["active"] > .5 ? 0. : 1.);
      syncWidgets();
    });
    connect(fetch, &QPushButton::clicked, this, [this] { fetch_requested_ = true; });
    connect(load, &QPushButton::clicked, this, [this] {
      const auto file = QFileDialog::getOpenFileName(this, "Load PID YAML",
                                                     QString::fromStdString(editablePath(path_)),
                                                     "YAML (*.yaml *.yml)");
      if (file.isEmpty())
        return;
      try {
        replaceAll(lane_pid::Config::load(file.toStdString()));
        path_ = file.toStdString();
        updateTitle();
      } catch (const std::exception &e) {
        QMessageBox::warning(this, "YAML", e.what());
      }
    });
    connect(save, &QPushButton::clicked, this, [this] {
      const auto file = QFileDialog::getSaveFileName(
          this, "Save PID YAML",
          QString::fromStdString(path_.empty() ? std::string("pid.yaml") : editablePath(path_)),
          "YAML (*.yaml)");
      if (file.isEmpty())
        return;
      try {
        config_.save(file.toStdString());
        path_ = file.toStdString();
        updateTitle();
        note_ = "저장함: " + path_ + " (launch 기본값에 반영하려면 다시 빌드)";
      } catch (const std::exception &e) {
        QMessageBox::warning(this, "YAML", e.what());
      }
    });
    connect(defaults, &QPushButton::clicked, this, [this] {
      lane_pid::Config c; // gains back to code defaults, node options (parking_calibration_stop ...) kept
      c.options = config_.options;
      replaceAll(c);
    });

    if (!path_.empty()) {
      try {
        config_ = lane_pid::Config::load(path_);
      } catch (const std::exception &e) {
        note_ = std::string("YAML 읽기 실패, 기본값 사용: ") + e.what();
      }
    }
    syncWidgets();
    updateTitle();
    auto timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { tick(); });
    timer->start(50);
  }

private:
  void updateTitle() {
    setWindowTitle(QString::fromStdString("Lane PID 제어 튜너 — " +
                                          (path_.empty() ? std::string("코드 기본값") : editablePath(path_))));
  }
  // A widget edit: validate locally, then queue for the node (coalesced so a
  // slider drag sends at most one request per 50 ms tick).
  void edit(const std::string &key, double value) {
    auto next = config_;
    next.values[key] = value;
    try {
      next.validate();
    } catch (const std::exception &e) {
      note_ = std::string("거부: ") + e.what();
      syncWidgets();
      return;
    }
    config_ = next;
    dirty_[key] = value;
    if (key == "active")
      syncWidgets();
  }
  void replaceAll(const lane_pid::Config &c) {
    config_ = c;
    for (const auto &[k, v] : c.values)
      dirty_[k] = v;
    syncWidgets();
  }
  void syncWidgets() {
    for (auto &[key, spin] : spins_) {
      const auto &p = lane_pid::spec(key);
      QSignalBlocker a(spin), b(sliders_.at(key));
      spin->setValue(config_[key]);
      sliders_.at(key)->setValue(qRound((config_[key] - p.min) / p.step));
    }
    const bool on = config_["active"] > .5;
    drive_->setText(on ? "■ 주행 정지" : "▶ 주행 시작");
    drive_->setStyleSheet(on ? "background:#c62828;color:white;font-weight:bold;"
                             : "background:#2e7d32;color:white;font-weight:bold;");
  }
  void tick() {
    const bool ready = client_->service_is_ready();
    // First connection (or button): show what the running node actually uses.
    // Without a running node, edits stay local until it appears; then the
    // window's values are pushed to it.
    if (ready && !was_ready_) {
      if (dirty_.empty())
        fetch_requested_ = true;
      else
        for (const auto &[k, v] : config_.values)
          dirty_[k] = v;
    }
    was_ready_ = ready;
    if (ready && fetch_requested_ && !get_.valid() && !set_.valid()) {
      fetch_requested_ = false;
      std::vector<std::string> names;
      for (const auto &s : lane_pid::specs())
        names.push_back(s.name);
      get_ = client_->get_parameters(names);
    }
    if (get_.valid() && get_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        auto c = config_;
        for (const auto &p : get_.get())
          c.values[p.get_name()] = p.as_double();
        c.validate();
        config_ = c;
        dirty_.clear();
        syncWidgets();
        note_ = "노드 값 불러옴";
      } catch (const std::exception &e) {
        note_ = std::string("노드 값 읽기 실패: ") + e.what();
      }
      get_ = {};
    }
    if (set_.valid() && set_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto r = set_.get();
      set_ = {};
      if (r.successful)
        note_ = "적용됨";
      else {
        note_ = "노드가 거부: " + r.reason;
        fetch_requested_ = true; // re-sync to the node's values
      }
    }
    if (ready && !dirty_.empty() && !set_.valid() && !get_.valid()) {
      std::vector<rclcpp::Parameter> ps;
      for (const auto &[k, v] : dirty_)
        ps.emplace_back(k, v);
      dirty_.clear();
      set_ = client_->set_parameters_atomically(ps);
    }
    connection_->setText(ready ? "<span style='color:#2e7d32'>● /lane_fusion_pid_controller 연결됨 — 변경 즉시 적용</span>"
                               : "<span style='color:#c62828'>● /lane_fusion_pid_controller 없음 — 값은 창에만 보관, "
                                 "노드가 뜨면 전송</span>");

    std::vector<float> s;
    double received;
    {
      std::lock_guard<std::mutex> lock(live_.mutex);
      s = live_.state;
      received = live_.received;
    }
    const bool fresh = s.size() >= 16 && wall() - received < .5;
    if (fresh) {
      const QString deg = " rad (%1°)";
      auto rad = [&](float v) {
        return QString("%1").arg(v, 7, 'f', 3) + deg.arg(v * 180 / M_PI, 0, 'f', 1);
      };
      readouts_[0]->setText(s[0] > .5 ? (s[14] > .5 ? "추종 중" : "추종 중 (주행 정지됨)")
                                      : "<span style='color:#c62828'>차선 없음/신뢰도 낮음 → 정지</span>");
      readouts_[1]->setText(QString("%1 m").arg(s[1], 7, 'f', 3));
      for (int i = 2; i <= 7; ++i)
        readouts_[i]->setText(rad(s[i]));
      readouts_[8]->setText(QString("%1 m/s").arg(s[8], 6, 'f', 3));
      readouts_[9]->setText(QString("%1 m").arg(s[9], 7, 'f', 3));
      readouts_[10]->setText(rad(s[10]));
      readouts_[11]->setText(QString("%1 1/m").arg(s[11], 7, 'f', 3));
      readouts_[12]->setText(QString("%1").arg(s[12], 5, 'f', 2));
      readouts_[13]->setText(s[13] > 50 ? "수신 없음" : QString("%1 s").arg(s[13], 5, 'f', 2));
      readouts_[14]->setText(QString("%1 m").arg(s[15], 7, 'f', 3));
      const double lim = std::max(1e-3, config_["max_steering"]);
      const float contrib[] = {s[2], s[3], s[4], s[5], s[7]};
      for (int i = 0; i < 5; ++i)
        bars_[i]->setValue(int(std::clamp(contrib[i] / lim, -1., 1.) * 1000));
    } else {
      for (auto *r : readouts_)
        r->setText("-");
      readouts_[0]->setText("/lane_fusion_pid/state 수신 없음");
    }
    status_->setText(QString::fromStdString(note_));
  }
  rclcpp::Node::SharedPtr node_;
  Live &live_;
  std::string path_, note_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  std::shared_future<std::vector<rclcpp::Parameter>> get_;
  std::shared_future<rcl_interfaces::msg::SetParametersResult> set_;
  lane_pid::Config config_;
  std::map<std::string, double> dirty_;
  bool fetch_requested_ = false, was_ready_ = false;
  std::map<std::string, QDoubleSpinBox *> spins_;
  std::map<std::string, QSlider *> sliders_;
  QLabel *readouts_[15]{}, *connection_ = nullptr, *status_ = nullptr;
  QProgressBar *bars_[5]{};
  QPushButton *drive_ = nullptr;
};
} // namespace

int main(int argc, char **argv) {
  std::string config, profile = "sim", screenshot;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help") {
      std::cout << "lane_fusion_pid_tuner [--profile sim|real] [--config pid.yaml] [--screenshot PNG]\n"
                   "  Edits /lane_fusion_pid_controller parameters live.\n"
                   "  --profile NAME loads config/NAME/pid.yaml from the package source tree "
                   "(default sim); --config overrides it.\n";
      return 0;
    }
    if (a == "--config" && i + 1 < argc)
      config = argv[++i];
    else if (a == "--profile" && i + 1 < argc)
      profile = argv[++i];
    else if (a == "--screenshot" && i + 1 < argc)
      screenshot = argv[++i];
  }
  if (config.empty())
    config = std::string(LANE_PID_SOURCE_DIR) + "/config/" + profile + "/pid.yaml";
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lane_fusion_pid_tuner");
  Live live;
  auto sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/lane_fusion_pid/state", rclcpp::QoS(1), [&](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(live.mutex);
        live.state = m->data;
        live.received = wall();
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros([&] { executor.spin(); });
  Window window(node, live, std::filesystem::exists(config) ? config : std::string());
  window.show();
  if (!screenshot.empty()) // verification only: grab the window and exit
    QTimer::singleShot(4000, [&] {
      window.grab().save(QString::fromStdString(screenshot));
      app.quit();
    });
  const int code = app.exec();
  executor.cancel();
  ros.join();
  rclcpp::shutdown();
  return code;
}
