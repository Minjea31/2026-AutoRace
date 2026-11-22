// Deep-only lane keeping = lane_ws logic (lane_ws.cpp) fed with the deep mask.
#include "autorace_lane_deep_lite/deep_lane.hpp"
#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace deep_lite {
const std::vector<Spec> &specs() { return lane_lite::specs(); } // lane_ws keys without the HSV ones
Config::Config() : values(lane_lite::Config().values) {}
lane_lite::Config Config::lane() const {
  lane_lite::Config c;
  c.values = values;
  return c;
}
void Config::validate() const { lane().validate(); }
Config Config::load(const std::string &path) {
  Config c;
  const auto root = YAML::LoadFile(path);
  auto n = root["lane_deep_lite"];
  if (!n)
    n = root["lane_lite"];
  if (!n || !n.IsMap())
    throw std::invalid_argument("YAML needs a lane_deep_lite (or lane_lite) map: " + path);
  for (const auto &kv : n) {
    const auto key = kv.first.as<std::string>();
    if (c.values.count(key))
      c.values[key] = kv.second.as<double>();
  }
  c.validate();
  return c;
}
void Config::save(const std::string &path) const {
  validate();
  YAML::Emitter e;
  e << YAML::BeginMap << YAML::Key << "lane_deep_lite" << YAML::Value << YAML::BeginMap;
  for (const auto &s : specs())
    e << YAML::Key << s.name << YAML::Value << values.at(s.name);
  e << YAML::EndMap << YAML::EndMap;
  std::ofstream f(path);
  f << e.c_str() << '\n';
  if (!f)
    throw std::runtime_error("Cannot save YAML: " + path);
}
const char *name(LineState state) {
  switch (state) {
  case LineState::Detected: return "DETECTED";
  case LineState::Held: return "HELD";
  default: return "LOST";
  }
}

Pipeline::Pipeline(const Config &config) { configure(config); }
void Pipeline::configure(const Config &config) {
  config.validate();
  config_ = config;
  lane_.configure(config.lane());
}
void Pipeline::reset() { lane_.reset(); }
Result Pipeline::process(const cv::Mat &classes, double time, bool views) {
  if (classes.empty() || classes.type() != CV_8U)
    throw std::invalid_argument("Expected a CV_8U class mask");
  if (blank_.size() != classes.size())
    blank_ = cv::Mat::zeros(classes.size(), CV_8UC3);
  const auto w = lane_.process(blank_, time, views, classes);
  Result r;
  r.bev_classes = w.bev_mask;
  if (views)
    r.view = w.path_view;
  auto line = [](const lane_lite::Line &in) {
    Line out;
    out.path = in.path;
    out.state = in.state == lane_lite::LineState::Detected  ? LineState::Detected
                : in.state == lane_lite::LineState::Predicted ? LineState::Held
                                                              : LineState::Lost;
    out.cls = in.path.empty() ? 0 : in.yellow ? 1 : 2;
    out.quality = in.quality;
    out.age = in.age;
    out.note = in.note;
    return out;
  };
  r.left = line(w.left);
  r.right = line(w.right);
  r.center_path = w.center_path;
  r.valid = w.valid;
  r.target_x = w.target_x, r.target_y = w.target_y;
  r.lateral_m = w.lateral_m, r.heading = w.heading, r.curvature = w.curvature;
  r.lookahead_m = w.lookahead_m, r.confidence = w.confidence;
  r.near_lateral_m = w.near_lateral_m, r.near_heading = w.near_heading, r.near_m = w.near_m;
  r.lane_width_px = w.lane_width_px, r.processing_ms = w.processing_ms;
  r.state = w.state;
  r.notes = w.notes;
  return r;
}
} // namespace deep_lite
