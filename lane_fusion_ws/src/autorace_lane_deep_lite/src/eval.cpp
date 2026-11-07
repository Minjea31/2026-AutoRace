// Offline evaluation on a bag + per-frame deep masks (tools/bag_masks.py --masks DIR):
//   lane_deep_lite_eval BAG CONFIG.yaml MASK_DIR [OUT_DIR] [--every N] [--video OUT.mp4]
// Same summary line as lane_fusion_eval, so the two can be compared directly.
#include "autorace_lane_deep_lite/deep_lane.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "lane_deep_lite_eval BAG CONFIG.yaml MASK_DIR [OUT_DIR] [--every N] "
                 "[--video OUT.mp4] [--topic T]\n";
    return 1;
  }
  const std::string bag_path = argv[1], config_path = argv[2], mask_dir = argv[3];
  std::string out, video_path, topic; // default: compressed camera, else raw
  size_t every = 0;
  for (int i = 4; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--every" && i + 1 < argc) every = std::stoul(argv[++i]);
    else if (a == "--video" && i + 1 < argc) video_path = argv[++i];
    else if (a == "--topic" && i + 1 < argc) topic = argv[++i];
    else out = a;
  }
  try {
    deep_lite::Pipeline pipeline(deep_lite::Config::load(config_path));
    rosbag2_cpp::Reader reader;
    reader.open(bag_path);
    bool raw = false;
    for (const auto &t : reader.get_all_topics_and_types()) {
      const bool image = t.type == "sensor_msgs/msg/CompressedImage" || t.type == "sensor_msgs/msg/Image";
      if (image && (t.name == topic || (topic.empty() && t.name.rfind("/camera/color/image_raw", 0) == 0 &&
                                         t.type == "sensor_msgs/msg/CompressedImage"))) {
        topic = t.name, raw = t.type == "sensor_msgs/msg/Image";
        break;
      }
      if (image && topic.empty() && t.name == "/camera/color/image_raw")
        raw = true;
    }
    if (topic.empty() && raw)
      topic = "/camera/color/image_raw";
    rosbag2_storage::StorageFilter filter;
    filter.topics = {topic};
    reader.set_filter(filter);
    std::ofstream csv;
    if (!out.empty()) {
      std::filesystem::create_directories(out);
      csv.open(out + "/frames.csv");
      csv << "frame,time,state,left,right,valid,lateral_m,heading,curvature,lookahead_m,"
             "confidence,width_px,ms,near_lateral_m,near_heading,notes\n";
    }
    cv::VideoWriter video;
    size_t n = 0, valid = 0, both = 0, single = 0, held = 0, jumps = 0;
    double ms = 0, worst_ms = 0, jitter = 0, previous = NAN, t0 = NAN, kappa_jitter = 0,
           previous_kappa = NAN;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
    rclcpp::Serialization<sensor_msgs::msg::Image> raw_serializer;
    for (size_t i = 0; reader.has_next(); ++i) {
      const auto m = reader.read_next();
      const double stamp = m->time_stamp * 1e-9;
      if (std::isnan(t0))
        t0 = stamp;
      char name[32];
      std::snprintf(name, sizeof name, "/%06zu.png", i);
      const cv::Mat classes = cv::imread(mask_dir + name, cv::IMREAD_UNCHANGED);
      if (classes.empty())
        throw std::runtime_error("missing deep mask " + mask_dir + name);
      const bool show = video_path.size() || (every && i % every == 0 && !out.empty());
      const auto r = pipeline.process(classes, stamp - t0, show);
      ++n;
      ms += r.processing_ms;
      worst_ms = std::max(worst_ms, r.processing_ms);
      valid += r.valid;
      both += r.state == "both";
      single += r.state == "left only" || r.state == "right only";
      held += r.state == "held";
      if (r.valid && std::isfinite(previous)) {
        const double d = std::abs(r.lateral_m - previous);
        jitter += d;
        jumps += d > .05;
        kappa_jitter += std::abs(r.curvature - previous_kappa);
      }
      previous = r.valid ? r.lateral_m : NAN;
      previous_kappa = r.curvature;
      if (csv.is_open()) {
        csv << i << ',' << stamp - t0 << ',' << r.state << ',' << deep_lite::name(r.left.state)
            << ',' << deep_lite::name(r.right.state) << ',' << r.valid << ',' << r.lateral_m << ','
            << r.heading << ',' << r.curvature << ',' << r.lookahead_m << ',' << r.confidence
            << ',' << r.lane_width_px << ',' << r.processing_ms << ',' << r.near_lateral_m << ','
            << r.near_heading << ',';
        for (size_t k = 0; k < r.notes.size(); ++k)
          csv << (k ? " | " : "") << r.notes[k];
        csv << '\n';
      }
      if (show) {
        rclcpp::SerializedMessage s(*m->serialized_data);
        cv::Mat original, row;
        if (raw) {
          sensor_msgs::msg::Image img;
          raw_serializer.deserialize_message(&s, &img);
          original = cv_bridge::toCvCopy(img, "bgr8")->image;
        } else {
          sensor_msgs::msg::CompressedImage img;
          serializer.deserialize_message(&s, &img);
          original = cv::imdecode(img.data, cv::IMREAD_COLOR);
        }
        cv::resize(original, original, r.view.size());
        cv::Mat sized;
        cv::resize(classes, sized, r.view.size(), 0, 0, cv::INTER_NEAREST);
        original.setTo(cv::Scalar(0, 220, 255), sized == 1);
        original.setTo(cv::Scalar(255, 255, 255), sized == 2);
        cv::putText(original, "#" + std::to_string(i), {10, 470}, cv::FONT_HERSHEY_SIMPLEX, .7,
                    {0, 255, 255}, 2);
        cv::hconcat(original, r.view, row);
        if (!video_path.empty()) {
          if (!video.isOpened())
            video.open(video_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, row.size());
          video.write(row);
        }
        if (every && i % every == 0 && !out.empty()) {
          cv::resize(row, row, {}, .5, .5, cv::INTER_AREA);
          cv::imwrite(out + "/frame_" + std::to_string(i) + ".jpg", row);
        }
      }
    }
    if (!n)
      throw std::runtime_error("no frames on " + topic);
    std::cout << "frames " << n << " | valid " << 100. * valid / n << "% | both " << 100. * both / n
              << "% | single " << 100. * single / n << "% | held " << 100. * held / n
              << "% | lateral jitter " << jitter / std::max<size_t>(1, valid)
              << " m/frame | >5cm jumps " << jumps << " | curvature jitter "
              << kappa_jitter / std::max<size_t>(1, valid) << " 1/m/frame | CV " << ms / n
              << " ms (max " << worst_ms << ")\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
