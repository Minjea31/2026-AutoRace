// Offline bag evaluation: per-frame CSV, summary, and sampled debug images.
#include "autorace_lane_fusion/lane.hpp"
#include "autorace_lane_fusion/bag.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "lane_fusion_eval BAG|IMAGE CONFIG.yaml [OUT_DIR|OUT.png] [--topic T] "
                 "[--every N] [--start I] [--count N] [--no-views] [--video OUT.mp4] [--tracks]\n"
                 "  [--deep-masks DIR (NNNNNN.png per bag frame)] [--deep-mode 0|1|2]\n";
    return 1;
  }
  std::string bag_path = argv[1], config_path = argv[2], out, topic, video_path, deep_dir;
  int deep_mode = -1; // -1: keep the YAML value
  bool tracks = false; // --tracks: also draw the tracked boundaries/centre in the video
  size_t every = 0, first = 0, count = ~size_t(0);
  bool views = true; // --no-views: time the pipeline as the driving node runs it
  for (int i = 3; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--topic" && i + 1 < argc) topic = argv[++i];
    else if (a == "--every" && i + 1 < argc) every = std::stoul(argv[++i]);
    else if (a == "--start" && i + 1 < argc) first = std::stoul(argv[++i]);
    else if (a == "--count" && i + 1 < argc) count = std::stoul(argv[++i]);
    else if (a == "--no-views") views = false;
    else if (a == "--video" && i + 1 < argc) video_path = argv[++i];
    else if (a == "--tracks") tracks = true;
    else if (a == "--deep-masks" && i + 1 < argc) deep_dir = argv[++i];
    else if (a == "--deep-mode" && i + 1 < argc) deep_mode = std::stoi(argv[++i]);
    else out = a;
  }
  try {
    auto config = lane_lite::Config::load(config_path);
    if (deep_mode >= 0)
      config.values["deep_mode"] = deep_mode;
    config.validate();
    lane_lite::Pipeline pipeline(config);
    size_t deep_frames = 0;
    auto deep_mask = [&](size_t index) {
      if (deep_dir.empty())
        return cv::Mat();
      char name[32];
      std::snprintf(name, sizeof name, "/%06zu.png", index);
      cv::Mat m = cv::imread(deep_dir + name, cv::IMREAD_UNCHANGED);
      if (m.empty())
        throw std::runtime_error("missing deep mask " + deep_dir + name);
      ++deep_frames;
      return m;
    };
    const auto ext = std::filesystem::path(bag_path).extension().string();
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
      // Single still image: a few identical frames let tracking settle.
      const cv::Mat image = cv::imread(bag_path);
      if (image.empty())
        throw std::runtime_error("cannot read " + bag_path);
      lane_lite::Result r;
      for (int i = 0; i < 5; ++i)
        r = pipeline.process(image, i * .05);
      std::cout << r.state << " valid=" << r.valid << " left=" << lane_lite::name(r.left.state)
                << " right=" << lane_lite::name(r.right.state) << " lateral_m=" << r.lateral_m
                << " heading=" << r.heading << " width_px=" << r.lane_width_px << '\n';
      for (const auto &note : r.notes)
        std::cout << "  " << note << '\n';
      if (!out.empty()) {
        cv::Mat mask, row;
        cv::cvtColor(r.feature_mask, mask, cv::COLOR_GRAY2BGR);
        cv::hconcat(std::vector<cv::Mat>{r.overlay, mask, r.search_view, r.path_view}, row);
        cv::imwrite(out, row);
      }
      return 0;
    }
    lane_lite::Bag bag;
    bag.open(bag_path, topic);
    std::ofstream csv;
    if (!out.empty()) {
      std::filesystem::create_directories(out);
      csv.open(out + "/frames.csv");
      csv << "frame,time,state,left,right,valid,lateral_m,heading,lookahead_m,"
             "confidence,width_px,ms,near_lateral_m,near_heading,notes\n";
    }
    // --video: recognised lane paint on the ORIGINAL frame (yellow / white).
    cv::VideoWriter video;
    if (!video_path.empty()) {
      views = true;
      const double fps = bag.size() > 1 ? (bag.size() - 1) / std::max(1e-3, bag.time(bag.size() - 1)) : 15;
      const auto probe = bag.frame(first).image;
      video.open(video_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, probe.size());
      if (!video.isOpened())
        throw std::runtime_error("cannot open video " + video_path);
    }
    size_t n = 0, valid = 0, both = 0, single = 0, held = 0, jumps = 0;
    double ms = 0, worst_ms = 0, jitter = 0, previous = NAN;
    const size_t last = std::min(bag.size(), first + count);
    for (size_t i = first; i < last; ++i) {
      auto frame = bag.frame(i);
      auto r = pipeline.process(frame.image, frame.time, views || (every && i % every == 0),
                                deep_mask(i));
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
      }
      previous = r.valid ? r.lateral_m : NAN;
      if (csv.is_open()) {
        csv << i << ',' << frame.time << ',' << r.state << ','
            << lane_lite::name(r.left.state) << ',' << lane_lite::name(r.right.state)
            << ',' << r.valid << ',' << r.lateral_m << ',' << r.heading << ','
            << r.lookahead_m << ',' << r.confidence << ',' << r.lane_width_px
            << ',' << r.processing_ms << ',' << r.near_lateral_m << ','
            << r.near_heading << ',';
        for (size_t k = 0; k < r.notes.size(); ++k)
          csv << (k ? " | " : "") << r.notes[k];
        csv << '\n';
      }
      if (video.isOpened()) {
        // Dim the original so recognised white paint stands out from raw paint.
        cv::Mat frame_out = frame.image * .55;
        frame_out.setTo(cv::Scalar(0, 220, 255), r.yellow_mask);
        frame_out.setTo(cv::Scalar(255, 255, 255), r.feature_mask & ~r.yellow_mask);
        if (tracks) { // projected boundaries (red) and centre path (blue) from r.overlay
          cv::Mat diff, drawn;
          cv::absdiff(r.overlay, r.original, diff);
          cv::cvtColor(diff, drawn, cv::COLOR_BGR2GRAY);
          r.overlay.copyTo(frame_out, drawn > 0);
        }
        const std::string label = std::string("lane_fusion (") + r.source + ")  #" +
                                  std::to_string(i) + "  " + r.state;
        cv::putText(frame_out, label, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 0, 0}, 4);
        cv::putText(frame_out, label, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 255, 0}, 2);
        video.write(frame_out);
      }
      if (!out.empty() && every && i % every == 0) {
        cv::Mat mask, row;
        cv::cvtColor(r.feature_mask, mask, cv::COLOR_GRAY2BGR);
        cv::hconcat(std::vector<cv::Mat>{r.overlay, mask, r.search_view, r.path_view}, row);
        cv::putText(row, "#" + std::to_string(i) + " " + r.state, {10, 470},
                    cv::FONT_HERSHEY_SIMPLEX, .7, {0, 255, 255}, 2);
        cv::resize(row, row, {}, .5, .5, cv::INTER_AREA);
        cv::imwrite(out + "/frame_" + std::to_string(i) + ".jpg", row);
      }
    }
    std::cout << "frames " << n << " | valid " << 100. * valid / n << "% | both "
              << 100. * both / n << "% | single " << 100. * single / n
              << "% | held " << 100. * held / n << "% | lateral jitter "
              << jitter / std::max<size_t>(1, valid) << " m/frame | >5cm jumps "
              << jumps << " | CV " << ms / n << " ms (max " << worst_ms << ")"
              << (deep_frames ? " | deep mode " + std::to_string(int(config["deep_mode"])) : "") << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
