#pragma once
// Sequential/random-access image reader for ROS 2 bags (tuner and eval only).
#include <atomic>
#include <memory>
#include <opencv2/core.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

namespace lane_lite {
cv::Mat decode(const sensor_msgs::msg::Image &msg);
cv::Mat decode(const sensor_msgs::msg::CompressedImage &msg);
struct Frame {
  cv::Mat image;
  double time = 0;
};
class Bag {
public:
  // The index holds timestamps only, never image payloads.
  void open(const std::string &path, const std::string &topic,
            const std::atomic<bool> *cancel = nullptr);
  Frame frame(size_t index);
  size_t size() const { return stamps_.size(); }
  double time(size_t index) const { return (stamps_.at(index) - stamps_.front()) * 1e-9; }
  const std::string &topic() const { return topic_; }

private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::vector<int64_t> stamps_;
  std::string topic_, type_;
  size_t next_ = 0;
};
} // namespace lane_lite
