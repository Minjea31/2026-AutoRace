#pragma once
// Sequential/random-access image reader for ROS 2 bags (tuner and eval only).
#include <array>
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
  // Odometry twist of the bag (topic empty: /odom if it has messages, else
  // /odometry/filtered) as (time on the time() clock, forward speed, yaw rate).
  // With an IMU topic (empty: none) the yaw rate is the gyro's, as in the node.
  std::vector<std::array<double, 3>> odometry(const std::string &topic = "", const std::string &imu = "/imu") const;
  const std::string &topic() const { return topic_; }

private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::vector<int64_t> stamps_;
  std::string path_, topic_, type_;
  size_t next_ = 0;
};
} // namespace lane_lite
