#include "autorace_lane_fusion/bag.hpp"
#include <algorithm>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/serialization.hpp>

namespace lane_lite {
cv::Mat decode(const sensor_msgs::msg::Image &msg) {
  return cv_bridge::toCvCopy(msg, "bgr8")->image;
}
cv::Mat decode(const sensor_msgs::msg::CompressedImage &msg) {
  auto image = cv::imdecode(msg.data, cv::IMREAD_COLOR);
  if (image.empty())
    throw std::runtime_error("Cannot decode compressed image");
  return image;
}
void Bag::open(const std::string &path, const std::string &topic,
               const std::atomic<bool> *cancel) {
  reader_ = std::make_unique<rosbag2_cpp::Reader>();
  reader_->open(path);
  path_ = path;
  stamps_.clear();
  topic_.clear();
  type_.clear();
  next_ = 0;
  for (const auto &t : reader_->get_all_topics_and_types())
    if (t.type == "sensor_msgs/msg/CompressedImage" || t.type == "sensor_msgs/msg/Image") {
      if ((!topic.empty() && topic == t.name) ||
          (topic.empty() && (topic_.empty() || t.name == "/camera/color/image_raw/compressed"))) {
        topic_ = t.name;
        type_ = t.type;
      }
    }
  if (topic_.empty())
    throw std::runtime_error("No supported image topic: " + topic);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic_};
  reader_->set_filter(filter);
  while (reader_->has_next()) {
    if (cancel && cancel->load())
      throw std::runtime_error("Bag indexing cancelled");
    stamps_.push_back(reader_->read_next()->time_stamp);
  }
  if (stamps_.empty())
    throw std::runtime_error("Selected image topic is empty");
  if (!std::is_sorted(stamps_.begin(), stamps_.end()))
    throw std::runtime_error("Bag timestamps are not ordered");
  reader_->seek(stamps_.front());
}
std::vector<std::array<double, 3>> Bag::odometry(const std::string &topic, const std::string &imu) const {
  std::vector<std::array<double, 3>> out;
  if (stamps_.empty())
    return out;
  rosbag2_cpp::Reader reader;
  reader.open(path_);
  std::string chosen = topic;
  if (chosen.empty())
    for (const auto *name : {"/odom", "/odometry/filtered"})
      for (const auto &t : reader.get_metadata().topics_with_message_count)
        if (chosen.empty() && t.topic_metadata.name == name && t.message_count > 0)
          chosen = name;
  if (chosen.empty())
    return out;
  rosbag2_storage::StorageFilter filter;
  filter.topics = {chosen};
  reader.set_filter(filter);
  rclcpp::Serialization<nav_msgs::msg::Odometry> serial;
  while (reader.has_next()) {
    const auto m = reader.read_next();
    rclcpp::SerializedMessage s(*m->serialized_data);
    nav_msgs::msg::Odometry msg;
    serial.deserialize_message(&s, &msg);
    out.push_back({(m->time_stamp - stamps_.front()) * 1e-9, msg.twist.twist.linear.x, msg.twist.twist.angular.z});
  }
  if (imu.empty() || out.empty())
    return out;
  std::vector<std::pair<double, double>> gyro;
  rosbag2_cpp::Reader gyro_reader;
  gyro_reader.open(path_);
  rosbag2_storage::StorageFilter gyro_filter;
  gyro_filter.topics = {imu};
  gyro_reader.set_filter(gyro_filter);
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serial;
  while (gyro_reader.has_next()) {
    const auto m = gyro_reader.read_next();
    rclcpp::SerializedMessage s(*m->serialized_data);
    sensor_msgs::msg::Imu msg;
    imu_serial.deserialize_message(&s, &msg);
    gyro.push_back({(m->time_stamp - stamps_.front()) * 1e-9, msg.angular_velocity.z});
  }
  if (gyro.empty())
    return out;
  size_t j = 0; // newest gyro sample at or before each odometry sample
  for (auto &o : out) {
    while (j + 1 < gyro.size() && gyro[j + 1].first <= o[0])
      ++j;
    if (std::abs(gyro[j].first - o[0]) < .1)
      o[2] = gyro[j].second;
  }
  return out;
}
Frame Bag::frame(size_t index) {
  if (index >= size())
    throw std::out_of_range("Frame index");
  if (index != next_) {
    reader_->seek(stamps_[index]);
    // Duplicate recording timestamps still represent separate frames.
    const auto first = std::lower_bound(stamps_.begin(), stamps_.end(), stamps_[index]);
    size_t skip = index - static_cast<size_t>(first - stamps_.begin());
    while (skip-- && reader_->has_next())
      reader_->read_next();
  }
  if (!reader_->has_next())
    throw std::runtime_error("Unexpected end of bag");
  const auto m = reader_->read_next();
  next_ = index + 1;
  rclcpp::SerializedMessage s(*m->serialized_data);
  Frame f;
  f.time = time(index);
  if (type_ == "sensor_msgs/msg/Image") {
    sensor_msgs::msg::Image msg;
    rclcpp::Serialization<sensor_msgs::msg::Image>().deserialize_message(&s, &msg);
    f.image = decode(msg);
  } else {
    sensor_msgs::msg::CompressedImage msg;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage>().deserialize_message(&s, &msg);
    f.image = decode(msg);
  }
  return f;
}
} // namespace lane_lite
