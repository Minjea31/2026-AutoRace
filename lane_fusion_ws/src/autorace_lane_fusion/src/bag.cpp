#include "autorace_lane_fusion/bag.hpp"
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
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
