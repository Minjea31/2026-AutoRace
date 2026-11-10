#pragma once
// Latest deep masks (/lane_fusion/deep_mask, mono8 class ids). The deep node
// copies the CAMERA header, so a mask's stamp is the capture time of the frame
// it was computed from. Frames never wait for the network: each frame takes
// the newest mask captured at or before it, at most `max_age` older (the
// pipeline then moves it into the frame with the odometry). A mask from the
// future (stamp > frame) is never used. Not thread-safe: the caller holds its
// own mutex. Shared by lane_fusion_node and the tuner.
#include <deque>
#include <sensor_msgs/msg/image.hpp>

namespace lane_lite {
class DeepLatest {
public:
  using Mask = sensor_msgs::msg::Image::ConstSharedPtr;
  static int64_t nanos(const builtin_interfaces::msg::Time &t) {
    return int64_t(t.sec) * 1000000000 + t.nanosec;
  }
  // `now` (wall time) of the frame arrival, to measure the network latency.
  void addFrame(int64_t stamp, double now) {
    arrivals_.push_back({stamp, now});
    while (arrivals_.size() > 64)
      arrivals_.pop_front();
  }
  // False (ignored) for a malformed mask.
  bool addMask(const Mask &m, double now) {
    if (m->encoding != "mono8" || m->step < m->width || m->data.size() < size_t(m->step) * m->height)
      return false;
    const auto t = nanos(m->header.stamp);
    for (const auto &[stamp, arrival] : arrivals_)
      if (stamp == t) {
        const double sample = now - arrival;
        latency_ = latency_ < 0 ? sample : .9 * latency_ + .1 * sample;
        break;
      }
    masks_.push_back(m);
    while (masks_.size() > 8)
      masks_.pop_front();
    return true;
  }
  // Mask for the frame captured at `stamp` (null: none usable). age: capture
  // time difference (s); own: the frame's own mask.
  Mask pick(int64_t stamp, double max_age, double &age, bool &own) const {
    for (auto m = masks_.rbegin(); m != masks_.rend(); ++m) {
      const auto s = nanos((*m)->header.stamp);
      if (s > stamp)
        continue;
      age = double(stamp - s) * 1e-9;
      own = s == stamp;
      return age <= max_age ? *m : nullptr;
    }
    age = -1, own = false;
    return nullptr;
  }
  double latency() const { return latency_; }

private:
  std::deque<std::pair<int64_t, double>> arrivals_;
  std::deque<Mask> masks_;
  double latency_ = -1;
};
} // namespace lane_lite
