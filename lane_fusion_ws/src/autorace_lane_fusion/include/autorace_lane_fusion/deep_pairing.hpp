#pragma once
// Pairs camera frames with /lane_fusion/deep_mask (mono8 class ids; the deep
// node copies the CAMERA header, so a mask belongs to the frame of the same
// stamp). Shared by lane_fusion_node and the tuner's Live mode. Not
// thread-safe: the caller holds its own mutex.
//
// Policy, per call of next():
//  * deep off, or no mask for `timeout` s: the newest frame, HSV only.
//  * the newest frame whose own mask has arrived, or
//  * the newest frame that has waited longer than a mask normally takes
//    (measured latency x1.2 + 10 ms, at most `wait`): the deep node skipped it
//    while busy, so it runs with the latest earlier mask (<= kStaleMask s
//    older than the frame) instead of being dropped. Whichever of the two is newer wins and
//    older frames are dropped (the worker is behind).
// So the output follows the camera rate, not the network rate, and no frame
// waits longer than the network actually needs.
#include <deque>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace lane_lite {
class DeepPairing {
public:
  using Frame = sensor_msgs::msg::CompressedImage::ConstSharedPtr;
  using Mask = sensor_msgs::msg::Image::ConstSharedPtr;
  static constexpr double kStaleMask = .25;

  void addFrame(const Frame &f, double now) {
    frames_.push_back({f, now});
    while (frames_.size() > 16)
      frames_.pop_front();
    // Arrival times outlive frames_, so the latency is measured also for a
    // frame already processed with an earlier mask (else it reads short).
    arrivals_.push_back({nanos(f->header.stamp), now});
    while (arrivals_.size() > 32)
      arrivals_.pop_front();
  }
  // False (ignored) for a malformed mask.
  bool addMask(const Mask &m, double now) {
    if (m->encoding != "mono8" || m->step < m->width ||
        m->data.size() < size_t(m->step) * m->height)
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
    seen_ = now;
    return true;
  }
  bool deepAlive(double now, double timeout) const { return now - seen_ < timeout; }
  double latency() const { return latency_; }
  // Picks the next frame (true) and the mask to use with it (null: HSV only;
  // `own` tells whether it is the frame's own mask).
  bool next(bool deep, double now, double wait, double timeout, Frame &frame, Mask &mask,
            bool &own) {
    mask.reset();
    own = false;
    if (frames_.empty())
      return false;
    if (!deep || !deepAlive(now, timeout)) {
      frame = frames_.back().msg;
      frames_.clear();
      return true;
    }
    const double patience = latency_ < 0 ? wait : std::min(wait, 1.2 * latency_ + .01);
    int pick = -1;
    for (int i = int(frames_.size()) - 1; i >= 0 && pick < 0; --i) {
      const auto t = nanos(frames_[i].msg->header.stamp);
      for (const auto &m : masks_)
        if (nanos(m->header.stamp) == t) {
          pick = i, mask = m, own = true;
          break;
        }
      if (pick < 0 && now - frames_[i].arrival >= patience)
        pick = i;
    }
    if (pick < 0)
      return false; // its mask may still come
    frame = frames_[pick].msg;
    const auto t = nanos(frame->header.stamp);
    if (!own) // latest earlier mask, if recent enough
      for (auto m = masks_.rbegin(); m != masks_.rend(); ++m) {
        const auto s = nanos((*m)->header.stamp);
        if (s <= t) {
          if (double(t - s) * 1e-9 <= kStaleMask)
            mask = *m;
          break;
        }
      }
    frames_.erase(frames_.begin(), frames_.begin() + pick + 1);
    // Keep the newest mask at or before t: the next skipped frame may reuse it.
    while (masks_.size() > 1 && nanos(masks_[1]->header.stamp) <= t)
      masks_.pop_front();
    return true;
  }

private:
  static int64_t nanos(const builtin_interfaces::msg::Time &t) {
    return int64_t(t.sec) * 1000000000 + t.nanosec;
  }
  struct Pending {
    Frame msg;
    double arrival;
  };
  std::deque<Pending> frames_;
  std::deque<std::pair<int64_t, double>> arrivals_;
  std::deque<Mask> masks_;
  double seen_ = -1e9, latency_ = -1;
};
} // namespace lane_lite
