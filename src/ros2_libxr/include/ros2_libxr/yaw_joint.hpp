#pragma once

#include "ros2_libxr/wire.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <numbers>
#include <utility>

namespace ros2_libxr {

inline constexpr char kYawJointTopic[] = "nav_yaw_joint";
// LibXR packet headers carry a 48-bit microsecond counter.
inline constexpr uint64_t kSourceCounterModulus = 1ULL << 48U;

// Maps the MCU sample counter onto the host (ROS/LIO) clock.
//
// The 48-bit counter is unwrapped across its modulus. Host time is estimated as
// source time + offset, where offset is the minimum of (receive time - source time)
// over a sliding window. Transport delay is never negative, so that minimum is the
// least-delayed packet: the mapped stamp never lies after the receive time and its
// error is the smallest transport latency seen in the window, not the latency of
// the current packet. Receive time is used only to estimate the offset.
class SourceClockMap
{
public:
  explicit SourceClockMap(int64_t window_ns = 5'000'000'000LL) : window_ns_(window_ns) {}

  // raw_us: 48-bit source counter. receive_ns: host clock at reception.
  // Returns the sample time on the host clock, in nanoseconds.
  int64_t Map(uint64_t raw_us, int64_t receive_ns)
  {
    raw_us %= kSourceCounterModulus;
    if (has_last_) {
      // Signed distance modulo 2^48: small positive across a wrap, small negative if reordered.
      int64_t delta = static_cast<int64_t>((raw_us - last_raw_) % kSourceCounterModulus);
      if (delta >= static_cast<int64_t>(kSourceCounterModulus / 2)) {
        delta -= static_cast<int64_t>(kSourceCounterModulus);
      }
      if (delta < -kRestartUs) {
        Reset();  // the MCU restarted: its counter went back by more than a late packet could
      } else {
        const int64_t unwrapped = last_unwrapped_ + delta;
        if (delta > 0) {
          last_raw_ = raw_us;
          last_unwrapped_ = unwrapped;
        }
        return Offset(unwrapped * 1000, receive_ns) + unwrapped * 1000;
      }
    }
    has_last_ = true;
    last_raw_ = raw_us;
    last_unwrapped_ = static_cast<int64_t>(raw_us);
    return Offset(last_unwrapped_ * 1000, receive_ns) + last_unwrapped_ * 1000;
  }

  void Reset()
  {
    has_last_ = false;
    candidates_.clear();
  }

private:
  static constexpr int64_t kRestartUs = 1'000'000;

  int64_t Offset(int64_t source_ns, int64_t receive_ns)
  {
    const int64_t candidate = receive_ns - source_ns;
    while (!candidates_.empty() && candidates_.back().second >= candidate) {
      candidates_.pop_back();
    }
    candidates_.emplace_back(receive_ns, candidate);
    while (candidates_.front().first < receive_ns - window_ns_) {
      candidates_.pop_front();
    }
    return candidates_.front().second;
  }

  int64_t window_ns_;
  bool has_last_{false};
  uint64_t last_raw_{0};
  int64_t last_unwrapped_{0};
  std::deque<std::pair<int64_t, int64_t>> candidates_;  // (receive ns, offset ns), offsets increasing
};

// Wraps an angle to [-pi, pi).
inline double WrapAngle(double angle)
{
  constexpr double kTwoPi = 2.0 * std::numbers::pi;
  double wrapped = std::fmod(angle + std::numbers::pi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - std::numbers::pi;
}

struct YawJointSample
{
  double angle{0.0};     // rad, [-pi, pi)
  double velocity{0.0};  // rad/s
  int64_t stamp_ns{0};   // measurement time on the host clock
  uint64_t mcu_stamp_us{0};
  int64_t receive_ns{0};
};

// Decodes one complete packet payload. Wrong sizes, unset timestamps and
// non-finite values are rejected; nothing is substituted for them.
inline bool DecodeYawJoint(const void * bytes,
  size_t size,
  uint64_t mcu_stamp_us,
  int64_t receive_ns,
  SourceClockMap & clock,
  YawJointSample & sample)
{
  wire::YawJointFeedback value{};
  if (!wire::DecodePayload(bytes, size, value) || mcu_stamp_us == 0 || !std::isfinite(value.angle_rad) ||
      !std::isfinite(value.velocity_rad_s)) {
    return false;
  }
  sample.angle = WrapAngle(value.angle_rad);
  sample.velocity = value.velocity_rad_s;
  sample.mcu_stamp_us = mcu_stamp_us % kSourceCounterModulus;
  sample.receive_ns = receive_ns;
  sample.stamp_ns = clock.Map(mcu_stamp_us, receive_ns);
  return true;
}

}  // namespace ros2_libxr
