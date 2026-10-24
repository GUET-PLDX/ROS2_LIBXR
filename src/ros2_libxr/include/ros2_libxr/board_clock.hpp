#pragma once

#include <cstdint>
#include <limits>

namespace ros2_libxr {

// Maps the gimbal board's 32-bit microsecond counter to host time.
// offset = host_receipt - board_time is smallest for the least delayed packet, so the
// running minimum tracks the true offset; it may rise by `drift_ppm` to follow clock drift.
// A board reset or a jump larger than `resync_us` restarts the mapping.
class BoardClockMapper
{
public:
  explicit BoardClockMapper(double drift_ppm = 100.0, int64_t resync_us = 500000)
  : drift_ppm_(drift_ppm), resync_us_(resync_us)
  {
  }

  void reset() { initialized_ = false; }

  // @param board_us gimbal-board counter (wraps at 2^32)
  // @param host_us host receipt time in microseconds
  // @return host time of the sample in microseconds, strictly increasing
  int64_t map(uint32_t board_us, int64_t host_us)
  {
    if (initialized_) {
      const int64_t delta = static_cast<int32_t>(board_us - last_board_raw_);
      board_us_ += delta;
      const int64_t offset = host_us - board_us_;
      // Backwards board time, or a mapping far from the observed delay: restart.
      if (delta < 0 || offset < offset_us_ - resync_us_ || offset > offset_us_ + resync_us_) {
        initialized_ = false;
      } else {
        // Let the minimum creep upward with drift, then take any lower observation.
        offset_us_ += static_cast<int64_t>(static_cast<double>(delta) * drift_ppm_ * 1e-6);
        if (offset < offset_us_) {
          offset_us_ = offset;
        }
      }
    }
    if (!initialized_) {
      initialized_ = true;
      board_us_ = board_us;
      offset_us_ = host_us - board_us_;
      last_out_us_ = std::numeric_limits<int64_t>::min();
    }
    last_board_raw_ = board_us;
    int64_t out = board_us_ + offset_us_;
    if (out <= last_out_us_) {
      out = last_out_us_ + 1;
    }
    last_out_us_ = out;
    return out;
  }

private:
  double drift_ppm_;
  int64_t resync_us_;
  bool initialized_{false};
  uint32_t last_board_raw_{0};
  int64_t board_us_{0};
  int64_t offset_us_{0};
  int64_t last_out_us_{0};
};

}  // namespace ros2_libxr
