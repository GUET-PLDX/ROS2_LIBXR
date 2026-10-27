#pragma once

#include <cmath>

namespace ros2_libxr {
struct ChassisSetpoint
{
  float fx{0.0F}, fy{0.0F}, mz{0.0F};  // N, N*m
};

struct TimedSetpoint
{
  ChassisSetpoint value{};
  double stamp{-1.0};
  double received{-1.0};  // steady-clock receipt time
};

inline ChassisSetpoint FreshCommand(
  const TimedSetpoint & command, double now, double steady_now, double timeout)
{
  const double age = now - command.stamp;
  const double receipt_age = steady_now - command.received;
  const auto & w = command.value;
  if (command.stamp < 0.0 || command.received < 0.0 || !std::isfinite(age) || !std::isfinite(receipt_age) ||
      age < 0.0 || age > timeout || receipt_age < 0.0 || receipt_age > timeout || !std::isfinite(w.fx) ||
      !std::isfinite(w.fy) || !std::isfinite(w.mz)) {
    return {};
  }
  return w;
}

inline ChassisSetpoint RotateToBody(const ChassisSetpoint & in, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {static_cast<float>(c * in.fx + s * in.fy), static_cast<float>(-s * in.fx + c * in.fy), in.mz};
}
}  // namespace ros2_libxr
