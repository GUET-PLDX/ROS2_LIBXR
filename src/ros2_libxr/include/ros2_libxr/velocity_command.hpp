#pragma once

#include <cmath>

namespace ros2_libxr {
struct ChassisSetpoint
{
  float vx{0.0F}, vy{0.0F}, wz{0.0F};  // m/s and rad/s
};

struct TimedSetpoint
{
  ChassisSetpoint value{};
  double stamp{-1.0};
  double received{-1.0};  // steady-clock receipt time
};

struct TimedHeading
{
  double yaw{0.0}, yaw_rate{0.0};
  double stamp{-1.0}, received{-1.0};
};

inline bool CommandFresh(const TimedSetpoint & command, double now, double steady_now, double timeout)
{
  const double age = now - command.stamp;
  const double receipt_age = steady_now - command.received;
  const auto & v = command.value;
  if (command.stamp < 0.0 || command.received < 0.0 || !std::isfinite(age) || !std::isfinite(receipt_age) ||
      age < 0.0 || age > timeout || receipt_age < 0.0 || receipt_age > timeout || !std::isfinite(v.vx) ||
      !std::isfinite(v.vy) || !std::isfinite(v.wz)) {
    return false;
  }
  return true;
}

inline ChassisSetpoint FreshCommand(
  const TimedSetpoint & command, double now, double steady_now, double timeout)
{
  return CommandFresh(command, now, steady_now, timeout) ? command.value : ChassisSetpoint{};
}

inline ChassisSetpoint WorldToBody(const TimedSetpoint & command,
  const TimedHeading & heading,
  double now,
  double steady_now,
  double command_timeout,
  double odom_timeout,
  double rotate_latency,
  double yaw_offset,
  bool * valid = nullptr)
{
  if (valid) {
    *valid = false;
  }
  const double age = now - heading.stamp;
  const double receipt_age = steady_now - heading.received;
  if (heading.stamp < 0.0 || heading.received < 0.0 || !std::isfinite(age) || !std::isfinite(receipt_age) ||
      age < 0.0 || age > odom_timeout || receipt_age < 0.0 || receipt_age > odom_timeout ||
      !std::isfinite(heading.yaw) || !std::isfinite(heading.yaw_rate) || !std::isfinite(rotate_latency) ||
      rotate_latency < 0.0 || !std::isfinite(yaw_offset) ||
      !CommandFresh(command, now, steady_now, command_timeout)) {
    return {};
  }
  const double yaw = heading.yaw + yaw_offset + heading.yaw_rate * (age + rotate_latency);
  if (!std::isfinite(yaw)) {
    return {};
  }
  const auto world = command.value;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double vx = c * world.vx + s * world.vy;
  const double vy = -s * world.vx + c * world.vy;
  // The inter-board velocity format stores signed values at 0.001 resolution.
  constexpr double kWireLimit = 32.767;
  if (!std::isfinite(vx) || !std::isfinite(vy) || std::abs(vx) > kWireLimit || std::abs(vy) > kWireLimit ||
      std::abs(world.wz) > kWireLimit) {
    return {};
  }
  if (valid) {
    *valid = true;
  }
  return {static_cast<float>(vx), static_cast<float>(vy), world.wz};
}

}  // namespace ros2_libxr
