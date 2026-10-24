#pragma once

#include <cmath>

namespace ros2_libxr {

// One velocity command with an optional drive wrench (force feedforward), in the frame the
// controller published it.
struct ChassisSetpoint
{
  float vx{0.0F}, vy{0.0F}, wz{0.0F};  // m/s, rad/s
  float fx{0.0F}, fy{0.0F}, mz{0.0F};  // N, N*m
};

struct TimedSetpoint
{
  ChassisSetpoint value{};
  double stamp{-1.0};  // receipt time, s; negative means never received
};

inline bool Fresh(const TimedSetpoint & s, double now, double timeout)
{
  return s.stamp >= 0.0 && now - s.stamp >= 0.0 && now - s.stamp <= timeout;
}

inline bool Finite(const ChassisSetpoint & s)
{
  return std::isfinite(s.vx) && std::isfinite(s.vy) && std::isfinite(s.wz) && std::isfinite(s.fx) &&
         std::isfinite(s.fy) && std::isfinite(s.mz);
}

// A fresh ChassisCommand (velocity + feedforward from one controller cycle) wins over the
// velocity-only Twist, so the two never mix. Otherwise a fresh Twist is sent with no force,
// and with neither fresh the chassis gets zero.
inline ChassisSetpoint Arbitrate(
  const TimedSetpoint & twist, const TimedSetpoint & chassis, double now, double timeout)
{
  if (Fresh(chassis, now, timeout) && Finite(chassis.value)) {
    return chassis.value;
  }
  if (Fresh(twist, now, timeout) && Finite(twist.value)) {
    ChassisSetpoint out = twist.value;
    out.fx = out.fy = out.mz = 0.0F;
    return out;
  }
  return {};
}

// Rotates the planar velocity and force by -yaw (world components to body components).
inline ChassisSetpoint RotateToBody(const ChassisSetpoint & in, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  ChassisSetpoint out = in;
  out.vx = static_cast<float>(c * in.vx + s * in.vy);
  out.vy = static_cast<float>(-s * in.vx + c * in.vy);
  out.fx = static_cast<float>(c * in.fx + s * in.fy);
  out.fy = static_cast<float>(-s * in.fx + c * in.fy);
  return out;
}

}  // namespace ros2_libxr
