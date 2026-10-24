#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace ros2_libxr::wire {
#pragma pack(push, 1)
struct ChassisTarget
{
  float vx, vy, wz, yaw, cvx, cvy, cw, fx, fy, fw;
  bool speed;
  float delta_yaw;
};
struct BehaviorData
{
  uint8_t pitch, stance, lifter;
  float scan_min, scan_max;
  uint16_t ammo;
  uint8_t revive, remote_revive, remote_ammo, remote_health;
  bool limited, no_aim, capacitor, tunnel;
  float tunnel_angle;
  bool gyro;
};
struct Ally
{
  uint8_t id;
  uint16_t hp;
  float x, y;
};
struct TeamInfo
{
  Ally allies[4];
  uint16_t outpost, base;
};
struct GameInfo
{
  uint16_t remaining, coins;
  uint32_t events;
  uint8_t status;
  float target_x, target_y;
  uint8_t key;
  uint16_t enemy_outpost, enemy_base;
};
struct OnlineInfo
{
  uint16_t hp, bullets, cooling, heat_limit, heat;
  float x, y, angle;
  uint32_t info1;
  uint16_t info2;
  uint64_t info3;
  uint8_t energy;
};
struct OfflineInfo
{
  float camera_yaw;
  uint8_t lifter;
  bool transformable;
  float transform;
  uint8_t capacitor;
  float imu_yaw;
  bool aligned;
};
struct Enemy
{
  uint8_t id;
  uint16_t hp, ammo, x, y;
};
struct RadarInfo
{
  Enemy enemies[6];
  uint16_t coins, accumulated;
  bool sensed;
};
// Measured chassis velocity from the wheel forward kinematics, chassis body frame
// (same axes and signs as ChassisTarget vx/vy/wz). Mirrors
// Pldx::DualBoardControl::ChassisFeedback in the MC02 DualBoard module.
struct ChassisFeedback
{
  float vx, vy, wz;   // wheel odometry, m/s and rad/s
  float gyro_z;       // chassis IMU yaw rate, rad/s (valid when FEEDBACK_GYRO_VALID)
  uint32_t sequence;  // extended CAN sequence; gaps mean CAN/USB loss
  uint32_t stamp_us;  // gimbal-board microseconds at CAN reception, low 32 bits
  uint8_t mode;       // chassis mode wire value
  uint8_t flags;
};
#pragma pack(pop)
static_assert(sizeof(ChassisTarget) == 45 && sizeof(BehaviorData) == 26);
static_assert(sizeof(TeamInfo) == 48 && sizeof(GameInfo) == 22);
static_assert(sizeof(OnlineInfo) == 37 && sizeof(OfflineInfo) == 16 && sizeof(RadarInfo) == 59);
static_assert(sizeof(ChassisFeedback) == 26 && offsetof(ChassisFeedback, sequence) == 16 &&
              offsetof(ChassisFeedback, stamp_us) == 20 && offsetof(ChassisFeedback, flags) == 25);
constexpr uint8_t FEEDBACK_VALID = 0x01U;
constexpr uint8_t FEEDBACK_WHEELS_MASK = 0x1EU;  // bit1..4: wheel0..3 online
constexpr uint8_t FEEDBACK_LINK = 0x20U;
constexpr uint8_t FEEDBACK_GYRO_VALID = 0x40U;
constexpr float DegreesToRadians(float degrees)
{
  return degrees * 0.01745329251994329577F;
}
}  // namespace ros2_libxr::wire
