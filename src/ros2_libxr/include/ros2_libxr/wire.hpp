#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace ros2_libxr::wire {
#pragma pack(push, 1)
struct ChassisTarget
{
  float vx, vy, wz;
  bool speed;
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
#pragma pack(pop)
static_assert(sizeof(ChassisTarget) == 13 && sizeof(BehaviorData) == 26);
static_assert(sizeof(TeamInfo) == 48 && sizeof(GameInfo) == 22);
static_assert(sizeof(OnlineInfo) == 37 && sizeof(OfflineInfo) == 16 && sizeof(RadarInfo) == 59);
constexpr float DegreesToRadians(float degrees)
{
  return degrees * 0.01745329251994329577F;
}
}  // namespace ros2_libxr::wire
