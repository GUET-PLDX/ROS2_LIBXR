// The package builds Release (-DNDEBUG); keep the checks active.
#undef NDEBUG
#include "ros2_libxr/board_clock.hpp"
#include "ros2_libxr/command_arbiter.hpp"
#include "ros2_libxr/wire.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>

int main()
{
  using namespace ros2_libxr::wire;
  assert(sizeof(ChassisTarget) == 45);
  assert(sizeof(BehaviorData) == 26);
  assert(sizeof(TeamInfo) == 48);
  assert(sizeof(GameInfo) == 22);
  assert(sizeof(OnlineInfo) == 37);
  assert(sizeof(OfflineInfo) == 16);
  assert(sizeof(RadarInfo) == 59);
  assert(std::fabs(DegreesToRadians(180.0F) - 3.14159265F) < 1e-5F);
  assert(DegreesToRadians(0.0F) == 0.0F);

  // Must match Pldx::DualBoardControl::ChassisFeedback in the MC02 DualBoard module.
  assert(sizeof(ChassisFeedback) == 26);
  assert(offsetof(ChassisFeedback, gyro_z) == 12);
  assert(offsetof(ChassisFeedback, sequence) == 16);
  assert(offsetof(ChassisFeedback, stamp_us) == 20);
  assert(offsetof(ChassisFeedback, mode) == 24);
  assert(offsetof(ChassisFeedback, flags) == 25);

  using ros2_libxr::BoardClockMapper;
  {
    // Jittered delay: the mapping converges to board time + minimum delay.
    BoardClockMapper clock;
    const int64_t host_offset = 1'000'000'000;
    const int64_t delays[] = {3000, 900, 2500, 1200, 800, 4000, 850};
    int64_t last = 0;
    for (int i = 0; i < 700; ++i) {
      const uint32_t board = 10'000U + static_cast<uint32_t>(i) * 4000U;
      const int64_t out = clock.map(board, host_offset + board + delays[i % 7]);
      assert(out > last);
      last = out;
      if (i > 7) {
        // Within the allowed drift creep of the minimum observed delay.
        assert(std::llabs(out - (host_offset + board + 800)) < 400);
      }
    }
  }
  {
    // The 32-bit counter wraps without a jump in host time.
    BoardClockMapper clock;
    const uint32_t start = 0xFFFFF000U;
    const int64_t a = clock.map(start, 5'000'000);
    const int64_t b = clock.map(start + 8192U, 5'008'192);
    assert(b - a == 8192);
  }
  {
    using ros2_libxr::Arbitrate;
    using ros2_libxr::ChassisSetpoint;
    using ros2_libxr::RotateToBody;
    using ros2_libxr::TimedSetpoint;
    const double timeout = 0.15;
    TimedSetpoint twist{{1.0F, 2.0F, 0.5F, 9.0F, 9.0F, 9.0F}, 10.0};
    TimedSetpoint chassis{{0.3F, -0.4F, 0.1F, 20.0F, -5.0F, 1.5F}, 10.05};
    // A fresh ChassisCommand is sent whole, never mixed with the Twist.
    ChassisSetpoint out = Arbitrate(twist, chassis, 10.1, timeout);
    assert(out.vx == 0.3F && out.vy == -0.4F && out.wz == 0.1F);
    assert(out.fx == 20.0F && out.fy == -5.0F && out.mz == 1.5F);
    // Stale ChassisCommand: velocity-only Twist, never with a force.
    out = Arbitrate(twist, chassis, 10.21, timeout);
    assert(out.vx == 0.0F && out.fx == 0.0F);  // the Twist is stale too
    twist.stamp = 10.2;
    out = Arbitrate(twist, chassis, 10.21, timeout);
    assert(out.vx == 1.0F && out.vy == 2.0F && out.wz == 0.5F);
    assert(out.fx == 0.0F && out.fy == 0.0F && out.mz == 0.0F);
    // Never received, or non-finite: zero.
    out = Arbitrate(TimedSetpoint{}, TimedSetpoint{}, 1.0, timeout);
    assert(out.vx == 0.0F && out.fx == 0.0F);
    chassis = {{0.3F, 0.0F, 0.0F, NAN, 0.0F, 0.0F}, 10.2};
    out = Arbitrate(twist, chassis, 10.21, timeout);
    assert(out.vx == 1.0F && out.fx == 0.0F);  // falls back to the Twist
    // The optional rotation turns velocity and force together; yaw rate and torque stay.
    const ChassisSetpoint world{1.0F, 0.0F, 0.2F, 10.0F, 0.0F, 3.0F};
    const ChassisSetpoint body = RotateToBody(world, M_PI / 2.0);
    assert(std::fabs(body.vx) < 1e-6F && std::fabs(body.vy + 1.0F) < 1e-6F);
    assert(std::fabs(body.fx) < 1e-5F && std::fabs(body.fy + 10.0F) < 1e-5F);
    assert(body.wz == 0.2F && body.mz == 3.0F);
  }
  {
    // A board reset (time goes backwards) restarts the mapping instead of jumping.
    BoardClockMapper clock;
    clock.map(50'000'000U, 60'000'000);
    const int64_t before = clock.map(50'004'000U, 60'004'000);
    const int64_t after = clock.map(1'000U, 60'008'000);
    assert(after > before);
    assert(std::llabs(after - 60'008'000) <= 1);
  }
}
