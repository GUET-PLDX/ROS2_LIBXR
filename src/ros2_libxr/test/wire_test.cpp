// The package builds Release (-DNDEBUG); keep the checks active.
#undef NDEBUG
#include "ros2_libxr/board_clock.hpp"
#include "ros2_libxr/force_command.hpp"
#include "ros2_libxr/wire.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>

int main()
{
  using namespace ros2_libxr::wire;
  assert(sizeof(ChassisTarget) == 12);
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
    using ros2_libxr::ChassisSetpoint;
    using ros2_libxr::FreshCommand;
    using ros2_libxr::RotateToBody;
    using ros2_libxr::TimedSetpoint;
    const TimedSetpoint chassis{{20.0F, -5.0F, 1.5F}, 10.0, 100.0};
    auto out = FreshCommand(chassis, 10.1, 100.1, 0.15);
    assert(out.fx == 20.0F && out.fy == -5.0F && out.mz == 1.5F);
    out = FreshCommand(chassis, 10.2, 100.1, 0.15);
    assert(out.fx == 0.0F && out.mz == 0.0F);
    // Paused ROS time must not keep an old force alive.
    out = FreshCommand(chassis, 10.1, 100.2, 0.15);
    assert(out.fx == 0.0F && out.mz == 0.0F);
    out = FreshCommand(chassis, 9.9, 100.1, 0.15);
    assert(out.fx == 0.0F);
    out = FreshCommand({{NAN, 0.0F, 0.0F}, 10.0, 100.0}, 10.1, 100.1, 0.15);
    assert(out.fx == 0.0F);
    out = FreshCommand({}, 10.0, 100.0, 0.15);
    assert(out.fx == 0.0F);
    const auto body = RotateToBody({10.0F, 0.0F, 3.0F}, M_PI / 2.0);
    assert(std::fabs(body.fx) < 1e-5F && std::fabs(body.fy + 10.0F) < 1e-5F);
    assert(body.mz == 3.0F);
    // The body-frame USB payload contains exactly this wrench.
    const ChassisTarget wire{body.fx, body.fy, body.mz};
    assert(wire.mz == 3.0F && offsetof(ChassisTarget, mz) == 8);
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
