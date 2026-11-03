// The package builds Release (-DNDEBUG); keep the checks active.
#undef NDEBUG
#include "message.hpp"
#include "ros2_libxr/velocity_command.hpp"
#include "ros2_libxr/wire.hpp"

#include <bit>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>

int main()
{
  using PacketHeader = LibXR::Topic::PackedDataHeader;
  static_assert(sizeof(PacketHeader) == 16);
  static_assert(LibXR::Topic::PACK_BASE_SIZE == 17);
  static_assert(offsetof(PacketHeader, version) == 14);
  static_assert(offsetof(PacketHeader, pack_header_crc8) == 15);
  static_assert(std::endian::native == std::endian::little);
  {
    PacketHeader header{};
    header.prefix = LibXR::Topic::PACKET_PREFIX;
    header.SetDataLen(0x123456U);
    header.topic_name_crc32 = 0x78563412U;
    header.SetTimestamp(LibXR::MicrosecondTimestamp(0x060504030201ULL));
    header.version = LibXR::Topic::PACKET_VERSION;
    assert(header.data_len_raw[0] == 0x56U);
    assert(header.data_len_raw[1] == 0x34U);
    assert(header.data_len_raw[2] == 0x12U);
    assert(header.timestamp_us_raw[0] == 0x01U);
    assert(header.timestamp_us_raw[5] == 0x06U);
    assert(header.GetDataLen() == 0x123456U);
    assert(static_cast<uint64_t>(header.GetTimestamp()) == 0x060504030201ULL);
    assert(header.version == 1U);
  }

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

  {
    using ros2_libxr::ChassisSetpoint;
    using ros2_libxr::FreshCommand;
    using ros2_libxr::TimedHeading;
    using ros2_libxr::TimedSetpoint;
    using ros2_libxr::WorldToBody;
    const TimedSetpoint chassis{{20.0F, -5.0F, 1.5F}, 10.0, 100.0};
    auto out = FreshCommand(chassis, 10.1, 100.1, 0.15);
    assert(out.vx == 20.0F && out.vy == -5.0F && out.wz == 1.5F);
    out = FreshCommand(chassis, 10.2, 100.1, 0.15);
    assert(out.vx == 0.0F && out.wz == 0.0F);
    // Paused ROS time must not keep an old velocity alive.
    out = FreshCommand(chassis, 10.1, 100.2, 0.15);
    assert(out.vx == 0.0F && out.wz == 0.0F);
    out = FreshCommand(chassis, 9.9, 100.1, 0.15);
    assert(out.vx == 0.0F);
    out = FreshCommand({{NAN, 0.0F, 0.0F}, 10.0, 100.0}, 10.1, 100.1, 0.15);
    assert(out.vx == 0.0F);
    out = FreshCommand({}, 10.0, 100.0, 0.15);
    assert(out.vx == 0.0F);
    ChassisTarget wire{};
    wire.vx = 1.25F;
    wire.wz = 3.0F;
    wire.speed = true;
    assert(wire.vx == 1.25F && wire.wz == 3.0F && wire.speed);
    assert(offsetof(ChassisTarget, wz) == 8 && offsetof(ChassisTarget, speed) == 40 &&
           offsetof(ChassisTarget, delta_yaw) == 41);

    const TimedSetpoint world{{1.0F, 0.0F, 0.5F}, 10.0, 100.0};
    const TimedHeading heading{M_PI / 2.0, 0.0, 10.0, 100.0};
    bool valid = false;
    const auto body = WorldToBody(world, heading, 10.1, 100.1, 0.15, 0.3, 0.0, 0.0, &valid);
    assert(valid);
    assert(std::fabs(body.vx) < 1e-5F && std::fabs(body.vy + 1.0F) < 1e-5F);
    assert(body.wz == 0.5F);
    assert(WorldToBody(world, {}, 10.1, 100.1, 0.15, 0.3, 0.0, 0.0).vx == 0.0F);
    const TimedSetpoint fresh{{1.0F, 0.0F, 0.5F}, 10.4, 100.4};
    const auto old_heading = WorldToBody(fresh, heading, 10.4, 100.4, 0.15, 0.3, 0.0, 0.0, &valid);
    assert(!valid && old_heading.vx == 0.0F && old_heading.wz == 0.0F);
    const TimedHeading future{0.0, 0.0, 10.5, 100.4};
    assert(WorldToBody(fresh, future, 10.4, 100.4, 0.15, 0.3, 0.0, 0.0).wz == 0.0F);
    const TimedHeading old_receipt{0.0, 0.0, 10.4, 100.0};
    assert(WorldToBody(fresh, old_receipt, 10.4, 100.4, 0.15, 0.3, 0.0, 0.0).wz == 0.0F);
    const TimedHeading invalid{NAN, 0.0, 10.4, 100.4};
    assert(WorldToBody(fresh, invalid, 10.4, 100.4, 0.15, 0.3, 0.0, 0.0).wz == 0.0F);
    const TimedHeading fresh_heading{0.0, 0.0, 10.4, 100.4};
    assert(WorldToBody(world, fresh_heading, 10.4, 100.4, 0.15, 0.3, 0.0, 0.0).wz == 0.0F);
    const TimedHeading rotating{0.0, 1.0, 10.4, 100.4};
    const auto compensated = WorldToBody(fresh, rotating, 10.4, 100.4, 0.15, 0.3, M_PI / 2.0, 0.0);
    assert(std::fabs(compensated.vx) < 1e-5F && std::fabs(compensated.vy + 1.0F) < 1e-5F);
    const TimedSetpoint stop{{0.0F, 0.0F, 0.0F}, 10.4, 100.4};
    WorldToBody(stop, fresh_heading, 10.4, 100.4, 0.15, 0.3, 0.0, 0.0, &valid);
    assert(valid);  // An intentional zero command is distinct from an invalid stream.
  }
}
