// The package builds Release (-DNDEBUG); keep the checks active.
#undef NDEBUG
#include "ros2_libxr/force_command.hpp"
#include "ros2_libxr/wire.hpp"
#include "message.hpp"

#include <cassert>
#include <bit>
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
  assert(sizeof(ChassisTarget) == 12);
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

}
