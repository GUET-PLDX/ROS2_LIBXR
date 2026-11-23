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
#include <vector>

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
  assert(sizeof(ChassisTarget) == 13);
  assert(sizeof(BehaviorData) == 26);
  assert(sizeof(TeamInfo) == 48);
  assert(sizeof(GameInfo) == 30);
  assert(sizeof(OnlineInfo) == 37);
  {
    // Independent wire bytes: two event words, all former fields, then controls.
    const uint8_t bytes[]{0x34,
      0x12,
      0x78,
      0x56,
      0xef,
      0xcd,
      0xab,
      0x89,
      0x67,
      0x45,
      0x23,
      0x01,
      0x04,
      0,
      0,
      0x80,
      0x3f,
      0,
      0,
      0,
      0x40,
      0x41,
      0x22,
      0x11,
      0x44,
      0x33,
      0x66,
      0x55,
      0x88,
      0x77};
    GameInfo game{};
    assert(DecodePayload(bytes, sizeof(bytes), game));
    assert(game.remaining == 0x1234 && game.coins == 0x5678);
    assert(game.events == 0x0123456789abcdefULL && game.status == 4);
    assert(game.target_x == 1.0F && game.target_y == 2.0F && game.key == 0x41);
    assert(game.enemy_outpost == 0x1122 && game.enemy_base == 0x3344);
    assert(game.ally_control_degree == 0x5566 && game.enemy_control_degree == 0x7788);
    assert(!DecodePayload(bytes, 22, game));
    assert(!DecodePayload(bytes, 31, game));
    assert(game.events == 0x0123456789abcdefULL);
    struct Received
    {
      size_t valid = 0;
      GameInfo game{};
    } received;
    LibXR::Topic incoming("referee_game_wire", 256);
    auto callback = LibXR::Topic::Callback::Create(
      [](bool, Received * target, const LibXR::ConstRawData & raw) {
        if (DecodePayload(raw.addr_, raw.size_, target->game))
          ++target->valid;
      },
      &received);
    incoming.RegisterCallback(callback);
    LibXR::Topic::Server server(256);
    server.Register(LibXR::Topic::Find("referee_game_wire"));
    for (const size_t length : {22U, 31U, 30U}) {
      std::vector<uint8_t> payload(bytes, bytes + sizeof(bytes));
      payload.resize(length, 0x5a);
      std::vector<uint8_t> packet(LibXR::Topic::PACK_BASE_SIZE + length);
      LibXR::Topic::PackData(LibXR::CRC32::Calculate("referee_game_wire", 17),
        {packet.data(), packet.size()},
        LibXR::MicrosecondTimestamp(123),
        {payload.data(), payload.size()});
      // Arbitrary stream chunks still go through LibXR's actual packet server.
      server.ParseData({packet.data(), 9});
      server.ParseData({packet.data() + 9, packet.size() - 9});
      assert(received.valid == (length == 30 ? 1U : 0U));
    }
    assert(received.game.events == 0x0123456789abcdefULL);
    assert(received.game.enemy_control_degree == 0x7788);
  }
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
    assert(offsetof(ChassisTarget, wz) == 8 && offsetof(ChassisTarget, speed) == 12);

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
