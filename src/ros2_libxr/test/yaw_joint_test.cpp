// The package builds Release (-DNDEBUG); keep the checks active.
#undef NDEBUG
#include "message.hpp"
#include "ros2_libxr/yaw_joint.hpp"
#include "server.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

using namespace ros2_libxr;

namespace {
struct Received
{
  YawJointSample sample;
  uint64_t packet_stamp_us;
};

// Replays MCU packets through the same LibXR packet parser, timestamped callback,
// decoder and clock map that the ROS node uses. Topics live for the whole process,
// so one callback is registered and routed to the replay under test.
class Replay
{
public:
  Replay()
  {
    Wiring & w = Wire();
    w.active = this;
  }
  ~Replay() { Wire().active = nullptr; }

  // MCU side: the payload and 48-bit header stamp it sends. Host side: arrival on the ROS clock.
  void Send(float angle, float velocity, uint64_t mcu_us, int64_t receive_ns)
  {
    Wiring & w = Wire();
    const wire::YawJointFeedback payload{angle, velocity};
    std::array<uint8_t, LibXR::Topic::PACK_BASE_SIZE + sizeof(payload)> packet{};
    LibXR::Topic::PackData(w.topic.GetKey(),
      LibXR::RawData(packet.data(), packet.size()),
      LibXR::MicrosecondTimestamp(mcu_us),
      LibXR::ConstRawData(&payload, sizeof(payload)));
    receive_ns_ = receive_ns;
    w.server.ParseData(LibXR::ConstRawData(packet.data(), packet.size()));
  }

  std::vector<Received> out_;
  SourceClockMap clock_;
  int64_t receive_ns_{0};

private:
  struct Wiring
  {
    LibXR::Topic topic{LibXR::Topic::CreateTopic<wire::YawJointFeedback>(kYawJointTopic)};
    LibXR::Topic::Server server{256};
    LibXR::Topic::Callback callback;
    Replay * active{nullptr};
    Wiring()
    {
      server.Register(topic);
      callback = LibXR::Topic::Callback::Create(
        [](bool, Wiring * self, LibXR::MicrosecondTimestamp stamp, const LibXR::ConstRawData & data) {
          Replay * r = self->active;
          YawJointSample sample;
          if (r &&
              DecodeYawJoint(
                data.addr_, data.size_, static_cast<uint64_t>(stamp), r->receive_ns_, r->clock_, sample)) {
            r->out_.push_back({sample, static_cast<uint64_t>(stamp)});
          }
        },
        this);
      topic.RegisterCallback(callback);
    }
  };
  static Wiring & Wire()
  {
    static Wiring wiring;
    return wiring;
  }
};

bool Near(double a, double b, double tol)
{
  return std::abs(a - b) <= tol;
}
}  // namespace

int main()
{
  // Packet layout shared with the firmware test: payload {1.5f, -0.25f}.
  static_assert(sizeof(wire::YawJointFeedback) == 8);
  {
    Replay replay;
    replay.Send(1.5F, -0.25F, 0x060504030201ULL, 5'000'000'000LL);
    assert(replay.out_.size() == 1);
    const auto & r = replay.out_[0];
    assert(r.packet_stamp_us == 0x060504030201ULL);
    assert(r.sample.mcu_stamp_us == 0x060504030201ULL);
    assert(Near(r.sample.angle, 1.5, 1e-7) && Near(r.sample.velocity, -0.25, 1e-7));
  }

  constexpr int64_t kRosAtMcuZero = 1'700'000'000'000'000'000LL;  // ROS ns when the MCU counter was 0
  constexpr int64_t kMinDelay = 2'000'000;                        // 2 ms transport floor
  {
    // 3 s of 1 kHz samples with a known angle ramp and 2-5 ms jittered transport.
    Replay replay;
    const double omega = 0.8;  // rad/s
    for (int k = 0; k < 3000; ++k) {
      const uint64_t mcu_us = 123'000'000ULL + static_cast<uint64_t>(k) * 1000ULL;
      const int64_t delay = kMinDelay + (k % 7 == 3 ? 3'000'000 : (k % 5) * 500'000);
      replay.Send(static_cast<float>(0.1 + omega * k * 1e-3),
        static_cast<float>(omega),
        mcu_us,
        kRosAtMcuZero + static_cast<int64_t>(mcu_us) * 1000 + delay);
    }
    assert(replay.out_.size() == 3000);
    for (size_t k = 0; k < replay.out_.size(); ++k) {
      const auto & s = replay.out_[k].sample;
      const int64_t truth = kRosAtMcuZero + static_cast<int64_t>(s.mcu_stamp_us) * 1000;
      // Never later than receipt, and off from the true measurement time by at most the
      // floor of the transport delay, not by each packet's own (jittered) delay.
      assert(s.stamp_ns <= s.receive_ns);
      assert(s.stamp_ns - truth >= 0 && s.stamp_ns - truth <= kMinDelay);
      assert(Near(s.angle, 0.1 + omega * static_cast<double>(k) * 1e-3, 1e-5));
      assert(Near(s.velocity, omega, 1e-6));
    }
    // After the first sample the estimate has locked on; late packets do not move it.
    assert(replay.out_[2999].sample.stamp_ns - replay.out_[0].sample.stamp_ns == 2'999'000'000LL);
  }
  {
    // A single 60 ms late packet must not pull the stamp later: receive time is not the sample time.
    Replay replay;
    for (int k = 0; k < 100; ++k) {
      const uint64_t mcu_us = 10'000'000ULL + static_cast<uint64_t>(k) * 1000ULL;
      const int64_t delay = (k == 80) ? 60'000'000 : kMinDelay;
      replay.Send(0.0F, 0.0F, mcu_us, kRosAtMcuZero + static_cast<int64_t>(mcu_us) * 1000 + delay);
    }
    const auto & late = replay.out_[80].sample;
    assert(late.receive_ns - late.stamp_ns == 60'000'000 - kMinDelay);
    assert(late.stamp_ns - (kRosAtMcuZero + static_cast<int64_t>(late.mcu_stamp_us) * 1000) == kMinDelay);
  }
  {
    // MCU time offset: the same stream shifted by 1000 s on the MCU side maps identically.
    std::vector<int64_t> stamps[2];
    const uint64_t mcu_base[2]{1'000'000ULL, 1'001'000'000ULL};
    for (int run = 0; run < 2; ++run) {
      Replay replay;
      for (int k = 0; k < 10; ++k) {
        replay.Send(
          0.0F, 0.0F, mcu_base[run] + static_cast<uint64_t>(k) * 1000ULL, kRosAtMcuZero + k * 1'000'000LL);
      }
      for (const auto & r : replay.out_) {
        stamps[run].push_back(r.sample.stamp_ns);
      }
    }
    assert(stamps[0].size() == 10 && stamps[0] == stamps[1]);
  }
  {
    // 48-bit counter boundary: stamps continue across the wrap instead of jumping back 2^48 us.
    Replay replay;
    const uint64_t modulus = 1ULL << 48U;
    const uint64_t raw[]{modulus - 2000, modulus - 1000, 500, 1500};
    const int64_t truth_us[]{0, 1000, 2500, 3500};
    for (int k = 0; k < 4; ++k) {
      replay.Send(0.0F, 0.0F, raw[k], kRosAtMcuZero + truth_us[k] * 1000 + kMinDelay);
    }
    assert(replay.out_.size() == 4);
    for (int k = 0; k < 4; ++k) {
      assert(replay.out_[k].sample.stamp_ns - replay.out_[0].sample.stamp_ns == truth_us[k] * 1000);
    }
    assert(replay.out_[2].sample.mcu_stamp_us == 500);
  }
  {
    // The MCU restarts: a counter that goes back by seconds starts a new mapping.
    Replay replay;
    replay.Send(0.0F, 0.0F, 50'000'000ULL, kRosAtMcuZero);
    replay.Send(0.0F, 0.0F, 50'001'000ULL, kRosAtMcuZero + 1'000'000LL);
    const int64_t reboot_rx = kRosAtMcuZero + 20'000'000'000LL;
    replay.Send(0.0F, 0.0F, 3'000ULL, reboot_rx);
    assert(replay.out_.size() == 3);
    assert(replay.out_[2].sample.stamp_ns == reboot_rx);
  }
  {
    // Joint angle crossing +-pi stays continuous modulo 2*pi and lands in [-pi, pi).
    Replay replay;
    const float angles[]{3.10F, 3.14F, 3.15F, 3.20F, -3.14F};
    uint64_t mcu_us = 1'000'000ULL;
    for (float angle : angles) {
      replay.Send(angle, 0.5F, mcu_us, kRosAtMcuZero + static_cast<int64_t>(mcu_us) * 1000);
      mcu_us += 1000;
    }
    assert(replay.out_.size() == 5);
    for (const auto & r : replay.out_) {
      assert(r.sample.angle >= -std::numbers::pi && r.sample.angle < std::numbers::pi);
    }
    const double two_pi = 2.0 * std::numbers::pi;
    assert(Near(replay.out_[2].sample.angle, 3.15 - two_pi, 1e-5));
    assert(Near(replay.out_[3].sample.angle, 3.20 - two_pi, 1e-5));
    assert(Near(replay.out_[4].sample.angle, -3.14, 1e-5));
    assert(Near(WrapAngle(std::numbers::pi), -std::numbers::pi, 1e-12));
    assert(Near(WrapAngle(-std::numbers::pi), -std::numbers::pi, 1e-12));
    assert(Near(WrapAngle(7.0 * std::numbers::pi / 2.0), -std::numbers::pi / 2.0, 1e-12));
  }
  {
    // Rejected input never reaches a message.
    SourceClockMap clock;
    YawJointSample sample;
    const wire::YawJointFeedback ok{1.0F, 1.0F};
    assert(DecodeYawJoint(&ok, sizeof(ok), 10, 0, clock, sample));
    assert(!DecodeYawJoint(&ok, sizeof(ok) - 1, 10, 0, clock, sample));
    assert(!DecodeYawJoint(&ok, sizeof(ok), 0, 0, clock, sample));
    const wire::YawJointFeedback nan_angle{std::numeric_limits<float>::quiet_NaN(), 0.0F};
    const wire::YawJointFeedback inf_rate{0.0F, std::numeric_limits<float>::infinity()};
    assert(!DecodeYawJoint(&nan_angle, sizeof(nan_angle), 10, 0, clock, sample));
    assert(!DecodeYawJoint(&inf_rate, sizeof(inf_rate), 10, 0, clock, sample));
  }
  return 0;
}
