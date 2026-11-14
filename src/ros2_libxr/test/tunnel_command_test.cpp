#include "ros2_libxr/navigation_link.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_interfaces/msg/behavior.hpp>
#include <thread>

namespace {
using namespace std::chrono_literals;

class TunnelCommandTest : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    bridge_ = ros2_libxr::createNavigationLink([this](const auto & command, const auto & behavior) {
      command_ = command;
      wire_behavior_ = behavior;
      ++outputs_;
    });
    input_ = std::make_shared<rclcpp::Node>("tunnel_command_input");
    velocity_ =
      input_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_mpc", rclcpp::SensorDataQoS());
    odometry_ =
      input_->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", rclcpp::SensorDataQoS());
    behavior_ = input_->create_publisher<ros_interfaces::msg::Behavior>("/sentry/behaivor_send", 10);
    executor_.add_node(bridge_);
    executor_.add_node(input_);
    ASSERT_TRUE(waitFor([this] {
      return velocity_->get_subscription_count() && odometry_->get_subscription_count() &&
             behavior_->get_subscription_count();
    }));
    velocity_message_.linear.x = 1.0;
    velocity_message_.linear.y = -0.4;
    velocity_message_.angular.z = 0.5;
  }

  void TearDown() override
  {
    executor_.remove_node(bridge_);
    executor_.remove_node(input_);
    bridge_.reset();
    input_.reset();
  }

  bool waitFor(const std::function<bool()> & predicate, bool publish = false)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (publish) {
        publishInputs();
      }
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  void publishInputs()
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = input_->now();
    odom.header.frame_id = "camera_init";
    odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    if (publish_odometry_) {
      odometry_->publish(odom);
    }
    if (publish_velocity_) {
      velocity_->publish(velocity_message_);
    }
    behavior_->publish(behavior_message_);
  }

  bool matches(float vx, float vy, float wz = 0.5F) const
  {
    return command_.speed && std::abs(command_.vx - vx) < 1e-5F && std::abs(command_.vy - vy) < 1e-5F &&
           std::abs(command_.wz - wz) < 1e-5F;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<rclcpp::Node> bridge_, input_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Publisher<ros_interfaces::msg::Behavior>::SharedPtr behavior_;
  geometry_msgs::msg::Twist velocity_message_;
  ros_interfaces::msg::Behavior behavior_message_;
  ros2_libxr::wire::ChassisTarget command_{};
  ros2_libxr::wire::BehaviorData wire_behavior_{};
  int outputs_{0};
  double yaw_{0.0};
  bool publish_velocity_{true}, publish_odometry_{true};
};

TEST_F(TunnelCommandTest, PreparationSlowsFinalCommandAndReadinessRestoresIt)
{
  ASSERT_TRUE(waitFor(
    [this] {
      return matches(1.0F, -0.4F);
    },
    true));
  behavior_message_.tunnel_prepare_active = true;
  ASSERT_TRUE(waitFor(
    [this] {
      return matches(0.15F, -0.06F);
    },
    true));
  behavior_message_.tunnel_ready = true;
  ASSERT_TRUE(waitFor(
    [this] {
      return matches(1.0F, -0.4F);
    },
    true));
}
TEST_F(TunnelCommandTest, EscapeReplacesNavigationBeforePreparationSlowdown)
{
  yaw_ = M_PI / 2.0;
  behavior_message_.tunnel_escape_active = true;
  behavior_message_.tunnel_escape_vx = -0.4F;
  behavior_message_.tunnel_escape_vy = 0.2F;
  for (const bool prepare : {false, true}) {
    for (const bool ready : {false, true}) {
      behavior_message_.tunnel_prepare_active = prepare;
      behavior_message_.tunnel_ready = ready;
      const float x = prepare && !ready ? 0.03F : 0.2F;
      const float y = prepare && !ready ? 0.06F : 0.4F;
      ASSERT_TRUE(waitFor(
        [this, x, y] {
          return matches(x, y);
        },
        true));
      const int baseline = outputs_;
      ASSERT_TRUE(waitFor(
        [this, baseline] {
          return outputs_ >= baseline + 25;
        },
        true));
      EXPECT_TRUE(matches(x, y));
    }
  }
  behavior_message_.tunnel_escape_active = false;
  ASSERT_TRUE(waitFor(
    [this] {
      return matches(-0.4F, -1.0F);
    },
    true));
}

TEST_F(TunnelCommandTest, PreparationTruthTablePreservesYawAndDoesNotAccumulateScaling)
{
  behavior_message_.tunnel_align_active = true;
  behavior_message_.tunnel_align_angle_deg = 90.0F;
  behavior_message_.use_gyro_mode = true;
  for (const bool prepare : {false, true}) {
    for (const bool ready : {false, true}) {
      behavior_message_.tunnel_prepare_active = prepare;
      behavior_message_.tunnel_ready = ready;
      const float x = prepare && !ready ? 0.15F : 1.0F;
      const float y = prepare && !ready ? -0.06F : -0.4F;
      ASSERT_TRUE(waitFor(
        [this, x, y] {
          return matches(x, y);
        },
        true));
      const int baseline = outputs_;
      ASSERT_TRUE(waitFor(
        [this, baseline] {
          return outputs_ >= baseline + 25;
        },
        true));
      EXPECT_TRUE(matches(x, y));
      EXPECT_TRUE(wire_behavior_.tunnel);
      EXPECT_TRUE(wire_behavior_.gyro);
      EXPECT_NEAR(wire_behavior_.tunnel_angle, 1.57079633F, 1e-5F);
    }
  }
  behavior_message_.tunnel_ready = false;
  velocity_message_ = geometry_msgs::msg::Twist{};
  ASSERT_TRUE(waitFor(
    [this] {
      return matches(0.0F, 0.0F, 0.0F);
    },
    true));
}

TEST_F(TunnelCommandTest, OverridesCannotReviveInvalidNavigationOrOdometry)
{
  behavior_message_.tunnel_escape_active = true;
  behavior_message_.tunnel_escape_vx = -0.4F;
  behavior_message_.tunnel_escape_vy = 0.2F;
  behavior_message_.tunnel_prepare_active = true;
  const auto stopped = [this] {
    return !command_.speed && command_.vx == 0.0F && command_.vy == 0.0F && command_.wz == 0.0F;
  };
  for (const double invalid : {static_cast<double>(NAN), static_cast<double>(INFINITY), 100.0}) {
    velocity_message_.linear.x = 1.0;
    ASSERT_TRUE(waitFor(
      [this] {
        return matches(-0.06F, 0.03F);
      },
      true));
    velocity_message_.linear.x = invalid;
    ASSERT_TRUE(waitFor(stopped, true));
  }
  velocity_message_.linear.x = 1.0;
  ASSERT_TRUE(waitFor(
    [this] {
      return matches(-0.06F, 0.03F);
    },
    true));
  publish_velocity_ = false;
  ASSERT_TRUE(waitFor(stopped, true));
  publish_velocity_ = true;
  ASSERT_TRUE(waitFor(
    [this] {
      return matches(-0.06F, 0.03F);
    },
    true));
  publish_odometry_ = false;
  ASSERT_TRUE(waitFor(stopped, true));
  publish_odometry_ = true;
  ASSERT_TRUE(waitFor(
    [this] {
      return matches(-0.06F, 0.03F);
    },
    true));
  behavior_message_.tunnel_escape_vx = NAN;
  ASSERT_TRUE(waitFor(stopped, true));
}
}  // namespace
