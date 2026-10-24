#include "ros2_libxr/board_clock.hpp"
#include "ros2_libxr/command_arbiter.hpp"
#include "ros2_libxr/wire.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/behavior.hpp>
#include <ros_interfaces/msg/chassis_command.hpp>
#include <ros_interfaces/msg/game_info.hpp>
#include <ros_interfaces/msg/radar_info.hpp>
#include <ros_interfaces/msg/sentry_info_offline.hpp>
#include <ros_interfaces/msg/sentry_info_online.hpp>
#include <ros_interfaces/msg/team_information.hpp>

#include "SharedTopic/SharedTopic.hpp"
#include "SharedTopicClient/SharedTopicClient.hpp"
#include "app_framework.hpp"
#include "linux_uart.hpp"
#include "message.hpp"
#include "thread.hpp"

namespace {
using namespace std::chrono_literals;
using namespace ros2_libxr::wire;
using ros2_libxr::Arbitrate;
using ros2_libxr::ChassisSetpoint;
using ros2_libxr::RotateToBody;

class NavigationLink final : public rclcpp::Node
{
public:
  NavigationLink() : Node("ros2_libxr")
  {
    const auto baud = declare_parameter<int>("baudrate", 921600);
    const auto vid = declare_parameter<std::string>("usb_vid", "16d0");
    const auto pid = declare_parameter<std::string>("usb_pid", "1492");
    const auto iface = declare_parameter<std::string>("control_interface", "XRUSB Navigation Control");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/cmd_vel_mpc");
    const auto behavior_topic = declare_parameter<std::string>("behavior_topic", "/sentry/behaivor_send");
    // Wheel odometry from the chassis board (body frame), for the controller's local integration.
    const auto feedback_topic =
      declare_parameter<std::string>("chassis_velocity_topic", "/sentry/chassis_velocity");
    feedback_frame_ = declare_parameter<std::string>("chassis_velocity_frame", "base_link");
    // "wheel": yaw rate from the wheel kinematics; "gyro": chassis IMU when valid, else wheel.
    gyro_yaw_rate_ = declare_parameter<std::string>("chassis_yaw_rate_source", "wheel") == "gyro";
    require_all_wheels_ = declare_parameter<bool>("chassis_velocity_require_all_wheels", true);
    // Sample age at CAN reception (chassis sampling + CAN transfer), subtracted from the stamp.
    sample_latency_us_ = declare_parameter<int>("chassis_velocity_latency_us", 1000);
    LibXR::PlatformInit();
    uart_ = std::make_unique<LibXR::LinuxUART>(
      vid, pid, iface, static_cast<unsigned int>(baud), LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);
    ramfs_ = std::make_unique<LibXR::RamFS>();
    terminal_ = std::make_unique<LibXR::Terminal<1024, 64, 16, 128>>(*ramfs_);
    terminal_thread_ = std::make_unique<LibXR::Thread>();
    terminal_thread_->Create(terminal_.get(),
      LibXR::Terminal<1024, 64, 16, 128>::ThreadFun,
      "terminal",
      81900,
      LibXR::Thread::Priority::MEDIUM);

    nav_ = LibXR::Topic::CreateTopic<ChassisTarget>("nav_data");
    behavior_ = LibXR::Topic::CreateTopic<BehaviorData>("behavior_data");
    team_ = LibXR::Topic::CreateTopic<TeamInfo>("team_info");
    game_ = LibXR::Topic::CreateTopic<GameInfo>("game_info");
    online_ = LibXR::Topic::CreateTopic<OnlineInfo>("online_info");
    offline_ = LibXR::Topic::CreateTopic<OfflineInfo>("offline_info");
    radar_ = LibXR::Topic::CreateTopic<RadarInfo>("radar_info");
    feedback_ = LibXR::Topic::CreateTopic<ChassisFeedback>("chassis_feedback");
    LibXR::HardwareContainer hw{LibXR::Entry<LibXR::RamFS>({*ramfs_, {"ramfs"}}),
      LibXR::Entry<LibXR::UART>({*uart_, {"uart_client"}})};
    static LibXR::ApplicationManager manager;
    static auto rx = std::make_unique<SharedTopic>(hw,
      manager,
      "uart_client",
      256,
      std::initializer_list<SharedTopic::TopicConfig>{{"team_info"},
        {"game_info"},
        {"online_info"},
        {"offline_info"},
        {"radar_info"},
        {"chassis_feedback"}});
    static auto tx = std::make_unique<SharedTopicClient>(hw,
      manager,
      "uart_client",
      16,
      std::initializer_list<SharedTopicClient::TopicConfig>{{"nav_data"}, {"behavior_data"}});

    // Velocity plus force feedforward from the controller; preferred over cmd_topic while fresh.
    const auto chassis_cmd_topic = declare_parameter<std::string>("chassis_cmd_topic", "/chassis_cmd_mpc");
    command_timeout_ = declare_parameter<double>("command_timeout", 0.15);
    // Default passthrough: commands are forwarded with the components the controller sent,
    // matching the behaviour verified on the robot. true rotates velocity and force together
    // from the controller's world frame to the chassis body frame with the LIO yaw.
    rotate_to_body_ = declare_parameter<bool>("rotate_world_to_body", false);
    rotate_latency_ = declare_parameter<double>("rotate_latency", 0.02);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        twist_cmd_.value = {static_cast<float>(msg->linear.x),
          static_cast<float>(msg->linear.y),
          static_cast<float>(msg->angular.z)};
        twist_cmd_.stamp = now().seconds();
      });
    chassis_cmd_sub_ = create_subscription<ros_interfaces::msg::ChassisCommand>(chassis_cmd_topic,
      rclcpp::SensorDataQoS(),
      [this](ros_interfaces::msg::ChassisCommand::ConstSharedPtr msg) {
        chassis_cmd_.value = {static_cast<float>(msg->twist.linear.x),
          static_cast<float>(msg->twist.linear.y),
          static_cast<float>(msg->twist.angular.z),
          static_cast<float>(msg->wrench_ff.force.x),
          static_cast<float>(msg->wrench_ff.force.y),
          static_cast<float>(msg->wrench_ff.torque.z)};
        chassis_cmd_.stamp = now().seconds();
      });
    if (rotate_to_body_) {
      const auto odom_topic = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          const auto & q = msg->pose.pose.orientation;
          odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
          odom_yaw_rate_ = msg->twist.twist.angular.z;
          odom_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).seconds();
        });
    }
    behavior_sub_ = create_subscription<ros_interfaces::msg::Behavior>(
      behavior_topic, 10, [this](ros_interfaces::msg::Behavior::ConstSharedPtr m) {
        BehaviorData d{};
        d.pitch = m->pitch_mode;
        d.stance = m->desired_stance;
        d.lifter = m->desire_lifter_pos;
        d.scan_min = DegreesToRadians(m->scan_yaw_min);
        d.scan_max = DegreesToRadians(m->scan_yaw_max);
        d.ammo = m->ammo_purchase_request;
        d.revive = m->revive_request;
        d.remote_revive = m->remote_revive_request;
        d.remote_ammo = m->remote_ammo_request;
        d.remote_health = m->remote_health_request;
        d.limited = m->use_limited_scan;
        d.no_aim = m->not_aim_enemy;
        d.capacitor = m->use_capacitor;
        d.tunnel = m->tunnel_align_active;
        d.tunnel_angle = DegreesToRadians(m->tunnel_align_angle_deg);
        d.gyro = m->use_gyro_mode;
        last_behavior_ = d;
      });
    team_pub_ = create_publisher<ros_interfaces::msg::TeamInformation>("/sentry/team_info", 10);
    game_pub_ = create_publisher<ros_interfaces::msg::GameInfo>("/sentry/game_info", 10);
    online_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 10);
    offline_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 10);
    radar_pub_ = create_publisher<ros_interfaces::msg::RadarInfo>("/sentry/radar_info", 10);
    feedback_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(feedback_topic, rclcpp::SensorDataQoS());
    Register<ChassisFeedback>(feedback_, [this](const ChassisFeedback & d) {
      onChassisFeedback(d);
    });
    Register<TeamInfo>(team_, [this](const TeamInfo & d) {
      ros_interfaces::msg::TeamInformation m;
      m.header.stamp = now();
      m.header.frame_id = "map";
      m.outpost_hp = d.outpost;
      m.base_hp = d.base;
      for (size_t i = 0; i < 4; i++) {
        m.allies[i].robot_id = d.allies[i].id;
        m.allies[i].robot_hp = d.allies[i].hp;
        m.allies[i].position.position.x = d.allies[i].x;
        m.allies[i].position.position.y = d.allies[i].y;
        m.allies[i].position.orientation.w = 1.;
      }
      team_pub_->publish(m);
    });
    Register<GameInfo>(game_, [this](const GameInfo & d) {
      ros_interfaces::msg::GameInfo m;
      m.header.stamp = now();
      m.header.frame_id = "map";
      m.game_time_remaining = d.remaining;
      m.coin_remaining = d.coins;
      m.event_code = d.events;
      m.game_status = d.status;
      m.manual_point_x = d.target_x;
      m.manual_point_y = d.target_y;
      m.manual_key = d.key;
      m.enemy_outpost_hp = d.enemy_outpost;
      m.enemy_base_hp = d.enemy_base;
      game_pub_->publish(m);
    });
    Register<OnlineInfo>(online_, [this](const OnlineInfo & d) {
      ros_interfaces::msg::SentryInfoOnline m;
      m.header.stamp = now();
      m.self_health = d.hp;
      m.bullets_remaining = d.bullets;
      m.cooling_value = d.cooling;
      m.heat_limit = d.heat_limit;
      m.current_heat = d.heat;
      m.sentry_pos.x = d.x;
      m.sentry_pos.y = d.y;
      m.speed_monitor_angle = d.angle;
      m.sentry_info_1 = d.info1;
      m.sentry_info_2 = d.info2;
      m.sentry_info_3 = d.info3;
      m.energy_ratio = d.energy;
      online_pub_->publish(m);
    });
    Register<OfflineInfo>(offline_, [this](const OfflineInfo & d) {
      ros_interfaces::msg::SentryInfoOffline m;
      m.header.stamp = now();
      m.yaw_camerainit_to_gimbal = d.camera_yaw;
      m.lifter_current_pos = d.lifter;
      m.is_transformable = d.transformable;
      m.transform_state = d.transform;
      m.capacitor_capacity = d.capacitor;
      m.chassis_imu_yaw = d.imu_yaw;
      m.tunnel_yaw_aligned = d.aligned;
      offline_pub_->publish(m);
    });
    Register<RadarInfo>(radar_, [this](const RadarInfo & d) {
      ros_interfaces::msg::RadarInfo m;
      m.header.stamp = now();
      m.enemy_coin_left = d.coins;
      m.enemy_coin_accumulated = d.accumulated;
      m.is_enemy_outpost_sensed = d.sensed;
      for (size_t i = 0; i < 6; i++) {
        m.enemies[i].robot_id = d.enemies[i].id;
        m.enemies[i].robot_hp = d.enemies[i].hp;
        m.enemies[i].allowed_projectile = d.enemies[i].ammo;
        m.enemies[i].position.position.x = d.enemies[i].x;
        m.enemies[i].position.position.y = d.enemies[i].y;
        m.enemies[i].position.orientation.w = 1.;
      }
      radar_pub_->publish(m);
    });
    timer_ = create_wall_timer(20ms, [this] {
      const double t = now().seconds();
      ChassisSetpoint cmd = Arbitrate(twist_cmd_, chassis_cmd_, t, command_timeout_);
      if (rotate_to_body_) {
        const double age = t - odom_stamp_;
        if (odom_stamp_ < 0.0 || age < 0.0 || age > 0.3) {
          // No current heading: a rotation would point the command anywhere.
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "rotate_world_to_body: odometry stale; sending zero");
          cmd = {};
        } else {
          // Heading when the command takes effect: odometry age plus the link latency.
          cmd = RotateToBody(cmd, odom_yaw_ + odom_yaw_rate_ * (age + rotate_latency_));
        }
      }
      ChassisTarget d{};
      d.vx = cmd.vx;
      d.vy = cmd.vy;
      d.wz = cmd.wz;
      // Drive wrench in the same frame as vx/vy (the firmware fields keep the legacy name fx_global).
      d.fx = cmd.fx;
      d.fy = cmd.fy;
      d.fw = cmd.mz;
      d.speed = true;
      nav_.Publish(d);
      BehaviorData b = last_behavior_;
      behavior_.Publish(b);
    });
    RCLCPP_INFO(get_logger(),
      "USB navigation link configured: %s:%ld baud, control interface '%s'",
      cmd_topic.c_str(),
      static_cast<long>(baud),
      iface.c_str());
  }

private:
  void onChassisFeedback(const ChassisFeedback & d)
  {
    const rclcpp::Time host_now = now();
    if (has_feedback_sequence_ && d.sequence != last_feedback_sequence_ + 1U &&
        d.sequence != last_feedback_sequence_) {
      feedback_gaps_ += d.sequence - last_feedback_sequence_ - 1U;
      RCLCPP_WARN_THROTTLE(get_logger(),
        *get_clock(),
        5000,
        "chassis_feedback sequence gap: %u -> %u (%lu frames lost in total)",
        last_feedback_sequence_,
        d.sequence,
        static_cast<unsigned long>(feedback_gaps_));
    }
    const bool duplicate = has_feedback_sequence_ && d.sequence == last_feedback_sequence_;
    last_feedback_sequence_ = d.sequence;
    has_feedback_sequence_ = true;
    const bool wheels_ok = !require_all_wheels_ || (d.flags & FEEDBACK_WHEELS_MASK) == FEEDBACK_WHEELS_MASK;
    // Invalid, partial or duplicate samples are not published: the controller times out
    // and falls back to LIO alone.
    if (duplicate || !(d.flags & FEEDBACK_VALID) || !(d.flags & FEEDBACK_LINK) || !wheels_ok ||
        !std::isfinite(d.vx) || !std::isfinite(d.vy) || !std::isfinite(d.wz)) {
      return;
    }
    const int64_t host_us = host_now.nanoseconds() / 1000;
    const int64_t stamp_us = clock_.map(d.stamp_us, host_us) - sample_latency_us_;
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = rclcpp::Time(stamp_us * 1000, host_now.get_clock_type());
    m.header.frame_id = feedback_frame_;
    m.twist.linear.x = d.vx;
    m.twist.linear.y = d.vy;
    const bool use_gyro = gyro_yaw_rate_ && (d.flags & FEEDBACK_GYRO_VALID) && std::isfinite(d.gyro_z);
    m.twist.angular.z = use_gyro ? d.gyro_z : d.wz;
    feedback_pub_->publish(m);
  }

  template <class T> void Register(LibXR::Topic & topic, std::function<void(const T &)> fn)
  {
    using Bound = std::function<void(const T &)>;
    auto cb = LibXR::Topic::Callback::Create(
      [](bool, Bound handler, T & value) {
        handler(value);
      },
      std::move(fn));
    topic.RegisterCallback(cb);
    callbacks_.push_back(std::move(cb));
  }
  std::unique_ptr<LibXR::LinuxUART> uart_;
  std::unique_ptr<LibXR::RamFS> ramfs_;
  std::unique_ptr<LibXR::Terminal<1024, 64, 16, 128>> terminal_;
  std::unique_ptr<LibXR::Thread> terminal_thread_;
  LibXR::Topic nav_, behavior_, team_, game_, online_, offline_, radar_, feedback_;
  std::vector<LibXR::Topic::Callback> callbacks_;
  ros2_libxr::TimedSetpoint twist_cmd_;
  ros2_libxr::TimedSetpoint chassis_cmd_;
  double command_timeout_{0.15};
  bool rotate_to_body_{false};
  double rotate_latency_{0.02};
  double odom_yaw_{0.0};
  double odom_yaw_rate_{0.0};
  double odom_stamp_{-1.0};
  rclcpp::Subscription<ros_interfaces::msg::ChassisCommand>::SharedPtr chassis_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  BehaviorData last_behavior_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<ros_interfaces::msg::Behavior>::SharedPtr behavior_sub_;
  rclcpp::Publisher<ros_interfaces::msg::TeamInformation>::SharedPtr team_pub_;
  rclcpp::Publisher<ros_interfaces::msg::GameInfo>::SharedPtr game_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOnline>::SharedPtr online_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_pub_;
  rclcpp::Publisher<ros_interfaces::msg::RadarInfo>::SharedPtr radar_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr feedback_pub_;
  std::string feedback_frame_;
  bool gyro_yaw_rate_{false};
  bool require_all_wheels_{true};
  int64_t sample_latency_us_{1000};
  ros2_libxr::BoardClockMapper clock_;  // used only from the SharedTopic callback
  uint32_t last_feedback_sequence_{0};
  bool has_feedback_sequence_{false};
  uint64_t feedback_gaps_{0};
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationLink>());
  rclcpp::shutdown();
  return 0;
}
