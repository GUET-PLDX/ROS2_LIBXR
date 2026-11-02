#include "ros2_libxr/force_command.hpp"
#include "ros2_libxr/wire.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
using ros2_libxr::ChassisSetpoint;
using ros2_libxr::FreshCommand;
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
    const auto behavior_topic = declare_parameter<std::string>("behavior_topic", "/sentry/behaivor_send");
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
        {"radar_info"}});
    static auto tx = std::make_unique<SharedTopicClient>(hw,
      manager,
      "uart_client",
      16,
      std::initializer_list<SharedTopicClient::TopicConfig>{{"nav_data"}, {"behavior_data"}});

    const auto chassis_cmd_topic = declare_parameter<std::string>("chassis_cmd_topic", "/chassis_cmd_mpc");
    command_timeout_ = declare_parameter<double>("command_timeout", 0.15);
    rotate_latency_ = declare_parameter<double>("rotate_latency", 0.02);
    chassis_yaw_offset_ = declare_parameter<double>("chassis_yaw_offset", 0.0);
    if (!std::isfinite(command_timeout_) || command_timeout_ <= 0.0 || !std::isfinite(rotate_latency_) ||
        rotate_latency_ < 0.0 || !std::isfinite(chassis_yaw_offset_)) {
      throw std::invalid_argument("Force command timeout, latency and yaw offset must be finite");
    }
    chassis_cmd_sub_ = create_subscription<ros_interfaces::msg::ChassisCommand>(chassis_cmd_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [this](ros_interfaces::msg::ChassisCommand::ConstSharedPtr msg) {
        const auto & w = msg->wrench;
        if (w.force.z != 0.0 || w.torque.x != 0.0 || w.torque.y != 0.0 || msg->header.frame_id.empty()) {
          chassis_cmd_ = {};
          return;
        }
        chassis_cmd_.value = {
          static_cast<float>(w.force.x), static_cast<float>(w.force.y), static_cast<float>(w.torque.z)};
        chassis_cmd_.stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).seconds();
        chassis_cmd_.received = steadySeconds();
        command_frame_ = msg->header.frame_id;
      });
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & q = msg->pose.pose.orientation;
        odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        odom_yaw_rate_ = msg->twist.twist.angular.z;
        odom_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).seconds();
        odom_received_ = steadySeconds();
        odom_frame_ = msg->header.frame_id;
      });
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
    timer_ = create_wall_timer(10ms, [this] {
      const double t = now().seconds();
      const double steady_now = steadySeconds();
      ChassisSetpoint cmd = FreshCommand(chassis_cmd_, t, steady_now, command_timeout_);
      const double age = t - odom_stamp_;
      if (odom_frame_ != command_frame_ || odom_frame_.empty() || odom_stamp_ < 0.0 || age < 0.0 ||
          age > 0.3 || steady_now - odom_received_ > 0.3 || !std::isfinite(odom_yaw_) ||
          !std::isfinite(odom_yaw_rate_)) {
        cmd = {};
      } else {
        cmd = RotateToBody(cmd, odom_yaw_ + chassis_yaw_offset_ + odom_yaw_rate_ * (age + rotate_latency_));
      }
      ChassisTarget d{cmd.fx, cmd.fy, cmd.mz};
      nav_.Publish(d);
      BehaviorData b = last_behavior_;
      behavior_.Publish(b);
    });
    RCLCPP_INFO(get_logger(),
      "USB navigation link configured: %s:%ld baud, control interface '%s'",
      chassis_cmd_topic.c_str(),
      static_cast<long>(baud),
      iface.c_str());
  }

private:
  static double steadySeconds()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
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
  LibXR::Topic nav_, behavior_, team_, game_, online_, offline_, radar_;
  std::vector<LibXR::Topic::Callback> callbacks_;
  ros2_libxr::TimedSetpoint chassis_cmd_;
  double command_timeout_{0.15};
  double rotate_latency_{0.02};
  double chassis_yaw_offset_{0.0};
  std::string command_frame_, odom_frame_;
  double odom_received_{-1.0};
  double odom_yaw_{0.0};
  double odom_yaw_rate_{0.0};
  double odom_stamp_{-1.0};
  rclcpp::Subscription<ros_interfaces::msg::ChassisCommand>::SharedPtr chassis_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  BehaviorData last_behavior_{};
  rclcpp::Subscription<ros_interfaces::msg::Behavior>::SharedPtr behavior_sub_;
  rclcpp::Publisher<ros_interfaces::msg::TeamInformation>::SharedPtr team_pub_;
  rclcpp::Publisher<ros_interfaces::msg::GameInfo>::SharedPtr game_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOnline>::SharedPtr online_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_pub_;
  rclcpp::Publisher<ros_interfaces::msg::RadarInfo>::SharedPtr radar_pub_;
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
