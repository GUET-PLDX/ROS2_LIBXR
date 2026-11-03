#include "ros2_libxr/velocity_command.hpp"
#include "ros2_libxr/wire.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/behavior.hpp>
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
using ros2_libxr::WorldToBody;

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
      std::initializer_list<SharedTopic::TopicConfig>{
        {"team_info"}, {"game_info"}, {"online_info"}, {"offline_info"}, {"radar_info"}});
    static auto tx = std::make_unique<SharedTopicClient>(hw,
      manager,
      "uart_client",
      16,
      std::initializer_list<SharedTopicClient::TopicConfig>{{"nav_data"}, {"behavior_data"}});

    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/cmd_vel_mpc");
    command_timeout_ = declare_parameter<double>("command_timeout", 0.15);
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    const auto odom_frame = declare_parameter<std::string>("odom_frame", "camera_init");
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.3);
    rotate_latency_ = declare_parameter<double>("rotate_latency", 0.02);
    chassis_yaw_offset_ = declare_parameter<double>("chassis_yaw_offset", 0.0);
    if (!std::isfinite(command_timeout_) || command_timeout_ <= 0.0 || !std::isfinite(odom_timeout_) ||
        odom_timeout_ <= 0.0 || !std::isfinite(rotate_latency_) || rotate_latency_ < 0.0 ||
        !std::isfinite(chassis_yaw_offset_) || odom_frame.empty()) {
      throw std::invalid_argument("Invalid velocity command timing, yaw offset or odometry frame");
    }
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(cmd_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
            !std::isfinite(msg->angular.z)) {
          chassis_cmd_ = {};
          return;
        }
        chassis_cmd_.value = {static_cast<float>(msg->linear.x),
          static_cast<float>(msg->linear.y),
          static_cast<float>(msg->angular.z)};
        chassis_cmd_.stamp = now().seconds();
        chassis_cmd_.received = steadySeconds();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [this, odom_frame](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & q = msg->pose.pose.orientation;
        const double norm = std::hypot(std::hypot(q.w, q.x), std::hypot(q.y, q.z));
        const double rate = msg->twist.twist.angular.z;
        if (msg->header.frame_id != odom_frame || !std::isfinite(norm) || norm < 1.0e-6 ||
            !std::isfinite(rate)) {
          heading_ = {};
          return;
        }
        const double w = q.w / norm, x = q.x / norm, y = q.y / norm, z = q.z / norm;
        heading_.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        heading_.yaw_rate = rate;
        heading_.stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).seconds();
        heading_.received = steadySeconds();
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
      bool valid = false;
      const ChassisSetpoint cmd = WorldToBody(chassis_cmd_,
        heading_,
        t,
        steady_now,
        command_timeout_,
        odom_timeout_,
        rotate_latency_,
        chassis_yaw_offset_,
        &valid);
      ChassisTarget d{};
      d.vx = cmd.vx;
      d.vy = cmd.vy;
      d.wz = cmd.wz;
      d.speed = valid;
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
  ros2_libxr::TimedHeading heading_;
  double command_timeout_{0.15};
  double odom_timeout_{0.3};
  double rotate_latency_{0.02};
  double chassis_yaw_offset_{0.0};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
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
