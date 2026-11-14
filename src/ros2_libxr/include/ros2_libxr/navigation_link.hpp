#pragma once

#include "ros2_libxr/wire.hpp"

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace ros2_libxr {
// An injected transport receives the same final payloads as the USB link.
using NavigationOutput = std::function<void(const wire::ChassisTarget &, const wire::BehaviorData &)>;
std::shared_ptr<rclcpp::Node> createNavigationLink(NavigationOutput output = {});
}  // namespace ros2_libxr
