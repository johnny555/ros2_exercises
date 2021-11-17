#include "my_components/odom_component.hpp"

#include <iostream>
#include <memory>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

namespace my_components {
OdomComponent::OdomComponent(const rclcpp::NodeOptions &options)
    : Node("odom_component", options) {
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&OdomComponent::topic_callback, this, _1));
}

void OdomComponent::topic_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto pose = msg->pose.pose.position;
  RCLCPP_INFO(this->get_logger(), "x: '%f'", pose.x, "y: '%f'", pose.y);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::OdomComponent)