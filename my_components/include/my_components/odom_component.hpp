#ifndef COMPOSITION__ODOM_COMPONENT_HPP_
#define COMPOSITION__ODOM_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

class OdomComponent : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit OdomComponent(const rclcpp::NodeOptions &options);

protected:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

} // namespace my_components

#endif // COMPOSITION__ODOM_COMPONENT_HPP_