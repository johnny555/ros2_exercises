#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node {
public:
  SimpleSubscriber() : Node("simple_subscriber") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto pose = msg->pose.pose.position;
    RCLCPP_INFO(this->get_logger(), "x: '%f'", pose.x, "y: '%f'", pose.y);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
