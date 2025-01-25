#include "joint_state_subscriber.hpp"

joint_state_subscriber::joint_state_subscriber()
    : Node("joint_state_subscriber"),
      best_effort(rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()) {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "robot_state", best_effort,
        std::bind(&joint_state_subscriber::callback, this,
                  std::placeholders::_1));
}

void joint_state_subscriber::callback(
    const std_msgs::msg::Int32::SharedPtr msg) {
    ready_ = (msg->data != 0);
}

bool joint_state_subscriber::ready() { return ready_; };
