#ifndef JOINT_STATE_SUBSCRIBER_HPP_
#define JOINT_STATE_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class joint_state_subscriber : public rclcpp::Node {
  public:
    joint_state_subscriber();
    void callback(const std_msgs::msg::Int32::SharedPtr msg);
    bool ready();

  private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

    rclcpp::QoS best_effort;
    bool ready_ = false;
};

#endif // JOINT_STATE_SUBSCRIBER_HPP_
