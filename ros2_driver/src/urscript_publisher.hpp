#ifndef URSCRIPT_PUBLISHER_HPP_
#define URSCRIPT_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class urscript_publisher : public rclcpp::Node {
  public:
    urscript_publisher();
    void activate_freedrive();
    void deactivate_freedrive();

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client;
    bool freedrive;
    bool executed;
    void publish_to_robot();
};

#endif // URSCRIPT_PUBLISHER_HPP
