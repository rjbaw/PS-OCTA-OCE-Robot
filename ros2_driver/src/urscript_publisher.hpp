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
    void
    execute_trajectory(const std::vector<std::array<double, 6>> &trajectory,
                       double velocity = 1.0, double acceleration = 1.0,
                       bool use_movej = true);
    void publish_script_now(const std::string &script);

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client;
    bool freedrive;
    bool executed;
    void publish_to_robot();

    bool traj_requested_;
    bool use_movej_;
    double traj_velocity_;
    double traj_acceleration_;
    std::vector<std::array<double, 6>> trajectory_points_;
};

#endif // URSCRIPT_PUBLISHER_HPP
