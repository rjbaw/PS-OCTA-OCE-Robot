/**
 * @file freedrive_node.hpp
 */

#ifndef FREEDRIVE_NODE_HPP
#define FREEDRIVE_NODE_HPP

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_msgs/msg/bool.hpp>

#include <octa_ros/action/freedrive.hpp>

class FreedriveActionServer : public rclcpp::Node {
  public:
    using Freedrive = octa_ros::action::Freedrive;
    using GoalHandleFreedrive = rclcpp_action::ServerGoalHandle<Freedrive>;
    using SwitchSrv = controller_manager_msgs::srv::SwitchController;

    explicit FreedriveActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    rclcpp_action::Server<Freedrive>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleFreedrive> active_goal_handle_;
    rclcpp::Client<SwitchSrv>::SharedPtr switch_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr freedrive_pub_;
    rclcpp::TimerBase::SharedPtr keepalive_timer_;

    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] rclcpp_action::GoalUUID goal_id,
                std::shared_ptr<const Freedrive::Goal> goal);

    rclcpp_action::CancelResponse
    handle_cancel(std::shared_ptr<GoalHandleFreedrive> goal_handle);

    void handle_accepted(std::shared_ptr<GoalHandleFreedrive> goal_handle);

    void execute(std::shared_ptr<GoalHandleFreedrive> goal_handle);

    void start_keepalive();
    void stop_keepalive();
    void publish_bool(bool value);
    bool switch_to_freedrive_controller(bool to_freedrive);
};

#endif // FREEDRIVE_NODE_HPP
