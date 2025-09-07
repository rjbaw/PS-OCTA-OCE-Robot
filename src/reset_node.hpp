/**
 * @file reset_node.hpp
 */

#ifndef RESET_NODE_HPP
#define RESET_NODE_HPP

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <octa_ros/action/reset.hpp>

#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>

#include "motion_utils.hpp"
#include "utils.hpp"

class ResetActionServer : public rclcpp::Node {
  public:
    using ResetAction = octa_ros::action::Reset;
    using GoalHandleResetAction = rclcpp_action::ServerGoalHandle<ResetAction>;

    explicit ResetActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void init();

  private:
    rclcpp_action::Server<ResetAction>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleResetAction> active_goal_handle_;
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
    std::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager>
        tem_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    bool failed_ = false;

    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const ResetAction::Goal> goal);
    rclcpp_action::CancelResponse
    handle_cancel(std::shared_ptr<GoalHandleResetAction> goal_handle);
    void handle_accepted(std::shared_ptr<GoalHandleResetAction> goal_handle);
    void publish_stop(double decel = 2.0);
    void execute(std::shared_ptr<GoalHandleResetAction> goal_handle);
};

#endif // RESET_NODE_HPP
