/**
 * @file reset_node.hpp
 * @author rjbaw
 * @brief Action server to move the robot to a known reset posture.
 *
 * Plans and executes a safe trajectory to return the robot to a known state,
 * with optional plan-only/offline modes. Publishes a URScript stop command when
 * canceling or on error.
 *
 * @par Parameters
 * - plan_only (bool, default: false): skip execution (plan-only/CI mode).
 * - urscript_robot_vel (double, default: 0.5): URScript fallback joint
 *   velocity (m/s equivalent in UR units).
 * - urscript_robot_acc (double, default: 0.5): URScript fallback joint
 *   acceleration (m/s^2 equivalent in UR units).
 *
 * @par Publishers
 * - `std_msgs::msg::String` on `/urscript_interface/script_command` (QoS:
 *   system default) – stop/URScript commands.
 *
 * @par Action Server
 * - Name: `reset_action`.
 *
 * @ingroup actions
 */

#ifndef RESET_NODE_HPP
#define RESET_NODE_HPP

#include <memory>

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

/**
 * @brief Action server that safely returns the robot to a reset/home state.
 *
 * Publishes a stop command as needed and plans a safe trajectory back to a
 * known configuration using MoveIt.
 */
class ResetActionServer : public rclcpp::Node {
  public:
    using ResetAction = octa_ros::action::Reset;
    using GoalHandleResetAction = rclcpp_action::ServerGoalHandle<ResetAction>;

    /**
     * @brief Construct the node; call init() to create interfaces.
     */
    explicit ResetActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Initialize action server, MoveIt components and publishers.
     */
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

    /** @brief Action callbacks for goal lifecycle and execution. */
    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const ResetAction::Goal> goal);
    rclcpp_action::CancelResponse
    handle_cancel(std::shared_ptr<GoalHandleResetAction> goal_handle);
    void handle_accepted(std::shared_ptr<GoalHandleResetAction> goal_handle);

    /**
     * @brief Publish an emergency stop/slowdown command.
     * @param decel Desired deceleration factor.
     */
    void publish_stop(double decel = 2.0);

    void execute(std::shared_ptr<GoalHandleResetAction> goal_handle);
};

#endif // RESET_NODE_HPP
