/**
 * @file reset_node.hpp
 * @author rjbaw
 * @brief Action server to move the robot to a known reset posture.
 *
 * Sends a bounded URScript reset motion and confirms completion from the robot
 * joint state. Cancel and timeout both publish a URScript stop command.
 *
 * @par Parameters
 * - plan_only (bool, default: false): skip the robot command and state
 *   confirmation (CI mode). This mode requires a restart to change.
 * - urscript_robot_vel (double, default: 0.5): reset joint velocity (rad/s).
 * - urscript_robot_acc (double, default: 0.5): reset joint acceleration
 *   (rad/s^2).
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

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <octa_ros/action/reset.hpp>

/**
 * @brief Action server that safely returns the robot to a reset/home state.
 *
 * Publishes a stop command as needed and confirms the reset posture using
 * MoveIt's monitored robot state.
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

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    std::atomic<bool> cancel_requested_ = false;
    std::mutex reset_mutex_;
    std::jthread worker_;

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

    void execute(std::shared_ptr<GoalHandleResetAction> goal_handle,
                 std::stop_token stop_token);
};

#endif // RESET_NODE_HPP
