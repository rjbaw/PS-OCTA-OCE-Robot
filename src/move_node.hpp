/**
 * @file move_node.hpp
 * @author rjbaw
 * @brief Action server to translate in XY and rotate about Z.
 *
 * Provides a motion primitive to adjust yaw and translate in XY at a given
 * radius, while respecting constraints via MoveIt.
 *
 * @par Parameters
 * - plan_only (bool, default: false): skip planning and execution (CI mode).
 *   This mode is selected at initialization and requires a restart to change.
 *
 * @par Action Server
 * - Name: `move_action`; Goal: `offset_x` (mm), `offset_y` (mm),
 *   `target_angle` (deg), `apply_offset` (bool), and `radius` (mm).
 *
 * @note Units: angles are degrees in the action goal; internally converted to
 * radians. Goal distances are millimeters and are converted to meters.
 *
 * @ingroup actions
 */

#ifndef MOVE_NODE_HPP
#define MOVE_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>

#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/robot_state/conversions.hpp>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>

#include <octa_ros/action/move.hpp>

#include "motion_utils.hpp"
#include "utils.hpp"

/**
 * @brief Action server that translates the tool in XY and adjusts yaw.
 *
 * Uses MoveIt to plan and execute a constrained motion, typically to reposition
 * the end-effector at a given radius and yaw angle while respecting limits.
 */
class MoveActionServer : public rclcpp::Node {
    using Move = octa_ros::action::Move;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  public:
    /**
     * @brief Construct the node; call init() to create interfaces.
     */
    explicit MoveActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MoveActionServer() override;

    /**
     * @brief Initialize action server and MoveIt components.
     */
    void init();

  private:
    rclcpp_action::Server<Move>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleMove> active_goal_handle_;

    moveit_cpp::MoveItCppPtr moveit_cpp_;
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
    std::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager>
        tem_;
    std::jthread worker_;
    std::jthread cancel_worker_;

    /** @brief Action callbacks for goal lifecycle and execution. */
    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const Move::Goal> goal);

    rclcpp_action::CancelResponse
    handle_cancel(std::shared_ptr<GoalHandleMove> goal_handle);

    void handle_accepted(std::shared_ptr<GoalHandleMove> goal_handle);

    void execute(std::shared_ptr<GoalHandleMove> goal_handle,
                 std::stop_token stop_token);
};

#endif // MOVE_NODE_HPP
