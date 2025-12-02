/**
 * @file move_node.hpp
 * @author rjbaw
 * @brief Action server to move along Z and correct yaw angle.
 *
 * Provides a motion primitive to adjust yaw and translate in XY at a given
 * radius, while respecting constraints via MoveIt.
 *
 * @par Parameters
 * - plan_only (bool, default: false): skip execution (plan-only/CI mode).
 *
 * @par Action Server
 * - Name: `move_action`; Goal: `offset_x` (mm), `offset_y` (mm),
 *   `target_angle` (deg), `apply_offset` (bool), `angle` (deg path angle),
 *   `radius` (mm).
 *
 * @note Units: angles are degrees in the action goal; internally converted to
 * radians. Distances are meters.
 *
 * @ingroup actions
 */

#ifndef MOVE_NODE_HPP
#define MOVE_NODE_HPP

#include <Eigen/Geometry>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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
 * @brief Action server that moves the tool along Z and adjusts yaw.
 *
 * Uses MoveIt to plan and execute a constrained motion, typically to reposition
 * the end-effector at a given radius and yaw angle while respecting limits.
 *
 * When `apply_offset` is false, the node captures the current TCP position as
 * the circle centre whenever `angle == 0` (degrees) and computes XY targets as
 * an absolute offset from that stored centre using the specified `radius` and
 * the cumulative path angle.
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

    double radius_ = 0.0;
    double angle_ = 0.0;

    Eigen::Vector3d centre_xyz_{0.0, 0.0, 0.0};
    bool centre_set_ = false;

    /** @brief Action callbacks for goal lifecycle and execution. */
    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const Move::Goal> goal);

    rclcpp_action::CancelResponse
    handle_cancel(std::shared_ptr<GoalHandleMove> goal_handle);

    void handle_accepted(std::shared_ptr<GoalHandleMove> goal_handle);

    void execute(std::shared_ptr<GoalHandleMove> goal_handle);
};

#endif // MOVE_NODE_HPP
