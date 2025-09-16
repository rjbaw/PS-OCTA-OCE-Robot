/**
 * @file move_z_angle_node.hpp
 * @author rjbaw
 * @brief Action server to move along Z and correct yaw angle.
 *
 * Provides a motion primitive to adjust yaw and translate in XY at a given
 * radius, while respecting constraints via MoveIt.
 *
 * @par Parameters
 * - plan_only (bool, default: false): skip execution (plan only).
 * - offline_mode (bool, default: false): disable MoveIt and execution.
 * - envelope_radius_m (double, default: 0.05 m): spherical envelope radius
 *   around current TCP used for path constraints (declared on first use).
 * - planning_pipelines (string[]; default: ["pilz_ptp", "pilz_lin"]): set of
 *   planning pipelines to try; the shortest plan is chosen.
 *
 * @par Action Server
 * - Name: `move_z_angle_action`; Goal: `target_angle` (deg), `radius` (m),
 *   `angle` (deg path angle for XY offset).
 *
 * @note Units: angles are degrees in the action goal; internally converted to
 * radians. Distances are meters.
 *
 * @ingroup actions
 */

#ifndef MOVE_Z_ANGLE_NODE_HPP
#define MOVE_Z_ANGLE_NODE_HPP

#include <Eigen/Geometry>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>

#include <octa_ros/action/move_z_angle.hpp>

#include "motion_utils.hpp"
#include "utils.hpp"

/**
 * @brief Action server that moves the tool along Z and adjusts yaw.
 *
 * Uses MoveIt to plan and execute a constrained motion, typically to reposition
 * the end-effector at a given radius and yaw angle while respecting limits.
 */
class MoveZAngleActionServer : public rclcpp::Node {
    using MoveZAngle = octa_ros::action::MoveZAngle;
    using GoalHandleMoveZAngle = rclcpp_action::ServerGoalHandle<MoveZAngle>;

  public:
    /**
     * @brief Construct the node; call init() to create interfaces.
     */
    explicit MoveZAngleActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Initialize action server and MoveIt components.
     */
    void init();

  private:
    rclcpp_action::Server<MoveZAngle>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleMoveZAngle> active_goal_handle_;

    moveit_cpp::MoveItCppPtr moveit_cpp_;
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
    std::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager>
        tem_;

    double radius_ = 0.0;
    double angle_ = 0.0;

    /** @brief Action callbacks for goal lifecycle and execution. */
    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const MoveZAngle::Goal> goal);

    rclcpp_action::CancelResponse
    handle_cancel(std::shared_ptr<GoalHandleMoveZAngle> goal_handle);

    void handle_accepted(std::shared_ptr<GoalHandleMoveZAngle> goal_handle);

    void execute(std::shared_ptr<GoalHandleMoveZAngle> goal_handle);
};

#endif // MOVE_Z_ANGLE_NODE_HPP
