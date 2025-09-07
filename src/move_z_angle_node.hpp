/**
 * @file move_z_angle_node.hpp
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

class MoveZAngleActionServer : public rclcpp::Node {
    using MoveZAngle = octa_ros::action::MoveZAngle;
    using GoalHandleMoveZAngle = rclcpp_action::ServerGoalHandle<MoveZAngle>;

  public:
    explicit MoveZAngleActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
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

    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const MoveZAngle::Goal> goal);

    rclcpp_action::CancelResponse
    handle_cancel(std::shared_ptr<GoalHandleMoveZAngle> goal_handle);

    void handle_accepted(std::shared_ptr<GoalHandleMoveZAngle> goal_handle);

    void execute(std::shared_ptr<GoalHandleMoveZAngle> goal_handle);
};

#endif // MOVE_Z_ANGLE_NODE_HPP
