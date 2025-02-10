#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "urscript_publisher.hpp"

double to_radian(const double degree);
double to_degree(const double radian);

void add_collision_obj(
    moveit::planning_interface::MoveGroupInterface &move_group_interface);

void print_target(rclcpp::Logger const &logger,
                  geometry_msgs::msg::Pose target_pose);
bool move_to_target(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    rclcpp::Logger const &logger);
bool move_to_target_urscript(const geometry_msgs::msg::Pose &target_pose,
                             rclcpp::Logger const &logger,
                             std::shared_ptr<urscript_publisher> urscript_node,
                             double velocity = 1.0, double acceleration = 1.0);

#endif // UTILS_HPP_
