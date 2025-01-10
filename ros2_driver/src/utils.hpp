#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>

double to_radian(const double degree);
double to_degree(const double radian);

void add_collision_obj(
    moveit::planning_interface::MoveGroupInterface &move_group_interface);

void print_target(rclcpp::Logger const &logger,
                  geometry_msgs::msg::Pose target_pose);
bool move_to_target(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    rclcpp::Logger const &logger);

#endif // UTILS_HPP_
