#ifndef UTILS_HPP_
#define UTILS_HPP_

// #include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

double to_radian(const double degree);
double to_degree(const double radian);
void add_collision_obj(moveit::planning_interface::MoveGroupInterface &move_group_interface);

#endif // UTILS_HPP_
