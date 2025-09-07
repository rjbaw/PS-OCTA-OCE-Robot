/**
 * @file utils.hpp
 * @brief utility functions
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

double to_radian(double degree);
double to_degree(double radian);

void print_target(rclcpp::Logger const &logger,
                  geometry_msgs::msg::Pose target_pose);

#endif // UTILS_HPP
