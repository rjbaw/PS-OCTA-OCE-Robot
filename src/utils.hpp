/**
 * @file utils.hpp
 * @author rjbaw
 * @brief Common math and logging utilities used across nodes.
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

/** @addtogroup utils
 *  @{ */

/**
 * @brief Convert degrees to radians.
 * @param degree Angle in degrees.
 * @return Angle in radians.
 */
double to_radian(double degree);

/**
 * @brief Convert radians to degrees.
 * @param radian Angle in radians.
 * @return Angle in degrees.
 */
double to_degree(double radian);

/**
 * @brief Log a 3D pose as a single line at INFO level.
 *
 * The position is logged in meters and the orientation as a unit quaternion.
 * This helper is primarily used to trace target end-effector poses.
 *
 * @param logger rclcpp logger to emit the message on.
 * @param target_pose Pose to log (position in meters, quaternion orientation).
 */
void print_target(rclcpp::Logger const &logger,
                  geometry_msgs::msg::Pose target_pose);

/** @} */

#endif // UTILS_HPP
