/**
 * @file motion_utils.hpp
 * @brief Shared helpers for building MoveIt constraints and motion utilities.
 */

#ifndef MOTION_UTILS_HPP
#define MOTION_UTILS_HPP

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>

namespace octa_ros::motion {

moveit_msgs::msg::Constraints make_envelope(const Eigen::Isometry3d &centre,
                                            const std::string &frame_id,
                                            const std::string &link_name,
                                            double lin_radius_m,
                                            double ang_radius_rad);

} // namespace octa_ros::motion

#endif // MOTION_UTILS_HPP
