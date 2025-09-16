/**
 * @file motion_utils.hpp
 * @author rjbaw
 * @brief Helpers for building MoveIt constraints and motion utilities.
 */

#ifndef MOTION_UTILS_HPP
#define MOTION_UTILS_HPP

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>

/** @addtogroup motion
 *  @{ */

namespace octa_ros::motion {

/**
 * @brief Create a spherical position envelope and angular tolerance constraint.
 *
 * Builds a MoveIt Constraints message that constrains a link to stay within a
 * sphere centered at @p centre (in @p frame_id) and within an angular radius
 * around the orientation of @p centre.
 *
 * @param centre       Target pose (translation and rotation) in @p frame_id.
 * @param frame_id     Frame in which the constraints are expressed.
 * @param link_name    Robot link to which the constraints apply.
 * @param lin_radius_m Linear allowed radius around the center (meters).
 * @param ang_radius_rad Angular tolerance around the center orientation
 *                       (radians), applied equally to X/Y/Z axes.
 * @return MoveIt constraints message with position and orientation constraints.
 */
moveit_msgs::msg::Constraints make_envelope(const Eigen::Isometry3d &centre,
                                            const std::string &frame_id,
                                            const std::string &link_name,
                                            double lin_radius_m,
                                            double ang_radius_rad);

} // namespace octa_ros::motion

/** @} */

#endif // MOTION_UTILS_HPP
