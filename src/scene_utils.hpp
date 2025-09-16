/**
 * @file scene_utils.hpp
 * @author rjbaw
 * @brief Builders for static collision objects in the planning scene.
 */

#ifndef SCENE_UTILS_HPP
#define SCENE_UTILS_HPP

#include <moveit_msgs/msg/collision_object.hpp>
#include <string>

/** @addtogroup scene
 *  @{ */

namespace octa_ros::scene {

/**
 * @brief Make a thin floor plane centered at the origin.
 * @param frame_id Frame for the collision object header.
 * @return Collision object representing the floor.
 */
moveit_msgs::msg::CollisionObject make_floor(const std::string &frame_id);

/**
 * @brief Make a box matching the robot base footprint.
 * @param frame_id Frame for the collision object header.
 * @return Collision object representing the robot base.
 */
moveit_msgs::msg::CollisionObject make_robot_base(const std::string &frame_id);

/**
 * @brief Make a box approximating a nearby monitor for collision avoidance.
 * @param frame_id Frame for the collision object header.
 * @return Collision object representing the monitor.
 */
moveit_msgs::msg::CollisionObject make_monitor(const std::string &frame_id);

} // namespace octa_ros::scene

/** @} */

#endif // SCENE_UTILS_HPP
