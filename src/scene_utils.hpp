/**
 * @file scene_utils.hpp
 * @brief Helpers to build static collision objects for the planning scene.
 */

#ifndef SCENE_UTILS_HPP
#define SCENE_UTILS_HPP

#include <moveit_msgs/msg/collision_object.hpp>
#include <string>

namespace octa_ros::scene {

moveit_msgs::msg::CollisionObject make_floor(const std::string &frame_id);
moveit_msgs::msg::CollisionObject make_robot_base(const std::string &frame_id);
moveit_msgs::msg::CollisionObject make_monitor(const std::string &frame_id);

} // namespace octa_ros::scene

#endif  // SCENE_UTILS_HPP
