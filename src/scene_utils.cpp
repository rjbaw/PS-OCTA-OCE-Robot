/**
 * @file scene_utils.cpp
 * @author rjbaw
 */

#include "scene_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace octa_ros::scene {

moveit_msgs::msg::CollisionObject make_floor(const std::string &frame_id) {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id;
    obj.id = "floor";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {0.6, 0.6, 0.01};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = -0.0855;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    return obj;
}

moveit_msgs::msg::CollisionObject make_robot_base(const std::string &frame_id) {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id;
    obj.id = "robot_base";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {0.27, 0.27, 0.085};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = -0.043;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    return obj;
}

moveit_msgs::msg::CollisionObject make_monitor(const std::string &frame_id) {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id;
    obj.id = "monitor";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {0.25, 0.6, 0.6};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = -0.2;
    pose.position.y = 0.7;
    pose.position.z = 0.215;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    return obj;
}

} // namespace octa_ros::scene
