/**
 * @file motion_utils.cpp
 * @author rjbaw
 */

#include "motion_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace octa_ros::motion {

moveit_msgs::msg::Constraints make_envelope(const Eigen::Isometry3d &centre,
                                            const std::string &frame_id,
                                            const std::string &link_name,
                                            double lin_radius_m,
                                            double ang_radius_rad) {
    moveit_msgs::msg::Constraints constraints;

    moveit_msgs::msg::PositionConstraint pos_constraint;
    pos_constraint.header.frame_id = frame_id;
    pos_constraint.link_name = link_name;
    pos_constraint.weight = 1.0;
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions = {lin_radius_m};
    pos_constraint.constraint_region.primitives.push_back(sphere);
    geometry_msgs::msg::Pose centre_pose;
    centre_pose.position.x = centre.translation().x();
    centre_pose.position.y = centre.translation().y();
    centre_pose.position.z = centre.translation().z();
    centre_pose.orientation.w = 1.0;
    pos_constraint.constraint_region.primitive_poses.push_back(centre_pose);

    moveit_msgs::msg::OrientationConstraint orient_constraint;
    orient_constraint.header.frame_id = frame_id;
    orient_constraint.link_name = link_name;
    orient_constraint.weight = 1.0;
    Eigen::Quaterniond quaternion(centre.rotation());
    orient_constraint.orientation.x = quaternion.x();
    orient_constraint.orientation.y = quaternion.y();
    orient_constraint.orientation.z = quaternion.z();
    orient_constraint.orientation.w = quaternion.w();
    orient_constraint.absolute_x_axis_tolerance =
        orient_constraint.absolute_y_axis_tolerance =
            orient_constraint.absolute_z_axis_tolerance = ang_radius_rad;

    constraints.position_constraints.push_back(pos_constraint);
    constraints.orientation_constraints.push_back(orient_constraint);
    return constraints;
}

} // namespace octa_ros::motion
