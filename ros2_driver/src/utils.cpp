#include "utils.hpp"

double to_radian(const double degree) {
    return (std::numbers::pi / 180 * degree);
}

double to_degree(const double radian) {
    return (180 / std::numbers::pi * radian);
}

void add_collision_obj(
    moveit::planning_interface::MoveGroupInterface &move_group_interface) {

    auto const collision_floor = [frame_id =
                                      move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_floor;
        collision_floor.header.frame_id = frame_id;
        collision_floor.id = "floor";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 10.0;
        primitive.dimensions[primitive.BOX_Y] = 10.0;
        primitive.dimensions[primitive.BOX_Z] = 0.01;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.0855;

        collision_floor.primitives.push_back(primitive);
        collision_floor.primitive_poses.push_back(box_pose);
        collision_floor.operation = collision_floor.ADD;

        return collision_floor;
    }();

    auto const collision_base = [frame_id =
                                     move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_base;
        collision_base.header.frame_id = frame_id;
        collision_base.id = "robot_base";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.27;
        primitive.dimensions[primitive.BOX_Y] = 0.27;
        primitive.dimensions[primitive.BOX_Z] = 0.085;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.043;

        collision_base.primitives.push_back(primitive);
        collision_base.primitive_poses.push_back(box_pose);
        collision_base.operation = collision_base.ADD;

        return collision_base;
    }();

    auto const collision_monitor =
        [frame_id = move_group_interface.getPlanningFrame()] {
            moveit_msgs::msg::CollisionObject collision_monitor;
            collision_monitor.header.frame_id = frame_id;
            collision_monitor.id = "monitor";
            shape_msgs::msg::SolidPrimitive primitive;

            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = 0.25;
            primitive.dimensions[primitive.BOX_Y] = 0.6;
            primitive.dimensions[primitive.BOX_Z] = 0.6;

            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = -0.2;
            box_pose.position.y = 0.435;
            box_pose.position.z = 0.215;

            collision_monitor.primitives.push_back(primitive);
            collision_monitor.primitive_poses.push_back(box_pose);
            collision_monitor.operation = collision_monitor.ADD;

            return collision_monitor;
        }();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_floor);
    planning_scene_interface.applyCollisionObject(collision_base);
    planning_scene_interface.applyCollisionObject(collision_monitor);
}

void print_target(rclcpp::Logger const &logger,
                  geometry_msgs::msg::Pose target_pose) {
    RCLCPP_INFO(logger,
                std::format("Target Pose: "
                            " x: {}, y: {}, z: {},"
                            " qx: {}, qy: {}, qz: {}, qw: {}",
                            target_pose.position.x, target_pose.position.y,
                            target_pose.position.z, target_pose.orientation.x,
                            target_pose.orientation.y,
                            target_pose.orientation.z,
                            target_pose.orientation.w)
                    .c_str());
}

bool move_to_target(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    rclcpp::Logger const &logger) {
    auto joint_values = move_group_interface.getCurrentJointValues();
    for (size_t i = 0; i < joint_values.size(); ++i) {
        RCLCPP_INFO(logger, "Joint[%zu]: %f", i, joint_values[i]);
    }
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan plan_feedback;
        auto const ok =
            static_cast<bool>(move_group_interface.plan(plan_feedback));
        return std::make_pair(ok, plan_feedback);
    }();
    if (success) {
        move_group_interface.execute(plan);
    }
    return success;
}

bool move_to_target_urscript(const geometry_msgs::msg::Pose &target_pose,
                             rclcpp::Logger const &logger,
                             std::shared_ptr<urscript_publisher> urscript_node,
                             double velocity, double acceleration) {
    const double x = target_pose.position.x;
    const double y = target_pose.position.y;
    const double z = target_pose.position.z;
    tf2::Quaternion q;
    tf2::fromMsg(target_pose.orientation, q);
    q.normalize();

    double angle = 2.0 * std::acos(q.getW());
    double norm = std::sqrt(q.getX() * q.getX() + q.getY() * q.getY() +
                            q.getZ() * q.getZ());

    double rx = 0.0, ry = 0.0, rz = 0.0;
    if (norm < 1e-8) {
        rx = ry = rz = 0.0;
    } else {
        rx = (q.getX() / norm) * angle;
        ry = (q.getY() / norm) * angle;
        rz = (q.getZ() / norm) * angle;
    }
    RCLCPP_INFO(logger,
                "URScript Cartesian move request:\n  p[%f, %f, %f, %f, %f, %f]",
                x, y, z, rx, ry, rz);
    std::ostringstream prog;
    prog << "def single_move():\n";
    prog << "  end_freedrive_mode()\n";
    prog << "  movel(p[" << x << ", " << y << ", " << z << ", " << rx << ", "
         << ry << ", " << rz << "]";
    prog << ", a=" << acceleration << ", v=" << velocity << ")\n";
    prog << "end\n";
    try {
        urscript_node->publish_script_now(prog.str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger,
                     "Failed to publish URScript single_move program: %s",
                     e.what());
        return false;
    }

    RCLCPP_INFO(logger, "URScript command published: \n%s", prog.str().c_str());
    return true;
}
