#include <Eigen/Dense>
#include <chrono>
#include <csignal>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "dds_publisher.hpp"
#include "dds_subscriber.hpp"
#include "img_subscriber.hpp"
#include "process_img.hpp"
#include "urscript_publisher.hpp"
#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

const bool use_urscript = false;

using namespace std::chrono_literals;
std::atomic<bool> running(true);

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"),
                "Signal %d received, shutting down...", signum);
    running = false;
    rclcpp::shutdown();
}

bool tol_measure(double &roll, double &pitch, double &angle_tolerance) {
    return ((std::abs(std::abs(roll)) < to_radian(angle_tolerance)) &&
            (std::abs(std::abs(pitch)) < to_radian(angle_tolerance)));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    using moveit::planning_interface::MoveGroupInterface;

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // 3D Parameters
    const int interval = 16;
    const bool single_interval = false;

    // Publisher Parameters
    std::string msg;
    double angle = 0.0;
    int circle_state = 1;
    bool apply_config = true;
    bool end_state = false;
    bool scan_3d = false;

    // Internal Parameters
    bool planning = false;
    double angle_increment;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    Eigen::Matrix3d rotmat_eigen;
    cv::Mat img;
    std::vector<cv::Mat> img_array;
    std::vector<Eigen::Vector3d> pc_lines;
    tf2::Quaternion q;
    tf2::Quaternion target_q;
    geometry_msgs::msg::Pose target_pose;

    // Subscriber Parameters
    double robot_vel;
    double robot_acc;
    double radius;
    double angle_limit;
    double dz;
    double z_tolerance;
    double angle_tolerance;
    double z_height;
    int num_pt;
    bool fast_axis = true;
    bool success = false;
    bool next = false;
    bool previous = false;
    bool home = false;
    bool z_focused = false;
    bool angle_focused = false;

    auto const move_group_node =
        std::make_shared<rclcpp::Node>("node_moveit", node_options);
    auto move_group_interface =
        MoveGroupInterface(move_group_node, "ur_manipulator");
    auto subscriber_node = std::make_shared<dds_subscriber>();
    auto urscript_node = std::make_shared<urscript_publisher>();
    auto publisher_node = std::make_shared<dds_publisher>(
        msg, angle, circle_state, fast_axis, apply_config, end_state, scan_3d);
    auto img_subscriber_node = std::make_shared<img_subscriber>();

    auto const logger = rclcpp::get_logger("logger_planning");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);
    executor.add_node(urscript_node);
    executor.add_node(img_subscriber_node);
    std::thread spinner([&executor]() { executor.spin(); });

    add_collision_obj(move_group_interface);

    move_group_interface.setPlanningTime(10.0);
    move_group_interface.setNumPlanningAttempts(30);
    move_group_interface.setPlanningPipelineId("ompl");
    // move_group_interface.setPlanningPipelineId("stomp");

    publisher_node->set_fast_axis(true);
    while (subscriber_node->fast_axis() != true) {
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    publisher_node->set_apply_config(true);

    while (rclcpp::ok() && running) {

        end_state = false;
        apply_config = false;
        fast_axis = true;
        publisher_node->set_fast_axis(fast_axis);
        publisher_node->set_apply_config(apply_config);
        publisher_node->set_end_state(end_state);
        next = subscriber_node->next();
        previous = subscriber_node->previous();
        home = subscriber_node->home();

        z_tolerance = subscriber_node->z_tolerance();
        angle_tolerance = subscriber_node->angle_tolerance();
        radius = subscriber_node->radius();
        angle_limit = subscriber_node->angle_limit();
        num_pt = subscriber_node->num_pt();
        robot_vel = subscriber_node->robot_vel();
        robot_acc = subscriber_node->robot_acc();
        z_height = subscriber_node->z_height();
        // robot_vel = 0.5;
        // robot_acc = 0.5;

        if (subscriber_node->freedrive()) {
            circle_state = 1;
            angle = 0.0;
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);
            urscript_node->activate_freedrive();
            while (subscriber_node->freedrive()) {
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
            urscript_node->deactivate_freedrive();
            if (!use_urscript) {
                urscript_node->resend_program();
            }
        }

        move_group_interface.setMaxVelocityScalingFactor(robot_vel);
        move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
        move_group_interface.setStartStateToCurrentState();

        if (subscriber_node->reset()) {
            angle = 0.0;
            circle_state = 1;
            msg = "[Action] Reset to default position";
            RCLCPP_INFO(logger, msg.c_str());
            publisher_node->set_msg(msg);
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);
            robot_vel = 0.8;
            robot_acc = 0.8;
            reset_robot_urscript(urscript_node, robot_vel, robot_acc);
	    success = true;
            //if (use_urscript) {
            //    reset_robot_urscript(urscript_node, robot_vel, robot_acc);
            //    // rclcpp::sleep_for(std::chrono::milliseconds(3000));
            //} else {
            //    move_group_interface.setJointValueTarget("shoulder_pan_joint",
            //                                             to_radian(0.0));
            //    move_group_interface.setJointValueTarget("shoulder_lift_joint",
            //                                             -to_radian(60.0));
            //    move_group_interface.setJointValueTarget("elbow_joint",
            //                                             to_radian(90.0));
            //    move_group_interface.setJointValueTarget("wrist_1_joint",
            //                                             to_radian(-120.0));
            //    move_group_interface.setJointValueTarget("wrist_2_joint",
            //                                             to_radian(-90.0));
            //    move_group_interface.setJointValueTarget("wrist_3_joint",
            //                                             to_radian(-135.0));
            //    success = move_to_target(move_group_interface, logger);
            //}
            if (!success) {
                msg = std::format("Reset Planning Failed!");
                publisher_node->set_msg(msg);
                RCLCPP_ERROR(logger, msg.c_str());
            }
        }

        if (subscriber_node->autofocus()) {
            if (!scan_3d) {
                scan_3d = true;
                apply_config = true;
                msg = "Starting 3D Scan";
                RCLCPP_INFO(logger, msg.c_str());
                publisher_node->set_msg(msg);
                publisher_node->set_scan_3d(scan_3d);
                while (subscriber_node->scan_3d() != scan_3d) {
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }
                publisher_node->set_apply_config(apply_config);
            }
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
            img_array.clear();
            for (int i = 0; i < interval; i++) {
                while (true) {
                    img = img_subscriber_node->get_img();
                    if (!img.empty()) {
                        break;
                    }
                    if (!subscriber_node->autofocus()) {
                        break;
                    }
                    // rclcpp::sleep_for(std::chrono::milliseconds(10000));
                }
                img_array.push_back(img);
                RCLCPP_INFO(logger, "Collected image %d", i + 1);
            }
            msg = "Calculating Rotations";
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());

            scan_3d = false;
            apply_config = true;
            publisher_node->set_scan_3d(scan_3d);
            while (subscriber_node->scan_3d() != scan_3d) {
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            publisher_node->set_apply_config(apply_config);

            pc_lines = lines_3d(img_array, interval, single_interval);
            open3d::geometry::PointCloud pcd_lines;
            for (const auto &point : pc_lines) {
                pcd_lines.points_.emplace_back(point);
            }
            auto boundbox = pcd_lines.GetMinimalOrientedBoundingBox(false);
            Eigen::Vector3d center = boundbox.GetCenter();
            msg += std::format("\n[Center] x:{} y:{} z:{}", center[0],
                               center[1], center[2]);
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());
            rotmat_eigen = align_to_direction(boundbox.R_);

            tf2::Matrix3x3 rotmat_tf(
                rotmat_eigen(0, 0), rotmat_eigen(0, 1), rotmat_eigen(0, 2),
                rotmat_eigen(1, 0), rotmat_eigen(1, 1), rotmat_eigen(1, 2),
                rotmat_eigen(2, 0), rotmat_eigen(2, 1), rotmat_eigen(2, 2));
            RCLCPP_INFO_STREAM(logger, "\nAligned Rotation Matrix:\n"
                                           << rotmat_eigen);

            /// temporary fix for swapped axis ///
            double tmp_roll;
            double tmp_pitch;
            double tmp_yaw;
            rotmat_tf.getRPY(tmp_roll, tmp_pitch, tmp_yaw);
            roll = tmp_yaw;
            pitch = -tmp_roll;
            yaw = tmp_pitch;
            rotmat_tf.setRPY(roll, pitch, yaw);
            ///////////////////////

            msg +=
                std::format("\nCalculated R:{:.2f}, P:{:.2f}, Y:{:.2f}",
                            to_degree(roll), to_degree(pitch), to_degree(yaw));
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());

            if (tol_measure(roll, pitch, angle_tolerance)) {
                angle_focused = true;
                msg += "\nAngle focused";
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, msg.c_str());
                scan_3d = false;
                apply_config = true;
                publisher_node->set_scan_3d(scan_3d);
                while (subscriber_node->scan_3d() != scan_3d) {
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }
                publisher_node->set_apply_config(apply_config);
            }

            if (angle_focused && !z_focused) {
                // dz = 0;
                dz = (z_height - center[1]) / (50 * 1000.0);
                msg += std::format("\ndz = {}", dz);
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, "dz: %f", dz);
                if (std::abs(dz) < (z_tolerance / 1000.0)) {
                    z_focused = true;
                    msg += "\nHeight focused";
                    publisher_node->set_msg(msg);
                    RCLCPP_INFO(logger, msg.c_str());
                } else {
                    planning = true;
                    if (use_urscript) {
                        success = move_to_target_urscript(0, 0, -dz, 0, 0, 0,
                                                          logger, urscript_node,
                                                          robot_vel, robot_acc);
                        // rclcpp::sleep_for(std::chrono::milliseconds(3000));
                    } else {
                        target_pose =
                            move_group_interface.getCurrentPose().pose;
                        target_pose.position.z += dz;
                        print_target(logger, target_pose);
                        move_group_interface.setPoseTarget(target_pose);
                        success = move_to_target(move_group_interface, logger);
                    }

                    if (!success) {
                        msg = std::format("Z-height Planning Failed!");
                        RCLCPP_ERROR(logger, msg.c_str());
                        publisher_node->set_msg(msg);
                    }
                }
            }

            if (!angle_focused) {
                planning = true;
                rotmat_tf.getRotation(q);
                q.normalize();
                if (use_urscript) {
                    double angle = 2.0 * std::acos(q.getW());
                    double norm =
                        std::sqrt(q.getX() * q.getX() + q.getY() * q.getY() +
                                  q.getZ() * q.getZ());
                    double rx = 0, ry = 0, rz = 0;
                    if (norm < 1e-8) {
                        rx = ry = rz = 0.0;
                    } else {
                        rx = (q.getX() / norm) * angle;
                        ry = (q.getY() / norm) * angle;
                        rz = (q.getZ() / norm) * angle;
                    }

                    success = move_to_target_urscript(
                        radius * std::cos(to_radian(angle)),
                        radius * std::sin(to_radian(angle)), dz, -rx, -ry, rz,
                        logger, urscript_node, robot_vel, robot_acc);
                    // rclcpp::sleep_for(std::chrono::milliseconds(3000));
                } else {
                    target_pose = move_group_interface.getCurrentPose().pose;
                    tf2::fromMsg(target_pose.orientation, target_q);
                    target_q = target_q * q;
                    target_pose.orientation = tf2::toMsg(target_q);
                    target_pose.position.x +=
                        radius * std::cos(to_radian(angle));
                    target_pose.position.y +=
                        radius * std::sin(to_radian(angle));
                    dz = (z_height - center[1]) / (50 * 1000.0);
                    target_pose.position.z += dz;
                    print_target(logger, target_pose);
                    move_group_interface.setPoseTarget(target_pose);
                    success = move_to_target(move_group_interface, logger);
                }
                if (!success) {
                    // msg = std::format("Angle Focus Planning Failed!");
                    msg = "Angle Focus Planning Failed!";
                    publisher_node->set_msg(msg);
                    RCLCPP_ERROR(logger, msg.c_str());
                }
            }

            if (angle_focused && z_focused) {
                angle_focused = false;
                z_focused = false;
                planning = false;
                end_state = true;
                msg += "\nWithin tolerance";
                publisher_node->set_msg(msg);
                publisher_node->set_end_state(end_state);
                move_group_interface.setStartStateToCurrentState();
                target_pose = move_group_interface.getCurrentPose().pose;
                while (subscriber_node->autofocus()) {
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }
            }
        } else {
            angle_increment = angle_limit / num_pt;
            roll = 0.0, pitch = 0.0, yaw = 0.0;

            if (next) {
                planning = true;
                yaw += to_radian(angle_increment);
                while (next == subscriber_node->next()) {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                }
            }
            if (previous) {
                planning = true;
                yaw += to_radian(-angle_increment);
                while (previous == subscriber_node->previous()) {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                }
            }
            if (home) {
                planning = true;
                yaw += to_radian(-angle);
                while (home == subscriber_node->home()) {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                }
            }
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);

            if (planning) {
                if (use_urscript) {
                    success = move_to_target_urscript(0, 0, 0, roll, pitch, yaw,
                                                      logger, urscript_node,
                                                      robot_vel, robot_acc);
                    rclcpp::sleep_for(std::chrono::milliseconds(4000));
                } else {
                    q.setRPY(roll, pitch, yaw);
                    q.normalize();
                    target_pose = move_group_interface.getCurrentPose().pose;
                    tf2::fromMsg(target_pose.orientation, target_q);
                    target_q = target_q * q;
                    target_pose.orientation = tf2::toMsg(target_q);
                    print_target(logger, target_pose);
                    move_group_interface.setPoseTarget(target_pose);
                    success = move_to_target(move_group_interface, logger);
                }
                if (success) {
                    msg = "Planning Success!";
                    RCLCPP_INFO(logger, msg.c_str());
                    if (next) {
                        angle += angle_increment;
                        circle_state++;
                    }
                    if (previous) {
                        angle -= angle_increment;
                        circle_state--;
                    }
                    if (home) {
                        circle_state = 1;
                        angle = 0.0;
                    }
                } else {
                    // msg = std::format("Z-axis Rotation Planning Failed! "
                    //                   "\nAttempt[{}] Retrying....");
                    msg = "Z-axis Rotation Planning Failed!";
                    RCLCPP_ERROR(logger, msg.c_str());
                    publisher_node->set_msg(msg);
                }
            }
        }

        if (planning) {
            planning = false;
            // while (!subscriber_node->changed()) {
            //     if (!subscriber_node->autofocus()) {
            //         rclcpp::sleep_for(std::chrono::milliseconds(10));
            //         break;
            //     }
            // }
            // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }

        if (scan_3d && !subscriber_node->autofocus()) {
            scan_3d = false;
            apply_config = true;
            publisher_node->set_scan_3d(scan_3d);
            while (subscriber_node->scan_3d() != scan_3d) {
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            publisher_node->set_apply_config(apply_config);
        }
    }
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
