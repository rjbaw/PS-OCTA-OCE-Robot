#include "octa_ros/action/focus.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "dds_subscriber.hpp"

using Focus = octa_ros::action::Focus;

class WatcherClientNode : public rclcpp::Node {

  public:
    using GoalHandleFocus = rclcpp_action::ClientGoalHandle<Focus>;

    WatcherClientNode() : Node("focus_client_node") {
        client_ = rclcpp_action::create_client<Focus>(this, "focus_action");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&WatcherClientNode::send_goal, this));
    }

    void send_goal() {
        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(),
                         "Focus action server not available");
            return;
        }

        auto goal_msg = Focus::Goal();
        goal_msg.angle_tolerance = 2.0; // your desired tolerance
        goal_msg.z_tolerance = 0.5;
        goal_msg.scan_3d = true;

        auto send_goal_options =
            rclcpp_action::Client<Focus>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](std::shared_future<GoalHandleFocus::SharedPtr> future) {
                auto goal_handle = future.get();
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(),
                                "Goal accepted by server, waiting for result");
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleFocus::SharedPtr,
                   const std::shared_ptr<const Focus::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(),
                            "Feedback: angle=%.2f height=%.2f msg=%s",
                            feedback->current_angle, feedback->current_height,
                            feedback->debug_msg.c_str());
            };

        send_goal_options.result_callback =
            [this](const GoalHandleFocus::WrappedResult &result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Result: %s",
                                result.result->result.c_str());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    break;
                }
            };

        // Send the goal
        auto goal_handle_future =
            client_->async_send_goal(goal_msg, send_goal_options);

        // Example: Cancel after 10s
        // This is how you'd do preemption if you want to cancel the current
        // goal.
        auto cancel_timer = this->create_wall_timer(
            std::chrono::seconds(10), [this, goal_handle_future]() {
                auto goal_handle = goal_handle_future.get();
                if (goal_handle) {
                    auto cancel_future =
                        client_->async_cancel_goal(goal_handle);
                    RCLCPP_INFO(this->get_logger(), "Sent cancel request");
                }
            });
    }

    // Cancel the currently active goal
    void cancel_goal() {
        RCLCPP_INFO(this->get_logger(), "Canceling goal...");
        // If you stored the goal handle from the response callback, you can
        // call: client_->async_cancel_goal(goal_handle) For demonstration, we
        // can do a "cancel all goals" request:
        client_->async_cancel_all_goals([this](auto /*unused*/) {
            RCLCPP_INFO(this->get_logger(), "All goals canceled");
        });
    }

  private:
    rclcpp_action::Client<Focus>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 3D Parameters
    const int interval = 4;
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
    double robot_vel, robot_acc, radius, angle_limit, dz;
    double z_tolerance, angle_tolerance, z_height;
    int num_pt;
    bool fast_axis = true;
    bool success = false;
    bool next = false;
    bool previous = false;
    bool home = false;
    bool z_focused = false;
    bool angle_focused = false;


    auto const logger = rclcpp::get_logger("logger_planning");

    auto subscriber_node = std::make_shared<dds_subscriber>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(subscriber_node);
    std::thread spinner([&executor]() { executor.spin(); });

    auto node = std::make_shared<WatcherClientNode>();

    while (rclcpp::ok() && running) {

        node->send_goal(10);


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
        }

        move_group_interface.setMaxVelocityScalingFactor(robot_vel);
        move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
        move_group_interface.setStartStateToCurrentState();

        // current_state = move_group_interface.getCurrentState(10.0);
        // bool is_valid = current_state->satisfiesBounds();
        // msg = std::format("Current Robot state is {}",
        //                   (is_valid ? "VALID" : "INVALID"));
        // RCLCPP_INFO(logger, msg.c_str());
        // publisher_node->set_msg(msg);

        // if (x_tol > 3.0 || y_tol > 3.0 || z_tol > 3.0 || path_enforce <= 0.0)
        // {
        //     end_state = true;
        //     msg = "Unrecoverable Error. Please restart";
        //     publisher_node->set_msg(msg);
        //     publisher_node->set_end_state(end_state);
        // }

        if (subscriber_node->reset()) {
            angle = 0.0;
            circle_state = 1;
            msg = "Reset to default position";
            RCLCPP_INFO(logger, msg.c_str());
            publisher_node->set_msg(msg);
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);
            move_group_interface.setJointValueTarget("shoulder_pan_joint",
                                                     to_radian(0.0));
            move_group_interface.setJointValueTarget("shoulder_lift_joint",
                                                     -to_radian(60.0));
            move_group_interface.setJointValueTarget("elbow_joint",
                                                     to_radian(90.0));
            move_group_interface.setJointValueTarget("wrist_1_joint",
                                                     to_radian(-120.0));
            move_group_interface.setJointValueTarget("wrist_2_joint",
                                                     to_radian(-90.0));
            move_group_interface.setJointValueTarget("wrist_3_joint",
                                                     to_radian(-135.0));
            attempt = 0;
            success = false;
            while (!success && (attempt < max_attempt)) {
                success = move_to_target(move_group_interface, logger);
                if (!success) {
                    msg = std::format(
                        "Reset Planning Failed! \nAttempt[{}] Retrying....",
                        attempt);
                    publisher_node->set_msg(msg);
                    x_tol += 0.1;
                    y_tol += 0.1;
                    z_tol += 0.1;
                    path_enforce -= 0.1;
                    ocm.absolute_x_axis_tolerance = x_tol;
                    ocm.absolute_y_axis_tolerance = y_tol;
                    ocm.absolute_z_axis_tolerance = z_tol;
                    ocm.weight = path_enforce;
                    path_constr.orientation_constraints.clear();
                    path_constr.orientation_constraints.push_back(ocm);
                    move_group_interface.setPathConstraints(path_constr);
                    attempt++;
                }
            }
            if (!success) {
                msg = std::format(
                    "Reset Planning Failed! Maximum Attempts Reached", attempt);
                publisher_node->set_msg(msg);
                RCLCPP_ERROR(logger, msg.c_str());
            }
        }

        if (subscriber_node->autofocus()) {
            apply_config = true;
            publisher_node->set_apply_config(apply_config);
            if (!scan_3d) {
                scan_3d = true;
                apply_config = true;
                // msg = "Starting 3D Scan";
                publisher_node->set_msg(msg);
                publisher_node->set_scan_3d(scan_3d);
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
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
                    rclcpp::sleep_for(std::chrono::milliseconds(10000));
                }
                img_array.push_back(img);
                RCLCPP_INFO(logger, "Collected image %d", i + 1);
            }
            msg = "Calculating Rotations";
            publisher_node->set_msg(msg);
            // scan_3d = false;
            // apply_config = true;
            // publisher_node->set_scan_3d(scan_3d);
            // publisher_node->set_apply_config(apply_config);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));
            // publisher_node->set_apply_config(apply_config);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));
            // publisher_node->set_apply_config(apply_config);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));
            // publisher_node->set_apply_config(apply_config);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));

            pc_lines = lines_3d(img_array, interval, single_interval);
            open3d::geometry::PointCloud pcd_lines;
            for (const auto &point : pc_lines) {
                pcd_lines.points_.emplace_back(point);
            }
            auto boundbox = pcd_lines.GetMinimalOrientedBoundingBox(false);
            Eigen::Vector3d center = boundbox.GetCenter();
            z_height = subscriber_node->z_height();
            msg = std::format("Center: {}", center[1]);
            publisher_node->set_msg(msg.c_str());
            RCLCPP_INFO(logger, "center: %f", center[1]);
            rotmat_eigen = align_to_direction(boundbox.R_);
            tf2::Matrix3x3 rotmat_tf(
                rotmat_eigen(0, 0), rotmat_eigen(0, 1), rotmat_eigen(0, 2),
                rotmat_eigen(1, 0), rotmat_eigen(1, 1), rotmat_eigen(1, 2),
                rotmat_eigen(2, 0), rotmat_eigen(2, 1), rotmat_eigen(2, 2));
            RCLCPP_INFO_STREAM(logger, "\nAligned Rotation Matrix:\n"
                                           << rotmat_eigen);
            double tmp_roll;
            double tmp_pitch;
            double tmp_yaw;
            rotmat_tf.getRPY(tmp_roll, tmp_pitch, tmp_yaw);
            roll = tmp_yaw;
            pitch = tmp_roll;
            yaw = tmp_pitch;
            msg =
                std::format("Calculated R:{:.2f}, P:{:.2f}, Y:{:.2f}",
                            to_degree(roll), to_degree(pitch), to_degree(yaw));
            RCLCPP_INFO(logger, msg.c_str());
            publisher_node->set_msg(msg);
            rotmat_tf.setRPY(roll, pitch, yaw);

            if (tol_measure(roll, pitch, angle_tolerance)) {
                angle_focused = true;
                msg += "\nAngle focused";
                publisher_node->set_msg(msg);
                scan_3d = false;
                apply_config = true;
                publisher_node->set_scan_3d(scan_3d);
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }

            if (angle_focused && !z_focused) {
                // dz = 0;
                dz = -(center[1] - z_height) / 150 * 1.2 / 1000.0;
                // dz = (z_height - center[1]) / (256.0 * 1000.0);
                msg += std::format("\ndz = {}", dz).c_str();
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, "dz: %f", dz);
                if (std::abs(dz) < (z_tolerance / 1000.0)) {
                    z_focused = true;
                    msg += "\nHeight focused";
                    publisher_node->set_msg(msg);
                } else {
                    planning = true;
                    target_pose = move_group_interface.getCurrentPose().pose;
                    target_pose.position.z += dz;
                    print_target(logger, target_pose);
                    move_group_interface.setPoseTarget(target_pose);
                    attempt = 0;
                    success = false;
                    while (!success && (attempt < max_attempt)) {
                        success = move_to_target(move_group_interface, logger);
                        if (success) {
                            RCLCPP_INFO(logger, msg.c_str());
                        } else {
                            msg = std::format("Z-height Planning Failed! "
                                              "\nAttempt[{}] Retrying....",
                                              attempt);
                            RCLCPP_ERROR(logger, msg.c_str());
                            publisher_node->set_msg(msg);
                            x_tol += 0.1;
                            y_tol += 0.1;
                            z_tol += 0.1;
                            path_enforce -= 0.1;
                            ocm.absolute_x_axis_tolerance = x_tol;
                            ocm.absolute_y_axis_tolerance = y_tol;
                            ocm.absolute_z_axis_tolerance = z_tol;
                            ocm.weight = path_enforce;
                            path_constr.orientation_constraints.clear();
                            path_constr.orientation_constraints.push_back(ocm);
                            move_group_interface.setPathConstraints(
                                path_constr);
                            attempt++;
                        }
                    }
                    if (!success) {
                        msg = std::format("Z-height Planning Failed! Maximum "
                                          "Attempts Reached",
                                          attempt);
                        publisher_node->set_msg(msg);
                        RCLCPP_ERROR(logger, msg.c_str());
                    }
                }
            }

            if (!angle_focused) {
                planning = true;
                rotmat_tf.getRotation(q);
                target_pose = move_group_interface.getCurrentPose().pose;
                tf2::fromMsg(target_pose.orientation, target_q);
                q.normalize();
                target_q = target_q * q;
                target_pose.orientation = tf2::toMsg(target_q);
                target_pose.position.x += radius * std::cos(to_radian(angle));
                target_pose.position.y += radius * std::sin(to_radian(angle));
                target_pose.position.z += dz;
                print_target(logger, target_pose);
                move_group_interface.setPoseTarget(target_pose);

                attempt = 0;
                success = false;
                while (!success && (attempt < max_attempt)) {
                    success = move_to_target(move_group_interface, logger);
                    if (success) {
                        RCLCPP_INFO(logger, msg.c_str());
                    } else {
                        msg = std::format("Angle Focus Planning Failed! "
                                          "\nAttempt[{}] Retrying....",
                                          attempt);
                        RCLCPP_ERROR(logger, msg.c_str());
                        publisher_node->set_msg(msg);
                    }
                    attempt++;
                }
                if (!success) {
                    msg = std::format(
                        "Angle Focus Planning Failed! Maximum Attempts Reached",
                        attempt);
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
            }
            if (previous) {
                planning = true;
                yaw += to_radian(-angle_increment);
            }
            if (home) {
                planning = true;
                yaw += to_radian(-angle);
            }
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);

            if (planning) {
                target_pose = move_group_interface.getCurrentPose().pose;
                tf2::fromMsg(target_pose.orientation, target_q);
                q.setRPY(roll, pitch, yaw);
                q.normalize();
                target_q = target_q * q;
                target_pose.orientation = tf2::toMsg(target_q);
                print_target(logger, target_pose);
                move_group_interface.setPoseTarget(target_pose);

                attempt = 0;
                success = false;
                while (!success && (attempt < max_attempt)) {
                    success = move_to_target(move_group_interface, logger);
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
                        msg = std::format("Z-axis Rotation Planning Failed! "
                                          "\nAttempt[{}] Retrying....",
                                          attempt);
                        RCLCPP_ERROR(logger, msg.c_str());
                        publisher_node->set_msg(msg);
                        x_tol += 0.1;
                        y_tol += 0.1;
                        z_tol += 0.1;
                        path_enforce -= 0.1;
                        ocm.absolute_x_axis_tolerance = x_tol;
                        ocm.absolute_y_axis_tolerance = y_tol;
                        ocm.absolute_z_axis_tolerance = z_tol;
                        ocm.weight = path_enforce;
                        path_constr.orientation_constraints.push_back(ocm);
                        move_group_interface.setPathConstraints(path_constr);
                    }
                    attempt++;
                }
                if (!success) {
                    msg = std::format("Z-axis Rotation Planning Failed! "
                                      "Maximum Attempts Reached",
                                      attempt);
                    publisher_node->set_msg(msg);
                    RCLCPP_ERROR(logger, msg.c_str());
                }
            }
        }

        if (planning) {
            planning = false;
            while (!subscriber_node->changed()) {
                if (!subscriber_node->autofocus()) {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                    break;
                }
            }
            // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }

        if (subscriber_node->scan_3d() && !subscriber_node->autofocus()) {
            scan_3d = false;
            apply_config = true;
            publisher_node->set_scan_3d(scan_3d);
            publisher_node->set_apply_config(apply_config);
        }
    }

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
