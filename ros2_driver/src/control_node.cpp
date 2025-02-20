#include "dds_subscriber.hpp"
#include "octa_ros/action/focus.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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

bool tol_measure(double &roll, double &pitch, double &angle_tolerance) {
    return ((std::abs(std::abs(roll)) < to_radian(angle_tolerance)) &&
            (std::abs(std::abs(pitch)) < to_radian(angle_tolerance)));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

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
    double angle_increment;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

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

    auto subscriber_node = std::make_shared<dds_subscriber>();
    auto publisher_node = std::make_shared<dds_publisher>(
        msg, angle, circle_state, fast_axis, apply_config, end_state, scan_3d);

    auto const logger = rclcpp::get_logger("logger_planning");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);
    std::thread spinner([&executor]() { executor.spin(); });

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

        if (subscriber_node->freedrive()) {
            circle_state = 1;
            angle = 0.0;
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);

            // action server handle freedrive
            // switch controller to freedrive_mode_controller
            // or send program to program to urscript service
            continue;
        }

        if (subscriber_node->reset()) {
            angle = 0.0;
            circle_state = 1;
            msg = "Reset to default position";
            RCLCPP_INFO(logger, msg.c_str());
            publisher_node->set_msg(msg);
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);

            // action server handle move to default position

            continue;
        }

        if (subscriber_node->autofocus()) {

            if (!scan_3d) {
                // activate 3D if not already enabled
                scan_3d = true;
                apply_config = true;
                msg = "Starting 3D Scan";
                RCLCPP_INFO(logger, msg.c_str());
                publisher_node->set_msg(msg);
                publisher_node->set_scan_3d(scan_3d);
                publisher_node->set_apply_config(apply_config);
            }

            // action server?
            // capture images to array according to interval and boolean
            // wrap-around

            msg = "Images captured";
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());

            // deactivate 3D scan after capture
            scan_3d = false;
            apply_config = true;
            publisher_node->set_scan_3d(scan_3d);
            publisher_node->set_apply_config(apply_config);

            // action server?
            // calculate rotation matrix and center[1], given image array

            rotmat_tf.setRPY(roll, pitch, yaw);
            msg +=
                std::format("\nCalculated R:{:.2f}, P:{:.2f}, Y:{:.2f}",
                            to_degree(roll), to_degree(pitch), to_degree(yaw));
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());

            // calculate angle tolerance
            if (tol_measure(roll, pitch, angle_tolerance)) {
                angle_focused = true;
                msg += "\nAngle focused";
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, msg.c_str());
                scan_3d = false;
                apply_config = true;
                publisher_node->set_scan_3d(scan_3d);
                publisher_node->set_apply_config(apply_config);
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

            // move end effector to target if not within tolerance bounds

            if (!angle_focused) {
                planning = true;
                // action server rotate end effector according to rotation
                // matrix
                if (!success) {
                    // msg = std::format("Angle Focus Planning Failed!");
                    msg = "Angle Focus Planning Failed!";
                    publisher_node->set_msg(msg);
                    RCLCPP_ERROR(logger, msg.c_str());
                }
                continue;
            } else {

                // calculate dz
                dz = (z_height - center[1]) / (50 * 1000.0);
                msg += std::format("\ndz = {}", dz);
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, "dz: %f", dz);
                // check tolerance and move it is not within bounds
                if (std::abs(dz) < (z_tolerance / 1000.0)) {
                    z_focused = true;
                    msg += "\nHeight focused";
                    publisher_node->set_msg(msg);
                    RCLCPP_INFO(logger, msg.c_str());
                } else {
                    planning = true;
                    if (use_urscript) {
                        success = move_to_target_urscript(0, 0, dz, 0, 0, 0,
                                                          logger, urscript_node,
                                                          robot_vel, robot_acc);
                        rclcpp::sleep_for(std::chrono::milliseconds(3000));
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

            continue;
        }

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
            if (use_urscript) {
                success = move_to_target_urscript(0, 0, 0, roll, pitch, yaw,
                                                  logger, urscript_node,
                                                  robot_vel, robot_acc);
                rclcpp::sleep_for(std::chrono::milliseconds(3000));
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

        if (scan_3d && !subscriber_node->autofocus()) {
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
}
