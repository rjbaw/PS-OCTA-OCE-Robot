#include <chrono>
#include <cmath>
#include <format>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <octa_ros/msg/labviewdata.hpp>
#include <octa_ros/msg/robotdata.hpp>

#include <octa_ros/action/focus.hpp>
#include <octa_ros/action/freedrive.hpp>
#include <octa_ros/action/move_z_angle.hpp>
#include <octa_ros/action/reset.hpp>

static double to_radian(double deg) { return deg * M_PI / 180.0; }

class ControlNode : public rclcpp::Node {
  public:
    using Focus = octa_ros::action::Focus;
    using MoveZAngle = octa_ros::action::MoveZAngle;
    using Freedrive = octa_ros::action::Freedrive;
    using ResetAction = octa_ros::action::Reset;

    using FocusGoalHandle = rclcpp_action::ClientGoalHandle<Focus>;
    using MoveZGoalHandle = rclcpp_action::ClientGoalHandle<MoveZAngle>;
    using FreedriveGoalHandle = rclcpp_action::ClientGoalHandle<Freedrive>;
    using ReetGoalHandle = rclcpp_action::ClientGoalHandle<ResetAction>;

    explicit ControlNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("control_node", options) {
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            robotdata_pub_ = this->create_publisher<octa_ros::msg::Robotdata>(
                "robot_data", qos);
        }

        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            labview_read_ =
                this->create_subscription<octa_ros::msg::Labviewdata>(
                    "labview_data", qos,
                    std::bind(&ControlNode::labviewCallback, this,
                              std::placeholders::_1));
        }

        robotdata_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ControlNode::publishRobotdata, this));

        focus_action_client_ =
            rclcpp_action::create_client<Focus>(this, "focus_action");
        move_z_angle_client_ = rclcpp_action::create_client<MoveZAngle>(
            this, "move_z_angle_action");
        freedrive_action_client_ =
            rclcpp_action::create_client<Freedrive>(this, "freedrive_action");
        reset_action_client_ =
            rclcpp_action::create_client<ResetAction>(this, "reset_action");

        main_loop_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&ControlNode::mainLoop, this));

        RCLCPP_INFO(get_logger(), "ControlNode constructed and ready.");
    }

  private:
    rclcpp_action::Client<Focus>::SharedPtr focus_action_client_;
    rclcpp_action::Client<MoveZAngle>::SharedPtr move_z_angle_client_;
    rclcpp_action::Client<Freedrive>::SharedPtr freedrive_action_client_;
    rclcpp_action::Client<ResetAction>::SharedPtr reset_action_client_;

    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr robotdata_pub_;
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr labview_read_;

    rclcpp::TimerBase::SharedPtr robotdata_timer_;
    rclcpp::TimerBase::SharedPtr main_loop_timer_;

    FocusGoalHandle::SharedPtr active_focus_goal_handle_;
    MoveZGoalHandle::SharedPtr active_move_z_goal_handle_;
    FreedriveGoalHandle::SharedPtr active_freedrive_goal_handle_;
    ResetGoalHandle::SharedPtr active_reset_goal_handle_;

    //     // 3D Parameters
    //     const int interval = 6;
    //     const bool single_interval = false;
    //
    //     // Publisher Parameters
    //     std::string msg;
    //     double angle = 0.0;
    //     int circle_state = 1;
    //     bool apply_config = true;
    //     bool end_state = false;
    //     bool scan_3d = false;
    //
    //     // Internal Parameters
    //     bool planning = false;
    //     double angle_increment;
    //     double roll = 0.0, pitch = 0.0, yaw = 0.0;
    //     Eigen::Matrix3d rotmat_eigen;
    //     cv::Mat img;
    //     std::vector<cv::Mat> img_array;
    //     std::vector<Eigen::Vector3d> pc_lines;
    //     tf2::Quaternion q;
    //     tf2::Quaternion target_q;
    //     geometry_msgs::msg::Pose target_pose;
    //
    //     // Subscriber Parameters
    //     double robot_vel;
    //     double robot_acc;
    //     double radius;
    //     double angle_limit;
    //     double dz;
    //     double z_tolerance;
    //     double angle_tolerance;
    //     double z_height;
    //     int num_pt;
    //     bool fast_axis = true;
    //     bool success = false;
    //     bool next = false;
    //     bool previous = false;
    //     bool home = false;
    //     bool z_focused = false;
    //     bool angle_focused = false;

    // Robotdata fields
    std::string msg_{"idle"};
    double angle_{0.0};
    int circle_state_{1};
    bool fast_axis_{true};
    bool apply_config_{false};
    bool end_state_{false};
    bool scan_3d_{false};

    //         move_group_interface.setMaxVelocityScalingFactor(robot_vel);
    //         move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
    //         move_group_interface.setStartStateToCurrentState();

    std::chrono::steady_clock::time_point apply_config_time_{
        std::chrono::steady_clock::now()};
    octa_ros::msg::Robotdata old_robotdata_;

    // Labview fields
    std::mutex data_mutex_;
    double robot_vel_{0.5};
    double robot_acc_{0.5};
    double z_tolerance_{0.0};
    double angle_tolerance_{0.0};
    double radius_{0.0};
    double angle_limit_{0.0};
    double dz_{0.0};
    double drot_{0.0};
    bool autofocus_{false};
    bool freedrive_{false};
    bool previous_{false};
    bool next_{false};
    bool home_{false};
    bool reset_{false};
    bool fast_axis_read_{false};
    bool scan_3d_read_{false};
    double z_height_{0.0};
    int num_pt_{1};

    bool changed_{false};
    octa_ros::msg::Labviewdata old_labview_msg_;

    double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
    double angle_increment_{0.0};

    void labviewCallback(const octa_ros::msg::Labviewdata::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        robot_vel_ = msg->robot_vel;
        robot_acc_ = msg->robot_acc;
        z_tolerance_ = msg->z_tolerance;
        angle_tolerance_ = msg->angle_tolerance;
        radius_ = msg->radius;
        angle_limit_ = msg->angle_limit;
        num_pt_ = msg->num_pt;
        dz_ = msg->dz;
        drot_ = msg->drot;
        autofocus_ = msg->autofocus;
        freedrive_ = msg->freedrive;
        previous_ = msg->previous;
        next_ = msg->next;
        home_ = msg->home;
        reset_ = msg->reset;
        fast_axis_read_ = msg->fast_axis;
        scan_3d_read_ = msg->scan_3d;
        z_height_ = msg->z_height;

        if (*msg != old_labview_msg_) {
            changed_ = true;
            RCLCPP_INFO(get_logger(),
                        std::format("[SUBSCRIBING]  robot_vel: {}, robot_acc: "
                                    "{}, z_tolerance: {}, "
                                    "angle_tolerance: {}, radius: {}, "
                                    "angle_limit: {}, num_pt: {}, "
                                    "dz: {}, drot: {}, autofocus: {}, "
                                    "freedrive: {}, previous: {}, "
                                    "next: {}, home: {}, reset: {}, fast_axis: "
                                    "{}, scan_3d: {}, z_height: {}",
                                    robot_vel_, robot_acc_, z_tolerance_,
                                    angle_tolerance_, radius_, angle_limit_,
                                    num_pt_, dz_, drot_, autofocus_, freedrive_,
                                    previous_, next_, home_, reset_,
                                    fast_axis_read_, scan_3d_read_, z_height_)
                            .c_str());
        } else {
            changed_ = false;
        }
        old_labview_msg_ = *msg;
    }

    void publishRobotdata() {
        octa_ros::msg::Robotdata message;
        message.msg = msg_;
        message.angle = angle_;
        message.circle_state = circle_state_;
        message.fast_axis = fast_axis_;
        message.apply_config = apply_config_;
        message.end_state = end_state_;
        message.scan_3d = scan_3d_;

        if (message != old_robotdata_) {
            RCLCPP_INFO(get_logger(),
                        "[PUBLISHING] msg: %s, angle: %.2f, circle_state: %d, "
                        "fast_axis: %s, "
                        "apply_config: %s, end_state: %s, scan_3d: %s",
                        message.msg.c_str(), message.angle,
                        message.circle_state,
                        (message.fast_axis ? "true" : "false"),
                        (message.apply_config ? "true" : "false"),
                        (message.end_state ? "true" : "false"),
                        (message.scan_3d ? "true" : "false"));
        }

        robotdata_pub_->publish(message);

        if (apply_config_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsedms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - apply_config_time_)
                    .count();
            if (elapsedms >= 300) {
                apply_config_ = false;
            }
        }

        old_robotdata_ = message;
    }

    void mainLoop() {
        // Wait for each server briefly (non-blocking style).
        if (!focus_action_client_->wait_for_action_server(
                std::chrono::milliseconds(200))) {
            RCLCPP_WARN(get_logger(), "Focus action server not available yet.");
        }
        if (!move_z_angle_client_->wait_for_action_server(
                std::chrono::milliseconds(200))) {
            RCLCPP_WARN(get_logger(),
                        "MoveZAngle action server not available yet.");
        }
        if (!freedrive_action_client_->wait_for_action_server(
                std::chrono::milliseconds(200))) {
            RCLCPP_WARN(get_logger(),
                        "Freedrive action server not available yet.");
        }
        if (!reset_action_client_->wait_for_action_server(
                std::chrono::milliseconds(200))) {
            RCLCPP_WARN(get_logger(), "Reset action server not available yet.");
        }

        std::lock_guard<std::mutex> lock(data_mutex_);

        if (freedrive_) {
            circle_state_ = 1;
            angle_ = 0.0;
            msg_ = "[Action] Freedrive Mode ON";
            RCLCPP_INFO(get_logger(), msg.c_str());
            sendFreedriveGoal(true);
        } else {
            sendFreedriveGoal(false);
        }

        if (reset_) {
            RCLCPP_INFO(get_logger(), "User RESET => sending Reset action");
            angle_ = 0.0;
            circle_state_ = 1;
            msg_ = "[Action] Reset to default position. It may take some time "
                   "please wait.";
            RCLCPP_INFO(get_logger(), msg.c_str());
            // sendResetGoal(robot_vel_, robot_acc_);
            sendResetGoal(0.8, 0.8);
        }

        if (autofocus_) {
            RCLCPP_INFO(get_logger(), "User AUTOFOCUS => sending Focus goal");
            scan_3d_ = true;
            apply_config_ = true;
            msg_ = "Starting 3D Scan";
            RCLCPP_INFO(get_logger, msg_.c_str());
            while (!scan_3d_read_) {
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }

            sendSurfaceGoal();

            scan_3d_ = false;
            apply_config_ = true;
            while (scan_3d_read_) {
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }

            //             rotmat_tf.getRPY(tmp_roll, tmp_pitch, tmp_yaw);
            //             roll = tmp_yaw;
            //             pitch = -tmp_roll;
            //             yaw = tmp_pitch;
            //             rotmat_tf.setRPY(roll, pitch, yaw);

            //             msg +=
            //                 std::format("\nCalculated R:{:.2f}, P:{:.2f},
            //                 Y:{:.2f}",
            //                             to_degree(roll), to_degree(pitch),
            //                             to_degree(yaw));
            //             publisher_node->set_msg(msg);
            //             RCLCPP_INFO(logger, msg.c_str());

            //             if (tol_measure(roll, pitch, angle_tolerance)) {
            //                 angle_focused = true;
            //                 msg += "\nAngle focused";
            //                 publisher_node->set_msg(msg);
            //                 RCLCPP_INFO(logger, msg.c_str());
            //                 scan_3d = false;
            //                 apply_config = true;
            //                 publisher_node->set_scan_3d(scan_3d);
            //                 while (subscriber_node->scan_3d() != scan_3d) {
            //                     rclcpp::sleep_for(std::chrono::milliseconds(50));
            //                 }
            //                 publisher_node->set_apply_config(apply_config);
            //             }

            sendFocusGoal();

            return;
        }

        //         if (subscriber_node->autofocus()) {
        //
        //             if (angle_focused && !z_focused) {
        //                 // dz = 0;
        //                 dz = (z_height - center[1]) / (50 * 1000.0);
        //                 msg += std::format("\ndz = {}", dz);
        //                 publisher_node->set_msg(msg);
        //                 RCLCPP_INFO(logger, "dz: %f", dz);
        //                 if (std::abs(dz) < (z_tolerance / 1000.0)) {
        //                     z_focused = true;
        //                     msg += "\nHeight focused";
        //                     publisher_node->set_msg(msg);
        //                     RCLCPP_INFO(logger, msg.c_str());
        //                 } else {
        //                     planning = true;
        //                     if (use_urscript) {
        //                         success = move_to_target_urscript(0, 0, -dz,
        //                         0, 0, 0,
        //                                                           logger,
        //                                                           urscript_node,
        //                                                           robot_vel,
        //                                                           robot_acc);
        //                         //
        //                         rclcpp::sleep_for(std::chrono::milliseconds(3000));
        //                     } else {
        //                         target_pose =
        //                             move_group_interface.getCurrentPose().pose;
        //                         target_pose.position.z += dz;
        //                         print_target(logger, target_pose);
        //                         move_group_interface.setPoseTarget(target_pose);
        //                         success =
        //                         move_to_target(move_group_interface, logger);
        //                     }
        //
        //                     if (!success) {
        //                         msg = std::format("Z-height Planning
        //                         Failed!"); RCLCPP_ERROR(logger, msg.c_str());
        //                         publisher_node->set_msg(msg);
        //                     }
        //                 }
        //             }
        //
        //             if (!angle_focused) {
        //                 planning = true;
        //                 rotmat_tf.getRotation(q);
        //                 q.normalize();
        //                 if (use_urscript) {
        //                     double angle = 2.0 * std::acos(q.getW());
        //                     double norm =
        //                         std::sqrt(q.getX() * q.getX() + q.getY() *
        //                         q.getY() +
        //                                   q.getZ() * q.getZ());
        //                     double rx = 0, ry = 0, rz = 0;
        //                     if (norm < 1e-8) {
        //                         rx = ry = rz = 0.0;
        //                     } else {
        //                         rx = (q.getX() / norm) * angle;
        //                         ry = (q.getY() / norm) * angle;
        //                         rz = (q.getZ() / norm) * angle;
        //                     }
        //
        //                     success = move_to_target_urscript(
        //                         radius * std::cos(to_radian(angle)),
        //                         radius * std::sin(to_radian(angle)), dz, -rx,
        //                         -ry, rz, logger, urscript_node, robot_vel,
        //                         robot_acc);
        //                     //
        //                     rclcpp::sleep_for(std::chrono::milliseconds(3000));
        //                 } else {
        //                     target_pose =
        //                     move_group_interface.getCurrentPose().pose;
        //                     tf2::fromMsg(target_pose.orientation, target_q);
        //                     target_q = target_q * q;
        //                     target_pose.orientation = tf2::toMsg(target_q);
        //                     target_pose.position.x +=
        //                         radius * std::cos(to_radian(angle));
        //                     target_pose.position.y +=
        //                         radius * std::sin(to_radian(angle));
        //                     dz = (z_height - center[1]) / (50 * 1000.0);
        //                     target_pose.position.z += dz;
        //                     print_target(logger, target_pose);
        //                     move_group_interface.setPoseTarget(target_pose);
        //                     success = move_to_target(move_group_interface,
        //                     logger);
        //                 }
        //                 if (!success) {
        //                     msg = "Angle Focus Planning Failed!";
        //                     publisher_node->set_msg(msg);
        //                     RCLCPP_ERROR(logger, msg.c_str());
        //                 }
        //             }
        //
        //             if (angle_focused && z_focused) {
        //                 angle_focused = false;
        //                 z_focused = false;
        //                 planning = false;
        //                 end_state = true;
        //                 msg += "\nWithin tolerance";
        //                 publisher_node->set_msg(msg);
        //                 publisher_node->set_end_state(end_state);
        //                 move_group_interface.setStartStateToCurrentState();
        //                 target_pose =
        //                 move_group_interface.getCurrentPose().pose; while
        //                 (subscriber_node->autofocus()) {
        //                     rclcpp::sleep_for(std::chrono::milliseconds(50));
        //                 }
        //             }
        //         }

        //             angle_increment = angle_limit / num_pt;
        //             roll = 0.0, pitch = 0.0, yaw = 0.0;

        angle_increment_ = (num_pt_ == 0)
                               ? 0.0
                               : (angle_limit_ / static_cast<double>(num_pt_));
        if (next_) {
            yaw_ += to_radian(angle_increment_);
            circle_state_++;
            RCLCPP_INFO(get_logger(),
                        "NEXT => new yaw=%.2f deg => sending MoveZAngle goal",
                        yaw_ * 180.0 / M_PI);
            sendMoveZAngleGoal(yaw_, robot_vel_, robot_acc_);
            while (next_) {
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }
        if (previous_) {
            yaw_ -= to_radian(angle_increment_);
            circle_state_--;
            RCLCPP_INFO(
                get_logger(),
                "PREVIOUS => new yaw=%.2f deg => sending MoveZAngle goal",
                yaw_ * 180.0 / M_PI);
            sendMoveZAngleGoal(yaw_, robot_vel_, robot_acc_);
            while (previous_) {
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }
        if (home_) {
            yaw_ -= to_radian(angle_);
            circle_state_ = 1;
            angle_ = 0.0;
            RCLCPP_INFO(get_logger(),
                        "HOME => revert yaw by %.2f deg => new yaw=%.2f => "
                        "sending MoveZAngle goal",
                        angle_, yaw_ * 180.0 / M_PI);
            sendMoveZAngleGoal(yaw_, robot_vel_, robot_acc_);
            while (home) {
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    void sendFocusGoal() {
        Focus::Goal goal_msg;
        goal_msg.angle_tolerance = angle_tolerance_;
        goal_msg.z_tolerance = z_tolerance_;

        auto options = rclcpp_action::Client<Focus>::SendGoalOptions();
        options.result_callback =
            [this](const FocusGoalHandle::WrappedResult &result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Focus action SUCCEEDED");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Focus action ABORTED");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Focus action CANCELED");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(),
                                "Focus action UNKNOWN result code");
                    break;
                }
            };

        options.feedback_callback =
            [this](FocusGoalHandle::SharedPtr,
                   const std::shared_ptr<const Focus::Feedback> fb) {
                // Display feedback in real-time (current progress)
                RCLCPP_INFO(this->get_logger(),
                            "Focus progress => current_angle=%.2f",
                            fb->current_angle);
                // Optionally, implement cancellation on the client side
                if (/* condition to cancel */) {
                    RCLCPP_INFO(this->get_logger(), "Canceling Focus action");
                    focus_action_client_->cancel_all_goals();
                }
            };

        focus_action_client_->async_send_goal(goal_msg, options);
    }

    void sendMoveZAngleGoal(double yaw_rad) {
        MoveZAngle::Goal goal_msg;
        goal_msg.target_angle = yaw_rad * 180.0 / M_PI;

        auto options = rclcpp_action::Client<MoveZAngle>::SendGoalOptions();
        options.result_callback =
            [this](const MoveZGoalHandle::WrappedResult &result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "MoveZAngle SUCCEEDED");
                    // msg = "Planning Success!";
                    // RCLCPP_INFO(logger, msg.c_str());

                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "MoveZAngle ABORTED");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "MoveZAngle CANCELED");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "MoveZAngle UNKNOWN code");
                    break;
                }
            };

        options.feedback_callback =
            [this](MoveZGoalHandle::SharedPtr,
                   const std::shared_ptr<const MoveZAngle::Feedback> fb) {
                RCLCPP_INFO(this->get_logger(),
                            "MoveZAngle feedback => current_z=%.2f",
                            fb->current_z_angle);
            };

        move_z_angle_client_->async_send_goal(goal_msg, options);
    }

    void sendFreedriveGoal(bool enable) {
        Freedrive::Goal goal_msg;
        goal_msg.enable = enable;

        auto options = rclcpp_action::Client<Freedrive>::SendGoalOptions();
        options.result_callback =
            [this](const FreedriveGoalHandle::WrappedResult &result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(),
                                "Freedrive result => success=%d",
                                result.result->success);
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Freedrive ABORTED");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Freedrive CANCELED");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "Freedrive UNKNOWN code");
                    break;
                }
            };

        options.feedback_callback =
            [this](FreedriveGoalHandle::SharedPtr,
                   const std::shared_ptr<const Freedrive::Feedback> fb) {
                RCLCPP_INFO(this->get_logger(), "Freedrive feedback => %s",
                            fb->status.c_str());
            };

        freedrive_action_client_->async_send_goal(goal_msg, options);
    }

    void sendResetGoal() {
        ResetAction::Goal goal_msg;
        goal_msg.reset = true; // or if your action doesn't need fields, omit

        auto options = rclcpp_action::Client<ResetAction>::SendGoalOptions();
        options.result_callback =
            [this](const ResetGoalHandle::WrappedResult &result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(),
                                "Reset result => success=%d",
                                result.result->success);
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Reset ABORTED");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Reset CANCELED");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "Reset UNKNOWN code");
                    break;
                }
            };

        options.feedback_callback =
            [this](ResetGoalHandle::SharedPtr,
                   const std::shared_ptr<const ResetAction::Feedback> fb) {
                RCLCPP_INFO(this->get_logger(), "Reset feedback => %s",
                            fb->status.c_str());
            };

        reset_action_client_->async_send_goal(goal_msg, options);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
