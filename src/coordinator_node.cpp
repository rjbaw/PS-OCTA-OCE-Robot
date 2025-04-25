/**
 * @file coordinator_node.cpp
 * @author rjbaw
 * @brief coordinator node that executes action servers, subscribe, and publish
 * to LabView interface.
 * Actions are toggled using rising edge to prevent multiple triggers.
 */

#include "octa_ros/msg/cancel_action.hpp"
#include <chrono>
#include <cmath>
#include <format>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <octa_ros/msg/cancel_action.hpp>
#include <octa_ros/msg/labviewdata.hpp>
#include <octa_ros/msg/robotdata.hpp>

#include <octa_ros/action/focus.hpp>
#include <octa_ros/action/freedrive.hpp>
#include <octa_ros/action/move_z_angle.hpp>
#include <octa_ros/action/reset.hpp>

#include <octa_ros/srv/activate3_dscan.hpp>
#include <octa_ros/srv/deactivate3_dscan.hpp>
#include <octa_ros/srv/deactivate_focus.hpp>

enum class UserAction {
    None,
    Freedrive,
    Reset,
    MoveZangle,
    Focus,
};

double to_radian_(const double degree) {
    return (std::numbers::pi / 180 * degree);
}

class CoordinatorNode : public rclcpp::Node {
  public:
    using Focus = octa_ros::action::Focus;
    using MoveZAngle = octa_ros::action::MoveZAngle;
    using Freedrive = octa_ros::action::Freedrive;
    using Reset = octa_ros::action::Reset;

    using FocusGoalHandle = rclcpp_action::ClientGoalHandle<Focus>;
    using MoveZGoalHandle = rclcpp_action::ClientGoalHandle<MoveZAngle>;
    using FreedriveGoalHandle = rclcpp_action::ClientGoalHandle<Freedrive>;
    using ResetGoalHandle = rclcpp_action::ClientGoalHandle<Reset>;

    using Activate3Dscan = octa_ros::srv::Activate3Dscan;
    using Deactivate3Dscan = octa_ros::srv::Deactivate3Dscan;
    using DeactivateFocus = octa_ros::srv::DeactivateFocus;

    explicit CoordinatorNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("coordinator_node",
               // options
               rclcpp::NodeOptions(options)
                   .automatically_declare_parameters_from_overrides(true)) {}

    void init() {
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            pub_handle_ = this->create_publisher<octa_ros::msg::Robotdata>(
                "robot_data", qos);
        }

        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            sub_handle_ = this->create_subscription<octa_ros::msg::Labviewdata>(
                "labview_data", qos,
                std::bind(&CoordinatorNode::subscriberCallback, this,
                          std::placeholders::_1));
        }
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            cancel_handle_ =
                this->create_subscription<octa_ros::msg::CancelAction>(
                    "cancel_current_action", qos,
                    std::bind(&CoordinatorNode::cancelCallback, this,
                              std::placeholders::_1));
        }

        {
            activate_3d_srv_ = create_service<Activate3Dscan>(
                "activate_3d_scan",
                std::bind(&CoordinatorNode::Activate3DscanCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
            deactivate_3d_srv_ = create_service<Deactivate3Dscan>(
                "deactivate_3d_scan",
                std::bind(&CoordinatorNode::Deactivate3DscanCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
            deactivate_focus_srv_ = create_service<DeactivateFocus>(
                "deactivate_focus",
                std::bind(&CoordinatorNode::DeactivateFocusCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
        }

        moveit_cpp_ =
            std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());
        auto psm_const = moveit_cpp_->getPlanningSceneMonitor();
        auto psm = std::const_pointer_cast<
            planning_scene_monitor::PlanningSceneMonitor>(psm_const);

        moveit_msgs::msg::CollisionObject collision_floor;
        collision_floor.header.frame_id =
            psm->getPlanningScene()->getPlanningFrame();
        collision_floor.id = "floor";
        collision_floor.operation = collision_floor.ADD;

        {
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {10.0, 10.0, 0.01};

            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.0;
            box_pose.position.y = 0.0;
            box_pose.position.z = -0.0855;

            collision_floor.primitives.push_back(primitive);
            collision_floor.primitive_poses.push_back(box_pose);
        }

        moveit_msgs::msg::CollisionObject collision_base;
        collision_base.header.frame_id =
            psm->getPlanningScene()->getPlanningFrame();
        collision_base.id = "robot_base";
        collision_base.operation = collision_base.ADD;

        {
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {0.27, 0.27, 0.085};

            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.0;
            box_pose.position.y = 0.0;
            box_pose.position.z = -0.043;

            collision_base.primitives.push_back(primitive);
            collision_base.primitive_poses.push_back(box_pose);
        }

        moveit_msgs::msg::CollisionObject collision_monitor;
        collision_monitor.header.frame_id =
            psm->getPlanningScene()->getPlanningFrame();
        collision_monitor.id = "monitor";
        collision_monitor.operation = collision_monitor.ADD;

        {
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {0.25, 0.6, 0.6};

            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = -0.2;
            box_pose.position.y = 0.435;
            box_pose.position.z = 0.215;

            collision_monitor.primitives.push_back(primitive);
            collision_monitor.primitive_poses.push_back(box_pose);
        }

        {
            planning_scene_monitor::LockedPlanningSceneRW scene(psm);
            scene->processCollisionObjectMsg(collision_floor);
            scene->processCollisionObjectMsg(collision_base);
            scene->processCollisionObjectMsg(collision_monitor);
        }
        RCLCPP_INFO(get_logger(), "Collision objects added to planning scene.");

        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CoordinatorNode::publisherCallback, this));

        focus_action_client_ =
            rclcpp_action::create_client<Focus>(this, "focus_action");
        move_z_angle_action_client_ = rclcpp_action::create_client<MoveZAngle>(
            this, "move_z_angle_action");
        freedrive_action_client_ =
            rclcpp_action::create_client<Freedrive>(this, "freedrive_action");
        reset_action_client_ =
            rclcpp_action::create_client<Reset>(this, "reset_action");

        main_loop_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&CoordinatorNode::mainLoop, this));

        if (!focus_action_client_->wait_for_action_server(
                std::chrono::milliseconds(200))) {
            RCLCPP_WARN(get_logger(), "Focus action server not available yet.");
        }
        if (!move_z_angle_action_client_->wait_for_action_server(
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

        RCLCPP_INFO(get_logger(), "Coordinator Node Initialized.");
    }

  private:
    rclcpp_action::Client<Focus>::SharedPtr focus_action_client_;
    rclcpp_action::Client<MoveZAngle>::SharedPtr move_z_angle_action_client_;
    rclcpp_action::Client<Freedrive>::SharedPtr freedrive_action_client_;
    rclcpp_action::Client<Reset>::SharedPtr reset_action_client_;

    moveit_cpp::MoveItCppPtr moveit_cpp_;

    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr pub_handle_;
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr sub_handle_;
    rclcpp::Subscription<octa_ros::msg::CancelAction>::SharedPtr cancel_handle_;

    rclcpp::Service<Activate3Dscan>::SharedPtr activate_3d_srv_;
    rclcpp::Service<Deactivate3Dscan>::SharedPtr deactivate_3d_srv_;
    rclcpp::Service<DeactivateFocus>::SharedPtr deactivate_focus_srv_;

    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr main_loop_timer_;

    FocusGoalHandle::SharedPtr active_focus_goal_handle_;
    MoveZGoalHandle::SharedPtr active_move_z_goal_handle_;
    FreedriveGoalHandle::SharedPtr active_freedrive_goal_handle_;
    ResetGoalHandle::SharedPtr active_reset_goal_handle_;

    UserAction current_action_ = UserAction::None;
    UserAction previous_action_ = UserAction::None;
    octa_ros::msg::Labviewdata old_sub_msg_;
    octa_ros::msg::Robotdata old_pub_msg_;
    bool cancel_action_ = false;
    bool triggered_service_ = false;

    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
    bool success_ = false;
    double angle_increment_ = 0.0;
    std::chrono::steady_clock::time_point apply_config_time_ =
        std::chrono::steady_clock::now();
    std::mutex data_mutex_;
    bool changed_ = false;

    // Publisher fields
    std::string msg_ = "idle";
    double angle_ = 0.0;
    int circle_state_ = 1;
    bool fast_axis_ = true;
    bool apply_config_ = true;
    bool end_state_ = false;
    bool scan_3d_ = false;

    // Subscriber fields
    double robot_vel_ = 0.5;
    double robot_acc_ = 0.5;
    double z_height_ = 0.0;
    double z_tolerance_ = 0.0;
    double angle_tolerance_ = 0.0;
    double radius_ = 0.0;
    double angle_limit_ = 0.0;
    double dz_ = 0.0;
    double drot_ = 0.0;
    bool autofocus_ = false;
    bool freedrive_ = false;
    bool previous_ = false;
    bool next_ = false;
    bool home_ = false;
    bool reset_ = false;
    bool fast_axis_read_ = true;
    bool scan_3d_read_ = false;
    int num_pt_ = 1;

    void subscriberCallback(const octa_ros::msg::Labviewdata::SharedPtr msg) {
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
        if (*msg != old_sub_msg_) {
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
        }
        old_sub_msg_ = *msg;
    }

    void publisherCallback() {
        octa_ros::msg::Robotdata msg;
        msg.msg = msg_;
        msg.angle = angle_;
        msg.circle_state = circle_state_;
        msg.fast_axis = fast_axis_;
        msg.apply_config = apply_config_;
        msg.end_state = end_state_;
        msg.scan_3d = scan_3d_;

        if (msg != old_pub_msg_) {
            RCLCPP_INFO(get_logger(),
                        "[PUBLISHING] msg: %s, angle: %.2f, circle_state: %d, "
                        "fast_axis: %s, "
                        "apply_config: %s, end_state: %s, scan_3d: %s",
                        msg.msg.c_str(), msg.angle, msg.circle_state,
                        (msg.fast_axis ? "true" : "false"),
                        (msg.apply_config ? "true" : "false"),
                        (msg.end_state ? "true" : "false"),
                        (msg.scan_3d ? "true" : "false"));
        }

        pub_handle_->publish(msg);

        if (apply_config_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsedms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - apply_config_time_)
                    .count();
	    //while (elapsedms < 300) { 
            //    elapsedms = std::chrono::duration_cast<std::chrono::milliseconds>(now - apply_config_time_).count();
	    //    pub_handle_->publish(msg);
	    //}
            if (elapsedms >= 300) {
                apply_config_ = false;
	    //    pub_handle_->publish(msg);
            }
        }
        old_pub_msg_ = msg;
    }

    void mainLoop() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (cancel_action_) {
            if (current_action_ == UserAction::Focus) {
                msg_ = "Canceling Focus action";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                focus_action_client_->async_cancel_all_goals();
            }
            if (current_action_ == UserAction::MoveZangle) {
                msg_ = "Canceling Move Z-angle action";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                move_z_angle_action_client_->async_cancel_all_goals();
            }
            if (current_action_ == UserAction::Freedrive) {
                msg_ = "Canceling Free-drive";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                freedrive_action_client_->async_cancel_all_goals();
            }
            if (current_action_ == UserAction::Reset) {
                msg_ = "Canceling Reset action";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                reset_action_client_->async_cancel_all_goals();
            }
            current_action_ = UserAction::None;
            cancel_action_ = false;
            return;
        }

        if (freedrive_) {
            current_action_ = UserAction::Freedrive;
        } else if (reset_) {
            current_action_ = UserAction::Reset;
        } else if (autofocus_) {
            current_action_ = UserAction::Focus;
        } else if (next_ || previous_ || home_) {
            current_action_ = UserAction::MoveZangle;
        }

        switch (current_action_) {
        case UserAction::Freedrive:
            if (freedrive_) {
                if (previous_action_ != current_action_) {
                    sendFreedriveGoal(true);
                    circle_state_ = 1;
                    angle_ = 0.0;
                    msg_ = "[Action] Freedrive Mode ON";
                    RCLCPP_INFO(get_logger(), msg_.c_str());
                    previous_action_ = UserAction::Freedrive;
                }
            } else {
                sendFreedriveGoal(false);
                msg_ = "[Action] Freedrive Mode OFF";
                RCLCPP_INFO(get_logger(), msg_.c_str());
                current_action_ = UserAction::None;
                previous_action_ = UserAction::None;
            }
            break;
        case UserAction::Reset:
            if (reset_) {
                if (previous_action_ != current_action_) {
                    angle_ = 0.0;
                    circle_state_ = 1;
                    msg_ = "[Action] Reset to default position. It may take "
                           "some time "
                           "please wait.";
                    RCLCPP_INFO(get_logger(), msg_.c_str());
                    sendResetGoal();
                    previous_action_ = UserAction::Reset;
                }
            } else {
                current_action_ = UserAction::None;
                previous_action_ = UserAction::None;
            }
            break;
        case UserAction::Focus:
            if (autofocus_) {
                if (previous_action_ != current_action_) {
                    sendFocusGoal();
                    msg_ = "[Action] Focusing";
                    RCLCPP_INFO(get_logger(), msg_.c_str());
                    previous_action_ = UserAction::Focus;
                    success_ = false;
                }
                if (end_state_) {
                    success_ = true;
                }
            } else {
                if (!success_) {
                    msg_ = "Canceling Focus action";
                    RCLCPP_INFO(this->get_logger(), msg_.c_str());
                    focus_action_client_->async_cancel_all_goals();
                }
                current_action_ = UserAction::None;
                previous_action_ = UserAction::None;
            }
            break;

        case UserAction::MoveZangle:
            angle_increment_ =
                (num_pt_ == 0) ? 0.0
                               : (angle_limit_ / static_cast<double>(num_pt_));
            if (next_) {
                yaw_ = angle_increment_;
                msg_ = std::format("[Action] Next: {}", yaw_);
            } else if (previous_) {
                yaw_ = -angle_increment_;
                msg_ =
                    std::format("[Action] Previous: {}", yaw_);
            } else if (home_) {
                yaw_ = -angle_;
                msg_ = std::format("[Action] Home: {}", yaw_);
                angle_ = 0.0;
            } else {
                RCLCPP_INFO(get_logger(), msg_.c_str());
                sendMoveZAngleGoal(yaw_);
                if (angle_ == 0) {
                    circle_state_ = 1;
                } else if (yaw_ > 0.0) {
                    circle_state_++;
                } else {
                    circle_state_--;
                }
                angle_ += yaw_;
                current_action_ = UserAction::None;
                previous_action_ = UserAction::None;
            }
            break;
        case UserAction::None:
            break;
        }
    }

    void sendFocusGoal() {
        Focus::Goal goal_msg;
        goal_msg.angle_tolerance = angle_tolerance_;
        goal_msg.z_tolerance = z_tolerance_;
        goal_msg.z_height = z_height_;

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
                msg_ = std::format("Focus progress => {}\n", fb->progress);
                msg_ += fb->debug_msg.c_str();
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
            };

        focus_action_client_->async_send_goal(goal_msg, options);
    }

    void sendMoveZAngleGoal(double yaw) {
        MoveZAngle::Goal goal_msg;
        goal_msg.target_angle = yaw;
        goal_msg.radius = radius_;
        goal_msg.angle = angle_;

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

        move_z_angle_action_client_->async_send_goal(goal_msg, options);
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
        Reset::Goal goal_msg;
        goal_msg.reset = true; // or if your action doesn't need fields, omit

        auto options = rclcpp_action::Client<Reset>::SendGoalOptions();
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
                   const std::shared_ptr<const Reset::Feedback> fb) {
                RCLCPP_INFO(this->get_logger(), "Reset feedback => %s",
                            fb->status.c_str());
            };

        reset_action_client_->async_send_goal(goal_msg, options);
    }

    void cancelCallback(const octa_ros::msg::CancelAction::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        cancel_action_ = msg->data;
    }

    void Activate3DscanCallback(
        [[maybe_unused]] const std::shared_ptr<Activate3Dscan::Request> request,
        std::shared_ptr<Activate3Dscan::Response> response) {
        if (!triggered_service_) {
            scan_3d_ = true;
            apply_config_ = true;
            apply_config_time_ = std::chrono::steady_clock::now();
            triggered_service_ = true;
        }
        if (scan_3d_read_) {
            // wait for scan to actually trigger
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
            response->done = true;
            triggered_service_ = false;
        } else {
            response->done = false;
        }
    }

    void Deactivate3DscanCallback(
        [[maybe_unused]] const std::shared_ptr<Deactivate3Dscan::Request>
            request,
        std::shared_ptr<Deactivate3Dscan::Response> response) {
        if (!triggered_service_) {
            scan_3d_ = false;
            apply_config_ = true;
            apply_config_time_ = std::chrono::steady_clock::now();
            triggered_service_ = true;
        }
        if (!scan_3d_read_) {
            response->done = true;
            triggered_service_ = false;
        } else {
            response->done = false;
        }
    }

    void DeactivateFocusCallback(
        [[maybe_unused]] const std::shared_ptr<DeactivateFocus::Request>
            request,
        std::shared_ptr<DeactivateFocus::Response> response) {
        if (!triggered_service_) {
            end_state_ = true;
            apply_config_ = true;
            apply_config_time_ = std::chrono::steady_clock::now();
            triggered_service_ = true;
        }
        if (!autofocus_) {
            response->done = true;
            triggered_service_ = false;
        } else {
            response->done = false;
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinatorNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
