#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Custom messages
#include <octa_ros/msg/labviewdata.hpp>
#include <octa_ros/msg/robotdata.hpp>

// Existing actions
#include <my_package/action/focus.hpp>        // e.g. Focus.action
#include <my_package/action/move_z_angle.hpp> // e.g. MoveZAngle.action

// New Freedrive & Reset actions
#include <my_package/action/freedrive.hpp> // Freedrive.action
#include <my_package/action/reset.hpp>     // Reset.action

#include <chrono>
#include <cmath>
#include <format>
#include <mutex>
#include <string>
#include <vector>

static double to_radian(double deg) { return deg * M_PI / 180.0; }

class ControlNode : public rclcpp::Node {
  public:
    // Aliases for your actions
    using Focus = my_package::action::Focus;
    using MoveZAngle = my_package::action::MoveZAngle;
    using Freedrive = my_package::action::Freedrive;
    using ResetAction = my_package::action::Reset;

    // Goal handles for each
    using FocusGoalHandle = rclcpp_action::ClientGoalHandle<Focus>;
    using MoveZGoalHandle = rclcpp_action::ClientGoalHandle<MoveZAngle>;
    using FreedriveGoalHandle = rclcpp_action::ClientGoalHandle<Freedrive>;
    using ResetGoalHandle = rclcpp_action::ClientGoalHandle<ResetAction>;

    explicit ControlNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("control_node", options) {
        // 1) Publisher for robot_data
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            robotdata_pub_ = this->create_publisher<octa_ros::msg::Robotdata>(
                "robot_data", qos);
        }

        // 2) Subscriber for labview_data
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            labview_sub_ =
                this->create_subscription<octa_ros::msg::Labviewdata>(
                    "labview_data", qos,
                    std::bind(&ControlNode::labviewCallback, this,
                              std::placeholders::_1));
        }

        // 3) Timer to publish Robotdata
        robotdata_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ControlNode::publishRobotdata, this));

        // 4) Action clients
        focus_action_client_ =
            rclcpp_action::create_client<Focus>(this, "focus_action");
        move_z_angle_client_ = rclcpp_action::create_client<MoveZAngle>(
            this, "move_z_angle_action");
        freedrive_action_client_ =
            rclcpp_action::create_client<Freedrive>(this, "freedrive_action");
        reset_action_client_ =
            rclcpp_action::create_client<ResetAction>(this, "reset_action");

        // 5) Main loop timer
        main_loop_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&ControlNode::mainLoop, this));

        RCLCPP_INFO(get_logger(), "ControlNode constructed and ready.");
    }

  private:
    // -----------------------
    // Action Clients
    // -----------------------
    rclcpp_action::Client<Focus>::SharedPtr focus_action_client_;
    rclcpp_action::Client<MoveZAngle>::SharedPtr move_z_angle_client_;
    rclcpp_action::Client<Freedrive>::SharedPtr freedrive_action_client_;
    rclcpp_action::Client<ResetAction>::SharedPtr reset_action_client_;

    // Publisher & Subscriber
    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr robotdata_pub_;
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr labview_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr robotdata_timer_;
    rclcpp::TimerBase::SharedPtr main_loop_timer_;

    // Robotdata fields
    std::string msg_{"idle"};
    double angle_{0.0};
    int circle_state_{1};
    bool fast_axis_{true};
    bool apply_config_{false};
    bool end_state_{false};
    bool scan_3d_{false};

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
    bool fast_axis_sub_{false};
    bool scan_3d_sub_{false};
    double z_height_{0.0};
    int num_pt_{1};

    bool changed_{false};
    octa_ros::msg::Labviewdata old_labview_msg_;

    // Some angles
    double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
    double angle_increment_{0.0};

    // -----------------------
    // SUBSCRIBER CALLBACK
    // -----------------------
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
        fast_axis_sub_ = msg->fast_axis;
        scan_3d_sub_ = msg->scan_3d;
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
                                    fast_axis_sub_, scan_3d_sub_, z_height_)
                            .c_str());
        } else {
            changed_ = false;
        }
        old_labview_msg_ = *msg;
    }

    // -----------------------
    // PUBLISHER TIMER
    // -----------------------
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

    // -----------------------
    // MAIN LOOP TIMER
    // -----------------------
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

        // Freedrive (enable/disable)
        if (freedrive_) {
            RCLCPP_INFO(get_logger(),
                        "User Freedrive => enabling Freedrive mode via action");
            sendFreedriveGoal(true);
            // Freedrive_ might stay true or revert after sending
        } else {
            // Possibly disable Freedrive if it was previously on. This is up to
            // your logic: sendFreedriveGoal(false);
        }

        // Reset
        if (reset_) {
            RCLCPP_INFO(get_logger(), "User RESET => sending Reset action");
            sendResetGoal();
        }

        // Autofocus
        if (autofocus_) {
            RCLCPP_INFO(get_logger(), "User AUTOFOCUS => sending Focus goal");
            sendFocusGoal();
            return;
        }

        // Normal circle stepping
        angle_increment_ = (num_pt_ == 0)
                               ? 0.0
                               : (angle_limit_ / static_cast<double>(num_pt_));
        if (next_) {
            yaw_ += to_radian(angle_increment_);
            RCLCPP_INFO(get_logger(),
                        "NEXT => new yaw=%.2f deg => sending MoveZAngle goal",
                        yaw_ * 180.0 / M_PI);
            sendMoveZAngleGoal(yaw_);
        }
        if (previous_) {
            yaw_ -= to_radian(angle_increment_);
            RCLCPP_INFO(
                get_logger(),
                "PREVIOUS => new yaw=%.2f deg => sending MoveZAngle goal",
                yaw_ * 180.0 / M_PI);
            sendMoveZAngleGoal(yaw_);
        }
        if (home_) {
            double old_ang_deg = angle_;
            yaw_ -= to_radian(old_ang_deg);
            RCLCPP_INFO(get_logger(),
                        "HOME => revert yaw by %.2f deg => new yaw=%.2f => "
                        "sending MoveZAngle goal",
                        old_ang_deg, yaw_ * 180.0 / M_PI);
            sendMoveZAngleGoal(yaw_);
        }
    }

    // -----------------------
    // ACTION SEND GOALS
    // -----------------------
    void sendFocusGoal() {
        Focus::Goal goal_msg;
        goal_msg.angle_tolerance = angle_tolerance_;
        goal_msg.z_tolerance = z_tolerance_;
        // etc...

        auto options = rclcpp_action::Client<Focus>::SendGoalOptions();
        options.result_callback =
            [this](const FocusGoalHandle::WrappedResult &result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Focus SUCCEEDED");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Focus ABORTED");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Focus CANCELED");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "Focus UNKNOWN code");
                    break;
                }
            };

        options.feedback_callback =
            [this](FocusGoalHandle::SharedPtr,
                   const std::shared_ptr<const Focus::Feedback> fb) {
                RCLCPP_INFO(this->get_logger(),
                            "Focus feedback => current_angle=%.2f",
                            fb->current_angle);
            };

        focus_action_client_->async_send_goal(goal_msg, options);
    }

    void sendMoveZAngleGoal(double yaw_rad) {
        MoveZAngle::Goal goal_msg;
        // Suppose the action expects a float64 target_angle in degrees:
        goal_msg.target_angle = yaw_rad * 180.0 / M_PI;

        auto options = rclcpp_action::Client<MoveZAngle>::SendGoalOptions();
        options.result_callback =
            [this](const MoveZGoalHandle::WrappedResult &result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "MoveZAngle SUCCEEDED");
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
