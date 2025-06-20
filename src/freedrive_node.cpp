/**
 * @file freedrive_node.cpp
 * @author rjbaw
 * @brief Node that activates or deactivates robot freedrive mode
 */

#include <chrono>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_msgs/msg/bool.hpp>

#include <octa_ros/action/freedrive.hpp>

class FreedriveActionServer : public rclcpp::Node {

  public:
    using Freedrive = octa_ros::action::Freedrive;
    using GoalHandleFreedrive = rclcpp_action::ServerGoalHandle<Freedrive>;
    using SwitchSrv = controller_manager_msgs::srv::SwitchController;

    FreedriveActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("freedrive_action_server", options) {
        this->declare_parameter<std::string>(
            "motion_controller", "scaled_joint_trajectory_controller");
        this->declare_parameter<std::string>("freedrive_controller",
                                             "freedrive_mode_controller");
        this->declare_parameter<double>("keepalive_rate",
                                        5.0);                   // Hz
        this->declare_parameter<double>("switch_timeout", 3.0); // s

        switch_client_ = this->create_client<SwitchSrv>(
            "/controller_manager/switch_controller");
        freedrive_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/freedrive_mode_controller/enable_freedrive_mode",
            rclcpp::QoS(1).reliable());

        if (!switch_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_FATAL(get_logger(), "controller_manager service not "
                                       "available – is ros2_control running?");
            throw std::runtime_error("no /controller_manager service");
        }

        action_server_ = rclcpp_action::create_server<Freedrive>(
            this, "freedrive_action",
            std::bind(&FreedriveActionServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&FreedriveActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&FreedriveActionServer::handle_accepted, this,
                      std::placeholders::_1));
    }

  private:
    rclcpp_action::Server<Freedrive>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleFreedrive> active_goal_handle_;
    rclcpp::Client<SwitchSrv>::SharedPtr switch_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr freedrive_pub_;
    rclcpp::TimerBase::SharedPtr keepalive_timer_;

    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &,
                std::shared_ptr<const Freedrive::Goal> goal) {
        RCLCPP_INFO(get_logger(), "Received freedrive goal – enable=%s",
                    goal->enable ? "true" : "false");
        if (active_goal_handle_ && active_goal_handle_->is_active()) {
            active_goal_handle_->canceled(
                std::make_shared<Freedrive::Result>());
            stop_keepalive();
            switch_to_freedrive_controller(false);
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
        if (!goal_handle->is_active()) {
            RCLCPP_INFO(get_logger(), "Freedrive goal no longer active");
            return rclcpp_action::CancelResponse::REJECT;
        }
        stop_keepalive();
        switch_to_freedrive_controller(false);
        RCLCPP_INFO(get_logger(), "Freedrive action cancelled by client");
        // goal_handle->canceled(std::make_shared<Freedrive::Result>());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void
    handle_accepted(const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
        if (active_goal_handle_ && active_goal_handle_->is_active()) {
            active_goal_handle_->abort(std::make_shared<Freedrive::Result>());
        }
        active_goal_handle_ = goal_handle;
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
        auto feedback = std::make_shared<Freedrive::Feedback>();
        auto result = std::make_shared<Freedrive::Result>();
        RCLCPP_INFO(get_logger(), "Starting Freedrive execution...");
        bool enable = goal_handle->get_goal()->enable;
        feedback->debug_msgs =
            enable ? "Enabling Freedrive\n" : "Disabling Freedrive\n";
        goal_handle->publish_feedback(feedback);

        if (enable) {
            if (!switch_to_freedrive_controller(true)) {
                result->status = "Controller switch failed\n";
                goal_handle->abort(result);
                return;
            };
            start_keepalive();
        } else {
            stop_keepalive();
            if (!switch_to_freedrive_controller(false)) {
                result->status = "Controller switch failed\n";
                goal_handle->abort(result);
                return;
            };
        }
        if (goal_handle->is_canceling()) {
            stop_keepalive();
            switch_to_freedrive_controller(false);
            result->status = "Freedrive Canceled";
            goal_handle->canceled(result);
            return;
        }
        // std::this_thread::sleep_for(std::chrono::seconds(2));

        feedback->debug_msgs =
            enable ? "Freedrive enabled and controller active\n"
                   : "Freedrive disabled - motion controller active\n";
        goal_handle->publish_feedback(feedback);

        result->status = "Freedrive toggle success\n";
        goal_handle->succeed(result);

        RCLCPP_INFO(get_logger(), "Freedrive action completed.");
    }

    void start_keepalive() {
        if (keepalive_timer_) {
            keepalive_timer_->cancel();
        }

        const double rate = this->get_parameter("keepalive_rate").as_double();
        auto period = std::chrono::duration<double>(1.0 / rate);
        keepalive_timer_ =
            this->create_wall_timer(period, [this]() { publish_bool(true); });
        publish_bool(true);
    }

    void stop_keepalive() {
        publish_bool(false);
        if (keepalive_timer_)
            keepalive_timer_->cancel();
        keepalive_timer_.reset();
    }

    void publish_bool(bool value) {
        std_msgs::msg::Bool msg;
        msg.data = value;
        freedrive_pub_->publish(msg);
    }

    bool switch_to_freedrive_controller(bool to_freedrive) {
        auto req = std::make_shared<SwitchSrv::Request>();
        std::string params_motion_controller =
            this->get_parameter("motion_controller").as_string();
        std::string params_freedrive_controller =
            this->get_parameter("freedrive_controller").as_string();

        if (to_freedrive) {
            req->activate_controllers = {params_freedrive_controller};
            req->deactivate_controllers = {params_motion_controller};
        } else {
            req->activate_controllers = {params_motion_controller};
            req->deactivate_controllers = {params_freedrive_controller};
        }
        req->strictness = SwitchSrv::Request::STRICT;
        req->timeout = rclcpp::Duration::from_seconds(
            this->get_parameter("switch_timeout").as_double());

        auto future = switch_client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(5)) !=
            std::future_status::ready)
            return false;
        return future.get()->ok;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FreedriveActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
