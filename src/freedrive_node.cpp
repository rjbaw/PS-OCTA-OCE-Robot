/**
 * @file freedrive_node.cpp
 * @author rjbaw
 * @brief Node that activates or deactivates robot freedrive mode
 */

#include "freedrive_node.hpp"

FreedriveActionServer::FreedriveActionServer(const rclcpp::NodeOptions &options)
    : Node("freedrive_action_server", options) {
    this->declare_parameter<std::string>("motion_controller",
                                         "scaled_joint_trajectory_controller");
    this->declare_parameter<std::string>("freedrive_controller",
                                         "freedrive_mode_controller");
    this->declare_parameter<double>("keepalive_rate", 5.0);
    this->declare_parameter<double>("switch_timeout", 3.0);
    if (!this->has_parameter("dry_run")) {
        this->declare_parameter<bool>("dry_run", false);
    }

    bool dry_run = this->get_parameter("dry_run").as_bool();
    if (!dry_run) {
        switch_client_ = this->create_client<SwitchSrv>(
            "/controller_manager/switch_controller");
        if (!switch_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_FATAL(get_logger(),
                         "controller_manager service not available – is "
                         "ros2_control running?");
            throw std::runtime_error("no /controller_manager service");
        }
    } else {
        RCLCPP_INFO(get_logger(),
                    "Dry-run mode: skipping controller_manager client setup");
    }
    freedrive_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/freedrive_mode_controller/enable_freedrive_mode",
        rclcpp::QoS(1).reliable());

    action_server_ = rclcpp_action::create_server<Freedrive>(
        this, "freedrive_action",
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const Freedrive::Goal> goal) {
            return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
            return this->handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
            this->handle_accepted(goal_handle);
        });
}

rclcpp_action::GoalResponse FreedriveActionServer::handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID goal_id,
    std::shared_ptr<const Freedrive::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received freedrive goal – enable=%s",
                goal->enable ? "true" : "false");
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        active_goal_handle_->canceled(std::make_shared<Freedrive::Result>());
        stop_keepalive();
        switch_to_freedrive_controller(false);
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FreedriveActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
    if (!goal_handle->is_active()) {
        RCLCPP_INFO(get_logger(), "Freedrive goal no longer active");
        return rclcpp_action::CancelResponse::REJECT;
    }
    stop_keepalive();
    switch_to_freedrive_controller(false);
    RCLCPP_INFO(get_logger(), "Freedrive action canceled by client");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FreedriveActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        active_goal_handle_->abort(std::make_shared<Freedrive::Result>());
    }
    active_goal_handle_ = goal_handle;
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void FreedriveActionServer::execute(
    const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
    auto feedback = std::make_shared<Freedrive::Feedback>();
    auto result = std::make_shared<Freedrive::Result>();
    RCLCPP_INFO(get_logger(), "Starting Freedrive execution...");
    bool enable = goal_handle->get_goal()->enable;
    bool dry_run = this->get_parameter("dry_run").as_bool();
    feedback->debug_msgs =
        enable ? "Enabling Freedrive\n" : "Disabling Freedrive\n";
    goal_handle->publish_feedback(feedback);

    if (dry_run) {
        RCLCPP_INFO(get_logger(), "Dry-run mode: skipping controller switches");
    } else if (enable) {
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
        result->status = "Freedrive Canceled\n";
        goal_handle->canceled(result);
        return;
    }

    feedback->debug_msgs =
        enable ? "Freedrive enabled and controller active\n"
               : "Freedrive disabled - motion controller active\n";
    goal_handle->publish_feedback(feedback);

    result->status = "Freedrive toggle success\n";
    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "Freedrive action completed.");
}

void FreedriveActionServer::start_keepalive() {
    if (keepalive_timer_) {
        keepalive_timer_->cancel();
    }
    const double rate = this->get_parameter("keepalive_rate").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    keepalive_timer_ =
        this->create_wall_timer(period, [this]() { publish_bool(true); });
    publish_bool(true);
}

void FreedriveActionServer::stop_keepalive() {
    publish_bool(false);
    if (keepalive_timer_) {
        keepalive_timer_->cancel();
    }
    keepalive_timer_.reset();
}

void FreedriveActionServer::publish_bool(bool value) {
    std_msgs::msg::Bool msg;
    msg.data = value;
    freedrive_pub_->publish(msg);
}

bool FreedriveActionServer::switch_to_freedrive_controller(bool to_freedrive) {
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

    if (!switch_client_) {
        RCLCPP_INFO(get_logger(), "Dry-run mode: pretend controller switch OK");
        return true;
    }
    auto future = switch_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        return false;
    }
    return future.get()->ok;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FreedriveActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
