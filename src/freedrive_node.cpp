/**
 * @file freedrive_node.cpp
 * @author rjbaw
 * @brief Node that activates or deactivates robot freedrive mode
 */

#include "freedrive_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <future>

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

    const bool dry_run = this->get_parameter("dry_run").as_bool();
    if (!dry_run) {
        switch_client_ = this->create_client<SwitchSrv>(
            "/controller_manager/switch_controller");
        list_client_ = this->create_client<ListSrv>(
            "/controller_manager/list_controllers");
        if (!switch_client_->wait_for_service(std::chrono::seconds(10))) {
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
    [[maybe_unused]] const rclcpp_action::GoalUUID &goal_id,
    std::shared_ptr<const Freedrive::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received freedrive goal – enable=%s",
                goal->enable ? "true" : "false");
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        RCLCPP_WARN(get_logger(),
                    "Freedrive controller switch is still in progress");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FreedriveActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
    if (!goal_handle->is_active()) {
        RCLCPP_INFO(get_logger(), "Freedrive goal no longer active");
        return rclcpp_action::CancelResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Freedrive cancellation accepted");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FreedriveActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        active_goal_handle_->abort(std::make_shared<Freedrive::Result>());
    }
    active_goal_handle_ = goal_handle;
    if (worker_.joinable()) {
        worker_.join();
    }
    worker_ = std::jthread([this, goal_handle](std::stop_token stop_token) {
        const auto terminate_goal = [this, goal_handle,
                                     stop_token](const char *status) {
            try {
                stop_keepalive();
                (void)switch_to_freedrive_controller(false, goal_handle,
                                                     stop_token, true);
            } catch (const std::exception &exception) {
                RCLCPP_ERROR(get_logger(),
                             "Could not restore the motion controller after a "
                             "Freedrive exception: %s",
                             exception.what());
            } catch (...) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Could not restore the motion controller after an unknown "
                    "Freedrive exception");
            }
            if (stop_token.stop_requested()) {
                return;
            }
            try {
                auto result = std::make_shared<Freedrive::Result>();
                result->status = status;
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                } else if (goal_handle->is_active()) {
                    goal_handle->abort(result);
                }
            } catch (...) {
                RCLCPP_ERROR(get_logger(),
                             "Could not terminate failed Freedrive goal");
            }
        };
        try {
            execute(goal_handle, stop_token);
        } catch (const std::exception &exception) {
            RCLCPP_ERROR(get_logger(), "Unhandled Freedrive exception: %s",
                         exception.what());
            terminate_goal("Freedrive failed due to an internal exception\n");
        } catch (...) {
            RCLCPP_ERROR(get_logger(),
                         "Unhandled non-standard Freedrive exception");
            terminate_goal("Freedrive failed due to an internal exception\n");
        }
    });
}

void FreedriveActionServer::execute(
    const std::shared_ptr<GoalHandleFreedrive> goal_handle,
    std::stop_token stop_token) {
    auto feedback = std::make_shared<Freedrive::Feedback>();
    auto result = std::make_shared<Freedrive::Result>();
    const bool enable = goal_handle->get_goal()->enable;
    const auto handle_interruption = [&](bool restore_motion_controller) {
        const bool canceling = goal_handle->is_canceling();
        if (!canceling && !stop_token.stop_requested()) {
            return false;
        }
        stop_keepalive();
        if (restore_motion_controller) {
            (void)switch_to_freedrive_controller(false, goal_handle, stop_token,
                                                 true);
        }
        if (canceling) {
            result->status = "Freedrive Canceled\n";
            goal_handle->canceled(result);
        }
        return true;
    };
    RCLCPP_INFO(get_logger(), "Starting Freedrive execution...");
    const bool dry_run = !switch_client_;
    feedback->debug_msgs =
        enable ? "Enabling Freedrive\n" : "Disabling Freedrive\n";
    goal_handle->publish_feedback(feedback);
    if (handle_interruption(false)) {
        return;
    }

    if (dry_run) {
        RCLCPP_INFO(get_logger(), "Dry-run mode: skipping controller switches");
    } else if (enable) {
        const double keepalive_rate =
            this->get_parameter("keepalive_rate").as_double();
        if (!std::isfinite(keepalive_rate) || keepalive_rate <= 0.0) {
            result->status =
                "keepalive_rate must be finite and greater than zero\n";
            goal_handle->abort(result);
            RCLCPP_ERROR(get_logger(), "%s", result->status.c_str());
            return;
        }
        if (handle_interruption(false)) {
            return;
        }
        if (!switch_to_freedrive_controller(true, goal_handle, stop_token)) {
            if (handle_interruption(true)) {
                return;
            }
            stop_keepalive();
            const bool motion_controller_restored =
                switch_to_freedrive_controller(false, goal_handle, stop_token,
                                               true);
            if (handle_interruption(false)) {
                return;
            }
            result->status = "Controller switch failed\n";
            if (!motion_controller_restored) {
                result->status +=
                    "Motion controller restoration was not confirmed\n";
            }
            goal_handle->abort(result);
            return;
        }
        if (handle_interruption(true)) {
            return;
        }
        start_keepalive(keepalive_rate);
    } else {
        stop_keepalive();
        if (!switch_to_freedrive_controller(false, goal_handle, stop_token)) {
            if (handle_interruption(false)) {
                return;
            }
            result->status = "Controller switch failed\n";
            goal_handle->abort(result);
            return;
        }
    }
    if (handle_interruption(enable)) {
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

void FreedriveActionServer::start_keepalive(double rate) {
    std::lock_guard lock(keepalive_mutex_);
    if (keepalive_timer_) {
        keepalive_timer_->cancel();
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    keepalive_timer_ = this->create_wall_timer(period, [this]() {
        std::lock_guard lock(keepalive_mutex_);
        if (keepalive_timer_) {
            publish_bool(true);
        }
    });
    publish_bool(true);
}

void FreedriveActionServer::stop_keepalive() {
    std::lock_guard lock(keepalive_mutex_);
    if (keepalive_timer_) {
        keepalive_timer_->cancel();
    }
    keepalive_timer_.reset();
    publish_bool(false);
}

void FreedriveActionServer::publish_bool(bool value) {
    std_msgs::msg::Bool msg;
    msg.data = value;
    freedrive_pub_->publish(msg);
}

bool FreedriveActionServer::switch_to_freedrive_controller(
    bool to_freedrive, const std::shared_ptr<GoalHandleFreedrive> &goal_handle,
    std::stop_token stop_token, bool force_request) {
    constexpr auto kStateCheckTimeout = std::chrono::seconds(1);
    constexpr double kMinimumOperationTimeoutSec = 5.0;
    constexpr double kOperationMarginSec = 2.0;

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
    const double switch_timeout =
        this->get_parameter("switch_timeout").as_double();
    if (!std::isfinite(switch_timeout) || switch_timeout <= 0.0) {
        RCLCPP_ERROR(get_logger(),
                     "switch_timeout must be finite and greater than zero");
        return false;
    }
    req->timeout = rclcpp::Duration::from_seconds(switch_timeout);

    if (!switch_client_) {
        RCLCPP_INFO(get_logger(), "Dry-run mode: pretend controller switch OK");
        return true;
    }

    bool motion_active = false;
    bool freedrive_active = false;
    const auto read_controller_state = [&](bool &requested_state_matches) {
        motion_active = false;
        freedrive_active = false;
        requested_state_matches = false;
        if (!list_client_->wait_for_service(kStateCheckTimeout)) {
            RCLCPP_WARN(get_logger(), "Controller list service unavailable");
            return false;
        }
        auto future = list_client_->async_send_request(
            std::make_shared<ListSrv::Request>());
        if (future.wait_for(kStateCheckTimeout) != std::future_status::ready) {
            list_client_->remove_pending_request(future);
            RCLCPP_WARN(get_logger(), "Controller list response timed out");
            return false;
        }

        const auto response = future.get();
        for (const auto &controller : response->controller) {
            if (controller.name == params_motion_controller) {
                motion_active = controller.state == "active";
            } else if (controller.name == params_freedrive_controller) {
                freedrive_active = controller.state == "active";
            }
        }
        requested_state_matches = to_freedrive
                                      ? (freedrive_active && !motion_active)
                                      : (motion_active && !freedrive_active);
        return true;
    };

    bool state_matches = false;
    const bool state_known = read_controller_state(state_matches);
    if (!force_request && !state_known) {
        return false;
    }
    if (!force_request && state_matches) {
        RCLCPP_INFO(get_logger(), "Controllers already match Freedrive %s",
                    to_freedrive ? "ON" : "OFF");
        return true;
    }
    if (!force_request) {
        if (to_freedrive) {
            if (freedrive_active) {
                req->activate_controllers.clear();
            }
            if (!motion_active) {
                req->deactivate_controllers.clear();
            }
        } else {
            if (motion_active) {
                req->activate_controllers.clear();
            }
            if (!freedrive_active) {
                req->deactivate_controllers.clear();
            }
        }
    }
    if (to_freedrive &&
        (stop_token.stop_requested() || goal_handle->is_canceling())) {
        return false;
    }

    if (!switch_client_->wait_for_service(kStateCheckTimeout)) {
        RCLCPP_WARN(get_logger(), "Controller switch service unavailable");
        return false;
    }
    if (to_freedrive &&
        (stop_token.stop_requested() || goal_handle->is_canceling())) {
        return false;
    }
    auto future = switch_client_->async_send_request(req);
    const auto operation_timeout = std::chrono::duration<double>(std::max(
        kMinimumOperationTimeoutSec, switch_timeout + kOperationMarginSec));
    if (future.wait_for(operation_timeout) != std::future_status::ready) {
        switch_client_->remove_pending_request(future);
        RCLCPP_WARN(get_logger(), "Controller switch response timed out");
        return false;
    }
    if (!future.get()->ok) {
        RCLCPP_WARN(get_logger(), "Controller switch request failed");
    }
    return read_controller_state(state_matches) && state_matches;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FreedriveActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
