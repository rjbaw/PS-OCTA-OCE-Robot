/**
 * @file reset_node.cpp
 * @author rjbaw
 * @brief Node that sends default position in URscript to the robot driver
 */

#include "reset_node.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <sstream>
#include <string>
#include <string_view>

#include "utils.hpp"

using namespace std::chrono_literals;

namespace {

constexpr auto kResetConfirmationTimeout = 30s;
constexpr auto kResetConfirmationPollPeriod = 50ms;
constexpr double kResetJointToleranceRad = 0.02;
constexpr std::array<std::string_view, 6> kResetJointNames = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
};

bool reset_pose_reached(const moveit::core::RobotState &state,
                        const std::array<double, 6> &targets) {
    for (std::size_t index = 0; index < targets.size(); ++index) {
        const double error =
            state.getVariablePosition(std::string(kResetJointNames[index])) -
            targets[index];
        if (!std::isfinite(error) ||
            std::abs(error) > kResetJointToleranceRad) {
            return false;
        }
    }
    return true;
}

} // namespace

ResetActionServer::ResetActionServer(const rclcpp::NodeOptions &options)
    : Node("reset_action_server",
           rclcpp::NodeOptions(options)
               .automatically_declare_parameters_from_overrides(true)) {}

void ResetActionServer::init() {
    action_server_ = rclcpp_action::create_server<ResetAction>(
        this, "reset_action",
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const ResetAction::Goal> goal) {
            return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleResetAction> goal_handle) {
            return this->handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleResetAction> goal_handle) {
            this->handle_accepted(goal_handle);
        });

    auto qos = rclcpp::SystemDefaultsQoS{};
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/urscript_interface/script_command", qos);
    if (!this->has_parameter("plan_only")) {
        this->declare_parameter<bool>("plan_only", false);
    }
    if (!this->has_parameter("urscript_robot_vel")) {
        this->declare_parameter<double>("urscript_robot_vel", 0.5);
    }
    if (!this->has_parameter("urscript_robot_acc")) {
        this->declare_parameter<double>("urscript_robot_acc", 0.5);
    }
    const bool plan_only = this->get_parameter("plan_only").as_bool();
    if (!plan_only) {
        moveit_cpp_ =
            std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());
    } else {
        RCLCPP_INFO(get_logger(),
                    "Plan-only/Offline mode: skipping MoveIt initialization");
    }
}

rclcpp_action::GoalResponse ResetActionServer::handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ResetAction::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received Reset goal with reset=%s",
                goal->reset ? "true" : "false");
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        RCLCPP_INFO(get_logger(), "Reset goal still processing!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ResetActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleResetAction> goal_handle) {
    {
        std::lock_guard lock(reset_mutex_);
        if (!goal_handle->is_active()) {
            RCLCPP_INFO(get_logger(), "Reset goal no longer active");
            return rclcpp_action::CancelResponse::REJECT;
        }
        cancel_requested_ = true;
    }
    RCLCPP_INFO(get_logger(), "Reset cancellation accepted");
    publish_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ResetActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleResetAction> goal_handle) {
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        active_goal_handle_->abort(std::make_shared<ResetAction::Result>());
    }
    cancel_requested_ = false;
    active_goal_handle_ = goal_handle;
    if (worker_.joinable()) {
        worker_.join();
    }
    worker_ = std::jthread([this, goal_handle](std::stop_token stop_token) {
        const auto terminate_goal = [this, goal_handle,
                                     stop_token](const char *status) {
            try {
                std::lock_guard lock(reset_mutex_);
                cancel_requested_ = true;
                publish_stop();
            } catch (const std::exception &exception) {
                RCLCPP_ERROR(get_logger(),
                             "Could not stop Reset after an exception: %s",
                             exception.what());
            } catch (...) {
                RCLCPP_ERROR(get_logger(),
                             "Could not stop Reset after an unknown exception");
            }
            if (stop_token.stop_requested()) {
                return;
            }
            try {
                auto result = std::make_shared<ResetAction::Result>();
                result->status = status;
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                } else if (goal_handle->is_active()) {
                    goal_handle->abort(result);
                }
            } catch (...) {
                RCLCPP_ERROR(get_logger(),
                             "Could not terminate failed Reset goal");
            }
        };
        try {
            execute(goal_handle, stop_token);
        } catch (const std::exception &exception) {
            RCLCPP_ERROR(get_logger(), "Unhandled Reset exception: %s",
                         exception.what());
            terminate_goal("Reset failed due to an internal exception");
        } catch (...) {
            RCLCPP_ERROR(get_logger(),
                         "Unhandled non-standard Reset exception");
            terminate_goal("Reset failed due to an internal exception");
        }
    });
}

void ResetActionServer::publish_stop(double decel) {
    if (publisher_->get_subscription_count() == 0) {
        RCLCPP_WARN(get_logger(),
                    "publish_stop(): no URScript subscriber – skip");
        return;
    }
    std_msgs::msg::String msg;
    std::ostringstream prog;
    prog << "def stop_motion():\n";
    prog << "  stopj(" << decel << ")\n";
    prog << "end\n";
    msg.data = prog.str();
    publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Sent stopj(%g) to robot", decel);
}

void ResetActionServer::execute(
    const std::shared_ptr<GoalHandleResetAction> goal_handle,
    std::stop_token stop_token) {
    auto feedback = std::make_shared<ResetAction::Feedback>();
    auto result = std::make_shared<ResetAction::Result>();
    const auto handle_interruption = [&]() {
        if (stop_token.stop_requested()) {
            return true;
        }
        if (!cancel_requested_.load() && !goal_handle->is_canceling()) {
            return false;
        }
        const auto cancel_deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (goal_handle->is_active() && !goal_handle->is_canceling() &&
               std::chrono::steady_clock::now() < cancel_deadline) {
            std::this_thread::sleep_for(1ms);
        }
        if (!goal_handle->is_active()) {
            return true;
        }
        if (goal_handle->is_canceling()) {
            result->status = "Cancel requested!";
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Cancel requested!");
        } else {
            result->status = "Reset cancel transition timed out";
            goal_handle->abort(result);
            RCLCPP_ERROR(get_logger(),
                         "Reset goal did not enter CANCELING; aborted after "
                         "the robot stop command");
        }
        return true;
    };
    const auto succeed_if_running = [&](std::string_view status) {
        std::lock_guard lock(reset_mutex_);
        if (cancel_requested_.load() || stop_token.stop_requested() ||
            !goal_handle->is_active() || goal_handle->is_canceling()) {
            return false;
        }
        result->status = status;
        goal_handle->succeed(result);
        return true;
    };

    RCLCPP_INFO(get_logger(), "Starting Reset execution...");
    feedback->debug_msgs = "Resetting... Please wait\n";
    goal_handle->publish_feedback(feedback);

    if (handle_interruption()) {
        return;
    }

    if (!moveit_cpp_) {
        if (handle_interruption()) {
            return;
        }
        if (!succeed_if_running("Reset completed (plan-only/offline)")) {
            (void)handle_interruption();
            return;
        }
        RCLCPP_INFO(get_logger(),
                    "Plan-only/Offline mode: skipping Reset execution");
        return;
    }

    const std::array<double, 6> joint_targets = {
        to_radian(0.0),    to_radian(-60.0), to_radian(90.0),
        to_radian(-120.0), to_radian(-90.0), to_radian(-135.0),
    };
    const double robot_vel =
        this->get_parameter("urscript_robot_vel").as_double();
    const double robot_acc =
        this->get_parameter("urscript_robot_acc").as_double();
    if (!std::isfinite(robot_vel) || robot_vel <= 0.0 ||
        !std::isfinite(robot_acc) || robot_acc <= 0.0) {
        result->status =
            "Reset velocity and acceleration must be finite and greater than "
            "zero";
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "%s", result->status.c_str());
        return;
    }
    std::ostringstream prog;
    prog << "def reset_position():\n";
    prog << "  movej([" << joint_targets[0] << ", " << joint_targets[1] << ", "
         << joint_targets[2] << ", " << joint_targets[3] << ", "
         << joint_targets[4] << ", " << joint_targets[5] << "], "
         << "a=" << robot_acc << ", v=" << robot_vel << ")\n";
    prog << "end\n";
    auto message = std_msgs::msg::String();
    message.data = prog.str();

    const auto subscriber_deadline = std::chrono::steady_clock::now() + 5s;
    while (publisher_->get_subscription_count() == 0) {
        if (handle_interruption()) {
            return;
        }
        if (std::chrono::steady_clock::now() >= subscriber_deadline) {
            RCLCPP_ERROR(get_logger(), "No subscriber found. Aborting Reset.");
            feedback->debug_msgs = "No URScript subscriber available\n";
            result->status = "Reset to default position failed";
            goal_handle->abort(result);
            return;
        }
        std::this_thread::sleep_for(kResetConfirmationPollPeriod);
    }

    bool command_published = false;
    {
        std::lock_guard lock(reset_mutex_);
        if (!cancel_requested_.load() && !stop_token.stop_requested()) {
            publisher_->publish(message);
            command_published = true;
        }
    }
    if (!command_published) {
        (void)handle_interruption();
        return;
    }

    const auto completion_deadline =
        std::chrono::steady_clock::now() + kResetConfirmationTimeout;
    RCLCPP_INFO(get_logger(),
                "Waiting up to %lld seconds for Reset confirmation",
                static_cast<long long>(kResetConfirmationTimeout.count()));
    while (std::chrono::steady_clock::now() < completion_deadline) {
        if (handle_interruption()) {
            return;
        }
        const moveit::core::RobotStatePtr state =
            moveit_cpp_->getCurrentState(0.1);
        if (state && reset_pose_reached(*state, joint_targets)) {
            if (handle_interruption()) {
                return;
            }
            if (!succeed_if_running("Reset action completed successfully")) {
                (void)handle_interruption();
                return;
            }
            RCLCPP_INFO(get_logger(), "Reset completed.");
            return;
        }
        std::this_thread::sleep_for(kResetConfirmationPollPeriod);
    }

    bool canceled_at_timeout = false;
    {
        std::lock_guard lock(reset_mutex_);
        canceled_at_timeout =
            cancel_requested_.load() || stop_token.stop_requested();
        if (!canceled_at_timeout) {
            publish_stop();
        }
    }
    if (canceled_at_timeout) {
        (void)handle_interruption();
        return;
    }
    if (handle_interruption()) {
        return;
    }
    result->status = "Reset position confirmation timed out";
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(),
                 "Reset position was not confirmed within %lld seconds",
                 static_cast<long long>(kResetConfirmationTimeout.count()));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResetActionServer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
