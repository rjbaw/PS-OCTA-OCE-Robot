/**
 * @file reconnect_node.cpp
 * @author rjbaw
 */

#include "reconnect_node.hpp"

using namespace std::chrono_literals;

ReconnectClient::ReconnectClient() : Node("reconnect_client_node") {
    get_robot_mode_client_ =
        this->create_client<ur_dashboard_msgs::srv::GetRobotMode>(
            "/dashboard_client/get_robot_mode");
    get_safety_mode_client_ =
        this->create_client<ur_dashboard_msgs::srv::GetSafetyMode>(
            "/dashboard_client/get_safety_mode");
    get_program_state_client_ =
        this->create_client<ur_dashboard_msgs::srv::GetProgramState>(
            "/dashboard_client/program_state");
    get_loaded_program_client_ =
        this->create_client<ur_dashboard_msgs::srv::GetLoadedProgram>(
            "/dashboard_client/get_loaded_program");
    running_program_client_ =
        this->create_client<ur_dashboard_msgs::srv::IsProgramRunning>(
            "/dashboard_client/program_running");

    connect_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/dashboard_client/connect");
    power_on_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/dashboard_client/power_on");
    play_client_ =
        this->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
    brake_release_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/dashboard_client/brake_release");
    resend_program_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/io_and_status_controller/resend_robot_program");
    restart_safety_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/dashboard_client/restart_safety");

    status_pub_ =
        this->create_publisher<std_msgs::msg::String>("/ur_status_msg", 10);
    driver_health_pub_ =
        this->create_publisher<std_msgs::msg::Bool>("/ur_driver_healthy", 1);

    timer_ = this->create_wall_timer(5s, [this]() { this->timerCallback(); });
}

void ReconnectClient::publish_status(const std::string &text) {
    if (!status_pub_) {
        return;
    }
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
}

void ReconnectClient::publish_driver_health(bool healthy) {
    if (!driver_health_pub_) {
        return;
    }
    driver_healthy_ = healthy;
    std_msgs::msg::Bool msg;
    msg.data = healthy;
    driver_health_pub_->publish(msg);
}

bool ReconnectClient::callTriggerService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
    const std::string &service_name) {
    if (!client->wait_for_service(2s)) {
        RCLCPP_WARN(this->get_logger(), "Service [%s] not available",
                    service_name.c_str());
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    client->async_send_request(
        request,
        [this, service_name](
            rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            const auto &resp = future.get();
            if (!resp->success) {
                RCLCPP_WARN(this->get_logger(),
                            "Service [%s] responded with failure: %s",
                            service_name.c_str(), resp->message.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Service [%s] succeeded: %s",
                            service_name.c_str(), resp->message.c_str());
            };
        });
    return true;
}

void ReconnectClient::timerCallback() {
    RCLCPP_DEBUG(this->get_logger(),
                 "[ReconnectClient] timer tick. Last state — "
                 "robot_mode: %d, safety_mode: %u, program_running: %s",
                 last_robot_mode_, last_safety_mode_,
                 last_program_running_ ? "true" : "false");

    if (!get_robot_mode_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(),
                    "Service [/dashboard_client/get_robot_mode] not available");
        publish_driver_health(false);
        publish_status(
            "[UR] dashboard_client/get_robot_mode not available; UR driver "
            "unhealthy.");
        return;
    }
    if (!get_safety_mode_client_->wait_for_service(1s)) {
        RCLCPP_WARN(
            this->get_logger(),
            "Service [/dashboard_client/get_safety_mode] not available");
        publish_driver_health(false);
        publish_status(
            "[UR] dashboard_client/get_safety_mode not available; UR driver "
            "unhealthy.");
        return;
    }

    const bool program_running_available =
        running_program_client_->wait_for_service(1s);
    if (!program_running_available) {
        RCLCPP_WARN(
            this->get_logger(),
            "Service [/dashboard_client/program_running] not available");
        publish_driver_health(false);
        publish_status(
            "[UR] dashboard_client/program_running not available; UR driver "
            "unhealthy.");
    }

    auto request =
        std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>();
    get_robot_mode_client_->async_send_request(
        request,
        [this](
            rclcpp::Client<ur_dashboard_msgs::srv::GetRobotMode>::SharedFuture
                future) {
            const auto &resp = future.get();
            if (!resp->success) {
                RCLCPP_WARN(this->get_logger(),
                            "[ReconnectClient] get_robot_mode failed: %s",
                            resp->answer.c_str());
                publish_driver_health(false);
                publish_status(std::string("[UR] get_robot_mode failed: ") +
                               resp->answer);
                return;
            }

            publish_driver_health(true);

            int8_t mode = resp->robot_mode.mode;
            if (mode != last_robot_mode_) {
                RCLCPP_INFO(this->get_logger(),
                            "[ReconnectClient] RobotMode transition: %d -> %d "
                            "(answer: %s)",
                            last_robot_mode_, mode, resp->answer.c_str());
                last_robot_mode_ = mode;
            } else {
                RCLCPP_DEBUG(this->get_logger(),
                             "[ReconnectClient] RobotMode unchanged: %d "
                             "(answer: %s)",
                             mode, resp->answer.c_str());
            }

            switch (mode) {
            case DISCONNECTED:
                RCLCPP_INFO(this->get_logger(),
                            "DISCONNECTED => connecting...");
                publish_status(
                    "[UR] Robot DISCONNECTED – attempting to connect.");
                callTriggerService(connect_client_,
                                   "/dashboard_client/connect");
                break;
            case POWER_OFF:
                RCLCPP_INFO(this->get_logger(), "POWER_OFF => powering on...");
                publish_status(
                    "[UR] Robot POWER_OFF – attempting to power on.");
                callTriggerService(power_on_client_,
                                   "/dashboard_client/power_on");
                break;
            case IDLE:
                RCLCPP_INFO(this->get_logger(), "IDLE => releasing brakes...");
                publish_status(
                    "[UR] Robot IDLE – releasing brakes to enter READY.");
                callTriggerService(brake_release_client_,
                                   "/dashboard_client/brake_release");
            default:
                break;
            }
        });

    auto safety_req =
        std::make_shared<ur_dashboard_msgs::srv::GetSafetyMode::Request>();
    get_safety_mode_client_->async_send_request(
        safety_req,
        [this](
            rclcpp::Client<ur_dashboard_msgs::srv::GetSafetyMode>::SharedFuture
                future) {
            const auto &resp_safety = future.get();
            if (!resp_safety->success) {
                RCLCPP_WARN(this->get_logger(),
                            "[ReconnectClient] get_safety_mode failed: %s",
                            resp_safety->answer.c_str());
                publish_driver_health(false);
                publish_status(std::string("[UR] get_safety_mode failed: ") +
                               resp_safety->answer);
                return;
            };
            uint8_t safety_mode = resp_safety->safety_mode.mode;
            if (safety_mode != last_safety_mode_) {
                RCLCPP_INFO(this->get_logger(),
                            "[ReconnectClient] SafetyMode transition: %u -> %u "
                            "(answer: %s)",
                            last_safety_mode_, safety_mode,
                            resp_safety->answer.c_str());
                last_safety_mode_ = safety_mode;
            } else {
                RCLCPP_DEBUG(this->get_logger(),
                             "[ReconnectClient] SafetyMode unchanged: %u "
                             "(answer: %s)",
                             safety_mode, resp_safety->answer.c_str());
            }

            if (safety_mode != SAFETY_MODE_NORMAL) {
                RCLCPP_INFO(this->get_logger(),
                            "Safety not NORMAL => calling restart_safety...");
                publish_status(
                    "[UR] Safety mode not NORMAL – restarting safety. Check "
                    "pendant.");
                callTriggerService(restart_safety_client_,
                                   "/dashboard_client/restart_safety");
            };
        });

    auto running_req =
        std::make_shared<ur_dashboard_msgs::srv::IsProgramRunning::Request>();
    if (program_running_available) {
        running_program_client_->async_send_request(
            running_req,
            [this](rclcpp::Client<
                   ur_dashboard_msgs::srv::IsProgramRunning>::SharedFuture
                       future) {
                const auto &resp = future.get();
                if (!resp->success) {
                    RCLCPP_WARN(this->get_logger(),
                                "[ReconnectClient] IsProgramRunning failed: %s",
                                resp->answer.c_str());
                    publish_driver_health(false);
                    publish_status(
                        std::string("[UR] IsProgramRunning failed: ") +
                        resp->answer);
                    return;
                }

                publish_driver_health(true);

                bool running_program = resp->program_running;
                if (running_program != last_program_running_) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[ReconnectClient] ProgramRunning transition: %s -> %s "
                        "(answer: %s)",
                        last_program_running_ ? "true" : "false",
                        running_program ? "true" : "false",
                        resp->answer.c_str());
                    last_program_running_ = running_program;
                } else {
                    RCLCPP_DEBUG(
                        this->get_logger(),
                        "[ReconnectClient] ProgramRunning unchanged: %s",
                        running_program ? "true" : "false");
                }

                if (!running_program) {
                    ++program_not_running_ticks_;
                    if (resend_cooldown_ticks_ > 0) {
                        --resend_cooldown_ticks_;
                    }

                    RCLCPP_INFO(this->get_logger(),
                                "Program is NOT running (robot_mode=%d, "
                                "safety_mode=%u) => attempting dashboard play.",
                                last_robot_mode_, last_safety_mode_);
                    publish_status(
                        "[UR] Program not running – attempting play.");
                    callTriggerService(play_client_, "/dashboard_client/play");

                    constexpr int kResendAfterTicks = 2;
                    constexpr int kResendCooldownTicks = 3;
                    if (program_not_running_ticks_ >= kResendAfterTicks &&
                        resend_cooldown_ticks_ == 0) {
                        RCLCPP_INFO(this->get_logger(),
                                    "Program still NOT running => resending "
                                    "external control program.");
                        publish_status(
                            "[UR] ExternalControl still not running – "
                            "resending robot program.");
                        callTriggerService(
                            resend_program_client_,
                            "/io_and_status_controller/resend_robot_program");
                        resend_cooldown_ticks_ = kResendCooldownTicks;
                    }
                } else {
                    program_not_running_ticks_ = 0;
                    resend_cooldown_ticks_ = 0;
                }
            });
    }

    // auto program_req =
    //     std::make_shared<ur_dashboard_msgs::srv::GetProgramState::Request>();
    // get_program_state_client_->async_send_request(
    //     program_req,
    //     [this](rclcpp::Client<
    //            ur_dashboard_msgs::srv::GetProgramState>::SharedFuture future)
    //            {
    //         auto resp_state = future.get();
    //         if (!resp_state->success) {
    //             RCLCPP_WARN(this->get_logger(),
    //                         "[ReconnectClient] get_program_state failed: %s",
    //                         resp_state->answer.c_str());
    //             return;
    //         };
    //         std::string program_state = resp_state->state.state;
    //         RCLCPP_INFO(this->get_logger(),
    //                     "Current Program State: %s (answer: %s)",
    //                     program_state.c_str(), resp_state->answer.c_str());
    //     });

    // auto load_req =
    //     std::make_shared<ur_dashboard_msgs::srv::GetLoadedProgram::Request>();
    // get_loaded_program_client_->async_send_request(
    //     load_req,
    //     [this](rclcpp::Client<
    //            ur_dashboard_msgs::srv::GetLoadedProgram>::SharedFuture
    //            future) {
    //         auto resp = future.get();
    //         if (!resp->success) {
    //             RCLCPP_WARN(this->get_logger(),
    //                         "[ReconnectClient] get_loaded_program failed:
    //                         %s", resp->answer.c_str());
    //             return;
    //         }
    //         std::string loaded_program = resp->program_name;
    //         RCLCPP_INFO(this->get_logger(),
    //                     "Current loaded program: %s (answer: %s)",
    //                     loaded_program.c_str(), resp->answer.c_str());
    //     });
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ReconnectClient>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
