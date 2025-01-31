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
    brake_release_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/dashboard_client/brake_release");
    resend_program_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/io_and_status_controller/resend_robot_program");
    restart_safety_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/dashboard_client/restart_safety");
    timer_ = this->create_wall_timer(
        2s, std::bind(&ReconnectClient::timerCallback, this));
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
        request, [this, service_name](
                     rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
            auto response = fut.get();
            if (!response->success) {
                RCLCPP_WARN(this->get_logger(),
                            "Service [%s] responded with failure: %s",
                            service_name.c_str(), response->message.c_str());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Service [%s] succeeded: %s",
                             service_name.c_str(), response->message.c_str());
            };
        });
    return true;
}

void ReconnectClient::timerCallback() {
    if (!get_robot_mode_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(),
                    "Service [/dashboard_client/get_robot_mode] not available");
        return;
    }
    if (!get_safety_mode_client_->wait_for_service(1s)) {
        RCLCPP_WARN(
            this->get_logger(),
            "Service [/dashboard_client/get_safety_mode] not available");
        return;
    }

    auto request =
        std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>();
    get_robot_mode_client_->async_send_request(
        request,
        [this](
            rclcpp::Client<ur_dashboard_msgs::srv::GetRobotMode>::SharedFuture
                future) {
            auto resp = future.get();
            if (!resp->success) {
                RCLCPP_WARN(this->get_logger(),
                            "[ReconnectClient] get_robot_mode failed: %s",
                            resp->answer.c_str());
                return;
            }

            int8_t mode = resp->robot_mode.mode;
            RCLCPP_DEBUG(this->get_logger(),
                         "Current RobotMode: %d (answer: %s)", mode,
                         resp->answer.c_str());

            switch (mode) {
            case NO_CONTROLLER:
                RCLCPP_INFO(this->get_logger(),
                            "NO_CONTROLLER => resending program...");
                callTriggerService(
                    resend_program_client_,
                    "/io_and_status_controller/resend_robot_program");
                break;
            case DISCONNECTED:
                RCLCPP_INFO(this->get_logger(),
                            "DISCONNECTED => connecting...");
                callTriggerService(connect_client_,
                                   "/dashboard_client/connect");
                break;
            case POWER_OFF:
                RCLCPP_INFO(this->get_logger(), "POWER_OFF => powering on...");
                callTriggerService(power_on_client_,
                                   "/dashboard_client/power_on");
                break;
            case IDLE:
                RCLCPP_INFO(this->get_logger(), "IDLE => releasing brakes...");
                callTriggerService(brake_release_client_,
                                   "/dashboard_client/brake_release");
                // callTriggerService(
                //     resend_program_client_,
                //     "/io_and_status_controller/resend_robot_program");
            default:
                break;
            }
	    if (mode == POWER_OFF || mode == IDLE) {
	        return;
	    }
        });

    auto safety_req =
        std::make_shared<ur_dashboard_msgs::srv::GetSafetyMode::Request>();
    get_safety_mode_client_->async_send_request(
        safety_req,
        [this](
            rclcpp::Client<ur_dashboard_msgs::srv::GetSafetyMode>::SharedFuture
                future) {
            auto resp_safety = future.get();
            if (!resp_safety->success) {
                RCLCPP_WARN(this->get_logger(),
                            "[ReconnectClient] get_safety_mode failed: %s",
                            resp_safety->answer.c_str());
                return;
            };
            uint8_t safety_mode = resp_safety->safety_mode.mode;
            RCLCPP_DEBUG(this->get_logger(),
                         "Current SafetyMode: %u (answer: %s)", safety_mode,
                         resp_safety->answer.c_str());

            if (safety_mode != SAFETY_MODE_NORMAL) {
                RCLCPP_INFO(this->get_logger(),
                            "Safety not NORMAL => calling restart_safety...");
                callTriggerService(restart_safety_client_,
                                   "/dashboard_client/restart_safety");
            };
        });

    auto program_req =
        std::make_shared<ur_dashboard_msgs::srv::GetProgramState::Request>();
    get_program_state_client_->async_send_request(
        program_req,
        [this](rclcpp::Client<
               ur_dashboard_msgs::srv::GetProgramState>::SharedFuture future) {
            auto resp_state = future.get();
            if (!resp_state->success) {
                RCLCPP_WARN(this->get_logger(),
                            "[ReconnectClient] get_program_state failed: %s",
                            resp_state->answer.c_str());
                return;
            };
            std::string program_state = resp_state->state.state;
            RCLCPP_DEBUG(this->get_logger(),
                         "Current Program State: %s (answer: %s)",
                         program_state.c_str(), resp_state->answer.c_str());
        });

    auto load_req =
        std::make_shared<ur_dashboard_msgs::srv::GetLoadedProgram::Request>();

    get_loaded_program_client_->async_send_request(
        load_req,
        [this](rclcpp::Client<
               ur_dashboard_msgs::srv::GetLoadedProgram>::SharedFuture future) {
            auto resp = future.get();
            if (!resp->success) {
                RCLCPP_WARN(this->get_logger(),
                            "[ReconnectClient] get_loaded_program failed: %s",
                            resp->answer.c_str());
                return;
            }
            std::string loaded_program = resp->program_name;
            RCLCPP_DEBUG(this->get_logger(),
                         "Current loaded program: %s (answer: %s)",
                         loaded_program.c_str(), resp->answer.c_str());
        });

    auto running_req =
        std::make_shared<ur_dashboard_msgs::srv::IsProgramRunning::Request>();

    running_program_client_->async_send_request(
        running_req,
        [this](rclcpp::Client<
               ur_dashboard_msgs::srv::IsProgramRunning>::SharedFuture future) {
            auto resp = future.get();
            if (!resp->success) {
                RCLCPP_WARN(this->get_logger(),
                            "[ReconnectClient] IsProgramRunning failed: %s",
                            resp->answer.c_str());
                return;
            }

            bool running_program = resp->program_running;
            RCLCPP_DEBUG(this->get_logger(), "Is running program: %s",
                         running_program ? "true" : "false");

            if (!running_program) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Program is NOT running => Re-sending external control...");
                callTriggerService(
                    resend_program_client_,
                    "/io_and_status_controller/resend_robot_program");
            }
        });
}
