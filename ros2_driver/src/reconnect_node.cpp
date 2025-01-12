#include "reconnect_node.hpp"

ReconnectClient::ReconnectClient() : Node("reconnect_client_node") {
    // set_client_ = rclcpp_action::create_client<SetMode>(
    //     this, "/dashboard_client/set_mode");
    status_client_ = this->create_client<ur_dashboard_msgs::srv::GetRobotMode>(
        "/dashboard_client/get_robot_mode");
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10000),
        std::bind(&ReconnectClient::timerCallback, this));
}

int8_t ReconnectClient::getCurrentMode() const { return current_mode_.load(); }

void ReconnectClient::send_goal(int8_t target_mode, bool stop_program,
                                bool play_program) {
    // if (!set_client_->wait_for_action_server(std::chrono::seconds(5))) {
    //     RCLCPP_ERROR(this->get_logger(),
    //                  "SetMode action server not available after waiting");
    //     return;
    // }
    auto goal_msg = SetMode::Goal();
    goal_msg.target_robot_mode = target_mode;
    goal_msg.stop_program = stop_program;
    goal_msg.play_program = play_program;

    RCLCPP_INFO(this->get_logger(),
                "Sending goal: target_mode=%d stop_program=%s play_program=%s",
                static_cast<int>(target_mode),
                (stop_program ? "true" : "false"),
                (play_program ? "true" : "false"));

    rclcpp_action::Client<SetMode>::SendGoalOptions send_goal_options;

    // auto send_goal_options =
    //     rclcpp_action::Client<SetMode>::SendGoalOptions();
    send_goal_options.feedback_callback =
        std::bind(&ReconnectClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback = std::bind(
        &ReconnectClient::result_callback, this, std::placeholders::_1);

    // set_client_->async_send_goal(goal_msg, send_goal_options);
}

void ReconnectClient::feedback_callback(
    GoalHandleSetMode::SharedPtr,
    const std::shared_ptr<const SetMode::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(),
                "Feedback - current_robot_mode: %d, current_safety_mode: %d",
                feedback->current_robot_mode, feedback->current_safety_mode);
}

void ReconnectClient::result_callback(
    const GoalHandleSetMode::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(),
                    "SetMode result: success=%s, message=%s",
                    result.result->success ? "true" : "false",
                    result.result->message.c_str());
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "SetMode goal was aborted");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "SetMode goal was canceled");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(),
                     "Unknown result code from SetMode action");
        break;
    }
}

void ReconnectClient::timerCallback() {
    using namespace std::chrono_literals;
    if (!status_client_->wait_for_service(5s)) {
        RCLCPP_WARN(
            this->get_logger(),
            "[ReconnectClient] /dashboard_client/get_robot_mode not available");
        return;
    }
    // if (!set_client_->wait_for_action_server(std::chrono::seconds(5))) {
    //     RCLCPP_WARN(
    //         this->get_logger(),
    //         "[ReconnectClient] /dashboard_client/set_mode not available");
    //     return;
    // }

    auto request =
        std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>();
    auto future_result = status_client_->async_send_request(request);

    auto status = future_result.wait_for(std::chrono::seconds(5));

    // if (status == std::future_status::ready) {
    auto response = future_result.get();
    if (response->success) {
        current_mode_.store(response->robot_mode.mode);

        RCLCPP_INFO(this->get_logger(),
                    "[ReconnectClient] Polled Robot mode: %d, answer: %s",
                    response->robot_mode.mode, response->answer.c_str());

        if (current_mode_.load() != RUNNING) {
            RCLCPP_INFO(this->get_logger(), "Requesting RUNNING...");
            this->send_goal(RUNNING, true, false);

            // rclcpp::sleep_for(std::chrono::seconds(1));
            trigger_client = this->create_client<std_srvs::srv::Trigger>(
                "/io_and_status_controller/resend_robot_program");
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = trigger_client->async_send_request(request);
            // rclcpp::sleep_for(std::chrono::seconds(5));
            //
            // auto robot_mode_node =
            // std::make_shared<GetRobotModeClient>(); auto robot_set_node =
            // std::make_shared<SetModeActionClient>();
            // RCLCPP_INFO(node->get_logger(), "Requesting POWER_ON...");
            // node->send_goal(POWER_ON);
        }
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "[ReconnectClient] RobotMode call succeeded but "
                    "reported failure: %s",
                    response->answer.c_str());
        current_mode_.store(DISCONNECTED);
    }
}
// else {
//        RCLCPP_ERROR(this->get_logger(),
//                     "[ReconnectClient] Timeout/error calling
//                     get_robot_mode");
//        current_mode_.store(DISCONNECTED);
//    }
// }

int main(int argc, char *argv[]) {
    // rclcpp::init(argc, argv);
    // auto node = std::make_shared<ReconnectClient>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    // return 0;

    //     rclcpp::init(argc, argv);
    //     auto node = std::make_shared<ReconnectClient>();

    //     // Possibly a loop for repeated checks, or just do everything once:
    //     while (rclcpp::ok()) {
    //         node->timerCallback(); // call it manually
    //         rclcpp::sleep_for(std::chrono::seconds(10));
    //     }

    //     rclcpp::shutdown();
    //     return 0;
    // }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("get_robot_mode");
    auto client = node->create_client<ur_dashboard_msgs::srv::GetRobotMode>(
        "/dashboard_client/get_robot_mode");

    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "get_robot_mode not available");
        return 1;
    }

    auto request =
        std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>();
    while (true) {
        auto future = client->async_send_request(request);
        auto ret = rclcpp::spin_until_future_complete(node, future,
                                                      std::chrono::seconds(5));
        if (ret == rclcpp::FutureReturnCode::SUCCESS) {
            auto resp = future.get();
            RCLCPP_DEBUG(node->get_logger(), "Success: mode=%d, answer = % s ",
                         resp->robot_mode.mode, resp->answer.c_str());

            if (resp->robot_mode.mode != RUNNING) {
                RCLCPP_INFO(node->get_logger(),
                            "Resending external control...");

                auto trigger_client =
                    node->create_client<std_srvs::srv::Trigger>(
                        "/io_and_status_controller/resend_robot_program");
                auto request =
                    std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = trigger_client->async_send_request(request);
                rclcpp::sleep_for(std::chrono::seconds(5));
            }

            if (resp->robot_mode.mode == POWER_OFF) {
                RCLCPP_INFO(node->get_logger(), "Powering on robot.");

                auto trigger_client =
                    node->create_client<std_srvs::srv::Trigger>(
                        "/dashboard_client/poweron");
                auto request =
                    std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = trigger_client->async_send_request(request);
                rclcpp::sleep_for(std::chrono::seconds(5));
            }
        } else {
            RCLCPP_ERROR(node->get_logger(),
                         "Timeout or error calling get_robot_mode");
        }
        rclcpp::sleep_for(std::chrono::seconds(3));
    }

    rclcpp::shutdown();
    return 0;
}
