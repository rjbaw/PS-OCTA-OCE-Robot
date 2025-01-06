#include "urscript_publisher.hpp"

urscript_publisher::urscript_publisher()
    : Node("urscript_publisher"), freedrive(false), executed(true) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    trigger_client = this->create_client<std_srvs::srv::Trigger>(
        "/io_and_status_controller/resend_robot_program");
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/urscript_interface/script_command", qos);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&urscript_publisher::publish_to_robot, this));
}
void urscript_publisher::activate_freedrive() {
    freedrive = true;
    executed = false;
}
void urscript_publisher::deactivate_freedrive() {
    freedrive = false;
    executed = false;
}

void urscript_publisher::publish_to_robot() {
    if (!executed) {
        executed = true;
        auto message = std_msgs::msg::String();
        if (freedrive) {
            message.data = R"(
def program():
 global check = "Made it"
 while(True):
  freedrive_mode()
 end
end
            )";
        } else {
            message.data = R"(
def program():
 end_freedrive_mode()
end
            )";
        }
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "URscript message published: '%s'",
                    message.data.c_str());
        if (!freedrive) {
            // ros2 service call
            // /io_and_status_controller/resend_robot_program
            // std_srvs/srv/Trigger;
            while (!trigger_client->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(), "Waiting for service...");
            }
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = trigger_client->async_send_request(request);
        }
    }
}
