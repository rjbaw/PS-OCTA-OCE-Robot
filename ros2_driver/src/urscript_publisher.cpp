#include "urscript_publisher.hpp"

urscript_publisher::urscript_publisher()
    : Node("urscript_publisher"), freedrive(false), executed(true),
      traj_requested_(false), use_movej_(true), traj_velocity_(1.0),
      traj_acceleration_(1.0) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    trigger_client = this->create_client<std_srvs::srv::Trigger>(
        "/io_and_status_controller/resend_robot_program");
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/urscript_interface/script_command", qos);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&urscript_publisher::publish_to_robot, this));
}
void urscript_publisher::resend_program() {
    // ros2 service call
    // /io_and_status_controller/resend_robot_program
    // std_srvs/srv/Trigger;
    while (!trigger_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = trigger_client->async_send_request(request);
}
void urscript_publisher::activate_freedrive() {
    freedrive = true;
    executed = false;
    traj_requested_ = false;
}
void urscript_publisher::deactivate_freedrive() {
    freedrive = false;
    executed = false;
}
void urscript_publisher::execute_trajectory(
    const std::vector<std::array<double, 6>> &trajectory, double velocity,
    double acceleration, bool use_movej) {
    trajectory_points_ = trajectory;
    traj_velocity_ = velocity;
    traj_acceleration_ = acceleration;
    use_movej_ = use_movej;
    freedrive = false;
    traj_requested_ = true;
    executed = false;
}
void urscript_publisher::publish_to_robot() {
    if (executed) {
        return;
    }
    if (!traj_requested_) {
        auto message = std_msgs::msg::String();
        executed = true;
        if (freedrive) {
            message.data = R"(
def program():
 global check = "Made it"
 while(True):
  freedrive_mode()
 end
end
	    )";
        }
        if (!freedrive) {
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
            urscript_publisher::resend_program();
        }
    }
    if (traj_requested_) {
        auto message = std_msgs::msg::String();
        std::ostringstream prog;
        prog << "def trajectory_program():\n";
        prog << "  end_freedrive_mode()\n";
        // movej([j0, j1, j2, j3, j4, j5], a=acc, v=vel)
        for (size_t i = 0; i < trajectory_points_.size(); i++) {
            prog << "  ";
            if (use_movej_) {
                prog << "movej([";
            } else {
                prog << "movel([";
            }
            for (int j = 0; j < 6; j++) {
                prog << trajectory_points_[i][j];
                if (j < 5) {
                    prog << ", ";
                }
            }
            prog << "], a=" << traj_acceleration_ << ", v=" << traj_velocity_
                 << ")\n";
        }
        prog << "end\n";
        message.data = prog.str();
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "URscript trajectory published:\n%s",
                    message.data.c_str());
        traj_requested_ = false;
        executed = true;
    }
}
void urscript_publisher::publish_script_now(const std::string &script) {
    auto message = std_msgs::msg::String();
    message.data = script;
    this->freedrive = false;
    this->traj_requested_ = false;
    this->executed = false;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "URScript message published:\n%s",
                script.c_str());
    executed = true;
}
