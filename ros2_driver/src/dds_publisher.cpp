#include "dds_publisher.hpp"

dds_publisher::dds_publisher(std::string msg, double angle, int circle_state,
                             bool fast_axis, bool apply_config, bool end_state,
                             bool scan_3d)
    : Node("pub_robot_data"), msg(msg), angle(angle),
      circle_state(circle_state), fast_axis(fast_axis),
      apply_config(apply_config), end_state(end_state), scan_3d(scan_3d) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    publisher_ =
        this->create_publisher<octa_ros::msg::Robotdata>("robot_data", qos);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
        auto message = octa_ros::msg::Robotdata();
        message.msg = this->msg;
        message.angle = this->angle;
        message.circle_state = this->circle_state;
        message.fast_axis = this->fast_axis;
        message.apply_config = this->apply_config;
        message.end_state = this->end_state;
        message.scan_3d = this->scan_3d;

        if (old_message != message) {
            RCLCPP_INFO(this->get_logger(),
                        "[PUBLISHING] msg: %s, angle: %f, circle_state: %d, "
                        "fast_axis: %s, apply_config: %s, end_state: %s, "
                        "scan_3d: %s",
                        this->msg.c_str(), this->angle, this->circle_state,
                        this->fast_axis ? "true" : "false",
                        this->apply_config ? "true" : "false",
                        this->end_state ? "true" : "false",
                        this->scan_3d ? "true" : "false");
        }

        publisher_->publish(message);

        if (this->apply_config) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - apply_config_time_)
                    .count();
            if (elapsed >= 100) {
                this->apply_config = false;
            }
        }

        old_message = message;
    });
}

void dds_publisher::set_msg(const std::string &new_msg) { msg = new_msg; }
void dds_publisher::set_angle(double new_angle) { angle = new_angle; }
void dds_publisher::set_circle_state(int new_circle_state) {
    circle_state = new_circle_state;
}
void dds_publisher::set_fast_axis(bool new_fast_axis) {
    fast_axis = new_fast_axis;
}
void dds_publisher::set_apply_config(bool new_apply_config) {
    if (new_apply_config) {
        apply_config_time_ = std::chrono::steady_clock::now();
    }
    apply_config = new_apply_config;
}
void dds_publisher::set_end_state(bool new_end_state) {
    end_state = new_end_state;
}
void dds_publisher::set_scan_3d(bool new_scan_3d) { scan_3d = new_scan_3d; }
