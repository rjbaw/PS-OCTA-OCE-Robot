#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class RunStateListener : public rclcpp::Node {
  public:
    RunStateListener() : Node("run_state_listener") {
        sub_ = create_subscription<std_msgs::msg::Bool>(
            "/run_state", rclcpp::QoS(1),
            std::bind(&RunStateListener::callback, this,
                      std::placeholders::_1));
    }

  private:
    void callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::cout << (msg->data ? "true" : "false") << '\n';
        std::cout.flush();
        rclcpp::shutdown();
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RunStateListener>();
    rclcpp::spin(node);
    return 0;
}
