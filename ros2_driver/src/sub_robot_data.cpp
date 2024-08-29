#include "octa_ros/msg/robotdata.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

class MinimalSubscriber : public rclcpp::Node {
  public:
    MinimalSubscriber()
        : Node("labview_sub"), best_effort(rclcpp::KeepLast(10)) {
        subscription_ = this->create_subscription<octa_ros::msg::Robotdata>(
            "labview_data", best_effort.reliability(be),
            [this](const octa_ros::msg::Robotdata::SharedPtr msg) {
                // Update member variables with incoming message data
                msg_ = msg->msg;
                angle_ = msg->angle;
                circle_state_ = msg->circle_state;
                fast_axis_ = msg->fast_axis;
                apply_config_ = msg->apply_config;
                end_state_ = msg->apply_config;

                // Log the received data
                std::stringstream ss;
                ss << "Subscribing: \n"
                   << " msg: " << msg_ << "\n"
                   << " angle: " << angle_ << "\n"
                   << " circle_state: " << circle_state_ << "\n"
                   << " fast_axis: " << fast_axis_ << "\n"
                   << " apply_config: " << apply_config_ << "\n"
                   << " end_state: " << end_state_ << "\n";

                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            });
    };

  private:
    rclcpp::Subscription<octa_ros::msg::Robotdata>::SharedPtr subscription_;

    rclcpp::QoS best_effort;
    std::string msg_;
    double angle_;
    int circle_state_;
    bool fast_axis_, apply_config_, end_state_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
