#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "octa_ros/msg/labviewdata.hpp"

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("labview_sub"),
        best_effort(rclcpp::KeepLast(10))

  {
    subscription_ = this->create_subscription<octa_ros::msg::Labviewdata>(
        "labview_data", best_effort.reliability(be), [this](const octa_ros::msg::Labviewdata::SharedPtr msg)
        { RCLCPP_INFO(this->get_logger(), "I heard: %f and %i", (double)msg->robot_vel, (int)msg->num_pt); });
  }

private:
  rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr subscription_;

  rclcpp::QoS best_effort;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

