#include <rclcpp/rclcpp.hpp>
#include <ur_dashboard_msgs/srv/get_robot_mode.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_get_robot_mode");
  auto client = node->create_client<ur_dashboard_msgs::srv::GetRobotMode>(
      "/dashboard_client/get_robot_mode");

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "get_robot_mode not available");
    return 1;
  }

  auto request = std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>();
  auto future = client->async_send_request(request);
  auto ret = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));
  if (ret == rclcpp::FutureReturnCode::SUCCESS) {
    auto resp = future.get();
    RCLCPP_INFO(node->get_logger(), "Success: mode=%d, answer=%s",
                resp->robot_mode.mode, resp->answer.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Timeout or error calling get_robot_mode");
  }

  rclcpp::shutdown();
  return 0;
}
