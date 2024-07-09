#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateListener : public rclcpp::Node
{
public:
  JointStateListener()
  : Node("joint_state_listener")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointStateListener::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("/robot_state", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    double sum_vel = 0.0;
    for (const auto& velocity : msg->velocity) {
      sum_vel += velocity;
    }

    auto state_msg = std_msgs::msg::Int32();
    state_msg.data = (sum_vel != 0.0);

    publisher_->publish(state_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateListener>());
  rclcpp::shutdown();
  return 0;
}
