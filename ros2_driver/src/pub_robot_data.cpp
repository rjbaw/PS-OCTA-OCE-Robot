// common stl includes
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "octa_ros/msg/robotdata.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("pub_robot_data")
  {
    publisher_ = this->create_publisher<octa_ros::msg::Robotdata>("robot_data", 10);

    // init the timer ptr with a wall timer object.

    timer_ = this->create_wall_timer(
        500ms, [this]()
        {
        // create string obj.
        auto message = octa_ros::msg::Robotdata();
        message.msg = "abc";
        message.angle = (this->count)+1;
        message.circle_state = (this->count)+1;
	message.fast_axis = true;
	message.apply_config = false;
        RCLCPP_INFO(this->get_logger(), "Publishing: %s and %f", message.msg, (double) message.angle);
        //publish message
        publisher_->publish(message);
        this->count++; });
  }

private:
  double count = 0;
  // shared ptr of a timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

