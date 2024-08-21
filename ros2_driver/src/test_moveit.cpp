#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "octa_ros/msg/labviewdata.hpp"
#include "octa_ros/msg/robotdata.hpp"

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

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
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  // rclcpp::spin(std::make_shared<MinimalSubscriber>());
	// intialize ROS and create the Node
	rclcpp::init(argc, argv);
	auto const node = std::make_shared<rclcpp::Node>(
			"hello_modeit",
			rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
			);
	auto const logger = rclcpp::get_logger("hello_moveit");

	// main
	using moveit::planning_interface::MoveGroupInterface;
	auto move_group_interface = MoveGroupInterface(node, "manipulator");

	auto const target_pose = []{
		geometry_msgs::msg::Pose msg;
		msg.orientation.w = 1.0;
		msg.position.x = 0.28;
		msg.position.y = -0.2;
		msg.position.z = 0.5;
		return msg;
	}();
	move_group_interface.setPoseTarget(target_pose);

	//create a plan to that target pose
	auto const [success, plan] = [&move_group_interface]{
		moveit::planning_interface::MoveGroupInterface::Plan msg;
		auto const ok = static_cast<bool>(move_group_interface.plan(msg));
		return std::make_pair(ok, msg);
	}();
	// Execute the plan
	if(success) {
		move_group_interface.execute(plan);
	} else {
		RCLCPP_ERROR(logger, "Planning failed!");
	}
	
	rclcpp::shutdown();
	return 0;
}
