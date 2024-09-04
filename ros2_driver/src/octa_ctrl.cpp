#include "octa_ros/msg/labviewdata.hpp"
#include "octa_ros/msg/robotdata.hpp"
#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <moveit/planning_interface/planning_interface.h>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include <moveit_msgs/msg/attached_collision_object.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

using namespace std::chrono_literals;

#include <csignal>  // Add this include for signal handling
std::atomic<bool> running(true);  // Add this global variable to control the loop
void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "Signal %d received, shutting down...", signum);
    running = false;
    rclcpp::shutdown();
}

class dds_publisher : public rclcpp::Node {
  public:
    dds_publisher() : dds_publisher("", 0.0, 0, false, false, false) {}

    dds_publisher(std::string msg = "", double angle = 0.0,
                  int circle_state = 0, bool fast_axis = false,
                  bool apply_config = false, bool end_state = false)
        : Node("pub_robot_data"), msg(msg), angle(angle),
          circle_state(circle_state), fast_axis(fast_axis),
          apply_config(apply_config), end_state(end_state) {
        publisher_ =
            this->create_publisher<octa_ros::msg::Robotdata>("robot_data", 10);

        timer_ = this->create_wall_timer(10ms, [this]() {
            // create string obj.
            auto message = octa_ros::msg::Robotdata();
            message.msg = this->msg;
            message.angle = this->angle;
            message.circle_state = this->circle_state;
            message.fast_axis = this->fast_axis;
            message.apply_config = this->apply_config;
            message.end_state = this->end_state;
            // RCLCPP_INFO(this->get_logger(),
            //             std::format("Publishing: \n"
            //                         " msg: {} \n"
            //                         " angle: {}\n"
            //                         " circle_state: {}\n"
            //                         " apply_config: {}\n",
            //                         " end_state: {}\n", this->msg,
            //                         this->angle, this->circle_state,
            //                         this->apply_config, this->end_state)
            //                 .c_str());
            publisher_->publish(message);
        });
    };

    void set_msg(const std::string &new_msg) { msg = new_msg; }
    void set_angle(double new_angle) { angle = new_angle; }
    void set_circle_state(int new_circle_state) {
        circle_state = new_circle_state;
    }
    void set_fast_axis(bool new_fast_axis) { fast_axis = new_fast_axis; }
    void set_apply_config(bool new_apply_config) {
        apply_config = new_apply_config;
    }
    void set_end_state(bool new_end_state) { end_state = new_end_state; }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr publisher_;
    std::string msg;
    double angle;
    int circle_state;
    bool fast_axis, apply_config, end_state;
};

class dds_subscriber : public rclcpp::Node {
  public:
    dds_subscriber()
        : Node("sub_labview"), best_effort(rclcpp::KeepLast(10))

    {
        subscription_ = this->create_subscription<octa_ros::msg::Labviewdata>(
            "labview_data", best_effort.reliability(be),
            [this](const octa_ros::msg::Labviewdata::SharedPtr msg) {
                robot_vel_ = msg->robot_vel;
                robot_acc_ = msg->robot_acc;
                z_tolerance_ = msg->z_tolerance;
                angle_tolerance_ = msg->angle_tolerance;
                radius_ = msg->radius;
                angle_limit_ = msg->angle_limit;
                num_pt_ = msg->num_pt;
                dz_ = msg->dz;
                drot_ = msg->drot;
                autofocus_ = msg->autofocus;
                freedrive_ = msg->freedrive;
                previous_ = msg->previous;
                next_ = msg->next;
                home_ = msg->home;
                reset_ = msg->reset;
                fast_axis_ = msg->fast_axis;
                // RCLCPP_INFO(this->get_logger(),
                //             std::format("Subscribing: \n"
                //                         " robot_vel: {}\n"
                //                         " robot_acc: {}\n"
                //                         " z_tolerance: {}\n"
                //                         " angle_tolerance: {}\n"
                //                         " radius: {}\n"
                //                         " angle_limit: {}\n"
                //                         " num_pt: {}\n"
                //                         " dz: {}\n"
                //                         " drot: {}\n"
                //                         " autofocus: {}\n"
                //                         " freedrive: {}\n"
                //                         " previous: {}\n"
                //                         " next: {}\n"
                //                         " home: {}\n"
                //                         " reset: {}\n"
                //                         " fast_axis: {}\n",
                //                         robot_vel_, robot_acc_, z_tolerance_,
                //                         angle_tolerance_, radius_,
                //                         angle_limit_, num_pt_, dz_, drot_,
                //                         autofocus_, freedrive_, previous_,
                //                         next_, home_, reset_, fast_axis_)
                //                 .c_str());
            });
    };
    double robot_vel() { return robot_vel_; };
    double robot_acc() { return robot_acc_; };
    double z_tolerance() { return z_tolerance_; };
    double angle_tolerance() { return angle_tolerance_; };
    double radius() { return radius_; };
    double angle_limit() { return angle_limit_; };
    int num_pt() { return num_pt_; };
    double dz() { return dz_; };
    double drot() { return drot_; };
    bool autofocus() { return autofocus_; };
    bool freedrive() { return freedrive_; };
    bool previous() { return previous_; };
    bool next() { return next_; };
    bool home() { return home_; };
    bool reset() { return reset_; };
    bool fast_axis() { return fast_axis_; };

  private:
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
    double robot_vel_ = 0, robot_acc_ = 0, z_tolerance_ = 0,
           angle_tolerance_ = 0, radius_ = 0, angle_limit_ = 0, dz_ = 0,
           drot_ = 0;
    int num_pt_ = 0;
    bool autofocus_ = false, freedrive_ = false, previous_ = false,
         next_ = false, home_ = false, reset_ = false, fast_axis_ = false;
};

double to_radian(double degree) { return (std::numbers::pi / 180 * degree); }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto const move_group_node =
        std::make_shared<rclcpp::Node>("node_moveit", node_options);

    auto const logger = rclcpp::get_logger("logger_planning");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(move_group_node, "ur_manipulator");

    rclcpp::executors::SingleThreadedExecutor exec;
    auto subscriber_node = std::make_shared<dds_subscriber>();
    exec.add_node(subscriber_node);
    std::thread([&exec]() { exec.spin(); }).detach();

    double robot_vel = subscriber_node->robot_vel();
    double robot_acc = subscriber_node->robot_acc();
    double z_tolerance = subscriber_node->z_tolerance();
    double angle_tolerance = subscriber_node->angle_tolerance();
    double radius = subscriber_node->radius();
    double angle_limit = subscriber_node->angle_limit();
    int num_pt = subscriber_node->num_pt();
    double dz = subscriber_node->dz();
    double drot = subscriber_node->drot();
    bool autofocus = subscriber_node->autofocus();
    bool freedrive = subscriber_node->freedrive();
    bool previous = subscriber_node->previous();
    bool next = subscriber_node->next();
    bool home = subscriber_node->home();
    bool reset = subscriber_node->reset();
    bool fast_axis = subscriber_node->fast_axis();

    std::string msg;
    double angle = 0.0;
    int circle_state = 1;
    bool apply_config = false;
    bool end_state = false;

    rclcpp::executors::SingleThreadedExecutor exec_pub;
    auto publisher_node = std::make_shared<dds_publisher>(
        msg = msg, angle = angle, circle_state = circle_state,
        fast_axis = fast_axis, apply_config = apply_config,
        end_state = end_state);
    exec_pub.add_node(publisher_node);
    std::thread([&exec_pub]() { exec_pub.spin(); }).detach();

    while (rclcpp::ok() && running) {
        autofocus = subscriber_node->autofocus();
        freedrive = subscriber_node->freedrive();
        if (freedrive || !autofocus) {
            continue;
        }
        apply_config = false;
        end_state = false;

        robot_vel = subscriber_node->robot_vel();
        robot_acc = subscriber_node->robot_acc();
        z_tolerance = subscriber_node->z_tolerance();
        angle_tolerance = subscriber_node->angle_tolerance();
        radius = subscriber_node->radius();
        angle_limit = subscriber_node->angle_limit();
        num_pt = subscriber_node->num_pt();
        dz = subscriber_node->dz();
        drot = subscriber_node->drot();
        previous = subscriber_node->previous();
        next = subscriber_node->next();
        home = subscriber_node->home();
        reset = subscriber_node->reset();
        fast_axis = subscriber_node->fast_axis();

        move_group_interface.setMaxVelocityScalingFactor(robot_vel);
        move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
        move_group_interface.setStartStateToCurrentState();

        double angle_increment = angle_limit / num_pt;
        double roll = 0, pitch = 0, yaw = 0;
        tf2::Quaternion q;
	geometry_msgs::msg::Pose target_pose =
	    move_group_interface.getCurrentPose().pose;

        if (next) {
            angle += angle_increment;
            yaw = to_radian(angle_increment);
        }
        if (previous) {
            angle -= angle_increment;
            yaw = to_radian(-angle_increment);
        }
        if (home) {
            yaw = to_radian(-angle);
        }
        if (fast_axis) {
            roll += 0.77 * drot;
        } else {
            pitch += 0.77 * -drot;
        }
        q.setRPY(roll, pitch, yaw);
        q.normalize();

	// geometry_msgs::msg::Quaternion q_msg;
	// tf2::convert(q_msg, q);
	// tf2::Quaternion target_q;
	// tf2::convert(target_pose.orientation, target_q);
	// target_q = target_q * q;
	// tf2::convert(target_q, target_pose.orientation);
	
	geometry_msgs::msg::Quaternion q_msg;
	tf2::Quaternion target_q;
	q_msg.x = target_pose.orientation.x;
	q_msg.y = target_pose.orientation.y;
	q_msg.z = target_pose.orientation.z;
	q_msg.w = target_pose.orientation.w;
	tf2::fromMsg(q_msg, target_q);
	target_q = q * target_q;
	q_msg = tf2::toMsg(target_q);
	//target_pose.orientation = q_msg;
	
        target_pose.orientation.x = q_msg.x;
        target_pose.orientation.y = q_msg.y;
        target_pose.orientation.z = q_msg.z;
        target_pose.orientation.w = q_msg.w;

	// target_pose.orientation = target_pose.orientation * q;
        // target_pose.orientation.x = q.x();
        // target_pose.orientation.y = q.y();
        // target_pose.orientation.z = q.z();
        // target_pose.orientation.w = q.w();

        target_pose.position.x += radius * std::cos(to_radian(angle));
        target_pose.position.y += radius * std::sin(to_radian(angle));
        target_pose.position.z += dz;

	if (reset) {
		msg = "", angle = 0.0, circle_state = 1;
		target_pose.orientation.x = -0.4;
		target_pose.orientation.y = 0.9;
		target_pose.orientation.z = 0.0;
		target_pose.orientation.w = 0.03;
		target_pose.position.x = 0.36;
		target_pose.position.y = -0.13;
		target_pose.position.z = 0.18;
	}

        move_group_interface.setPoseTarget(target_pose);
        // auto const [success, plan] = [&move_group_interface] {
        //     moveit::planning_interface::MoveGroupInterface::Plan msg;
        //     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        //     return std::make_pair(ok, msg);
        // }();
        // if (success) {
        //     move_group_interface.execute(plan);
        // } else {
        //     RCLCPP_ERROR(logger, "Planning failed!");
        // }
        move_group_interface.move();
	rclcpp::sleep_for(std::chrono::seconds(5));
	
	// bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	// if (success)
	// {
	//     RCLCPP_INFO(rclcpp::get_logger("move_group"), "Motion executed successfully.");
	// }
	// else
	// {
	//     RCLCPP_WARN(rclcpp::get_logger("move_group"), "Motion execution failed.");
	// }

        if ((std::abs(drot) < angle_tolerance) &&
            (std::abs(dz) && z_tolerance)) {
            if (!fast_axis) {
                fast_axis = true;
                apply_config = true;
            } else {
                fast_axis = false;
                apply_config = true;
                end_state = true;
            }
        }

        publisher_node->set_msg(msg);
        publisher_node->set_angle(angle);
        publisher_node->set_circle_state(circle_state);
        publisher_node->set_fast_axis(fast_axis);
        publisher_node->set_apply_config(apply_config);
        publisher_node->set_end_state(end_state);

        // rclcpp::spin_some(std::make_shared<dds_publisher>(
        //     msg = msg, angle = angle, circle_state = circle_state,
        //     fast_axis = fast_axis, apply_config = apply_config,
        //     end_state = end_state));
    }

    rclcpp::shutdown();
    return 0;
}

