#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <csignal>
#include <atomic> 
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

std::atomic<bool> running(true);  // Atomic flag to control the shutdown
moveit::planning_interface::MoveGroupInterface* move_group_ptr = nullptr;  // Pointer to control the move group

// Signal handler function
void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "Signal %d received, cancelling execution...", signum);
    running = false;
    if (move_group_ptr) {
        move_group_ptr->stop();  
    }
    rclcpp::shutdown();
}

double to_radians(double angle) { return (std::numbers::pi/180 * angle); }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    auto const logger = rclcpp::get_logger("hello_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // rclcpp::spin_some(node);

    // std::string robot_description;
    // if (!node->get_parameter("robot_description", robot_description)) {
    //     RCLCPP_ERROR(logger, "Failed to get robot_description parameter. "
    //                          "Ensure the parameter is set.");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    move_group_interface.setPlanningTime(10.0);
    move_group_interface.setStartStateToCurrentState();
    // RCLCPP_INFO(logger, "Waiting for MoveGroupInterface to be ready...");
    // rclcpp::sleep_for(std::chrono::seconds(2));

    move_group_ptr = &move_group_interface;

    // std::string reference_frame = "wrist_3_link";
    // move_group_interface.setPoseReferenceFrame(reference_frame);
    // geometry_msgs::msg::Pose target_pose =
    //     move_group_interface.getCurrentPose(reference_frame).pose;

    // const moveit::core::JointModelGroup *joint_model_group =
    //     move_group_interface.getCurrentState()->getJointModelGroup(
    //         "ur_manipulator");
    geometry_msgs::msg::Pose target_pose =
        move_group_interface.getCurrentPose().pose;
    target_pose.orientation.x += 0.0;
    target_pose.orientation.y += 0.0;
    target_pose.orientation.z += 0.0;
    target_pose.orientation.w += 0.0;
    target_pose.position.x += 0.1;
    target_pose.position.y += 0;
    target_pose.position.z += 0;

    // auto const target_pose = [] {
    //     geometry_msgs::msg::Pose msg;
    //     // real environment
    //     msg.orientation.x = -0.4;
    //     msg.orientation.y = 0.9;
    //     msg.orientation.z = 0.0;
    //     msg.orientation.w = 0.03;
    //     msg.position.x = 0.36;
    //     msg.position.y = -0.13;
    //     msg.position.z = 0.18;
    //     return msg;
    // }();
    move_group_interface.setPoseTarget(target_pose);
    // move_group_interface.setRPYTarget(to_radians(0),to_radians(0),to_radians(0));

    if (running) {
        auto const [success, plan] = [&move_group_interface] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        if (success && running) {
            move_group_interface.execute(plan);
        } else if (!running) {
            RCLCPP_INFO(logger, "Execution was cancelled!");
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }
    }

    rclcpp::shutdown();
    return 0;
}
