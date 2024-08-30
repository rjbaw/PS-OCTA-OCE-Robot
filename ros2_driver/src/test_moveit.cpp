#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <csignal>  // Include for signal handling
#include <atomic>   // Include for atomic variables
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

std::atomic<bool> running(true);  // Atomic flag to control the shutdown
moveit::planning_interface::MoveGroupInterface* move_group_ptr = nullptr;  // Pointer to control the move group

// Signal handler function
void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "Signal %d received, cancelling execution...", signum);
    running = false;
    if (move_group_ptr) {
        move_group_ptr->stop();  // Cancel the execution of the plan
    }
    rclcpp::shutdown();
}

double to_radians(double angle) { return (std::numbers::pi/180 * angle); }

int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);

    // Register the signal handler for SIGINT and SIGTERM
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    move_group_interface.setStartStateToCurrentState();

    move_group_ptr = &move_group_interface;  // Assign the pointer to the move group

    // std::string reference_frame = "wrist_3_link";
    // move_group_interface.setPoseReferenceFrame(reference_frame);
    // geometry_msgs::msg::Pose target_pose = move_group_interface.getCurrentPose(reference_frame).pose;
    // target_pose.orientation.x += 0.0;
    // target_pose.orientation.y += 0.0;
    // target_pose.orientation.z += 0.0;
    // target_pose.orientation.w += 0.0;
    // target_pose.position.x += 0.1;
    // target_pose.position.y += 0;
    // target_pose.position.z += 0;

    // Set a target Pose
    auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
// x: 0, y: 0, z: -0.00021918399999999995, qx: 0.008000801860057399, qy: 0, qz: 0, qw: 0.9999679930725763
        msg.orientation.x = -0.4;
        msg.orientation.y = 0.9;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.03;
        msg.position.x = 0.36;
        msg.position.y = -0.13;
        msg.position.z = 0.18;
        //
        // msg.orientation.w = 1.0;
        // msg.position.x = 0.28;
        // msg.position.y = -0.2;
        // msg.position.z = 0.5;
        return msg;
    }();
    // move_group_interface.setPoseTarget(target_pose);
    move_group_interface.setRPYTarget(to_radians(0),to_radians(0),to_radians(0));

    // Check if the program should keep running
    if (running) {
        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        // Execute the plan if successful
        if (success && running) {
            move_group_interface.execute(plan);
        } else if (!running) {
            RCLCPP_INFO(logger, "Execution was cancelled!");
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
