#include <atomic>
#include <csignal>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::atomic<bool> running(true); // Atomic flag to control the shutdown
moveit::planning_interface::MoveGroupInterface *move_group_ptr =
    nullptr; // Pointer to control the move group

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"),
                "Signal %d received, cancelling execution...", signum);
    running = false;
    if (move_group_ptr) {
        move_group_ptr->stop();
    }
    rclcpp::shutdown();
}

double to_radian(double angle) { return (std::numbers::pi / 180 * angle); }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto const node = std::make_shared<rclcpp::Node>(
        "test_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    auto const logger = rclcpp::get_logger("test_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    move_group_interface.setPlanningTime(10.0);
    move_group_interface.setNumPlanningAttempts(5);
    move_group_interface.setStartStateToCurrentState();

    RCLCPP_INFO(logger, "Planning Frame: %s", move_group_interface.getPlanningFrame().c_str());
    auto current_state = move_group_interface.getCurrentState(10);
    if (!current_state) {
        RCLCPP_ERROR(logger, "Failed to get current robot state");
        return 1; 
    }
    if (!current_state->satisfiesBounds()) {
        RCLCPP_ERROR(logger, "Current state is not within bounds, adjusting...");
	current_state->enforceBounds();
    }
    move_group_interface.setStartState(*current_state);

    RCLCPP_INFO(logger, "Waiting for MoveGroupInterface to be ready...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    move_group_ptr = &move_group_interface;

    // std::string reference_frame = "tcp";
    // move_group_interface.setPoseReferenceFrame(reference_frame);
    // geometry_msgs::msg::Pose target_pose =
    //     move_group_interface.getCurrentPose(reference_frame).pose;

    geometry_msgs::msg::Pose target_pose =
        move_group_interface.getCurrentPose().pose;

    RCLCPP_INFO(
        logger,
        std::format(
            "[Current Position] x: {}, y:{}, z:{}, qx:{}, qy:{}, qz:{}, qw:{}",
            target_pose.position.x, target_pose.position.y,
            target_pose.position.z, target_pose.orientation.x,
            target_pose.orientation.y, target_pose.orientation.z,
            target_pose.orientation.w)
            .c_str());

    target_pose.position.x += 0;
    target_pose.position.y += 0;
    target_pose.position.z += 0;

    tf2::Quaternion q;
    tf2::Quaternion target_q;
    q.setRPY(to_radian(0),to_radian(0),to_radian(20));
    q.normalize();
    tf2::fromMsg(target_pose.orientation, target_q);
    target_q = target_q * q;
    target_pose.orientation = tf2::toMsg(target_q);

    // target_pose.orientation.x = -0.7071068;
    // target_pose.orientation.y = 0.7071068;
    // target_pose.orientation.z = 0.0;
    // target_pose.orientation.w = 0.0;
    // target_pose.position.x = 0.4;
    // target_pose.position.y = 0.0;
    // target_pose.position.z = 0.0;

    move_group_interface.setPoseTarget(target_pose);
    // move_group_interface.setPoseTarget(target_pose, reference_frame);

    RCLCPP_INFO(
        logger,
        std::format(
            "[Target Position] x: {}, y:{}, z:{}, qx:{}, qy:{}, qz:{}, qw:{}",
            target_pose.position.x, target_pose.position.y,
            target_pose.position.z, target_pose.orientation.x,
            target_pose.orientation.y, target_pose.orientation.z,
            target_pose.orientation.w)
            .c_str());

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
