#include "dds_publisher.hpp"
#include "dds_subscriber.hpp"
#include "img_subscriber.hpp"
#include "octa_ros/action/focus.hpp"
#include "urscript_publisher.hpp"
#include "utils.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

#include <octa_ros/action/move_z_angle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
std::atomic<bool> running(true);

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"),
                "Signal %d received, shutting down...", signum);
    running = false;
    rclcpp::shutdown();
}

bool tol_measure(double &roll, double &pitch, double &angle_tolerance) {
    return ((std::abs(std::abs(roll)) < to_radian(angle_tolerance)) &&
            (std::abs(std::abs(pitch)) < to_radian(angle_tolerance)));
}

#include <octa_ros/action/move_z_angle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class MoveZAngleActionServer : public rclcpp::Node {
  public:
    using MoveZAngle = octa_ros::action::MoveZAngle;
    using GoalHandleMoveZAngle = rclcpp_action::ServerGoalHandle<MoveZAngle>;

    MoveZAngleActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("move_z_angle_action_server", options) {
        action_server_ = rclcpp_action::create_server<MoveZAngle>(
            this, "move_z_angle_action",
            std::bind(&MoveZAngleActionServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveZAngleActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&MoveZAngleActionServer::handle_accepted, this,
                      std::placeholders::_1));
    }

  private:
    rclcpp_action::Server<MoveZAngle>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleMoveZAngle> active_goal_handle_;

    // Goal handler: decide if the goal can be accepted
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const MoveZAngle::Goal> goal) {
        RCLCPP_INFO(get_logger(),
                    "Received MoveZAngle goal with target_angle=%.2f",
                    goal->target_angle);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Cancel handler: handle cancellations
    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
        RCLCPP_INFO(get_logger(), "MoveZAngle action canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Accepted handler: the goal has been accepted, so start processing it
    void
    handle_accepted(const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
        active_goal_handle_ =
            goal_handle; // Store active goal handle for possible preemption
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    // Execution of the goal
    void execute(const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
        auto feedback = std::make_shared<MoveZAngle::Feedback>();
        auto result = std::make_shared<MoveZAngle::Result>();

        // Simulate moving the Z angle
        RCLCPP_INFO(get_logger(), "Starting MoveZAngle execution...");
        double target_angle = goal_handle->get_goal()->target_angle;
        double current_angle = 0.0;
        while (current_angle < target_angle) {
            if (goal_handle->is_canceling()) {
                result->result = "MoveZAngle was canceled.";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_z_angle = current_angle;
            goal_handle->publish_feedback(feedback);
            current_angle +=
                10.0; // Simulate moving by 10 degrees per iteration
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        result->result = "MoveZAngle completed successfully!";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "MoveZAngle completed.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveZAngleActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
