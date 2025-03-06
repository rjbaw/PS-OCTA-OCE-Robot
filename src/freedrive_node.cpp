#include <octa_ros/action/freedrive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class FreedriveActionServer : public rclcpp::Node {
  public:
    using Freedrive = octa_ros::action::Freedrive;
    using GoalHandleFreedrive = rclcpp_action::ServerGoalHandle<Freedrive>;

    FreedriveActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("freedrive_action_server", options) {
        action_server_ = rclcpp_action::create_server<Freedrive>(
            this, "freedrive_action",
            std::bind(&FreedriveActionServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&FreedriveActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&FreedriveActionServer::handle_accepted, this,
                      std::placeholders::_1));
    }

  private:
    rclcpp_action::Server<Freedrive>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleFreedrive> active_goal_handle_;

    // Goal handler: decide if the goal can be accepted
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const Freedrive::Goal> goal) {
        RCLCPP_INFO(get_logger(), "Received Freedrive goal with enable=%s",
                    goal->enable ? "true" : "false");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Cancel handler: handle cancellations
    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
        RCLCPP_INFO(get_logger(), "Freedrive action canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Accepted handler: the goal has been accepted, so start processing it
    void
    handle_accepted(const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
        active_goal_handle_ =
            goal_handle; // Store active goal handle for possible preemption
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    // Execution of the goal
    void execute(const std::shared_ptr<GoalHandleFreedrive> goal_handle) {
        auto feedback = std::make_shared<Freedrive::Feedback>();
        auto result = std::make_shared<Freedrive::Result>();

        // Simulate enabling or disabling freedrive
        RCLCPP_INFO(get_logger(), "Starting Freedrive execution...");
        bool enable = goal_handle->get_goal()->enable;
        feedback->status = enable ? "Freedrive enabled" : "Freedrive disabled";
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        result->success = true;
        result->status = "Freedrive action completed successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Freedrive completed.");
    }
};

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FreedriveActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
