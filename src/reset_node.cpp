#include <octa_ros/action/reset.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ResetActionServer : public rclcpp::Node {
  public:
    using ResetAction = octa_ros::action::Reset;
    using GoalHandleResetAction = rclcpp_action::ServerGoalHandle<ResetAction>;

    ResetActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("reset_action_server", options) {
        action_server_ = rclcpp_action::create_server<ResetAction>(
            this, "reset_action",
            std::bind(&ResetActionServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&ResetActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&ResetActionServer::handle_accepted, this,
                      std::placeholders::_1));
    }

  private:
    rclcpp_action::Server<ResetAction>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleResetAction> active_goal_handle_;

    // Goal handler: decide if the goal can be accepted
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const ResetAction::Goal> goal) {
        RCLCPP_INFO(get_logger(), "Received Reset goal with reset=%s",
                    goal->reset ? "true" : "false");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Cancel handler: handle cancellations
    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleResetAction> goal_handle) {
        RCLCPP_INFO(get_logger(), "Reset action canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Accepted handler: the goal has been accepted, so start processing it
    void
    handle_accepted(const std::shared_ptr<GoalHandleResetAction> goal_handle) {
        active_goal_handle_ =
            goal_handle; // Store active goal handle for possible preemption
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    // Execution of the goal
    void execute(const std::shared_ptr<GoalHandleResetAction> goal_handle) {
        auto feedback = std::make_shared<ResetAction::Feedback>();
        auto result = std::make_shared<ResetAction::Result>();

        // Simulate reset action
        RCLCPP_INFO(get_logger(), "Starting Reset execution...");
        feedback->status = "Resetting...";
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        result->success = true;
        result->status = "Reset action completed successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Reset completed.");
    }
};

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResetActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
