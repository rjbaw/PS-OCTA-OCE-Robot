#include <octa_ros/action/focus.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class FocusActionServer : public rclcpp::Node {
  public:
    using Focus = octa_ros::action::Focus;
    using GoalHandleFocus = rclcpp_action::ServerGoalHandle<Focus>;

    FocusActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("focus_action_server", options) {
        action_server_ = rclcpp_action::create_server<Focus>(
            this, "focus_action",
            std::bind(&FocusActionServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&FocusActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&FocusActionServer::handle_accepted, this,
                      std::placeholders::_1));
    }

  private:
    rclcpp_action::Server<Focus>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleFocus> active_goal_handle_;

    // Goal handler: decide if the goal can be accepted
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const Focus::Goal> goal) {
        RCLCPP_INFO(get_logger(),
                    "Received Focus goal with angle_tolerance=%.2f",
                    goal->angle_tolerance);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Cancel handler: handle cancellations
    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleFocus> goal_handle) {
        RCLCPP_INFO(get_logger(), "Focus action canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Accepted handler: the goal has been accepted, so start processing it
    void handle_accepted(const std::shared_ptr<GoalHandleFocus> goal_handle) {
        active_goal_handle_ =
            goal_handle; // Store active goal handle for possible preemption
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    // Execution of the goal
    void execute(const std::shared_ptr<GoalHandleFocus> goal_handle) {
        auto feedback = std::make_shared<Focus::Feedback>();
        auto result = std::make_shared<Focus::Result>();

        // Simulate focus execution with periodic feedback
        RCLCPP_INFO(get_logger(), "Starting focus execution...");

        for (int i = 0; i < 10; ++i) {
            if (goal_handle->is_canceling()) {
                result->result = "Focus was canceled.";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Focus action canceled.");
                return;
            }

            // Send feedback: progress and status
            feedback->current_angle = i * 10.0; // Simulating progress (angle)
            feedback->debug_msg = "Focusing in progress...";
            goal_handle->publish_feedback(feedback);

            // Simulate work (e.g., focusing logic)
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        result->result = "Focus completed successfully!";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Focus action completed successfully.");
    }
};

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FocusActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
