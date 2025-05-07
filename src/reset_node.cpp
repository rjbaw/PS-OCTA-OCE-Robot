/**
 * @file reset_node.cpp
 * @author rjbaw
 * @brief Node that sends default position in URscript to the robot driver
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <thread>

#include "utils.hpp"
#include <octa_ros/action/reset.hpp>

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

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/urscript_interface/script_command", qos);
    }

  private:
    rclcpp_action::Server<ResetAction>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleResetAction> active_goal_handle_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const ResetAction::Goal> goal) {
        RCLCPP_INFO(get_logger(), "Received Reset goal with reset=%s",
                    goal->reset ? "true" : "false");
        if (active_goal_handle_ && active_goal_handle_->is_active()) {
            RCLCPP_INFO(get_logger(), "Reset goal still processing!");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleResetAction> goal_handle) {
        RCLCPP_INFO(get_logger(), "Reset action canceled");
        publish_stop();
        goal_handle->canceled(std::make_shared<ResetAction::Result>());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void
    handle_accepted(const std::shared_ptr<GoalHandleResetAction> goal_handle) {

        active_goal_handle_ = goal_handle;
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void publish_stop(double decel = 2.0) {
        if (publisher_->get_subscription_count() == 0) {
            RCLCPP_WARN(get_logger(),
                        "publish_stop(): no URScript subscriber – skip");
            return;
        }
        std_msgs::msg::String msg;
        std::ostringstream prog;
        prog << "def stop_motion():\n";
        prog << "  stopj(" << decel << ")\n";
        prog << "end\n";
        msg.data = prog.str();
        publisher_->publish(msg);
        RCLCPP_INFO(get_logger(), "Sent stopj(%g) to robot", decel);
    }

    void execute(const std::shared_ptr<GoalHandleResetAction> goal_handle) {
        auto feedback = std::make_shared<ResetAction::Feedback>();
        auto result = std::make_shared<ResetAction::Result>();

        RCLCPP_INFO(get_logger(), "Starting Reset execution...");
        feedback->status = "Resetting... Please wait";
        goal_handle->publish_feedback(feedback);

        // move_group_interface.setJointValueTarget("shoulder_pan_joint",
        //                                          to_radian(0.0));
        // move_group_interface.setJointValueTarget("shoulder_lift_joint",
        //                                          -to_radian(60.0));
        // move_group_interface.setJointValueTarget("elbow_joint",
        //                                          to_radian(90.0));
        // move_group_interface.setJointValueTarget("wrist_1_joint",
        //                                          to_radian(-120.0));
        // move_group_interface.setJointValueTarget("wrist_2_joint",
        //                                          to_radian(-90.0));
        // move_group_interface.setJointValueTarget("wrist_3_joint",
        //                                          to_radian(-135.0));

        float robot_vel = 0.8;
        float robot_acc = 0.8;
        double j0 = to_radian(0.0);
        double j1 = to_radian(-60.0);
        double j2 = to_radian(90.0);
        double j3 = to_radian(-120.0);
        double j4 = to_radian(-90.0);
        double j5 = to_radian(-135.0);
        std::ostringstream prog;
        prog << "def reset_position():\n";
        prog << "  end_freedrive_mode()\n";
        prog << "  movej([" << j0 << ", " << j1 << ", " << j2 << ", " << j3
             << ", " << j4 << ", " << j5 << "], " << "a=" << robot_acc
             << ", v=" << robot_vel << ")\n";
        prog << "end\n";
        auto message = std_msgs::msg::String();
        message.data = prog.str();

        const auto start_time = now();
        const auto timeout = rclcpp::Duration::from_seconds(5.0);
        while (publisher_->get_subscription_count() == 0) {
            if ((now() - start_time) > timeout) {
                RCLCPP_ERROR(get_logger(),
                             "No subscriber found. Aborting Reset.");
                result->success = false;
                result->status = "No URScript subscriber available";
                goal_handle->abort(result);
                return;
            }
        }

        publisher_->publish(message);

        for (int i = 0; i < 40; ++i) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->status = "Cancel requested!";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Cancel requested!");
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        result->success = true;
        result->status = "Reset action completed successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Reset completed.");
        active_goal_handle_.reset();

        // std::string pkg_share =
        //     ament_index_cpp::get_package_share_directory("octa_ros");
        // std::string bg_path = pkg_share + "/config/bg.jpg";
        // cv::Mat bg;
        // cv::imwrite(bg_path, bg, cv::IMREAD_GRAYSCALE);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResetActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
