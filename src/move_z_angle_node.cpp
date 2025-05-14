/**
 * @file move_z_angle_node.cpp
 * @author rjbaw
 * @brief Node that move the Z-axis of the TCP
 */

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <octa_ros/action/move_z_angle.hpp>

double to_radian_(const double degree) {
    return (std::numbers::pi / 180 * degree);
}

double to_degree_(const double radian) {
    return (180 / std::numbers::pi * radian);
}

void print_target_(rclcpp::Logger const &logger,
                   geometry_msgs::msg::Pose target_pose) {
    RCLCPP_INFO(logger,
                std::format("Target Pose: "
                            " x: {}, y: {}, z: {},"
                            " qx: {}, qy: {}, qz: {}, qw: {}",
                            target_pose.position.x, target_pose.position.y,
                            target_pose.position.z, target_pose.orientation.x,
                            target_pose.orientation.y,
                            target_pose.orientation.z,
                            target_pose.orientation.w)
                    .c_str());
}

class MoveZAngleActionServer : public rclcpp::Node {
    using MoveZAngle = octa_ros::action::MoveZAngle;
    using GoalHandleMoveZAngle = rclcpp_action::ServerGoalHandle<MoveZAngle>;

  public:
    explicit MoveZAngleActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("move_z_angle_action_server",
               // options
               rclcpp::NodeOptions(options)
                   .automatically_declare_parameters_from_overrides(true)) {}
    void init() {
        moveit_cpp_ =
            std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());
        planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(
            "ur_manipulator", moveit_cpp_);

        action_server_ = rclcpp_action::create_server<MoveZAngle>(
            this, "move_z_angle_action",
            std::bind(&MoveZAngleActionServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveZAngleActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&MoveZAngleActionServer::handle_accepted, this,
                      std::placeholders::_1));
        RCLCPP_INFO(get_logger(),
                    "MoveZAngleActionServer using MoveItCpp is ready.");
    }

  private:
    rclcpp_action::Server<MoveZAngle>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleMoveZAngle> active_goal_handle_;

    moveit_cpp::MoveItCppPtr moveit_cpp_;
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;

    double radius_ = 0.0;
    double angle_ = 0.0;

    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const MoveZAngle::Goal> goal) {
        RCLCPP_INFO(this->get_logger(),
                    "Received Move Z Angle goal with target_angle = %.2f",
                    goal->target_angle);
        radius_ = goal->radius;
        angle_ = goal->angle;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
        RCLCPP_INFO(this->get_logger(),
                    "Cancel request received for Move Z Angle.");
        goal_handle->canceled(std::make_shared<MoveZAngle::Result>());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void
    handle_accepted(const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
        if (active_goal_handle_ && active_goal_handle_->is_active()) {
            active_goal_handle_->abort(std::make_shared<MoveZAngle::Result>());
        }
        active_goal_handle_ = goal_handle;
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
        RCLCPP_INFO(get_logger(),
                    "Starting Move Z Angle execution with MoveItCpp...");
        auto feedback = std::make_shared<MoveZAngle::Feedback>();
        auto result = std::make_shared<MoveZAngle::Result>();

        double target_angle = goal_handle->get_goal()->target_angle;
        RCLCPP_INFO(get_logger(), "Target angle: %.2f deg", target_angle);

        if (goal_handle->is_canceling()) {
            result->result = "MoveZAngle was canceled before starting.";
            goal_handle->canceled(result);
            return;
        }

        planning_component_->setStartStateToCurrentState();
        moveit::core::RobotStatePtr current_state =
            moveit_cpp_->getCurrentState();
        Eigen::Isometry3d current_pose =
            current_state->getGlobalLinkTransform("tcp");
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = moveit_cpp_->getPlanningSceneMonitor()
                                          ->getPlanningScene()
                                          ->getPlanningFrame();
        ;
        target_pose.pose = tf2::toMsg(current_pose);

        tf2::Quaternion target_q;
        tf2::Quaternion apply_q;
        tf2::fromMsg(target_pose.pose.orientation, target_q);
        apply_q.setRPY(0, 0, to_radian_(target_angle));
        apply_q.normalize();
        target_q = target_q * apply_q;
        target_q.normalize();
        target_pose.pose.orientation = tf2::toMsg(target_q);
        target_pose.pose.position.x += radius_ * std::cos(to_radian_(angle_));
        target_pose.pose.position.y += radius_ * std::sin(to_radian_(angle_));
        print_target_(get_logger(), target_pose.pose);

        planning_component_->setGoal(target_pose, "tcp");

        if (goal_handle->is_canceling()) {
            result->result = "Move Z Angle was canceled before planning.";
            goal_handle->canceled(result);
            return;
        }

        auto req =
            moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters(
                shared_from_this(), {"pilz_ptp", "pilz_lin", "ompl_rrtc"});

        // auto stop_on_first =
        //     [](const PlanningComponent::PlanSolutions &sols,
        //        const auto &) { return sols.hasSuccessfulSolution();
        //        };
        auto choose_shortest =
            [](const std::vector<planning_interface::MotionPlanResponse>
                   &sols) {
                return *std::min_element(
                    sols.begin(), sols.end(), [](const auto &a, const auto &b) {
                        if (a && b)
                            return robot_trajectory::pathLength(*a.trajectory) <
                                   robot_trajectory::pathLength(*b.trajectory);
                        return static_cast<bool>(a);
                    });
            };
        planning_interface::MotionPlanResponse plan_solution =
            planning_component_->plan(req, choose_shortest);
        if (plan_solution.error_code.val !=
            moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_WARN(get_logger(), "Planning failed!");
            result->result = "Planning failed!";
            goal_handle->abort(result);
            return;
        }

        if (goal_handle->is_canceling()) {
            result->result = "Canceled before execution.";
            goal_handle->canceled(result);
            return;
        }

        bool execute_success = moveit_cpp_->execute(plan_solution.trajectory);
        if (!execute_success) {
            RCLCPP_ERROR(get_logger(), "Execution failed!");
            result->result = "Execution failed!";
            goal_handle->abort(result);
            return;
        }

        feedback->current_z_angle = target_angle;
        goal_handle->publish_feedback(feedback);
        result->result = "Move Z Angle completed successfully!";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Move Z Angle done.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveZAngleActionServer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
