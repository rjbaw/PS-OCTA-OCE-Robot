/**
 * @file move_z_angle_node.cpp
 * @author rjbaw
 * @brief Node that move the Z-axis of the TCP
 */

#include "move_z_angle_node.hpp"

MoveZAngleActionServer::MoveZAngleActionServer(
    const rclcpp::NodeOptions &options)
    : Node("move_z_angle_action_server",
           rclcpp::NodeOptions(options)
               .automatically_declare_parameters_from_overrides(true)) {}

void MoveZAngleActionServer::init() {
    if (!this->has_parameter("plan_only")) {
        this->declare_parameter<bool>("plan_only", false);
    }
    if (!this->has_parameter("offline_mode")) {
        this->declare_parameter<bool>("offline_mode", false);
    }
    bool plan_only = false;
    bool offline_mode = false;
    if (this->has_parameter("plan_only")) {
        plan_only = this->get_parameter("plan_only").as_bool();
    }
    if (this->has_parameter("offline_mode")) {
        offline_mode = this->get_parameter("offline_mode").as_bool();
    }
    if (!(plan_only || offline_mode)) {
        moveit_cpp_ =
            std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());
        tem_ = moveit_cpp_->getTrajectoryExecutionManagerNonConst();
        planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(
            "ur_manipulator", moveit_cpp_);
    } else {
        RCLCPP_INFO(get_logger(),
                    "Plan-only/Offline mode: skipping MoveIt initialization");
    }

    action_server_ = rclcpp_action::create_server<MoveZAngle>(
        this, "move_z_angle_action",
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const MoveZAngle::Goal> goal) {
            return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
            return this->handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
            this->handle_accepted(goal_handle);
        });
    RCLCPP_INFO(get_logger(),
                "MoveZAngleActionServer using MoveItCpp is ready.");
}

rclcpp_action::GoalResponse MoveZAngleActionServer::handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveZAngle::Goal> goal) {
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(),
                "Received Move Z Angle goal with target_angle = %.2f",
                goal->target_angle);
    radius_ = goal->radius;
    angle_ = goal->angle;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveZAngleActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
    if (!goal_handle->is_active()) {
        RCLCPP_INFO(get_logger(), "Move Z angle goal no longer active");
        return rclcpp_action::CancelResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(),
                "Cancel request received for Move Z Angle.");
    tem_->stopExecution(true);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveZAngleActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        active_goal_handle_->abort(std::make_shared<MoveZAngle::Result>());
    }
    active_goal_handle_ = goal_handle;
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void MoveZAngleActionServer::execute(
    const std::shared_ptr<GoalHandleMoveZAngle> goal_handle) {
    RCLCPP_INFO(get_logger(), "Starting Move Z Angle execution...");
    auto feedback = std::make_shared<MoveZAngle::Feedback>();
    auto result = std::make_shared<MoveZAngle::Result>();

    double target_angle = goal_handle->get_goal()->target_angle;
    RCLCPP_INFO(get_logger(), "Target angle: %.2f deg", target_angle);

    bool plan_only = false;
    bool offline_mode = false;
    if (this->has_parameter("plan_only")) {
        plan_only = this->get_parameter("plan_only").as_bool();
    }
    if (this->has_parameter("offline_mode")) {
        offline_mode = this->get_parameter("offline_mode").as_bool();
    }
    if (plan_only || offline_mode) {
        feedback->debug_msgs =
            "Plan-only/Offline mode: skipping planning and execution.\n";
        feedback->current_z_angle = target_angle;
        goal_handle->publish_feedback(feedback);
        result->status = "Move Z Angle completed (plan-only/offline)\n";
        goal_handle->succeed(result);
        return;
    }

    if (goal_handle->is_canceling()) {
        feedback->debug_msgs = "MoveZAngle was canceled before starting.\n";
        feedback->current_z_angle = 0.0;
        result->status = "Move Z Angle Canceled\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->canceled(result);
        return;
    }

    planning_component_->setStartStateToCurrentState();
    const std::string tool_link =
        this->has_parameter("tool_link")
            ? this->get_parameter("tool_link").as_string()
            : this->declare_parameter<std::string>("tool_link", "tcp");
    moveit::core::RobotStatePtr current_state = moveit_cpp_->getCurrentState();
    Eigen::Isometry3d current_pose =
        current_state->getGlobalLinkTransform(tool_link);
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = moveit_cpp_->getPlanningSceneMonitor()
                                      ->getPlanningScene()
                                      ->getPlanningFrame();
    target_pose.pose = tf2::toMsg(current_pose);

    tf2::Quaternion target_q;
    tf2::Quaternion apply_q;
    tf2::fromMsg(target_pose.pose.orientation, target_q);
    apply_q.setRPY(0, 0, to_radian(target_angle));
    apply_q.normalize();
    target_q = target_q * apply_q;
    target_q.normalize();
    target_pose.pose.orientation = tf2::toMsg(target_q);
    target_pose.pose.position.x += radius_ * std::cos(to_radian(angle_));
    target_pose.pose.position.y += radius_ * std::sin(to_radian(angle_));
    print_target(get_logger(), target_pose.pose);

    moveit::core::RobotStatePtr cur_state = moveit_cpp_->getCurrentState();
    Eigen::Isometry3d start_tcp = cur_state->getGlobalLinkTransform(tool_link);
    const std::string planning_frame = moveit_cpp_->getPlanningSceneMonitor()
                                           ->getPlanningScene()
                                           ->getPlanningFrame();
    const double envelope_radius =
        this->has_parameter("envelope_radius_m")
            ? this->get_parameter("envelope_radius_m").as_double()
            : this->declare_parameter<double>("envelope_radius_m", 0.05);
    moveit_msgs::msg::Constraints envelope = octa_ros::motion::make_envelope(
        start_tcp, planning_frame, tool_link, envelope_radius, M_PI);
    planning_component_->setPathConstraints(envelope);

    planning_component_->setGoal(target_pose, tool_link);

    if (goal_handle->is_canceling()) {
        feedback->debug_msgs = "Move Z Angle was canceled before planning.\n";
        feedback->current_z_angle = 0.0;
        result->status = "Move Z Angle Canceled\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->canceled(result);
        return;
    }

    std::vector<std::string> pipelines =
        this->has_parameter("planning_pipelines")
            ? this->get_parameter("planning_pipelines").as_string_array()
            : this->declare_parameter<std::vector<std::string>>(
                  "planning_pipelines",
                  std::vector<std::string>{"pilz_ptp", "pilz_lin"});
    auto req =
        moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters(
            shared_from_this(), pipelines);

    auto choose_shortest =
        [](const std::vector<planning_interface::MotionPlanResponse>
               &solutions) {
            return *std::min_element(
                solutions.begin(), solutions.end(),
                [](const auto &lhs, const auto &rhs) {
                    if (lhs && rhs) {
                        return robot_trajectory::pathLength(*lhs.trajectory) <
                               robot_trajectory::pathLength(*rhs.trajectory);
                    }
                    return static_cast<bool>(lhs);
                });
        };
    planning_interface::MotionPlanResponse plan_solution =
        planning_component_->plan(req, choose_shortest);
    if (plan_solution.error_code.val !=
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_WARN(get_logger(), "Planning failed!");
        feedback->debug_msgs = "Planning failed!\n";
        feedback->current_z_angle = 0.0;
        result->status = "Move Z angle failed!\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->abort(result);
        return;
    }

    feedback->debug_msgs = "Planning succeeded; starting execution.\n";
    feedback->current_z_angle = 0.0;
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) {
        feedback->debug_msgs = "Canceled before execution.\n";
        feedback->current_z_angle = 0.0;
        result->status = "Move Z Angle Canceled\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->canceled(result);
        return;
    }

    bool execute_success = true;
    if (!(plan_only || offline_mode)) {
        auto exec_status = moveit_cpp_->execute(plan_solution.trajectory);
        execute_success = static_cast<bool>(exec_status);
    } else {
        RCLCPP_INFO(get_logger(), "Plan-only/Offline mode: skipping execution");
    }
    if (!execute_success) {
        RCLCPP_ERROR(get_logger(), "Execution failed!");
        feedback->debug_msgs = "Execution failed!\n";
        feedback->current_z_angle = 0.0;
        result->status = "Move Z angle failed\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->abort(result);
        return;
    }

    if (goal_handle->is_canceling()) {
        feedback->debug_msgs = "Canceled after execution.\n";
        feedback->current_z_angle = 0.0;
        result->status = "Move Z Angle Canceled\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->canceled(result);
        return;
    }

    feedback->debug_msgs = "Move Z Angle completed successfully!\n";
    feedback->current_z_angle = angle_;
    result->status = "Move Z Angle completed\n";
    goal_handle->publish_feedback(feedback);
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Move Z Angle done.");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveZAngleActionServer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
