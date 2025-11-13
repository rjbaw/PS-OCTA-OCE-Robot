/**
 * @file move_node.cpp
 * @author rjbaw
 * @brief Movement node
 */

#include "move_node.hpp"

MoveActionServer::MoveActionServer(const rclcpp::NodeOptions &options)
    : Node("move_action_server",
           rclcpp::NodeOptions(options)
               .automatically_declare_parameters_from_overrides(true)) {}

void MoveActionServer::init() {
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

    action_server_ = rclcpp_action::create_server<Move>(
        this, "move_action",
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const Move::Goal> goal) {
            return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleMove> goal_handle) {
            return this->handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleMove> goal_handle) {
            this->handle_accepted(goal_handle);
        });
    RCLCPP_INFO(get_logger(), "MoveActionServer using MoveItCpp is ready.");
}

rclcpp_action::GoalResponse MoveActionServer::handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Move::Goal> goal) {
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(),
                "Received Move goal with target_angle = %.2f",
                goal->target_angle);
    radius_ = goal->radius;
    angle_ = goal->angle;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
    if (!goal_handle->is_active()) {
        RCLCPP_INFO(get_logger(), "Move goal no longer active");
        return rclcpp_action::CancelResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Cancel request received for Move.");
    tem_->stopExecution(true);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        active_goal_handle_->abort(std::make_shared<Move::Result>());
    }
    active_goal_handle_ = goal_handle;
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void MoveActionServer::execute(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(get_logger(), "Starting Move execution...");
    auto feedback = std::make_shared<Move::Feedback>();
    auto result = std::make_shared<Move::Result>();

    double offset_x = goal_handle->get_goal()->offset_x;
    double offset_y = goal_handle->get_goal()->offset_y;
    double target_angle = goal_handle->get_goal()->target_angle;
    bool apply_offset = goal_handle->get_goal()->apply_offset;
    RCLCPP_INFO(get_logger(), "Offset X: %.2f mm", offset_x);
    RCLCPP_INFO(get_logger(), "Offset Y: %.2f mm", offset_y);
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
        goal_handle->publish_feedback(feedback);
        result->status = "Move completed (plan-only/offline)\n";
        goal_handle->succeed(result);
        return;
    }

    if (goal_handle->is_canceling()) {
        feedback->debug_msgs = "Move was canceled before starting.\n";
        result->status = "Move Canceled\n";
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

    if (apply_offset) {
        target_pose.pose.position.x += offset_x * 0.001;
        target_pose.pose.position.y += offset_y * 0.001;
    } else {
        if ((goal_handle->get_goal()->centre_set) || !centre_set_) {
            centre_xyz_ = current_pose.translation();
            const double scanning_radius = 8.6 / 2.0;
            centre_xyz_.y() -= (scanning_radius - radius_) * 0.001;
            centre_set_ = true;
            RCLCPP_INFO(get_logger(),
                        "Captured centre at angle=0 -> (%.4f, %.4f, %.4f)m",
                        centre_xyz_.x(), centre_xyz_.y(), centre_xyz_.z());
        } else {
            const double path_angle_deg = angle_ + target_angle;
            const double dx =
                -radius_ * std::cos(to_radian(path_angle_deg)) * 0.001;
            const double dy =
                radius_ * std::sin(to_radian(path_angle_deg)) * 0.001;

            target_pose.pose.position.x = centre_xyz_.x() + dx;
            target_pose.pose.position.y = centre_xyz_.y() + dy;
            target_pose.pose.position.z = centre_xyz_.z();
        }

        tf2::Quaternion target_q;
        tf2::Quaternion apply_q;
        tf2::fromMsg(target_pose.pose.orientation, target_q);
        apply_q.setRPY(0, 0, to_radian(target_angle));
        apply_q.normalize();
        target_q = target_q * apply_q;
        target_q.normalize();
        target_pose.pose.orientation = tf2::toMsg(target_q);
    }

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
        feedback->debug_msgs = "Move was canceled before planning.\n";
        result->status = "Move Canceled\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->canceled(result);
        return;
    }

    const std::vector<std::string> pipelines = {"pilz_ptp", "pilz_lin"};
    auto req =
        moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters(
            shared_from_this(), pipelines);

    const bool ccw = (to_radian(target_angle) > 0.0);
    auto enforce_direction =
        [tool_link,
         ccw](const std::vector<planning_interface::MotionPlanResponse>
                  &solutions) -> planning_interface::MotionPlanResponse {
        planning_interface::MotionPlanResponse best_sol;
        double best_traj_len = std::numeric_limits<double>::infinity();

        for (const auto &sol : solutions) {
            if (!sol || !sol.trajectory) {
                continue;
            }
            const auto &traj = *sol.trajectory;
            if (traj.getWayPointCount() < 2) {
                continue;
            }

            const Eigen::Matrix3d wp0 =
                traj.getWayPoint(0).getGlobalLinkTransform(tool_link).linear();
            const Eigen::Matrix3d wp1 =
                traj.getWayPoint(1).getGlobalLinkTransform(tool_link).linear();

            const Eigen::Matrix3d Rrel = wp0.transpose() * wp1;
            const Eigen::AngleAxisd angle_axis(Rrel);
            const double yaw = angle_axis.angle() * angle_axis.axis().z();

            if (ccw ? (yaw < 0.0) : (yaw > 0.0)) {
                continue;
            }

            const double traj_len =
                robot_trajectory::pathLength(*sol.trajectory);
            if (traj_len < best_traj_len) {
                best_traj_len = traj_len;
                best_sol = sol;
            }
        }

        if (best_sol) {
            return best_sol;
        }

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
        planning_component_->plan(req, enforce_direction);
    if (plan_solution.error_code.val !=
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_WARN(get_logger(), "Planning failed!");
        feedback->debug_msgs = "Planning failed!\n";
        result->status = "Move failed!\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->abort(result);
        return;
    }

    feedback->debug_msgs = "Planning succeeded; starting execution.\n";
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) {
        feedback->debug_msgs = "Canceled before execution.\n";
        result->status = "Move Canceled\n";
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
        result->status = "Move Z angle failed\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->abort(result);
        return;
    }

    if (goal_handle->is_canceling()) {
        feedback->debug_msgs = "Canceled after execution.\n";
        result->status = "Move Canceled\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->canceled(result);
        return;
    }

    feedback->debug_msgs = "Move completed successfully!\n";
    result->status = "Move completed\n";
    goal_handle->publish_feedback(feedback);
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Move done.");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveActionServer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
