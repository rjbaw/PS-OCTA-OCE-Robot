/**
 * @file reset_node.cpp
 * @author rjbaw
 * @brief Node that sends default position in URscript to the robot driver
 */

#include "reset_node.hpp"

using namespace std::chrono_literals;

ResetActionServer::ResetActionServer(const rclcpp::NodeOptions &options)
    : Node("reset_action_server",
           rclcpp::NodeOptions(options)
               .automatically_declare_parameters_from_overrides(true)) {}

void ResetActionServer::init() {
    action_server_ = rclcpp_action::create_server<ResetAction>(
        this, "reset_action",
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const ResetAction::Goal> goal) {
            return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleResetAction> goal_handle) {
            return this->handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleResetAction> goal_handle) {
            this->handle_accepted(goal_handle);
        });

    auto qos = rclcpp::SystemDefaultsQoS{};
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/urscript_interface/script_command", qos);
    if (!this->has_parameter("plan_only")) {
        this->declare_parameter<bool>("plan_only", false);
    }
    if (!this->has_parameter("offline_mode")) {
        this->declare_parameter<bool>("offline_mode", false);
    }
    bool plan_only = this->get_parameter("plan_only").as_bool();
    bool offline_mode = this->get_parameter("offline_mode").as_bool();
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
}

rclcpp_action::GoalResponse ResetActionServer::handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ResetAction::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received Reset goal with reset=%s\n",
                goal->reset ? "true" : "false");
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        RCLCPP_INFO(get_logger(), "Reset goal still processing!\n");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ResetActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleResetAction> goal_handle) {
    if (!goal_handle->is_active()) {
        RCLCPP_INFO(get_logger(), "Reset goal no longer active");
        return rclcpp_action::CancelResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Reset action canceled");
    tem_->stopExecution(true);
    publish_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ResetActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleResetAction> goal_handle) {
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        active_goal_handle_->abort(std::make_shared<ResetAction::Result>());
    }
    active_goal_handle_ = goal_handle;
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void ResetActionServer::publish_stop(double decel) {
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

void ResetActionServer::execute(
    const std::shared_ptr<GoalHandleResetAction> goal_handle) {
    auto feedback = std::make_shared<ResetAction::Feedback>();
    auto result = std::make_shared<ResetAction::Result>();

    failed_ = true; // true -> force urscript

    RCLCPP_INFO(get_logger(), "Starting Reset execution...");
    feedback->debug_msgs = "Resetting... Please wait\n";
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) {
        result->status = "Cancel requested!";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Cancel requested!");
        planning_component_->setStartStateToCurrentState();
        return;
    }

    if (!failed_) {
        planning_component_->setStartStateToCurrentState();
        std::vector<double> joint_values = {
            to_radian(0.0),    // shoulder_pan_joint
            -to_radian(60.0),  // shoulder_lift_joint
            to_radian(90.0),   // elbow_joint
            to_radian(-120.0), // wrist_1_joint
            to_radian(-90.0),  // wrist_2_joint
            to_radian(-135.0), // wrist_3_joint
        };
        moveit::core::RobotState goal_state(moveit_cpp_->getRobotModel());
        goal_state.setToDefaultValues();
        goal_state.setJointGroupPositions("ur_manipulator", joint_values);

        moveit::core::RobotStatePtr cur_state = moveit_cpp_->getCurrentState();
        const std::string tool_link =
            this->has_parameter("tool_link")
                ? this->get_parameter("tool_link").as_string()
                : this->declare_parameter<std::string>("tool_link", "tcp");
        Eigen::Isometry3d start_tcp =
            cur_state->getGlobalLinkTransform(tool_link);
        const std::string planning_frame =
            moveit_cpp_->getPlanningSceneMonitor()
                ->getPlanningScene()
                ->getPlanningFrame();
        const double envelope_radius =
            this->has_parameter("envelope_radius_m")
                ? this->get_parameter("envelope_radius_m").as_double()
                : this->declare_parameter<double>("envelope_radius_m", 0.05);
        auto envelope = octa_ros::motion::make_envelope(
            start_tcp, planning_frame, tool_link, envelope_radius, M_PI / 3.0);

        planning_component_->setGoal(goal_state);
        auto req =
            moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters(
                shared_from_this(), {"ompl_rrtc"});
        auto choose_shortest =
            [](const std::vector<planning_interface::MotionPlanResponse>
                   &solutions) {
                return *std::min_element(
                    solutions.begin(), solutions.end(),
                    [](const auto &lhs, const auto &rhs) {
                        if (lhs && rhs) {
                            return robot_trajectory::pathLength(
                                       *lhs.trajectory) <
                                   robot_trajectory::pathLength(
                                       *rhs.trajectory);
                        }
                        return static_cast<bool>(lhs);
                    });
            };
        planning_interface::MotionPlanResponse plan_solution =
            planning_component_->plan(req, choose_shortest);
        planning_component_->setPathConstraints(
            moveit_msgs::msg::Constraints());
        if (plan_solution) {
            if (goal_handle->is_canceling()) {
                result->status = "Cancel requested!";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Cancel requested!");
                planning_component_->setStartStateToCurrentState();
                return;
            }

            bool plan_only = this->get_parameter("plan_only").as_bool();
            bool offline_mode = this->get_parameter("offline_mode").as_bool();
            bool execute_success = true;
            if (!(plan_only || offline_mode)) {
                auto execute_status =
                    moveit_cpp_->execute(plan_solution.trajectory);
                execute_success =
                    (execute_status ==
                     moveit_controller_manager::ExecutionStatus::SUCCEEDED);
            } else {
                RCLCPP_INFO(get_logger(),
                            "Plan-only/Offline mode: skipping execution");
            }

            if (execute_success) {
                RCLCPP_INFO(get_logger(), "Execute Success!");
            } else {
                RCLCPP_INFO(get_logger(), "Execute Failed!");
                if (tem_) {
                    tem_->stopExecution(true);
                }
                failed_ = true;
            }
        } else {
            RCLCPP_INFO(get_logger(),
                        "Planning failed_! Falling back to URScript.");
            failed_ = true;
        }
    }

    if (failed_) {
        RCLCPP_INFO(get_logger(), "URScript fall back");
        float robot_vel = 0.5;
        float robot_acc = 0.5;
        double joint0 = to_radian(0.0);
        double joint1 = to_radian(-60.0);
        double joint2 = to_radian(90.0);
        double joint3 = to_radian(-120.0);
        double joint4 = to_radian(-90.0);
        double joint5 = to_radian(-135.0);
        std::ostringstream prog;
        prog << "def reset_position():\n";
        prog << "  movej([" << joint0 << ", " << joint1 << ", " << joint2
             << ", " << joint3 << ", " << joint4 << ", " << joint5 << "], "
             << "a=" << robot_acc << ", v=" << robot_vel << ")\n";
        prog << "end\n";
        auto message = std_msgs::msg::String();
        message.data = prog.str();

        const auto start_time = now();
        const auto timeout = rclcpp::Duration::from_seconds(5.0);
        while (publisher_->get_subscription_count() == 0) {
            if ((now() - start_time) > timeout) {
                RCLCPP_ERROR(get_logger(),
                             "No subscriber found. Aborting Reset.");
                feedback->debug_msgs = "No URScript subscriber available\n";
                result->status = "Reset to default position failed";
                goal_handle->abort(result);
                return;
            }
        }
        publisher_->publish(message);
    }

    if (goal_handle->is_canceling()) {
        result->status = "Cancel requested!";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Cancel requested!");
        planning_component_->setStartStateToCurrentState();
        return;
    }
    result->status = "Reset action completed successfully";
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Reset completed.");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResetActionServer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
