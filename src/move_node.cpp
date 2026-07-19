/**
 * @file move_node.cpp
 * @author rjbaw
 * @brief Movement node
 */

#include "move_node.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <exception>

MoveActionServer::MoveActionServer(const rclcpp::NodeOptions &options)
    : Node("move_action_server",
           rclcpp::NodeOptions(options)
               .automatically_declare_parameters_from_overrides(true)) {}

MoveActionServer::~MoveActionServer() {
    action_server_.reset();
    if (cancel_worker_.joinable()) {
        cancel_worker_.request_stop();
        cancel_worker_.join();
    }
    if (worker_.joinable()) {
        worker_.request_stop();
        try {
            if (tem_ && active_goal_handle_ &&
                active_goal_handle_->is_active()) {
                tem_->stopExecution(true);
            }
        } catch (const std::exception &exception) {
            RCLCPP_ERROR(get_logger(),
                         "Could not stop Move during shutdown: %s",
                         exception.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(),
                         "Could not stop Move during shutdown: unknown "
                         "exception");
        }
        worker_.join();
    }
    active_goal_handle_.reset();
}

void MoveActionServer::init() {
    const auto get_bool_param = [this](const std::string &name,
                                       bool default_value) -> bool {
        return this->has_parameter(name)
                   ? this->get_parameter(name).as_bool()
                   : this->declare_parameter<bool>(name, default_value);
    };

    const bool plan_only = get_bool_param("plan_only", false);
    if (!plan_only) {
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
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
    if (!goal_handle->is_active()) {
        RCLCPP_INFO(get_logger(), "Move goal no longer active");
        return rclcpp_action::CancelResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Cancel request received for Move.");
    if (!cancel_worker_.joinable()) {
        cancel_worker_ = std::jthread([this, goal_handle](
                                          std::stop_token stop_token) {
            try {
                while (!stop_token.stop_requested() &&
                       goal_handle->is_active() &&
                       !goal_handle->is_canceling()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                if (!stop_token.stop_requested() &&
                    goal_handle->is_canceling() && tem_) {
                    tem_->stopExecution(true);
                }
            } catch (const std::exception &exception) {
                RCLCPP_ERROR(get_logger(), "Could not stop Move trajectory: %s",
                             exception.what());
            } catch (...) {
                RCLCPP_ERROR(get_logger(),
                             "Could not stop Move trajectory: unknown "
                             "exception");
            }
        });
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
    if (cancel_worker_.joinable()) {
        cancel_worker_.join();
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    active_goal_handle_ = goal_handle;
    worker_ = std::jthread([this, goal_handle](std::stop_token stop_token) {
        const auto terminate_goal = [this, goal_handle,
                                     stop_token](const char *status) {
            if (stop_token.stop_requested()) {
                return;
            }
            try {
                auto result = std::make_shared<Move::Result>();
                result->status = status;
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                } else if (goal_handle->is_active()) {
                    goal_handle->abort(result);
                }
            } catch (...) {
                RCLCPP_ERROR(get_logger(),
                             "Could not terminate failed Move goal");
            }
        };
        try {
            execute(goal_handle, stop_token);
        } catch (const std::exception &exception) {
            RCLCPP_ERROR(get_logger(), "Unhandled Move exception: %s",
                         exception.what());
            terminate_goal("Move failed due to an internal exception\n");
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "Unhandled non-standard Move exception");
            terminate_goal("Move failed due to an internal exception\n");
        }
    });
}

void MoveActionServer::execute(
    const std::shared_ptr<GoalHandleMove> goal_handle,
    std::stop_token stop_token) {
    if (stop_token.stop_requested() || !goal_handle->is_active()) {
        return;
    }
    RCLCPP_INFO(get_logger(), "Starting Move execution...");
    auto feedback = std::make_shared<Move::Feedback>();
    auto result = std::make_shared<Move::Result>();
    const auto handle_interruption = [&]() {
        if (stop_token.stop_requested()) {
            return true;
        }
        if (!goal_handle->is_canceling()) {
            return false;
        }
        feedback->debug_msgs = "Move canceled by request.\n";
        result->status = "Move Canceled\n";
        goal_handle->canceled(result);
        return true;
    };

    const auto goal = goal_handle->get_goal();
    const double offset_x_mm = goal->offset_x;
    const double offset_y_mm = goal->offset_y;
    const double target_angle_deg = goal->target_angle;
    const double radius_mm = goal->radius;
    const bool apply_offset = goal->apply_offset;
    RCLCPP_INFO(get_logger(), "Offset X: %.2f mm", offset_x_mm);
    RCLCPP_INFO(get_logger(), "Offset Y: %.2f mm", offset_y_mm);
    RCLCPP_INFO(get_logger(), "Target angle: %.2f deg", target_angle_deg);

    const bool plan_only = !moveit_cpp_;
    if (plan_only) {
        if (handle_interruption()) {
            return;
        }
        feedback->debug_msgs =
            "Plan-only mode: skipping planning and execution.\n";
        goal_handle->publish_feedback(feedback);
        if (handle_interruption()) {
            return;
        }
        result->status = "Move completed (plan-only/offline)\n";
        goal_handle->succeed(result);
        return;
    }

    if (handle_interruption()) {
        return;
    }

    planning_component_->setStartStateToCurrentState();
    const std::string tool_link =
        this->has_parameter("tool_link")
            ? this->get_parameter("tool_link").as_string()
            : this->declare_parameter<std::string>("tool_link", "tcp");
    moveit::core::RobotStatePtr current_state = moveit_cpp_->getCurrentState();
    if (handle_interruption()) {
        return;
    }
    Eigen::Isometry3d current_pose =
        current_state->getGlobalLinkTransform(tool_link);
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = moveit_cpp_->getPlanningSceneMonitor()
                                      ->getPlanningScene()
                                      ->getPlanningFrame();
    target_pose.pose = tf2::toMsg(current_pose);

    if (apply_offset) {
        // T_world_target = T_world_tcp * T_tcp_target
        const Eigen::Vector3d offset_tcp(offset_x_mm * 0.001,
                                         offset_y_mm * 0.001, 0.0);
        const Eigen::Vector3d offset_world = current_pose.linear() * offset_tcp;
        target_pose.pose.position.x += offset_world.x();
        target_pose.pose.position.y += offset_world.y();
        target_pose.pose.position.z += offset_world.z();
    } else {
        const double offset_radius_mm = 4.3 - radius_mm;
        const double offset_radius_m = offset_radius_mm * 0.001;
        const double target_angle_rad = to_radian(target_angle_deg);
        Eigen::Isometry3d T_tcp_oce = Eigen::Isometry3d::Identity();
        T_tcp_oce.translation() = Eigen::Vector3d(0.0, -offset_radius_m, 0.0);
        Eigen::Isometry3d T_oce_rotate = Eigen::Isometry3d::Identity();
        T_oce_rotate.linear() =
            Eigen::AngleAxisd(target_angle_rad, Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
        Eigen::Isometry3d T_target =
            current_pose * T_tcp_oce * T_oce_rotate * T_tcp_oce.inverse();

        target_pose.pose.position.x = T_target.translation().x();
        target_pose.pose.position.y = T_target.translation().y();
        target_pose.pose.position.z = T_target.translation().z();

        Eigen::Quaterniond q_target(T_target.linear());
        q_target.normalize();
        target_pose.pose.orientation.x = q_target.x();
        target_pose.pose.orientation.y = q_target.y();
        target_pose.pose.orientation.z = q_target.z();
        target_pose.pose.orientation.w = q_target.w();
    }

    print_target(get_logger(), target_pose.pose);

    const Eigen::Isometry3d start_tcp =
        current_state->getGlobalLinkTransform(tool_link);
    const std::string planning_frame = moveit_cpp_->getPlanningSceneMonitor()
                                           ->getPlanningScene()
                                           ->getPlanningFrame();
    const double target_rad = to_radian(target_angle_deg);
    const bool want_long = std::fabs(target_rad) > M_PI + 1e-6;
    const double yaw_limit = want_long ? (2.0 * M_PI + 1e-3) : M_PI;
    const double envelope_radius =
        this->has_parameter("envelope_radius_m")
            ? this->get_parameter("envelope_radius_m").as_double()
            : this->declare_parameter<double>("envelope_radius_m", 0.05);
    moveit_msgs::msg::Constraints envelope = octa_ros::motion::make_envelope(
        start_tcp, planning_frame, tool_link, envelope_radius, yaw_limit);
    planning_component_->setPathConstraints(envelope);

    auto wrist_spin_sign =
        [tool_link](const moveit::core::RobotState &rs,
                    const std::string &wrist_joint) -> double {
        moveit::core::RobotState s0(rs);
        moveit::core::RobotState s1(rs);
        const double dq = 1.0 * M_PI / 180.0;
        const double q = s0.getVariablePosition(wrist_joint);
        s1.setVariablePosition(wrist_joint, q + dq);

        const Eigen::Matrix3d R0 =
            s0.getGlobalLinkTransform(tool_link).linear();
        const Eigen::Matrix3d R1 =
            s1.getGlobalLinkTransform(tool_link).linear();
        const Eigen::Matrix3d dR = R0.transpose() * R1;
        const Eigen::AngleAxisd aa(dR);
        return (aa.angle() * aa.axis().z() >= 0.0) ? +1.0 : -1.0;
    };

    bool used_joint_goal = false;
    {
        const std::string wrist_joint = "wrist_3_joint";
        moveit::core::RobotStatePtr seed_state = moveit_cpp_->getCurrentState();
        if (handle_interruption()) {
            return;
        }
        const auto *jmg = seed_state->getJointModelGroup("ur_manipulator");

        const double sign = wrist_spin_sign(*seed_state, wrist_joint);
        const double q0 = seed_state->getVariablePosition(wrist_joint);
        const double qref = q0 + sign * target_rad;

        moveit::core::RobotState goal_state(*seed_state);
        goal_state.setVariablePosition(wrist_joint, qref); // bias IK seed
        Eigen::Isometry3d T_goal;
        tf2::fromMsg(target_pose.pose, T_goal);

        if (goal_state.setFromIK(jmg, T_goal, 0.05)) {
            double qg = goal_state.getVariablePosition(wrist_joint);
            qg += std::round((qref - qg) / (2.0 * M_PI)) * 2.0 * M_PI;
            goal_state.setVariablePosition(wrist_joint, qg);

            planning_component_->setGoal(
                {kinematic_constraints::constructGoalConstraints(goal_state,
                                                                 jmg)});
            used_joint_goal = true;
            RCLCPP_INFO(get_logger(), "Using joint goal");
        } else {
            RCLCPP_WARN(
                get_logger(),
                "IK bias couldn't enforce wrist direction; using pose goal.");
            planning_component_->setGoal(target_pose, tool_link);
        }
    }

    const std::vector<std::string> pipelines =
        used_joint_goal ? std::vector<std::string>{"pilz_ptp"}
                        : std::vector<std::string>{"pilz_ptp", "pilz_lin"};
    auto req =
        moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters(
            shared_from_this(), pipelines);

    const bool ccw = (target_rad > 0.0);
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

            double yaw = 0.0;
            const std::size_t N = traj.getWayPointCount();
            for (std::size_t i = 0; i + 1 < N; ++i) {
                const Eigen::Matrix3d wp0 =
                    traj.getWayPoint(i)
                        .getGlobalLinkTransform(tool_link)
                        .linear();
                const Eigen::Matrix3d wp1 =
                    traj.getWayPoint(i + 1)
                        .getGlobalLinkTransform(tool_link)
                        .linear();
                const Eigen::Matrix3d dR = wp0.transpose() * wp1;
                const Eigen::AngleAxisd angle_axis(dR);
                yaw += angle_axis.angle() * angle_axis.axis().z();
            }

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
        planning_interface::MotionPlanResponse fail;
        fail.error_code.val =
            moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
        return fail;
    };
    planning_interface::MotionPlanResponse plan_solution;
    if (handle_interruption()) {
        return;
    }
    if (std::fabs(target_rad) > 1e-6) {
        plan_solution = planning_component_->plan(req, enforce_direction);
    } else {
        plan_solution = planning_component_->plan(req);
    }
    if (handle_interruption()) {
        return;
    }
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
    if (handle_interruption()) {
        return;
    }
    goal_handle->publish_feedback(feedback);

    if (handle_interruption()) {
        return;
    }

    const auto exec_status = moveit_cpp_->execute(plan_solution.trajectory);
    if (handle_interruption()) {
        return;
    }
    const bool execute_success = static_cast<bool>(exec_status);
    if (!execute_success) {
        RCLCPP_ERROR(get_logger(), "Execution failed!");
        feedback->debug_msgs = "Execution failed!\n";
        result->status = "Move Z angle failed\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->abort(result);
        return;
    }

    feedback->debug_msgs = "Move completed successfully!\n";
    result->status = "Move completed\n";
    if (handle_interruption()) {
        return;
    }
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
