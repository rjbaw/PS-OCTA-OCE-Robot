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
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <octa_ros/action/reset.hpp>

#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "utils.hpp"

using namespace std::chrono_literals;

class ResetActionServer : public rclcpp::Node {
  public:
    using ResetAction = octa_ros::action::Reset;
    using GoalHandleResetAction = rclcpp_action::ServerGoalHandle<ResetAction>;

    ResetActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("reset_action_server",
               rclcpp::NodeOptions(options)
                   .automatically_declare_parameters_from_overrides(true)) {}
    void init() {
        action_server_ = rclcpp_action::create_server<ResetAction>(
            this, "reset_action",
            std::bind(&ResetActionServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&ResetActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&ResetActionServer::handle_accepted, this,
                      std::placeholders::_1));

        // auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto qos = rclcpp::SystemDefaultsQoS{};

        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/urscript_interface/script_command", qos);
        service_capture_background_ =
            create_client<std_srvs::srv::Trigger>("capture_background");
        moveit_cpp_ =
            std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());
        tem_ = moveit_cpp_->getTrajectoryExecutionManagerNonConst();
        planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(
            "ur_manipulator", moveit_cpp_);
    }

  private:
    rclcpp_action::Server<ResetAction>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleResetAction> active_goal_handle_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        service_capture_background_;
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
    std::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager>
        tem_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    bool failed_ = false;

    moveit_msgs::msg::Constraints makeEnvelope(const Eigen::Isometry3d &centre,
                                               double lin_radius_m,
                                               double ang_radius_rad) const {
        moveit_msgs::msg::Constraints c;

        const std::string planning_frame =
            moveit_cpp_->getPlanningSceneMonitor()
                ->getPlanningScene()
                ->getPlanningFrame();

        moveit_msgs::msg::PositionConstraint pc;
        pc.header.frame_id = planning_frame;
        pc.link_name = "tcp";
        pc.weight = 1.0;
        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type = sphere.SPHERE;
        sphere.dimensions = {lin_radius_m};
        pc.constraint_region.primitives.push_back(sphere);
        geometry_msgs::msg::Pose centre_pose;
        centre_pose.position.x = centre.translation().x();
        centre_pose.position.y = centre.translation().y();
        centre_pose.position.z = centre.translation().z();
        centre_pose.orientation.w = 1.0;
        pc.constraint_region.primitive_poses.push_back(centre_pose);

        moveit_msgs::msg::OrientationConstraint oc;
        oc.header.frame_id = planning_frame;
        oc.link_name = "tcp";
        oc.weight = 1.0;
        Eigen::Quaterniond q(centre.rotation());
        oc.orientation.x = q.x();
        oc.orientation.y = q.y();
        oc.orientation.z = q.z();
        oc.orientation.w = q.w();
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance =
            oc.absolute_z_axis_tolerance = ang_radius_rad;

        c.position_constraints.push_back(pc);
        c.orientation_constraints.push_back(oc);
        return c;
    }

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
        tem_->stopExecution(true);
        publish_stop();
        goal_handle->canceled(std::make_shared<ResetAction::Result>());
        active_goal_handle_.reset();
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

        failed_ = false; // force urscript

        RCLCPP_INFO(get_logger(), "Starting Reset execution...");
        feedback->status = "Resetting... Please wait";
        goal_handle->publish_feedback(feedback);

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

            moveit::core::RobotStatePtr cur_state =
                moveit_cpp_->getCurrentState();
            Eigen::Isometry3d start_tcp =
                cur_state->getGlobalLinkTransform("tcp");
            auto envelope = makeEnvelope(start_tcp,   // centre pose
                                         0.05,        // 5 cm translation radius
                                         M_PI / 3.0); // 60 deg rotation radius
            // planning_component_->setPathConstraints(envelope);
            // moveit_msgs::msg::Constraints env;
            // moveit_msgs::msg::JointConstraint jc;
            // jc.joint_name = "elbow_joint";
            // double q0 = cur_state->getVariablePosition("elbow_joint");
            // jc.position = q0;
            // jc.tolerance_above = jc.tolerance_below = to_radian(10.0);
            // jc.weight = 1.0;
            // env.joint_constraints.push_back(jc);
            // planning_component_->setPathConstraints(env);

            planning_component_->setGoal(goal_state);
            auto req = moveit_cpp::PlanningComponent::
                MultiPipelinePlanRequestParameters(
                    shared_from_this(), {"stomp_joint", "ompl_rrtc"});
            auto choose_shortest =
                [](const std::vector<planning_interface::MotionPlanResponse>
                       &sols) {
                    return *std::min_element(
                        sols.begin(), sols.end(),
                        [](const auto &a, const auto &b) {
                            if (a && b)
                                return robot_trajectory::pathLength(
                                           *a.trajectory) <
                                       robot_trajectory::pathLength(
                                           *b.trajectory);
                            return static_cast<bool>(a);
                        });
                };
            planning_interface::MotionPlanResponse plan_solution =
                planning_component_->plan(req, choose_shortest);
            planning_component_->setPathConstraints(
                moveit_msgs::msg::Constraints());
            if (plan_solution) {
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->status = "Cancel requested!";
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Cancel requested!");
                    planning_component_->setStartStateToCurrentState();
                    return;
                }

                auto execute_status =
                    moveit_cpp_->execute(plan_solution.trajectory);

                auto execute_success =
                    (execute_status ==
                     moveit_controller_manager::ExecutionStatus::SUCCEEDED);

                if (execute_success) {
                    RCLCPP_INFO(get_logger(), "Execute Success!");
                } else {
                    RCLCPP_INFO(get_logger(), "Execute Failed!");
                    tem_->stopExecution(true);
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
                    // goal_handle->abort(result);
                    goal_handle->canceled(result);
                    return;
                }
            }
            publisher_->publish(message);
            for (int i = 0; i < 40; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        call_capture_background();
        result->success = true;
        result->status = "Reset action completed successfully";
        if (goal_handle->is_active()) {
            if (failed_) {
                goal_handle->abort(result);
            } else {
                goal_handle->succeed(result);
            }
        }
        RCLCPP_INFO(get_logger(), "Reset completed.");
        active_goal_handle_.reset();
    }

    bool call_capture_background() {
        if (!service_capture_background_->wait_for_service(0s))
            return false;

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto fut = service_capture_background_->async_send_request(req);
        return fut.wait_for(2s) == std::future_status::ready &&
               fut.get()->success;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResetActionServer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
