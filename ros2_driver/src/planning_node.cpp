#include "dds_publisher.hpp"
#include "dds_subscriber.hpp"
#include "img_subscriber.hpp"
#include "octa_ros/action/focus.hpp"
#include "urscript_publisher.hpp"
#include "utils.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

using namespace std::chrono_literals;
std::atomic<bool> running(true);

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"),
                "Signal %d received, shutting down...", signum);
    running = false;
    rclcpp::shutdown();
}

bool tol_measure(double &roll, double &pitch, double &angle_tolerance) {
    return ((std::abs(std::abs(roll)) < to_radian(angle_tolerance)) &&
            (std::abs(std::abs(pitch)) < to_radian(angle_tolerance)));
}

using Focus = octa_ros::action::Focus;

class FocusActionServer : public rclcpp::Node {
  public:
    using GoalHandleFocus = rclcpp_action::ServerGoalHandle<Focus>;

    explicit FocusActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    // ========== ACTION SERVER ==========
    rclcpp_action::Server<Focus>::SharedPtr action_server_;

    // Handlers for the action server
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const Focus::Goal> goal);
    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleFocus> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFocus> goal_handle);
    void execute(const std::shared_ptr<GoalHandleFocus> goal_handle);
    bool check_if_done();

    // ========== HELPER NODES & EXECUTOR ==========
    // We keep separate node(s) for MoveGroup, etc.
    rclcpp::NodeOptions node_options_;
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spinner_;

    // Subscribers/Publishers
    std::shared_ptr<dds_subscriber> subscriber_node_;
    std::shared_ptr<dds_publisher> publisher_node_;
    std::shared_ptr<img_subscriber> img_subscriber_node_;
    std::shared_ptr<urscript_publisher> urscript_node_;

    // MoveIt
    std::shared_ptr<rclcpp::Node> move_group_node_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;

    // Internal parameters
    const int interval_ = 4;
    const bool single_interval_ = false;

    std::string msg_;
    double angle_ = 0.0;
    int circle_state_ = 1;
    bool apply_config_ = true;
    bool end_state_ = false;
    bool scan_3d_ = false;

    bool planning_ = false;
    double angle_increment_ = 0.0;
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    Eigen::Matrix3d rotmat_eigen_;
    cv::Mat img_;
    std::vector<cv::Mat> img_array_;
    std::vector<Eigen::Vector3d> pc_lines_;
    tf2::Quaternion q_;
    tf2::Quaternion target_q_;
    geometry_msgs::msg::Pose target_pose_;

    // Subscriber parameters
    double robot_vel_ = 0.0, robot_acc_ = 0.0, radius_ = 0.0,
           angle_limit_ = 0.0, dz_ = 0.0;
    double z_tolerance_ = 0.0, angle_tolerance_ = 0.0, z_height_ = 0.0;
    int num_pt_ = 0;
    bool fast_axis_ = true;
    bool success_ = false;
    bool next_ = false;
    bool previous_ = false;
    bool home_ = false;
    bool z_focused_ = false;
    bool angle_focused_ = false;

    // Path constraints
    moveit_msgs::msg::Constraints path_constr_;
    moveit_msgs::msg::OrientationConstraint ocm_;
    double x_tol_ = 0.5;
    double y_tol_ = 0.5;
    double z_tol_ = 0.5;
    double path_enforce_ = 1.0;
    int attempt_ = 0;
    int max_attempt_ = 20;
};

// ========== CONSTRUCTOR ==========
FocusActionServer::FocusActionServer(const rclcpp::NodeOptions &options)
    : Node("focus_action_server", options), node_options_(options),
      move_group_node_(
          std::make_shared<rclcpp::Node>("node_moveit", node_options_)),
      // Initialize the MoveGroupInterface with your planning group
      move_group_interface_(move_group_node_, "ur_manipulator") {
    // Declare that we automatically pull ROS params
    node_options_.automatically_declare_parameters_from_overrides(true);

    // Initialize sub/pubs
    subscriber_node_ = std::make_shared<dds_subscriber>();
    publisher_node_ =
        std::make_shared<dds_publisher>(msg_, angle_, circle_state_, fast_axis_,
                                        apply_config_, end_state_, scan_3d_);
    img_subscriber_node_ = std::make_shared<img_subscriber>();
    urscript_node_ = std::make_shared<urscript_publisher>();

    // Add nodes to the executor
    executor_.add_node(move_group_node_);
    executor_.add_node(subscriber_node_);
    executor_.add_node(publisher_node_);
    executor_.add_node(img_subscriber_node_);
    executor_.add_node(urscript_node_);

    // Spin in a separate thread
    spinner_ = std::thread([this]() { executor_.spin(); });

    // Setup path constraints
    ocm_.link_name = "tcp";
    ocm_.header.frame_id = "base_link";
    ocm_.absolute_x_axis_tolerance = x_tol_;
    ocm_.absolute_y_axis_tolerance = y_tol_;
    ocm_.absolute_z_axis_tolerance = z_tol_;
    ocm_.weight = path_enforce_;

    path_constr_.orientation_constraints.push_back(ocm_);
    move_group_interface_.setPathConstraints(path_constr_);

    // Add collision objects
    add_collision_obj(move_group_interface_);

    // Create the Action Server last (so everything else is ready)
    action_server_ = rclcpp_action::create_server<Focus>(
        this, "focus_action",
        std::bind(&FocusActionServer::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&FocusActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&FocusActionServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "FocusActionServer is up and running...");
}

// ========== ACTION SERVER HANDLERS ==========

rclcpp_action::GoalResponse
FocusActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                               std::shared_ptr<const Focus::Goal> goal) {
    RCLCPP_INFO(get_logger(),
                "Received goal request: angle_tolerance=%.3f, "
                "z_tolerance=%.3f, scan_3d=%s",
                goal->angle_tolerance, goal->z_tolerance,
                goal->scan_3d ? "true" : "false");

    // Decide if you accept or reject this goal
    if (goal->angle_tolerance < 0.0 || goal->z_tolerance < 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid negative tolerance, rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FocusActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFocus> goal_handle) {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    // Accept cancellation
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FocusActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleFocus> goal_handle) {
    // Start a new thread to perform the work so that `handle_accepted` can
    // return immediately
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void FocusActionServer::execute(
    const std::shared_ptr<GoalHandleFocus> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing goal...");
    rclcpp::Rate loop_rate(2); // 2 Hz
    auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<Focus::Feedback>();
    auto result = std::make_shared<Focus::Result>();
    bool preempted = false;

    // Example main loop to emulate your existing while (rclcpp::ok()) usage
    while (rclcpp::ok()) {
        // Check if goal is canceled
        if (goal_handle->is_canceling()) {
            result->result = "Goal was canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal canceled");
            preempted = true;
            break;
        }

        // ======================
        // Example snippet from your original code:
        // (Adapt these calls to your actual usage)
        // ======================
        end_state_ = false;
        apply_config_ = false;
        fast_axis_ = true;
        publisher_node_->set_fast_axis(fast_axis_);
        publisher_node_->set_apply_config(apply_config_);
        publisher_node_->set_end_state(end_state_);

        // Reading from subscriber
        next_ = subscriber_node_->next();
        previous_ = subscriber_node_->previous();
        home_ = subscriber_node_->home();
        z_tolerance_ = subscriber_node_->z_tolerance();
        angle_tolerance_ = subscriber_node_->angle_tolerance();
        radius_ = subscriber_node_->radius();
        angle_limit_ = subscriber_node_->angle_limit();
        num_pt_ = subscriber_node_->num_pt();
        robot_vel_ = subscriber_node_->robot_vel();
        robot_acc_ = subscriber_node_->robot_acc();

        // Freedrive example
        if (subscriber_node_->freedrive()) {
            circle_state_ = 1;
            angle_ = 0.0;
            publisher_node_->set_angle(angle_);
            publisher_node_->set_circle_state(circle_state_);
            urscript_node_->activate_freedrive();
            while (subscriber_node_->freedrive()) {
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                // Also check if cancel requested inside the loop
                if (goal_handle->is_canceling()) {
                    result->result = "Goal was canceled (in freedrive)";
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Goal canceled");
                    preempted = true;
                    break;
                }
            }
            urscript_node_->deactivate_freedrive();
            if (preempted)
                break; // break if canceled
        }

        // Example MoveIt usage
        move_group_interface_.setMaxVelocityScalingFactor(robot_vel_);
        move_group_interface_.setMaxAccelerationScalingFactor(robot_acc_);
        move_group_interface_.setStartStateToCurrentState();

        // ...
        // (Insert your big logic here, including scanning, planning, etc.)
        // ...
        // Publish some feedback for demonstration
        feedback->current_angle = angle_; // or a real measured angle
        feedback->current_height = 0.0;   // or a real measured height
        feedback->debug_msg = "Focusing...";
        goal_handle->publish_feedback(feedback);

        // Example done condition
        bool done = check_if_done();
        if (done) {
            result->result = "Focusing completed successfully";
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal succeeded");
            break;
        }

        loop_rate.sleep();
    }

    // If the while loop exited for reasons other than success
    if (!preempted && !rclcpp::ok()) {
        // Possibly node is shutting down
        result->result = "Aborted: Node is shutting down";
        goal_handle->abort(result);
    }
}

bool FocusActionServer::check_if_done() {
    // Replace with your logic
    return false;
}

// ========== MAIN ==========
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FocusActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
