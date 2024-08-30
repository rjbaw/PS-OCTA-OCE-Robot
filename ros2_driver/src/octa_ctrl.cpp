#include "octa_ros/msg/labviewdata.hpp"
#include "octa_ros/msg/robotdata.hpp"
#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include <moveit_msgs/msg/attached_collision_object.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

using namespace std::chrono_literals;

#include <csignal>  // Add this include for signal handling
std::atomic<bool> running(true);  // Add this global variable to control the loop
void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "Signal %d received, shutting down...", signum);
    running = false;
    rclcpp::shutdown();
}

class dds_publisher : public rclcpp::Node {
  public:
    dds_publisher() : dds_publisher("", 0.0, 0, false, false, false) {}

    dds_publisher(std::string msg = "", double angle = 0.0,
                  int circle_state = 0, bool fast_axis = false,
                  bool apply_config = false, bool end_state = false)
        : Node("pub_robot_data"), msg(msg), angle(angle),
          circle_state(circle_state), fast_axis(fast_axis),
          apply_config(apply_config), end_state(end_state) {
        publisher_ =
            this->create_publisher<octa_ros::msg::Robotdata>("robot_data", 10);

        timer_ = this->create_wall_timer(10ms, [this]() {
            // create string obj.
            auto message = octa_ros::msg::Robotdata();
            message.msg = this->msg;
            message.angle = this->angle;
            message.circle_state = this->circle_state;
            message.fast_axis = this->fast_axis;
            message.apply_config = this->apply_config;
            message.end_state = this->end_state;
            // RCLCPP_INFO(this->get_logger(),
            //             std::format("Publishing: \n"
            //                         " msg: {} \n"
            //                         " angle: {}\n"
            //                         " circle_state: {}\n"
            //                         " apply_config: {}\n",
            //                         " end_state: {}\n", this->msg,
            //                         this->angle, this->circle_state,
            //                         this->apply_config, this->end_state)
            //                 .c_str());
            publisher_->publish(message);
        });
    };

    void set_msg(const std::string &new_msg) { msg = new_msg; }
    void set_angle(double new_angle) { angle = new_angle; }
    void set_circle_state(int new_circle_state) {
        circle_state = new_circle_state;
    }
    void set_fast_axis(bool new_fast_axis) { fast_axis = new_fast_axis; }
    void set_apply_config(bool new_apply_config) {
        apply_config = new_apply_config;
    }
    void set_end_state(bool new_end_state) { end_state = new_end_state; }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr publisher_;
    std::string msg;
    double angle;
    int circle_state;
    bool fast_axis, apply_config, end_state;
};

class dds_subscriber : public rclcpp::Node {
  public:
    dds_subscriber()
        : Node("sub_labview"), best_effort(rclcpp::KeepLast(10))

    {
        subscription_ = this->create_subscription<octa_ros::msg::Labviewdata>(
            "labview_data", best_effort.reliability(be),
            [this](const octa_ros::msg::Labviewdata::SharedPtr msg) {
                robot_vel_ = msg->robot_vel;
                robot_acc_ = msg->robot_acc;
                z_tolerance_ = msg->z_tolerance;
                angle_tolerance_ = msg->angle_tolerance;
                radius_ = msg->radius;
                angle_limit_ = msg->angle_limit;
                num_pt_ = msg->num_pt;
                dz_ = msg->dz;
                drot_ = msg->drot;
                autofocus_ = msg->autofocus;
                freedrive_ = msg->freedrive;
                previous_ = msg->previous;
                next_ = msg->next;
                home_ = msg->home;
                reset_ = msg->reset;
                fast_axis_ = msg->fast_axis;
                // RCLCPP_INFO(this->get_logger(),
                //             std::format("Subscribing: \n"
                //                         " robot_vel: {}\n"
                //                         " robot_acc: {}\n"
                //                         " z_tolerance: {}\n"
                //                         " angle_tolerance: {}\n"
                //                         " radius: {}\n"
                //                         " angle_limit: {}\n"
                //                         " num_pt: {}\n"
                //                         " dz: {}\n"
                //                         " drot: {}\n"
                //                         " autofocus: {}\n"
                //                         " freedrive: {}\n"
                //                         " previous: {}\n"
                //                         " next: {}\n"
                //                         " home: {}\n"
                //                         " reset: {}\n"
                //                         " fast_axis: {}\n",
                //                         robot_vel_, robot_acc_, z_tolerance_,
                //                         angle_tolerance_, radius_,
                //                         angle_limit_, num_pt_, dz_, drot_,
                //                         autofocus_, freedrive_, previous_,
                //                         next_, home_, reset_, fast_axis_)
                //                 .c_str());
            });
    };
    double robot_vel() { return robot_vel_; };
    double robot_acc() { return robot_acc_; };
    double z_tolerance() { return z_tolerance_; };
    double angle_tolerance() { return angle_tolerance_; };
    double radius() { return radius_; };
    double angle_limit() { return angle_limit_; };
    int num_pt() { return num_pt_; };
    double dz() { return dz_; };
    double drot() { return drot_; };
    bool autofocus() { return autofocus_; };
    bool freedrive() { return freedrive_; };
    bool previous() { return previous_; };
    bool next() { return next_; };
    bool home() { return home_; };
    bool reset() { return reset_; };
    bool fast_axis() { return fast_axis_; };

  private:
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
    double robot_vel_ = 0, robot_acc_ = 0, z_tolerance_ = 0,
           angle_tolerance_ = 0, radius_ = 0, angle_limit_ = 0, dz_ = 0,
           drot_ = 0;
    int num_pt_ = 0;
    bool autofocus_ = false, freedrive_ = false, previous_ = false,
         next_ = false, home_ = false, reset_ = false, fast_axis_ = false;
};

double to_radian(double degree) { return (std::numbers::pi / 180 * degree); }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto const node =
        std::make_shared<rclcpp::Node>("node_moveit", node_options);

    auto const logger = rclcpp::get_logger("logger_planning");

    rclcpp::executors::SingleThreadedExecutor exec;
    auto subscriber_node = std::make_shared<dds_subscriber>();
    exec.add_node(subscriber_node);
    std::thread([&exec]() { exec.spin(); }).detach();

    double robot_vel = subscriber_node->robot_vel();
    double robot_acc = subscriber_node->robot_acc();
    double z_tolerance = subscriber_node->z_tolerance();
    double angle_tolerance = subscriber_node->angle_tolerance();
    double radius = subscriber_node->radius();
    double angle_limit = subscriber_node->angle_limit();
    int num_pt = subscriber_node->num_pt();
    double dz = subscriber_node->dz();
    double drot = subscriber_node->drot();
    bool autofocus = subscriber_node->autofocus();
    bool freedrive = subscriber_node->freedrive();
    bool previous = subscriber_node->previous();
    bool next = subscriber_node->next();
    bool home = subscriber_node->home();
    bool reset = subscriber_node->reset();
    bool fast_axis = subscriber_node->fast_axis();

    std::string msg;
    double angle = 0.0;
    int circle_state = 1;
    bool apply_config = false;
    bool end_state = false;

    rclcpp::executors::SingleThreadedExecutor exec_pub;
    auto publisher_node = std::make_shared<dds_publisher>(
        msg = msg, angle = angle, circle_state = circle_state,
        fast_axis = fast_axis, apply_config = apply_config,
        end_state = end_state);
    exec_pub.add_node(publisher_node);
    std::thread([&exec_pub]() { exec_pub.spin(); }).detach();

    int count = 1;

    while (rclcpp::ok() && running) {
        autofocus = subscriber_node->autofocus();
        freedrive = subscriber_node->freedrive();
        // if (freedrive || !autofocus) {
        //     continue;
        // }
        apply_config = false;
        end_state = false;

        robot_vel = subscriber_node->robot_vel();
        robot_acc = subscriber_node->robot_acc();
        z_tolerance = subscriber_node->z_tolerance();
        angle_tolerance = subscriber_node->angle_tolerance();
        radius = subscriber_node->radius();
        angle_limit = subscriber_node->angle_limit();
        num_pt = subscriber_node->num_pt();
        dz = subscriber_node->dz();
        drot = subscriber_node->drot();
        previous = subscriber_node->previous();
        next = subscriber_node->next();
        home = subscriber_node->home();
        reset = subscriber_node->reset();
        fast_axis = subscriber_node->fast_axis();

        if (reset) {
            msg = "", angle = 0.0, circle_state = 1;
        }

        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
        move_group_interface.setMaxVelocityScalingFactor(robot_vel);
        move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
        move_group_interface.setStartStateToCurrentState();

        double angle_increment = angle_limit / num_pt;
        double roll = 0, pitch = 0, yaw = 0;
        tf2::Quaternion q;
        geometry_msgs::msg::Pose target_pose;

        if (next) {
            angle += angle_increment;
            yaw = to_radian(angle_increment);
        }
        if (previous) {
            angle -= angle_increment;
            yaw = to_radian(-angle_increment);
        }
        if (home) {
            yaw = to_radian(-angle);
        }
        if (fast_axis) {
            roll = -drot;
        } else {
            yaw = -drot;
        }
        q.setRPY(roll, pitch, yaw);
        q.normalize();
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();

        target_pose.position.x = radius * std::cos(to_radian(angle));
        target_pose.position.y = radius * std::sin(to_radian(angle));
        target_pose.position.z = dz;

        // target_pose.orientation.w = 1.0;
        // target_pose.position.x = 0.28;
        // target_pose.position.y = -0.2 + count * -0.1;
        // target_pose.position.z = 0.5;

        move_group_interface.setPoseTarget(target_pose);
        auto const [success, plan] = [&move_group_interface] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        if (success) {
            move_group_interface.execute(plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }

        if ((std::abs(drot) < angle_tolerance) &&
            (std::abs(dz) && z_tolerance)) {
            if (!fast_axis) {
                fast_axis = true;
                apply_config = true;
            } else {
                fast_axis = false;
                apply_config = true;
                end_state = true;
            }
        }

        publisher_node->set_msg(msg);
        publisher_node->set_angle(angle);
        publisher_node->set_circle_state(circle_state);
        publisher_node->set_fast_axis(fast_axis);
        publisher_node->set_apply_config(apply_config);
        publisher_node->set_end_state(end_state);

        // rclcpp::spin_some(std::make_shared<dds_publisher>(
        //     msg = msg, angle = angle, circle_state = circle_state,
        //     fast_axis = fast_axis, apply_config = apply_config,
        //     end_state = end_state));
    }

    rclcpp::shutdown();
    return 0;
}

// // All source files that use ROS logging should define a file-specific
// // static const rclcpp::Logger named LOGGER, located at the top of the
// file
// // and inside the namespace with the narrowest scope (if there is one)
// static const rclcpp::Logger LOGGER =
// rclcpp::get_logger("move_group_demo");

//     // BEGIN_TUTORIAL
//     //
//     // Setup
//     // ^^^^^
//     //
//     // MoveIt operates on sets of joints called "planning groups" and
//     stores
//     // them in an object called the ``JointModelGroup``. Throughout
//     MoveIt, the
//     // terms "planning group" and "joint model group" are used
//     interchangeably. static const std::string PLANNING_GROUP =
//     "panda_arm";

//     // The
//     //
//     :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
//     // class can be easily set up using just the name of the planning
//     group you
//     // would like to control and plan for.
//     moveit::planning_interface::MoveGroupInterface
//     move_group(move_group_node,
//                                                               PLANNING_GROUP);

//     // We will use the
//     //
//     :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
//     // class to add and remove collision objects in our "virtual world"
//     scene moveit::planning_interface::PlanningSceneInterface
//     planning_scene_interface;

//     // Raw pointers are frequently used to refer to the planning group
//     for
//     // improved performance.
//     const moveit::core::JointModelGroup *joint_model_group =
//         move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//     // Visualization
//     // ^^^^^^^^^^^^^
//     namespace rvt = rviz_visual_tools;
//     moveit_visual_tools::MoveItVisualTools visual_tools(
//         move_group_node, "panda_link0", "move_group_tutorial",
//         move_group.getRobotModel());

//     visual_tools.deleteAllMarkers();

//     /* Remote control is an introspection tool that allows users to step
//     through
//      * a high level script */
//     /* via buttons and keyboard shortcuts in RViz */
//     visual_tools.loadRemoteControl();

//     // RViz provides many types of markers, in this demo we will use
//     text,
//     // cylinders, and spheres
//     Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//     text_pose.translation().z() = 1.0;
//     visual_tools.publishText(text_pose, "MoveGroupInterface_Demo",
//     rvt::WHITE,
//                              rvt::XLARGE);

//     // Batch publishing is used to reduce the number of messages being
//     sent to
//     // RViz for large visualizations
//     visual_tools.trigger();

//     // Getting Basic Information
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^
//     //
//     // We can print the name of the reference frame for this robot.
//     RCLCPP_INFO(LOGGER, "Planning frame: %s",
//                 move_group.getPlanningFrame().c_str());

//     // We can also print the name of the end-effector link for this
//     group. RCLCPP_INFO(LOGGER, "End effector link: %s",
//                 move_group.getEndEffectorLink().c_str());

//     // We can get a list of all the groups in the robot:
//     RCLCPP_INFO(LOGGER, "Available Planning Groups:");
//     std::copy(move_group.getJointModelGroupNames().begin(),
//               move_group.getJointModelGroupNames().end(),
//               std::ostream_iterator<std::string>(std::cout, ", "));

//     // Start the demo
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^
//     visual_tools.prompt(
//         "Press 'next' in the RvizVisualToolsGui window to start the
//         demo");

//     // .. _move_group_interface-planning-to-pose-goal:
//     //
//     // Planning to a Pose goal
//     // ^^^^^^^^^^^^^^^^^^^^^^^
//     // We can plan a motion for this group to a desired pose for the
//     // end-effector.
//     geometry_msgs::msg::Pose target_pose1;
//     target_pose1.orientation.w = 1.0;
//     target_pose1.position.x = 0.28;
//     target_pose1.position.y = -0.2;
//     target_pose1.position.z = 0.5;
//     move_group.setPoseTarget(target_pose1);

//     // Now, we call the planner to compute the plan and visualize it.
//     // Note that we are just planning, not asking move_group
//     // to actually move the robot.
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//     bool success =
//         (move_group.plan(my_plan) ==
//         moveit::core::MoveItErrorCode::SUCCESS);

//     RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s",
//                 success ? "" : "FAILED");

//     // Visualizing plans
//     // ^^^^^^^^^^^^^^^^^
//     // We can also visualize the plan as a line with markers in RViz.
//     RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
//     visual_tools.publishAxisLabeled(target_pose1, "pose1");
//     visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE,
//     rvt::XLARGE); visual_tools.publishTrajectoryLine(my_plan.trajectory,
//     joint_model_group); visual_tools.trigger(); visual_tools.prompt(
//         "Press 'next' in the RvizVisualToolsGui window to continue the
//         demo");

//     // Moving to a pose goal
//     // ^^^^^^^^^^^^^^^^^^^^^
//     //
//     // Moving to a pose goal is similar to the step above
//     // except we now use the ``move()`` function. Note that
//     // the pose goal we had set earlier is still active
//     // and so the robot will try to move to that goal. We will
//     // not use that function in this tutorial since it is
//     // a blocking function and requires a controller to be active
//     // and report success on execution of a trajectory.

//     /* Uncomment below line when working with a real robot */
//     /* move_group.move(); */

//     // Planning to a joint-space goal
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//     //
//     // Let's set a joint space goal and move towards it.  This will
//     replace the
//     // pose target we set above.
//     //
//     // To start, we'll create an pointer that references the current
//     robot's
//     // state. RobotState is the object that contains all the current
//     // position/velocity/acceleration data.
//     moveit::core::RobotStatePtr current_state =
//     move_group.getCurrentState(10);
//     //
//     // Next get the current set of joint values for the group.
//     std::vector<double> joint_group_positions;
//     current_state->copyJointGroupPositions(joint_model_group,
//                                            joint_group_positions);

//     // Now, let's modify one of the joints, plan to the new joint space
//     goal,
//     // and visualize the plan.
//     joint_group_positions[0] = -1.0; // radians
//     bool within_bounds =
//     move_group.setJointValueTarget(joint_group_positions); if
//     (!within_bounds) {
//         RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of
//         limits,
//         "
//                             "but we will plan and clamp to the limits ");
//     }

//     // We lower the allowed maximum velocity and acceleration to 5% of
//     their
//     // maximum. The default values are 10% (0.1). Set your preferred
//     defaults in
//     // the joint_limits.yaml file of your robot's moveit_config or set
//     explicit
//     // factors in your code if you need your robot to move faster.
//     move_group.setMaxVelocityScalingFactor(0.05);
//     move_group.setMaxAccelerationScalingFactor(0.05);

//     success =
//         (move_group.plan(my_plan) ==
//         moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s",
//                 success ? "" : "FAILED");

//     // Visualize the plan in RViz:
//     visual_tools.deleteAllMarkers();
//     visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE,
//                              rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory,
//     joint_model_group); visual_tools.trigger(); visual_tools.prompt(
//         "Press 'next' in the RvizVisualToolsGui window to continue the
//         demo");

//     // Planning with Path Constraints
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//     //
//     // Path constraints can easily be specified for a link on the robot.
//     // Let's specify a path constraint and a pose goal for our group.
//     // First define the path constraint.
//     moveit_msgs::msg::OrientationConstraint ocm;
//     ocm.link_name = "panda_link7";
//     ocm.header.frame_id = "panda_link0";
//     ocm.orientation.w = 1.0;
//     ocm.absolute_x_axis_tolerance = 0.1;
//     ocm.absolute_y_axis_tolerance = 0.1;
//     ocm.absolute_z_axis_tolerance = 0.1;
//     ocm.weight = 1.0;

//     // Now, set it as the path constraint for the group.
//     moveit_msgs::msg::Constraints test_constraints;
//     test_constraints.orientation_constraints.push_back(ocm);
//     move_group.setPathConstraints(test_constraints);

//     // Enforce Planning in Joint Space
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//     //
//     // Depending on the planning problem MoveIt chooses between
//     // ``joint space`` and ``cartesian space`` for problem
//     representation.
//     // Setting the group parameter
//     ``enforce_joint_model_state_space:true`` in
//     // the ompl_planning.yaml file enforces the use of ``joint space``
//     for all
//     // plans.
//     //
//     // By default, planning requests with orientation path constraints
//     // are sampled in ``cartesian space`` so that invoking IK serves as a
//     // generative sampler.
//     //
//     // By enforcing ``joint space``, the planning process will use
//     rejection
//     // sampling to find valid requests. Please note that this might
//     // increase planning time considerably.
//     //
//     // We will reuse the old goal that we had and plan to it.
//     // Note that this will only work if the current state already
//     // satisfies the path constraints. So we need to set the start
//     // state to a new pose.
//     moveit::core::RobotState start_state(*move_group.getCurrentState());
//     geometry_msgs::msg::Pose start_pose2;
//     start_pose2.orientation.w = 1.0;
//     start_pose2.position.x = 0.55;
//     start_pose2.position.y = -0.05;
//     start_pose2.position.z = 0.8;
//     start_state.setFromIK(joint_model_group, start_pose2);
//     move_group.setStartState(start_state);

//     // Now, we will plan to the earlier pose target from the new
//     // start state that we just created.
//     move_group.setPoseTarget(target_pose1);

//     // Planning with constraints can be slow because every sample must
//     call an
//     // inverse kinematics solver. Let's increase the planning time from
//     the
//     // default 5 seconds to be sure the planner has enough time to
//     succeed. move_group.setPlanningTime(10.0);

//     success =
//         (move_group.plan(my_plan) ==
//         moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s",
//                 success ? "" : "FAILED");

//     // Visualize the plan in RViz:
//     visual_tools.deleteAllMarkers();
//     visual_tools.publishAxisLabeled(start_pose2, "start");
//     visual_tools.publishAxisLabeled(target_pose1, "goal");
//     visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE,
//                              rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory,
//     joint_model_group); visual_tools.trigger(); visual_tools.prompt(
//         "Press 'next' in the RvizVisualToolsGui window to continue the
//         demo");

//     // When done with the path constraint, be sure to clear it.
//     move_group.clearPathConstraints();

//     // Cartesian Paths
//     // ^^^^^^^^^^^^^^^
//     // You can plan a Cartesian path directly by specifying a list of
//     waypoints
//     // for the end-effector to go through. Note that we are starting
//     // from the new start state above.  The initial pose (start state)
//     does not
//     // need to be added to the waypoint list but adding it can help with
//     // visualizations
//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     waypoints.push_back(start_pose2);

//     geometry_msgs::msg::Pose target_pose3 = start_pose2;

//     target_pose3.position.z -= 0.2;
//     waypoints.push_back(target_pose3); // down

//     target_pose3.position.y -= 0.2;
//     waypoints.push_back(target_pose3); // right

//     target_pose3.position.z += 0.2;
//     target_pose3.position.y += 0.2;
//     target_pose3.position.x -= 0.2;
//     waypoints.push_back(target_pose3); // up and left

//     // We want the Cartesian path to be interpolated at a resolution of 1
//     cm
//     // which is why we will specify 0.01 as the max step in Cartesian
//     // translation.  We will specify the jump threshold as 0.0,
//     effectively
//     // disabling it. Warning - disabling the jump threshold while
//     operating real
//     // hardware can cause large unpredictable motions of redundant joints
//     and
//     // could be a safety issue
//     moveit_msgs::msg::RobotTrajectory trajectory;
//     const double jump_threshold = 0.0;
//     const double eef_step = 0.01;
//     double fraction = move_group.computeCartesianPath(
//         waypoints, eef_step, jump_threshold, trajectory);
//     RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%%
//     achieved)",
//                 fraction * 100.0);

//     // Visualize the plan in RViz
//     visual_tools.deleteAllMarkers();
//     visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE,
//                              rvt::XLARGE);
//     visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//     for (std::size_t i = 0; i < waypoints.size(); ++i)
//         visual_tools.publishAxisLabeled(waypoints[i], "pt" +
//         std::to_string(i),
//                                         rvt::SMALL);
//     visual_tools.trigger();
//     visual_tools.prompt(
//         "Press 'next' in the RvizVisualToolsGui window to continue the
//         demo");

//     // Cartesian motions should often be slow, e.g. when approaching
//     objects.
//     // The speed of Cartesian plans cannot currently be set through the
//     // maxVelocityScalingFactor, but requires you to time the trajectory
//     // manually, as described `here
//     //
//     <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
//     // Pull requests are welcome.
//     //
//     // You can execute a trajectory like this.
//     /* move_group.execute(trajectory); */

//     // Adding objects to the environment
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//     //
//     // First, let's plan to another simple goal with no objects in the
//     way. move_group.setStartState(*move_group.getCurrentState());
//     geometry_msgs::msg::Pose another_pose;
//     another_pose.orientation.w = 0;
//     another_pose.orientation.x = -1.0;
//     another_pose.position.x = 0.7;
//     another_pose.position.y = 0.0;
//     another_pose.position.z = 0.59;
//     move_group.setPoseTarget(another_pose);

//     success =
//         (move_group.plan(my_plan) ==
//         moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s",
//                 success ? "" : "FAILED");

//     visual_tools.deleteAllMarkers();
//     visual_tools.publishText(text_pose, "Clear_Goal", rvt::WHITE,
//     rvt::XLARGE); visual_tools.publishAxisLabeled(another_pose, "goal");
//     visual_tools.publishTrajectoryLine(my_plan.trajectory,
//     joint_model_group); visual_tools.trigger(); visual_tools.prompt(
//         "Press 'next' in the RvizVisualToolsGui window to continue the
//         demo");

//     // The result may look like this:
//     //
//     // .. image:: ./move_group_interface_tutorial_clear_path.gif
//     //    :alt: animation showing the arm moving relatively straight
//     toward the
//     //    goal
//     //
//     // Now, let's define a collision object ROS message for the robot to
//     avoid. moveit_msgs::msg::CollisionObject collision_object;
//     collision_object.header.frame_id = move_group.getPlanningFrame();

//     // The id of the object is used to identify it.
//     collision_object.id = "box1";

//     // Define a box to add to the world.
//     shape_msgs::msg::SolidPrimitive primitive;
//     primitive.type = primitive.BOX;
//     primitive.dimensions.resize(3);
//     primitive.dimensions[primitive.BOX_X] = 0.1;
//     primitive.dimensions[primitive.BOX_Y] = 1.5;
//     primitive.dimensions[primitive.BOX_Z] = 0.5;

//     // Define a pose for the box (specified relative to frame_id).
//     geometry_msgs::msg::Pose box_pose;
//     box_pose.orientation.w = 1.0;
//     box_pose.position.x = 0.48;
//     box_pose.position.y = 0.0;
//     box_pose.position.z = 0.25;

//     collision_object.primitives.push_back(primitive);
//     collision_object.primitive_poses.push_back(box_pose);
//     collision_object.operation = collision_object.ADD;

//     std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//     collision_objects.push_back(collision_object);

//     // Now, let's add the collision object into the world
//     // (using a vector that could contain additional objects)
//     RCLCPP_INFO(LOGGER, "Add an object into the world");
//     planning_scene_interface.addCollisionObjects(collision_objects);

//     // Show text in RViz of status and wait for MoveGroup to receive and
//     process
//     // the collision object message
//     visual_tools.publishText(text_pose, "Add_object", rvt::WHITE,
//     rvt::XLARGE); visual_tools.trigger(); visual_tools.prompt("Press
//     'next' in the RvizVisualToolsGui window to once "
//                         "the collision object appears in RViz");

//     // Now, when we plan a trajectory it will avoid the obstacle.
//     success =
//         (move_group.plan(my_plan) ==
//         moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER, "Visualizing plan 6 (pose goal move around
//     cuboid) %s",
//                 success ? "" : "FAILED");
//     visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE,
//                              rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(my_plan.trajectory,
//     joint_model_group); visual_tools.trigger();
//     visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window
//     once "
//                         "the plan is complete");

//     // The result may look like this:
//     //
//     // .. image:: ./move_group_interface_tutorial_avoid_path.gif
//     //    :alt: animation showing the arm moving avoiding the new
//     obstacle
//     //
//     // Attaching objects to the robot
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//     //
//     // You can attach an object to the robot, so that it moves with the
//     robot
//     // geometry. This simulates picking up the object for the purpose of
//     // manipulating it. The motion planning should avoid collisions
//     between
//     // objects as well.
//     moveit_msgs::msg::CollisionObject object_to_attach;
//     object_to_attach.id = "cylinder1";

//     shape_msgs::msg::SolidPrimitive cylinder_primitive;
//     cylinder_primitive.type = primitive.CYLINDER;
//     cylinder_primitive.dimensions.resize(2);
//     cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
//     cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

//     // We define the frame/pose for this cylinder so that it appears in
//     the
//     // gripper.
//     object_to_attach.header.frame_id = move_group.getEndEffectorLink();
//     geometry_msgs::msg::Pose grab_pose;
//     grab_pose.orientation.w = 1.0;
//     grab_pose.position.z = 0.2;

//     // First, we add the object to the world (without using a vector).
//     object_to_attach.primitives.push_back(cylinder_primitive);
//     object_to_attach.primitive_poses.push_back(grab_pose);
//     object_to_attach.operation = object_to_attach.ADD;
//     planning_scene_interface.applyCollisionObject(object_to_attach);

//     // Then, we "attach" the object to the robot. It uses the frame_id to
//     // determine which robot link it is attached to. We also need to tell
//     MoveIt
//     // that the object is allowed to be in collision with the finger
//     links of
//     // the gripper. You could also use applyAttachedCollisionObject to
//     attach an
//     // object to the robot directly.
//     RCLCPP_INFO(LOGGER, "Attach the object to the robot");
//     std::vector<std::string> touch_links;
//     touch_links.push_back("panda_rightfinger");
//     touch_links.push_back("panda_leftfinger");
//     move_group.attachObject(object_to_attach.id, "panda_hand",
//     touch_links);

//     visual_tools.publishText(text_pose, "Object_attached_to_robot",
//     rvt::WHITE,
//                              rvt::XLARGE);
//     visual_tools.trigger();

//     /* Wait for MoveGroup to receive and process the attached collision
//     object
//      * message */
//     visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window
//     once "
//                         "the new object is attached to the robot");

//     // Replan, but now with the object in hand.
//     move_group.setStartStateToCurrentState();
//     success =
//         (move_group.plan(my_plan) ==
//         moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(LOGGER,
//                 "Visualizing plan 7 (move around cuboid with cylinder)
//                 %s", success ? "" : "FAILED");
//     visual_tools.publishTrajectoryLine(my_plan.trajectory,
//     joint_model_group); visual_tools.trigger();
//     visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window
//     once "
//                         "the plan is complete");

//     // The result may look something like this:
//     //
//     // .. image:: ./move_group_interface_tutorial_attached_object.gif
//     //    :alt: animation showing the arm moving differently once the
//     object is
//     //    attached
//     //
//     // Detaching and Removing Objects
//     // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//     //
//     // Now, let's detach the cylinder from the robot's gripper.
//     RCLCPP_INFO(LOGGER, "Detach the object from the robot");
//     move_group.detachObject(object_to_attach.id);

//     // Show text in RViz of status
//     visual_tools.deleteAllMarkers();
//     visual_tools.publishText(text_pose, "Object_detached_from_robot",
//                              rvt::WHITE, rvt::XLARGE);
//     visual_tools.trigger();

//     /* Wait for MoveGroup to receive and process the attached collision
//     object
//      * message */
//     visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window
//     once "
//                         "the new object is detached from the robot");

//     // Now, let's remove the objects from the world.
//     RCLCPP_INFO(LOGGER, "Remove the objects from the world");
//     std::vector<std::string> object_ids;
//     object_ids.push_back(collision_object.id);
//     object_ids.push_back(object_to_attach.id);
//     planning_scene_interface.removeCollisionObjects(object_ids);

//     // Show text in RViz of status
//     visual_tools.publishText(text_pose, "Objects_removed", rvt::WHITE,
//                              rvt::XLARGE);
//     visual_tools.trigger();

//     /* Wait for MoveGroup to receive and process the attached collision
//     object
//      * message */
//     visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to
//     once "
//                         "the collision object disappears");

//     // END_TUTORIAL
//     visual_tools.deleteAllMarkers();
//     visual_tools.trigger();

//     rclcpp::shutdown();
//     return 0;
// }


