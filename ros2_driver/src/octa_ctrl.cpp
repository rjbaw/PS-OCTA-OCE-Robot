#include <Eigen/Dense>
#include <chrono>
#include <csignal>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "dds_publisher.hpp"
#include "dds_subscriber.hpp"
#include "img_subscriber.hpp"
#include "process_img.hpp"
#include "urscript_publisher.hpp"
#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

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

/// reconnect

#include "rclcpp_action/rclcpp_action.hpp"
#include "ur_dashboard_msgs/ur_dashboard_msgs/action/set_mode.hpp"
#include "ur_dashboard_msgs/ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/ur_dashboard_msgs/srv/get_program_state.hpp"
#include "ur_dashboard_msgs/ur_dashboard_msgs/srv/get_robot_mode.hpp"
#include <std_srvs/srv/trigger.hpp>

using SetMode = ur_dashboard_msgs::action::SetMode;
using GoalHandleSetMode = rclcpp_action::ClientGoalHandle<SetMode>;

class SetModeActionClient : public rclcpp::Node {
  public:
    SetModeActionClient() : Node("set_mode_action_client") {
        client_ = rclcpp_action::create_client<SetMode>(
            this, "/dashboard_client/set_mode");
    }

    void send_goal(int8_t target_mode, bool stop_program = false,
                   bool play_program = false) {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(),
                         "Action server not available after waiting");
            return;
        }

        auto goal_msg = SetMode::Goal();
        goal_msg.target_robot_mode = target_mode;
        goal_msg.stop_program = stop_program;
        goal_msg.play_program = play_program;

        RCLCPP_INFO(
            this->get_logger(),
            "Sending goal: target_mode=%d stop_program=%s play_program=%s",
            static_cast<int>(target_mode), stop_program ? "true" : "false",
            play_program ? "true" : "false");

        auto send_goal_options =
            rclcpp_action::Client<SetMode>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&SetModeActionClient::feedback_callback, this,
                      std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(
            &SetModeActionClient::result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    void
    feedback_callback(GoalHandleSetMode::SharedPtr,
                      const std::shared_ptr<const SetMode::Feedback> feedback) {
        RCLCPP_INFO(
            this->get_logger(),
            "Feedback - current_robot_mode: %d, current_safety_mode: %d",
            feedback->current_robot_mode, feedback->current_safety_mode);
    }

    void result_callback(const GoalHandleSetMode::WrappedResult &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
                        "Result received: success=%s, message=%s",
                        result.result->success ? "true" : "false",
                        result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
    }

    rclcpp_action::Client<SetMode>::SharedPtr client_;
};

static const int8_t NO_CONTROLLER = -1;
static const int8_t DISCONNECTED = 0;
static const int8_t CONFIRM_SAFETY = 1;
static const int8_t BOOTING = 2;
static const int8_t POWER_OFF = 3;
static const int8_t POWER_ON = 4;
static const int8_t IDLE = 5;
static const int8_t BACKDRIVE = 6;
static const int8_t RUNNING = 7;
static const int8_t UPDATING_FIRMWARE = 8;

#include "ur_dashboard_msgs/ur_dashboard_msgs/srv/get_robot_mode.hpp"

class GetRobotModeClient : public rclcpp::Node {
  public:
    // Constructor name matches the class name
    GetRobotModeClient()
        : Node("get_robot_mode_client") // name your node
    {
        // Create the service client for the official UR Dashboard service
        client_ = this->create_client<ur_dashboard_msgs::srv::GetRobotMode>(
            "/dashboard_client/get_robot_mode");

        // Create a timer that calls timerCallback() every 500ms (2 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&GetRobotModeClient::timerCallback, this));
    }

    // Provide a getter so other code can read the last known robot mode
    int8_t getCurrentMode() const { return current_mode_.load(); }

  private:
    void timerCallback() {
        using namespace std::chrono_literals;

        // Check if the service is up
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(),
                        "[GetRobotModeClient] Service "
                        "/dashboard_client/get_robot_mode not available");
            return;
        }

        // Create an empty request
        auto request =
            std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>();

        // Asynchronously call the service
        auto future_result = client_->async_send_request(request);

        // Instead of spin_until_future_complete, use wait_for
        auto status = future_result.wait_for(std::chrono::seconds(2));

        if (status == std::future_status::ready) {
            // We got a response
            auto response = future_result.get();
            if (response->success) {
                // Store the last known mode in our atomic variable
                current_mode_.store(response->robot_mode.mode);

                RCLCPP_INFO(
                    this->get_logger(),
                    "[GetRobotModeClient] Polled Robot mode: %d, answer: %s",
                    response->robot_mode.mode, response->answer.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "[GetRobotModeClient] Service call succeeded, but "
                            "robot reported failure: %s",
                            response->answer.c_str());
                current_mode_.store(DISCONNECTED);
            }
        } else {
            // Timed out or an error occurred
            RCLCPP_ERROR(
                this->get_logger(),
                "[GetRobotModeClient] Timeout or error calling get_robot_mode");
            current_mode_.store(DISCONNECTED);
        }
    }

    // The client to /dashboard_client/get_robot_mode
    rclcpp::Client<ur_dashboard_msgs::srv::GetRobotMode>::SharedPtr client_;

    // The timer that periodically calls timerCallback()
    rclcpp::TimerBase::SharedPtr timer_;

    // Atomic to hold the last known robot mode from the UR driver
    std::atomic<int8_t> current_mode_{DISCONNECTED};
};

/// reconnect

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    using moveit::planning_interface::MoveGroupInterface;

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // 3D Parameters
    const int interval = 4;
    const bool single_interval = false;

    // Publisher Parameters
    std::string msg;
    double angle = 0.0;
    int circle_state = 1;
    bool apply_config = true;
    bool end_state = false;
    bool scan_3d = false;

    // Internal Parameters
    bool planning = false;
    double angle_increment;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    Eigen::Matrix3d rotmat_eigen;
    cv::Mat img;
    std::vector<cv::Mat> img_array;
    std::vector<Eigen::Vector3d> pc_lines;
    tf2::Quaternion q;
    tf2::Quaternion target_q;
    geometry_msgs::msg::Pose target_pose;

    // Subscriber Parameters
    double robot_vel;
    double robot_acc;
    double radius;
    double angle_limit;
    double dz;
    double z_tolerance;
    double angle_tolerance;
    double z_height;
    int num_pt;
    bool fast_axis = true;
    bool success = false;
    bool next = false;
    bool previous = false;
    bool home = false;
    bool z_focused = false;
    bool angle_focused = false;

    auto const move_group_node =
        std::make_shared<rclcpp::Node>("node_moveit", node_options);
    auto move_group_interface =
        MoveGroupInterface(move_group_node, "ur_manipulator");
    auto subscriber_node = std::make_shared<dds_subscriber>();
    auto urscript_node = std::make_shared<urscript_publisher>();
    auto publisher_node = std::make_shared<dds_publisher>(
        msg, angle, circle_state, fast_axis, apply_config, end_state, scan_3d);
    auto img_subscriber_node = std::make_shared<img_subscriber>();
    auto robot_mode_node = std::make_shared<GetRobotModeClient>();
    auto robot_set_node = std::make_shared<SetModeActionClient>();

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client;

    auto const logger = rclcpp::get_logger("logger_planning");

    // int attempt = 0;
    // int max_attempt = 5;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);
    executor.add_node(urscript_node);
    executor.add_node(img_subscriber_node);
    executor.add_node(robot_mode_node);
    executor.add_node(robot_set_node);
    std::thread spinner([&executor]() { executor.spin(); });

    add_collision_obj(move_group_interface);

    move_group_interface.setPlanningTime(10.0);
    move_group_interface.setNumPlanningAttempts(30);
    move_group_interface.setPlanningPipelineId("ompl");

    // auto robot_mode_node = std::make_shared<GetRobotModeClient>();
    // auto robot_set_node = std::make_shared<SetModeActionClient>();
    // RCLCPP_INFO(node->get_logger(), "Requesting POWER_ON...");
    // node->send_goal(POWER_ON);

    while (rclcpp::ok() && running) {

        end_state = false;
        apply_config = false;
        fast_axis = true;
        publisher_node->set_fast_axis(fast_axis);
        publisher_node->set_apply_config(apply_config);
        publisher_node->set_end_state(end_state);
        next = subscriber_node->next();
        previous = subscriber_node->previous();
        home = subscriber_node->home();

        z_tolerance = subscriber_node->z_tolerance();
        angle_tolerance = subscriber_node->angle_tolerance();
        radius = subscriber_node->radius();
        angle_limit = subscriber_node->angle_limit();
        num_pt = subscriber_node->num_pt();
        robot_vel = subscriber_node->robot_vel();
        robot_acc = subscriber_node->robot_acc();
        z_height = subscriber_node->z_height();
        robot_vel = 0.1;
        robot_acc = 0.1;

        if (subscriber_node->freedrive()) {
            circle_state = 1;
            angle = 0.0;
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);
            urscript_node->activate_freedrive();
            while (subscriber_node->freedrive()) {
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
            urscript_node->deactivate_freedrive();
        } else {
            if (!robot_mode_node->getCurrentMode()) {
                RCLCPP_INFO(logger, "Requesting RUNNING...");
                robot_set_node->send_goal(RUNNING, true, false);

                trigger_client =
                    robot_mode_node->create_client<std_srvs::srv::Trigger>(
                        "/io_and_status_controller/resend_robot_program");
                auto request =
                    std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = trigger_client->async_send_request(request);
            }
        }

        move_group_interface.setMaxVelocityScalingFactor(robot_vel);
        move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
        move_group_interface.setStartStateToCurrentState();

        if (subscriber_node->reset()) {
            angle = 0.0;
            circle_state = 1;
            msg = "Reset to default position";
            RCLCPP_INFO(logger, msg.c_str());
            publisher_node->set_msg(msg);
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);
            move_group_interface.setJointValueTarget("shoulder_pan_joint",
                                                     to_radian(0.0));
            move_group_interface.setJointValueTarget("shoulder_lift_joint",
                                                     -to_radian(60.0));
            move_group_interface.setJointValueTarget("elbow_joint",
                                                     to_radian(90.0));
            move_group_interface.setJointValueTarget("wrist_1_joint",
                                                     to_radian(-120.0));
            move_group_interface.setJointValueTarget("wrist_2_joint",
                                                     to_radian(-90.0));
            move_group_interface.setJointValueTarget("wrist_3_joint",
                                                     to_radian(-135.0));

            success = move_to_target(move_group_interface, logger);
            // if (!success) {
            //     msg = std::format("Reset Planning Failed!");
            //     publisher_node->set_msg(msg);
            //     RCLCPP_ERROR(logger, msg.c_str());
            // }
        }

        if (subscriber_node->autofocus()) {
            apply_config = true;
            publisher_node->set_apply_config(apply_config);
            if (!scan_3d) {
                scan_3d = true;
                apply_config = true;
                msg = "Starting 3D Scan";
                RCLCPP_INFO(logger, msg.c_str());
                publisher_node->set_msg(msg);
                publisher_node->set_scan_3d(scan_3d);
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
            img_array.clear();
            for (int i = 0; i < interval; i++) {
                while (true) {
                    img = img_subscriber_node->get_img();
                    if (!img.empty()) {
                        break;
                    }
                    if (!subscriber_node->autofocus()) {
                        break;
                    }
                    // rclcpp::sleep_for(std::chrono::milliseconds(10000));
                }
                img_array.push_back(img);
                RCLCPP_INFO(logger, "Collected image %d", i + 1);
            }
            msg = "Calculating Rotations";
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());

            // scan_3d = false;
            // apply_config = true;
            // publisher_node->set_scan_3d(scan_3d);
            // publisher_node->set_apply_config(apply_config);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));

            pc_lines = lines_3d(img_array, interval, single_interval);
            open3d::geometry::PointCloud pcd_lines;
            for (const auto &point : pc_lines) {
                pcd_lines.points_.emplace_back(point);
            }
            auto boundbox = pcd_lines.GetMinimalOrientedBoundingBox(false);
            Eigen::Vector3d center = boundbox.GetCenter();
            msg += std::format("\n[Center] x:{} y:{} z:{}", center[0],
                               center[1], center[2]);
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());
            rotmat_eigen = align_to_direction(boundbox.R_);

            tf2::Matrix3x3 rotmat_tf(
                rotmat_eigen(0, 0), rotmat_eigen(0, 1), rotmat_eigen(0, 2),
                rotmat_eigen(1, 0), rotmat_eigen(1, 1), rotmat_eigen(1, 2),
                rotmat_eigen(2, 0), rotmat_eigen(2, 1), rotmat_eigen(2, 2));
            RCLCPP_INFO_STREAM(logger, "\nAligned Rotation Matrix:\n"
                                           << rotmat_eigen);

            /// temporary fix for swapped axis ///
            double tmp_roll;
            double tmp_pitch;
            double tmp_yaw;
            rotmat_tf.getRPY(tmp_roll, tmp_pitch, tmp_yaw);
            roll = tmp_yaw;
            pitch = tmp_roll;
            yaw = tmp_pitch;
            rotmat_tf.setRPY(roll, pitch, yaw);
            ///////////////////////

            msg +=
                std::format("\nCalculated R:{:.2f}, P:{:.2f}, Y:{:.2f}",
                            to_degree(roll), to_degree(pitch), to_degree(yaw));
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());

            if (tol_measure(roll, pitch, angle_tolerance)) {
                angle_focused = true;
                msg += "\nAngle focused";
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, msg.c_str());
                scan_3d = false;
                apply_config = true;
                publisher_node->set_scan_3d(scan_3d);
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }

            if (angle_focused && !z_focused) {
                // dz = 0;
                dz = (z_height - center[1]) / (50 * 1000.0);
                msg += std::format("\ndz = {}", dz);
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, "dz: %f", dz);
                if (std::abs(dz) < (z_tolerance / 1000.0)) {
                    z_focused = true;
                    msg += "\nHeight focused";
                    publisher_node->set_msg(msg);
                    RCLCPP_INFO(logger, msg.c_str());
                } else {
                    planning = true;
                    target_pose = move_group_interface.getCurrentPose().pose;
                    target_pose.position.z += dz;
                    print_target(logger, target_pose);
                    move_group_interface.setPoseTarget(target_pose);
                    success = move_to_target(move_group_interface, logger);
                    // if (!success) {
                    //     msg = std::format("Z-height Planning Failed!");
                    //     RCLCPP_ERROR(logger, msg.c_str());
                    //     publisher_node->set_msg(msg);
                    // }
                }
            }

            if (!angle_focused) {
                planning = true;
                rotmat_tf.getRotation(q);
                target_pose = move_group_interface.getCurrentPose().pose;
                tf2::fromMsg(target_pose.orientation, target_q);
                q.normalize();
                target_q = target_q * q;
                target_pose.orientation = tf2::toMsg(target_q);
                target_pose.position.x += radius * std::cos(to_radian(angle));
                target_pose.position.y += radius * std::sin(to_radian(angle));
                target_pose.position.z += dz;
                print_target(logger, target_pose);
                move_group_interface.setPoseTarget(target_pose);
                success = move_to_target(move_group_interface, logger);
                if (!success) {
                    // msg = std::format("Angle Focus Planning Failed!");
                    msg = "Angle Focus Planning Failed!";
                    publisher_node->set_msg(msg);
                    RCLCPP_ERROR(logger, msg.c_str());
                }
            }

            if (angle_focused && z_focused) {
                angle_focused = false;
                z_focused = false;
                planning = false;
                end_state = true;
                msg += "\nWithin tolerance";
                publisher_node->set_msg(msg);
                publisher_node->set_end_state(end_state);
                move_group_interface.setStartStateToCurrentState();
                target_pose = move_group_interface.getCurrentPose().pose;
                while (subscriber_node->autofocus()) {
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }
            }
        } else {
            angle_increment = angle_limit / num_pt;
            roll = 0.0, pitch = 0.0, yaw = 0.0;

            if (next) {
                planning = true;
                yaw += to_radian(angle_increment);
            }
            if (previous) {
                planning = true;
                yaw += to_radian(-angle_increment);
            }
            if (home) {
                planning = true;
                yaw += to_radian(-angle);
            }
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);

            if (planning) {
                target_pose = move_group_interface.getCurrentPose().pose;
                tf2::fromMsg(target_pose.orientation, target_q);
                q.setRPY(roll, pitch, yaw);
                q.normalize();
                target_q = target_q * q;
                target_pose.orientation = tf2::toMsg(target_q);
                print_target(logger, target_pose);
                move_group_interface.setPoseTarget(target_pose);
                success = move_to_target(move_group_interface, logger);
                if (success) {
                    msg = "Planning Success!";
                    RCLCPP_INFO(logger, msg.c_str());
                    if (next) {
                        angle += angle_increment;
                        circle_state++;
                    }
                    if (previous) {
                        angle -= angle_increment;
                        circle_state--;
                    }
                    if (home) {
                        circle_state = 1;
                        angle = 0.0;
                    }
                } else {
                    // msg = std::format("Z-axis Rotation Planning Failed! "
                    //                   "\nAttempt[{}] Retrying....");
                    msg = "Z-axis Rotation Planning Failed!";
                    RCLCPP_ERROR(logger, msg.c_str());
                    publisher_node->set_msg(msg);
                }
            }
        }

        if (planning) {
            planning = false;
            while (!subscriber_node->changed()) {
                if (!subscriber_node->autofocus()) {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                    break;
                }
            }
            // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }

        if (scan_3d && !subscriber_node->autofocus()) {
            scan_3d = false;
            apply_config = true;
            publisher_node->set_scan_3d(scan_3d);
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
