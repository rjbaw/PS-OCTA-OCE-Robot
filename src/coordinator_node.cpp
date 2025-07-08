/**
 * @file coordinator_node.cpp
 * @author rjbaw
 * @brief coordinator node that executes action servers, subscribe, and publish
 * to LabView interface.
 * Actions are toggled using rising edge to prevent multiple triggers.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <format>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <std_msgs/msg/bool.hpp>

#include <action_msgs/msg/goal_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <octa_ros/msg/labviewdata.hpp>
#include <octa_ros/msg/robotdata.hpp>

#include <octa_ros/action/focus.hpp>
#include <octa_ros/action/freedrive.hpp>
#include <octa_ros/action/move_z_angle.hpp>
#include <octa_ros/action/reset.hpp>

#include <octa_ros/srv/scan3d.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "utils.hpp"

using namespace std::chrono_literals;

enum class UserAction {
    None,
    Freedrive,
    Reset,
    MoveZangle,
    Focus,
    Scan,
};

enum class Mode {
    ROBOT,
    OCT,
    OCTA,
    OCE,
};

enum class ScanState {
    IDLE,
    BUSY,
};

struct Step {
    UserAction action;
    Mode mode;
    double arg;
};

const std::vector<Step> full_scan_recipe = {
    {UserAction::Focus, Mode::ROBOT, 0},
    // octa
    {UserAction::Scan, Mode::OCTA, 0},
    // first 60
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    // octa
    {UserAction::Scan, Mode::OCTA, 0},
    // second 60
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    // octa
    {UserAction::Scan, Mode::OCTA, 0},
    // third 60
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    {UserAction::MoveZangle, Mode::OCT, +10},
    {UserAction::Scan, Mode::OCT, 0},
    {UserAction::Scan, Mode::OCE, 0},
    // final
    {UserAction::Scan, Mode::OCTA, 0},
};

class CoordinatorNode : public rclcpp::Node {
  public:
    using FocusAction = octa_ros::action::Focus;
    using MoveZAngle = octa_ros::action::MoveZAngle;
    using Freedrive = octa_ros::action::Freedrive;
    using Reset = octa_ros::action::Reset;

    using Scan3d = octa_ros::srv::Scan3d;

    using FocusGoalHandle = rclcpp_action::ClientGoalHandle<FocusAction>;
    using MoveZGoalHandle = rclcpp_action::ClientGoalHandle<MoveZAngle>;
    using FreedriveGoalHandle = rclcpp_action::ClientGoalHandle<Freedrive>;
    using ResetGoalHandle = rclcpp_action::ClientGoalHandle<Reset>;

    explicit CoordinatorNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("coordinator_node",
               rclcpp::NodeOptions(options)
                   .automatically_declare_parameters_from_overrides(true)) {}

    void init() {
        // apply_config_ = true;
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            pub_handle_ = this->create_publisher<octa_ros::msg::Robotdata>(
                "robot_data", qos);
        }

        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            sub_handle_ = this->create_subscription<octa_ros::msg::Labviewdata>(
                "labview_data", qos,
                std::bind(&CoordinatorNode::subscriberCallback, this,
                          std::placeholders::_1));
        }
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            cancel_handle_ = this->create_subscription<std_msgs::msg::Bool>(
                "cancel_current_action", qos,
                std::bind(&CoordinatorNode::cancelCallback, this,
                          std::placeholders::_1));
        }

        {
            scan_3d_srv_ = create_service<Scan3d>(
                "scan_3d",
                std::bind(&CoordinatorNode::scan3dCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
            deactivate_focus_srv_ = create_service<std_srvs::srv::Trigger>(
                "deactivate_focus",
                std::bind(&CoordinatorNode::deactivateFocusCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
        }

        moveit_cpp_ =
            std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());

        moveit_msgs::msg::CollisionObject collision_floor;
        collision_floor.header.frame_id = moveit_cpp_->getPlanningSceneMonitor()
                                              ->getPlanningScene()
                                              ->getPlanningFrame();
        collision_floor.id = "floor";
        collision_floor.operation = collision_floor.ADD;

        {
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {10.0, 10.0, 0.01};

            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.0;
            box_pose.position.y = 0.0;
            box_pose.position.z = -0.0855;

            collision_floor.primitives.push_back(primitive);
            collision_floor.primitive_poses.push_back(box_pose);
        }

        moveit_msgs::msg::CollisionObject collision_base;
        collision_base.header.frame_id = moveit_cpp_->getPlanningSceneMonitor()
                                             ->getPlanningScene()
                                             ->getPlanningFrame();
        collision_base.id = "robot_base";
        collision_base.operation = collision_base.ADD;

        {
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {0.27, 0.27, 0.085};

            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.0;
            box_pose.position.y = 0.0;
            box_pose.position.z = -0.043;

            collision_base.primitives.push_back(primitive);
            collision_base.primitive_poses.push_back(box_pose);
        }

        moveit_msgs::msg::CollisionObject collision_monitor;
        collision_monitor.header.frame_id =
            moveit_cpp_->getPlanningSceneMonitor()
                ->getPlanningScene()
                ->getPlanningFrame();
        collision_monitor.id = "monitor";
        collision_monitor.operation = collision_monitor.ADD;

        {
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {0.25, 0.6, 0.6};

            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = -0.2;
            box_pose.position.y = 0.435;
            box_pose.position.z = 0.215;

            collision_monitor.primitives.push_back(primitive);
            collision_monitor.primitive_poses.push_back(box_pose);
        }

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_floor);
        collision_objects.push_back(collision_base);
        collision_objects.push_back(collision_monitor);
        psi.addCollisionObjects(collision_objects);

        RCLCPP_INFO(get_logger(), "Collision objects added to planning scene.");

        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&CoordinatorNode::publisherCallback, this));

        focus_action_client_ =
            rclcpp_action::create_client<FocusAction>(this, "focus_action");
        move_z_angle_action_client_ = rclcpp_action::create_client<MoveZAngle>(
            this, "move_z_angle_action");
        freedrive_action_client_ =
            rclcpp_action::create_client<Freedrive>(this, "freedrive_action");
        reset_action_client_ =
            rclcpp_action::create_client<Reset>(this, "reset_action");

        service_capture_background_ =
            create_client<std_srvs::srv::Trigger>("capture_background");

        main_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&CoordinatorNode::mainLoop, this));

        if (!focus_action_client_->wait_for_action_server(
                std::chrono::milliseconds(200))) {
            RCLCPP_WARN(get_logger(), "Focus action server not available yet.");
        }
        if (!move_z_angle_action_client_->wait_for_action_server(
                std::chrono::milliseconds(200))) {
            RCLCPP_WARN(get_logger(),
                        "MoveZAngle action server not available yet.");
        }
        if (!freedrive_action_client_->wait_for_action_server(
                std::chrono::milliseconds(200))) {
            RCLCPP_WARN(get_logger(),
                        "Freedrive action server not available yet.");
        }
        if (!reset_action_client_->wait_for_action_server(
                std::chrono::milliseconds(200))) {
            RCLCPP_WARN(get_logger(), "Reset action server not available yet.");
        }

        RCLCPP_INFO(get_logger(), "Coordinator Node Initialized.");
    }

  private:
    rclcpp_action::Client<FocusAction>::SharedPtr focus_action_client_;
    rclcpp_action::Client<MoveZAngle>::SharedPtr move_z_angle_action_client_;
    rclcpp_action::Client<Freedrive>::SharedPtr freedrive_action_client_;
    rclcpp_action::Client<Reset>::SharedPtr reset_action_client_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        service_capture_background_;

    moveit_cpp::MoveItCppPtr moveit_cpp_;
    moveit::planning_interface::PlanningSceneInterface psi;

    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr pub_handle_;
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr sub_handle_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_handle_;

    rclcpp::Service<Scan3d>::SharedPtr scan_3d_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr deactivate_focus_srv_;

    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr main_loop_timer_;

    FocusGoalHandle::SharedPtr active_focus_goal_handle_;
    MoveZGoalHandle::SharedPtr active_move_z_goal_handle_;
    FreedriveGoalHandle::SharedPtr active_freedrive_goal_handle_;
    ResetGoalHandle::SharedPtr active_reset_goal_handle_;

    // Internal variables
    UserAction current_action_ = UserAction::None;
    UserAction previous_action_ = UserAction::None;
    octa_ros::msg::Labviewdata old_sub_msg_;
    octa_ros::msg::Robotdata old_pub_msg_;
    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
    double angle_increment_ = 0.0;
    std::mutex data_mutex_;
    rclcpp::Time start;
    bool scan_trigger_store_ = false;
    std::atomic<unsigned int> pc_ = 0;
    std::atomic<ScanState> scan_state_ = ScanState::IDLE;

    rclcpp::TimerBase::SharedPtr config_timer_;
    std::weak_ptr<rclcpp::TimerBase> config_timer_weak_;
    rclcpp::TimerBase::SharedPtr scan_timer_;
    std::weak_ptr<rclcpp::TimerBase> scan_timer_weak_;

    // Service variables
    std::atomic<bool> cancel_action_ = false;
    std::atomic<bool> triggered_service_ = false;

    // Publisher fields
    std::string msg_ = "idle";
    std::atomic<double> angle_ = 0.0;
    std::atomic<int> circle_state_ = 1;
    std::atomic<bool> scan_trigger_ = false;
    std::atomic<bool> apply_config_ = false;
    std::atomic<bool> end_state_ = false;
    std::atomic<bool> scan_3d_ = false;
    std::atomic<bool> robot_mode_ = true;
    std::atomic<bool> oct_mode_ = false;
    std::atomic<bool> octa_mode_ = false;
    std::atomic<bool> oce_mode_ = false;

    // Subscriber fields
    std::atomic<double> robot_vel_ = 0.5;
    std::atomic<double> robot_acc_ = 0.5;
    std::atomic<double> z_height_ = 0.0;
    std::atomic<double> z_tolerance_ = 0.0;
    std::atomic<double> angle_tolerance_ = 0.0;
    std::atomic<double> radius_ = 0.0;
    std::atomic<double> angle_limit_ = 0.0;
    std::atomic<double> dz_ = 0.0;
    std::atomic<double> drot_ = 0.0;
    std::atomic<bool> autofocus_ = false;
    std::atomic<bool> freedrive_ = false;
    std::atomic<bool> previous_ = false;
    std::atomic<bool> next_ = false;
    std::atomic<bool> home_ = false;
    std::atomic<bool> reset_ = false;
    std::atomic<bool> scan_trigger_read_ = false;
    std::atomic<bool> scan_3d_read_ = false;
    std::atomic<bool> full_scan_ = false;
    std::atomic<bool> full_scan_read_ = false;
    std::atomic<int> num_pt_ = 1;
    std::atomic<bool> robot_mode_read_ = true;
    std::atomic<bool> oct_mode_read_ = false;
    std::atomic<bool> octa_mode_read_ = false;
    std::atomic<bool> oce_mode_read_ = false;

    template <class Flag>
    void triggerFlag(Flag &flag, rclcpp::TimerBase::SharedPtr &timer_ptr,
                     std::weak_ptr<rclcpp::TimerBase> &weak_ptr,
                     rclcpp::Node &node, std::chrono::milliseconds duration,
                     bool block) {
        if (timer_ptr) {
            timer_ptr->cancel();
            timer_ptr.reset();
        }
        flag = true;
        auto done_ptr = std::make_shared<std::promise<void>>();
        auto future = done_ptr->get_future();
        auto weak_copy = weak_ptr;
        timer_ptr =
            node.create_wall_timer(duration, [&, done_ptr, weak_copy]() {
                if (auto t = weak_copy.lock()) {
                    t->cancel();
                }
                flag = false;
                done_ptr->set_value();
            });
        weak_ptr = timer_ptr;
        if (block) {
            future.wait();
        }
    }

    template <typename GH> bool goal_still_active(const GH &handle) {
        if (!handle) {
            return false;
        }
        auto status = handle->get_status();
        return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
               status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    }

    void trigger_scan() {
        triggerFlag(scan_trigger_, scan_timer_, scan_timer_weak_, *this, 20ms,
                    true);
        scan_state_ = ScanState::BUSY;
    }

    void trigger_apply_config() {
        triggerFlag(apply_config_, config_timer_, config_timer_weak_, *this,
                    20ms, false);
    }

    void subscriberCallback(const octa_ros::msg::Labviewdata::SharedPtr msg) {
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
        scan_trigger_read_ = msg->scan_trigger;
        scan_3d_read_ = msg->scan_3d;
        z_height_ = msg->z_height;
        full_scan_read_ = msg->full_scan;
        robot_mode_read_ = msg->robot_mode;
        oct_mode_read_ = msg->oct_mode;
        octa_mode_read_ = msg->octa_mode;
        oce_mode_read_ = msg->oce_mode;
        if (*msg != old_sub_msg_) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            RCLCPP_INFO(
                get_logger(),
                std::format("[SUBSCRIBING]\n"
                            "  robot_vel: {}, robot_acc: {}, \n"
                            "  z_tolerance: {}, angle_tolerance: {}, \n"
                            "  z_height: {}, \n"
                            "  radius: {}, \n"
                            "  angle_limit: {}, \n"
                            "  num_pt: {}, "
                            "  dz: {}, drot: {}, \n"
                            "  autofocus: {}, \n"
                            "  freedrive: {}, \n"
                            "  previous: {}, next: {}, home: {}, \n"
                            "  reset: {}, \n"
                            "  scan_trigger: {}, scan_3d: {}, \n"
                            "  full_scan: {}, \n"
                            " robot_mode: {}, oct_mode: {}, octa_mode: {}, "
                            "oce_mode: {}",
                            robot_vel_.load(), robot_acc_.load(),
                            z_tolerance_.load(), angle_tolerance_.load(),
                            z_height_.load(), radius_.load(),
                            angle_limit_.load(), num_pt_.load(), dz_.load(),
                            drot_.load(), autofocus_.load(), freedrive_.load(),
                            previous_.load(), next_.load(), home_.load(),
                            reset_.load(), scan_trigger_read_.load(),
                            scan_3d_read_.load(), full_scan_read_.load(),
                            robot_mode_read_.load(), oct_mode_read_.load(),
                            octa_mode_read_.load(), oce_mode_read_.load())
                    .c_str());
        }
        old_sub_msg_ = *msg;
    }

    void cancelCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        cancel_action_ = msg->data;
        if (cancel_action_) {
            autofocus_ = false;
        }
    }

    void publisherCallback() {
        octa_ros::msg::Robotdata msg;
        msg.msg = msg_;
        msg.angle = angle_.load();
        msg.circle_state = circle_state_.load();
        msg.scan_trigger = scan_trigger_.load();
        msg.apply_config = apply_config_.load();
        msg.end_state = end_state_.load();
        msg.scan_3d = scan_3d_.load();
        msg.full_scan = full_scan_.load();
        msg.robot_mode = robot_mode_.load();
        msg.oct_mode = oct_mode_.load();
        msg.octa_mode = octa_mode_.load();
        msg.oce_mode = oce_mode_.load();

        if (msg != old_pub_msg_) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            RCLCPP_INFO(get_logger(),
                        "[PUBLISHING] \n"
                        "  angle: %.2f, \n"
                        "  circle_state: %d, \n"
                        "  scan_trigger: %s, \n"
                        "  apply_config: %s, \n"
                        "  end_state: %s, \n"
                        "  scan_3d: %s, \n"
                        "  full_scan: %s, \n"
                        "  robot_mode: %s, \n"
                        "  oct_mode: %s, \n"
                        "  octa_mode: %s, \n"
                        "  oce_mode: %s \n",
                        msg.angle, msg.circle_state,
                        (msg.scan_trigger ? "true" : "false"),
                        (msg.apply_config ? "true" : "false"),
                        (msg.end_state ? "true" : "false"),
                        (msg.scan_3d ? "true" : "false"),
                        (msg.full_scan ? "true" : "false"),
                        (msg.robot_mode ? "true" : "false"),
                        (msg.oct_mode ? "true" : "false"),
                        (msg.octa_mode ? "true" : "false"),
                        (msg.oce_mode ? "true" : "false"));
        }

        pub_handle_->publish(msg);
        old_pub_msg_ = msg;
    }

    void mainLoop() {
        if (cancel_action_) {
            if (goal_still_active(active_focus_goal_handle_)) {
                msg_ = "Canceling Focus action\n";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                focus_action_client_->async_cancel_goal(
                    active_focus_goal_handle_);
            }
            if (goal_still_active(active_move_z_goal_handle_)) {
                msg_ = "Canceling Move Z-angle action\n";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                move_z_angle_action_client_->async_cancel_goal(
                    active_move_z_goal_handle_);
            }
            if (goal_still_active(active_freedrive_goal_handle_)) {
                msg_ = "Canceling Free-drive\n";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                freedrive_action_client_->async_cancel_goal(
                    active_freedrive_goal_handle_);
            }
            if (goal_still_active(active_reset_goal_handle_)) {
                msg_ = "Canceling Reset action\n";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                reset_action_client_->async_cancel_goal(
                    active_reset_goal_handle_);
            }
            if (full_scan_read_) {
                full_scan_ = false;
                msg_ = "Canceling Full Scan action\n";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                start = now();
                while (full_scan_read_.load() != full_scan_) {
                    rclcpp::sleep_for(std::chrono::milliseconds(1));
                    if ((now() - start).seconds() > 1.0) {
                        msg_ += "Timed out changing full scan variable\n";
                        RCLCPP_INFO(this->get_logger(), msg_.c_str());
                        break;
                    }
                };
            }
            pc_ = 0;
            scan_state_ = ScanState::IDLE;
            current_action_ = UserAction::None;
            previous_action_ = UserAction::None;
            cancel_action_ = false;
            return;
        }

        if (full_scan_read_) {
            full_scan_ = true;
            if (scan_trigger_read_ != scan_trigger_store_) {
                msg_ += "Scan Complete";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                scan_state_ = ScanState::IDLE;
                scan_trigger_store_ = scan_trigger_read_.load();
            }
            if ((pc_.load() + 1) >= full_scan_recipe.size()) {
                pc_ = 0;
                full_scan_ = false;
                msg_ = "Full Scan complete!\n";
                start = now();
                while (full_scan_read_.load() != full_scan_) {
                    rclcpp::sleep_for(std::chrono::milliseconds(1));
                    if ((now() - start).seconds() > 1.0) {
                        msg_ += "Timed out changing full scan variable\n";
                        RCLCPP_INFO(this->get_logger(), msg_.c_str());
                        break;
                    }
                };
                return;
            }
            const Step &step = full_scan_recipe[pc_.load()];
            robot_mode_ = (step.mode == Mode::ROBOT);
            oct_mode_ = (step.mode == Mode::OCT);
            octa_mode_ = (step.mode == Mode::OCTA);
            oce_mode_ = (step.mode == Mode::OCE);
            std::string action_mode;
            std::string scan_mode;
            if (robot_mode_) {
                scan_mode = "ROBOT Mode";
            } else if (oct_mode_) {
                scan_mode = "OCT Mode";
            } else if (octa_mode_) {
                scan_mode = "OCTA Mode";
            } else if (oce_mode_) {
                scan_mode = "OCE Mode";
            }
            if (step.action == UserAction::Focus) {
                action_mode = "Focus Action";
            } else if (step.action == UserAction::MoveZangle) {
                action_mode = "MoveZangle Action";
            } else if (step.action == UserAction::Scan) {
                action_mode = "Scanning Action";
            }
            msg_ = std::format("Step [{}/{}]: {}, {}\n", pc_.load() + 1,
                               full_scan_recipe.size(), action_mode, scan_mode);
            start = now();
            while (robot_mode_read_.load() != robot_mode_ ||
                   oct_mode_read_.load() != oct_mode_ ||
                   octa_mode_read_.load() != octa_mode_ ||
                   oce_mode_read_.load() != oce_mode_) {
                rclcpp::sleep_for(std::chrono::milliseconds(1));
                if ((now() - start).seconds() > 1.0) {
                    msg_ += "Timed out changing control mode\n";
                    RCLCPP_INFO(this->get_logger(), msg_.c_str());
                    break;
                }
                if (!full_scan_read_ || cancel_action_) {
                    full_scan_ = false;
                    return;
                }
            }
            yaw_ = step.arg;
            current_action_ = step.action;
            autofocus_ = (current_action_ == UserAction::Focus);
        } else {
            if (freedrive_) {
                current_action_ = UserAction::Freedrive;
            } else if (reset_) {
                current_action_ = UserAction::Reset;
            } else if (autofocus_) {
                current_action_ = UserAction::Focus;
            } else if (next_ || previous_ || home_) {
                current_action_ = UserAction::MoveZangle;
            }
        }

        switch (current_action_) {
        case UserAction::Freedrive:
            if (freedrive_) {
                if (previous_action_ != current_action_) {
                    sendFreedriveGoal(true);
                    circle_state_ = 1;
                    angle_ = 0.0;
                    msg_ = "[Action] Freedrive Mode ON\n";
                    RCLCPP_INFO(get_logger(), msg_.c_str());
                    previous_action_ = UserAction::Freedrive;
                }
            } else {
                sendFreedriveGoal(false);
                msg_ = "[Action] Freedrive Mode OFF\n";
                RCLCPP_INFO(get_logger(), msg_.c_str());
                current_action_ = UserAction::None;
                previous_action_ = UserAction::None;
            }
            break;
        case UserAction::Reset:
            if (previous_action_ != current_action_) {
                angle_ = 0.0;
                circle_state_ = 1;
                msg_ = "[Action] Reset to default position. It may take "
                       "some time "
                       "please wait.\n";
                RCLCPP_INFO(get_logger(), msg_.c_str());
                sendResetGoal();
                previous_action_ = UserAction::Reset;
            }
            break;
        case UserAction::Focus:
            if (autofocus_ && !end_state_) {
                if (previous_action_ != current_action_) {
                    sendFocusGoal();
                    msg_ = "[Action] Focusing\n";
                    RCLCPP_INFO(get_logger(), msg_.c_str());
                    previous_action_ = UserAction::Focus;
                }
            } else {
                if (!end_state_) {
                    msg_ = "Canceling Focus action\n";
                    end_state_ = true;
                    RCLCPP_INFO(this->get_logger(), msg_.c_str());
                    if (goal_still_active(active_focus_goal_handle_)) {
                        focus_action_client_->async_cancel_goal(
                            active_focus_goal_handle_);
                    }
                }
            }
            break;
        case UserAction::MoveZangle:
            if (previous_action_ != current_action_) {
                angle_increment_ =
                    (num_pt_ == 0)
                        ? 0.0
                        : (angle_limit_ / static_cast<double>(num_pt_));
                if (next_) {
                    yaw_ = angle_increment_;
                    msg_ = std::format("[Action] Next: {}\n", yaw_);
                } else if (previous_) {
                    yaw_ = -angle_increment_;
                    msg_ = std::format("[Action] Previous: {}\n", yaw_);
                } else if (home_) {
                    yaw_ = -angle_;
                    msg_ = std::format("[Action] Home: {}\n", yaw_);
                }
                RCLCPP_INFO(get_logger(), msg_.c_str());
                sendMoveZAngleGoal(yaw_);
                if (std::abs(angle_.load()) < 1e-6) {
                    circle_state_ = 1;
                }
                current_action_ = UserAction::None;
                previous_action_ = UserAction::MoveZangle;
            }
            break;
        case UserAction::Scan:
            if (previous_action_ != current_action_) {
                if (scan_state_ == ScanState::IDLE) {
                    msg_ += std::format("  [Action] Scanning\n");
                    RCLCPP_INFO(get_logger(), msg_.c_str());
                    trigger_scan();
                    scan_trigger_store_ = scan_trigger_read_.load();
                    current_action_ = UserAction::None;
                    previous_action_ = UserAction::Scan;
                }
            } else {
                if (scan_state_ == ScanState::IDLE) {
                    previous_action_ = UserAction::None;
                    pc_.fetch_add(1);
                }
            }
            break;
        default:
            robot_mode_ = robot_mode_read_.load();
            oct_mode_ = oct_mode_read_.load();
            octa_mode_ = octa_mode_read_.load();
            oce_mode_ = oce_mode_read_.load();
            scan_3d_ = false;
            triggered_service_ = false;
            if (end_state_ && !autofocus_) {
                end_state_ = false;
            }
            break;
        }
    }

    void sendFocusGoal() {
        FocusAction::Goal goal_msg;
        goal_msg.angle_tolerance = angle_tolerance_;
        goal_msg.z_tolerance = z_tolerance_;
        goal_msg.z_height = z_height_;

        auto options = rclcpp_action::Client<FocusAction>::SendGoalOptions();

        options.feedback_callback =
            [this](FocusGoalHandle::SharedPtr,
                   const std::shared_ptr<const FocusAction::Feedback> fb) {
                msg_ += fb->debug_msgs;
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
            };

        options.result_callback =
            [this](const FocusGoalHandle::WrappedResult &result) {
                current_action_ = UserAction::None;
                previous_action_ = UserAction::None;
                msg_ += result.result->status;
                end_state_ = true;
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Focus action SUCCEEDED");
                    if (full_scan_read_) {
                        pc_.fetch_add(1);
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Focus action ABORTED");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Focus action CANCELED");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(),
                                "Focus action UNKNOWN result code");
                    break;
                }
                active_focus_goal_handle_.reset();
            };

        options.goal_response_callback =
            [this](FocusGoalHandle::SharedPtr goal_handle) {
                active_focus_goal_handle_ = goal_handle;
                if (!active_focus_goal_handle_) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Focus goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(),
                                "Focus goal accepted; waiting for result");
                }
            };

        focus_action_client_->async_send_goal(goal_msg, options);
    }

    void sendMoveZAngleGoal(double yaw) {
        MoveZAngle::Goal goal_msg;
        goal_msg.target_angle = yaw;
        goal_msg.radius = radius_.load();
        goal_msg.angle = angle_.load();

        auto options = rclcpp_action::Client<MoveZAngle>::SendGoalOptions();

        options.feedback_callback =
            [this](MoveZGoalHandle::SharedPtr,
                   const std::shared_ptr<const MoveZAngle::Feedback> fb) {
                msg_ += fb->debug_msgs;
                RCLCPP_INFO(this->get_logger(),
                            "MoveZAngle feedback => target_angle_z=%.2f",
                            fb->current_z_angle);
            };

        options.result_callback =
            [this, yaw](const MoveZGoalHandle::WrappedResult &result) {
                current_action_ = UserAction::None;
                previous_action_ = UserAction::None;
                msg_ += result.result->status;
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    if (yaw > 0.0) {
                        circle_state_++;
                    } else {
                        circle_state_--;
                    }
                    angle_.fetch_add(yaw);
                    RCLCPP_INFO(this->get_logger(), "MoveZAngle SUCCEEDED");
                    if (full_scan_read_) {
                        pc_.fetch_add(1);
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "MoveZAngle ABORTED");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "MoveZAngle CANCELED");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "MoveZAngle UNKNOWN code");
                    break;
                }
                active_move_z_goal_handle_.reset();
            };

        options.goal_response_callback =
            [this](MoveZGoalHandle::SharedPtr goal_handle) {
                active_move_z_goal_handle_ = goal_handle;
                if (!active_move_z_goal_handle_) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Move Z Angle goal was rejected by server");
                } else {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Move Z Angle goal accepted; waiting for result");
                }
            };

        move_z_angle_action_client_->async_send_goal(goal_msg, options);
    }

    void sendFreedriveGoal(bool enable) {
        Freedrive::Goal goal_msg;
        goal_msg.enable = enable;

        auto options = rclcpp_action::Client<Freedrive>::SendGoalOptions();

        options.feedback_callback =
            [this](FreedriveGoalHandle::SharedPtr,
                   const std::shared_ptr<const Freedrive::Feedback> fb) {
                msg_ += fb->debug_msgs;
                RCLCPP_INFO(this->get_logger(), "Freedrive feedback => %s",
                            fb->debug_msgs.c_str());
            };

        options.result_callback =
            [this](const FreedriveGoalHandle::WrappedResult &result) {
                msg_ += result.result->status;
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Freedrive SUCCESS");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Freedrive ABORTED");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Freedrive CANCELED");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "Freedrive UNKNOWN code");
                    break;
                }
                active_freedrive_goal_handle_.reset();
            };

        options.goal_response_callback =
            [this](FreedriveGoalHandle::SharedPtr goal_handle) {
                active_freedrive_goal_handle_ = goal_handle;
                if (!active_freedrive_goal_handle_) {
                    RCLCPP_ERROR(this->get_logger(),
                                 " Freedrive goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(),
                                " Freedrive goal accepted; waiting for result");
                }
            };

        freedrive_action_client_->async_send_goal(goal_msg, options);
    }

    void sendResetGoal() {
        Reset::Goal goal_msg;
        goal_msg.reset = true;

        auto options = rclcpp_action::Client<Reset>::SendGoalOptions();

        options.feedback_callback =
            [this](ResetGoalHandle::SharedPtr,
                   const std::shared_ptr<const Reset::Feedback> fb) {
                msg_ += fb->debug_msgs;
                RCLCPP_INFO(this->get_logger(), "Reset feedback => %s",
                            fb->debug_msgs.c_str());
            };

        options.result_callback =
            [this](const ResetGoalHandle::WrappedResult &result) {
                current_action_ = UserAction::None;
                previous_action_ = UserAction::None;
                msg_ += result.result->status;
                // trigger_apply_config();
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    if (call_capture_background()) {
                        msg_ += "\nBackground Captured\n";
                    }
                    RCLCPP_INFO(this->get_logger(), "Reset SUCCESS");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    msg_ += "\nReset position abort\n";
                    RCLCPP_WARN(this->get_logger(), "Reset ABORTED");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    msg_ += "\nReset position canceled\n";
                    RCLCPP_WARN(this->get_logger(), "Reset CANCELED");
                    break;
                default:
                    msg_ += "\nReset position unknown code\n";
                    RCLCPP_WARN(this->get_logger(), "Reset UNKNOWN code");
                    break;
                }
                active_reset_goal_handle_.reset();
            };

        options.goal_response_callback =
            [this](ResetGoalHandle::SharedPtr goal_handle) {
                active_reset_goal_handle_ = goal_handle;
                if (!active_reset_goal_handle_) {
                    RCLCPP_ERROR(this->get_logger(),
                                 " Reset goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(),
                                " Reset goal accepted; waiting for result");
                }
            };

        reset_action_client_->async_send_goal(goal_msg, options);
    }

    void scan3dCallback(const std::shared_ptr<Scan3d::Request> request,
                        std::shared_ptr<Scan3d::Response> response) {
        if (!triggered_service_) {
            scan_3d_ = request->activate;
            triggered_service_ = true;
        }
        if (request->activate) {
            if (scan_3d_read_) {
                trigger_apply_config();
                // wait for scan to actually trigger
                rclcpp::sleep_for(std::chrono::milliseconds(50));
                response->success = true;
                triggered_service_ = false;
            } else {
                response->success = false;
            }
        } else {
            if (!scan_3d_read_) {
                trigger_apply_config();
                response->success = true;
                triggered_service_ = false;
            } else {
                response->success = false;
            }
        }
    }

    void deactivateFocusCallback(
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>
            request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!triggered_service_) {
            end_state_ = true;
            triggered_service_ = true;
            trigger_apply_config();
        }
        if (!autofocus_) {
            end_state_ = false;
            triggered_service_ = false;
            response->success = true;
        } else {
            response->success = false;
        }
    }

    bool call_capture_background() {
        if (!service_capture_background_->wait_for_service(
                std::chrono::milliseconds(200)))
            return false;

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto fut = service_capture_background_->async_send_request(req);
        return fut.wait_for(std::chrono::milliseconds(1000)) ==
                   std::future_status::ready &&
               fut.get()->success;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinatorNode>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
