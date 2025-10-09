/**
 * @file coordinator_node.hpp
 * @author rjbaw
 * @brief Main coordinator node orchestrating actions, pub/sub, and services.
 *
 * Coordinates high-level user actions across motion (MoveIt), OCT/OCE
 * acquisition, and LabVIEW I/O. Maintains a pub/sub loop for the GUI, drives
 * the action clients for motion primitives, and exposes a Scan3d service for
 * acquisition windows.
 *
 * @par Parameters
 * - topic_robot_data (string, default: "robot_data"): publish topic for
 *   `octa_ros::msg::Robotdata`.
 * - topic_labview_data (string, default: "labview_data"): subscribe topic for
 *   `octa_ros::msg::Labviewdata`.
 * - topic_cancel (string, default: "cancel_current_action"): subscribe topic
 *   for cancel requests (`std_msgs::msg::Bool`).
 * - srv_scan3d (string, default: "scan_3d"): service name for `Scan3d`.
 * - action_focus_name (string, default: "focus_action"): Focus action server.
 * - action_movez_name (string, default: "move_z_angle_action"): MoveZAngle
 *   action server.
 * - action_freedrive_name (string, default: "freedrive_action"): Freedrive
 *   action server.
 * - action_reset_name (string, default: "reset_action"): Reset action server.
 * - pub_period_ms (int, default: 5 ms): publisher timer period.
 * - main_loop_period_ms (int, default: 5 ms): state machine tick period.
 * - action_server_wait_ms (int, default: 200 ms): wait for action servers.
 * - config_apply_ms (int, default: 50 ms): debounce time before applying mode
 *   changes.
 * - scan_trigger_timeout_sec (double, default: 2.0 s): timeout for scan
 *   triggers.
 * - scan3d_window_ms (int, default: 50 ms): duration of scan window.
 * - service_poll_interval_ms (int, default: 1 ms): poll interval for services.
 *
 * @par Publishers
 * - `octa_ros::msg::Robotdata` on `topic_robot_data` (QoS: reliable).
 *
 * @par Subscribers
 * - `octa_ros::msg::Labviewdata` on `topic_labview_data` (QoS: reliable).
 * - `std_msgs::msg::Bool` on `topic_cancel` (QoS: reliable) – cancels current
 *   action.
 *
 * @par Services (clients/servers)
 * - Server: `Scan3d` on `srv_scan3d` (QoS: reliable).
 *
 * @par Action Clients
 * - Focus, MoveZAngle, Freedrive, Reset (names via parameters above).
 *
 * @note Units: lengths are meters unless noted; angles are degrees in GUI
 * messages but radians internally for planning; times are ms or seconds as
 * specified in parameter names.
 *
 * @ingroup coordinator
 */

#ifndef COORDINATOR_NODE_HPP
#define COORDINATOR_NODE_HPP

#include <atomic>
#include <chrono>
#include <mutex>
#include <sstream>
#include <string>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <action_msgs/msg/goal_status.hpp>
#include <octa_ros/action/focus.hpp>
#include <octa_ros/action/freedrive.hpp>
#include <octa_ros/action/move_z_angle.hpp>
#include <octa_ros/action/reset.hpp>
#include <octa_ros/msg/labviewdata.hpp>
#include <octa_ros/msg/robotdata.hpp>
#include <octa_ros/srv/scan3d.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

/**
 * @brief High-level user intent states handled by the coordinator.
 */
enum class UserAction : uint8_t {
    None,       ///< Idle / no action selected.
    Freedrive,  ///< Robot freedrive mode.
    Reset,      ///< Return to default position.
    MoveZangle, ///< Execute Z/yaw repositioning.
    Focus,      ///< Run focus alignment routine.
    Scan,       ///< LabVIEW acquisition trigger.
};

/**
 * @brief System operating modes.
 */
enum class Mode : uint8_t {
    ROBOT, ///< LabVIEW Robot mode.
    OCT,   ///< LabVIEW OCT mode.
    OCTA,  ///< LabVIEW OCT mode.
    OCE,   ///< LabVIEW OCE mode.
};

/**
 * @brief Central node coordinating user actions, motion, and OCT/OCE tasks.
 *
 * Provides action clients to specialized servers (freedrive, reset, focus,
 * move_z_angle), exposes a Scan3d service, and maintains a high-rate pub/sub
 * loop to exchange robot and LabVIEW state.
 */
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

    /** @brief Construct the node; call init() to create interfaces. */
    explicit CoordinatorNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /** @brief Create publishers, subscribers, timers, and action/service
     * clients. */
    void init();

  private:
    // Action clients
    rclcpp_action::Client<FocusAction>::SharedPtr focus_action_client_;
    rclcpp_action::Client<MoveZAngle>::SharedPtr move_z_angle_action_client_;
    rclcpp_action::Client<Freedrive>::SharedPtr freedrive_action_client_;
    rclcpp_action::Client<Reset>::SharedPtr reset_action_client_;

    // Services
    rclcpp::Service<Scan3d>::SharedPtr scan_3d_srv_;

    // MoveIt
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    moveit::planning_interface::PlanningSceneInterface psi;

    // Execution groups & timers
    rclcpp::CallbackGroup::SharedPtr parallel_group_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr main_loop_timer_;
    rclcpp::TimerBase::SharedPtr config_timer_;
    std::weak_ptr<rclcpp::TimerBase> config_timer_weak_;
    rclcpp::TimerBase::SharedPtr scan_timer_;
    std::weak_ptr<rclcpp::TimerBase> scan_timer_weak_;

    // Tunable parameters
    int64_t pub_period_ms_ = 5;
    int64_t main_loop_period_ms_ = 5;
    int64_t action_server_wait_ms_ = 200;
    int64_t config_apply_ms_ = 60;
    double scan_trigger_timeout_sec_ = 2.0;
    int64_t scan3d_window_ms_ = 50;
    int64_t service_poll_interval_ms_ = 1;
    static constexpr std::chrono::milliseconds kFullScanOffDelay{3000};

    // Parameter callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        param_cb_handle_;

    // Pub/Sub
    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr pub_handle_;
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr sub_handle_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_handle_;

    // Active goals
    FocusGoalHandle::SharedPtr active_focus_goal_handle_;
    MoveZGoalHandle::SharedPtr active_move_z_goal_handle_;
    FreedriveGoalHandle::SharedPtr active_freedrive_goal_handle_;
    ResetGoalHandle::SharedPtr active_reset_goal_handle_;

    // Internal state
    std::atomic<UserAction> current_action_ = UserAction::None;
    std::atomic<UserAction> previous_action_ = UserAction::None;
    octa_ros::msg::Labviewdata old_sub_msg_;
    octa_ros::msg::Robotdata old_pub_msg_;
    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
    double angle_increment_ = 0.0;
    std::mutex data_mutex_;
    bool scan_trigger_store_ = false;
    std::atomic<bool> success_ = false;
    std::atomic<unsigned int> pc_ = 0;
    rclcpp::Time scan_start;
    bool full_scan_false_timer_active_ = false;
    std::chrono::steady_clock::time_point full_scan_false_since_;

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

    void trigger_apply_config();

    template <typename GH> bool goal_still_active(const GH &handle) {
        if (!handle) {
            return false;
        }
        auto status = handle->get_status();
        return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
               status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    }

    template <typename T>
    void log_if_changed(const T &new_val, const T &old_val,
                        const std::string &name, std::ostringstream &log) {
        if (new_val != old_val) {
            log << " " << name << ": " << new_val << "\n";
        }
    }

    /** @brief Handle LabVIEW data subscription messages. */
    void subscriber_callback(octa_ros::msg::Labviewdata::SharedPtr msg);
    /** @brief Handle external cancel request. */
    void cancel_callback(std_msgs::msg::Bool::SharedPtr msg);
    /** @brief Publish Robotdata at a fixed rate. */
    void publisher_callback();
    /** @brief Core state machine tick; sends goals based on current state. */
    void main_loop();

    /** @brief Send a focus action goal if not already active. */
    void send_focus_goal();
    /** @brief Send a MoveZAngle goal with the requested yaw increment. */
    void send_move_z_angle_goal(double yaw);
    /** @brief Send a Freedrive goal to enable/disable manual guidance. */
    void send_freedrive_goal(bool enable);
    /** @brief Send a Reset goal to return to a safe posture. */
    void send_reset_goal();
    /** @brief Scan3d service handler. */
    void scan3d_callback(std::shared_ptr<Scan3d::Request> request,
                         std::shared_ptr<Scan3d::Response> response);
    /** @brief Call OCT background capture service; returns true if success. */
    bool call_capture_background();

    /**
     * @brief Apply velocity/acceleration scaling [0,1] to a planning node.
     *
     * Updates Pilz per-pipeline plan_request_params so the next plan uses the
     * requested scale.
     */
    void apply_speed_scale_to_node(const std::string &node_name,
                                   double vel_scale, double acc_scale);
};

#endif // COORDINATOR_NODE_HPP
