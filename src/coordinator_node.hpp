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
 * - action_move_name (string, default: "move_action"): Move action server.
 * - action_freedrive_name (string, default: "freedrive_action"): Freedrive
 *   action server.
 * - action_reset_name (string, default: "reset_action"): Reset action server.
 * - config_apply_ms (int, default: 60 ms): debounce time before applying mode
 *   changes.
 * - scan_trigger_timeout_sec (double, default: 10.0 s): timeout for scan
 *   triggers.
 * - scan3d_window_ms (int, default: 50 ms): post-acknowledgement delay before
 *   Scan3D activation succeeds.
 *
 * @note Coordinator parameters are read during initialization. Restart the
 * node after changing them.
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
 * - Focus, Move, Freedrive, Reset (names via parameters above).
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
#include <cstdint>
#include <sstream>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <action_msgs/msg/goal_status.hpp>
#include <octa_ros/action/focus.hpp>
#include <octa_ros/action/freedrive.hpp>
#include <octa_ros/action/move.hpp>
#include <octa_ros/action/reset.hpp>
#include <octa_ros/msg/labviewdata.hpp>
#include <octa_ros/msg/robotdata.hpp>
#include <octa_ros/srv/scan3d.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

namespace octa_ros {

/**
 * @brief Convert LabVIEW's Fullscan level into start and cancel events.
 *
 * A HIGH level starts immediately while the latch is armed. During an active
 * run, a LOW level starts a grace period so LabVIEW's temporary OCT-A
 * completion dropout does not stop the recipe. If LOW remains present for the
 * complete grace period, poll() emits one PersistentLow event and ends the
 * active run.
 *
 * finish() is for completion or explicit cancellation. It disarms the latch,
 * so a held HIGH (or recovery HIGH after an OCT-A dropout) cannot start another
 * run. A genuine HIGH-to-LOW edge after finish() rearms it.
 */
class FullScanRequestLatch {
  public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    enum class Event : std::uint8_t {
        None,
        StartRequested,
        PersistentLow,
    };

    explicit FullScanRequestLatch(Clock::duration low_grace_duration)
        : low_grace_duration_(low_grace_duration) {}

    /**
     * @brief Observe the latest Fullscan level from LabVIEW.
     *
     * @return StartRequested exactly once when an armed latch observes HIGH.
     *         Falling and recovery edges are handled without an immediate
     *         event; poll() emits PersistentLow if the grace period expires.
     */
    [[nodiscard]] Event observe(bool level, TimePoint now) noexcept {
        const bool previous_level = input_level_;
        input_level_ = level;

        if (!active_) {
            if (!level) {
                if (previous_level) {
                    armed_ = true;
                }
                return Event::None;
            }
            if (!armed_) {
                return Event::None;
            }

            active_ = true;
            armed_ = false;
            low_grace_active_ = false;
            return Event::StartRequested;
        }

        if (level) {
            low_grace_active_ = false;
            return Event::None;
        }

        if (!low_grace_active_) {
            low_grace_active_ = true;
            low_since_ = now;
        }
        return Event::None;
    }

    /**
     * @brief Check the active LOW grace period from a wall timer.
     *
     * @return PersistentLow exactly once when LOW has remained present for the
     *         configured grace duration. The event ends the active run and the
     *         sustained LOW rearms the latch for a future HIGH request.
     */
    [[nodiscard]] Event poll(TimePoint now) noexcept {
        if (!active_ || !low_grace_active_ || input_level_ ||
            now - low_since_ < low_grace_duration_) {
            return Event::None;
        }

        active_ = false;
        low_grace_active_ = false;
        armed_ = true;
        return Event::PersistentLow;
    }

    /**
     * @brief End a completed or explicitly cancelled run and disarm.
     *
     * A subsequent HIGH-to-LOW edge is required before another HIGH can start.
     */
    void finish() noexcept {
        active_ = false;
        low_grace_active_ = false;
        armed_ = false;
    }

    [[nodiscard]] bool active() const noexcept { return active_; }

  private:
    Clock::duration low_grace_duration_;
    TimePoint low_since_;
    bool input_level_ = false;
    bool low_grace_active_ = false;
    bool armed_ = true;
    bool active_ = false;
};

} // namespace octa_ros

/**
 * @brief High-level user intent states handled by the coordinator.
 */
enum class UserAction : std::uint8_t {
    None,      ///< Idle / no action selected.
    Freedrive, ///< Robot freedrive mode.
    Reset,     ///< Return to default position.
    Move,      ///< Execute repositioning.
    Focus,     ///< Run focus alignment routine.
    Scan,      ///< LabVIEW acquisition trigger.
};

/** @brief Last controller status reported by the Freedrive action server. */
enum class FreedriveStatus : std::uint8_t {
    Off,
    On,
    Unconfirmed,
};

/**
 * @brief System operating modes.
 */
enum class Mode : std::uint8_t {
    ROBOT, ///< LabVIEW Robot mode.
    OCT,   ///< LabVIEW OCT mode.
    OCTA,  ///< LabVIEW OCT-A mode.
    OCE,   ///< LabVIEW OCE mode.
};

struct Step {
    UserAction action;
    Mode mode;
    double yaw = 0.0;
    double x = 0.0;
    double y = 0.0;
};

/**
 * @brief Central node coordinating user actions, motion, and OCT/OCE tasks.
 *
 * Provides action clients to specialized servers (freedrive, reset, focus,
 * move), exposes a Scan3d service, and maintains a high-rate pub/sub loop to
 * exchange robot and LabVIEW state.
 */
class CoordinatorNode : public rclcpp::Node {
  public:
    using FocusAction = octa_ros::action::Focus;
    using Move = octa_ros::action::Move;
    using Freedrive = octa_ros::action::Freedrive;
    using Reset = octa_ros::action::Reset;

    using Scan3d = octa_ros::srv::Scan3d;

    using FocusGoalHandle = rclcpp_action::ClientGoalHandle<FocusAction>;
    using MoveGoalHandle = rclcpp_action::ClientGoalHandle<Move>;
    using FreedriveGoalHandle = rclcpp_action::ClientGoalHandle<Freedrive>;
    using ResetGoalHandle = rclcpp_action::ClientGoalHandle<Reset>;

    /** @brief Construct the node; call init() to create interfaces. */
    explicit CoordinatorNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /** @brief Create publishers, subscribers, timers, and action/service
     * clients. */
    void init();
    std::vector<Step> build_full_scan_recipe() const;

  private:
    // Action clients
    rclcpp_action::Client<FocusAction>::SharedPtr focus_action_client_;
    rclcpp_action::Client<Move>::SharedPtr move_action_client_;
    rclcpp_action::Client<Freedrive>::SharedPtr freedrive_action_client_;
    rclcpp_action::Client<Reset>::SharedPtr reset_action_client_;

    // Services
    rclcpp::Service<Scan3d>::SharedPtr scan_3d_srv_;

    // MoveIt
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    moveit::planning_interface::PlanningSceneInterface psi;

    // Execution groups & timers
    rclcpp::CallbackGroup::SharedPtr coordinator_group_;
    rclcpp::CallbackGroup::SharedPtr service_response_group_;
    rclcpp::CallbackGroup::SharedPtr scan3d_service_group_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr main_loop_timer_;
    rclcpp::TimerBase::SharedPtr config_timer_;
    rclcpp::TimerBase::SharedPtr full_scan_latch_timer_;
    std::weak_ptr<rclcpp::TimerBase> config_timer_weak_;

    int64_t pub_period_ms_ = 5;
    int64_t main_loop_period_ms_ = 5;
    int64_t action_server_wait_ms_ = 200;
    int64_t config_apply_ms_ = 60;
    double scan_trigger_timeout_sec_ = 10.0;
    int64_t scan3d_window_ms_ = 50;
    static constexpr std::chrono::milliseconds kFullScanOffGrace{1000};
    static constexpr std::chrono::milliseconds kFullScanLatchPollPeriod{10};
    static constexpr std::chrono::milliseconds kFullScanPreviewSettleDelay{350};
    static constexpr std::chrono::seconds kActionCancelTimeout{2};
    static constexpr std::chrono::seconds kFreedriveCancelTimeout{6};
    static constexpr std::chrono::seconds kFreedriveRetryInterval{1};
    static constexpr std::chrono::seconds kResetActionTimeout{40};

    // Pub/Sub
    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr pub_handle_;
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr sub_handle_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_handle_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ur_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ur_health_sub_;

    // Active goals
    FocusGoalHandle::SharedPtr active_focus_goal_handle_;
    MoveGoalHandle::SharedPtr active_move_handle_;
    FreedriveGoalHandle::SharedPtr active_freedrive_goal_handle_;
    ResetGoalHandle::SharedPtr active_reset_goal_handle_;
    std::atomic<UserAction> active_action_ = UserAction::None;
    FreedriveStatus freedrive_status_ = FreedriveStatus::Unconfirmed;

    // Internal state
    octa_ros::msg::Labviewdata old_sub_msg_;
    octa_ros::msg::Robotdata old_pub_msg_;
    std::vector<Step> full_scan_recipe_;
    double yaw_ = 0.0;
    double angle_increment_ = 0.0;
    bool scan_trigger_before_request_ = false;
    bool waiting_for_scan_completion_ = false;
    std::atomic<bool> success_ = false;
    std::atomic<unsigned int> pc_ = 0;
    rclcpp::Time scan_start;
    octa_ros::FullScanRequestLatch full_scan_request_{kFullScanOffGrace};
    // Cancel stops this source so callbacks from the interrupted action cannot
    // change coordinator state after cancellation.
    std::stop_source action_callback_stop_;
    std::chrono::steady_clock::time_point action_wait_started_at_;
    std::chrono::steady_clock::time_point labview_mode_changed_at_;
    bool action_toggle_release_required_ = false;

    // Service variables
    std::atomic<bool> cancel_in_progress_ = false;
    bool cancel_button_pressed_ = false;

    // Publisher fields
    std::string msg_ = "idle";
    std::string ur_status_msg_;
    std::atomic<bool> ur_driver_healthy_{true};
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
    std::atomic<double> angle_init_ = 0.0;
    std::atomic<double> offset_x_ = 0.0;
    std::atomic<double> offset_y_ = 0.0;
    std::atomic<bool> autofocus_ = false;
    bool labview_freedrive_ = false;
    std::atomic<bool> previous_ = false;
    std::atomic<bool> next_ = false;
    std::atomic<bool> home_ = false;
    std::atomic<bool> reset_ = false;
    std::atomic<bool> apply_offset_ = false;
    std::atomic<bool> scan_trigger_read_ = false;
    std::atomic<bool> scan_3d_read_ = false;
    std::atomic<bool> full_scan_ = false;
    std::atomic<bool> full_scan_read_ = false;
    std::atomic<int> num_pt_ = 1;
    std::atomic<int> n_oct_ = 3;
    std::atomic<bool> apply_octa_ = true;
    std::atomic<bool> robot_mode_read_ = true;
    std::atomic<bool> oct_mode_read_ = false;
    std::atomic<bool> octa_mode_read_ = false;
    std::atomic<bool> oce_mode_read_ = false;

    void trigger_apply_config();
    void
    handle_full_scan_request_event(octa_ros::FullScanRequestLatch::Event event);
    void request_cancel(bool disarm_full_scan_latch);
    void continue_cancel();
    void finish_full_scan_request(bool disarm_latch = true);
    [[nodiscard]] bool action_in_flight(UserAction action) const noexcept;
    [[nodiscard]] bool any_action_in_flight() const noexcept;
    [[nodiscard]] bool freedrive_safely_off() const noexcept;
    [[nodiscard]] bool block_action_request(std::string_view action_name);

    template <typename GH> bool goal_can_be_canceled(const GH &handle) {
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

    /** @brief Send a focus action goal if its server is ready. */
    [[nodiscard]] bool send_focus_goal();
    /** @brief Send a Move goal.
     *
     * @param yaw Target yaw increment [deg].
     * @param offset_x Translation in X [mm].
     * @param offset_y Translation in Y [mm].
     * @param apply_offset If true, use XY translation.
     */
    [[nodiscard]] bool send_move_goal(double yaw, double offset_x,
                                      double offset_y, bool apply_offset);
    /** @brief Send a Freedrive goal to enable/disable manual guidance. */
    void send_freedrive_goal(bool enable);
    /** @brief Send a Reset goal to return to a safe posture. */
    [[nodiscard]] bool send_reset_goal();
    /** @brief Scan3d service handler. */
    void scan3d_callback(std::shared_ptr<Scan3d::Request> request,
                         std::shared_ptr<Scan3d::Response> response);

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
