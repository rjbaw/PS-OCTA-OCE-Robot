/**
 * @file freedrive_node.hpp
 * @author rjbaw
 * @brief Action server to toggle freedrive/manual guidance mode.
 *
 * Switches controllers to enable gravity-compensated manual guidance, publishes
 * a keepalive signal while enabled, and exposes a simple action interface to
 * toggle the mode.
 *
 * @par Parameters
 * - motion_controller (string, default: "scaled_joint_trajectory_controller"):
 *   primary motion controller name.
 * - freedrive_controller (string, default: "freedrive_mode_controller"):
 *   freedrive controller name.
 * - keepalive_rate (double, default: 5.0 Hz): publish rate for keepalive.
 * - switch_timeout (double, default: 3.0 s): controller switch timeout.
 * - dry_run (bool, default: false): do not create controller_manager client;
 *   pretend switches succeed (useful for dev without hardware).
 *
 * @par Publishers
 * - `std_msgs::msg::Bool` on
 *   `/freedrive_mode_controller/enable_freedrive_mode` (QoS: reliable).
 *
 * @par Services
 * - Client: `controller_manager/switch_controller` (unless dry_run=true).
 *
 * @par Action Server
 * - Name: `freedrive_action`; Goal: `enable` (bool).
 *
 * @ingroup actions
 */

#ifndef FREEDRIVE_NODE_HPP
#define FREEDRIVE_NODE_HPP

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_msgs/msg/bool.hpp>

#include <octa_ros/action/freedrive.hpp>

/**
 * @brief Action server that enables/disables freedrive (gravity compensation).
 *
 * Switches controllers, publishes keepalive signals, and exposes a simple
 * action interface to transition into and out of freedrive mode.
 */
class FreedriveActionServer : public rclcpp::Node {
  public:
    using Freedrive = octa_ros::action::Freedrive;
    using GoalHandleFreedrive = rclcpp_action::ServerGoalHandle<Freedrive>;
    using SwitchSrv = controller_manager_msgs::srv::SwitchController;

    /**
     * @brief Construct the node and setup interfaces.
     */
    explicit FreedriveActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    rclcpp_action::Server<Freedrive>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleFreedrive> active_goal_handle_;
    rclcpp::Client<SwitchSrv>::SharedPtr switch_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr freedrive_pub_;
    rclcpp::TimerBase::SharedPtr keepalive_timer_;

    /** @brief Action callbacks for goal lifecycle and execution. */
    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] rclcpp_action::GoalUUID goal_id,
                std::shared_ptr<const Freedrive::Goal> goal);

    rclcpp_action::CancelResponse
    handle_cancel(std::shared_ptr<GoalHandleFreedrive> goal_handle);

    void handle_accepted(std::shared_ptr<GoalHandleFreedrive> goal_handle);

    void execute(std::shared_ptr<GoalHandleFreedrive> goal_handle);

    /** @brief Periodic keepalive toggling while in freedrive mode. */
    void start_keepalive();
    void stop_keepalive();

    /** @brief Publish a boolean freedrive state. */
    void publish_bool(bool value);

    /** @brief Switch between freedrive and standard controllers. */
    bool switch_to_freedrive_controller(bool to_freedrive);
};

#endif // FREEDRIVE_NODE_HPP
