/**
 * @file coordinator_node.hpp
 */

#ifndef COORDINATOR_NODE_HPP
#define COORDINATOR_NODE_HPP

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <sstream>

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

enum class UserAction : uint8_t {
  None,
  Freedrive,
  Reset,
  MoveZangle,
  Focus,
  Scan,
};

enum class Mode : uint8_t {
  ROBOT,
  OCT,
  OCTA,
  OCE,
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
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void init();

private:
  // Action clients
  rclcpp_action::Client<FocusAction>::SharedPtr focus_action_client_;
  rclcpp_action::Client<MoveZAngle>::SharedPtr move_z_angle_action_client_;
  rclcpp_action::Client<Freedrive>::SharedPtr freedrive_action_client_;
  rclcpp_action::Client<Reset>::SharedPtr reset_action_client_;

  // Services
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_capture_background_;
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
  int pub_period_ms_ = 5;
  int main_loop_period_ms_ = 5;
  int action_server_wait_ms_ = 200;
  int config_apply_ms_ = 50;
  double scan_trigger_timeout_sec_ = 2.0;
  int capture_service_wait_ms_ = 200;
  int capture_response_timeout_ms_ = 1000;
  int scan3d_window_ms_ = 50;
  int service_poll_interval_ms_ = 1;

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

  void subscriber_callback(const octa_ros::msg::Labviewdata::SharedPtr msg);
  void cancel_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void publisher_callback();
  void main_loop();
  void send_focus_goal();
  void send_move_z_angle_goal(double yaw);
  void send_freedrive_goal(bool enable);
  void send_reset_goal();
  void scan3d_callback(const std::shared_ptr<Scan3d::Request> request,
                       std::shared_ptr<Scan3d::Response> response);
  bool call_capture_background();
  };
  
#endif  // COORDINATOR_NODE_HPP
