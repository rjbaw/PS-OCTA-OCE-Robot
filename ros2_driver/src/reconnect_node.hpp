#ifndef RECONNECT_NODE_HPP
#define RECONNECT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "ur_dashboard_msgs/action/set_mode.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/srv/get_program_state.hpp"
#include "ur_dashboard_msgs/srv/get_robot_mode.hpp"
#include <std_srvs/srv/trigger.hpp>

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

using SetMode = ur_dashboard_msgs::action::SetMode;
using GoalHandleSetMode = rclcpp_action::ClientGoalHandle<SetMode>;

class ReconnectClient : public rclcpp::Node {
  public:
    ReconnectClient();
    int8_t getCurrentMode() const;
    void send_goal(int8_t target_mode, bool stop_program = false,
                   bool play_program = false);
    void timerCallback();
  private:
    void
    feedback_callback(GoalHandleSetMode::SharedPtr,
                      const std::shared_ptr<const SetMode::Feedback> feedback);
    void result_callback(const GoalHandleSetMode::WrappedResult &result);
    rclcpp_action::Client<SetMode>::SharedPtr set_client_;
    rclcpp::Client<ur_dashboard_msgs::srv::GetRobotMode>::SharedPtr
        status_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<int8_t> current_mode_{DISCONNECTED};
};

#endif // RECONNECT_NODE_HPP
