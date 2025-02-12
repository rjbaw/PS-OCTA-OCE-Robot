#ifndef RECONNECT_NODE_HPP
#define RECONNECT_NODE_HPP

// services
// /dashboard_client/add_to_log /dashboard_client/brake_release
// /dashboard_client/clear_operational_mode /dashboard_client/close_popup
// /dashboard_client/close_safety_popup /dashboard_client/connect
// /dashboard_client/describe_parameters /dashboard_client/get_loaded_program
// /dashboard_client/get_parameter_types /dashboard_client/get_parameters
// /dashboard_client/get_robot_mode /dashboard_client/get_safety_mode
// /dashboard_client/get_type_description /dashboard_client/list_parameters
// /dashboard_client/load_installation /dashboard_client/load_program
// /dashboard_client/pause /dashboard_client/play /dashboard_client/popup
// /dashboard_client/power_off /dashboard_client/power_on
// /dashboard_client/program_running /dashboard_client/program_saved
// /dashboard_client/program_state /dashboard_client/quit
// /dashboard_client/raw_request /dashboard_client/restart_safety
// /dashboard_client/set_parameters /dashboard_client/set_parameters_atomically
// /dashboard_client/shutdown /dashboard_client/stop

#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_dashboard_msgs/srv/get_loaded_program.hpp>
#include <ur_dashboard_msgs/srv/get_program_state.hpp>
#include <ur_dashboard_msgs/srv/get_robot_mode.hpp>
#include <ur_dashboard_msgs/srv/get_safety_mode.hpp>
#include <ur_dashboard_msgs/srv/is_program_running.hpp>

static constexpr int8_t NO_CONTROLLER = -1;
static constexpr int8_t DISCONNECTED = 0;
static constexpr int8_t CONFIRM_SAFETY = 1;
static constexpr int8_t BOOTING = 2;
static constexpr int8_t POWER_OFF = 3;
static constexpr int8_t POWER_ON = 4;
static constexpr int8_t IDLE = 5;
static constexpr int8_t BACKDRIVE = 6;
static constexpr int8_t RUNNING = 7;
static constexpr int8_t UPDATING_FIRMWARE = 8;

static constexpr uint8_t SAFETY_MODE_NORMAL = 1;
static constexpr uint8_t REDUCED = 2;
static constexpr uint8_t PROTECTIVE_STOP = 3;
static constexpr uint8_t RECOVERY = 4;
static constexpr uint8_t SAFEGUARD_STOP = 5;
static constexpr uint8_t SYSTEM_EMERGENCY_STOP = 6;
static constexpr uint8_t ROBOT_EMERGENCY_STOP = 7;
static constexpr uint8_t VIOLATION = 8;
static constexpr uint8_t FAULT = 9;
static constexpr uint8_t VALIDATE_JOINT_ID = 10;
static constexpr uint8_t UNDEFINED_SAFETY_MODE = 11;
static constexpr uint8_t AUTOMATIC_MODE_SAFEGUARD_STOP = 12;
static constexpr uint8_t SYSTEM_THREE_POSITION_ENABLING_STOP = 13;

class ReconnectClient : public rclcpp::Node {
  public:
    ReconnectClient();

  private:
    void timerCallback();
    bool callTriggerService(
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
        const std::string &service_name);

    rclcpp::Client<ur_dashboard_msgs::srv::GetRobotMode>::SharedPtr
        get_robot_mode_client_;
    rclcpp::Client<ur_dashboard_msgs::srv::GetSafetyMode>::SharedPtr
        get_safety_mode_client_;
    rclcpp::Client<ur_dashboard_msgs::srv::GetProgramState>::SharedPtr
        get_program_state_client_;
    rclcpp::Client<ur_dashboard_msgs::srv::GetLoadedProgram>::SharedPtr
        get_loaded_program_client_;
    rclcpp::Client<ur_dashboard_msgs::srv::IsProgramRunning>::SharedPtr
        running_program_client_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr connect_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resend_program_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr restart_safety_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr brake_release_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool executed_ = false;
};

#endif // RECONNECT_NODE_HPP
