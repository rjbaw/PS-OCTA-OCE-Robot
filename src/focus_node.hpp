/**
 * @file focus_node.hpp
 * @author rjbaw
 * @brief Node that focuses robot end effector to the normal of the target
 */

#ifndef FOCUS_NODE_HPP
#define FOCUS_NODE_HPP

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <future>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <octa_ros/action/focus.hpp>
#include <octa_ros/msg/img.hpp>
#include <octa_ros/srv/scan3d.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "utils.hpp"

class FocusActionServer : public rclcpp::Node {
    using Focus = octa_ros::action::Focus;
    using GoalHandleFocus = rclcpp_action::ServerGoalHandle<Focus>;
    using Scan3d = octa_ros::srv::Scan3d;

  public:
    explicit FocusActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void init();

  private:
    bool early_terminate_ = false;
    bool skip_angle_tolerance_ = true;
    const int interval_ = 6;

    rclcpp_action::Server<Focus>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleFocus> active_goal_handle_;

    moveit_cpp::MoveItCppPtr moveit_cpp_;
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
    std::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager>
        tem_;

    rclcpp::Time last_store_time_;

    static constexpr size_t kBufferSize = 8; // must be powers of 2
    std::array<cv::Mat, kBufferSize> buffer_;
    std::atomic<size_t> head_ = 0;
    std::atomic<size_t> tail_ = 0;
    rclcpp::Subscription<octa_ros::msg::Img>::SharedPtr img_subscriber_;

    std::vector<Eigen::Vector3d> pc_lines_;
    Eigen::Matrix3d rotmat_eigen_;
    tf2::Quaternion current_quat_;
    tf2::Quaternion target_q_;
    double dz_ = 0.0;

    std::string msg_;
    geometry_msgs::msg::PoseStamped target_pose_;

    bool angle_focused_ = false;
    bool z_focused_ = false;
    bool planning_ = false;
    bool angle_corrected_ = false;

    rclcpp::Client<Scan3d>::SharedPtr service_scan_3d_;

    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
    double tmp_roll_ = 0.0;
    double tmp_pitch_ = 0.0;
    double tmp_yaw_ = 0.0;

    double gating_interval_ = 0.02;
    int image_width_ = 500;
    int image_height_ = 512;
    double px_per_mm_ = 65.0;

    // Tunables
    double focus_step_timeout_sec_ = 5.0;
    int scan3d_service_wait_ms_ = 200;
    int scan3d_response_timeout_ms_ = 2000;

    double angle_tolerance_ = 0.0;
    double z_tolerance_ = 0.0;
    double z_height_ = 0.0;

    rclcpp::Time start;

    bool is_black(const cv::Mat &img, uint8_t pixel_thres = 5,
                  double ratio = 0.98);
    void push_frame(const cv::Mat &frame);
    bool pop_new(cv::Mat &frame);

    cv::Mat get_img();

    void image_callback(const octa_ros::msg::Img::SharedPtr msg);

    bool call_scan3d(bool activate);

    bool tol_measure(const double &roll, const double &pitch,
                     const double &angle_tolerance);

    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const Focus::Goal> goal);
    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleFocus> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFocus> goal_handle);
    void execute(const std::shared_ptr<GoalHandleFocus> goal_handle);
};

#endif  // FOCUS_NODE_HPP
