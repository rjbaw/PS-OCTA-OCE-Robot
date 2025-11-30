/**
 * @file focus_node.hpp
 * @author rjbaw
 * @brief Action server that aligns the end-effector normal to the target using
 * OCT scans.
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

/**
 * @brief Action server that focuses the robot end effector to the target
 * normal.
 *
 * Subscribes to OCT frames, estimates the surface normal and z-offset, and
 * executes small corrective motions until angular and height tolerances are
 * met. Provides progress through an action interface and optionally triggers a
 * 3D scan window.
 *
 * @par Parameters
 * - plan_only (bool, default: false): skip execution (plan only).
 * - offline_mode (bool, default: false): disable MoveIt and execution.
 * - gating_interval_sec (double, default: 0.02 s): minimum time between stored
 *   frames.
 * - focus_step_timeout_sec (double, default: 5.0 s): timeout per correction.
 * - scan3d_service_wait_ms (int, default: 200 ms): wait for Scan3d service.
 * - scan3d_response_timeout_ms (int, default: 2000 ms): Scan3d RPC timeout.
 * - image_width (int, default: 500 px): expected width of incoming frames.
 * - image_height (int, default: 512 px): expected height of incoming frames.
 * - image_topic (string, default: "/oct_image"): OCT topic to subscribe.
 * - px_per_mm (double, default: 65.0 px/mm): pixel density used for dz.
 * - tool_link (string, default: "tcp"): end-effector link name.
 * - envelope_radius_m (double, default: 0.05 m): spherical envelope radius.
 * - curve_model_path (string): override ONNX model path used by detection.
 *
 * @par Subscriptions
 * - `octa_ros::msg::Img` on `image_topic` (QoS: best-effort).
 *
 * @par Services
 * - Client: `octa_ros::srv::Scan3d` on `scan_3d` to toggle scanning.
 *
 * @par Action Server
 * - Name: `focus_action`; Goal fields include angle and z tolerances (degrees,
 *   mm) and z height target (mm).
 *
 * @note Units: angles are degrees in the action goal; internally converted to
 * radians for planning. Heights are in millimeters in the goal and converted to
 * meters for execution.
 *
 * @ingroup actions
 */
class FocusActionServer : public rclcpp::Node {
    using Focus = octa_ros::action::Focus;
    using GoalHandleFocus = rclcpp_action::ServerGoalHandle<Focus>;
    using Scan3d = octa_ros::srv::Scan3d;

  public:
    /**
     * @brief Construct the node; call init() to create interfaces.
     */
    explicit FocusActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Create publishers/subscribers, action server and service clients.
     */
    void init();

  private:
    bool early_terminate_ = false;
    bool skip_angle_tolerance_ = false;
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
    int64_t image_width_ = 500;
    int64_t image_height_ = 512;
    double px_per_mm_ = 65.0;
    std::string image_topic_ = "/oct_image";

    // Tunables
    double focus_step_timeout_sec_ = 10.0;
    int64_t scan3d_service_wait_ms_ = 200;
    int64_t scan3d_response_timeout_ms_ = 4000;

    // Parameter callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        param_cb_handle_;

    double angle_tolerance_ = 0.0;
    double z_tolerance_ = 0.0;
    double z_height_ = 0.0;

    rclcpp::Time start;

#ifdef LEGACY_IMG_PIPELINE
    /**
     * @brief Capture current frame as background (for classic pipeline).
     *
     * Saves to package share `config/bg.jpg`.
     */
    void capture_background_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_background_srv_;
#endif

    /**
     * @brief Fast check if the image is near-blank.
     * @param img Grayscale image.
     * @param pixel_thres Pixel threshold to consider as black.
     * @param ratio Fraction of pixels below threshold to qualify as black.
     */
    bool is_black(const cv::Mat &img, uint8_t pixel_thres = 5,
                  double ratio = 0.98);

    /**
     * @brief Push a new frame into the lock-free ring buffer.
     */
    void push_frame(const cv::Mat &frame);

    /**
     * @brief Pop the newest frame not yet processed.
     * @return True if a new frame was returned.
     */
    bool pop_new(cv::Mat &frame);

    /**
     * @brief Get the latest buffered image (copy).
     */
    cv::Mat get_img();

    /**
     * @brief OCT image subscription callback.
     */
    void image_callback(octa_ros::msg::Img::SharedPtr msg);

    /**
     * @brief Toggle 3D scan acquisition via service.
     * @param activate Whether to start (true) or stop (false) scanning.
     */
    bool call_scan3d(bool activate);

    /**
     * @brief Check whether roll/pitch are within the requested tolerance.
     */
    bool tol_measure(const double &roll, const double &pitch,
                     const double &angle_tolerance);

    /**
     * @brief Action callbacks for goal management and execution.
     */
    rclcpp_action::GoalResponse
    handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const Focus::Goal> goal);
    rclcpp_action::CancelResponse
    handle_cancel(std::shared_ptr<GoalHandleFocus> goal_handle);
    void handle_accepted(std::shared_ptr<GoalHandleFocus> goal_handle);
    void execute(std::shared_ptr<GoalHandleFocus> goal_handle);
};

#endif // FOCUS_NODE_HPP
