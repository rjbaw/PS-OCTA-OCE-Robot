/**
 * @file focus_node.cpp
 * @author rjbaw
 * @brief Node that focuses robot end effector to the normal of the target
 */

#include "focus_node.hpp"
#include "motion_utils.hpp"
#include "process_img.hpp"
#include <filesystem>
#include <format>
#include <open3d/Open3D.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/utilities.hpp>

using namespace std::chrono_literals;

FocusActionServer::FocusActionServer(const rclcpp::NodeOptions &options)
    : Node("focus_action_server",
           rclcpp::NodeOptions(options)
               .automatically_declare_parameters_from_overrides(true)) {}

void FocusActionServer::init() {
    action_server_ = rclcpp_action::create_server<Focus>(
        this, "focus_action",
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const Focus::Goal> goal) {
            return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleFocus> goal_handle) {
            return this->handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleFocus> goal_handle) {
            this->handle_accepted(goal_handle);
        });

    if (!this->has_parameter("plan_only")) {
        this->declare_parameter<bool>("plan_only", false);
    }
    if (!this->has_parameter("focus_step_timeout_sec")) {
        this->declare_parameter<double>("focus_step_timeout_sec",
                                        focus_step_timeout_sec_);
    }
    if (!this->has_parameter("scan3d_service_wait_ms")) {
        this->declare_parameter<int64_t>("scan3d_service_wait_ms",
                                         scan3d_service_wait_ms_);
    }
    if (!this->has_parameter("scan3d_response_timeout_ms")) {
        this->declare_parameter<int64_t>("scan3d_response_timeout_ms",
                                         scan3d_response_timeout_ms_);
    }
    if (!this->has_parameter("px_per_mm")) {
        this->declare_parameter<double>("px_per_mm", px_per_mm_);
    }
    if (!this->has_parameter("image_topic")) {
        this->declare_parameter<std::string>("image_topic", image_topic_);
    }
    // Declare planning related parameters once here
    if (!this->has_parameter("tool_link")) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description =
            "Tool link used to build constraints and goals";
        this->declare_parameter<std::string>("tool_link", std::string("tcp"),
                                             descriptor);
    }
    if (!this->has_parameter("curve_model_path")) {
        this->declare_parameter<std::string>("curve_model_path",
                                             std::string(""));
    }

    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "";
            for (const auto &param : params) {
                const auto &name = param.get_name();
                if (name == "focus_step_timeout_sec") {
                    if (param.as_double() <= 0.0) {
                        result.successful = false;
                        result.reason = "focus_step_timeout_sec must be > 0";
                        return result;
                    }
                } else if (name == "scan3d_service_wait_ms" ||
                           name == "scan3d_response_timeout_ms") {
                    if (param.as_int() <= 0) {
                        result.successful = false;
                        result.reason = "integer parameter must be > 0";
                        return result;
                    }
                } else if (name == "px_per_mm") {
                    if (param.as_double() <= 0.0) {
                        result.successful = false;
                        result.reason = "px_per_mm must be > 0";
                        return result;
                    }
                } else if (name == "curve_model_path") {
                    const auto &path_val = param.as_string();
                    if (!path_val.empty() &&
                        !std::filesystem::exists(path_val)) {
                        result.successful = false;
                        result.reason = "curve_model_path does not exist";
                        return result;
                    }
                }
            }
            for (const auto &param : params) {
                const auto &name = param.get_name();
                if (name == "focus_step_timeout_sec") {
                    focus_step_timeout_sec_ = param.as_double();
                } else if (name == "scan3d_service_wait_ms") {
                    scan3d_service_wait_ms_ = param.as_int();
                } else if (name == "scan3d_response_timeout_ms") {
                    scan3d_response_timeout_ms_ = param.as_int();
                } else if (name == "px_per_mm") {
                    px_per_mm_ = param.as_double();
                } else if (name == "curve_model_path") {
                    const std::string &path_val = param.as_string();
                    if (!path_val.empty()) {
                        octa_ros::img::set_curve_model_path(path_val);
                    }
                }
            }
            return result;
        });

    bool plan_only = this->get_parameter("plan_only").as_bool();
    focus_step_timeout_sec_ =
        this->get_parameter("focus_step_timeout_sec").as_double();
    scan3d_service_wait_ms_ =
        this->get_parameter("scan3d_service_wait_ms").as_int();
    scan3d_response_timeout_ms_ =
        this->get_parameter("scan3d_response_timeout_ms").as_int();
    px_per_mm_ = this->get_parameter("px_per_mm").as_double();
    auto curve_model_path = this->get_parameter("curve_model_path").as_string();
    if (!curve_model_path.empty() &&
        std::filesystem::exists(curve_model_path)) {
        octa_ros::img::set_curve_model_path(curve_model_path);
    }
#ifndef LEGACY_IMG_PIPELINE
    octa_ros::img::preload_curve_model(static_cast<std::size_t>(interval_));
#endif

    if (!plan_only) {
        moveit_cpp_ =
            std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());
        tem_ = moveit_cpp_->getTrajectoryExecutionManagerNonConst();
        planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(
            "ur_manipulator", moveit_cpp_);
    } else {
        RCLCPP_INFO(get_logger(),
                    "Plan-only/Offline mode: skipping MoveIt initialization");
    }

    auto parallel_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions img_options;
    img_options.callback_group = parallel_group_;
    last_store_time_ =
        now() - rclcpp::Duration::from_seconds(gating_interval_);
    buffer_.fill(cv::Mat());
    image_topic_ = this->get_parameter("image_topic").as_string();
    img_subscriber_ = create_subscription<octa_ros::msg::Img>(
        image_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this](const octa_ros::msg::Img::SharedPtr msg) {
            this->image_callback(msg);
        },
        img_options);

    service_scan_3d_ = create_client<Scan3d>("scan_3d");
#ifdef LEGACY_IMG_PIPELINE
    capture_background_srv_ = create_service<std_srvs::srv::Trigger>(
        "capture_background",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            this->capture_background_callback(request, response);
        });
#endif
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FocusActionServer>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

rclcpp_action::GoalResponse FocusActionServer::handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Focus::Goal> goal) {
    angle_tolerance_ = goal->angle_tolerance;
    z_tolerance_ = goal->z_tolerance;
    z_height_ = goal->z_height;
    RCLCPP_INFO(get_logger(),
                "Focus goal: angle_tolerance=%.2f deg, "
                "z_height_tolerance=%.2f mm",
                angle_tolerance_, z_tolerance_);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FocusActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFocus> goal_handle) {
    if (!goal_handle->is_active()) {
        RCLCPP_INFO(get_logger(), "Focus goal no longer active");
        return rclcpp_action::CancelResponse::REJECT;
    }
    auto result = std::make_shared<Focus::Result>();
    result->status = "Focus action canceled by user request\n";
    call_scan3d(false);
    head_.notify_all();
    tem_->stopExecution(true);
    planning_component_->setStartStateToCurrentState();
    RCLCPP_INFO(get_logger(), "Focus action canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FocusActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleFocus> goal_handle) {
    auto result = std::make_shared<Focus::Result>();
    result->status = "Pre-empted by new goal\n";
    RCLCPP_INFO(get_logger(), "Preempting old goal...");

    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        active_goal_handle_->abort(result);
    }
    active_goal_handle_ = goal_handle;
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

bool FocusActionServer::is_black(const cv::Mat &img, uint8_t pixel_thres,
                                 double ratio) {
    if (img.empty()) {
        return true;
    }
    CV_Assert(img.type() == CV_8UC1);
    cv::Mat thresh;
    cv::threshold(img, thresh, pixel_thres, 255, cv::THRESH_BINARY);
    int black_pixels = static_cast<int>(img.total()) - cv::countNonZero(thresh);
    bool black = ((static_cast<double>(black_pixels) /
                   static_cast<double>(img.total())) >= ratio);
    return black;
}

void FocusActionServer::push_frame(const cv::Mat &frame) {
    size_t head = head_.load();
    buffer_[head] = frame.clone();
    size_t next = (head + 1) & (kBufferSize - 1); // wrap
    head_ = next;
    if (next == tail_.load()) {
        tail_ = (tail_ + 1) & (kBufferSize - 1);
    }
    head_.notify_all();
}

bool FocusActionServer::pop_new(cv::Mat &frame) {
    size_t head = head_.load();
    size_t tail = tail_.load();
    if (head == tail) {
        return false;
    }
    size_t newest = (head + kBufferSize - 1) & (kBufferSize - 1);
    frame = buffer_[newest];
    tail_ = (newest + 1) & (kBufferSize - 1);
    return true;
}

cv::Mat FocusActionServer::get_img() {
    cv::Mat frame;
    pop_new(frame);
    return frame;
}

#ifdef LEGACY_IMG_PIPELINE
void FocusActionServer::capture_background_callback(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>
        request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    cv::Mat frame = get_img();
    if (!frame.empty()) {
        std::string pkg_share =
            ament_index_cpp::get_package_share_directory("octa_ros");
        std::string bg_path = pkg_share + "/config/bg.jpg";
        cv::imwrite(bg_path, frame);
        response->success = true;
    } else {
        RCLCPP_INFO(get_logger(), "No image captured – background not saved");
        response->success = false;
    }
}
#endif

void FocusActionServer::image_callback(
    const octa_ros::msg::Img::SharedPtr msg) {
    auto now = this->now();
    auto elapsed = (now - last_store_time_).seconds();
    if (elapsed < gating_interval_) {
        RCLCPP_DEBUG(get_logger(), "Skipping frame (%.2f sec since last store)",
                     elapsed);
        return;
    }
    RCLCPP_DEBUG(get_logger(), "Storing new frame after %.2f sec (size=%zu)",
                 elapsed, msg->img.size());

    cv::Mat new_img(static_cast<int>(image_height_),
                    static_cast<int>(image_width_), CV_8UC1, msg->img.data());

    if (is_black(new_img)) {
        RCLCPP_DEBUG(get_logger(), "Discarding black frame");
        return;
    };

    last_store_time_ = now;
    push_frame(new_img);
}

bool FocusActionServer::call_scan3d(bool activate) {
    if (!service_scan_3d_->wait_for_service(
            std::chrono::milliseconds(scan3d_service_wait_ms_))) {
        return false;
    }
    auto req = std::make_shared<Scan3d::Request>();
    req->activate = activate;
    auto fut = service_scan_3d_->async_send_request(req);
    return fut.wait_for(std::chrono::milliseconds(
               scan3d_response_timeout_ms_)) == std::future_status::ready &&
           fut.get()->success;
}

bool FocusActionServer::tol_measure(const double &roll, const double &pitch,
                                    const double &angle_tolerance) {
    return ((std::abs(roll) < to_radian(angle_tolerance)) &&
            (std::abs(pitch) < to_radian(angle_tolerance)));
}

void FocusActionServer::execute(
    const std::shared_ptr<GoalHandleFocus> goal_handle) {
    if (!goal_handle->is_active()) {
        return;
    }
    auto feedback = std::make_shared<Focus::Feedback>();
    auto result = std::make_shared<Focus::Result>();

    goal_handle->publish_feedback(feedback);
    bool plan_only_fb = this->get_parameter("plan_only").as_bool();
    if (plan_only_fb) {
        feedback->debug_msgs = "Plan-only mode: skipping execution.\n";
        goal_handle->publish_feedback(feedback);
        result->status = "Focus completed (plan-only/offline)\n";
        goal_handle->succeed(result);
        return;
    }
    bool plan_only = this->get_parameter("plan_only").as_bool();
    if (plan_only) {
        feedback->debug_msgs = "Plan-only mode: skipping execution.\n";
        goal_handle->publish_feedback(feedback);
        result->status = "Focus completed (plan-only/offline)\n";
        goal_handle->succeed(result);
        return;
    }
    angle_focused_ = false;
    z_focused_ = false;
    angle_corrected_ = false;
    planning_ = false;

    while (!angle_focused_ || !z_focused_) {
        if (goal_handle->is_canceling()) {
            tem_->stopExecution(true);
            result->status = "Cancel requested!\n";
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Cancel requested!");
            planning_component_->setStartStateToCurrentState();
            return;
        }
        start = now();
        while (!call_scan3d(true)) {
            if (goal_handle->is_canceling()) {
                tem_->stopExecution(true);
                result->status = "Cancel requested!\n";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Cancel requested!");
                planning_component_->setStartStateToCurrentState();
                return;
            }
            if ((now() - start).seconds() > focus_step_timeout_sec_) {
                RCLCPP_WARN(get_logger(), "activate_3d_scan not responding...");
                result->status = "activate_3d_scan timed out\n";
                goal_handle->abort(result);
                return;
            }
        }
        std::this_thread::sleep_for(400ms);
        std::vector<cv::Mat> img_array;
        for (int i = 0; i < interval_; i++) {
            start = now();
            while (true) {
                cv::Mat frame = get_img();
                if (!frame.empty()) {
                    img_array.push_back(frame);
                    break;
                }
                if (goal_handle->is_canceling()) {
                    tem_->stopExecution(true);
                    result->status = "Cancel requested!\n";
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Cancel requested!");
                    planning_component_->setStartStateToCurrentState();
                    return;
                }
                if ((now() - start).seconds() > focus_step_timeout_sec_) {
                    RCLCPP_WARN(get_logger(),
                                "timed out. cannot acquire image");
                    result->status = "timed out. cannot acquire image.\n";
                    goal_handle->abort(result);
                    return;
                }
            }
            msg_ = std::format("Collected image {}", i + 1);
            RCLCPP_INFO(get_logger(), msg_.c_str());
        }

        start = now();
        while (!call_scan3d(false)) {
            if (goal_handle->is_canceling()) {
                tem_->stopExecution(true);
                result->status = "Cancel requested!\n";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Cancel requested!");
                planning_component_->setStartStateToCurrentState();
                return;
            }
            if ((now() - start).seconds() > focus_step_timeout_sec_) {
                RCLCPP_WARN(get_logger(), "activate_3d_scan not responding...");
                result->status = "activate_3d_scan timed out\n";
                goal_handle->abort(result);
                return;
            }
        }
        msg_ = "Calculating Rotations";
        RCLCPP_INFO(get_logger(), msg_.c_str());

        pc_lines_ = octa_ros::img::lines_3d(img_array, interval_);
        if (pc_lines_.empty()) {
            feedback->debug_msgs = "Background detected"
                                   "; reacquiring image stack...\n";
            goal_handle->publish_feedback(feedback);
            RCLCPP_WARN(get_logger(),
                        "Background detected in the image stack; retrying...");
            continue;
        }

        open3d::geometry::PointCloud pcd;
        for (const auto &point : pc_lines_) {
            pcd.points_.emplace_back(point);
        }
        auto boundbox = pcd.GetMinimalOrientedBoundingBox(false);
        Eigen::Vector3d center = boundbox.GetCenter();
        rotmat_eigen_ = octa_ros::img::align_to_direction(boundbox.R_);

        planning_component_->setStartStateToCurrentState();
        moveit::core::RobotStatePtr current_state =
            moveit_cpp_->getCurrentState();
        Eigen::Isometry3d current_pose =
            current_state->getGlobalLinkTransform("tcp");
        target_pose_.header.frame_id = moveit_cpp_->getPlanningSceneMonitor()
                                           ->getPlanningScene()
                                           ->getPlanningFrame();
        target_pose_.pose = tf2::toMsg(current_pose);

        tf2::Matrix3x3 rotmat_tf_(
            rotmat_eigen_(0, 0), rotmat_eigen_(0, 1), rotmat_eigen_(0, 2),
            rotmat_eigen_(1, 0), rotmat_eigen_(1, 1), rotmat_eigen_(1, 2),
            rotmat_eigen_(2, 0), rotmat_eigen_(2, 1), rotmat_eigen_(2, 2));
        RCLCPP_INFO_STREAM(get_logger(), "\nAligned Rotation Matrix:\n"
                                             << rotmat_eigen_);
        rotmat_tf_.getRPY(tmp_roll_, tmp_pitch_, tmp_yaw_);
        roll_ = -tmp_pitch_;
        pitch_ = tmp_roll_;
        yaw_ = tmp_yaw_;
        rotmat_tf_.setRPY(roll_, pitch_, yaw_);

        dz_ = (z_height_ - center[2]) / (px_per_mm_ * 1000.0);

        msg_ = std::format("Calculated:\n"
                           "    [Rotation] R:{:.2f} P:{:.2f} Y:{:.2f}\n"
                           "    [Center]   x:{:.2f}  y:{:.2f}  z:{:.2f}\n"
                           "    [Height]   dz:{:.4f}\n",
                           to_degree(roll_), to_degree(pitch_), to_degree(yaw_),
                           center[0], center[1], center[2], dz_ * 1000);
        feedback->debug_msgs = msg_;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), msg_.c_str());

        if (tol_measure(roll_, pitch_, angle_tolerance_)) {
            angle_focused_ = true;
            msg_ = "=> Angle focused\n";
            feedback->debug_msgs = msg_;
            RCLCPP_INFO(get_logger(), msg_.c_str());
            goal_handle->publish_feedback(feedback);
        } else {
            if (!angle_corrected_) {
                planning_ = true;
                angle_corrected_ = true;
            } else if (skip_angle_tolerance_) {
                angle_focused_ = true;
            } else {
                angle_focused_ = false;
            }
        }
        if (std::abs(dz_) < (z_tolerance_ / 1000.0)) {
            z_focused_ = true;
            msg_ = "=> Height focused\n";
            feedback->debug_msgs = msg_;
            RCLCPP_INFO(get_logger(), msg_.c_str());
            goal_handle->publish_feedback(feedback);
        } else {
            z_focused_ = false;
        }

        if (angle_focused_ && !z_focused_) {
            planning_ = true;
            target_pose_.pose.position.z += dz_;
            print_target(get_logger(), target_pose_.pose);
        }

        if (!angle_focused_) {
            planning_ = true;
            rotmat_tf_.getRotation(current_quat_);
            tf2::fromMsg(target_pose_.pose.orientation, target_q_);
            target_q_ = target_q_ * current_quat_;
            target_q_.normalize();
            target_pose_.pose.orientation = tf2::toMsg(target_q_);
            target_pose_.pose.position.z += dz_;
            print_target(get_logger(), target_pose_.pose);
        }

        if (planning_) {
            planning_component_->setStartStateToCurrentState();
            moveit::core::RobotStatePtr cur_state =
                moveit_cpp_->getCurrentState();
            const std::string tool_link =
                this->get_parameter("tool_link").as_string();
            Eigen::Isometry3d start_tcp =
                cur_state->getGlobalLinkTransform(tool_link);
            const std::string planning_frame =
                moveit_cpp_->getPlanningSceneMonitor()
                    ->getPlanningScene()
                    ->getPlanningFrame();
            const double envelope_radius =
                this->has_parameter("envelope_radius_m")
                    ? this->get_parameter("envelope_radius_m").as_double()
                    : this->declare_parameter<double>("envelope_radius_m",
                                                      0.05);
            auto envelope = octa_ros::motion::make_envelope(
                start_tcp, planning_frame, tool_link, envelope_radius, M_PI);
            planning_component_->setPathConstraints(envelope);
            planning_component_->setGoal(target_pose_, tool_link);
            auto req = moveit_cpp::PlanningComponent::
                MultiPipelinePlanRequestParameters(shared_from_this(),
                                                   {"pilz_ptp", "pilz_lin"});
            auto choose_shortest =
                [](const std::vector<planning_interface::MotionPlanResponse>
                       &sols) {
                    return *std::min_element(
                        sols.begin(), sols.end(),
                        [](const auto &lhs, const auto &rhs) {
                            if (lhs && rhs) {
                                return robot_trajectory::pathLength(
                                           *lhs.trajectory) <
                                       robot_trajectory::pathLength(
                                           *rhs.trajectory);
                            }
                            return static_cast<bool>(lhs);
                        });
                };
            planning_interface::MotionPlanResponse plan_solution =
                planning_component_->plan(req, choose_shortest);
            if (plan_solution) {
                if (goal_handle->is_canceling()) {
                    result->status = "Cancel requested!\n";
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Cancel requested!");
                    planning_component_->setStartStateToCurrentState();
                    return;
                }
                bool execute_success = static_cast<bool>(
                    moveit_cpp_->execute(plan_solution.trajectory));
                if (execute_success) {
                    planning_ = false;
                    RCLCPP_INFO(get_logger(), "Execute Success!");
                    if (early_terminate_) {
                        angle_focused_ = true;
                        z_focused_ = true;
                        break;
                    }
                } else {
                    RCLCPP_ERROR(get_logger(), "Execution failed!");
                    result->status = "Execution failed!\n";
                    result->status +=
                        std::string(" => Retrying with next image set...\n");
                    goal_handle->publish_feedback(feedback);
                    planning_ = false;
                }
            } else {
                planning_ = false;
                RCLCPP_WARN(get_logger(),
                            "Planning failed – retrying with new images");
            }
        }
    }

    if (goal_handle->is_canceling()) {
        result->status = "Focus action canceled\n";
        goal_handle->publish_feedback(feedback);
        goal_handle->canceled(result);
        return;
    }

    if (goal_handle->is_active()) {
        result->status = "Focus completed successfully\n";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Focus action completed successfully.");
    }
}
