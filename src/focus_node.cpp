/**
 * @file focus_node.cpp
 * @author rjbaw
 * @brief Node that focuses robot end effector to the normal of the target
 */

#include "focus_node.hpp"
#include "motion_utils.hpp"
#include "process_img.hpp"
#include <exception>
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

FocusActionServer::~FocusActionServer() {
    action_server_.reset();
    if (cancel_worker_.joinable()) {
        cancel_worker_.request_stop();
        cancel_worker_.join();
    }
    if (worker_.joinable()) {
        worker_.request_stop();
        try {
            if (tem_ && active_goal_handle_ &&
                active_goal_handle_->is_active()) {
                tem_->stopExecution(true);
            }
        } catch (const std::exception &exception) {
            RCLCPP_ERROR(get_logger(),
                         "Could not stop Focus during shutdown: %s",
                         exception.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(),
                         "Could not stop Focus during shutdown: unknown "
                         "exception");
        }
        worker_.join();
    }
    active_goal_handle_.reset();
}

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

    const bool plan_only = this->get_parameter("plan_only").as_bool();
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
    octa_ros::img::preload_curve_model(static_cast<std::size_t>(interval_));
    if (!plan_only) {
        cv::Mat warmup_img(static_cast<int>(image_height_),
                           static_cast<int>(image_width_), CV_8UC1,
                           cv::Scalar(0));
        octa_ros::img::warmup_curve_model(warmup_img,
                                          static_cast<std::size_t>(interval_));
    }

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

    image_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions img_options;
    img_options.callback_group = image_callback_group_;
    last_store_time_ = now() - rclcpp::Duration::from_seconds(gating_interval_);
    buffer_.fill(cv::Mat());
    image_topic_ = this->get_parameter("image_topic").as_string();
    img_subscriber_ = create_subscription<octa_ros::msg::Img>(
        image_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this](const octa_ros::msg::Img::SharedPtr msg) {
            this->image_callback(msg);
        },
        img_options);

    service_scan_3d_ = create_client<Scan3d>("scan_3d");
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
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        RCLCPP_WARN(get_logger(), "Focus goal still processing");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(),
                "Focus goal: angle_tolerance=%.2f deg, "
                "z_height_tolerance=%.2f mm",
                goal->angle_tolerance, goal->z_tolerance);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FocusActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFocus> goal_handle) {
    if (!goal_handle->is_active()) {
        RCLCPP_INFO(get_logger(), "Focus goal no longer active");
        return rclcpp_action::CancelResponse::REJECT;
    }
    try {
        if (service_scan_3d_->service_is_ready()) {
            auto request = std::make_shared<Scan3d::Request>();
            request->activate = false;
            service_scan_3d_->async_send_request(request);
        }
    } catch (const std::exception &exception) {
        RCLCPP_WARN(get_logger(), "Could not request Scan3D OFF: %s",
                    exception.what());
    } catch (...) {
        RCLCPP_WARN(get_logger(),
                    "Could not request Scan3D OFF: unknown exception");
    }
    if (!cancel_worker_.joinable()) {
        cancel_worker_ = std::jthread([this, goal_handle](
                                          std::stop_token stop_token) {
            try {
                while (!stop_token.stop_requested() &&
                       goal_handle->is_active() &&
                       !goal_handle->is_canceling()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                if (!stop_token.stop_requested() &&
                    goal_handle->is_canceling() && tem_) {
                    tem_->stopExecution(true);
                }
            } catch (const std::exception &exception) {
                RCLCPP_ERROR(get_logger(),
                             "Could not stop Focus trajectory: %s",
                             exception.what());
            } catch (...) {
                RCLCPP_ERROR(get_logger(),
                             "Could not stop Focus trajectory: unknown "
                             "exception");
            }
        });
    }
    RCLCPP_INFO(get_logger(), "Focus cancel request accepted");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FocusActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleFocus> goal_handle) {
    if (cancel_worker_.joinable()) {
        cancel_worker_.join();
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    active_goal_handle_ = goal_handle;
    worker_ = std::jthread([this, goal_handle](std::stop_token stop_token) {
        const auto terminate_goal = [this, goal_handle,
                                     stop_token](const char *status) {
            if (stop_token.stop_requested()) {
                return;
            }
            try {
                auto result = std::make_shared<Focus::Result>();
                result->status = status;
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                } else if (goal_handle->is_active()) {
                    goal_handle->abort(result);
                }
            } catch (...) {
                RCLCPP_ERROR(get_logger(),
                             "Could not terminate failed Focus goal");
            }
        };
        try {
            execute(goal_handle, stop_token);
        } catch (const std::exception &exception) {
            RCLCPP_ERROR(get_logger(), "Unhandled Focus exception: %s",
                         exception.what());
            terminate_goal("Focus failed due to an internal exception\n");
        } catch (...) {
            RCLCPP_ERROR(get_logger(),
                         "Unhandled non-standard Focus exception");
            terminate_goal("Focus failed due to an internal exception\n");
        }
    });
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

bool FocusActionServer::call_scan3d(
    bool activate, const std::shared_ptr<GoalHandleFocus> &goal_handle,
    std::stop_token stop_token) {
    constexpr auto kInterruptPollPeriod = 20ms;
    const auto interrupted = [&]() {
        return stop_token.stop_requested() || goal_handle->is_canceling();
    };
    const auto service_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(scan3d_service_wait_ms_);
    while (!service_scan_3d_->service_is_ready()) {
        if (interrupted() ||
            std::chrono::steady_clock::now() >= service_deadline) {
            return false;
        }
        (void)service_scan_3d_->wait_for_service(kInterruptPollPeriod);
    }
    if (interrupted()) {
        return false;
    }

    auto req = std::make_shared<Scan3d::Request>();
    req->activate = activate;
    auto fut = service_scan_3d_->async_send_request(req);
    const auto response_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(scan3d_response_timeout_ms_);
    while (std::chrono::steady_clock::now() < response_deadline) {
        if (interrupted()) {
            service_scan_3d_->remove_pending_request(fut);
            return false;
        }
        if (fut.wait_for(kInterruptPollPeriod) == std::future_status::ready) {
            return fut.get()->success;
        }
    }
    service_scan_3d_->remove_pending_request(fut);
    return false;
}

bool FocusActionServer::tol_measure(const double &roll, const double &pitch,
                                    const double &angle_tolerance) {
    return ((std::abs(roll) < to_radian(angle_tolerance)) &&
            (std::abs(pitch) < to_radian(angle_tolerance)));
}

void FocusActionServer::execute(
    const std::shared_ptr<GoalHandleFocus> goal_handle,
    std::stop_token stop_token) {
    if (stop_token.stop_requested() || !goal_handle->is_active()) {
        return;
    }
    auto feedback = std::make_shared<Focus::Feedback>();
    auto result = std::make_shared<Focus::Result>();
    const auto handle_interruption = [&]() {
        if (stop_token.stop_requested()) {
            return true;
        }
        if (!goal_handle->is_canceling()) {
            return false;
        }
        result->status = "Cancel requested!\n";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Cancel requested!");
        if (planning_component_) {
            planning_component_->setStartStateToCurrentState();
        }
        return true;
    };
    const auto goal = goal_handle->get_goal();
    angle_tolerance_ = goal->angle_tolerance;
    z_tolerance_ = goal->z_tolerance;
    z_height_ = goal->z_height;

    if (handle_interruption()) {
        return;
    }
    goal_handle->publish_feedback(feedback);
    const bool plan_only = !moveit_cpp_;
    if (plan_only) {
        if (handle_interruption()) {
            return;
        }
        feedback->debug_msgs = "Plan-only mode: skipping execution.\n";
        goal_handle->publish_feedback(feedback);
        if (handle_interruption()) {
            return;
        }
        result->status = "Focus completed (plan-only/offline)\n";
        goal_handle->succeed(result);
        return;
    }
    angle_focused_ = false;
    z_focused_ = false;
    angle_corrected_ = false;
    planning_ = false;

    while (!angle_focused_ || !z_focused_) {
        if (handle_interruption()) {
            return;
        }
        start = now();
        while (!call_scan3d(true, goal_handle, stop_token)) {
            if (handle_interruption()) {
                return;
            }
            if ((now() - start).seconds() > focus_step_timeout_sec_) {
                RCLCPP_WARN(get_logger(), "activate_3d_scan not responding...");
                result->status = "activate_3d_scan timed out\n";
                goal_handle->abort(result);
                return;
            }
        }
        std::vector<cv::Mat> img_array;
        for (int i = 0; i < interval_; i++) {
            start = now();
            while (true) {
                if (handle_interruption()) {
                    return;
                }
                cv::Mat frame = get_img();
                if (!frame.empty()) {
                    img_array.push_back(frame);
                    break;
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
            RCLCPP_INFO(get_logger(), "%s", msg_.c_str());
        }

        start = now();
        while (!call_scan3d(false, goal_handle, stop_token)) {
            if (handle_interruption()) {
                return;
            }
            if ((now() - start).seconds() > focus_step_timeout_sec_) {
                RCLCPP_WARN(get_logger(), "activate_3d_scan not responding...");
                result->status = "activate_3d_scan timed out\n";
                goal_handle->abort(result);
                return;
            }
        }
        if (handle_interruption()) {
            return;
        }
        msg_ = "Calculating Rotations";
        RCLCPP_INFO(get_logger(), "%s", msg_.c_str());

        auto surface = octa_ros::img::reconstruct_surface(img_array, interval_);
        if (handle_interruption()) {
            return;
        }
        pc_lines_ = surface.point_cloud;
        if (pc_lines_.empty()) {
            feedback->debug_msgs = "Background detected"
                                   "; reacquiring image stack...\n";
            if (handle_interruption()) {
                return;
            }
            goal_handle->publish_feedback(feedback);
            RCLCPP_WARN(get_logger(),
                        "Background detected in the image stack; retrying...");
            continue;
        }

        Eigen::Vector3d center;
        if (surface.pose_ok) {
            center = surface.center;
            rotmat_eigen_ = octa_ros::img::align_to_direction(surface.rotation);
            RCLCPP_INFO_STREAM(get_logger(), "\nRobust Plane Rotation:\n"
                                                 << surface.rotation
                                                 << "\nRobust Plane Normal: "
                                                 << surface.normal.transpose()
                                                 << "\nRobust Plane Scale: "
                                                 << surface.robust_scale);
        } else {
            open3d::geometry::PointCloud pcd;
            for (const auto &point : pc_lines_) {
                pcd.points_.emplace_back(point);
            }
            auto boundbox = pcd.GetMinimalOrientedBoundingBox(false);
            center = boundbox.GetCenter();
            rotmat_eigen_ = octa_ros::img::align_to_direction(boundbox.R_);
            RCLCPP_WARN(get_logger(),
                        "Robust plane fit unavailable; using OBB fallback.");
        }

        const std::string tool_link =
            this->get_parameter("tool_link").as_string();
        planning_component_->setStartStateToCurrentState();
        moveit::core::RobotStatePtr current_state =
            moveit_cpp_->getCurrentState();
        if (handle_interruption()) {
            return;
        }
        Eigen::Isometry3d current_pose =
            current_state->getGlobalLinkTransform(tool_link);
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

        dz_ = (center[2] - z_height_) / (px_per_mm_ * 1000.0);
        // T_world_target = T_world_tcp * T_tcp_target
        const Eigen::Vector3d dz_tcp(0.0, 0.0, dz_);
        const Eigen::Vector3d dz_world = current_pose.linear() * dz_tcp;

        msg_ = std::format("Calculated:\n"
                           "    [Rotation] R:{:.2f} P:{:.2f} Y:{:.2f}\n"
                           "    [Center]   x:{:.2f}  y:{:.2f}  z:{:.2f}\n"
                           "    [Height]   dz:{:.4f}\n",
                           to_degree(roll_), to_degree(pitch_), to_degree(yaw_),
                           center[0], center[1], center[2], dz_ * 1000);
        feedback->debug_msgs = msg_;
        if (handle_interruption()) {
            return;
        }
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "%s", msg_.c_str());

        if (tol_measure(roll_, pitch_, angle_tolerance_)) {
            angle_focused_ = true;
            msg_ = "=> Angle focused\n";
            feedback->debug_msgs = msg_;
            RCLCPP_INFO(get_logger(), "%s", msg_.c_str());
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
            RCLCPP_INFO(get_logger(), "%s", msg_.c_str());
            goal_handle->publish_feedback(feedback);
        } else {
            z_focused_ = false;
        }

        if (angle_focused_ && !z_focused_) {
            planning_ = true;
            target_pose_.pose.position.x += dz_world.x();
            target_pose_.pose.position.y += dz_world.y();
            target_pose_.pose.position.z += dz_world.z();
            print_target(get_logger(), target_pose_.pose);
        }

        if (!angle_focused_) {
            planning_ = true;
            rotmat_tf_.getRotation(current_quat_);
            tf2::fromMsg(target_pose_.pose.orientation, target_q_);
            target_q_ = target_q_ * current_quat_;
            target_q_.normalize();
            target_pose_.pose.orientation = tf2::toMsg(target_q_);
            target_pose_.pose.position.x += dz_world.x();
            target_pose_.pose.position.y += dz_world.y();
            target_pose_.pose.position.z += dz_world.z();
            print_target(get_logger(), target_pose_.pose);
        }

        if (planning_) {
            if (handle_interruption()) {
                return;
            }
            planning_component_->setStartStateToCurrentState();
            moveit::core::RobotStatePtr cur_state =
                moveit_cpp_->getCurrentState();
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
            if (handle_interruption()) {
                return;
            }
            if (plan_solution) {
                bool execute_success = static_cast<bool>(
                    moveit_cpp_->execute(plan_solution.trajectory));
                if (handle_interruption()) {
                    return;
                }
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

    if (handle_interruption()) {
        return;
    }

    if (goal_handle->is_active()) {
        result->status = "Focus completed successfully\n";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Focus action completed successfully.");
    }
}
