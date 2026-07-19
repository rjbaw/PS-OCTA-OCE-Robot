/**
 * @file coordinator_node.cpp
 * @author rjbaw
 * @brief coordinator node that executes action servers, subscribe, and publish
 * to LabView interface.
 * Actions are toggled using rising edge to prevent multiple triggers.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <format>
#include <sstream>
#include <string>
#include <vector>

#include "coordinator_node.hpp"

using namespace std::chrono_literals;

#include "moveit_msgs/msg/collision_object.hpp"
#include "scene_utils.hpp"

CoordinatorNode::CoordinatorNode(const rclcpp::NodeOptions &options)
    : Node("coordinator_node",
           rclcpp::NodeOptions(options)
               .automatically_declare_parameters_from_overrides(true)) {}

void CoordinatorNode::init() {
    coordinator_group_ =
        this->get_node_base_interface()->get_default_callback_group();
    service_response_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    scan3d_service_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Topic/action/service name parameters
    auto declare_str = [&](const char *name, const char *def,
                           const char *desc) {
        if (!this->has_parameter(name)) {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.description = desc;
            this->declare_parameter<std::string>(name, std::string(def),
                                                 descriptor);
        }
        return this->get_parameter(name).as_string();
    };
    const std::string topic_robot_data = declare_str(
        "topic_robot_data", "robot_data", "Robotdata publish topic");
    const std::string topic_labview_data = declare_str(
        "topic_labview_data", "labview_data", "Labviewdata subscription topic");
    const std::string topic_cancel = declare_str(
        "topic_cancel", "cancel_current_action", "Cancel Bool topic");
    const std::string srv_scan3d =
        declare_str("srv_scan3d", "scan_3d", "Scan3d service name");
    const std::string action_focus_name =
        declare_str("action_focus_name", "focus_action", "Focus action name");
    const std::string action_move_name =
        declare_str("action_move_name", "move_action", "Move action name");
    const std::string action_freedrive_name = declare_str(
        "action_freedrive_name", "freedrive_action", "Freedrive action name");
    const std::string action_reset_name =
        declare_str("action_reset_name", "reset_action", "Reset action name");

    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        pub_handle_ = this->create_publisher<octa_ros::msg::Robotdata>(
            topic_robot_data, qos);
    }

    {
        rclcpp::SubscriptionOptions options;
        options.callback_group = coordinator_group_;
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        sub_handle_ = this->create_subscription<octa_ros::msg::Labviewdata>(
            topic_labview_data, qos,
            [this](const octa_ros::msg::Labviewdata::SharedPtr msg) {
                this->subscriber_callback(msg);
            },
            options);
    }
    {
        rclcpp::SubscriptionOptions options;
        options.callback_group = coordinator_group_;
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        cancel_handle_ = this->create_subscription<std_msgs::msg::Bool>(
            topic_cancel, qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->cancel_callback(msg);
            },
            options);
    }
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        ur_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/ur_status_msg", qos,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                ur_status_msg_ = msg->data;
            });
    }
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        ur_health_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/ur_driver_healthy", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                ur_driver_healthy_.store(msg->data);
            });
    }

    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        scan_3d_srv_ = create_service<Scan3d>(
            srv_scan3d,
            [this](const std::shared_ptr<Scan3d::Request> request,
                   const std::shared_ptr<Scan3d::Response> response) {
                this->scan3d_callback(request, response);
            },
            qos, scan3d_service_group_);
    }

    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());

    const std::string planning_frame = moveit_cpp_->getPlanningSceneMonitor()
                                           ->getPlanningScene()
                                           ->getPlanningFrame();

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(octa_ros::scene::make_floor(planning_frame));
    collision_objects.push_back(
        octa_ros::scene::make_robot_base(planning_frame));
    // collision_objects.push_back(octa_ros::scene::make_monitor(planning_frame));
    psi.addCollisionObjects(collision_objects);

    RCLCPP_INFO(get_logger(), "Collision objects added to planning scene.");

    // Declare tunable parameters with defaults
    if (!this->has_parameter("pub_period_ms")) {
        this->declare_parameter<int64_t>("pub_period_ms", pub_period_ms_);
    }
    if (!this->has_parameter("main_loop_period_ms")) {
        this->declare_parameter<int64_t>("main_loop_period_ms",
                                         main_loop_period_ms_);
    }
    if (!this->has_parameter("action_server_wait_ms")) {
        this->declare_parameter<int64_t>("action_server_wait_ms",
                                         action_server_wait_ms_);
    }
    if (!this->has_parameter("config_apply_ms")) {
        this->declare_parameter<int64_t>("config_apply_ms", config_apply_ms_);
    }
    if (!this->has_parameter("scan_trigger_timeout_sec")) {
        this->declare_parameter<double>("scan_trigger_timeout_sec",
                                        scan_trigger_timeout_sec_);
    }
    if (!this->has_parameter("scan3d_window_ms")) {
        this->declare_parameter<int64_t>("scan3d_window_ms", scan3d_window_ms_);
    }

    // Load parameters
    pub_period_ms_ = this->get_parameter("pub_period_ms").as_int();
    main_loop_period_ms_ = this->get_parameter("main_loop_period_ms").as_int();
    action_server_wait_ms_ =
        this->get_parameter("action_server_wait_ms").as_int();
    config_apply_ms_ = this->get_parameter("config_apply_ms").as_int();
    scan_trigger_timeout_sec_ =
        this->get_parameter("scan_trigger_timeout_sec").as_double();
    scan3d_window_ms_ = this->get_parameter("scan3d_window_ms").as_int();

    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(pub_period_ms_),
        [this]() { this->publisher_callback(); }, coordinator_group_);

    main_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(main_loop_period_ms_),
        [this]() { this->main_loop(); }, coordinator_group_);

    full_scan_latch_timer_ = this->create_wall_timer(
        kFullScanLatchPollPeriod,
        [this]() {
            handle_full_scan_request_event(
                full_scan_request_.poll(std::chrono::steady_clock::now()));
        },
        coordinator_group_);

    focus_action_client_ =
        rclcpp_action::create_client<FocusAction>(this, action_focus_name);
    move_action_client_ =
        rclcpp_action::create_client<Move>(this, action_move_name);
    freedrive_action_client_ =
        rclcpp_action::create_client<Freedrive>(this, action_freedrive_name);
    reset_action_client_ =
        rclcpp_action::create_client<Reset>(this, action_reset_name);
    if (!focus_action_client_->wait_for_action_server(
            std::chrono::milliseconds(action_server_wait_ms_))) {
        RCLCPP_WARN(get_logger(), "Focus action server not available yet.");
    }
    if (!move_action_client_->wait_for_action_server(
            std::chrono::milliseconds(action_server_wait_ms_))) {
        RCLCPP_WARN(get_logger(), "Move action server not available yet.");
    }
    if (!freedrive_action_client_->wait_for_action_server(
            std::chrono::milliseconds(action_server_wait_ms_))) {
        RCLCPP_WARN(get_logger(), "Freedrive action server not available yet.");
    }
    if (!reset_action_client_->wait_for_action_server(
            std::chrono::milliseconds(action_server_wait_ms_))) {
        RCLCPP_WARN(get_logger(), "Reset action server not available yet.");
    }

    RCLCPP_INFO(get_logger(), "Coordinator Node Initialized.");
}

void CoordinatorNode::trigger_apply_config() {
    auto duration = std::chrono::milliseconds(config_apply_ms_);
    apply_config_ = true;
    if (config_timer_) {
        config_timer_->cancel();
        config_timer_.reset();
    }
    config_timer_ = create_wall_timer(duration, [this]() {
        if (auto timer = config_timer_weak_.lock()) {
            timer->cancel();
        }
        apply_config_ = false;
    });
    config_timer_weak_ = config_timer_;
}

void CoordinatorNode::handle_full_scan_request_event(
    octa_ros::FullScanRequestLatch::Event event) {
    using Event = octa_ros::FullScanRequestLatch::Event;
    if (event == Event::None) {
        return;
    }

    if (event == Event::PersistentLow) {
        RCLCPP_INFO(get_logger(),
                    "[Fullscan] CANCEL_REQUESTED: LabVIEW Fullscan "
                    "remained OFF for %lld ms",
                    static_cast<long long>(kFullScanOffGrace.count()));
        request_cancel(false);
        return;
    }

    if (block_action_request("Full Scan")) {
        finish_full_scan_request();
        return;
    }

    full_scan_read_ = true;
    full_scan_recipe_ = build_full_scan_recipe();
    pc_ = 0;
    angle_ = 0.0;
    circle_state_ = 1;
    scan_trigger_before_request_ = scan_trigger_read_.load();
    RCLCPP_INFO(get_logger(),
                "[Fullscan] REQUEST_ACCEPTED: LabVIEW Fullscan ON");
}

void CoordinatorNode::request_cancel(bool disarm_full_scan_latch) {
    if (full_scan_read_.load() || full_scan_.load()) {
        RCLCPP_INFO(get_logger(),
                    "[Fullscan] CANCEL: completed_steps=%u total_steps=%zu",
                    pc_.load(), full_scan_recipe_.size());
    }
    const UserAction interrupted_action = active_action_.load();
    const bool cancel_was_in_progress = cancel_in_progress_.load();
    if (interrupted_action != UserAction::Freedrive || !labview_freedrive_) {
        (void)action_callback_stop_.request_stop();
    }
    finish_full_scan_request(disarm_full_scan_latch);
    cancel_in_progress_ = true;

    if (!cancel_was_in_progress &&
        goal_can_be_canceled(active_focus_goal_handle_)) {
        focus_action_client_->async_cancel_goal(active_focus_goal_handle_);
    }
    if (!cancel_was_in_progress && goal_can_be_canceled(active_move_handle_)) {
        move_action_client_->async_cancel_goal(active_move_handle_);
    }
    if (!cancel_was_in_progress && !labview_freedrive_ &&
        goal_can_be_canceled(active_freedrive_goal_handle_)) {
        freedrive_action_client_->async_cancel_goal(
            active_freedrive_goal_handle_);
    }
    if (!cancel_was_in_progress &&
        goal_can_be_canceled(active_reset_goal_handle_)) {
        reset_action_client_->async_cancel_goal(active_reset_goal_handle_);
    }
    if (!cancel_was_in_progress && interrupted_action != UserAction::None &&
        (interrupted_action != UserAction::Freedrive || !labview_freedrive_)) {
        action_wait_started_at_ = std::chrono::steady_clock::now();
    }
    autofocus_ = false;
    reset_ = false;
    previous_ = false;
    next_ = false;
    home_ = false;
    apply_offset_ = false;
    scan_trigger_ = false;
    scan_3d_ = false;
    end_state_ = false;
    action_toggle_release_required_ = true;
}

void CoordinatorNode::finish_full_scan_request(bool disarm_latch) {
    if (disarm_latch) {
        full_scan_request_.finish();
    }
    full_scan_read_ = false;
    full_scan_ = false;
    full_scan_recipe_.clear();
    pc_ = 0;
    waiting_for_scan_completion_ = false;
}

bool CoordinatorNode::action_in_flight(UserAction action) const noexcept {
    return action != UserAction::None && action != UserAction::Scan &&
           active_action_.load() == action;
}

bool CoordinatorNode::any_action_in_flight() const noexcept {
    return active_action_.load() != UserAction::None;
}

void CoordinatorNode::continue_cancel() {
    pc_ = 0;
    autofocus_ = false;
    reset_ = false;
    previous_ = false;
    next_ = false;
    home_ = false;
    apply_offset_ = false;
    success_ = false;
    end_state_ = false;
    scan_trigger_ = false;
    scan_3d_ = false;
    scan_trigger_before_request_ = scan_trigger_read_.load();
    robot_mode_ = true;
    oct_mode_ = false;
    octa_mode_ = false;
    oce_mode_ = false;

    const bool labview_in_robot_mode =
        robot_mode_read_.load() && !oct_mode_read_.load() &&
        !octa_mode_read_.load() && !oce_mode_read_.load();
    if (!labview_in_robot_mode && !apply_config_.load()) {
        trigger_apply_config();
    }

    const UserAction canceling_action = active_action_.load();
    const auto cancel_timeout = canceling_action == UserAction::Freedrive
                                    ? kFreedriveCancelTimeout
                                    : kActionCancelTimeout;
    if ((!labview_freedrive_ || canceling_action != UserAction::Freedrive) &&
        canceling_action != UserAction::None &&
        std::chrono::steady_clock::now() - action_wait_started_at_ >=
            cancel_timeout) {
        switch (canceling_action) {
        case UserAction::Focus:
            if (active_focus_goal_handle_) {
                focus_action_client_->stop_callbacks(active_focus_goal_handle_);
                active_focus_goal_handle_.reset();
            }
            scan_3d_ = false;
            break;
        case UserAction::Move:
            if (active_move_handle_) {
                move_action_client_->stop_callbacks(active_move_handle_);
                active_move_handle_.reset();
            }
            break;
        case UserAction::Freedrive:
            if (active_freedrive_goal_handle_) {
                freedrive_action_client_->stop_callbacks(
                    active_freedrive_goal_handle_);
                active_freedrive_goal_handle_.reset();
            }
            freedrive_status_ = FreedriveStatus::Unconfirmed;
            break;
        case UserAction::Reset:
            if (active_reset_goal_handle_) {
                reset_action_client_->stop_callbacks(active_reset_goal_handle_);
                active_reset_goal_handle_.reset();
            }
            break;
        default:
            break;
        }
        active_action_ = UserAction::None;
        RCLCPP_ERROR(
            get_logger(),
            "Cancel stopped waiting for an unresponsive action server");
    }

    if (action_in_flight(UserAction::Focus) ||
        action_in_flight(UserAction::Move) ||
        action_in_flight(UserAction::Reset)) {
        msg_ = "[Cancel] Waiting for the active action to stop.\n";
        return;
    }

    if (!labview_freedrive_) {
        if (action_in_flight(UserAction::Freedrive)) {
            msg_ = "[Cancel] Waiting for Freedrive shutdown to finish.\n";
            return;
        }
        if (freedrive_status_ != FreedriveStatus::Off) {
            msg_ = "[Cancel] Confirming Freedrive Mode OFF.\n";
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                                 msg_.c_str());
            send_freedrive_goal(false);
            return;
        }
    }

    if (!labview_in_robot_mode) {
        msg_ = "[Cancel] Returning LabVIEW to Robot mode.\n";
        return;
    }

    cancel_in_progress_ = false;
    msg_ = labview_freedrive_
               ? "[Cancel] Complete; Freedrive remains ON because LabVIEW "
                 "requests it.\n"
               : "[Ready] Cancel complete.\n";
}

bool CoordinatorNode::freedrive_safely_off() const noexcept {
    return !labview_freedrive_ && freedrive_status_ == FreedriveStatus::Off &&
           !action_in_flight(UserAction::Freedrive);
}

bool CoordinatorNode::block_action_request(std::string_view action_name) {
    std::string_view reason;
    if (cancel_in_progress_.load()) {
        reason = "Cancel is returning the robot to Ready";
    } else if (labview_freedrive_) {
        reason = "Freedrive is ON";
    } else if (!freedrive_safely_off()) {
        reason = "Freedrive OFF has not been confirmed";
    } else if (any_action_in_flight()) {
        reason = "another action is active";
    }

    if (reason.empty()) {
        return false;
    }

    msg_ = std::format("[Blocked] {} because {}.\n", action_name, reason);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", msg_.c_str());
    return true;
}

std::vector<Step> CoordinatorNode::build_full_scan_recipe() const {
    const double angle_init = angle_init_.load();
    const double angle_limit = angle_limit_.load();
    const int num_pt = std::max(1, num_pt_.load());
    const int signed_n_oct = n_oct_.load();
    const bool full_scan_autofocus = signed_n_oct >= 0;
    const std::int64_t n_oct_magnitude =
        std::abs(static_cast<std::int64_t>(signed_n_oct));
    const int n_oct =
        static_cast<int>(std::min<std::int64_t>(n_oct_magnitude, num_pt));
    const bool apply_octa = apply_octa_.load();
    const double sweep = angle_limit - angle_init;
    const double angle_increment =
        sweep / static_cast<double>(std::max(1, num_pt));
    const double scan_offset = radius_.load() - 4.3;

    std::vector<Step> recipe;
    if (full_scan_autofocus) {
        recipe.push_back(
            {.action = UserAction::Focus, .mode = Mode::ROBOT, .yaw = 0.0});
    }
    if (std::abs(angle_init) > 1e-12) {
        const Mode move_mode = apply_octa ? Mode::OCTA : Mode::OCE;
        recipe.push_back(
            {.action = UserAction::Move, .mode = move_mode, .yaw = angle_init});
    }

    if (apply_octa) {
        recipe.push_back(
            {.action = UserAction::Move, .mode = Mode::OCTA, .y = scan_offset});
        recipe.push_back({.action = UserAction::Scan, .mode = Mode::OCTA});
        recipe.push_back({.action = UserAction::Move,
                          .mode = Mode::OCTA,
                          .y = -scan_offset});
    }
    recipe.push_back({.action = UserAction::Scan, .mode = Mode::OCE});

    std::vector<int> oct_breakpoints;
    if (n_oct > 0) {
        oct_breakpoints.reserve(n_oct);
        int last_bp = 1;
        for (int i = 1; i <= n_oct; ++i) {
            double pos = static_cast<double>(i) * static_cast<double>(num_pt) /
                         static_cast<double>(std::max(1, n_oct));
            int bp = static_cast<int>(std::lround(pos));
            bp = std::clamp(bp, last_bp, num_pt);
            last_bp = bp;
            oct_breakpoints.push_back(bp);
        }
        if (!oct_breakpoints.empty()) {
            oct_breakpoints.back() = num_pt;
        }
    }

    int completed_moves = 0;
    for (int i = 0; i < num_pt; ++i) {
        recipe.push_back({.action = UserAction::Move,
                          .mode = Mode::OCE,
                          .yaw = angle_increment});
        ++completed_moves;
        while (!oct_breakpoints.empty() &&
               completed_moves >= oct_breakpoints.front()) {
            if (full_scan_autofocus) {
                recipe.push_back(
                    {.action = UserAction::Focus, .mode = Mode::ROBOT});
            }
            recipe.push_back({.action = UserAction::Move,
                              .mode = Mode::OCT,
                              .y = scan_offset});
            recipe.push_back({.action = UserAction::Scan, .mode = Mode::OCT});
            recipe.push_back({.action = UserAction::Move,
                              .mode = Mode::OCT,
                              .y = -scan_offset});
            oct_breakpoints.erase(oct_breakpoints.begin());
        }
        recipe.push_back({.action = UserAction::Scan, .mode = Mode::OCE});
    }
    return recipe;
}

void CoordinatorNode::subscriber_callback(
    const octa_ros::msg::Labviewdata::SharedPtr msg) {
    robot_vel_ = msg->robot_vel;
    robot_acc_ = msg->robot_acc;
    z_tolerance_ = msg->z_tolerance;
    angle_tolerance_ = msg->angle_tolerance;
    radius_ = msg->radius;
    angle_limit_ = msg->angle_limit;
    num_pt_ = msg->num_pt;
    angle_init_ = msg->angle_init;
    n_oct_ = msg->n_oct;
    apply_octa_ = msg->apply_octa;
    labview_freedrive_ = msg->freedrive;
    if (msg->freedrive && !old_sub_msg_.freedrive &&
        (full_scan_request_.active() || full_scan_read_.load() ||
         full_scan_.load())) {
        msg_ = "[Fullscan] Canceling because Freedrive Mode was requested.\n";
        RCLCPP_WARN(get_logger(), "%s", msg_.c_str());
        request_cancel(true);
    }
    if (action_toggle_release_required_) {
        if ((msg->autofocus || msg->reset) && !freedrive_safely_off()) {
            (void)block_action_request(msg->reset ? "Reset" : "Focus");
        }
        autofocus_ = false;
        reset_ = false;
        if (!msg->autofocus && !msg->reset) {
            action_toggle_release_required_ = false;
        }
    } else {
        autofocus_ = msg->autofocus;
        reset_ = msg->reset;
    }
    scan_trigger_read_ = msg->scan_trigger;
    scan_3d_read_ = msg->scan_3d;
    z_height_ = msg->z_height;
    offset_x_ = msg->offset_x;
    offset_y_ = msg->offset_y;
    if (msg->previous == old_sub_msg_.previous) {
        previous_ = false;
    } else {
        previous_ = msg->previous;
    }
    if (msg->next == old_sub_msg_.next) {
        next_ = false;
    } else {
        next_ = msg->next;
    }
    if (msg->home == old_sub_msg_.home) {
        home_ = false;
    } else {
        home_ = msg->home;
    }
    if (msg->apply_offset == old_sub_msg_.apply_offset) {
        apply_offset_ = false;
    } else {
        apply_offset_ = msg->apply_offset;
    }
    handle_full_scan_request_event(full_scan_request_.observe(
        msg->full_scan, std::chrono::steady_clock::now()));
    if (msg->robot_mode != robot_mode_read_.load() ||
        msg->oct_mode != oct_mode_read_.load() ||
        msg->octa_mode != octa_mode_read_.load() ||
        msg->oce_mode != oce_mode_read_.load()) {
        labview_mode_changed_at_ = std::chrono::steady_clock::now();
    }
    robot_mode_read_ = msg->robot_mode;
    oct_mode_read_ = msg->oct_mode;
    octa_mode_read_ = msg->octa_mode;
    oce_mode_read_ = msg->oce_mode;
    if (*msg != old_sub_msg_) {
        std::ostringstream sub_log;
        sub_log << "[SUBSCRIBING]: Changed fields \n";

        log_if_changed(msg->robot_vel, old_sub_msg_.robot_vel, "robot_vel",
                       sub_log);
        log_if_changed(msg->robot_acc, old_sub_msg_.robot_acc, "robot_acc",
                       sub_log);
        log_if_changed(msg->z_tolerance, old_sub_msg_.z_tolerance,
                       "z_tolerance", sub_log);
        log_if_changed(msg->angle_tolerance, old_sub_msg_.angle_tolerance,
                       "angle_tolerance", sub_log);
        log_if_changed(msg->radius, old_sub_msg_.radius, "radius", sub_log);
        log_if_changed(msg->angle_limit, old_sub_msg_.angle_limit,
                       "angle_limit", sub_log);
        log_if_changed(msg->num_pt, old_sub_msg_.num_pt, "num_pt", sub_log);
        log_if_changed(msg->angle_init, old_sub_msg_.angle_init, "angle_init",
                       sub_log);
        log_if_changed(msg->n_oct, old_sub_msg_.n_oct, "n_oct", sub_log);
        log_if_changed(msg->apply_octa, old_sub_msg_.apply_octa, "apply_octa",
                       sub_log);
        log_if_changed(msg->offset_x, old_sub_msg_.offset_x, "offset_x",
                       sub_log);
        log_if_changed(msg->offset_y, old_sub_msg_.offset_y, "offset_y",
                       sub_log);
        log_if_changed(msg->apply_offset, old_sub_msg_.apply_offset,
                       "apply_offset", sub_log);
        log_if_changed(msg->autofocus, old_sub_msg_.autofocus, "autofocus",
                       sub_log);
        log_if_changed(msg->freedrive, old_sub_msg_.freedrive, "freedrive",
                       sub_log);
        log_if_changed(msg->previous, old_sub_msg_.previous, "previous",
                       sub_log);
        log_if_changed(msg->next, old_sub_msg_.next, "next", sub_log);
        log_if_changed(msg->home, old_sub_msg_.home, "home", sub_log);
        log_if_changed(msg->reset, old_sub_msg_.reset, "reset", sub_log);
        log_if_changed(msg->scan_trigger, old_sub_msg_.scan_trigger,
                       "scan_trigger", sub_log);
        log_if_changed(msg->scan_3d, old_sub_msg_.scan_3d, "scan_3d", sub_log);
        log_if_changed(msg->z_height, old_sub_msg_.z_height, "z_height",
                       sub_log);
        log_if_changed(msg->full_scan, old_sub_msg_.full_scan, "full_scan",
                       sub_log);
        log_if_changed(msg->robot_mode, old_sub_msg_.robot_mode, "robot_mode",
                       sub_log);
        log_if_changed(msg->oct_mode, old_sub_msg_.oct_mode, "oct_mode",
                       sub_log);
        log_if_changed(msg->octa_mode, old_sub_msg_.octa_mode, "octa_mode",
                       sub_log);
        log_if_changed(msg->oce_mode, old_sub_msg_.oce_mode, "oce_mode",
                       sub_log);

        RCLCPP_INFO(get_logger(), "%s", sub_log.str().c_str());
    }
    if (!autofocus_.load()) {
        end_state_ = false;
    }
    if (!full_scan_read_) {
        pc_ = 0;
    }
    old_sub_msg_ = *msg;
}

void CoordinatorNode::cancel_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) {
        cancel_button_pressed_ = false;
        return;
    }
    if (cancel_button_pressed_) {
        return;
    }
    cancel_button_pressed_ = true;
    RCLCPP_INFO(get_logger(), "Explicit action cancellation requested");
    request_cancel(full_scan_request_.active() || full_scan_read_.load() ||
                   full_scan_.load());
}

void CoordinatorNode::publisher_callback() {
    octa_ros::msg::Robotdata msg;
    const bool ur_ok = ur_driver_healthy_.load();
    if (ur_ok) {
        msg.msg = msg_;
    } else {
        if (!ur_status_msg_.empty()) {
            msg.msg = ur_status_msg_;
        } else {
            msg.msg =
                "[UR] Driver unhealthy; check robot connection and controller.";
        }
    }
    msg.angle = angle_.load();
    msg.circle_state = circle_state_.load();
    msg.scan_trigger = scan_trigger_.load();
    msg.apply_config = apply_config_.load();
    msg.end_state = end_state_.load();
    msg.scan_3d = scan_3d_.load();
    msg.full_scan = full_scan_.load();
    msg.robot_mode = robot_mode_.load();
    msg.oct_mode = oct_mode_.load();
    msg.octa_mode = octa_mode_.load();
    msg.oce_mode = oce_mode_.load();

    if (msg != old_pub_msg_) {
        std::ostringstream pub_log;
        pub_log << "[PUBLISHING]: Changed fields \n";

        log_if_changed(msg.angle, old_pub_msg_.angle, "angle", pub_log);
        log_if_changed(msg.circle_state, old_pub_msg_.circle_state,
                       "circle_state", pub_log);
        log_if_changed(msg.scan_trigger, old_pub_msg_.scan_trigger,
                       "scan_trigger", pub_log);
        log_if_changed(msg.apply_config, old_pub_msg_.apply_config,
                       "apply_config", pub_log);
        log_if_changed(msg.end_state, old_pub_msg_.end_state, "end_state",
                       pub_log);
        log_if_changed(msg.scan_3d, old_pub_msg_.scan_3d, "scan_3d", pub_log);
        log_if_changed(msg.full_scan, old_pub_msg_.full_scan, "full_scan",
                       pub_log);
        log_if_changed(msg.robot_mode, old_pub_msg_.robot_mode, "robot_mode",
                       pub_log);
        log_if_changed(msg.oct_mode, old_pub_msg_.oct_mode, "oct_mode",
                       pub_log);
        log_if_changed(msg.octa_mode, old_pub_msg_.octa_mode, "octa_mode",
                       pub_log);
        log_if_changed(msg.oce_mode, old_pub_msg_.oce_mode, "oce_mode",
                       pub_log);

        RCLCPP_INFO(get_logger(), "%s", pub_log.str().c_str());
    }
    if (scan_trigger_.load() &&
        (now() - scan_start).seconds() > scan_trigger_timeout_sec_) {
        scan_trigger_ = false;
    }
    pub_handle_->publish(msg);
    old_pub_msg_ = msg;
}

void CoordinatorNode::main_loop() {
    if (action_in_flight(UserAction::Reset) && active_reset_goal_handle_ &&
        std::chrono::steady_clock::now() - action_wait_started_at_ >=
            kResetActionTimeout) {
        msg_ = "[Reset] No terminal result received; stopping the robot.\n";
        RCLCPP_ERROR(get_logger(), "%s", msg_.c_str());
        request_cancel(false);
        return;
    }

    if (cancel_in_progress_) {
        continue_cancel();
        return;
    }

    if (!ur_driver_healthy_.load()) {
        if (freedrive_status_ != FreedriveStatus::Off &&
            !any_action_in_flight()) {
            msg_ =
                "[Safety] Disabling Freedrive while the driver is unhealthy.\n";
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                                 msg_.c_str());
            send_freedrive_goal(false);
        }
        return;
    }

    bool request_blocked = false;
    UserAction requested_action = UserAction::None;
    if (full_scan_read_) {
        const bool starting = !full_scan_;
        if (full_scan_recipe_.empty()) {
            msg_ = "Full Scan recipe is empty; aborting.\n";
            RCLCPP_WARN(get_logger(),
                        "[Fullscan] ABORT: generated recipe is empty");
            finish_full_scan_request();
            return;
        }
        if ((pc_.load() + 1) > full_scan_recipe_.size()) {
            msg_ = "Full Scan complete!\n";
            RCLCPP_INFO(get_logger(),
                        "[Fullscan] COMPLETE: completed_steps=%u "
                        "total_steps=%zu",
                        pc_.load(), full_scan_recipe_.size());
            finish_full_scan_request();
            return;
        }
        if (starting) {
            RCLCPP_INFO(get_logger(), "[Fullscan] START: total_steps=%zu",
                        full_scan_recipe_.size());
        }
        full_scan_ = true;
        const Step &step = full_scan_recipe_[pc_.load()];
        robot_mode_ = (step.mode == Mode::ROBOT);
        oct_mode_ = (step.mode == Mode::OCT);
        octa_mode_ = (step.mode == Mode::OCTA);
        oce_mode_ = (step.mode == Mode::OCE);
        std::string action_mode;
        std::string scan_mode;
        if (robot_mode_) {
            scan_mode = "ROBOT Mode";
        } else if (oct_mode_) {
            scan_mode = "OCT Mode";
        } else if (octa_mode_) {
            scan_mode = "OCTA Mode";
        } else if (oce_mode_) {
            scan_mode = "OCE Mode";
        }
        if (step.action == UserAction::Focus) {
            action_mode = "Focus Action";
        } else if (step.action == UserAction::Move) {
            action_mode = "Move Action";
        } else if (step.action == UserAction::Scan) {
            action_mode = "Scanning Action";
        }
        msg_ = std::format("Step [{}/{}]: {}, {}\n", pc_.load() + 1,
                           full_scan_recipe_.size(), action_mode, scan_mode);

        if (robot_mode_read_.load() != robot_mode_.load() ||
            oct_mode_read_.load() != oct_mode_.load() ||
            octa_mode_read_.load() != octa_mode_.load() ||
            oce_mode_read_.load() != oce_mode_.load()) {
            if (!apply_config_) {
                trigger_apply_config();
            }
            return;
        }
        if (std::chrono::steady_clock::now() - labview_mode_changed_at_ <
            kFullScanPreviewSettleDelay) {
            return;
        }

        yaw_ = step.yaw;

        requested_action = step.action;
        autofocus_ = (requested_action == UserAction::Focus);
    } else {
        const UserAction active_action = active_action_.load();
        if (active_action == UserAction::Focus ||
            active_action == UserAction::Move ||
            active_action == UserAction::Reset) {
            if (active_action != UserAction::Move) {
                requested_action = active_action;
            }
        } else if (!freedrive_safely_off()) {
            const bool other_action_requested =
                reset_.load() || autofocus_.load() || previous_.load() ||
                next_.load() || home_.load() || apply_offset_.load();
            if (other_action_requested) {
                const char *action_name = "Move";
                if (reset_.load()) {
                    action_name = "Reset";
                } else if (autofocus_.load()) {
                    action_name = "Focus";
                }
                request_blocked = block_action_request(action_name);
                if (autofocus_.load() || reset_.load()) {
                    action_toggle_release_required_ = true;
                    autofocus_ = false;
                    reset_ = false;
                }
                previous_ = false;
                next_ = false;
                home_ = false;
                apply_offset_ = false;
            }
            requested_action = UserAction::Freedrive;
        } else if (reset_) {
            requested_action = UserAction::Reset;
        } else if (autofocus_) {
            requested_action = UserAction::Focus;
        } else if (next_ || previous_ || home_ || apply_offset_) {
            requested_action = UserAction::Move;
        }
    }

    switch (requested_action) {
    case UserAction::Freedrive: {
        if (any_action_in_flight()) {
            break;
        }
        const bool enable = labview_freedrive_;
        if (freedrive_status_ == FreedriveStatus::Unconfirmed) {
            if (!request_blocked) {
                msg_ = enable ? "[Action] Confirming Freedrive Mode ON\n"
                              : "[Action] Confirming Freedrive Mode OFF\n";
            }
            send_freedrive_goal(enable);
        } else {
            const bool enabled = freedrive_status_ == FreedriveStatus::On;
            if (enable != enabled) {
                circle_state_ = 1;
                angle_ = 0.0;
                if (!request_blocked) {
                    msg_ = enable ? "[Action] Freedrive Mode ON\n"
                                  : "[Action] Freedrive Mode OFF\n";
                }
                send_freedrive_goal(enable);
            }
        }
        break;
    }
    case UserAction::Reset:
        if (!any_action_in_flight()) {
            angle_ = 0.0;
            circle_state_ = 1;
            msg_ = "[Action] Reset to default position. It may take "
                   "some time please wait.\n";
            if (send_reset_goal()) {
                RCLCPP_INFO(get_logger(), "%s", msg_.c_str());
                reset_ = false;
                action_toggle_release_required_ = true;
            }
        }
        break;
    case UserAction::Focus:
        if (autofocus_.load() && !end_state_.load()) {
            if (!any_action_in_flight()) {
                success_ = false;
                msg_ = "[Action] Focusing\n";
                if (send_focus_goal()) {
                    RCLCPP_INFO(get_logger(), "%s", msg_.c_str());
                }
            }
        } else {
            if (!success_.load()) {
                scan_3d_ = false;
                if (!action_callback_stop_.stop_requested() &&
                    goal_can_be_canceled(active_focus_goal_handle_)) {
                    (void)action_callback_stop_.request_stop();
                    msg_ = "Canceling Focus action\n";
                    RCLCPP_INFO(this->get_logger(), "%s", msg_.c_str());
                    focus_action_client_->async_cancel_goal(
                        active_focus_goal_handle_);
                } else if (action_in_flight(UserAction::Focus) &&
                           !active_focus_goal_handle_) {
                    RCLCPP_INFO(this->get_logger(),
                                "Focus request canceled before acceptance");
                    request_cancel(false);
                }
            }
        }
        break;
    case UserAction::Move:
        if (!any_action_in_flight()) {
            const bool from_recipe = full_scan_read_.load();
            Step step{};
            if (from_recipe) {
                step = full_scan_recipe_[pc_.load()];
            }

            const double offset_x = from_recipe ? step.x : offset_x_.load();
            const double offset_y = from_recipe ? step.y : offset_y_.load();

            bool apply = from_recipe ? (std::abs(offset_x) > 1e-12 ||
                                        std::abs(offset_y) > 1e-12)
                                     : apply_offset_.load();

            if (apply) {
                msg_ = std::format("[Action] Translate to (x,y): ({},{})\n",
                                   offset_x, offset_y);
            } else {
                if (from_recipe) {
                    yaw_ = step.yaw;
                } else {
                    angle_increment_ =
                        (num_pt_.load() == 0)
                            ? 0.0
                            : (angle_limit_.load() /
                               static_cast<double>(num_pt_.load()));
                    if (next_) {
                        yaw_ = angle_increment_;
                        msg_ = std::format("[Action] Next: {}\n", yaw_);
                    } else if (previous_) {
                        yaw_ = -angle_increment_;
                        msg_ = std::format("[Action] Previous: {}\n", yaw_);
                    } else if (home_) {
                        yaw_ = -angle_;
                        msg_ = std::format("[Action] Home: {}\n", yaw_);
                        circle_state_ = 1;
                    }
                }
                if (std::abs(angle_.load()) < 1e-10) {
                    circle_state_ = 1;
                }
            }
            if (send_move_goal(yaw_, offset_x, offset_y, apply)) {
                RCLCPP_INFO(get_logger(), "%s", msg_.c_str());
                previous_ = false;
                next_ = false;
                home_ = false;
                apply_offset_ = false;
            }
        }
        break;
    case UserAction::Scan:
        if (!waiting_for_scan_completion_) {
            msg_ += std::format("  [Action] Scanning\n");
            RCLCPP_INFO(get_logger(), "%s", msg_.c_str());
            waiting_for_scan_completion_ = true;
            scan_trigger_ = true;
            scan_trigger_before_request_ = scan_trigger_read_.load();
            scan_start = now();
        } else {
            if (scan_trigger_read_.load() != scan_trigger_before_request_) {
                scan_trigger_ = false;
                msg_ += "Scan Complete\n";
                RCLCPP_INFO(this->get_logger(), "%s", msg_.c_str());
                pc_.fetch_add(1);
                scan_trigger_before_request_ = scan_trigger_read_.load();
                waiting_for_scan_completion_ = false;
            }
        }
        break;
    default:
        robot_mode_ = robot_mode_read_.load();
        oct_mode_ = oct_mode_read_.load();
        octa_mode_ = octa_mode_read_.load();
        oce_mode_ = oce_mode_read_.load();
        scan_3d_ = false;
        scan_trigger_ = false;
        scan_trigger_before_request_ = scan_trigger_read_.load();
        break;
    }
}

bool CoordinatorNode::send_focus_goal() {
    if (!focus_action_client_->action_server_is_ready()) {
        msg_ = "Focus action server is not available.\n";
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                             msg_.c_str());
        return false;
    }

    FocusAction::Goal goal_msg;
    goal_msg.angle_tolerance = angle_tolerance_;
    goal_msg.z_tolerance = z_tolerance_;
    goal_msg.z_height = z_height_;

    apply_speed_scale_to_node("/focus_node", robot_vel_.load(),
                              robot_acc_.load());

    action_callback_stop_ = std::stop_source{};
    const std::stop_token stop_token = action_callback_stop_.get_token();
    auto options = rclcpp_action::Client<FocusAction>::SendGoalOptions();

    options.feedback_callback =
        [this, stop_token](
            FocusGoalHandle::SharedPtr,
            const std::shared_ptr<const FocusAction::Feedback> feedback) {
            if (stop_token.stop_requested() ||
                stop_token != action_callback_stop_.get_token()) {
                return;
            }
            msg_ += feedback->debug_msgs;
            RCLCPP_INFO(this->get_logger(), "Focus feedback => %s",
                        msg_.c_str());
        };

    options.result_callback =
        [this, stop_token](const FocusGoalHandle::WrappedResult &result) {
            if (stop_token != action_callback_stop_.get_token()) {
                RCLCPP_INFO(this->get_logger(),
                            "Ignoring result from an old Focus request");
                return;
            }
            active_focus_goal_handle_.reset();
            active_action_ = UserAction::None;
            if (stop_token.stop_requested()) {
                scan_3d_ = false;
                RCLCPP_INFO(
                    this->get_logger(),
                    "Ignoring Focus result after its goal was canceled");
                return;
            }
            scan_3d_ = false;
            msg_ += result.result->status;
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                end_state_ = true;
                success_ = true;
                RCLCPP_INFO(this->get_logger(), "Focus action SUCCEEDED");
                if (full_scan_.load()) {
                    pc_.fetch_add(1);
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                end_state_ = true;
                success_ = false;
                RCLCPP_WARN(this->get_logger(), "Focus action ABORTED");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                end_state_ = false;
                success_ = false;
                RCLCPP_WARN(this->get_logger(), "Focus action CANCELED");
                break;
            default:
                end_state_ = true;
                success_ = false;
                RCLCPP_WARN(this->get_logger(),
                            "Focus action UNKNOWN result code");
                break;
            }
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED &&
                full_scan_.load()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Fullscan canceled after terminal Focus failure");
                request_cancel(true);
            }
        };

    options.goal_response_callback =
        [this, stop_token](FocusGoalHandle::SharedPtr goal_handle) {
            if (stop_token.stop_requested() ||
                stop_token != action_callback_stop_.get_token()) {
                if (goal_handle) {
                    RCLCPP_INFO(this->get_logger(),
                                "Canceling a late Focus goal acceptance");
                    focus_action_client_->async_cancel_goal(goal_handle);
                }
                return;
            }
            active_focus_goal_handle_ = goal_handle;
            if (!active_focus_goal_handle_) {
                active_action_ = UserAction::None;
                if (!stop_token.stop_requested()) {
                    autofocus_ = false;
                    action_toggle_release_required_ = true;
                    msg_ = "Focus request rejected by action server.\n";
                    scan_3d_ = false;
                    if (full_scan_.load()) {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Fullscan canceled because its Focus goal was "
                            "rejected");
                        request_cancel(true);
                    }
                }
                RCLCPP_ERROR(this->get_logger(),
                             "Focus goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Focus goal accepted; waiting for result");
            }
        };

    active_action_ = UserAction::Focus;
    action_wait_started_at_ = std::chrono::steady_clock::now();
    try {
        focus_action_client_->async_send_goal(goal_msg, options);
    } catch (const std::exception &error) {
        (void)action_callback_stop_.request_stop();
        active_action_ = UserAction::None;
        autofocus_ = false;
        scan_3d_ = false;
        action_toggle_release_required_ = true;
        msg_ = std::format("Focus goal request failed: {}\n", error.what());
        RCLCPP_ERROR(get_logger(), "%s", msg_.c_str());
        if (full_scan_.load()) {
            request_cancel(true);
        }
        return false;
    }
    return true;
}

bool CoordinatorNode::send_move_goal(double yaw, double offset_x,
                                     double offset_y, bool apply) {
    Move::Goal goal_msg;

    goal_msg.offset_x = offset_x;
    goal_msg.offset_y = offset_y;
    goal_msg.target_angle = yaw;
    goal_msg.apply_offset = apply;
    goal_msg.radius = radius_.load();

    apply_speed_scale_to_node("/move_node", robot_vel_.load(),
                              robot_acc_.load());

    action_callback_stop_ = std::stop_source{};
    const std::stop_token stop_token = action_callback_stop_.get_token();
    auto options = rclcpp_action::Client<Move>::SendGoalOptions();

    options.feedback_callback =
        [this,
         stop_token](MoveGoalHandle::SharedPtr,
                     const std::shared_ptr<const Move::Feedback> feedback) {
            if (stop_token.stop_requested() ||
                stop_token != action_callback_stop_.get_token()) {
                return;
            }
            msg_ += feedback->debug_msgs;
        };

    options.result_callback = [this, yaw, apply, stop_token](
                                  const MoveGoalHandle::WrappedResult &result) {
        if (stop_token != action_callback_stop_.get_token()) {
            RCLCPP_INFO(this->get_logger(),
                        "Ignoring result from an old Move request");
            return;
        }
        active_move_handle_.reset();
        active_action_ = UserAction::None;
        if (stop_token.stop_requested()) {
            RCLCPP_INFO(this->get_logger(),
                        "Ignoring Move result after its goal was canceled");
            return;
        }
        msg_ += result.result->status;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (!apply) {
                if (yaw > 0.0) {
                    circle_state_++;
                } else {
                    circle_state_--;
                }
                angle_.fetch_add(yaw);
                RCLCPP_INFO(this->get_logger(), "Move SUCCEEDED");
            }
            if (full_scan_.load()) {
                pc_.fetch_add(1);
            }
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Move ABORTED");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Move CANCELED");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Move UNKNOWN code");
            break;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED &&
            full_scan_.load()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Fullscan canceled after terminal Move failure");
            request_cancel(true);
        }
    };

    options
        .goal_response_callback = [this, stop_token](
                                      MoveGoalHandle::SharedPtr goal_handle) {
        if (stop_token.stop_requested() ||
            stop_token != action_callback_stop_.get_token()) {
            if (goal_handle) {
                RCLCPP_INFO(this->get_logger(),
                            "Canceling a late Move goal acceptance");
                move_action_client_->async_cancel_goal(goal_handle);
            }
            return;
        }
        active_move_handle_ = goal_handle;
        if (!active_move_handle_) {
            active_action_ = UserAction::None;
            RCLCPP_ERROR(this->get_logger(),
                         "Move goal was rejected by server");
            if (!stop_token.stop_requested()) {
                previous_ = false;
                next_ = false;
                home_ = false;
                apply_offset_ = false;
                msg_ = "Move request rejected by action server.\n";
                if (full_scan_.load()) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Fullscan canceled because its Move goal was rejected");
                    request_cancel(true);
                }
            }
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Move goal accepted; waiting for result");
        }
    };

    active_action_ = UserAction::Move;
    action_wait_started_at_ = std::chrono::steady_clock::now();
    try {
        move_action_client_->async_send_goal(goal_msg, options);
    } catch (const std::exception &error) {
        (void)action_callback_stop_.request_stop();
        active_action_ = UserAction::None;
        msg_ = std::format("Move goal request failed: {}\n", error.what());
        RCLCPP_ERROR(get_logger(), "%s", msg_.c_str());
        if (full_scan_.load()) {
            request_cancel(true);
        } else {
            previous_ = false;
            next_ = false;
            home_ = false;
            apply_offset_ = false;
        }
        return false;
    }
    return true;
}

void CoordinatorNode::apply_speed_scale_to_node(const std::string &node_name,
                                                double vel_scale,
                                                double acc_scale) {
    vel_scale = std::clamp(vel_scale, 0.0, 1.0);
    acc_scale = std::clamp(acc_scale, 0.0, 1.0);
    try {
        auto client = std::make_shared<rclcpp::AsyncParametersClient>(
            shared_from_this(), node_name, rclcpp::ParametersQoS(),
            service_response_group_);
        if (!client->wait_for_service(std::chrono::milliseconds(250))) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Param service not available on %s; skip speed scaling",
                node_name.c_str());
            return;
        }
        std::vector<rclcpp::Parameter> params;
        params.emplace_back(
            "pilz_ptp.plan_request_params.max_velocity_scaling_factor",
            vel_scale);
        params.emplace_back(
            "pilz_ptp.plan_request_params.max_acceleration_scaling_factor",
            acc_scale);
        params.emplace_back(
            "pilz_lin.plan_request_params.max_velocity_scaling_factor",
            vel_scale);
        params.emplace_back(
            "pilz_lin.plan_request_params.max_acceleration_scaling_factor",
            acc_scale);
        auto fut = client->set_parameters(params);
        (void)fut.wait_for(std::chrono::milliseconds(250));
    } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(),
                    "Failed to set speed scaling parameters on %s: %s",
                    node_name.c_str(), e.what());
    }
}

void CoordinatorNode::send_freedrive_goal(bool enable) {
    const auto request_time = std::chrono::steady_clock::now();
    if (request_time - action_wait_started_at_ < kFreedriveRetryInterval) {
        return;
    }
    action_wait_started_at_ = request_time;

    if (!freedrive_action_client_->action_server_is_ready()) {
        msg_ = "Freedrive action server is not available.\n";
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                             msg_.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "Requesting Freedrive Mode %s",
                enable ? "ON" : "OFF");

    Freedrive::Goal goal_msg;
    goal_msg.enable = enable;

    action_callback_stop_ = std::stop_source{};
    const std::stop_token stop_token = action_callback_stop_.get_token();
    auto options = rclcpp_action::Client<Freedrive>::SendGoalOptions();

    options.feedback_callback =
        [this, stop_token](
            FreedriveGoalHandle::SharedPtr,
            const std::shared_ptr<const Freedrive::Feedback> feedback) {
            if (stop_token.stop_requested() ||
                stop_token != action_callback_stop_.get_token()) {
                return;
            }
            msg_ += feedback->debug_msgs;
            RCLCPP_INFO(this->get_logger(), "Freedrive feedback => %s",
                        feedback->debug_msgs.c_str());
        };

    options.result_callback =
        [this, enable,
         stop_token](const FreedriveGoalHandle::WrappedResult &result) {
            if (stop_token != action_callback_stop_.get_token()) {
                RCLCPP_INFO(this->get_logger(),
                            "Ignoring result from an old Freedrive request");
                return;
            }
            active_freedrive_goal_handle_.reset();
            active_action_ = UserAction::None;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                freedrive_status_ =
                    enable ? FreedriveStatus::On : FreedriveStatus::Off;
            } else {
                freedrive_status_ = FreedriveStatus::Unconfirmed;
            }
            if (stop_token.stop_requested()) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Ignoring Freedrive result after its goal was canceled");
                return;
            }
            msg_ += result.result->status;
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Freedrive SUCCESS");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(),
                            "Freedrive state not confirmed; retrying");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Freedrive CANCELED");
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Freedrive UNKNOWN code");
                break;
            }
        };

    options.goal_response_callback =
        [this, stop_token](FreedriveGoalHandle::SharedPtr goal_handle) {
            if (stop_token.stop_requested() ||
                stop_token != action_callback_stop_.get_token()) {
                if (goal_handle) {
                    RCLCPP_INFO(this->get_logger(),
                                "Canceling a late Freedrive goal acceptance");
                    freedrive_action_client_->async_cancel_goal(goal_handle);
                }
                return;
            }
            active_freedrive_goal_handle_ = goal_handle;
            if (!active_freedrive_goal_handle_) {
                active_action_ = UserAction::None;
                RCLCPP_ERROR(this->get_logger(),
                             "Freedrive goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Freedrive goal accepted; waiting for result");
            }
        };

    active_action_ = UserAction::Freedrive;
    try {
        freedrive_action_client_->async_send_goal(goal_msg, options);
    } catch (const std::exception &error) {
        (void)action_callback_stop_.request_stop();
        active_action_ = UserAction::None;
        freedrive_status_ = FreedriveStatus::Unconfirmed;
        msg_ = std::format("Freedrive goal request failed: {}\n", error.what());
        RCLCPP_ERROR(get_logger(), "%s", msg_.c_str());
    }
}

bool CoordinatorNode::send_reset_goal() {
    Reset::Goal goal_msg;
    goal_msg.reset = true;

    action_callback_stop_ = std::stop_source{};
    const std::stop_token stop_token = action_callback_stop_.get_token();
    auto options = rclcpp_action::Client<Reset>::SendGoalOptions();

    options.feedback_callback =
        [this,
         stop_token](ResetGoalHandle::SharedPtr,
                     const std::shared_ptr<const Reset::Feedback> feedback) {
            if (stop_token.stop_requested() ||
                stop_token != action_callback_stop_.get_token()) {
                return;
            }
            msg_ += feedback->debug_msgs;
            RCLCPP_INFO(this->get_logger(), "Reset feedback => %s",
                        feedback->debug_msgs.c_str());
        };

    options.result_callback =
        [this, stop_token](const ResetGoalHandle::WrappedResult &result) {
            if (stop_token != action_callback_stop_.get_token()) {
                RCLCPP_INFO(this->get_logger(),
                            "Ignoring result from an old Reset request");
                return;
            }
            active_reset_goal_handle_.reset();
            active_action_ = UserAction::None;
            if (stop_token.stop_requested()) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Ignoring Reset result after its goal was canceled");
                return;
            }
            msg_ += result.result->status;
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Reset SUCCESS");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                msg_ += "\nReset position abort\n";
                RCLCPP_WARN(this->get_logger(), "Reset ABORTED");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                msg_ += "\nReset position canceled\n";
                RCLCPP_WARN(this->get_logger(), "Reset CANCELED");
                break;
            default:
                msg_ += "\nReset position unknown code\n";
                RCLCPP_WARN(this->get_logger(), "Reset UNKNOWN code");
                break;
            }
        };

    options.goal_response_callback =
        [this, stop_token](ResetGoalHandle::SharedPtr goal_handle) {
            if (stop_token.stop_requested() ||
                stop_token != action_callback_stop_.get_token()) {
                if (goal_handle) {
                    RCLCPP_INFO(this->get_logger(),
                                "Canceling a late Reset goal acceptance");
                    reset_action_client_->async_cancel_goal(goal_handle);
                }
                return;
            }
            active_reset_goal_handle_ = goal_handle;
            if (!active_reset_goal_handle_) {
                active_action_ = UserAction::None;
                RCLCPP_ERROR(this->get_logger(),
                             "Reset goal was rejected by server");
                if (!stop_token.stop_requested()) {
                    reset_ = false;
                    action_toggle_release_required_ = true;
                    msg_ = "Reset request rejected by action server.\n";
                }
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Reset goal accepted; waiting for result");
            }
        };

    freedrive_status_ = FreedriveStatus::Unconfirmed;
    active_action_ = UserAction::Reset;
    action_wait_started_at_ = std::chrono::steady_clock::now();
    try {
        reset_action_client_->async_send_goal(goal_msg, options);
    } catch (const std::exception &error) {
        (void)action_callback_stop_.request_stop();
        active_action_ = UserAction::None;
        reset_ = false;
        action_toggle_release_required_ = true;
        msg_ = std::format("Reset goal request failed: {}\n", error.what());
        RCLCPP_ERROR(get_logger(), "%s", msg_.c_str());
        return false;
    }
    return true;
}

void CoordinatorNode::scan3d_callback(
    const std::shared_ptr<Scan3d::Request> request,
    std::shared_ptr<Scan3d::Response> response) {
    const bool requested_state = request->activate;
    if (requested_state && (active_action_.load() != UserAction::Focus ||
                            cancel_in_progress_.load())) {
        scan_3d_ = false;
        response->success = false;
        return;
    }
    scan_3d_.store(requested_state);

    if (requested_state && (active_action_.load() != UserAction::Focus ||
                            cancel_in_progress_.load())) {
        scan_3d_ = false;
        response->success = false;
        return;
    }

    if (scan_3d_read_.load() != requested_state) {
        response->success = false;
        return;
    }

    if (requested_state) {
        rclcpp::sleep_for(std::chrono::milliseconds(scan3d_window_ms_));
        if (!scan_3d_.load() || active_action_.load() != UserAction::Focus ||
            cancel_in_progress_.load() || !scan_3d_read_.load()) {
            scan_3d_ = false;
            response->success = false;
            return;
        }
    }

    response->success = true;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinatorNode>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
