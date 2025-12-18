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
#include <format>
#include <mutex>
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
    parallel_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
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
        options.callback_group = parallel_group_;
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
        options.callback_group = parallel_group_;
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
            qos, parallel_group_);
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
    if (!this->has_parameter("service_poll_interval_ms")) {
        this->declare_parameter<int64_t>("service_poll_interval_ms",
                                         service_poll_interval_ms_);
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
    service_poll_interval_ms_ =
        this->get_parameter("service_poll_interval_ms").as_int();

    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(pub_period_ms_),
        [this]() { this->publisher_callback(); }, parallel_group_);

    main_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(main_loop_period_ms_),
        [this]() { this->main_loop(); }, parallel_group_);

    focus_action_client_ =
        rclcpp_action::create_client<FocusAction>(this, action_focus_name);
    move_action_client_ =
        rclcpp_action::create_client<Move>(this, action_move_name);
    freedrive_action_client_ =
        rclcpp_action::create_client<Freedrive>(this, action_freedrive_name);
    reset_action_client_ =
        rclcpp_action::create_client<Reset>(this, action_reset_name);
#ifdef LEGACY_IMG_PIPELINE
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        capture_background_client_ = create_client<std_srvs::srv::Trigger>(
            "capture_background", qos, parallel_group_);
    }
#endif

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

    // Parameter update callback
    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult res;
            res.successful = true;
            res.reason = "";
            auto must_be_positive_int = [&](const rclcpp::Parameter &param) {
                if (param.as_int() <= 0) {
                    res.successful = false;
                    res.reason = std::string(param.get_name()) + " must be > 0";
                    return false;
                }
                return true;
            };
            for (const auto &param : params) {
                const auto &name = param.get_name();
                if (name == "pub_period_ms" || name == "main_loop_period_ms" ||
                    name == "action_server_wait_ms" ||
                    name == "config_apply_ms" || name == "scan3d_window_ms" ||
                    name == "service_poll_interval_ms") {
                    if (!must_be_positive_int(param)) {
                        return res;
                    }
                } else if (name == "scan_trigger_timeout_sec") {
                    if (param.as_double() < 0.0) {
                        res.successful = false;
                        res.reason = "scan_trigger_timeout_sec must be >= 0";
                        return res;
                    }
                }
            }
            for (const auto &param : params) {
                const auto &name = param.get_name();
                if (name == "pub_period_ms") {
                    pub_period_ms_ = param.as_int();
                } else if (name == "main_loop_period_ms") {
                    main_loop_period_ms_ = param.as_int();
                } else if (name == "action_server_wait_ms") {
                    action_server_wait_ms_ = param.as_int();
                } else if (name == "config_apply_ms") {
                    config_apply_ms_ = param.as_int();
                } else if (name == "scan3d_window_ms") {
                    scan3d_window_ms_ = param.as_int();
                } else if (name == "service_poll_interval_ms") {
                    service_poll_interval_ms_ = param.as_int();
                } else if (name == "scan_trigger_timeout_sec") {
                    scan_trigger_timeout_sec_ = param.as_double();
                }
            }
            return res;
        });

    full_scan_recipe_.clear();
    full_scan_plan_stale_ = true;
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

std::vector<Step> CoordinatorNode::build_full_scan_recipe() const {
    const double angle_init = angle_init_.load();
    const double angle_limit = angle_limit_.load();
    const int num_pt = std::max(1, num_pt_.load());
    const int n_oct = std::max(0, n_oct_.load());
    const bool apply_octa = apply_octa_.load();
    const double sweep = angle_limit - angle_init;
    const double angle_increment =
        sweep / static_cast<double>(std::max(1, num_pt));
    const double scan_offset = radius_.load() - 4.3;

    std::vector<Step> recipe;
    recipe.push_back(
        {.action = UserAction::Focus, .mode = Mode::ROBOT, .yaw = 0.0});
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
            recipe.push_back(
                {.action = UserAction::Focus, .mode = Mode::ROBOT});
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
    autofocus_ = msg->autofocus;
    freedrive_ = msg->freedrive;
    reset_ = msg->reset;
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
    {
        const bool prev_full_scan = full_scan_read_.load();
        if (msg->full_scan != prev_full_scan) {
            if (!full_scan_delay_timer_active_) {
                full_scan_delay_since_ = std::chrono::steady_clock::now();
                full_scan_delay_timer_active_ = true;
            }
            const auto elapsed =
                std::chrono::steady_clock::now() - full_scan_delay_since_;
            if (elapsed >= kFullScanSwitchDelay) {
                full_scan_read_ = msg->full_scan;
                full_scan_delay_timer_active_ = false;
                if (!prev_full_scan && full_scan_read_) {
                    pc_ = 0;
                    full_scan_center_seeded_ = false;
                    angle_ = 0.0;
                    circle_state_ = 1;
                    scan_trigger_store_ = scan_trigger_read_.load();
                }
            }
        } else {
            full_scan_delay_timer_active_ = false;
        }
    }
    robot_mode_read_ = msg->robot_mode;
    oct_mode_read_ = msg->oct_mode;
    octa_mode_read_ = msg->octa_mode;
    oce_mode_read_ = msg->oce_mode;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const bool recipe_param_changed =
            msg->angle_limit != old_sub_msg_.angle_limit ||
            msg->num_pt != old_sub_msg_.num_pt ||
            msg->angle_init != old_sub_msg_.angle_init ||
            msg->n_oct != old_sub_msg_.n_oct ||
            msg->apply_octa != old_sub_msg_.apply_octa;
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
            log_if_changed(msg->angle_init, old_sub_msg_.angle_init,
                           "angle_init", sub_log);
            log_if_changed(msg->n_oct, old_sub_msg_.n_oct, "n_oct", sub_log);
            log_if_changed(msg->apply_octa, old_sub_msg_.apply_octa,
                           "apply_octa", sub_log);
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
            log_if_changed(msg->scan_3d, old_sub_msg_.scan_3d, "scan_3d",
                           sub_log);
            log_if_changed(msg->z_height, old_sub_msg_.z_height, "z_height",
                           sub_log);
            log_if_changed(msg->full_scan, old_sub_msg_.full_scan, "full_scan",
                           sub_log);
            log_if_changed(msg->robot_mode, old_sub_msg_.robot_mode,
                           "robot_mode", sub_log);
            log_if_changed(msg->oct_mode, old_sub_msg_.oct_mode, "oct_mode",
                           sub_log);
            log_if_changed(msg->octa_mode, old_sub_msg_.octa_mode, "octa_mode",
                           sub_log);
            log_if_changed(msg->oce_mode, old_sub_msg_.oce_mode, "oce_mode",
                           sub_log);

            RCLCPP_INFO(get_logger(), sub_log.str().c_str());
        }
        if (!autofocus_.load()) {
            end_state_ = false;
        }
        if (!full_scan_read_) {
            pc_ = 0;
        }
        if (recipe_param_changed) {
            pc_ = 0;
            full_scan_plan_stale_ = true;
        }
        old_sub_msg_ = *msg;
    }
}

void CoordinatorNode::cancel_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    cancel_action_ = msg->data;
    if (cancel_action_) {
        autofocus_ = false;
        full_scan_ = false;
    }
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

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
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
            log_if_changed(msg.scan_3d, old_pub_msg_.scan_3d, "scan_3d",
                           pub_log);
            log_if_changed(msg.full_scan, old_pub_msg_.full_scan, "full_scan",
                           pub_log);
            log_if_changed(msg.robot_mode, old_pub_msg_.robot_mode,
                           "robot_mode", pub_log);
            log_if_changed(msg.oct_mode, old_pub_msg_.oct_mode, "oct_mode",
                           pub_log);
            log_if_changed(msg.octa_mode, old_pub_msg_.octa_mode, "octa_mode",
                           pub_log);
            log_if_changed(msg.oce_mode, old_pub_msg_.oce_mode, "oce_mode",
                           pub_log);

            RCLCPP_INFO(get_logger(), pub_log.str().c_str());
        }
        if (scan_trigger_.load() &&
            (now() - scan_start).seconds() > scan_trigger_timeout_sec_) {
            scan_trigger_ = false;
        }
        pub_handle_->publish(msg);
        old_pub_msg_ = msg;
    }
}

void CoordinatorNode::main_loop() {
    if (!ur_driver_healthy_.load()) {
        return;
    }

    if (cancel_action_) {
        if (goal_still_active(active_focus_goal_handle_)) {
            msg_ = "Canceling Focus action\n";
            RCLCPP_INFO(this->get_logger(), msg_.c_str());
            focus_action_client_->async_cancel_goal(active_focus_goal_handle_);
        }
        if (goal_still_active(active_move_handle_)) {
            msg_ = "Canceling Move action\n";
            RCLCPP_INFO(this->get_logger(), msg_.c_str());
            move_action_client_->async_cancel_goal(active_move_handle_);
        }
        if (goal_still_active(active_freedrive_goal_handle_)) {
            msg_ = "Canceling Free-drive\n";
            RCLCPP_INFO(this->get_logger(), msg_.c_str());
            freedrive_action_client_->async_cancel_goal(
                active_freedrive_goal_handle_);
        }
        if (goal_still_active(active_reset_goal_handle_)) {
            msg_ = "Canceling Reset action\n";
            RCLCPP_INFO(this->get_logger(), msg_.c_str());
            reset_action_client_->async_cancel_goal(active_reset_goal_handle_);
        }
        if (full_scan_read_) {
            full_scan_ = false;
            msg_ = "Canceling Full Scan action\n";
            RCLCPP_INFO(this->get_logger(), msg_.c_str());
            full_scan_recipe_.clear();
        }
        pc_ = 0;
        current_action_ = UserAction::None;
        previous_action_ = UserAction::None;
        cancel_action_ = false;
        success_ = false;
        triggered_service_ = false;
        scan_trigger_store_ = scan_trigger_read_.load();
        robot_mode_ = true;
        octa_mode_ = false;
        oct_mode_ = false;
        oce_mode_ = false;
        trigger_apply_config();
        return;
    }

    if (full_scan_read_) {
        if (!full_scan_ &&
            (full_scan_plan_stale_ || full_scan_recipe_.empty())) {
            full_scan_recipe_ = build_full_scan_recipe();
            full_scan_plan_stale_ = false;
            full_scan_center_seeded_ = false;
            pc_ = 0;
        }
        full_scan_ = true;
        if (full_scan_recipe_.empty()) {
            msg_ = "Full Scan recipe is empty; aborting.\n";
            full_scan_ = false;
            return;
        }
        if ((pc_.load() + 1) > full_scan_recipe_.size()) {
            full_scan_ = false;
            msg_ = "Full Scan complete!\n";
            if (full_scan_plan_stale_) {
                full_scan_recipe_ = build_full_scan_recipe();
                full_scan_plan_stale_ = false;
            }
            return;
        }
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

        yaw_ = step.yaw;

        if (full_scan_ && step.action == UserAction::Move &&
            !full_scan_center_seeded_) {
            home_ = true;
            angle_ = 0.0;
            circle_state_ = 1;
            full_scan_center_seeded_ = true;
        }

        current_action_ = step.action;
        autofocus_ = (current_action_ == UserAction::Focus);
    } else {
        if (freedrive_) {
            current_action_ = UserAction::Freedrive;
        } else if (reset_) {
            current_action_ = UserAction::Reset;
        } else if (autofocus_) {
            current_action_ = UserAction::Focus;
        } else if (next_ || previous_ || home_ || apply_offset_) {
            current_action_ = UserAction::Move;
        }
    }

    switch (current_action_) {
    case UserAction::Freedrive:
        if (freedrive_) {
            if (!goal_still_active(active_freedrive_goal_handle_) &&
                previous_action_ != UserAction::Freedrive) {
                circle_state_ = 1;
                angle_ = 0.0;
                msg_ = "[Action] Freedrive Mode ON\n";
                RCLCPP_INFO(get_logger(), msg_.c_str());
                previous_action_ = UserAction::Freedrive;
                send_freedrive_goal(true);
            }
        } else {
            if (goal_still_active(active_freedrive_goal_handle_) ||
                previous_action_ == UserAction::Freedrive) {
                msg_ = "[Action] Freedrive Mode OFF\n";
                RCLCPP_INFO(get_logger(), msg_.c_str());
                send_freedrive_goal(false);
            }
            current_action_ = UserAction::None;
            previous_action_ = UserAction::None;
        }
        break;
    case UserAction::Reset:
        if (!goal_still_active(active_reset_goal_handle_) &&
            previous_action_ != UserAction::Reset) {
            angle_ = 0.0;
            circle_state_ = 1;
            msg_ = "[Action] Reset to default position. It may take "
                   "some time please wait.\n";
            RCLCPP_INFO(get_logger(), msg_.c_str());
            previous_action_ = UserAction::Reset;
            send_reset_goal();
        }
        break;
    case UserAction::Focus:
        if (autofocus_.load() && !end_state_.load()) {
            if (!goal_still_active(active_focus_goal_handle_) &&
                previous_action_ != UserAction::Focus) {
                success_ = false;
                msg_ = "[Action] Focusing\n";
                RCLCPP_INFO(get_logger(), msg_.c_str());
                previous_action_ = UserAction::Focus;
                send_focus_goal();
            }
        } else {
            if (!success_.load()) {
                msg_ = "Canceling Focus action\n";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                if (goal_still_active(active_focus_goal_handle_)) {
                    focus_action_client_->async_cancel_goal(
                        active_focus_goal_handle_);
                }
            }
        }
        break;
    case UserAction::Move:
        if (!goal_still_active(active_move_handle_) &&
            previous_action_ != UserAction::Move) {
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
                angle_increment_ = (num_pt_.load() == 0)
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
                if (std::abs(angle_.load()) < 1e-10) {
                    circle_state_ = 1;
                }
            }
            RCLCPP_INFO(get_logger(), msg_.c_str());
            previous_action_ = UserAction::Move;
            current_action_ = UserAction::None;
            send_move_goal(yaw_, offset_x, offset_y, apply);
            home_ = false;
        }
        break;
    case UserAction::Scan:
        if (previous_action_ != UserAction::Scan) {
            msg_ += std::format("  [Action] Scanning\n");
            RCLCPP_INFO(get_logger(), msg_.c_str());
            previous_action_ = UserAction::Scan;
            scan_trigger_ = true;
            scan_trigger_store_ = scan_trigger_read_.load();
            scan_start = now();
        } else {
            if (scan_trigger_read_.load() != scan_trigger_store_) {
                scan_trigger_ = false;
                msg_ += "Scan Complete\n";
                RCLCPP_INFO(this->get_logger(), msg_.c_str());
                pc_.fetch_add(1);
                scan_trigger_store_ = scan_trigger_read_.load();
                previous_action_ = UserAction::None;
                current_action_ = UserAction::None;
            }
        }
        break;
    default:
        robot_mode_ = robot_mode_read_.load();
        oct_mode_ = oct_mode_read_.load();
        octa_mode_ = octa_mode_read_.load();
        oce_mode_ = oce_mode_read_.load();
        scan_3d_ = false;
        triggered_service_ = false;
        scan_trigger_ = false;
        scan_trigger_store_ = scan_trigger_read_.load();
        break;
    }
}

void CoordinatorNode::send_focus_goal() {
    FocusAction::Goal goal_msg;
    goal_msg.angle_tolerance = angle_tolerance_;
    goal_msg.z_tolerance = z_tolerance_;
    goal_msg.z_height = z_height_;

    apply_speed_scale_to_node("/focus_node", robot_vel_.load(),
                              robot_acc_.load());

    auto options = rclcpp_action::Client<FocusAction>::SendGoalOptions();

    options.feedback_callback =
        [this](FocusGoalHandle::SharedPtr,
               const std::shared_ptr<const FocusAction::Feedback> feedback) {
            msg_ += feedback->debug_msgs;
            RCLCPP_INFO(this->get_logger(), "Focus feedback => %s",
                        msg_.c_str());
        };

    options.result_callback =
        [this](const FocusGoalHandle::WrappedResult &result) {
            current_action_ = UserAction::None;
            previous_action_ = UserAction::None;
            msg_ += result.result->status;
            end_state_ = true;
            success_ = true;
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Focus action SUCCEEDED");
                if (full_scan_) {
                    pc_.fetch_add(1);
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Focus action ABORTED");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Focus action CANCELED");
                break;
            default:
                RCLCPP_WARN(this->get_logger(),
                            "Focus action UNKNOWN result code");
                break;
            }
            active_focus_goal_handle_.reset();
        };

    options.goal_response_callback =
        [this](FocusGoalHandle::SharedPtr goal_handle) {
            active_focus_goal_handle_ = goal_handle;
            if (!active_focus_goal_handle_) {
                RCLCPP_ERROR(this->get_logger(),
                             "Focus goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Focus goal accepted; waiting for result");
            }
        };

    focus_action_client_->async_send_goal(goal_msg, options);
}

void CoordinatorNode::send_move_goal(double yaw, double offset_x,
                                     double offset_y, bool apply) {
    Move::Goal goal_msg;

    goal_msg.offset_x = offset_x;
    goal_msg.offset_y = offset_y;
    goal_msg.target_angle = yaw;
    goal_msg.apply_offset = apply;
    goal_msg.radius = radius_.load();
    goal_msg.angle = angle_.load();
    goal_msg.centre_set = home_.load();

    apply_speed_scale_to_node("/move_node", robot_vel_.load(),
                              robot_acc_.load());

    auto options = rclcpp_action::Client<Move>::SendGoalOptions();

    options.feedback_callback =
        [this](MoveGoalHandle::SharedPtr,
               const std::shared_ptr<const Move::Feedback> feedback) {
            msg_ += feedback->debug_msgs;
        };

    options.result_callback =
        [this, yaw, apply](const MoveGoalHandle::WrappedResult &result) {
            current_action_ = UserAction::None;
            previous_action_ = UserAction::None;
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
                if (full_scan_) {
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
            active_move_handle_.reset();
        };

    options.goal_response_callback =
        [this](MoveGoalHandle::SharedPtr goal_handle) {
            active_move_handle_ = goal_handle;
            if (!active_move_handle_) {
                RCLCPP_ERROR(this->get_logger(),
                             "Move goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Move goal accepted; waiting for result");
            }
        };

    move_action_client_->async_send_goal(goal_msg, options);
}

void CoordinatorNode::apply_speed_scale_to_node(const std::string &node_name,
                                                double vel_scale,
                                                double acc_scale) {
    vel_scale = std::clamp(vel_scale, 0.0, 1.0);
    acc_scale = std::clamp(acc_scale, 0.0, 1.0);
    try {
        auto client = std::make_shared<rclcpp::AsyncParametersClient>(
            shared_from_this(), node_name);
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
    Freedrive::Goal goal_msg;
    goal_msg.enable = enable;

    auto options = rclcpp_action::Client<Freedrive>::SendGoalOptions();

    options.feedback_callback =
        [this](FreedriveGoalHandle::SharedPtr,
               const std::shared_ptr<const Freedrive::Feedback> feedback) {
            msg_ += feedback->debug_msgs;
            RCLCPP_INFO(this->get_logger(), "Freedrive feedback => %s",
                        feedback->debug_msgs.c_str());
        };

    options.result_callback =
        [this](const FreedriveGoalHandle::WrappedResult &result) {
            msg_ += result.result->status;
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Freedrive SUCCESS");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Freedrive ABORTED");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Freedrive CANCELED");
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Freedrive UNKNOWN code");
                break;
            }
            active_freedrive_goal_handle_.reset();
        };

    options.goal_response_callback =
        [this](FreedriveGoalHandle::SharedPtr goal_handle) {
            active_freedrive_goal_handle_ = goal_handle;
            if (!active_freedrive_goal_handle_) {
                RCLCPP_ERROR(this->get_logger(),
                             " Freedrive goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(),
                            " Freedrive goal accepted; waiting for result");
            }
        };

    freedrive_action_client_->async_send_goal(goal_msg, options);
}

void CoordinatorNode::send_reset_goal() {
    Reset::Goal goal_msg;
    goal_msg.reset = true;

    auto options = rclcpp_action::Client<Reset>::SendGoalOptions();

    options.feedback_callback =
        [this](ResetGoalHandle::SharedPtr,
               const std::shared_ptr<const Reset::Feedback> feedback) {
            msg_ += feedback->debug_msgs;
            RCLCPP_INFO(this->get_logger(), "Reset feedback => %s",
                        feedback->debug_msgs.c_str());
        };

    options.result_callback =
        [this](const ResetGoalHandle::WrappedResult &result) {
            current_action_ = UserAction::None;
            previous_action_ = UserAction::None;
            msg_ += result.result->status;
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Reset SUCCESS");
#ifdef LEGACY_IMG_PIPELINE
                if (call_capture_background()) {
                    msg_ += "\nBackground Captured\n";
                    RCLCPP_INFO(this->get_logger(),
                                "Background capture succeeded after reset.");
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "Background capture after reset failed.");
                }
#endif
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
            active_reset_goal_handle_.reset();
        };

    options.goal_response_callback =
        [this](ResetGoalHandle::SharedPtr goal_handle) {
            active_reset_goal_handle_ = goal_handle;
            if (!active_reset_goal_handle_) {
                RCLCPP_ERROR(this->get_logger(),
                             " Reset goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(),
                            " Reset goal accepted; waiting for result");
            }
        };

    reset_action_client_->async_send_goal(goal_msg, options);
}

#ifdef LEGACY_IMG_PIPELINE
bool CoordinatorNode::call_capture_background() {
    if (!capture_background_client_) {
        return false;
    }
    if (!capture_background_client_->wait_for_service(
            std::chrono::milliseconds(service_poll_interval_ms_))) {
        return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = capture_background_client_->async_send_request(req);
    return fut.wait_for(std::chrono::milliseconds(1000)) ==
               std::future_status::ready &&
           fut.get()->success;
}
#endif
void CoordinatorNode::scan3d_callback(
    const std::shared_ptr<Scan3d::Request> request,
    std::shared_ptr<Scan3d::Response> response) {
    if (!triggered_service_) {
        scan_3d_ = request->activate;
        triggered_service_ = true;
    }
    if (request->activate) {
        if (scan_3d_read_) {
            rclcpp::Time deadline =
                now() + rclcpp::Duration(0, scan3d_window_ms_ * 1'000'000);
            while (now() < deadline && !scan_3d_read_) {
                rclcpp::spin_some(get_node_base_interface());
                rclcpp::sleep_for(
                    std::chrono::milliseconds(service_poll_interval_ms_));
            }

            response->success = true;
            triggered_service_ = false;
        } else {
            response->success = false;
        }
    } else {
        if (!scan_3d_read_) {
            response->success = true;
            triggered_service_ = false;
        } else {
            response->success = false;
        }
    }
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
