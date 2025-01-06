#include <Eigen/Dense>
#include <chrono>
#include <csignal>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "dds_publisher.hpp"
#include "dds_subscriber.hpp"
#include "img_subscriber.hpp"
#include "urscript_publisher.hpp"
#include "utils.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
std::atomic<bool> running(true);

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"),
                "Signal %d received, shutting down...", signum);
    running = false;
    rclcpp::shutdown();
}

struct SegmentResult {
    cv::Mat image;
    std::vector<cv::Point> coordinates;
};

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord) {
    for (size_t i = 0; i < ret_coord.size() - 1; ++i) {
        cv::Point pt1 = ret_coord[i];
        cv::Point pt2 = ret_coord[i + 1];
        cv::line(image, pt1, pt2, cv::Scalar(255, 0, 0), 2);
    }
}

std::vector<double> kalmanFilter1D(const std::vector<double> &observations,
                                   double Q = 0.01, double R = 0.5) {
    std::vector<double> x_k_estimates;
    x_k_estimates.reserve(observations.size());
    double x_k = (observations.empty()) ? 0.0 : observations[0];
    double P_k = 1.0;
    for (double z_k : observations) {
        double x_k_pred = x_k;
        double P_k_pred = P_k + Q;
        double K_k = P_k_pred / (P_k_pred + R);
        x_k = x_k_pred + K_k * (z_k - x_k_pred);
        P_k = (1.0 - K_k) * P_k_pred;
        x_k_estimates.push_back(x_k);
    }
    return x_k_estimates;
}

void removeOutliers(std::vector<double> &vals, double z_threshold = 0.5) {
    if (vals.empty())
        return;
    size_t obs_length = vals.size();
    int window = std::max(1, (int)obs_length / 10);
    for (size_t i = 0; i < obs_length; ++i) {
        int start = std::max(0, (int)i - window);
        int end = std::min((int)obs_length, (int)i + window + 1);
        std::vector<double> local(vals.begin() + start, vals.begin() + end);
        double mu =
            std::accumulate(local.begin(), local.end(), 0.0) / local.size();
        double accum = 0.0;
        for (double val : local) {
            accum += (val - mu) * (val - mu);
        }
        double sigma = std::sqrt(accum / local.size());
        std::nth_element(local.begin(), local.begin() + local.size() / 2,
                         local.end());
        double med = local[local.size() / 2];
        double z = (sigma != 0.0) ? (vals[i] - mu) / sigma : 0.0;
        if (z > z_threshold) {
            vals[i] = med;
        }
    }
}

std::vector<cv::Point> get_max_coor(const cv::Mat &img) {
    std::vector<cv::Point> ret_coords;
    // int height = img.rows;
    int width = img.cols;

    for (int x = 0; x < width; ++x) {
        cv::Mat intensity = img.col(x);
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(intensity, &minVal, &maxVal, &minLoc, &maxLoc);
        int detected_y = maxLoc.y;
        ret_coords.emplace_back(x, detected_y);
    }
    return ret_coords;
}

SegmentResult detect_lines(const cv::Mat &inputImg) {
    CV_Assert(!inputImg.empty());
    cv::Mat gray;
    if (inputImg.channels() == 3) {
        cv::cvtColor(inputImg, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = inputImg.clone();
    }

    cv::Mat denoised;
    cv::Mat denoised_f;
    cv::medianBlur(gray, denoised, 5);
    denoised.convertTo(denoised_f, CV_32F);

    cv::Mat row_means;
    cv::reduce(denoised_f, row_means, 1, cv::REDUCE_AVG, CV_32F);
    cv::Mat mean_mat;
    cv::repeat(row_means, 1, denoised_f.cols, mean_mat);
    denoised_f -= mean_mat;

    cv::Mat img_median1d;
    denoised_f.convertTo(img_median1d, CV_8U, 1.0, 128.0);
    cv::medianBlur(img_median1d, img_median1d, 3);
    img_median1d.convertTo(img_median1d, CV_32F, 1.0, -128.0);

    cv::Mat img_gauss;
    cv::GaussianBlur(img_median1d, img_gauss, cv::Size(0, 0), 3.0, 3.0);

    cv::Mat sobely;
    cv::Sobel(img_gauss, sobely, CV_32F, 0, 1, 3);

    std::vector<cv::Point> rawCoords = get_max_coor(sobely);

    std::vector<double> observations;
    observations.reserve(rawCoords.size());
    for (const auto &pt : rawCoords) {
        observations.push_back(static_cast<double>(pt.y));
    }

    removeOutliers(observations, 0.5);
    std::vector<double> x_est = kalmanFilter1D(observations, 0.01, 0.5);
    std::vector<cv::Point> ret_coords(rawCoords.size());
    for (size_t i = 0; i < x_est.size(); ++i) {
        ret_coords[i] = cv::Point(static_cast<int>(i),
                                  static_cast<int>(std::round(x_est[i])));
    }

    cv::Mat detected_img;
    gray.convertTo(detected_img, CV_8U);
    draw_line(detected_img, ret_coords);

    SegmentResult result;
    result.image = detected_img;
    result.coordinates = ret_coords;
    return result;
}

std::vector<Eigen::Vector3d> lines_3d(const std::vector<cv::Mat> &img_array,
                                      const int interval,
                                      const bool acq_interval = false) {
    std::vector<Eigen::Vector3d> pc_3d;
    int num_frames = interval > 1 ? interval : 2;
    double increments = 499.0 / static_cast<double>(num_frames - 1);

    for (size_t i = 0; i < img_array.size(); ++i) {
        cv::Mat img = img_array[i];
        cv::imwrite(std::format("raw_image{}.jpg", i).c_str(), img);
        SegmentResult pc = detect_lines(img);
        cv::imwrite(std::format("detected_image{}.jpg", i).c_str(), pc.image);
        assert(!pc.coordinates.empty());

        int idx = static_cast<int>(i) % interval;
        double z_val = idx * increments;

        for (size_t j = 0; j < pc.coordinates.size(); ++j) {
            double x = static_cast<double>(pc.coordinates[j].x);
            double y = static_cast<double>(pc.coordinates[j].y);
            pc_3d.emplace_back(Eigen::Vector3d(x, y, z_val));
        }

        if (acq_interval && pc_3d.size() >= static_cast<size_t>(interval)) {
            break;
        }
    }

    return pc_3d;
}

Eigen::Matrix3d align_to_direction(const Eigen::Matrix3d &rot_matrix) {
    Eigen::Matrix3d out_matrix = Eigen::Matrix3d::Zero();
    for (int col = 0; col < 3; ++col) {
        int max_idx;
        rot_matrix.col(col).cwiseAbs().maxCoeff(&max_idx);
        out_matrix.col(max_idx) = rot_matrix.col(col);
    }
    for (int col = 0; col < 3; ++col) {
        if (out_matrix(col, col) < 0) {
            out_matrix.col(col) *= -1.0;
        }
    }
    return out_matrix;
}

bool tol_measure(double &roll, double &pitch, double &angle_tolerance) {
    return ((std::abs(std::abs(roll)) < to_radian(angle_tolerance)) &&
            (std::abs(std::abs(pitch)) < to_radian(angle_tolerance)));
}

void print_target(auto &logger, geometry_msgs::msg::Pose target_pose) {
    RCLCPP_INFO(logger,
                std::format("Target Pose: "
                            " x: {}, y: {}, z: {},"
                            " qx: {}, qy: {}, qz: {}, qw: {}",
                            target_pose.position.x, target_pose.position.y,
                            target_pose.position.z, target_pose.orientation.x,
                            target_pose.orientation.y,
                            target_pose.orientation.z,
                            target_pose.orientation.w)
                    .c_str());
}

bool move_to_target(auto &move_group_interface, auto &logger) {
    auto joint_values = move_group_interface.getCurrentJointValues();
    for (size_t i = 0; i < joint_values.size(); ++i) {
        RCLCPP_INFO(logger, "Joint[%zu]: %f", i, joint_values[i]);
    }
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan plan_feedback;
        auto const ok =
            static_cast<bool>(move_group_interface.plan(plan_feedback));
        return std::make_pair(ok, plan_feedback);
    }();
    if (success) {
        move_group_interface.execute(plan);
    }
    return success;
}

/// reconnect

// class SetModeActionClient : public rclcpp::Node {
//   public:
//     SetModeActionClient() : Node("set_mode_action_client") {
//         // Adjust the action name if your driver uses a different topic
//         client_ = rclcpp_action::create_client<SetMode>(
//             this, "/dashboard_client/set_mode");
//     }

//     void send_goal(int8_t target_mode, bool stop_program = false,
//                    bool play_program = false) {
//         // Wait until the action server is available
//         if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
//       RCLCPP_ERROR(this->get_logger(), "Action server not available after
//       waiting"); return;
//         }

//         // Create a goal
//         auto goal_msg = SetMode::Goal();
//         goal_msg.target_robot_mode = target_mode;
//         goal_msg.stop_program = stop_program;
//         goal_msg.play_program = play_program;

//         RCLCPP_INFO(this->get_logger(),
//                     "Sending goal: target_mode=%d
//                     stop_program = % s play_program = % s ",
//                                                       target_mode,
//                     stop_program ? "true" : "false",
//                     play_program ? "true" : "false");

//         // Send goal
//         auto send_goal_options =
//             rclcpp_action::Client<SetMode>::SendGoalOptions();
//         send_goal_options.feedback_callback =
//             std::bind(&SetModeActionClient::feedback_callback, this,
//                       std::placeholders::_1, std::placeholders::_2);
//         send_goal_options.result_callback = std::bind(
//             &SetModeActionClient::result_callback, this,
//             std::placeholders::_1);

//         client_->async_send_goal(goal_msg, send_goal_options);
//     }

//   private:
//     void
//     feedback_callback(GoalHandleSetMode::SharedPtr,
//                       const std::shared_ptr<const SetMode::Feedback>
//                       feedback) {
//         RCLCPP_INFO(
//             this->get_logger(),
//             "Feedback - current_robot_mode: %d, current_safety_mode: %d",
//             feedback->current_robot_mode, feedback->current_safety_mode);
//     }

//     void result_callback(const GoalHandleSetMode::WrappedResult &result) {
//         switch (result.code) {
//         case rclcpp_action::ResultCode::SUCCEEDED:
//             RCLCPP_INFO(this->get_logger(),
//                         "Result received: success=%s,
//                         message = % s ",
//                                           result.result->success
//                                       ? "true"
//                                       : "false",
//                         result.result->message.c_str());
//             break;
//         case rclcpp_action::ResultCode::ABORTED:
//             RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//             break;
//         case rclcpp_action::ResultCode::CANCELED:
//             RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//             break;
//         default:
//             RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//             break;
//         }
//     }

//     rclcpp_action::Client<SetMode>::SharedPtr client_;
// };

// #include "rclcpp/rclcpp.hpp"
// #include "ur_dashboard_msgs/srv/get_robot_mode.hpp"

// static const int8_t POWER_OFF = 3;
// static const int8_t POWER_ON = 4;
// static const int8_t RUNNING = 7;

// using SetMode = ur_dashboard_msgs::action::SetMode;
// using GoalHandleSetMode = rclcpp_action::ClientGoalHandle<SetMode>;

// class GetRobotModeClient : public rclcpp::Node {
//   public:
//     GetRobotModeClient() : Node("get_robot_mode_client") {
//         client_ = this->create_client<ur_dashboard_msgs::srv::GetRobotMode>(
//             "/dashboard_client/get_robot_mode");
//     }

//     void callService() {
//         if (!client_->wait_for_service(5s)) {
//       RCLCPP_ERROR(this->get_logger(), "Service
//       /dashboard_client/get_robot_mode not available"); return;
//         }
//         auto request =
//             std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>();
//         auto future_result = client_->async_send_request(request);
//         if
//         (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
//                                                future_result, 5s) ==
//             rclcpp::FutureReturnCode::SUCCESS) {
//             auto response = future_result.get();
//             if (response->success) {
//                 RCLCPP_INFO(this->get_logger(), "Robot mode: %d, answer: %s",
//                             response->mode, response->answer.c_str());
//             } else {
//                 RCLCPP_WARN(this->get_logger(),
//                             "Service call succeeded but reported
//                                 failure : %
//                                 s ",
//                                 response->answer.c_str());
//             }
//         } else {
//       RCLCPP_ERROR(this->get_logger(), "Failed to call get_robot_mode
//       service");
//         }
//     }

//   private:
//     rclcpp::Client<ur_dashboard_msgs::srv::GetRobotMode>::SharedPtr client_;
// };

/// reconnect

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    using moveit::planning_interface::MoveGroupInterface;

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // 3D Parameters
    const int interval = 4;
    const bool single_interval = false;

    // Publisher Parameters
    std::string msg;
    double angle = 0.0;
    int circle_state = 1;
    bool apply_config = true;
    bool end_state = false;
    bool scan_3d = false;

    // Internal Parameters
    bool planning = false;
    double angle_increment;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    Eigen::Matrix3d rotmat_eigen;
    cv::Mat img;
    std::vector<cv::Mat> img_array;
    std::vector<Eigen::Vector3d> pc_lines;
    tf2::Quaternion q;
    tf2::Quaternion target_q;
    geometry_msgs::msg::Pose target_pose;

    // Subscriber Parameters
    double robot_vel;
    double robot_acc;
    double radius;
    double angle_limit;
    double dz;
    double z_tolerance;
    double angle_tolerance;
    double z_height;
    int num_pt;
    bool fast_axis = true;
    bool success = false;
    bool next = false;
    bool previous = false;
    bool home = false;
    bool z_focused = false;
    bool angle_focused = false;

    auto const move_group_node =
        std::make_shared<rclcpp::Node>("node_moveit", node_options);
    auto move_group_interface =
        MoveGroupInterface(move_group_node, "ur_manipulator");
    auto subscriber_node = std::make_shared<dds_subscriber>();
    auto urscript_node = std::make_shared<urscript_publisher>();
    auto publisher_node = std::make_shared<dds_publisher>(
        msg, angle, circle_state, fast_axis, apply_config, end_state, scan_3d);
    auto img_subscriber_node = std::make_shared<img_subscriber>();

    auto const logger = rclcpp::get_logger("logger_planning");

    int attempt = 0;
    int max_attempt = 5;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);
    executor.add_node(urscript_node);
    executor.add_node(img_subscriber_node);
    std::thread spinner([&executor]() { executor.spin(); });

    add_collision_obj(move_group_interface);

    move_group_interface.setPlanningTime(10.0);
    move_group_interface.setNumPlanningAttempts(30);
    // move_group_interface.setPlanningPipelineId("stomp");
    move_group_interface.setPlanningPipelineId("ompl");

    // auto robot_mode_node = std::make_shared<GetRobotModeClient>();
    // auto robot_set_node = std::make_shared<SetModeActionClient>();
    // RCLCPP_INFO(node->get_logger(), "Requesting POWER_ON...");
    // node->send_goal(POWER_ON);

    while (rclcpp::ok() && running) {

        end_state = false;
        apply_config = false;
        fast_axis = true;
        publisher_node->set_fast_axis(fast_axis);
        publisher_node->set_apply_config(apply_config);
        publisher_node->set_end_state(end_state);
        next = subscriber_node->next();
        previous = subscriber_node->previous();
        home = subscriber_node->home();

        z_tolerance = subscriber_node->z_tolerance();
        angle_tolerance = subscriber_node->angle_tolerance();
        radius = subscriber_node->radius();
        angle_limit = subscriber_node->angle_limit();
        num_pt = subscriber_node->num_pt();
        robot_vel = subscriber_node->robot_vel();
        robot_acc = subscriber_node->robot_acc();
        z_height = subscriber_node->z_height();
        robot_vel = 0.1;
        robot_acc = 0.1;

        // if (!robot_mode_node->callService()) {
        //     RCLCPP_INFO(node->get_logger(), "Requesting RUNNING...");
        //     node->send_goal(RUNNING, /*stop_program=*/true,
        //                     /*play_program=*/false);
        //     trigger_client = this->create_client<std_srvs::srv::Trigger>(
        //         "/io_and_status_controller/resend_robot_program");
        //     auto request =
        //     std::make_shared<std_srvs::srv::Trigger::Request>(); auto future
        //     = trigger_client->async_send_request(request);
        // };

        if (subscriber_node->freedrive()) {
            circle_state = 1;
            angle = 0.0;
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);
            urscript_node->activate_freedrive();
            while (subscriber_node->freedrive()) {
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
            urscript_node->deactivate_freedrive();
        }

        move_group_interface.setMaxVelocityScalingFactor(robot_vel);
        move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
        move_group_interface.setStartStateToCurrentState();

        if (subscriber_node->reset()) {
            angle = 0.0;
            circle_state = 1;
            msg = "Reset to default position";
            RCLCPP_INFO(logger, msg.c_str());
            publisher_node->set_msg(msg);
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);
            move_group_interface.setJointValueTarget("shoulder_pan_joint",
                                                     to_radian(0.0));
            move_group_interface.setJointValueTarget("shoulder_lift_joint",
                                                     -to_radian(60.0));
            move_group_interface.setJointValueTarget("elbow_joint",
                                                     to_radian(90.0));
            move_group_interface.setJointValueTarget("wrist_1_joint",
                                                     to_radian(-120.0));
            move_group_interface.setJointValueTarget("wrist_2_joint",
                                                     to_radian(-90.0));
            move_group_interface.setJointValueTarget("wrist_3_joint",
                                                     to_radian(-135.0));

            success = move_to_target(move_group_interface, logger);
            // if (!success) {
            //     msg = std::format("Reset Planning Failed!");
            //     publisher_node->set_msg(msg);
            //     RCLCPP_ERROR(logger, msg.c_str());
            // }
        }

        if (subscriber_node->autofocus()) {
            apply_config = true;
            publisher_node->set_apply_config(apply_config);
            if (!scan_3d) {
                scan_3d = true;
                apply_config = true;
                msg = "Starting 3D Scan";
                RCLCPP_INFO(logger, msg.c_str());
                publisher_node->set_msg(msg);
                publisher_node->set_scan_3d(scan_3d);
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                // publisher_node->set_apply_config(apply_config);
                // rclcpp::sleep_for(std::chrono::milliseconds(100));
                // publisher_node->set_apply_config(apply_config);
                // rclcpp::sleep_for(std::chrono::milliseconds(100));
                // publisher_node->set_apply_config(apply_config);
                // rclcpp::sleep_for(std::chrono::milliseconds(400));
            }
            img_array.clear();
            for (int i = 0; i < interval; i++) {
                while (true) {
                    img = img_subscriber_node->get_img();
                    if (!img.empty()) {
                        break;
                    }
                    if (!subscriber_node->autofocus()) {
                        break;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(10000));
                }
                img_array.push_back(img);
                RCLCPP_INFO(logger, "Collected image %d", i + 1);
            }
            msg = "Calculating Rotations";
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());

            // scan_3d = false;
            // apply_config = true;
            // publisher_node->set_scan_3d(scan_3d);
            // publisher_node->set_apply_config(apply_config);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));

            pc_lines = lines_3d(img_array, interval, single_interval);
            open3d::geometry::PointCloud pcd_lines;
            for (const auto &point : pc_lines) {
                pcd_lines.points_.emplace_back(point);
            }
            auto boundbox = pcd_lines.GetMinimalOrientedBoundingBox(false);
            Eigen::Vector3d center = boundbox.GetCenter();
            msg += std::format("\n[Center] x:{} y:{} z:{}", center[0],
                               center[1], center[2]);
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());
            rotmat_eigen = align_to_direction(boundbox.R_);

            tf2::Matrix3x3 rotmat_tf(
                rotmat_eigen(0, 0), rotmat_eigen(0, 1), rotmat_eigen(0, 2),
                rotmat_eigen(1, 0), rotmat_eigen(1, 1), rotmat_eigen(1, 2),
                rotmat_eigen(2, 0), rotmat_eigen(2, 1), rotmat_eigen(2, 2));
            RCLCPP_INFO_STREAM(logger, "\nAligned Rotation Matrix:\n"
                                           << rotmat_eigen);

            /// temporary fix for swapped axis ///
            double tmp_roll;
            double tmp_pitch;
            double tmp_yaw;
            rotmat_tf.getRPY(tmp_roll, tmp_pitch, tmp_yaw);
            roll = tmp_yaw;
            pitch = tmp_roll;
            yaw = tmp_pitch;
            rotmat_tf.setRPY(roll, pitch, yaw);
            ///////////////////////

            msg +=
                std::format("\nCalculated R:{:.2f}, P:{:.2f}, Y:{:.2f}",
                            to_degree(roll), to_degree(pitch), to_degree(yaw));
            publisher_node->set_msg(msg);
            RCLCPP_INFO(logger, msg.c_str());

            if (tol_measure(roll, pitch, angle_tolerance)) {
                angle_focused = true;
                msg += "\nAngle focused";
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, msg.c_str());
                scan_3d = false;
                apply_config = true;
                publisher_node->set_scan_3d(scan_3d);
                publisher_node->set_apply_config(apply_config);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                // publisher_node->set_apply_config(apply_config);
                // rclcpp::sleep_for(std::chrono::milliseconds(100));
                // publisher_node->set_apply_config(apply_config);
                // rclcpp::sleep_for(std::chrono::milliseconds(100));
                // publisher_node->set_apply_config(apply_config);
                // rclcpp::sleep_for(std::chrono::milliseconds(100));
            }

            if (angle_focused && !z_focused) {
                // dz = 0;
                // dz = (z_height - center[1]) / (100 * 1000.0);
                dz = (z_height - center[1]) / (50 * 1000.0);
                msg += std::format("\ndz = {}", dz);
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, "dz: %f", dz);
                if (std::abs(dz) < (z_tolerance / 1000.0)) {
                    z_focused = true;
                    msg += "\nHeight focused";
                    publisher_node->set_msg(msg);
                    RCLCPP_INFO(logger, msg.c_str());
                } else {
                    planning = true;
                    target_pose = move_group_interface.getCurrentPose().pose;
                    target_pose.position.z += dz;
                    print_target(logger, target_pose);
                    move_group_interface.setPoseTarget(target_pose);
                    success = move_to_target(move_group_interface, logger);
                    // if (!success) {
                    //     msg = std::format("Z-height Planning Failed!");
                    //     RCLCPP_ERROR(logger, msg.c_str());
                    //     publisher_node->set_msg(msg);
                    // }
                }
            }

            if (!angle_focused) {
                planning = true;
                rotmat_tf.getRotation(q);
                target_pose = move_group_interface.getCurrentPose().pose;
                tf2::fromMsg(target_pose.orientation, target_q);
                q.normalize();
                target_q = target_q * q;
                target_pose.orientation = tf2::toMsg(target_q);
                target_pose.position.x += radius * std::cos(to_radian(angle));
                target_pose.position.y += radius * std::sin(to_radian(angle));
                target_pose.position.z += dz;
                print_target(logger, target_pose);
                move_group_interface.setPoseTarget(target_pose);
                success = move_to_target(move_group_interface, logger);
                if (!success) {
                    // msg = std::format("Angle Focus Planning Failed!");
                    msg = "Angle Focus Planning Failed!";
                    publisher_node->set_msg(msg);
                    RCLCPP_ERROR(logger, msg.c_str());
                }
            }

            if (angle_focused && z_focused) {
                angle_focused = false;
                z_focused = false;
                planning = false;
                end_state = true;
                msg += "\nWithin tolerance";
                publisher_node->set_msg(msg);
                publisher_node->set_end_state(end_state);
                move_group_interface.setStartStateToCurrentState();
                target_pose = move_group_interface.getCurrentPose().pose;
                while (subscriber_node->autofocus()) {
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }
            }
        } else {
            angle_increment = angle_limit / num_pt;
            roll = 0.0, pitch = 0.0, yaw = 0.0;

            if (next) {
                planning = true;
                yaw += to_radian(angle_increment);
            }
            if (previous) {
                planning = true;
                yaw += to_radian(-angle_increment);
            }
            if (home) {
                planning = true;
                yaw += to_radian(-angle);
            }
            publisher_node->set_angle(angle);
            publisher_node->set_circle_state(circle_state);

            if (planning) {
                target_pose = move_group_interface.getCurrentPose().pose;
                tf2::fromMsg(target_pose.orientation, target_q);
                q.setRPY(roll, pitch, yaw);
                q.normalize();
                target_q = target_q * q;
                target_pose.orientation = tf2::toMsg(target_q);
                print_target(logger, target_pose);
                move_group_interface.setPoseTarget(target_pose);
                success = move_to_target(move_group_interface, logger);
                if (success) {
                    msg = "Planning Success!";
                    RCLCPP_INFO(logger, msg.c_str());
                    if (next) {
                        angle += angle_increment;
                        circle_state++;
                    }
                    if (previous) {
                        angle -= angle_increment;
                        circle_state--;
                    }
                    if (home) {
                        circle_state = 1;
                        angle = 0.0;
                    }
                } else {
                    // msg = std::format("Z-axis Rotation Planning Failed! "
                    //                   "\nAttempt[{}] Retrying....");
                    msg = "Z-axis Rotation Planning Failed!";
                    RCLCPP_ERROR(logger, msg.c_str());
                    publisher_node->set_msg(msg);
                }
            }
        }

        if (planning) {
            planning = false;
            while (!subscriber_node->changed()) {
                if (!subscriber_node->autofocus()) {
                    rclcpp::sleep_for(std::chrono::milliseconds(10));
                    break;
                }
            }
            // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }

        if (scan_3d && !subscriber_node->autofocus()) {
            scan_3d = false;
            apply_config = true;
            publisher_node->set_scan_3d(scan_3d);
            publisher_node->set_apply_config(apply_config);
        }
    }

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
