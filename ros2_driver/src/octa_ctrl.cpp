#include "octa_ros/msg/img.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <csignal>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <octa_ros/msg/labviewdata.hpp>
#include <octa_ros/msg/robotdata.hpp>
#include <open3d/Open3D.h>
#include <opencv2/img_hash.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

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

double to_radian(const double degree) {
    return (std::numbers::pi / 180 * degree);
}

double to_degree(const double radian) {
    return (180 / std::numbers::pi * radian);
}

std::vector<cv::Point> get_max_coor(const cv::Mat &img) {
    std::vector<cv::Point> ret_coords;
    // int height = img.rows;
    int width = img.cols;

    for (int x = 0; x < width; ++x) {
        cv::Mat intensity = img.col(x);
        double minVal, maxVal;
        int minIndex, maxIndex; // use int, not cv::Point
        cv::minMaxIdx(intensity, &minVal, &maxVal, &minIndex, &maxIndex);

        int detected_y = maxIndex;
        ret_coords.emplace_back(cv::Point(x, detected_y));
    }
    return ret_coords;
}

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

void add_collision_obj(auto &move_group_interface) {

    auto const collision_floor = [frame_id =
                                      move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_floor;
        collision_floor.header.frame_id = frame_id;
        collision_floor.id = "floor";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 10.0;
        primitive.dimensions[primitive.BOX_Y] = 10.0;
        primitive.dimensions[primitive.BOX_Z] = 0.01;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.0855;

        collision_floor.primitives.push_back(primitive);
        collision_floor.primitive_poses.push_back(box_pose);
        collision_floor.operation = collision_floor.ADD;

        return collision_floor;
    }();

    auto const collision_base = [frame_id =
                                     move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_base;
        collision_base.header.frame_id = frame_id;
        collision_base.id = "robot_base";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.27;
        primitive.dimensions[primitive.BOX_Y] = 0.27;
        primitive.dimensions[primitive.BOX_Z] = 0.085;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.043;

        collision_base.primitives.push_back(primitive);
        collision_base.primitive_poses.push_back(box_pose);
        collision_base.operation = collision_base.ADD;

        return collision_base;
    }();

    auto const collision_monitor =
        [frame_id = move_group_interface.getPlanningFrame()] {
            moveit_msgs::msg::CollisionObject collision_monitor;
            collision_monitor.header.frame_id = frame_id;
            collision_monitor.id = "monitor";
            shape_msgs::msg::SolidPrimitive primitive;

            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = 0.25;
            primitive.dimensions[primitive.BOX_Y] = 0.6;
            primitive.dimensions[primitive.BOX_Z] = 0.6;

            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = -0.2;
            box_pose.position.y = 0.435;
            box_pose.position.z = 0.215;

            collision_monitor.primitives.push_back(primitive);
            collision_monitor.primitive_poses.push_back(box_pose);
            collision_monitor.operation = collision_monitor.ADD;

            return collision_monitor;
        }();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_floor);
    planning_scene_interface.applyCollisionObject(collision_base);
    planning_scene_interface.applyCollisionObject(collision_monitor);
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

// Classes

class img_subscriber : public rclcpp::Node {

  public:
    img_subscriber()
        : Node("img_subscriber"),
          best_effort(rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()) {
        subscription_ = this->create_subscription<octa_ros::msg::Img>(
            "oct_image", best_effort,
            std::bind(&img_subscriber::imageCallback, this,
                      std::placeholders::_1));
    }

    cv::Mat get_img() {
        std::unique_lock<std::mutex> lock(img_mutex_);
        img_status_.wait(lock, [this] { return new_img_; });
        cv::Mat img_copy = img_.clone();
        new_img_ = false;
        return img_copy;
    }

  private:
    void imageCallback(const octa_ros::msg::Img::SharedPtr msg) {
        RCLCPP_INFO(
            this->get_logger(),
            std::format("Subscribing to image, length: {}", msg->img.size())
                .c_str());
        cv::Mat img(height_, width_, CV_8UC1);
        std::copy(msg->img.begin(), msg->img.end(), img.data);

        cv::Mat current_hash;
        cv::img_hash::AverageHash::create()->compute(img, current_hash);
        {
            std::lock_guard<std::mutex> lock(img_mutex_);
            if (img_.empty() || cv::norm(img_hash_, current_hash) > 0) {
                img_ = img.clone();
                img_hash_ = current_hash;
                new_img_ = true;
                img_status_.notify_one();
            }
        }
    }

    const int width_ = 500;
    const int height_ = 512;

    cv::Mat img_;
    cv::Mat img_hash_;
    std::mutex img_mutex_;
    std::condition_variable img_status_;
    bool new_img_ = false;

    rclcpp::Subscription<octa_ros::msg::Img>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
};

class dds_publisher : public rclcpp::Node {
  public:
    dds_publisher() : dds_publisher("", 0.0, 1, true, false, false, false) {}

    dds_publisher(std::string msg = "", double angle = 0.0,
                  int circle_state = 1, bool fast_axis = true,
                  bool apply_config = false, bool end_state = false,
                  bool scan_3d = false)
        : Node("pub_robot_data"), msg(msg), angle(angle),
          circle_state(circle_state), fast_axis(fast_axis),
          apply_config(apply_config), end_state(end_state), scan_3d(scan_3d) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        publisher_ =
            this->create_publisher<octa_ros::msg::Robotdata>("robot_data", qos);

        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
                auto message = octa_ros::msg::Robotdata();
                message.msg = this->msg;
                message.angle = this->angle;
                message.circle_state = this->circle_state;
                message.fast_axis = this->fast_axis;
                message.apply_config = this->apply_config;
                message.end_state = this->end_state;
                message.scan_3d = this->scan_3d;

                if (old_message != message) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[PUBLISHING] msg: %s, angle: %f, circle_state: %d, "
                        "fast_axis: %s, apply_config: %s, end_state: %s, "
                        "scan_3d: %s",
                        this->msg.c_str(), this->angle, this->circle_state,
                        this->fast_axis ? "true" : "false",
                        this->apply_config ? "true" : "false",
                        this->end_state ? "true" : "false",
                        this->scan_3d ? "true" : "false");
                }

                publisher_->publish(message);

                if (this->apply_config) {
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed =
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - apply_config_time_)
                            .count();
                    if (elapsed >= 100) {
                        this->apply_config = false;
                    }
                }

                old_message = message;
            });
    }

    void set_msg(const std::string &new_msg) { msg = new_msg; }
    void set_angle(double new_angle) { angle = new_angle; }
    void set_circle_state(int new_circle_state) {
        circle_state = new_circle_state;
    }
    void set_fast_axis(bool new_fast_axis) { fast_axis = new_fast_axis; }
    void set_apply_config(bool new_apply_config) {
        if (new_apply_config) {
            apply_config_time_ = std::chrono::steady_clock::now();
        }
        apply_config = new_apply_config;
    }
    void set_end_state(bool new_end_state) { end_state = new_end_state; }
    void set_scan_3d(bool new_scan_3d) { scan_3d = new_scan_3d; }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr publisher_;
    std::string msg;
    double angle;
    int circle_state;
    bool fast_axis, apply_config, end_state, scan_3d;
    octa_ros::msg::Robotdata old_message = octa_ros::msg::Robotdata();
    std::chrono::steady_clock::time_point apply_config_time_;
};

class dds_subscriber : public rclcpp::Node {
  public:
    dds_subscriber()
        : Node("sub_labview")

    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        subscription_ = this->create_subscription<octa_ros::msg::Labviewdata>(
            "labview_data", qos,
            [this](const octa_ros::msg::Labviewdata::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                robot_vel_ = msg->robot_vel;
                robot_acc_ = msg->robot_acc;
                z_tolerance_ = msg->z_tolerance;
                angle_tolerance_ = msg->angle_tolerance;
                radius_ = msg->radius;
                angle_limit_ = msg->angle_limit;
                num_pt_ = msg->num_pt;
                dz_ = msg->dz;
                drot_ = msg->drot;
                autofocus_ = msg->autofocus;
                freedrive_ = msg->freedrive;
                previous_ = msg->previous;
                next_ = msg->next;
                home_ = msg->home;
                reset_ = msg->reset;
                fast_axis_ = msg->fast_axis;
                scan_3d_ = msg->scan_3d;
                z_height_ = msg->z_height;
                if (old_msg != *msg) {
                    RCLCPP_INFO(this->get_logger(),
                                std::format("[SUBSCRIBING] "
                                            " robot_vel: {},"
                                            " robot_acc: {},"
                                            " z_tolerance: {},"
                                            " angle_tolerance: {},"
                                            " radius: {},"
                                            " angle_limit: {},"
                                            " num_pt: {},"
                                            " dz: {},"
                                            " drot: {},"
                                            " autofocus: {},"
                                            " freedrive: {},"
                                            " previous: {},"
                                            " next: {},"
                                            " home: {},"
                                            " reset: {},"
                                            " fast_axis: {},"
                                            " scan_3d: {}"
                                            " z_height: {}",
                                            robot_vel_, robot_acc_,
                                            z_tolerance_, angle_tolerance_,
                                            radius_, angle_limit_, num_pt_, dz_,
                                            drot_, autofocus_, freedrive_,
                                            previous_, next_, home_, reset_,
                                            fast_axis_, scan_3d_, z_height_)
                                    .c_str());
                    changed_ = true;
                } else {
                    changed_ = false;
                }
                old_msg = *msg;
            });
    };
    double robot_vel() { return robot_vel_; };
    double robot_acc() { return robot_acc_; };
    double z_tolerance() { return z_tolerance_; };
    double angle_tolerance() { return angle_tolerance_; };
    double radius() { return radius_; };
    double angle_limit() { return angle_limit_; };
    int num_pt() { return num_pt_; };
    double dz() { return dz_; };
    double drot() { return drot_; };
    double z_height() { return z_height_; };
    bool autofocus() { return autofocus_; };
    bool freedrive() { return freedrive_; };
    bool previous() { return previous_; };
    bool next() { return next_; };
    bool home() { return home_; };
    bool reset() { return reset_; };
    bool fast_axis() { return fast_axis_; };
    bool changed() { return changed_; };
    bool scan_3d() { return scan_3d_; };

  private:
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr subscription_;
    double robot_vel_ = 0, robot_acc_ = 0, z_tolerance_ = 0,
           angle_tolerance_ = 0, radius_ = 0, angle_limit_ = 0, dz_ = 0,
           drot_ = 0, z_height_;
    int num_pt_ = 0;
    bool autofocus_ = false, freedrive_ = false, previous_ = false,
         next_ = false, home_ = false, reset_ = false, fast_axis_ = false,
         changed_ = false, scan_3d_ = false;
    octa_ros::msg::Labviewdata old_msg = octa_ros::msg::Labviewdata();
    std::mutex data_mutex_;
};

class urscript_publisher : public rclcpp::Node {
  public:
    urscript_publisher()
        : Node("urscript_publisher"), freedrive(false), executed(true) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        trigger_client = this->create_client<std_srvs::srv::Trigger>(
            "/io_and_status_controller/resend_robot_program");
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/urscript_interface/script_command", qos);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&urscript_publisher::publish_to_robot, this));
    }
    void activate_freedrive() {
        freedrive = true;
        executed = false;
    }
    void deactivate_freedrive() {
        freedrive = false;
        executed = false;
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client;
    bool freedrive, executed;
    void publish_to_robot() {
        if (!executed) {
            executed = true;
            auto message = std_msgs::msg::String();
            if (freedrive) {
                message.data = R"(
def program():
 global check = "Made it"
 while(True):
  freedrive_mode()
 end
end
            )";
            } else {
                message.data = R"(
def program():
 end_freedrive_mode()
end
            )";
            }
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "URscript message published: '%s'",
                        message.data.c_str());
            if (!freedrive) {
                // ros2 service call
                // /io_and_status_controller/resend_robot_program
                // std_srvs/srv/Trigger;
                while (!trigger_client->wait_for_service(
                    std::chrono::seconds(1))) {
                    RCLCPP_INFO(this->get_logger(), "Waiting for service...");
                }
                auto request =
                    std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = trigger_client->async_send_request(request);
            }
        }
    }
};

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
    double robot_vel, robot_acc, radius, angle_limit, dz;
    double z_tolerance, angle_tolerance, z_height;
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

    // tf2::Quaternion q_down;
    // q_down.setRPY(0, M_PI, 0);
    // q_down.normalize();
    moveit_msgs::msg::Constraints path_constr;
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "tcp";
    ocm.header.frame_id = "base_link";
    // ocm.orientation = tf2::toMsg(q_down);
    double x_tol = 0.5;
    double y_tol = 0.5;
    double z_tol = 0.5;
    double path_enforce = 1.0;
    int attempt = 0;
    int max_attempt = 5;
    ocm.absolute_x_axis_tolerance = x_tol;
    ocm.absolute_y_axis_tolerance = y_tol;
    ocm.absolute_z_axis_tolerance = z_tol;
    ocm.weight = path_enforce;
    path_constr.orientation_constraints.push_back(ocm);
    move_group_interface.setPathConstraints(path_constr);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);
    executor.add_node(urscript_node);
    executor.add_node(img_subscriber_node);
    std::thread spinner([&executor]() { executor.spin(); });

    add_collision_obj(move_group_interface);

    auto client =
        publisher_node->create_client<moveit_msgs::srv::GetStateValidity>(
            "/check_state_validity");
    auto request =
        std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    request->group_name = "ur_manipulator";

    auto current_state = move_group_interface.getCurrentState(10);
    moveit_msgs::msg::RobotState current_state_msg;
    moveit::core::robotStateToRobotStateMsg(*current_state, current_state_msg);
    request->robot_state = current_state_msg;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(publisher_node, future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->valid) {
            msg = "No errors with setup";
            publisher_node->set_msg(msg);
            RCLCPP_WARN(logger, msg.c_str());
        } else {
            msg = "Robot Start State is in collision with setup!";
            publisher_node->set_msg(msg);
            RCLCPP_WARN(logger, msg.c_str());
        }
    }

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

        current_state = move_group_interface.getCurrentState(10.0);
        bool is_valid = current_state->satisfiesBounds();
        msg = std::format("Current Robot state is {}",
                          (is_valid ? "VALID" : "INVALID"));
        RCLCPP_INFO(logger, msg.c_str());
        publisher_node->set_msg(msg);

        if (x_tol > 2.0 || y_tol > 2.0 || z_tol > 2.0 || path_enforce <= 0.0) {
            end_state = true;
            msg = "Unrecoverable Error. Please restart";
            publisher_node->set_msg(msg);
            publisher_node->set_end_state(end_state);
        }

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
            attempt = 0;
            success = false;
            while (!success && (attempt < max_attempt)) {
                success = move_to_target(move_group_interface, logger);
                if (!success) {
                    msg = std::format(
                        "Reset Planning Failed! \nAttempt[{}] Retrying....",
                        attempt);
                    publisher_node->set_msg(msg);
                    x_tol += 0.1;
                    y_tol += 0.1;
                    z_tol += 0.1;
                    path_enforce -= 0.1;
                    ocm.absolute_x_axis_tolerance = x_tol;
                    ocm.absolute_y_axis_tolerance = y_tol;
                    ocm.absolute_z_axis_tolerance = z_tol;
                    ocm.weight = path_enforce;
                    path_constr.orientation_constraints.clear();
                    path_constr.orientation_constraints.push_back(ocm);
                    move_group_interface.setPathConstraints(path_constr);
                    attempt++;
                }
            }
            if (!success) {
                msg = std::format(
                    "Reset Planning Failed! Maximum Attempts Reached", attempt);
                publisher_node->set_msg(msg);
                RCLCPP_ERROR(logger, msg.c_str());
            }
        }

        if (subscriber_node->autofocus()) {
            apply_config = true;
            publisher_node->set_apply_config(apply_config);
            scan_3d = true;
            apply_config = true;
            // msg = "Starting 3D Scan";
            publisher_node->set_msg(msg);
            publisher_node->set_scan_3d(scan_3d);
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
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
            scan_3d = false;
            apply_config = true;
            msg = "Calculating Rotations";
            publisher_node->set_msg(msg);
            publisher_node->set_scan_3d(scan_3d);
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publisher_node->set_apply_config(apply_config);
            rclcpp::sleep_for(std::chrono::milliseconds(100));

            pc_lines = lines_3d(img_array, interval, single_interval);
            open3d::geometry::PointCloud pcd_lines;
            for (const auto &point : pc_lines) {
                pcd_lines.points_.emplace_back(point);
            }
            auto boundbox = pcd_lines.GetMinimalOrientedBoundingBox(false);
            Eigen::Vector3d center = boundbox.GetCenter();
            z_height = subscriber_node->z_height();
            RCLCPP_INFO(logger, "center: %f", center[1]);
            rotmat_eigen = align_to_direction(boundbox.R_);
            tf2::Matrix3x3 rotmat_tf(
                rotmat_eigen(0, 0), rotmat_eigen(0, 1), rotmat_eigen(0, 2),
                rotmat_eigen(1, 0), rotmat_eigen(1, 1), rotmat_eigen(1, 2),
                rotmat_eigen(2, 0), rotmat_eigen(2, 1), rotmat_eigen(2, 2));
            RCLCPP_INFO_STREAM(logger, "\nAligned Rotation Matrix:\n"
                                           << rotmat_eigen);
            double tmp_roll;
            double tmp_pitch;
            double tmp_yaw;
            rotmat_tf.getRPY(tmp_roll, tmp_pitch, tmp_yaw);
            roll = tmp_yaw;
            pitch = tmp_roll;
            yaw = tmp_pitch;
            msg =
                std::format("Calculated R:{:.2f}, P:{:.2f}, Y:{:.2f}",
                            to_degree(roll), to_degree(pitch), to_degree(yaw));
            RCLCPP_INFO(logger, msg.c_str());
            publisher_node->set_msg(msg);
            rotmat_tf.setRPY(roll, pitch, yaw);

            if (tol_measure(roll, pitch, angle_tolerance)) {
                angle_focused = true;
                msg += "\nAngle focused";
                publisher_node->set_msg(msg);
            }

            if (angle_focused && !z_focused) {
                // dz = 0;
                dz = -(center[1] - z_height) / 150 * 1.2 / 1000.0;
                // dz = (z_height - center[1]) / (256.0 * 1000.0);
                msg += std::format("\ndz = {}", dz).c_str();
                publisher_node->set_msg(msg);
                RCLCPP_INFO(logger, "dz: %f", dz);
                if (std::abs(dz) < (z_tolerance / 1000.0)) {
                    z_focused = true;
                    msg += "\nHeight focused";
                    publisher_node->set_msg(msg);
                } else {
                    planning = true;
                    target_pose = move_group_interface.getCurrentPose().pose;
                    target_pose.position.z += dz;
                    print_target(logger, target_pose);
                    move_group_interface.setPoseTarget(target_pose);
                    attempt = 0;
                    success = false;
                    while (!success && (attempt < max_attempt)) {
                        success = move_to_target(move_group_interface, logger);
                        if (success) {
                            RCLCPP_INFO(logger, msg.c_str());
                        } else {
                            msg = std::format("Z-height Planning Failed! "
                                              "\nAttempt[{}] Retrying....",
                                              attempt);
                            RCLCPP_ERROR(logger, msg.c_str());
                            publisher_node->set_msg(msg);
                            x_tol += 0.1;
                            y_tol += 0.1;
                            z_tol += 0.1;
                            path_enforce -= 0.1;
                            ocm.absolute_x_axis_tolerance = x_tol;
                            ocm.absolute_y_axis_tolerance = y_tol;
                            ocm.absolute_z_axis_tolerance = z_tol;
                            ocm.weight = path_enforce;
                            path_constr.orientation_constraints.clear();
                            path_constr.orientation_constraints.push_back(ocm);
                            move_group_interface.setPathConstraints(
                                path_constr);
                            attempt++;
                        }
                    }
                    if (!success) {
                        msg = std::format("Z-height Planning Failed! Maximum "
                                          "Attempts Reached",
                                          attempt);
                        publisher_node->set_msg(msg);
                        RCLCPP_ERROR(logger, msg.c_str());
                    }
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

                attempt = 0;
                success = false;
                while (!success && (attempt < max_attempt)) {
                    success = move_to_target(move_group_interface, logger);
                    if (success) {
                        RCLCPP_INFO(logger, msg.c_str());
                    } else {
                        msg = std::format("Angle Focus Planning Failed! "
                                          "\nAttempt[{}] Retrying....",
                                          attempt);
                        RCLCPP_ERROR(logger, msg.c_str());
                        publisher_node->set_msg(msg);
                    }
                    attempt++;
                }
                if (!success) {
                    msg = std::format(
                        "Angle Focus Planning Failed! Maximum Attempts Reached",
                        attempt);
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

                attempt = 0;
                success = false;
                while (!success && (attempt < max_attempt)) {
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
                        msg = std::format("Z-axis Rotation Planning Failed! "
                                          "\nAttempt[{}] Retrying....",
                                          attempt);
                        RCLCPP_ERROR(logger, msg.c_str());
                        publisher_node->set_msg(msg);
                        x_tol += 0.1;
                        y_tol += 0.1;
                        z_tol += 0.1;
                        path_enforce -= 0.1;
                        ocm.absolute_x_axis_tolerance = x_tol;
                        ocm.absolute_y_axis_tolerance = y_tol;
                        ocm.absolute_z_axis_tolerance = z_tol;
                        ocm.weight = path_enforce;
                        path_constr.orientation_constraints.push_back(ocm);
                        move_group_interface.setPathConstraints(path_constr);
                    }
                    attempt++;
                }
                if (!success) {
                    msg = std::format("Z-axis Rotation Planning Failed! "
                                      "Maximum Attempts Reached",
                                      attempt);
                    publisher_node->set_msg(msg);
                    RCLCPP_ERROR(logger, msg.c_str());
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

        if (subscriber_node->scan_3d() && !subscriber_node->autofocus()) {
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
