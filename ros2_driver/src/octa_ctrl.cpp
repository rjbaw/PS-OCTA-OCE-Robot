#include "octa_ros/msg/img.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <csignal>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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

// #include "subscribers.h"

using namespace std::chrono_literals;
std::atomic<bool> running(true);

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"),
                "Signal %d received, shutting down...", signum);
    running = false;
    // rclcpp::shutdown();
}

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

struct SegmentResult {
    cv::Mat image;
    std::vector<cv::Point> coordinates;
};

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

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord) {
    for (size_t i = 0; i < ret_coord.size() - 1; ++i) {
        cv::Point pt1 = ret_coord[i];
        cv::Point pt2 = ret_coord[i + 1];
        cv::line(image, pt1, pt2, cv::Scalar(255, 0, 0), 2);
    }
}

SegmentResult detect_lines(const cv::Mat &img) {
    SegmentResult result;

    cv::Mat denoised_image;
    cv::medianBlur(img, denoised_image, 5);

    cv::Mat processed_img = denoised_image.clone();
    processed_img.convertTo(processed_img, CV_64F);

    cv::Mat row_means;
    cv::reduce(processed_img, row_means, 1, cv::REDUCE_AVG);
    cv::Mat mean_mat;
    cv::repeat(row_means, 1, processed_img.cols, mean_mat);
    processed_img -= mean_mat;

    cv::Mat sobely;
    cv::Sobel(processed_img, sobely, CV_64F, 0, 1, 9);

    std::vector<cv::Point> ret_coords = get_max_coor(sobely);

    std::vector<double> observations;
    for (const auto &pt : ret_coords) {
        observations.push_back(pt.y);
    }

    size_t obs_length = observations.size();
    int window = std::max(1, static_cast<int>(obs_length / 20));

    for (size_t i = 0; i < obs_length; ++i) {
        int start = std::max(0, static_cast<int>(i) - window);
        int end = std::min(static_cast<int>(obs_length),
                           static_cast<int>(i) + window + 1);
        std::vector<double> window_vals(observations.begin() + start,
                                        observations.begin() + end);

        double mu =
            std::accumulate(window_vals.begin(), window_vals.end(), 0.0) /
            window_vals.size();

        double accum = 0.0;
        for (double val : window_vals) {
            accum += (val - mu) * (val - mu);
        }
        double sigma = std::sqrt(accum / window_vals.size());

        std::vector<double> sorted_vals = window_vals;
        std::nth_element(sorted_vals.begin(),
                         sorted_vals.begin() + sorted_vals.size() / 2,
                         sorted_vals.end());
        double med = sorted_vals[sorted_vals.size() / 2];

        double z = (sigma != 0.0) ? (observations[i] - mu) / sigma
                                  : observations[i] - mu;

        if (z > 0.5) {
            observations[i] = med;
        }
    }

    double x_k = observations[0];
    double P_k = 1.0;
    double Q = 0.01;
    double R = 5.0;
    std::vector<double> x_k_estimates;

    for (double z_k : observations) {
        double x_k_pred = x_k;
        double P_k_pred = P_k + Q;
        double K_k = P_k_pred / (P_k_pred + R);
        x_k = x_k_pred + K_k * (z_k - x_k_pred);
        P_k = (1 - K_k) * P_k_pred;
        x_k_estimates.push_back(x_k);
    }

    for (size_t i = 0; i < ret_coords.size(); ++i) {
        ret_coords[i].y = static_cast<int>(x_k_estimates[i]);
    }

    cv::Mat detected_image = img.clone();
    draw_line(detected_image, ret_coords);
    cv::imwrite("detected_image.jpg", detected_image);

    result.image = detected_image;
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

        timer_ = this->create_wall_timer(10ms, [this]() {
            auto message = octa_ros::msg::Robotdata();
            message.msg = this->msg;
            message.angle = this->angle;
            message.circle_state = this->circle_state;
            message.fast_axis = this->fast_axis;
            message.apply_config = this->apply_config;
            message.end_state = this->end_state;
            message.scan_3d = this->scan_3d;
            if (old_message != message) {
                RCLCPP_INFO(this->get_logger(),
                            std::format("[PUBLISHING] "
                                        " msg: {} ,"
                                        " angle: {},"
                                        " circle_state: {},"
                                        " fast_axis: {},"
                                        " apply_config: {},"
                                        " end_state: {},"
                                        " scan_3d: {}",
                                        this->msg, this->angle,
                                        this->circle_state, this->fast_axis,
                                        this->apply_config, this->end_state,
                                        this->scan_3d)
                                .c_str());
            }
            publisher_->publish(message);
            old_message = message;
        });
    };

    void set_msg(const std::string &new_msg) { msg = new_msg; }
    void set_angle(double new_angle) { angle = new_angle; }
    void set_circle_state(int new_circle_state) {
        circle_state = new_circle_state;
    }
    void set_fast_axis(bool new_fast_axis) { fast_axis = new_fast_axis; }
    void set_apply_config(bool new_apply_config) {
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
                                            " scan_3d: {}",
                                            robot_vel_, robot_acc_,
                                            z_tolerance_, angle_tolerance_,
                                            radius_, angle_limit_, num_pt_, dz_,
                                            drot_, autofocus_, freedrive_,
                                            previous_, next_, home_, reset_,
                                            fast_axis_, scan_3d_)
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
           drot_ = 0;
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

double to_radian(const double degree) {
    return (std::numbers::pi / 180 * degree);
}
bool tol_measure(Eigen::Matrix3d &drot, double &dz, double &angle_tolerance,
                 double &z_tolerance, double &scale_factor) {
    return ((std::abs((1 / scale_factor * std::abs(drot.trace() - 3))) <
             to_radian(angle_tolerance)) &&
            (std::abs(dz) < (z_tolerance / 1000.0)));
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
    bool apply_config = false;
    bool end_state = false;
    bool scan_3d = false;

    // Internal Parameters
    bool planning = false;
    double scale_factor = 0.25;
    double angle_increment;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    Eigen::Matrix3d rotmat_eigen;
    cv::Mat img;
    std::vector<cv::Mat> img_array;
    std::vector<Eigen::Vector3d> pc_lines;

    // Subscriber Parameters
    double robot_vel, robot_acc, radius, angle_limit, dz, drot;
    double z_tolerance, angle_tolerance;
    int num_pt;
    bool autofocus, freedrive, previous, next, home, reset, fast_axis;

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

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);
    executor.add_node(urscript_node);
    executor.add_node(img_subscriber_node);
    std::thread spinner([&executor]() { executor.spin(); });

    add_collision_obj(move_group_interface);

    while (rclcpp::ok() && running) {

        end_state = false;
        apply_config = false;
        fast_axis = true;
        autofocus = subscriber_node->autofocus();
        freedrive = subscriber_node->freedrive();
        robot_vel = subscriber_node->robot_vel();
        robot_acc = subscriber_node->robot_acc();
        z_tolerance = subscriber_node->z_tolerance();
        angle_tolerance = subscriber_node->angle_tolerance();
        radius = subscriber_node->radius();
        angle_limit = subscriber_node->angle_limit();
        num_pt = subscriber_node->num_pt();
        dz = subscriber_node->dz();
        drot = subscriber_node->drot();
        drot *= scale_factor;
        previous = subscriber_node->previous();
        next = subscriber_node->next();
        home = subscriber_node->home();
        reset = subscriber_node->reset();
        scan_3d = subscriber_node->scan_3d();

        if (freedrive) {
            circle_state = 1;
            angle = 0.0;
            urscript_node->activate_freedrive();
            while (subscriber_node->freedrive()) {
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
            urscript_node->deactivate_freedrive();
        }

        move_group_interface.setMaxVelocityScalingFactor(robot_vel);
        move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
        move_group_interface.setStartStateToCurrentState();
        geometry_msgs::msg::Pose target_pose;

        if (reset) {
            planning = true;
            msg = "Reset to default position", angle = 0.0, circle_state = 1;
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
                                                     to_radian(45.0));
        } else {
            angle_increment = angle_limit / num_pt;
            roll = 0.0, pitch = 0.0, yaw = 0.0;
            target_pose = move_group_interface.getCurrentPose().pose;

            if (autofocus) {
                if (!scan_3d) {
                    planning = false;
                    scan_3d = true;
                    apply_config = true;
                    msg = "Starting 3D Scan";
                } else {
                    img_array.clear();
                    for (int i = 0; i < interval; i++) {
                        while (true) {
                            img = img_subscriber_node->get_img();
                            if (!img.empty()) {
                                break;
                            }
                            rclcpp::sleep_for(std::chrono::milliseconds(100));
                        }
                        img_array.push_back(img);
                        RCLCPP_INFO(logger, "Collected image %d", i + 1);
                    }
                    pc_lines = lines_3d(img_array, interval, single_interval);
                    open3d::geometry::PointCloud pcd_lines;
                    for (const auto &point : pc_lines) {
                        pcd_lines.points_.emplace_back(point);
                    }
                    auto boundbox =
                        pcd_lines.GetMinimalOrientedBoundingBox(false);
                    Eigen::Vector3d center = boundbox.GetCenter();
                    double z_height = center[2];
                    dz = (z_height - 190) / 150 * 1.2 / 1000;
                    rotmat_eigen = align_to_direction(boundbox.R_);
                    tf2::Matrix3x3 rotmat_tf(
                        rotmat_eigen(0, 0), rotmat_eigen(0, 1),
                        rotmat_eigen(0, 2), rotmat_eigen(1, 0),
                        rotmat_eigen(1, 1), rotmat_eigen(1, 2),
                        rotmat_eigen(2, 0), rotmat_eigen(2, 1),
                        rotmat_eigen(2, 2));
                    std::cout << "Aligned Rotation Matrix:\n"
                              << rotmat_eigen << std::endl;

                    if (tol_measure(rotmat_eigen, dz, angle_tolerance,
                                    z_tolerance, scale_factor)) {
                        planning = false;
                        scan_3d = false;
			apply_config = true;
                        end_state = true;
                        msg = "Autofocus complete";
                    } else {
                        planning = true;
                        tf2::Quaternion q;
                        tf2::Quaternion target_q;
                        tf2::fromMsg(target_pose.orientation, target_q);
                        rotmat_tf.getRotation(q);
                        q.normalize();
                        target_q = target_q * q;
                        target_pose.orientation = tf2::toMsg(target_q);
                        target_pose.position.x +=
                            radius * std::cos(to_radian(angle));
                        target_pose.position.y +=
                            radius * std::sin(to_radian(angle));
                        target_pose.position.z += -dz;
                        RCLCPP_INFO(
                            logger,
                            std::format("Target Pose: "
                                        " x: {}, y: {}, z: {},"
                                        " qx: {}, qy: {}, qz: {}, qw: {}",
                                        target_pose.position.x,
                                        target_pose.position.y,
                                        target_pose.position.z,
                                        target_pose.orientation.x,
                                        target_pose.orientation.y,
                                        target_pose.orientation.z,
                                        target_pose.orientation.w)
                                .c_str());
                        move_group_interface.setPoseTarget(target_pose);
                    }
                }
            } else {
                if (next) {
                    planning = true;
                    angle += angle_increment;
                    circle_state++;
                    yaw += to_radian(angle_increment);
                }
                if (previous) {
                    planning = true;
                    angle -= angle_increment;
                    circle_state--;
                    yaw += to_radian(-angle_increment);
                }
                if (home) {
                    planning = true;
                    yaw += to_radian(-angle);
                    circle_state = 0;
                    angle = 0.0;
                }

                if (planning) {
                    tf2::Quaternion q;
                    tf2::Quaternion target_q;
                    tf2::fromMsg(target_pose.orientation, target_q);
                    q.setRPY(roll, pitch, yaw);
                    q.normalize();
                    target_q = target_q * q;
                    target_pose.orientation = tf2::toMsg(target_q);
                    RCLCPP_INFO(logger,
                                std::format("Target Pose: "
                                            " x: {}, y: {}, z: {},"
                                            " qx: {}, qy: {}, qz: {}, qw: {}",
                                            target_pose.position.x,
                                            target_pose.position.y,
                                            target_pose.position.z,
                                            target_pose.orientation.x,
                                            target_pose.orientation.y,
                                            target_pose.orientation.z,
                                            target_pose.orientation.w)
                                    .c_str());
                    move_group_interface.setPoseTarget(target_pose);
                }
            }
        }

        if (planning) {
            auto const [success, plan] = [&move_group_interface] {
                moveit::planning_interface::MoveGroupInterface::Plan
                    plan_feedback;
                auto const ok =
                    static_cast<bool>(move_group_interface.plan(plan_feedback));
                return std::make_pair(ok, plan_feedback);
            }();
            if (success) {
                move_group_interface.execute(plan);
                msg = "Planning Success!";
            } else {
                RCLCPP_ERROR(logger, "Planning failed!");
                msg = "Planning failed!";
            }
        }

        publisher_node->set_msg(msg);
        publisher_node->set_angle(angle);
        publisher_node->set_circle_state(circle_state);
        publisher_node->set_fast_axis(fast_axis);
        publisher_node->set_apply_config(apply_config);
        publisher_node->set_end_state(end_state);
        publisher_node->set_scan_3d(scan_3d);

        if (apply_config && !end_state && scan_3d) {
            rclcpp::sleep_for(std::chrono::milliseconds(4000));
        }

        if (planning) {
            planning = false;
            while (!subscriber_node->changed()) {
                if (tol_measure(rotmat_eigen, dz, angle_tolerance, z_tolerance,
                                scale_factor)) {
                    break;
                }
                if (!subscriber_node->autofocus()) {
                    break;
                }
            }
        }
	if (scan_3d && !autofocus) {
            publisher_node->set_apply_config(true);
            publisher_node->set_scan_3d(false);
	}
    }

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
