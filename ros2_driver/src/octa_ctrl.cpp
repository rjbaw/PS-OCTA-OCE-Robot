#include "octa_ros/msg/labviewdata.hpp"
#include "octa_ros/msg/robotdata.hpp"
#include <chrono>
#include <csignal>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

using namespace std::chrono_literals;
std::atomic<bool> running(true);

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"),
                "Signal %d received, shutting down...", signum);
    running = false;
    rclcpp::shutdown();
}

class dds_publisher : public rclcpp::Node {
  public:
    dds_publisher() : dds_publisher("", 0.0, 0, false, false, false) {}

    dds_publisher(std::string msg = "", double angle = 0.0,
                  int circle_state = 0, bool fast_axis = false,
                  bool apply_config = false, bool end_state = false)
        : Node("pub_robot_data"), msg(msg), angle(angle),
          circle_state(circle_state), fast_axis(fast_axis),
          apply_config(apply_config), end_state(end_state) {
        publisher_ =
            this->create_publisher<octa_ros::msg::Robotdata>("robot_data", 10);

        timer_ = this->create_wall_timer(10ms, [this]() {
            auto message = octa_ros::msg::Robotdata();
            message.msg = this->msg;
            message.angle = this->angle;
            message.circle_state = this->circle_state;
            message.fast_axis = this->fast_axis;
            message.apply_config = this->apply_config;
            message.end_state = this->end_state;
            if (old_message != message) {
                RCLCPP_INFO(this->get_logger(),
                            std::format("Publishing: "
                                        " msg: {} ,"
                                        " angle: {},"
                                        " circle_state: {},"
                                        " apply_config: {},"
                                        " end_state: {},",
                                        this->msg, this->angle,
                                        this->circle_state, this->apply_config,
                                        this->end_state)
                                .c_str());
                publisher_->publish(message);
            }
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

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr publisher_;
    std::string msg;
    double angle;
    int circle_state;
    bool fast_axis, apply_config, end_state;
    octa_ros::msg::Robotdata old_message = octa_ros::msg::Robotdata();
};

class dds_subscriber : public rclcpp::Node {
  public:
    dds_subscriber()
        : Node("sub_labview"), best_effort(rclcpp::KeepLast(10))

    {
        subscription_ = this->create_subscription<octa_ros::msg::Labviewdata>(
            "labview_data", best_effort.reliability(be),
            [this](const octa_ros::msg::Labviewdata::SharedPtr msg) {
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
                if (old_msg != *msg) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        std::format("Subscribing: "
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
                                    " fast_axis: {},",
                                    robot_vel_, robot_acc_, z_tolerance_,
                                    angle_tolerance_, radius_, angle_limit_,
                                    num_pt_, dz_, drot_, autofocus_, freedrive_,
                                    previous_, next_, home_, reset_, fast_axis_)
                            .c_str());
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

  private:
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
    double robot_vel_ = 0, robot_acc_ = 0, z_tolerance_ = 0,
           angle_tolerance_ = 0, radius_ = 0, angle_limit_ = 0, dz_ = 0,
           drot_ = 0;
    int num_pt_ = 0;
    bool autofocus_ = false, freedrive_ = false, previous_ = false,
         next_ = false, home_ = false, reset_ = false, fast_axis_ = false;
    octa_ros::msg::Labviewdata old_msg = octa_ros::msg::Labviewdata();
};

double to_radian(const double degree) {
    return (std::numbers::pi / 180 * degree);
}
bool tol_measure(double &drot, double &dz, double &angle_tolerance,
                 double &z_tolerance) {
    return ((std::abs((1 / 0.4 * drot)) < to_radian(angle_tolerance)) &&
            (std::abs(dz) < z_tolerance));
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

    auto const move_group_node =
        std::make_shared<rclcpp::Node>("node_moveit", node_options);
    auto move_group_interface =
        MoveGroupInterface(move_group_node, "ur_manipulator");
    auto subscriber_node = std::make_shared<dds_subscriber>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(subscriber_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto const logger = rclcpp::get_logger("logger_planning");

    std::string msg;
    double angle = 0.0;
    int circle_state = 1;
    bool apply_config = false;
    bool end_state = false;

    bool changed = false;
    // double old_dz = 0.0;
    // double old_drot = 0.0;
    int counter = 0;

    double robot_vel = subscriber_node->robot_vel();
    double robot_acc = subscriber_node->robot_acc();
    double z_tolerance = subscriber_node->z_tolerance();
    double angle_tolerance = subscriber_node->angle_tolerance();
    double radius = subscriber_node->radius();
    double angle_limit = subscriber_node->angle_limit();
    int num_pt = subscriber_node->num_pt();
    double dz = subscriber_node->dz();
    double drot = subscriber_node->drot();
    bool autofocus = subscriber_node->autofocus();
    bool freedrive = subscriber_node->freedrive();
    bool previous = subscriber_node->previous();
    bool next = subscriber_node->next();
    bool home = subscriber_node->home();
    bool reset = subscriber_node->reset();
    bool fast_axis = subscriber_node->fast_axis();
    drot *= 0.25;

    rclcpp::executors::SingleThreadedExecutor exec_pub;
    auto publisher_node = std::make_shared<dds_publisher>(
        msg = msg, angle = angle, circle_state = circle_state,
        fast_axis = fast_axis, apply_config = apply_config,
        end_state = end_state);
    exec_pub.add_node(publisher_node);
    std::thread([&exec_pub]() { exec_pub.spin(); }).detach();

    add_collision_obj(move_group_interface);

    while (rclcpp::ok() && running) {

        apply_config = false;
        end_state = false;
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
        previous = subscriber_node->previous();
        next = subscriber_node->next();
        home = subscriber_node->home();
        reset = subscriber_node->reset();
        fast_axis = subscriber_node->fast_axis();

        if (freedrive) {
            continue;
        }

        move_group_interface.setMaxVelocityScalingFactor(robot_vel);
        move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
        move_group_interface.setStartStateToCurrentState();
        // move_group_interface.setPoseReferenceFrame("tool0");

        double angle_increment = angle_limit / num_pt;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Quaternion q;
        geometry_msgs::msg::Pose target_pose =
            move_group_interface.getCurrentPose().pose;

        if (autofocus
            // if (autofocus &&
            //     (std::abs(std::abs(old_dz) - std::abs(dz)) >
            //     (z_tolerance*0.01)) && (std::abs(std::abs(old_drot) -
            //     std::abs(drot)) > to_radian(angle_tolerance*0.01))
        ) {
            if (fast_axis) {
                // roll += drot;
                roll += -drot;
            } else {
                // pitch += -drot;
                pitch += -drot;
            }
            target_pose.position.x += radius * std::cos(to_radian(angle));
            target_pose.position.y += radius * std::sin(to_radian(angle));
            target_pose.position.z += -dz;
            // old_dz = dz;
            // old_drot = drot;
        } else {

            counter = 0;
            if (next) {
                if (!changed) {
                    angle += angle_increment;
                    yaw += to_radian(angle_increment);
                    changed = true;
                }
            }
            if (previous) {
                angle -= angle_increment;
                yaw += to_radian(-angle_increment);
            }
            if (home) {
                yaw += to_radian(-angle);
                angle = 0.0;
            }
            if (changed && !next && !previous && !home) {
                changed = false;
            }
        }

        q.setRPY(roll, pitch, yaw);
        q.normalize();

        tf2::Quaternion target_q;
        tf2::fromMsg(target_pose.orientation, target_q);
        target_q = target_q * q;
        target_pose.orientation = tf2::toMsg(target_q);

        if (reset) {
            // geometry_msgs::msg::Pose target_pose;
            msg = "", angle = 0.0, circle_state = 1;
            target_pose.orientation.x = -0.4;
            target_pose.orientation.y = 0.9;
            target_pose.orientation.z = 0.0;
            target_pose.orientation.w = 0.03;
            target_pose.position.x = 0.36;
            target_pose.position.y = -0.13;
            target_pose.position.z = 0.18;
            // move_group_interface.setPoseTarget(target_pose);
            // move_group_interface.move();
            // continue;
        }

        if (autofocus || reset || next || previous || home) {
            if (autofocus &&
                tol_measure(drot, dz, angle_tolerance, z_tolerance)) {
                if (fast_axis) {
                    fast_axis = false;
                    apply_config = true;
                    counter += 1;
                } else {
                    fast_axis = true;
                    apply_config = true;
                    counter += 1;
                }
                if (counter >= 2) {
                    end_state = true;
                    counter = 0;
                }
            } else {
                RCLCPP_INFO(
                    logger,
                    std::format(
                        "Target Pose: "
                        " x: {}, y: {}, z: {},"
                        " qx: {}, qy: {}, qz: {}, qw: {},",
                        target_pose.position.x, target_pose.position.y,
                        target_pose.position.z, target_pose.orientation.x,
                        target_pose.orientation.y, target_pose.orientation.z,
                        target_pose.orientation.w)
                        .c_str());

                move_group_interface.setPoseTarget(target_pose);
                // move_group_interface.setPoseTarget(target_pose,
                // "end_effector_link");
                // move_group_interface.setPoseTarget(target_pose, "tool0");
                auto const [success, plan] = [&move_group_interface] {
                    moveit::planning_interface::MoveGroupInterface::Plan
                        plan_feedback;
                    auto const ok = static_cast<bool>(
                        move_group_interface.plan(plan_feedback));
                    return std::make_pair(ok, plan_feedback);
                }();
                if (success) {
                    move_group_interface.execute(plan);
                } else {
                    RCLCPP_ERROR(logger, "Planning failed!");
                }

                // rclcpp::sleep_for(std::chrono::seconds(1));
                while (
                    (autofocus &&
                     (std::pow(std::abs(subscriber_node->dz()) - std::abs(dz),
                               2) < (z_tolerance * 0.005)) &&
                     (std::pow(std::abs(subscriber_node->drot()) -
                                   std::abs(drot),
                               2) < to_radian(angle_tolerance * 0.005))) ||
                    tol_measure(drot, dz, angle_tolerance, z_tolerance)) {
                    if (!subscriber_node->autofocus()) {
                        break;
                    }
                }
            }
        }

        publisher_node->set_msg(msg);
        publisher_node->set_angle(angle);
        publisher_node->set_circle_state(circle_state);
        publisher_node->set_fast_axis(fast_axis);
        publisher_node->set_apply_config(apply_config);
        publisher_node->set_end_state(end_state);
    }

    rclcpp::shutdown();
    return 0;
}
