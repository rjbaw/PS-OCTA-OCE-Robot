#include <chrono>
#include <csignal>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <octa_ros/msg/labviewdata.hpp>
#include <octa_ros/msg/robotdata.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
        : Node("sub_labview")

    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        subscription_ = this->create_subscription<octa_ros::msg::Labviewdata>(
            "labview_data", qos,
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

  private:
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr subscription_;
    double robot_vel_ = 0, robot_acc_ = 0, z_tolerance_ = 0,
           angle_tolerance_ = 0, radius_ = 0, angle_limit_ = 0, dz_ = 0,
           drot_ = 0;
    int num_pt_ = 0;
    bool autofocus_ = false, freedrive_ = false, previous_ = false,
         next_ = false, home_ = false, reset_ = false, fast_axis_ = false,
         changed_ = false;
    octa_ros::msg::Labviewdata old_msg = octa_ros::msg::Labviewdata();
};

class urscript_publisher : public rclcpp::Node {
  public:
    urscript_publisher()
        : Node("urscript_publisher"), freedrive(false), executed(true) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        trigger_client = this->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/resend_robot_program");
        publisher_ = this->create_publisher<std_msgs::msg::String>("/urscript_interface/script_command", qos);
	timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&urscript_publisher::publish_to_robot, this));
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
                // ros2 service call /io_and_status_controller/resend_robot_program
                // std_srvs/srv/Trigger;
                while (!trigger_client->wait_for_service(std::chrono::seconds(1))) {
                    RCLCPP_INFO(this->get_logger(), "Waiting for service...");
                }
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = trigger_client->async_send_request(request);
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", response->message.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", response->message.c_str());
                }
            }
            executed = true;
        }
    }
};

// class urscript_publisher : public rclcpp::Node {
//   public:
//     urscript_publisher()
//         : Node("urscript_publisher") {
// 	if (!executed) {
// 
//             auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
//             publisher_ = this->create_publisher<std_msgs::msg::String>(
//                 "/urscript_interface/script_command", qos);
// 
//             auto message = std_msgs::msg::String();
//             if (freedrive) {
//                 message.data = R"(
// def program():
//  global check = "Made it"
//  while(True):
//   freedrive_mode()
//  end
// end
//             )";
//             } else {
//                 message.data = R"(
// def program():
//  end_freedrive_mode()
// end
//             )";
//             }
//             publisher_->publish(message);
//             RCLCPP_INFO(this->get_logger(), "URscript message published: '%s'",
//                         message.data.c_str());
//             if (!freedrive) {
//                 // ros2 service call /io_and_status_controller/resend_robot_program
//                 // std_srvs/srv/Trigger;
//                 auto trigger_client =
//                     this->create_client<std_srvs::srv::Trigger>(
//                         "/io_and_status_controller/resend_robot_program");
//                 while (!trigger_client->wait_for_service(std::chrono::seconds(1))) {
//                     RCLCPP_INFO(this->get_logger(), "Waiting for service...");
//                 }
//                 auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//                 auto future = trigger_client->async_send_request(request);
//                 auto response = future.get();
//                 if (response->success) {
//                     RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", response->message.c_str());
//                 } else {
//                     RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", response->message.c_str());
//                 }
//             }
//             executed = true;
//             }
//         }
//     void activate_freedrive() {
//         freedrive = true;
// 	executed = false;
//     }
//     void deactivate_freedrive() {
//         freedrive = false;
// 	executed = false;
//     }
//   private:
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//     bool freedrive = false, executed = true;
// };


double to_radian(const double degree) {
    return (std::numbers::pi / 180 * degree);
}
bool tol_measure(double &drot, double &dz, double &angle_tolerance,
                 double &z_tolerance) {
    return ((std::abs((1 / 0.1 * drot)) < to_radian(angle_tolerance)) &&
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

bool image_changed(auto &subscriber_node, double &drot, double &dz,
                   double &angle_tolerance, double &z_tolerance) {
    return ((std::pow(std::abs(subscriber_node->dz()) - std::abs(dz), 2) <
             (z_tolerance)) &&
            (std::pow(std::abs(subscriber_node->drot()) - std::abs(drot), 2) <
             to_radian(angle_tolerance * 1)));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    using moveit::planning_interface::MoveGroupInterface;

    //while (rclcpp::ok() && running) {

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::string msg;
    double angle = 0.0;
    int circle_state = 1;
    bool apply_config = false;
    bool end_state = false;

    int counter = 0;
    bool fast_focused = false;
    bool slow_focused = false;

    double robot_vel;
    double robot_acc;
    double z_tolerance;
    double angle_tolerance;
    double radius;
    double angle_limit;
    double dz;
    double drot;
    int num_pt;
    bool autofocus;
    bool freedrive;
    bool previous;
    bool next;
    bool home;
    bool reset;
    bool fast_axis = false;

    auto const move_group_node =
        std::make_shared<rclcpp::Node>("node_moveit", node_options);
    auto move_group_interface =
        MoveGroupInterface(move_group_node, "ur_manipulator");
    auto subscriber_node = std::make_shared<dds_subscriber>();
    auto urscript_node = std::make_shared<urscript_publisher>();
    //auto activate_freedrive_node = std::make_shared<urscript_publisher>(true);
    //auto deactivate_freedrive_node = std::make_shared<urscript_publisher>(false);

    auto const logger = rclcpp::get_logger("logger_planning");

    // double robot_vel = subscriber_node->robot_vel();
    // double robot_acc = subscriber_node->robot_acc();
    // double z_tolerance = subscriber_node->z_tolerance();
    // double angle_tolerance = subscriber_node->angle_tolerance();
    // double radius = subscriber_node->radius();
    // double angle_limit = subscriber_node->angle_limit();
    // int num_pt = subscriber_node->num_pt();
    // double dz = subscriber_node->dz();
    // double drot = subscriber_node->drot();
    // bool autofocus = subscriber_node->autofocus();
    // bool freedrive = subscriber_node->freedrive();
    // bool previous = subscriber_node->previous();
    // bool next = subscriber_node->next();
    // bool home = subscriber_node->home();
    // bool reset = subscriber_node->reset();
    // bool fast_axis = subscriber_node->fast_axis();

    // rclcpp::executors::SingleThreadedExecutor exec_pub;
    auto publisher_node = std::make_shared<dds_publisher>(
        msg = msg, angle = angle, circle_state = circle_state,
        fast_axis = fast_axis, apply_config = apply_config,
        end_state = end_state);
    // exec_pub.add_node(publisher_node);
    // std::thread([&exec_pub]() { exec_pub.spin(); }).detach();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);
    executor.add_node(urscript_node);
    std::thread([&executor]() { executor.spin(); }).detach();

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
        drot *= 0.2;
        previous = subscriber_node->previous();
        next = subscriber_node->next();
        home = subscriber_node->home();
        reset = subscriber_node->reset();
        fast_axis = subscriber_node->fast_axis();

        if (freedrive) {
            urscript_node->activate_freedrive();
            while (subscriber_node->freedrive()) {
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
            urscript_node->deactivate_freedrive();
        }

        move_group_interface.setMaxVelocityScalingFactor(robot_vel);
        move_group_interface.setMaxAccelerationScalingFactor(robot_acc);
        // move_group_interface.setStartStateToCurrentState();
        move_group_interface.setPoseReferenceFrame("tool0");

        double angle_increment = angle_limit / num_pt;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        geometry_msgs::msg::Pose target_pose = move_group_interface.getCurrentPose().pose;

        if (reset) {
            msg = "Reset to default position", angle = 0.0, circle_state = 1;
            target_pose.orientation.x = -0.7071068;
            target_pose.orientation.y = 0.7071068;
            target_pose.orientation.z = 0.0;
            target_pose.orientation.w = 0.0;
            target_pose.position.x = 0.4;
            target_pose.position.y = 0.0;
            target_pose.position.z = 0.0;
        } else {
            if (autofocus) {
                // while (
                //     !image_changed(subscriber_node, drot, dz,
                //     angle_tolerance, z_tolerance) || !tol_measure(drot, dz,
                //     angle_tolerance, z_tolerance) ) { if
                //     (!subscriber_node->autofocus()) {
                //         break;
                //     }
                // }
                if (fast_axis) {
                    pitch += -drot;
                } else {
                    roll += drot;
                }
                target_pose.position.x += radius * std::cos(to_radian(angle));
                target_pose.position.y += radius * std::sin(to_radian(angle));
                target_pose.position.z += -dz;
            } else {
                counter = 0;
                fast_focused = false;
                slow_focused = false;
                if (next) {
                    angle += angle_increment;
                    yaw += to_radian(angle_increment);
                }
                if (previous) {
                    angle -= angle_increment;
                    yaw += to_radian(-angle_increment);
                }
                if (home) {
                    yaw += to_radian(-angle);
                    angle = 0.0;
                }
            }

            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q.normalize();

            tf2::Quaternion target_q;
            tf2::fromMsg(target_pose.orientation, target_q);
            target_q = target_q * q;
            target_pose.orientation = tf2::toMsg(target_q);
        }

        if (reset || autofocus || next || previous || home) {
            if (tol_measure(drot, dz, angle_tolerance, z_tolerance) && autofocus) {
                if (fast_axis) {
                    fast_axis = false;
                    apply_config = true;
                    counter += 1;
                    fast_focused = true;
                } else {
                    fast_axis = true;
                    apply_config = true;
                    counter += 1;
                    slow_focused = true;
                }
                if ((counter > 3) && fast_focused && slow_focused) {
                    end_state = true;
                    msg = "Autofocus complete";
                }
            } else {
                RCLCPP_INFO(logger, std::format("Target Pose: "
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
        }

        if (autofocus) {
            //while (!image_changed(subscriber_node, drot, dz, angle_tolerance, z_tolerance)) {
            while (!subscriber_node->changed()) {
		if (tol_measure(drot, dz, angle_tolerance, z_tolerance)) {
		    break;
		}
                if (!subscriber_node->autofocus()) {
                    break;
                }
            }
            // rclcpp::sleep_for(std::chrono::seconds(1));
	
            if (tol_measure(drot, dz, angle_tolerance, z_tolerance) && autofocus) {
                if (fast_axis) {
                    fast_axis = false;
                    apply_config = true;
                    counter += 1;
                    fast_focused = true;
                } else {
                    fast_axis = true;
                    apply_config = true;
                    counter += 1;
                    slow_focused = true;
                }
                if ((counter > 3) && fast_focused && slow_focused) {
                    end_state = true;
                    msg = "Autofocus complete";
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
    //}
    return 0;
}
