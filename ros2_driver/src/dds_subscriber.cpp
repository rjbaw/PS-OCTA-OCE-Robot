#include "dds_subscriber.hpp"

dds_subscriber::dds_subscriber()
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
            if (old_msg_ != *msg) {
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
                                        robot_vel_, robot_acc_, z_tolerance_,
                                        angle_tolerance_, radius_, angle_limit_,
                                        num_pt_, dz_, drot_, autofocus_,
                                        freedrive_, previous_, next_, home_,
                                        reset_, fast_axis_, scan_3d_, z_height_)
                                .c_str());
                changed_ = true;
            } else {
                changed_ = false;
            }
            old_msg_ = *msg;
        });
};

double dds_subscriber::robot_vel() { return robot_vel_; };
double dds_subscriber::robot_acc() { return robot_acc_; };
double dds_subscriber::z_tolerance() { return z_tolerance_; };
double dds_subscriber::angle_tolerance() { return angle_tolerance_; };
double dds_subscriber::radius() { return radius_; };
double dds_subscriber::angle_limit() { return angle_limit_; };
int dds_subscriber::num_pt() { return num_pt_; };
double dds_subscriber::dz() { return dz_; };
double dds_subscriber::drot() { return drot_; };
double dds_subscriber::z_height() { return z_height_; };
bool dds_subscriber::autofocus() { return autofocus_; };
bool dds_subscriber::freedrive() { return freedrive_; };
bool dds_subscriber::previous() { return previous_; };
bool dds_subscriber::next() { return next_; };
bool dds_subscriber::home() { return home_; };
bool dds_subscriber::reset() { return reset_; };
bool dds_subscriber::fast_axis() { return fast_axis_; };
bool dds_subscriber::changed() { return changed_; };
bool dds_subscriber::scan_3d() { return scan_3d_; };
