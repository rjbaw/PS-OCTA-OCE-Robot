#ifndef DDS_SUBSCRIBER_HPP_
#define DDS_SUBSCRIBER_HPP_

#include <mutex>
#include <octa_ros/msg/labviewdata.hpp>
#include <rclcpp/rclcpp.hpp>

class dds_subscriber : public rclcpp::Node {
  public:
    dds_subscriber();
    double robot_vel();
    double robot_acc();
    double z_tolerance();
    double angle_tolerance();
    double radius();
    double angle_limit();
    int num_pt();
    double dz();
    double drot();
    double z_height();
    bool autofocus();
    bool freedrive();
    bool previous();
    bool next();
    bool home();
    bool reset();
    bool fast_axis();
    bool changed();
    bool scan_3d();

  private:
    rclcpp::Subscription<octa_ros::msg::Labviewdata>::SharedPtr subscription_;

    double robot_vel_ = 0.0;
    double robot_acc_ = 0.0;
    double z_tolerance_ = 0.0;
    double angle_tolerance_ = 0.0;
    double radius_ = 0.0;
    double angle_limit_ = 0.0;
    double dz_ = 0.0;
    double drot_ = 0.0;
    double z_height_ = 0.0;

    int num_pt_ = 0;

    bool autofocus_ = false;
    bool freedrive_ = false;
    bool previous_ = false;
    bool next_ = false;
    bool home_ = false;
    bool reset_ = false;
    bool fast_axis_ = false;
    bool changed_ = false;
    bool scan_3d_ = false;

    octa_ros::msg::Labviewdata old_msg_;
    std::mutex data_mutex_;
};

#endif // DDS_SUBSCRIBER_HPP_
