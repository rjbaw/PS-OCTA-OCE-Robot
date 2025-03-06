#ifndef DDS_PUBLISHER_HPP
#define DDS_PUBLISHER_HPP

#include <octa_ros/msg/robotdata.hpp>
#include <rclcpp/rclcpp.hpp>

class dds_publisher : public rclcpp::Node {
  public:
    dds_publisher(std::string msg = "", double angle = 0.0,
                  int circle_state = 1, bool fast_axis = true,
                  bool apply_config = false, bool end_state = false,
                  bool scan_3d = false);

    void set_msg(const std::string &new_msg);
    void set_angle(double new_angle);
    void set_circle_state(int new_circle_state);
    void set_fast_axis(bool new_fast_axis);
    void set_apply_config(bool new_apply_config);
    void set_end_state(bool new_end_state);
    void set_scan_3d(bool new_scan_3d);

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<octa_ros::msg::Robotdata>::SharedPtr publisher_;
    std::string msg;
    double angle;
    int circle_state;
    bool fast_axis;
    bool apply_config;
    bool end_state;
    bool scan_3d;
    octa_ros::msg::Robotdata old_message = octa_ros::msg::Robotdata();
    std::chrono::steady_clock::time_point apply_config_time_;
};

#endif // DDS_PUBLISHER_HPP_
