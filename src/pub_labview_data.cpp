#include "octa_ros/msg/labviewdata.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
  public:
    MinimalPublisher() : Node("pub_labview_data") {
        publisher_ = this->create_publisher<octa_ros::msg::Labviewdata>(
            "labview_data", 10);

        timer_ = this->create_wall_timer(500ms, [this]() {
            auto message = octa_ros::msg::Labviewdata();
            message.robot_vel = this->robot_vel;
            message.robot_acc = this->robot_acc;
            message.z_tolerance = this->z_tolerance;
            message.angle_tolerance = this->angle_tolerance;
            message.radius = this->radius;
            message.angle_limit = this->angle_limit;
            message.num_pt = this->num_pt;
            message.dz = this->dz;
            message.drot = this->drot;
            message.autofocus = this->autofocus;
            message.freedrive = this->freedrive;
            message.previous = this->previous;
            message.next = this->next;
            message.home = this->home;
            message.reset = this->reset;
            message.fast_axis = this->fast_axis;
            publisher_->publish(message);
        });
    }

  private:
    double count = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<octa_ros::msg::Labviewdata>::SharedPtr publisher_;
    double robot_vel = 0.1;
    double robot_acc = 0.1;
    double z_tolerance = 100;
    double angle_tolerance = 100;
    double radius = 0;
    double angle_limit = 180;
    int num_pt = 18;
    double dz = 1;
    double drot = std::numbers::pi / 2;
    bool autofocus = true;
    bool freedrive = false;
    bool previous = false;
    bool next = false;
    bool home = false;
    bool reset = true;
    bool fast_axis = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
