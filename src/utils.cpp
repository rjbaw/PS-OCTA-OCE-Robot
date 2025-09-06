#include "utils.hpp"
#include <format>

double to_radian(const double degree) {
    return (std::numbers::pi / 180 * degree);
}

double to_degree(const double radian) {
    return (180 / std::numbers::pi * radian);
}

void print_target(rclcpp::Logger const &logger,
                  geometry_msgs::msg::Pose target_pose) {
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
