#ifndef IMG_SUBSCRIBER_HPP
#define IMG_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <octa_ros/msg/img.hpp>
#include <mutex>

class img_subscriber : public rclcpp::Node {
public:
    img_subscriber();
    cv::Mat get_img();

private:
    void imageCallback(const octa_ros::msg::Img::SharedPtr msg);

    const int width_ = 500;
    const int height_ = 512;

    cv::Mat img_;
    std::mutex img_mutex_;

    rclcpp::Subscription<octa_ros::msg::Img>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
};

#endif // IMG_SUBSCRIBER_HPP
