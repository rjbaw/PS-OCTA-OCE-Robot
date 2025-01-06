#ifndef IMG_SUBSCRIBER_HPP_
#define IMG_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/img_hash.hpp>
#include <opencv2/opencv.hpp>
#include <octa_ros/msg/img.hpp>

class img_subscriber : public rclcpp::Node {
public:
    img_subscriber();
    cv::Mat get_img();

private:
    void imageCallback(const octa_ros::msg::Img::SharedPtr msg);

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

#endif // IMG_SUBSCRIBER_HPP_
