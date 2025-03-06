#ifndef IMG_SUBSCRIBER_HPP_
#define IMG_SUBSCRIBER_HPP_

#include <condition_variable>
#include <mutex>
#include <octa_ros/msg/img.hpp>
#include <opencv2/img_hash.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

class img_subscriber : public rclcpp::Node {
  public:
    img_subscriber();
    cv::Mat get_img();

  private:
    void imageCallback(const octa_ros::msg::Img::SharedPtr msg);
    void timerCallback();
    const double gating_interval_ = 0.05;
    const int width_ = 500;
    const int height_ = 512;
    cv::Mat img_;
    cv::Mat img_hash_;
    rclcpp::QoS best_effort;
    rclcpp::Time last_store_time_;
    uint64_t img_seq_{0};
    uint64_t last_read_seq_{0};
    std::mutex img_mutex_;
    std::condition_variable img_cv_;
    rclcpp::Subscription<octa_ros::msg::Img>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // IMG_SUBSCRIBER_HPP_
