#include "subscribers.h"

class img_subscriber : public rclcpp::Node {
  public:
    img_subscriber()
        : Node("img_subscriber"),
          best_effort(rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()) {
        subscription_ = this->create_subscription<octa_ros::msg::Img>(
            "oct_image", best_effort,
            std::bind(&img_subscriber::imageCallback, this,
                      std::placeholders::_1));
    }

    cv::Mat get_img() {
        std::lock_guard<std::mutex> lock(img_mutex_);
        return img_.clone();
    }
  private:
    void imageCallback(const octa_ros::msg::Img::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),
                    std::format("Subscribing to image, array length: {}",
                                msg->img.size())
                        .c_str());

        // std::vector<uint8_t> &source_img = msg->img;
        // std::vector<uint8_t> source_img(msg->img.begin(), msg->img.end());
        // cv::Mat img(height_, width_, CV_8UC1, source_img.data());
        cv::Mat img(height_, width_, CV_8UC1);
        std::copy(msg->img.begin(), msg->img.end(), img.data);
        {
            std::lock_guard<std::mutex> lock(img_mutex_);
            img_ = img.clone();
        }
    }

    const int width_ = 500;
    const int height_ = 512;

    cv::Mat img_;
    std::mutex img_mutex_;

    rclcpp::Subscription<octa_ros::msg::Img>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
};
