#include "img_subscriber.hpp"

img_subscriber::img_subscriber()
    : Node("img_subscriber"),
      best_effort(rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()) {
    subscription_ = this->create_subscription<octa_ros::msg::Img>(
        "oct_image", best_effort,
        std::bind(&img_subscriber::imageCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&img_subscriber::timerCallback, this));
}

cv::Mat img_subscriber::get_img() {
    std::unique_lock<std::mutex> lock(img_mutex_);
    img_status_.wait(lock, [this] { return new_img_; });
    cv::Mat img_copy = img_.clone();
    new_img_ = false;
    return img_copy;
}

void img_subscriber::imageCallback(const octa_ros::msg::Img::SharedPtr msg) {
    RCLCPP_DEBUG(
        this->get_logger(),
        std::format("Subscribing to image, length: {}", msg->img.size())
            .c_str());
    cv::Mat new_img(height_, width_, CV_8UC1);
    std::copy(msg->img.begin(), msg->img.end(), new_img.data);
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        img_ = new_img;
        new_img_ = true;
        img_status_.notify_one();
    }
}

void img_subscriber::timerCallback() {
    cv::Mat image_to_process;
    {
        std::lock_guard<std::mutex> lock(img_mutex_);

        if (img_.empty()) {
            RCLCPP_DEBUG(this->get_logger(),
                         "No new image to process at this timer tick");
            return;
        }
        image_to_process = img_.clone();
    }
    cv::Mat current_hash;
    cv::img_hash::AverageHash::create()->compute(image_to_process,
                                                 current_hash);
    double diff = cv::norm(img_hash_, current_hash);

    RCLCPP_INFO(this->get_logger(),
                "Timer triggered - processing image. Hash diff = %.2f", diff);
    img_hash_ = current_hash;
}
