#include "img_subscriber.hpp"

img_subscriber::img_subscriber()
    : Node("img_subscriber"),
      best_effort(rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()),
      last_store_time_(this->now()) {
    subscription_ = this->create_subscription<octa_ros::msg::Img>(
        "oct_image", best_effort,
        std::bind(&img_subscriber::imageCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&img_subscriber::timerCallback, this));
}

cv::Mat img_subscriber::get_img() {
    std::unique_lock<std::mutex> lock(img_mutex_);
    img_cv_.wait(lock, [this]() { return (img_seq_ > last_read_seq_); });
    last_read_seq_ = img_seq_;
    cv::Mat image_copy = img_.clone();
    return image_copy;
}

void img_subscriber::imageCallback(const octa_ros::msg::Img::SharedPtr msg) {
    auto now = this->now();
    double elapsed = (now - last_store_time_).seconds();
    if (elapsed < gating_interval_) {

        RCLCPP_INFO(this->get_logger(),
                    "Skipping frame (%.2f sec since last store)", elapsed);
        return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Storing new frame after %.2f sec (size=%zu)", elapsed,
                msg->img.size());
    cv::Mat new_img(height_, width_, CV_8UC1);
    std::copy(msg->img.begin(), msg->img.end(), new_img.data);
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        img_ = new_img;
        ++img_seq_;
        last_store_time_ = now;
        img_cv_.notify_all();
    }
}

void img_subscriber::timerCallback() {
    cv::Mat image_copy;
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (img_.empty()) {
            RCLCPP_INFO(this->get_logger(),
                        "timerCallback: No image to process");
            return;
        }
        image_copy = img_.clone();
    }
    cv::Mat current_hash;
    cv::img_hash::AverageHash::create()->compute(image_copy, current_hash);
    if (img_hash_.empty()) {
        img_hash_ = current_hash.clone();
        RCLCPP_INFO(this->get_logger(),
                    "First hash stored. current_hash size=[%dx%d]",
                    current_hash.rows, current_hash.cols);
        return;
    }
    double diff = 0.0; // define early
    if (img_hash_.size == current_hash.size &&
        img_hash_.type() == current_hash.type()) {
        diff = cv::norm(img_hash_, current_hash);
        RCLCPP_INFO(this->get_logger(),
                    "Timer triggered - processing. Hash diff=%.2f", diff);
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Hash mismatch in size or type! old=(%d x %d, type=%d), "
                    "new=(%d x %d, type=%d)",
                    img_hash_.rows, img_hash_.cols, img_hash_.type(),
                    current_hash.rows, current_hash.cols, current_hash.type());
    }
    RCLCPP_INFO(this->get_logger(),
                "Timer triggered - final update. Hash diff=%.2f", diff);
    img_hash_ = current_hash.clone();
}
