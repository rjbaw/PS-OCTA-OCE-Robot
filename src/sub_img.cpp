/**
 * @file sub_img.cpp
 * @author rjbaw
 */

#include "octa_ros/msg/img.hpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <memory>
#include <opencv2/opencv.hpp>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

class MinimalSubscriber : public rclcpp::Node {
  public:
    MinimalSubscriber()
        : Node("img_sub"), best_effort(rclcpp::KeepLast(10))

    {
        subscription_ = this->create_subscription<octa_ros::msg::Img>(
            "oct_image", best_effort.reliability(be),
            [this](const octa_ros::msg::Img::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Subscribing");
                RCLCPP_INFO(this->get_logger(),
                            std::format("length: {}", msg->img.size()).c_str());
                int width = 500;
                int height = 512;
                cv::Mat img(height, width, CV_8UC1, msg->img.data());

                cv::imwrite("test/test.jpg", img);
                std::string filename = std::format("test/test{}.jpg", count);
                if (std::filesystem::exists(filename.c_str())) {
                    count++;
                    filename = std::format("test/test{}.jpg", count);
                }
                cv::imwrite(filename, img);
            });
    }

  private:
    rclcpp::Subscription<octa_ros::msg::Img>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
    int count = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
