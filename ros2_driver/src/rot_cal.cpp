#include "octa_ros/msg/img.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <memory>
#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
std::atomic<bool> running(true);

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"),
                "Signal %d received, shutting down...", signum);
    running = false;
    rclcpp::shutdown();
}

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
        RCLCPP_INFO(this->get_logger(), "Subscribing to image");
        RCLCPP_INFO(this->get_logger(),
                    std::format("length: {}", msg->img.size()).c_str());

        // uint8_t &source_img = msg->img;
        // for (int row = 0; row < height; row++) {
        //     for (int col = 0; col < width; col++) {
        //         reshape_img[row][col] = source_img[(col + row *
        //         width)];
        //     }
        // }

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

    // uint8_t reshape_img[512][500];
    cv::Mat img_;
    std::mutex img_mutex_;

    rclcpp::Subscription<octa_ros::msg::Img>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
};

struct SegmentResult {
    cv::Mat image;
    cv::Mat coordinates;
};

SegmentResult detect_lines(const cv::Mat &img) {
    SegmentResult result;

    cv::Mat gray_img;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    } else {
        gray_img = img.clone();
    }

    double alpha = 1.5;
    int beta = 100;
    cv::Mat adjusted_img;
    gray_img.convertTo(adjusted_img, -1, alpha, beta);

    cv::Mat laplacian_img;
    cv::Laplacian(adjusted_img, laplacian_img, CV_64F, 3);
    cv::convertScaleAbs(laplacian_img, laplacian_img);

    cv::Mat gray_image = laplacian_img;

    cv::Mat output_image;
    cv::cvtColor(gray_image, output_image, cv::COLOR_GRAY2BGR);

    int height = gray_image.rows;
    int width = gray_image.cols;

    std::vector<std::pair<int, int>> dpoints;

    for (int x = 0; x < width; ++x) {
        std::vector<int> black_regions;

        for (int y = 0; y < height; ++y) {
            uchar intensity = gray_image.at<uchar>(y, x);
            if (intensity < 128) {
                black_regions.push_back(y);
            }
        }

        if (!black_regions.empty()) {
            std::vector<std::vector<int>> segments;
            std::vector<int> current_segment;
            current_segment.push_back(black_regions[0]);

            for (size_t i = 1; i < black_regions.size(); ++i) {
                if (black_regions[i] == black_regions[i - 1] + 1) {
                    current_segment.push_back(black_regions[i]);
                } else {
                    segments.push_back(current_segment);
                    current_segment.clear();
                    current_segment.push_back(black_regions[i]);
                }
            }
            segments.push_back(current_segment);

            auto max_segment_it = std::max_element(
                segments.begin(), segments.end(),
                [](const std::vector<int> &a, const std::vector<int> &b) {
                    return a.size() < b.size();
                });
            std::vector<int> max_segment = *max_segment_it;

            int detected_y =
                *std::min_element(max_segment.begin(), max_segment.end());
            dpoints.emplace_back(std::make_pair(x, detected_y));
        }
    }

    std::vector<int> dpoints_x;
    std::vector<float> dpoints_y;

    dpoints_x.reserve(dpoints.size());
    dpoints_y.reserve(dpoints.size());

    for (const auto &point : dpoints) {
        dpoints_x.push_back(point.first);
        dpoints_y.push_back(static_cast<float>(point.second));
    }

    cv::Mat y_mat(dpoints_y);
    y_mat = y_mat.reshape(1, 1);
    cv::Mat y_blurred;
    cv::GaussianBlur(y_mat, y_blurred, cv::Size(0, 0), 40);
    y_blurred = y_blurred.reshape(1, static_cast<int>(dpoints_y.size()));

    std::vector<float> dpoints_y_blurred;
    dpoints_y_blurred.reserve(dpoints_y.size());
    for (int i = 0; i < y_blurred.cols; ++i) {
        dpoints_y_blurred.push_back(y_blurred.at<float>(0, i));
    }

    for (size_t i = 0; i < dpoints_x.size() - 1; ++i) {
        cv::Point pt1(dpoints_x[i], static_cast<int>(dpoints_y_blurred[i]));
        cv::Point pt2(dpoints_x[i + 1],
                      static_cast<int>(dpoints_y_blurred[i + 1]));
        cv::line(output_image, pt1, pt2, cv::Scalar(255, 0, 0), 2);
    }

    cv::Mat ret_coord(static_cast<int>(dpoints_x.size()), 2, CV_32F);
    for (size_t i = 0; i < dpoints_x.size(); ++i) {
        ret_coord.at<float>(i, 0) = static_cast<float>(dpoints_x[i]);
        ret_coord.at<float>(i, 1) = dpoints_y_blurred[i];
    }

    result.image = output_image;
    result.coordinates = ret_coord;

    return result;
}

std::vector<Eigen::Vector3d> lines_3d(const std::vector<cv::Mat> &img_array,
                                      const int interval,
                                      const bool acq_interval = false) {
    std::vector<Eigen::Vector3d> pc_3d;
    int num_frames = interval;
    double increments = 499.0 / static_cast<double>(num_frames - 1);

    for (size_t i = 0; i < img_array.size(); ++i) {
        cv::Mat img = img_array[i];
        SegmentResult pc = detect_lines(img);
        if (pc.coordinates.empty()) {
            continue;
        }

        int idx = static_cast<int>(i) % interval;
        double z_val = idx * increments;

        for (int j = 0; j < pc.coordinates.rows; ++j) {
            float x = pc.coordinates.at<float>(j, 0);
            float y = pc.coordinates.at<float>(j, 1);
            pc_3d.emplace_back(Eigen::Vector3d(static_cast<double>(x),
                                               static_cast<double>(y), z_val));
        }

        if (acq_interval && pc_3d.size() >= static_cast<size_t>(interval)) {
            break;
        }
    }

    return pc_3d;
}

Eigen::Matrix3d align_to_direction(const Eigen::Matrix3d &rot_matrix) {
    Eigen::Matrix3d out_matrix = Eigen::Matrix3d::Zero();
    for (int col = 0; col < 3; ++col) {
        int max_idx;
        rot_matrix.col(col).cwiseAbs().maxCoeff(&max_idx);
        out_matrix.col(max_idx) = rot_matrix.col(col);
    }
    for (int col = 0; col < 3; ++col) {
        if (out_matrix(col, col) < 0) {
            out_matrix.col(col) *= -1.0;
        }
    }
    return out_matrix;
}

// std::vector<Eigen::Vector3d>
// interpolate_3d(const std::vector<std::vector<Eigen::Vector3d>> &data_frames,
//                int interval) {
//     int total_frames = 500;
//     int num_frames = data_frames.size();
//     int num_x_indices = 500;
//     std::vector<std::vector<float>> y_values(
//         num_frames, std::vector<float>(num_x_indices, 0.0f));
//     std::vector<float> z_key_values(num_frames, 0.0f);

//     for (int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
//         for (int x = 0; x < num_x_indices; ++x) {
//             y_values[frame_idx][x] =
//                 static_cast<float>(data_frames[frame_idx][x].y());
//         }
//         z_key_values[frame_idx] =
//             static_cast<float>(data_frames[frame_idx][0].z());
//     }

//     int num_segments = num_frames - 1;
//     int total_interpolated_frames = total_frames - num_frames;
//     int interpolated_frames_per_segment =
//         total_interpolated_frames / num_segments;
//     int remainder_frames = total_interpolated_frames % num_segments;

//     std::vector<std::vector<float>> interpolated_frames;
//     std::vector<std::vector<float>> all_frames;

//     for (int i = 0; i < num_segments; ++i) {
//         int frames_this_segment = interpolated_frames_per_segment;
//         if (i < remainder_frames) {
//             frames_this_segment += 1;
//         }

//         for (int p = 0; p < frames_this_segment; ++p) {
//             float t = static_cast<float>(p + 1) /
//                       static_cast<float>(frames_this_segment + 1);
//             std::vector<float> y_interp(num_x_indices, 0.0f);
//             for (int x = 0; x < num_x_indices; ++x) {
//                 y_interp[x] =
//                     (1.0f - t) * y_values[i][x] + t * y_values[i + 1][x];
//             }
//             interpolated_frames.push_back(y_interp);
//         }
//         std::vector<float> original_frame = y_values[i];
//         all_frames.emplace_back(original_frame);
//         // Append interpolated frames for this segment
//         for (int p = 0; p < frames_this_segment; ++p) {
//             all_frames.emplace_back(
//                 interpolated_frames[i * interpolated_frames_per_segment +
//                 p]);
//         }
//     }
//     all_frames.emplace_back(y_values[num_frames - 1]);
//     std::vector<Eigen::Vector3d> final_points;
//     int total_generated_frames = all_frames.size();
//     for (int i = 0; i < total_generated_frames; ++i) {
//         float z_val = static_cast<float>(std::round(
//             (499.0f / static_cast<float>(total_generated_frames - 1)) * i));
//         for (int x = 0; x < num_x_indices; ++x) {
//             final_points.emplace_back(Eigen::Vector3d(
//                 static_cast<double>(x),
//                 static_cast<double>(all_frames[i][x]),
//                 static_cast<double>(z_val)));
//         }
//     }
//     return final_points;
// }

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const int interval = 4;
    const bool single_interval = false;

    auto img_subscriber_node = std::make_shared<img_subscriber>();

    auto const logger = rclcpp::get_logger("img_processing");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(img_subscriber_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // // Run the executor in a separate thread
    // std::thread spinner([&executor]() { executor.spin(); });

    cv::Mat img;
    std::vector<cv::Mat> img_array;
    for (int i = 0; i < interval; i++) {
        while (true) {
            img = img_subscriber_node->get_img();
            if (!img.empty()) {
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        img_array.push_back(img);
        RCLCPP_INFO(logger, "Collected image %d", i + 1);
    }
    // executor.cancel();
    // spinner.join();

    std::vector<Eigen::Vector3d> pc_lines =
        lines_3d(img_array, interval, single_interval);
    open3d::geometry::PointCloud pcd_lines;
    for (const auto &point : pc_lines) {
        pcd_lines.points_.emplace_back(point);
    }
    auto boundbox = pcd_lines.GetMinimalOrientedBoundingBox(false);
    Eigen::Matrix3d rot_mat = align_to_direction(boundbox.R_);
    // RCLCPP_INFO(logger->get_logger(), "Aligned Rotation Matrix:\n" <<
    // rot_mat);
    std::cout << "Aligned Rotation Matrix:\n" << rot_mat << std::endl;

    rclcpp::shutdown();
    return 0;
}
