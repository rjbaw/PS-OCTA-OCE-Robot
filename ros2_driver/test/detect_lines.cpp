#include <algorithm>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <vector>

std::vector<cv::Point> get_max_coor(const cv::Mat &img) {
    std::vector<cv::Point> ret_coords;
    int height = img.rows;
    int width = img.cols;

    for (int x = 0; x < width; ++x) {
        cv::Mat intensity = img.col(x);
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(intensity, &minVal, &maxVal, &minLoc, &maxLoc);
        int detected_y = maxLoc.y;
        ret_coords.emplace_back(x, detected_y);
    }
    return ret_coords;
}

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord) {
    for (size_t i = 0; i < ret_coord.size() - 1; ++i) {
        cv::Point pt1 = ret_coord[i];
        cv::Point pt2 = ret_coord[i + 1];
        cv::line(image, pt1, pt2, cv::Scalar(255, 0, 0), 2);
    }
}

std::vector<cv::Point> detect_lines(const std::string &image_path,
                                    const std::string &save_dir) {
    cv::Mat img_raw = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (img_raw.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return {};
    }

    std::filesystem::path img_path(image_path);
    std::string base_name = img_path.stem().string();
    std::filesystem::path base_path =
        std::filesystem::path(save_dir) / base_name;

    cv::imwrite(base_path.string() + "_raw.jpg", img_raw);

    cv::Mat denoised_image;
    cv::medianBlur(img_raw, denoised_image, 5);

    cv::Mat img = denoised_image.clone();
    img.convertTo(img, CV_64F);

    cv::Mat row_means;
    cv::reduce(img, row_means, 1, cv::REDUCE_AVG);
    cv::Mat mean_mat;
    cv::repeat(row_means, 1, img.cols, mean_mat);
    img -= mean_mat;

    cv::Mat sobely;
    cv::Sobel(img, sobely, CV_64F, 0, 1, 9);
    img = sobely;

    std::vector<cv::Point> ret_coords = get_max_coor(img);

    std::vector<double> observations;
    for (const auto &pt : ret_coords) {
        observations.push_back(pt.y);
    }

    size_t obs_length = observations.size();
    int window = std::max(1, static_cast<int>(obs_length / 20));

    for (size_t i = 0; i < obs_length; ++i) {
        int start = std::max(0, static_cast<int>(i) - window);
        int end = std::min(obs_length, i + window + 1);
        std::vector<double> window_vals(observations.begin() + start,
                                        observations.begin() + end);

        double mu =
            std::accumulate(window_vals.begin(), window_vals.end(), 0.0) /
            window_vals.size();

        double accum = 0.0;
        for (double val : window_vals) {
            accum += (val - mu) * (val - mu);
        }
        double sigma = std::sqrt(accum / window_vals.size());

        std::vector<double> sorted_vals = window_vals;
        std::nth_element(sorted_vals.begin(),
                         sorted_vals.begin() + sorted_vals.size() / 2,
                         sorted_vals.end());
        double med = sorted_vals[sorted_vals.size() / 2];

        double z = (sigma != 0.0) ? (observations[i] - mu) / sigma
                                  : observations[i] - mu;

        if (z > 0.5) {
            observations[i] = med;
        }
    }

    double x_k = observations[0];
    double P_k = 1.0;
    double Q = 0.01;
    double R = 5.0;
    std::vector<double> x_k_estimates;

    for (double z_k : observations) {
        double x_k_pred = x_k;
        double P_k_pred = P_k + Q;
        double K_k = P_k_pred / (P_k_pred + R);
        x_k = x_k_pred + K_k * (z_k - x_k_pred);
        P_k = (1 - K_k) * P_k_pred;
        x_k_estimates.push_back(x_k);
    }

    for (size_t i = 0; i < ret_coords.size(); ++i) {
        ret_coords[i].y = static_cast<int>(x_k_estimates[i]);
    }

    cv::Mat detected_image = img_raw.clone();
    draw_line(detected_image, ret_coords);
    cv::imwrite(base_path.string() + "_detected.jpg", detected_image);

    return ret_coords;
}

int main() {
    std::string image_path = "test.jpg";
    std::string save_dir = "result";

    std::vector<cv::Point> detected_coords = detect_lines(image_path, save_dir);

    for (const auto &pt : detected_coords) {
        std::cout << "(" << pt.x << ", " << pt.y << ")\n";
    }

    return 0;
}
