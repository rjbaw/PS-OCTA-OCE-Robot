#include "process_img.hpp"

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord) {
    for (size_t i = 0; i < ret_coord.size() - 1; ++i) {
        cv::Point pt1 = ret_coord[i];
        cv::Point pt2 = ret_coord[i + 1];
        cv::line(image, pt1, pt2, cv::Scalar(255, 0, 0), 2);
    }
}

std::vector<double> kalmanFilter1D(const std::vector<double> &observations,
                                   double Q, double R) {
    std::vector<double> x_k_estimates;
    x_k_estimates.reserve(observations.size());
    double x_k = (observations.empty()) ? 0.0 : observations[0];
    double P_k = 1.0;
    for (double z_k : observations) {
        double x_k_pred = x_k;
        double P_k_pred = P_k + Q;
        double K_k = P_k_pred / (P_k_pred + R);
        x_k = x_k_pred + K_k * (z_k - x_k_pred);
        P_k = (1.0 - K_k) * P_k_pred;
        x_k_estimates.push_back(x_k);
    }
    return x_k_estimates;
}

void removeOutliers(std::vector<double> &vals, double z_threshold) {
    if (vals.empty())
        return;
    size_t obs_length = vals.size();
    int window = std::max(1, (int)obs_length / 10);
    for (size_t i = 0; i < obs_length; ++i) {
        int start = std::max(0, (int)i - window);
        int end = std::min((int)obs_length, (int)i + window + 1);
        std::vector<double> local(vals.begin() + start, vals.begin() + end);
        double mu =
            std::accumulate(local.begin(), local.end(), 0.0) / local.size();
        double accum = 0.0;
        for (double val : local) {
            accum += (val - mu) * (val - mu);
        }
        double sigma = std::sqrt(accum / local.size());
        std::nth_element(local.begin(), local.begin() + local.size() / 2,
                         local.end());
        double med = local[local.size() / 2];
        double z = (sigma != 0.0) ? (vals[i] - mu) / sigma : 0.0;
        if (z > z_threshold) {
            vals[i] = med;
        }
    }
}

std::vector<cv::Point> get_max_coor(const cv::Mat &img) {
    std::vector<cv::Point> ret_coords;
    // int height = img.rows;
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

SegmentResult detect_lines(const cv::Mat &inputImg) {
    CV_Assert(!inputImg.empty());
    cv::Mat gray;
    if (inputImg.channels() == 3) {
        cv::cvtColor(inputImg, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = inputImg.clone();
    }

    cv::Mat denoised;
    cv::Mat denoised_f;
    cv::medianBlur(gray, denoised, 5);
    denoised.convertTo(denoised_f, CV_32F);

    cv::Mat row_means;
    cv::reduce(denoised_f, row_means, 1, cv::REDUCE_AVG, CV_32F);
    cv::Mat mean_mat;
    cv::repeat(row_means, 1, denoised_f.cols, mean_mat);
    denoised_f -= mean_mat;

    cv::Mat img_median1d;
    denoised_f.convertTo(img_median1d, CV_8U, 1.0, 128.0);
    cv::medianBlur(img_median1d, img_median1d, 3);
    img_median1d.convertTo(img_median1d, CV_32F, 1.0, -128.0);

    cv::Mat img_gauss;
    cv::GaussianBlur(img_median1d, img_gauss, cv::Size(0, 0), 3.0, 3.0);

    cv::Mat sobely;
    cv::Sobel(img_gauss, sobely, CV_32F, 0, 1, 3);

    std::vector<cv::Point> rawCoords = get_max_coor(sobely);

    std::vector<double> observations;
    observations.reserve(rawCoords.size());
    for (const auto &pt : rawCoords) {
        observations.push_back(static_cast<double>(pt.y));
    }

    removeOutliers(observations, 0.5);
    std::vector<double> x_est = kalmanFilter1D(observations, 0.01, 0.5);
    std::vector<cv::Point> ret_coords(rawCoords.size());
    for (size_t i = 0; i < x_est.size(); ++i) {
        ret_coords[i] = cv::Point(static_cast<int>(i),
                                  static_cast<int>(std::round(x_est[i])));
    }

    cv::Mat detected_img;
    gray.convertTo(detected_img, CV_8U);
    draw_line(detected_img, ret_coords);

    SegmentResult result;
    result.image = detected_img;
    result.coordinates = ret_coords;
    return result;
}

std::vector<Eigen::Vector3d> lines_3d(const std::vector<cv::Mat> &img_array,
                                      const int interval,
                                      const bool acq_interval) {
    std::vector<Eigen::Vector3d> pc_3d;
    int num_frames = interval > 1 ? interval : 2;
    double increments = 499.0 / static_cast<double>(num_frames - 1);

    for (size_t i = 0; i < img_array.size(); ++i) {
        cv::Mat img = img_array[i];
        cv::imwrite(std::format("raw_image{}.jpg", i).c_str(), img);
        SegmentResult pc = detect_lines(img);
        cv::imwrite(std::format("detected_image{}.jpg", i).c_str(), pc.image);
        assert(!pc.coordinates.empty());

        int idx = static_cast<int>(i) % interval;
        double z_val = idx * increments;

        for (size_t j = 0; j < pc.coordinates.size(); ++j) {
            double x = static_cast<double>(pc.coordinates[j].x);
            double y = static_cast<double>(pc.coordinates[j].y);
            pc_3d.emplace_back(Eigen::Vector3d(x, y, z_val));
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
