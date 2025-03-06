#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <cassert>

struct SegmentResult {
    cv::Mat image;
    std::vector<cv::Point> coordinates;
};

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord) {
    for (size_t i = 0; i + 1 < ret_coord.size(); ++i) {
        cv::line(image, ret_coord[i], ret_coord[i + 1], cv::Scalar(0,0,255), 2);
    }
}

std::vector<cv::Point> get_max_coor(const cv::Mat &img) {
    std::vector<cv::Point> ret_coords;
    int width = img.cols;
    for (int x = 0; x < width; ++x) {
        cv::Mat intensity = img.col(x);
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(intensity, &minVal, &maxVal, &minLoc, &maxLoc);
        ret_coords.emplace_back(x, maxLoc.y);
    }
    return ret_coords;
}

std::vector<double> kalmanFilter1D(const std::vector<double> &observations,
                                   double Q = 0.01, double R = 0.5) {
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

void removeOutliers(std::vector<double> &vals, double z_threshold = 0.5) {
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

void shiftDFT(cv::Mat &fImage) {
    std::vector<cv::Mat> planes;
    cv::split(fImage, planes);

    for (auto &plane : planes) {
        plane = plane(cv::Rect(0, 0, plane.cols & -2, plane.rows & -2));

        int cx = plane.cols / 2;
        int cy = plane.rows / 2;

        cv::Mat q0(plane, cv::Rect(0, 0, cx, cy));
        cv::Mat q1(plane, cv::Rect(cx, 0, cx, cy));
        cv::Mat q2(plane, cv::Rect(0, cy, cx, cy));
        cv::Mat q3(plane, cv::Rect(cx, cy, cx, cy));

        cv::Mat tmp;

        q0.copyTo(tmp);
        q3.copyTo(q0);
        tmp.copyTo(q3);

        q1.copyTo(tmp);
        q2.copyTo(q1);
        tmp.copyTo(q2);
    }
    cv::merge(planes, fImage);
}

//fft
// SegmentResult detect_lines(const cv::Mat &img) {
//     SegmentResult result;

//     cv::Mat denoised_image;
//     cv::medianBlur(img, denoised_image, 5);

//     cv::Mat padded;
//     int m = cv::getOptimalDFTSize(denoised_image.rows);
//     int n = cv::getOptimalDFTSize(denoised_image.cols);
//     cv::copyMakeBorder(denoised_image, padded, 0, m - denoised_image.rows, 0,
//                        n - denoised_image.cols, cv::BORDER_CONSTANT,
//                        cv::Scalar::all(0));

//     padded.convertTo(padded, CV_32F);

//     cv::Mat planes[] = {padded, cv::Mat::zeros(padded.size(), CV_32F)};
//     cv::Mat complexI;
//     cv::merge(planes, 2, complexI);

//     cv::dft(complexI, complexI);

//     shiftDFT(complexI);

//     int rows = complexI.rows;
//     int cols = complexI.cols;
//     int crow = rows / 2;
//     int ccol = cols / 2;
//     cv::Mat mask = cv::Mat::zeros(rows, cols, CV_32F);
//     float sigma = 40.0f;

//     for (int i = 0; i < rows; i++) {
//         float *mask_row = mask.ptr<float>(i);
//         for (int j = 0; j < cols; j++) {
//             float val =
//                 std::exp(-((i - crow) * (i - crow) + (j - ccol) * (j - ccol))
//                 /
//                          (2 * sigma * sigma));
//             mask_row[j] = val;
//         }
//     }

//     cv::split(complexI, planes);
//     planes[0] = planes[0].mul(mask);
//     planes[1] = planes[1].mul(mask);
//     cv::merge(planes, 2, complexI);

//     shiftDFT(complexI);

//     cv::Mat inverseTransform;
//     cv::idft(complexI, inverseTransform, cv::DFT_REAL_OUTPUT |
//     cv::DFT_SCALE);

//     inverseTransform = inverseTransform(cv::Rect(0, 0, img.cols, img.rows));

//     cv::Mat processed_img = inverseTransform.clone();
//     processed_img.convertTo(processed_img, CV_64F);

//     cv::Mat row_means;
//     cv::reduce(processed_img, row_means, 1, cv::REDUCE_AVG, CV_64F);
//     cv::Mat mean_mat;
//     cv::repeat(row_means, 1, processed_img.cols, mean_mat);
//     processed_img -= mean_mat;

//     cv::normalize(processed_img, processed_img, 0, 255, cv::NORM_MINMAX);
//     processed_img.convertTo(processed_img, CV_8U);

//     cv::Mat sobely;
//     cv::Sobel(processed_img, sobely, CV_64F, 0, 1, 9);

//     std::vector<cv::Point> ret_coords = get_max_coor(sobely);

//     std::vector<double> observations;
//     for (const auto &pt : ret_coords) {
//         observations.push_back(pt.y);
//     }

//     size_t obs_length = observations.size();
//     int window = std::max(1, static_cast<int>(obs_length / 20));

//     for (size_t i = 0; i < obs_length; ++i) {
//         int start = std::max(0, static_cast<int>(i) - window);
//         int end = std::min(static_cast<int>(obs_length),
//                            static_cast<int>(i) + window + 1);
//         std::vector<double> window_vals(observations.begin() + start,
//                                         observations.begin() + end);

//         double mu =
//             std::accumulate(window_vals.begin(), window_vals.end(), 0.0) /
//             window_vals.size();

//         double accum = 0.0;
//         for (double val : window_vals) {
//             accum += (val - mu) * (val - mu);
//         }
//         double sigma = std::sqrt(accum / window_vals.size());

//         std::vector<double> sorted_vals = window_vals;
//         std::nth_element(sorted_vals.begin(),
//                          sorted_vals.begin() + sorted_vals.size() / 2,
//                          sorted_vals.end());
//         double med = sorted_vals[sorted_vals.size() / 2];

//         double z = (sigma != 0.0) ? (observations[i] - mu) / sigma
//                                   : observations[i] - mu;

//         if (z > 0.5) {
//             observations[i] = med;
//         }
//     }

//     double x_k = observations[0];
//     double P_k = 1.0;
//     double Q = 0.01;
//     double R = 5.0;
//     std::vector<double> x_k_estimates;

//     for (double z_k : observations) {
//         double x_k_pred = x_k;
//         double P_k_pred = P_k + Q;
//         double K_k = P_k_pred / (P_k_pred + R);
//         x_k = x_k_pred + K_k * (z_k - x_k_pred);
//         P_k = (1 - K_k) * P_k_pred;
//         x_k_estimates.push_back(x_k);
//     }

//     for (size_t i = 0; i < ret_coords.size(); ++i) {
//         ret_coords[i].y = static_cast<int>(x_k_estimates[i]);
//     }

//     cv::Mat detected_image = img.clone();
//     draw_line(detected_image, ret_coords);

//     result.image = detected_image;
//     result.coordinates = ret_coords;

//     return result;
// }

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

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.jpg> [output.jpg]" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    cv::Mat inputImage = cv::imread(inputFile, cv::IMREAD_GRAYSCALE);
    if (inputImage.empty()) {
        std::cerr << "Cannot open " << inputFile << std::endl;
        return 1;
    }

    SegmentResult result = detect_lines(inputImage);

    std::string outputFile = (argc > 2) ? argv[2] : "result.jpg";
    if (!cv::imwrite(outputFile, result.image)) {
        std::cerr << "Could not save result to " << outputFile << std::endl;
        return 1;
    }
    std::cout << "Saved result to " << outputFile << std::endl;
    return 0;
}
