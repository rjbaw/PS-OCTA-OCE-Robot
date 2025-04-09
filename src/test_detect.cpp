#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

struct SegmentResult {
    cv::Mat image;
    std::vector<cv::Point> coordinates;
};

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord) {
    for (size_t i = 0; i + 1 < ret_coord.size(); ++i) {
        cv::line(image, ret_coord[i], ret_coord[i + 1], cv::Scalar(255, 0, 0),
                 2);
    }
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

static void medianFilter1D(std::vector<double> &signal, int window_size) {
    int half_win = window_size / 2;
    std::vector<double> extended;
    extended.reserve(signal.size() + 2 * half_win);

    for (int i = 0; i < half_win; ++i)
        extended.push_back(signal.front());
    for (double val : signal)
        extended.push_back(val);
    for (int i = 0; i < half_win; ++i)
        extended.push_back(signal.back());

    for (int i = 0; i < (int)signal.size(); ++i) {
        std::vector<double> window(extended.begin() + i,
                                   extended.begin() + i + window_size);
        std::nth_element(window.begin(), window.begin() + window_size / 2,
                         window.end());
        double med = window[window_size / 2];
        signal[i] = med;
    }
}

cv::Mat getPSD() {
    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("octa_ros");
    std::string psd_path = pkg_share + "/config/psd.yml";

    cv::FileStorage fs(psd_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open PSD: " << psd_path << std::endl;
        return cv::Mat();
    }
    cv::Mat psd;
    fs["psd"] >> psd;
    return psd;
}

cv::Mat wienerFilter(const cv::Mat &input) {

    cv::Mat floatImg;
    input.convertTo(floatImg, CV_32F);

    cv::Mat dft_complex;
    cv::dft(floatImg, dft_complex, cv::DFT_COMPLEX_OUTPUT);

    cv::Mat S_nn = getPSD();
    if (S_nn.empty()) {
        std::cerr << "[wienerFilter] PSD is empty. Returning input.\n";
        return input.clone();
    }

    std::vector<cv::Mat> channels(2);
    cv::split(dft_complex, channels);

    cv::Mat mag;
    cv::magnitude(channels[0], channels[1], mag);
    cv::Mat mag2 = mag.mul(mag);

    cv::Mat S_xx = mag2 - S_nn;
    cv::max(S_xx, 0.0f, S_xx);

    float eps = 1e-8f;
    cv::Mat denom = S_xx + S_nn + eps;
    cv::Mat H;
    cv::divide(S_xx, denom, H);

    channels[0] = channels[0].mul(H);
    channels[1] = channels[1].mul(H);

    cv::Mat dft_filt;
    cv::merge(channels, dft_filt);

    cv::Mat out_float;
    cv::dft(dft_filt, out_float,
            cv::DFT_REAL_OUTPUT | cv::DFT_SCALE | cv::DFT_INVERSE);

    double minVal, maxVal;
    cv::minMaxLoc(out_float, &minVal, &maxVal);
    cv::Mat out_8u;
    double eps_d = 1e-8;
    out_float.convertTo(out_8u, CV_8U, 255.0 / (maxVal - minVal + eps_d),
                        -minVal * 255.0 / (maxVal - minVal + eps_d));

    return out_8u;
}

void zero_dc(cv::Mat &img, const std::vector<int> &zidx, int window) {
    img.convertTo(img, CV_32F);

    int rows = img.rows;
    int cols = img.cols;

    for (int center : zidx) {
        int half = window / 2;
        int start_idx = std::max(center - half, 0);
        int end_idx = std::min(center + half, rows);

        for (int r = start_idx; r < end_idx; ++r) {
            double sumRow = 0.0;
            for (int c = 0; c < cols; ++c) {
                sumRow += img.at<float>(r, c);
            }
            double meanRow = sumRow / cols;
            for (int c = 0; c < cols; ++c) {
                float val = img.at<float>(r, c);
                img.at<float>(r, c) = val - static_cast<float>(meanRow);
            }
        }
    }

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            float val = img.at<float>(r, c);
            val = std::max(0.0f, std::min(val, 255.0f));
            img.at<float>(r, c) = val;
        }
    }
    img.convertTo(img, CV_8U);
}

cv::Mat spatialFilter(cv::Mat input) {
    std::vector<int> zidx = {0, 5, 12, 24, 37, 51, 63, 75, 87, 99, 112, 126};
    zero_dc(input, zidx, 14);
    return wienerFilter(input);
}

std::vector<cv::Point> get_max_coor(const cv::Mat &img) {
    std::vector<cv::Point> ret_coords;
    ret_coords.reserve(img.cols);

    for (int x = 0; x < img.cols; ++x) {
        cv::Mat col = img.col(x);
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(col, &minVal, &maxVal, &minLoc, &maxLoc);
        ret_coords.emplace_back(x, maxLoc.y);
    }
    return ret_coords;
}

std::vector<double> kalmanFilter1D(const std::vector<double> &observations,
                                   double Q = 0.01, double R = 0.5) {
    std::vector<double> x_k_estimates;
    x_k_estimates.reserve(observations.size());
    if (observations.empty()) {
        return x_k_estimates;
    }

    int initCount = std::min((int)observations.size(), 10);
    double x0 = 0.0;
    if (initCount > 0) {
        std::vector<double> initSeg(observations.begin(),
                                    observations.begin() + initCount);
        std::nth_element(initSeg.begin(), initSeg.begin() + initSeg.size() / 2,
                         initSeg.end());
        x0 = initSeg[initSeg.size() / 2];
    }

    double x_k = x0;
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

std::vector<cv::Point> ol_removal(const std::vector<cv::Point> &coords) {
    if (coords.empty()) {
        return {};
    }

    std::vector<double> observations;
    observations.reserve(coords.size());
    for (auto &pt : coords) {
        observations.push_back(pt.y);
    }

    int obs_length = (int)observations.size();
    int window = std::max(1, obs_length / 3);
    double z_max = 30.0;

    double best_slope = 0.0;
    double best_sigma = std::numeric_limits<double>::infinity();

    int segmentCount = obs_length / 3;
    for (int w = 0; w < segmentCount; ++w) {
        int start = w * window;
        int end = std::min(start + window, obs_length - 1);
        if (end <= start) {
            continue;
        }

        std::vector<double> segment(observations.begin() + start,
                                    observations.begin() + end);

        double meanVal = std::accumulate(segment.begin(), segment.end(), 0.0) /
                         segment.size();
        double accum = 0.0;
        for (double val : segment) {
            accum += (val - meanVal) * (val - meanVal);
        }
        double stdv = std::sqrt(accum / segment.size());

        medianFilter1D(segment, window);
        double slopeCandidate =
            (segment.back() - segment.front()) / (double)segment.size();

        if (stdv < best_sigma && std::fabs(slopeCandidate) < z_max) {
            best_sigma = stdv;
            best_slope = slopeCandidate;
        }
    }

    for (int i = 0; i < obs_length; ++i) {
        if (i == 0) {
            int end = std::min(20, obs_length);
            std::vector<double> firstChunk(observations.begin(),
                                           observations.begin() + end);
            std::nth_element(firstChunk.begin(),
                             firstChunk.begin() + firstChunk.size() / 2,
                             firstChunk.end());
            observations[0] = firstChunk[firstChunk.size() / 2];
        } else {
            double prev_pt = observations[i - 1];
            double pt = observations[i];
            double mse = std::fabs(pt - prev_pt);
            if (mse > z_max) {
                observations[i] = prev_pt + best_slope;
            }
        }
    }

    std::vector<cv::Point> new_coords;
    new_coords.reserve(coords.size());
    for (int i = 0; i < obs_length; ++i) {
        new_coords.push_back(
            cv::Point(coords[i].x, (int)std::round(observations[i])));
    }
    return new_coords;
}

SegmentResult detect_lines(const cv::Mat &inputImg) {
    CV_Assert(!inputImg.empty());

    cv::Mat img_raw;
    if (inputImg.channels() == 3) {
        cv::cvtColor(inputImg, img_raw, cv::COLOR_BGR2GRAY);
    } else {
        img_raw = inputImg.clone();
    }

    cv::Mat denoised_image = spatialFilter(img_raw.clone());

    std::vector<cv::Point> ret_coords = get_max_coor(denoised_image);

    ret_coords = ol_removal(ret_coords);

    std::vector<double> obs;
    obs.reserve(ret_coords.size());
    for (auto &pt : ret_coords) {
        obs.push_back(pt.y);
    }
    std::vector<double> kf_out = kalmanFilter1D(obs, 0.01, 0.5);
    for (size_t i = 0; i < ret_coords.size(); ++i) {
        ret_coords[i].y = static_cast<int>(std::round(kf_out[i]));
    }

    cv::Mat detected_img = img_raw.clone();
    draw_line(detected_img, ret_coords);

    SegmentResult result;
    result.image = detected_img;
    result.coordinates = ret_coords;
    return result;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.jpg> [output.jpg]"
                  << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = (argc > 2) ? argv[2] : "result.jpg";

    cv::Mat inputImage = cv::imread(inputFile, cv::IMREAD_COLOR);
    if (inputImage.empty()) {
        std::cerr << "Cannot open " << inputFile << std::endl;
        return 1;
    }

    SegmentResult result = detect_lines(inputImage);

    if (!cv::imwrite(outputFile, result.image)) {
        std::cerr << "Could not save result to " << outputFile << std::endl;
        return 1;
    }
    std::cout << "Saved result to " << outputFile << std::endl;

    // for (auto &pt : result.coordinates) {
    //     std::cout << "(" << pt.x << ", " << pt.y << ")\n";
    // }
    return 0;
}
