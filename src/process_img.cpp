#include "process_img.hpp"

double median(std::vector<double> &vals) {
    size_t n = vals.size();
    if (n == 0)
        return 0.0;
    std::nth_element(vals.begin(), vals.begin() + n / 2, vals.end());
    return vals[n / 2];
}

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord) {
    for (size_t i = 0; i < ret_coord.size() - 1; ++i) {
        cv::Point pt1 = ret_coord[i];
        cv::Point pt2 = ret_coord[i + 1];
        cv::line(image, pt1, pt2, cv::Scalar(255, 0, 0), 2);
    }
}

std::vector<double> kalmanFilter1D(const std::vector<double> &observations,
                                   double Q, double R, double x0) {
    std::vector<double> x_k_estimates;
    x_k_estimates.reserve(observations.size());
    if (observations.empty()) {
        return x_k_estimates;
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

void computeMeanStd(const std::vector<double> &data, int start, int end,
                    double &mean, double &stddev) {
    if (end <= start) {
        mean = 0.0;
        stddev = 0.0;
        return;
    }

    double sum = 0.0;
    for (int i = start; i < end; ++i) {
        sum += data[i];
    }
    mean = sum / (end - start);

    double var = 0.0;
    for (int i = start; i < end; ++i) {
        double diff = data[i] - mean;
        var += diff * diff;
    }
    var /= (end - start);
    stddev = std::sqrt(var);
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
            if (val < 0.0f) {
                val = 0.0f;
            }
            if (val > 255.0f) {
                val = 255.0f;
            }
            img.at<float>(r, c) = val;
        }
    }
    img.convertTo(img, CV_8U);
}

void removeOutliers(std::vector<double> &vals, double z_threshold,
                    int window_fraction) {
    if (vals.empty())
        return;
    size_t obs_length = vals.size();
    int window = std::max(1, static_cast<int>(obs_length) / window_fraction);

    for (size_t i = 0; i < obs_length; ++i) {
        int start = std::max(0, static_cast<int>(i) - window);
        int end = std::min(static_cast<int>(obs_length),
                           static_cast<int>(i) + window + 1);
        double mu, sigma;
        computeMeanStd(vals, start, end, mu, sigma);

        std::vector<double> local(vals.begin() + start, vals.begin() + end);
        double med = median(local);

        double diff = vals[i] - mu;
        double z = (sigma != 0.0) ? std::abs(diff / sigma) : std::abs(diff);
        if (z > z_threshold) {
            vals[i] = med;
        }
    }
}

void linearizeOutliers(std::vector<double> &observations, double z_threshold,
                       int window_fraction) {
    int obs_length = (int)observations.size();
    if (obs_length < 2)
        return;

    int window = std::max(1, obs_length / window_fraction);

    for (int i = 0; i < obs_length; ++i) {
        int start = std::max(0, i - window);
        int end = i - 1;
        if (end <= start)
            continue;

        double mu, stdv;
        computeMeanStd(observations, start, end, mu, stdv);

        double diff = observations[i] - mu;
        double z = (stdv != 0.0) ? std::abs(diff / stdv) : std::abs(diff);
        if (z > z_threshold) {
            bool found = false;
            int jmax = window;
            for (int j = 0; j < jmax; ++j) {
                int idx = i + j;
                if (idx >= obs_length)
                    break;

                double diff2 = observations[idx] - mu;
                double z2 =
                    (stdv != 0.0) ? std::abs(diff2 / stdv) : std::abs(diff2);
                if (z2 < z_threshold) {
                    found = true;
                    int prev_idx = std::max(0, i - 1);
                    double dy = (observations[idx] - observations[prev_idx]) /
                                double(j + 1);

                    for (int k = 0; k <= j; ++k) {
                        int cur_i = prev_idx + 1 + k;
                        if (cur_i < obs_length) {
                            observations[cur_i] =
                                observations[prev_idx] + k * dy;
                        }
                    }
                    break;
                }
            }
            if (!found) {
                int last_good_idx = std::max(0, i - 1);
                double last_good_val = observations[last_good_idx];
                int limit = std::min(obs_length, i + window + 1);
                for (int m = i; m < limit; ++m) {
                    observations[m] = last_good_val;
                }
            }
        }
    }
}

std::vector<cv::Point> get_max_coor(const cv::Mat &img) {
    int width = img.cols;
    std::vector<cv::Point> ret_coords(width);

    for (int x = 0; x < width; ++x) {
        cv::Mat intensity = img.col(x);
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(intensity, &minVal, &maxVal, &minLoc, &maxLoc);
        int detected_y = maxLoc.y;
        ret_coords[x] = cv::Point(x, detected_y);
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
    int height = gray.rows;

    cv::Mat sobely;
    cv::Sobel(gray, sobely, CV_8U, 0, 1, 3);

    cv::Mat denoised;
    cv::medianBlur(sobely, denoised, 3);

    cv::Mat img_gauss;
    cv::GaussianBlur(denoised, img_gauss, cv::Size(3, 3), 0);

    std::vector<int> zidx = {0, 5, 12, 24, 37, 51, 63, 75, 87, 99, 112, 126};
    zero_dc(img_gauss, zidx, 14);

    std::vector<cv::Point> rawCoords = get_max_coor(img_gauss);

    std::vector<double> observations;
    observations.reserve(rawCoords.size());
    for (const auto &pt : rawCoords) {
        observations.push_back(static_cast<double>(pt.y));
    }

    removeOutliers(observations, 0.5, 20);
    linearizeOutliers(observations, 18.0, 5);

    double x0 = 0.0;
    if (!observations.empty()) {
        int n_init = std::min<int>(10, (int)observations.size());
        std::vector<double> firstN(observations.begin(),
                                   observations.begin() + n_init);
        x0 = median(firstN);
    }
    std::vector<double> x_est = kalmanFilter1D(observations, 0.01, 0.5, x0);

    std::vector<cv::Point> img_coords(rawCoords.size());
    std::vector<cv::Point> ret_coords(rawCoords.size());
    for (size_t i = 0; i < x_est.size(); ++i) {
        int x = static_cast<int>(i);
        int y = static_cast<int>(x_est[i]);
        img_coords[i] = cv::Point(x, y);
        ret_coords[i] = cv::Point(x, height - y);
    }

    cv::Mat detected_img = gray.clone();
    draw_line(detected_img, img_coords);
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
        std::string raw_filename = std::format("raw_image{}.jpg", i);
        cv::imwrite(raw_filename.c_str(), img);
        SegmentResult pc = detect_lines(img);
        std::string processed_filename = std::format("detected_image{}.jpg", i);
        cv::imwrite(processed_filename.c_str(), pc.image);
        assert(!pc.coordinates.empty());

        int idx = static_cast<int>(i) % interval;
        double z_val = idx * increments;

        for (size_t j = 0; j < pc.coordinates.size(); ++j) {
            double x = static_cast<double>(pc.coordinates[j].x);
            double y = static_cast<double>(pc.coordinates[j].y);
            pc_3d.emplace_back(Eigen::Vector3d(x, z_val, y));
            //pc_3d.emplace_back(Eigen::Vector3d(-y, z_val, x));
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
