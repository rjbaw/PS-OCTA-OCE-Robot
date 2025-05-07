#include "process_img.hpp"

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

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord) {
    for (size_t i = 0; i < ret_coord.size() - 1; ++i) {
        cv::Point pt1 = ret_coord[i];
        cv::Point pt2 = ret_coord[i + 1];
        cv::line(image, pt1, pt2, cv::Scalar(255, 255, 255), 2);
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

cv::Mat gradient(const cv::Mat &img) {
    CV_Assert(img.channels() == 1);

    // img_f = img.astype(np.float32);
    // kx = np.array([ [ 0, 0, 0 ], [ -0.5, 0, 0.5 ], [ 0, 0, 0 ] ],
    // np.float32); ky = kx.T; cv::Mat gy; cv::Mat gx; cv::filter2D(img_f, gy,
    // -1, ky, borderType = cv::BORDER_REPLICATE); cv::filter2D(img_f, gx, -1,
    // kx, borderType = cv::BORDER_REPLICATE); return np.sqrt(0.65 * gy * *2 +
    // gx * *2, dtype = np.float32);

    cv::Mat img_f;
    img.convertTo(img_f, CV_32F);

    float kxVals[9] = {0.f, 0.f, 0.f, -0.5f, 0.f, 0.5f, 0.f, 0.f, 0.f};
    cv::Mat kx(3, 3, CV_32F, kxVals);

    cv::Mat ky = kx.t();

    cv::Mat gx, gy;
    cv::filter2D(img_f, gx, -1, kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(img_f, gy, -1, ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

    cv::Mat gx2, gy2, mag;
    cv::multiply(gx, gx, gx2);
    cv::multiply(gy, gy, gy2);

    gy2 *= 0.65f;
    cv::add(gx2, gy2, mag);
    cv::sqrt(mag, mag);

    return mag;
}

cv::Mat build_gaussian_filter(int nx, int ny) {
    // def build_gaussian_filter(nx, ny):
    //     x = np.arange(nx, dtype=np.float32) - (nx - 1) / 2
    //     y = np.arange(ny, dtype=np.float32) - (ny - 1) / 2
    //     xx, yy = np.meshgrid(x, y)
    //     k = np.exp(-(xx**2) / (nx / 4) ** 2) * np.exp(-(yy**2) / (ny / 4) **
    //     2) k /= k.sum() return k

    cv::Mat kernel(ny, nx, CV_32F);

    float cx = (nx - 1) / 2.0f;
    float cy = (ny - 1) / 2.0f;
    float sigmaX = nx / 4.0f;
    float sigmaY = ny / 4.0f;

    double sumVal = 0.0;
    for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
            float x = static_cast<float>(i) - cx;
            float y = static_cast<float>(j) - cy;
            float val = std::exp(-(x * x) / (sigmaX * sigmaX)) *
                        std::exp(-(y * y) / (sigmaY * sigmaY));
            kernel.at<float>(j, i) = val;
            sumVal += val;
        }
    }

    kernel /= sumVal;

    return kernel;
}

cv::Mat lowpass(const cv::Mat &img, int nx, int ny) {
    cv::Mat kernel = build_gaussian_filter(nx, ny);

    cv::Mat dst;
    cv::filter2D(img, dst, -1, kernel, cv::Point(-1, -1), 0,
                 cv::BORDER_REPLICATE);

    return dst;
}

cv::Mat load_bg() {
    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("octa_ros");
    std::string bg_path = pkg_share + "/config/bg.jpg";
    cv::Mat bg = cv::imread(bg_path, cv::IMREAD_GRAYSCALE);
    if (bg.empty()) {
        std::cerr << "Failed to load background image: " << bg_path
                  << std::endl;
    }
    return bg;
}

cv::Mat bg_sub(const cv::Mat &input) {
    cv::Mat bg = load_bg();
    CV_Assert(input.size() == bg.size() && input.type() == bg.type());

    cv::Mat input_f;
    cv::Mat bg_f;
    input.convertTo(input_f, CV_32F);
    bg.convertTo(bg_f, CV_32F);

    cv::Mat sub_f = input_f - bg_f;
    cv::Mat output;
    cv::normalize(sub_f, output, 0, 255, cv::NORM_MINMAX, CV_8U);

    return output;
}

// cv::Mat wienerFilter(const cv::Mat &input) {

//     cv::Mat floatImg;
//     input.convertTo(floatImg, CV_32F);

//     cv::Mat dft_complex;
//     cv::dft(floatImg, dft_complex, cv::DFT_COMPLEX_OUTPUT);

//     cv::Mat S_nn = getPSD();
//     if (S_nn.empty()) {
//         std::cerr << "[wienerFilter] PSD is empty. Returning input.\n";
//         return input.clone();
//     }

//     std::vector<cv::Mat> channels(2);
//     cv::split(dft_complex, channels);

//     cv::Mat mag;
//     cv::magnitude(channels[0], channels[1], mag);
//     cv::Mat mag2 = mag.mul(mag);

//     cv::Mat S_xx = mag2 - S_nn;
//     cv::max(S_xx, 0.0f, S_xx);

//     float eps = 1e-8f;
//     cv::Mat denom = S_xx + S_nn + eps;
//     cv::Mat H;
//     cv::divide(S_xx, denom, H);

//     channels[0] = channels[0].mul(H);
//     channels[1] = channels[1].mul(H);

//     cv::Mat dft_filt;
//     cv::merge(channels, dft_filt);

//     cv::Mat out_float;
//     cv::dft(dft_filt, out_float,
//             cv::DFT_REAL_OUTPUT | cv::DFT_SCALE | cv::DFT_INVERSE);

//     double minVal, maxVal;
//     cv::minMaxLoc(out_float, &minVal, &maxVal);
//     cv::Mat out_8u;
//     double eps_d = 1e-8;
//     out_float.convertTo(out_8u, CV_8U, 255.0 / (maxVal - minVal + eps_d),
//                         -minVal * 255.0 / (maxVal - minVal + eps_d));

//     return out_8u;
// }

cv::Mat spatialFilter(cv::Mat &input) {

    cv::Mat raw;
    input.convertTo(raw, CV_32F);
    cv::Mat lp = lowpass(raw, 11, 5);
    cv::Mat grad = gradient(lp);
    cv::Mat grad_lp = lowpass(grad, 1, 3);
    cv::Mat output = lp.mul(grad_lp);
    cv::normalize(output, output, 0, 255, cv::NORM_MINMAX, CV_8U);

    return output;
}

double median(const std::vector<double> &values, int N) {
    int initCount = std::min(static_cast<int>(values.size()), N);
    if (initCount == 0) {
        return 0.0;
    }

    std::vector<double> subset(values.begin(), values.begin() + initCount);
    std::sort(subset.begin(), subset.end());

    if (initCount % 2 == 1) {
        return subset[initCount / 2];
    } else {
        double lower = subset[(initCount / 2) - 1];
        double upper = subset[initCount / 2];
        return 0.5 * (lower + upper);
    }
}

std::vector<double> kalmanFilter1D(const std::vector<double> &observations,
                                   double Q = 0.01, double R = 0.5) {
    std::vector<double> x_k_estimates;
    x_k_estimates.reserve(observations.size());
    if (observations.empty()) {
        return x_k_estimates;
    }

    double x0 = median(observations, 10);
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
    double z_max = 40.0;

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

    cv::Mat sub_image = bg_sub(img_raw);
    cv::Mat denoised_image = spatialFilter(sub_image);

    std::vector<cv::Point> ret_coords = get_max_coor(denoised_image);

    // std::vector<double> ys;
    // ys.reserve(ret_coords.size());
    // for (const auto &pt : ret_coords)
    //     ys.push_back(static_cast<double>(pt.y));
    // medianFilter1D(ys, 15);
    // for (std::size_t i = 0; i < ret_coords.size(); ++i)
    //     ret_coords[i].y = static_cast<int>(std::round(ys[i]));

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
            // pc_3d.emplace_back(Eigen::Vector3d(-y, z_val, x));
        }

        if (acq_interval && pc_3d.size() >= static_cast<size_t>(interval)) {
            break;
        }
    }

    return pc_3d;
}
