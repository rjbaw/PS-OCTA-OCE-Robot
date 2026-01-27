/**
 * @file process_img.cpp
 * @author rjbaw
 */

#include "process_img.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
#include <numeric>
#include <unordered_map>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <onnxruntime/onnxruntime_c_api.h>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <onnxruntime/provider_options.h>
#include <opencv4/opencv2/core/base.hpp>

namespace octa_ros::img {

void medianFilter1D(std::vector<cv::Point> &coords) {
    if (coords.empty()) {
        return;
    }
    std::vector<double> signal;
    signal.reserve(coords.size());
    for (const auto &pt : coords) {
        signal.push_back(static_cast<double>(pt.y));
    }

    int window_size = 20;
    int half_win = window_size / 2;
    std::vector<double> extended;
    {
        const auto pad =
            static_cast<std::vector<double>::size_type>(2LL * half_win);
        extended.reserve(signal.size() + pad);
    }

    for (int i = 0; i < half_win; ++i) {
        extended.push_back(signal.front());
    }
    for (double val : signal) {
        extended.push_back(val);
    }
    for (int i = 0; i < half_win; ++i) {
        extended.push_back(signal.back());
    }

    for (int i = 0; i < (int)signal.size(); ++i) {
        std::vector<double> window(extended.begin() + i,
                                   extended.begin() + i + window_size);
        std::nth_element(window.begin(), window.begin() + window_size / 2,
                         window.end());
        double med = window[window_size / 2];
        signal[i] = med;
    }

    for (size_t i = 0; i < coords.size(); ++i) {
        coords[i].y = static_cast<int>(std::lround(signal[i]));
    }
}

void onlineKalmanFilter1D(std::vector<cv::Point> &coords) {
    if (coords.empty()) {
        return;
    }

    const int Tinit = 4;
    const double beta = 2.0 / std::log(static_cast<double>(Tinit));
    const double lambda = 10.0;

    CV_Assert(Tinit > (beta * std::log(static_cast<double>(Tinit))));

    const int N_length = static_cast<int>(coords.size());
    if (N_length <= 0) {
        return;
    }

    Eigen::VectorXd y(N_length);
    for (int i = 0; i < N_length; ++i) {
        y[i] = static_cast<double>(coords[static_cast<size_t>(i)].y);
    }

    Eigen::VectorXd ytilde = Eigen::VectorXd::Zero(N_length);
    std::vector<uint8_t> has_pred(static_cast<size_t>(N_length), 0);

    int i = 1;
    while (true) {
        const int T =
            static_cast<int>(std::pow(2.0, static_cast<double>(i - 1))) * Tinit;
        const int p =
            std::max(1, static_cast<int>(std::ceil(
                            beta * std::log(static_cast<double>(T)))));

        if (T <= p || (T + p) > N_length) {
            break;
        }

        Eigen::MatrixXd V_inv;
        {
            // V̄_{T-1} = lambda * I_{p} + ∑_{t=p}^{T-1} Z_t,p @ Z_t,p^{*}
            Eigen::MatrixXd V_prev = lambda * Eigen::MatrixXd::Identity(p, p);
            Eigen::RowVectorXd yZ_sum = Eigen::RowVectorXd::Zero(p);
            for (int t = p; t < (T - 1); ++t) {
                Eigen::Map<const Eigen::VectorXd> Z_t(&y[t - p], p);
                V_prev.noalias() += Z_t * Z_t.transpose();
                yZ_sum.noalias() += (y[t] * Z_t.transpose());
            }

            // G̃_{T-1} = (∑_{t=p}^{T-1} y_t @ Z_t,p^{*} ) @ V̄_{T-1}^{-1}
            V_inv = V_prev.inverse();
            Eigen::RowVectorXd G_prev = yZ_sum * V_inv;

            for (int k = T; k < std::min(2 * T - 1, N_length); ++k) {

                Eigen::Map<const Eigen::VectorXd> Z_k(&y[k - p], p);

                // predict ỹ_k = G̃_{k-1} @ Z_k,p
                ytilde[k] = (G_prev * Z_k)(0, 0);
                has_pred[static_cast<size_t>(k)] = 1;

                // update V̄_k = V̄_{k-1} + Z_k,p @ Z_k,p^{*}

                // Sherman-Morrison
                // V_k^{-1} = (V + z z^T)^{-1} = V^{-1} − (V^{-1} z z^T V^{-1})
                // / (1 + z^T V^{-1} z)
                V_inv = V_inv - (V_inv * Z_k * Z_k.transpose() * V_inv) /
                                    (1.0 + Z_k.transpose() * V_inv * Z_k);

                // G̃_k = G̃_{k-1} + (y_k - ỹ_k) @ (Z_k,p^{*} * V̄_k^{-1})
                const Eigen::RowVectorXd G_k =
                    G_prev + (y[k] - ytilde[k]) * (Z_k.transpose() * V_inv);
                G_prev = G_k;
            }
        }
        ++i;
    }

    for (int n = 0; n < N_length; ++n) {
        if (has_pred[static_cast<size_t>(n)] != 0U) {
            coords[static_cast<size_t>(n)].y =
                static_cast<int>(std::lround(ytilde[n]));
        }
    }
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

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord) {
    for (size_t i = 0; i < ret_coord.size() - 1; ++i) {
        cv::Point pt1 = ret_coord[i];
        cv::Point pt2 = ret_coord[i + 1];
        cv::line(image, pt1, pt2, cv::Scalar(255, 255, 255), 2);
    }
}

static std::string g_curve_model_path;

void set_curve_model_path(const std::string &path) {
    g_curve_model_path = path;
}

static Ort::Session &load_session(const std::string &path,
                                  int64_t batch_override);

void preload_curve_model(std::size_t batch_size) {
    (void)batch_size;
#ifndef LEGACY_IMG_PIPELINE
    std::string model_path;
    if (!g_curve_model_path.empty()) {
        model_path = g_curve_model_path;
    } else {
        const auto share =
            ament_index_cpp::get_package_share_directory("octa_ros");
        std::filesystem::path model_stdpath =
            std::filesystem::path(share) / "config/curve_model.onnx";
        model_path = model_stdpath.string();
    }
    (void)load_session(model_path,
                       static_cast<int64_t>(batch_size == 0 ? 1 : batch_size));
#endif
}

[[maybe_unused]] static cv::Mat gradient_legacy(const cv::Mat &img) {
    CV_Assert(img.channels() == 1);

    cv::Mat img_f;
    img.convertTo(img_f, CV_32F);

    std::array<float, 9> kx_vals{0.0F, 0.0F, 0.0F, -0.5F, 0.0F,
                                 0.5F, 0.0F, 0.0F, 0.0F};
    cv::Mat kx(3, 3, CV_32F, kx_vals.data());

    cv::Mat ky = kx.t();

    cv::Mat gx;
    cv::Mat gy;
    cv::filter2D(img_f, gx, -1, kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(img_f, gy, -1, ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

    cv::Mat gx2;
    cv::Mat gy2;
    cv::Mat mag;
    cv::multiply(gx, gx, gx2);
    cv::multiply(gy, gy, gy2);

    gy2 *= 0.65F;
    cv::add(gx2, gy2, mag);
    cv::sqrt(mag, mag);

    return mag;
}

[[maybe_unused]] static cv::Mat build_gaussian_filter_legacy(int nx, int ny) {
    cv::Mat kernel(ny, nx, CV_32F);

    float cx = static_cast<float>(nx - 1) / 2.0F;
    float cy = static_cast<float>(ny - 1) / 2.0F;
    float sigmaX = static_cast<float>(nx) / 4.0F;
    float sigmaY = static_cast<float>(ny) / 4.0F;

    double sumVal = 0.0;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            float x = static_cast<float>(i) - cx;
            float y = static_cast<float>(j) - cy;
            float val = std::exp(-(x * x) / (sigmaX * sigmaX)) *
                        std::exp(-(y * y) / (sigmaY * sigmaY));
            kernel.at<float>(j, i) = val;
            sumVal += static_cast<double>(val);
        }
    }

    kernel /= static_cast<float>(sumVal);

    return kernel;
}

[[maybe_unused]] static cv::Mat lowpass_legacy(const cv::Mat &img, int nx,
                                               int ny) {
    cv::Mat kernel = build_gaussian_filter_legacy(nx, ny);

    cv::Mat dst;
    cv::filter2D(img, dst, -1, kernel, cv::Point(-1, -1), 0,
                 cv::BORDER_REPLICATE);

    return dst;
}

[[maybe_unused]] static cv::Mat load_bg_legacy() {
    try {
        std::string pkg_share =
            ament_index_cpp::get_package_share_directory("octa_ros");
        std::string bg_path = pkg_share + "/config/bg.jpg";
        cv::Mat bg = cv::imread(bg_path, cv::IMREAD_GRAYSCALE);
        if (bg.empty()) {
            std::cerr << "[process_img] Background image not found at: "
                      << bg_path << "; skipping background subtraction.\n";
        }
        return bg;
    } catch (const std::exception &e) {
        std::cerr << "[process_img] Could not resolve package share ("
                  << e.what() << "); skipping background subtraction.\n";
        return {};
    }
}

[[maybe_unused]] static cv::Mat bg_sub_legacy(const cv::Mat &input) {
    cv::Mat bg = load_bg_legacy();
    if (bg.empty() || bg.size() != input.size() || bg.type() != input.type()) {
        return input.clone();
    }

    cv::Mat input_f;
    cv::Mat bg_f;
    input.convertTo(input_f, CV_32F);
    bg.convertTo(bg_f, CV_32F);

    cv::Mat sub_f = input_f - bg_f;
    cv::Mat output;
    cv::normalize(sub_f, output, 0, 255, cv::NORM_MINMAX, CV_8U);

    return output;
}

[[maybe_unused]] static cv::Mat spatialFilter_legacy(const cv::Mat &input) {
    cv::Mat raw;
    input.convertTo(raw, CV_32F);
    cv::Mat lp = lowpass_legacy(raw, 11, 5);
    cv::Mat grad = gradient_legacy(lp);
    cv::Mat grad_lp = lowpass_legacy(grad, 1, 3);
    cv::Mat output = lp.mul(grad_lp);
    cv::normalize(output, output, 0, 255, cv::NORM_MINMAX, CV_8U);

    return output;
}

[[maybe_unused]] static double median_legacy(const std::vector<double> &values,
                                             int N) {
    int initCount = std::min(static_cast<int>(values.size()), N);
    if (initCount == 0) {
        return 0.0;
    }

    std::vector<double> subset(
        values.begin(),
        values.begin() +
            static_cast<std::vector<double>::difference_type>(initCount));
    std::sort(subset.begin(), subset.end());

    if ((initCount % 2) == 1) {
        return subset[static_cast<size_t>(initCount / 2)];
    }
    double lower = subset[static_cast<size_t>((initCount / 2) - 1)];
    double upper = subset[static_cast<size_t>(initCount / 2)];
    return 0.5 * (lower + upper);
}

[[maybe_unused]] static std::vector<double>
kalmanFilter1D_legacy(const std::vector<double> &observations, double Q = 0.01,
                      double R = 0.5) {
    std::vector<double> x_k_estimates;
    x_k_estimates.reserve(observations.size());
    if (observations.empty()) {
        return x_k_estimates;
    }

    double x0 = median_legacy(observations, 10);
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

[[maybe_unused]] static std::vector<cv::Point>
ol_removal_legacy(const std::vector<cv::Point> &coords) {
    if (coords.empty()) {
        return {};
    }

    std::vector<double> observations;
    observations.reserve(coords.size());
    for (const auto &pt : coords) {
        observations.push_back(static_cast<double>(pt.y));
    }

    const int obs_length = static_cast<int>(observations.size());
    const int window = std::max(1, obs_length / 3);
    const double z_max = 40.0;

    double best_slope = 0.0;
    double best_sigma = std::numeric_limits<double>::infinity();

    const int segmentCount = obs_length / 3;
    for (int w = 0; w < segmentCount; ++w) {
        const int start = w * window;
        const int end = std::min(start + window, obs_length - 1);
        if (end <= start) {
            continue;
        }

        std::vector<double> segment(observations.begin() + start,
                                    observations.begin() + end);

        const double meanVal =
            std::accumulate(segment.begin(), segment.end(), 0.0) /
            static_cast<double>(segment.size());
        double accum = 0.0;
        for (double val : segment) {
            const double diff = val - meanVal;
            accum += diff * diff;
        }
        double stdv =
            (segment.empty())
                ? 0.0
                : std::sqrt(accum / static_cast<double>(segment.size()));

        auto median_filter_legacy = [](std::vector<double> &signal,
                                       int window_size) {
            const int half_win = window_size / 2;
            std::vector<double> extended;
            extended.reserve(signal.size() +
                             static_cast<std::size_t>(2 * half_win));

            for (int i = 0; i < half_win; ++i) {
                extended.push_back(signal.front());
            }
            for (double val : signal) {
                extended.push_back(val);
            }
            for (int i = 0; i < half_win; ++i) {
                extended.push_back(signal.back());
            }

            for (int i = 0; i < static_cast<int>(signal.size()); ++i) {
                std::vector<double> window(extended.begin() + i,
                                           extended.begin() + i + window_size);
                std::nth_element(
                    window.begin(),
                    window.begin() +
                        static_cast<std::vector<double>::difference_type>(
                            window_size / 2),
                    window.end());
                double med = window[static_cast<std::size_t>(window_size / 2)];
                signal[static_cast<std::size_t>(i)] = med;
            }
        };
        median_filter_legacy(segment, window);
        double slopeCandidate = (segment.back() - segment.front()) /
                                static_cast<double>(segment.size());

        if (stdv < best_sigma && std::fabs(slopeCandidate) < z_max) {
            best_sigma = stdv;
            best_slope = slopeCandidate;
        }
    }

    for (int i = 0; i < obs_length; ++i) {
        if (i == 0) {
            const int end = std::min(20, obs_length);
            std::vector<double> firstChunk(observations.begin(),
                                           observations.begin() + end);
            std::nth_element(firstChunk.begin(),
                             firstChunk.begin() +
                                 static_cast<long>(firstChunk.size() / 2),
                             firstChunk.end());
            observations[0] =
                firstChunk[static_cast<size_t>(firstChunk.size() / 2)];
        } else {
            double prev_pt = observations[static_cast<size_t>(i - 1)];
            double pt = observations[static_cast<size_t>(i)];
            double mse = std::fabs(pt - prev_pt);
            if (mse > z_max) {
                observations[static_cast<size_t>(i)] = prev_pt + best_slope;
            }
        }
    }

    std::vector<cv::Point> new_coords;
    new_coords.reserve(coords.size());
    for (int i = 0; i < obs_length; ++i) {
        new_coords.emplace_back(
            coords[static_cast<size_t>(i)].x,
            static_cast<int>(std::round(observations[static_cast<size_t>(i)])));
    }
    return new_coords;
}

[[maybe_unused]] static std::vector<cv::Point>
get_max_coor_legacy(const cv::Mat &img) {
    const int width = img.cols;
    std::vector<cv::Point> ret_coords(static_cast<size_t>(width));

    for (int x = 0; x < width; ++x) {
        cv::Mat intensity = img.col(x);
        double minVal;
        double maxVal;
        cv::Point minLoc;
        cv::Point maxLoc;
        cv::minMaxLoc(intensity, &minVal, &maxVal, &minLoc, &maxLoc);
        int detected_y = maxLoc.y;
        ret_coords[static_cast<size_t>(x)] = cv::Point(x, detected_y);
    }
    return ret_coords;
}

[[maybe_unused]] static SegmentResult
detect_lines_legacy(const cv::Mat &inputImg) {
    CV_Assert(!inputImg.empty());

    cv::Mat img_raw;
    if (inputImg.channels() == 3) {
        cv::cvtColor(inputImg, img_raw, cv::COLOR_BGR2GRAY);
    } else {
        img_raw = inputImg.clone();
    }

    cv::Mat sub_image = bg_sub_legacy(img_raw);
    cv::Mat denoised_image = spatialFilter_legacy(sub_image);

    std::vector<cv::Point> ret_coords = get_max_coor_legacy(denoised_image);
    ret_coords = ol_removal_legacy(ret_coords);

    std::vector<double> obs;
    obs.reserve(ret_coords.size());
    for (const auto &pt : ret_coords) {
        obs.push_back(static_cast<double>(pt.y));
    }
    std::vector<double> kf_out = kalmanFilter1D_legacy(obs, 0.01, 0.5);
    for (size_t i = 0; i < ret_coords.size(); ++i) {
        ret_coords[i].y = static_cast<int>(std::round(kf_out[i]));
    }

    cv::Mat detected_img = img_raw.clone();
    draw_line(detected_img, ret_coords);

    SegmentResult result;
    result.image = std::move(detected_img);
    result.coordinates = std::move(ret_coords);
    return result;
}

static Ort::Env &get_ort_env() {
    static auto *env = new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "octa_ros");
    return *env;
}

static Ort::SessionOptions build_session_options(const OrtApi &api,
                                                 int64_t batch_override) {
    Ort::SessionOptions opts;
    OrtSessionOptions *raw = opts;
    if (batch_override > 0) {
        Ort::ThrowOnError(
            api.AddFreeDimensionOverrideByName(raw, "batch", batch_override));
    }
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    return opts;
}

static Ort::Session &load_session(const std::string &path,
                                  int64_t batch_override) {
    static Ort::Session *session = nullptr;
    static std::string cached_path;
    static int64_t cached_batch = -1;

    auto &env = get_ort_env();

    if (session == nullptr || cached_path != path ||
        cached_batch != batch_override) {
        const OrtApi &api = Ort::GetApi();

        for (auto &provider : Ort::GetAvailableProviders()) {
            std::cerr << "[ORT] available EP: " << provider << "\n";
        }

        Ort::Session *new_session = nullptr;
        const char *ep_name = "CPU";
        try {
            auto opts = build_session_options(api, batch_override);
            OrtCUDAProviderOptionsV2 *cuda_opts = nullptr;
            Ort::ThrowOnError(api.CreateCUDAProviderOptions(&cuda_opts));
            Ort::ThrowOnError(api.SessionOptionsAppendExecutionProvider_CUDA_V2(
                opts, cuda_opts));
            api.ReleaseCUDAProviderOptions(cuda_opts);
            new_session = new Ort::Session(env, path.c_str(), opts);
            ep_name = "CUDA";
        } catch (const Ort::Exception &ex) {
            std::cerr << "[ORT] CUDA session init failed: " << ex.what()
                      << "\n";
        }

        if (new_session == nullptr) {
            try {
                auto opts = build_session_options(api, batch_override);
                std::unordered_map<std::string, std::string> ov_opts;
                ov_opts["device_type"] = "HETERO:GPU,NPU";
                ov_opts["precision"] = "FP16";
                opts.AppendExecutionProvider_OpenVINO_V2(ov_opts);
                new_session = new Ort::Session(env, path.c_str(), opts);
                ep_name = "OpenVINO";
            } catch (const Ort::Exception &ex) {
                std::cerr << "[ORT] OpenVINO session init failed: " << ex.what()
                          << "\n";
            }
        }

        if (new_session == nullptr) {
            auto opts = build_session_options(api, batch_override);
            new_session = new Ort::Session(env, path.c_str(), opts);
            ep_name = "CPU";
        }

        delete session;
        session = new_session;
        cached_path = path;
        cached_batch = batch_override;
        static bool printed = false;
        if (!printed) {
            std::cerr << "[ORT] Using EP: " << ep_name << "\n";
            printed = true;
        }
    }
    return *session;
}

[[maybe_unused]] static std::vector<SegmentResult>
infer_batch(const std::vector<cv::Mat> &frames) {
    if (frames.empty()) {
        return {};
    }

    const size_t batch_size = frames.size();
    const cv::Mat &first_img = frames.front();
    CV_Assert(!first_img.empty());
    CV_Assert(first_img.channels() == 1 || first_img.channels() == 3);
    const int img_h = first_img.rows;
    const int img_w = first_img.cols;
    CV_Assert(img_h > 0 && img_w > 0);

    for (size_t idx = 1; idx < batch_size; ++idx) {
        const cv::Mat &img = frames[idx];
        CV_Assert(!img.empty());
        CV_Assert(img.channels() == 1 || img.channels() == 3);
        CV_Assert(img.rows == img_h && img.cols == img_w);
    }

    std::string model_path;
    if (!g_curve_model_path.empty()) {
        model_path = g_curve_model_path;
    } else {
        const auto share =
            ament_index_cpp::get_package_share_directory("octa_ros");
        std::filesystem::path model_stdpath =
            std::filesystem::path(share) / "config/curve_model.onnx";
        model_path = model_stdpath.string();
    }

    auto &session = load_session(model_path, static_cast<int64_t>(batch_size));
    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name_alloc = session.GetInputNameAllocated(0, allocator);
    std::string input_name = input_name_alloc.get();
    auto input_type_info = session.GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> expected_shape = input_tensor_info.GetShape();
    const int run_h = img_h;
    const int run_w = img_w;
    if (expected_shape.size() == 4) {
        const int64_t exp_h = expected_shape[2];
        const int64_t exp_w = expected_shape[3];
        if (exp_h > 0) {
            CV_Assert(exp_h == run_h);
        }
        if (exp_w > 0) {
            CV_Assert(exp_w == run_w);
        }
    }

    std::vector<float> input_data(static_cast<size_t>(batch_size) * 3 * run_h *
                                  run_w);
    for (size_t idx = 0; idx < batch_size; ++idx) {
        const cv::Mat &img = frames[idx];
        cv::Mat img_u8;
        if (img.channels() == 1) {
            img_u8 = img;
        } else {
            CV_Assert(img.channels() == 3);
            cv::cvtColor(img, img_u8, cv::COLOR_BGR2GRAY);
        }
        CV_Assert(img_u8.type() == CV_8UC1);

        cv::Mat img_f32;
        img_u8.convertTo(img_f32, CV_32F, 1.0 / 255.0);

        cv::Scalar mu_s;
        cv::Scalar sigma_s;
        cv::meanStdDev(img_f32, mu_s, sigma_s);
        const auto mu = static_cast<float>(mu_s[0]);
        auto sigma = static_cast<float>(sigma_s[0]);
        sigma = std::max(sigma, 1e-6F);

        const size_t plane = static_cast<size_t>(run_h) * run_w;
        const size_t base = idx * static_cast<size_t>(3) * plane;
        for (int py = 0; py < run_h; ++py) {
            const float *row = img_f32.ptr<float>(py);
            for (int px = 0; px < run_w; ++px) {
                const float v = (row[px] - mu) / sigma;
                const size_t hw = static_cast<size_t>(py) * run_w + px;
                input_data[base + 0 * plane + hw] = v;
                input_data[base + 1 * plane + hw] = v;
                input_data[base + 2 * plane + hw] = v;
            }
        }
    }

    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemTypeDefault);
    std::array<int64_t, 4> input_shape{static_cast<int64_t>(batch_size), 3,
                                       static_cast<int64_t>(run_h),
                                       static_cast<int64_t>(run_w)};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem_info, input_data.data(), input_data.size(), input_shape.data(),
        input_shape.size());

    size_t num_outputs = session.GetOutputCount();
    CV_Assert(num_outputs >= 2);
    std::vector<std::string> output_names(num_outputs);
    for (size_t i = 0; i < num_outputs; ++i) {
        auto out_name_alloc = session.GetOutputNameAllocated(i, allocator);
        output_names[i] = out_name_alloc.get();
    }
    const std::array<const char *, 1> in_names{input_name.c_str()};
    std::vector<const char *> out_names;
    out_names.reserve(output_names.size());
    for (const auto &name_str : output_names) {
        out_names.push_back(name_str.c_str());
    }

    auto outputs =
        session.Run(Ort::RunOptions{nullptr}, in_names.data(), &input_tensor, 1,
                    out_names.data(), out_names.size());

    CV_Assert(outputs.size() >= 2);

    const Ort::Value &presence_val = outputs[0];
    const auto *presence_ptr = presence_val.GetTensorData<float>();
    const size_t presence_elems =
        presence_val.GetTensorTypeAndShapeInfo().GetElementCount();

    const Ort::Value &curve_val = outputs[1];
    const auto *curve_ptr = curve_val.GetTensorData<float>();
    const size_t curve_elems =
        curve_val.GetTensorTypeAndShapeInfo().GetElementCount();

    CV_Assert(presence_elems >= batch_size &&
              (presence_elems % batch_size) == 0);
    CV_Assert(curve_elems >= batch_size && (curve_elems % batch_size) == 0);

    const size_t presence_stride = presence_elems / batch_size;
    const size_t curve_stride = curve_elems / batch_size;
    const auto to_prob = [](float logit) {
        return 1.0F / (1.0F + std::exp(-logit));
    };

    const double denom = (img_w > 1) ? static_cast<double>(img_w - 1) : 1.0;
    const double scale_h =
        static_cast<double>(img_h) / static_cast<double>(run_h);

    std::vector<SegmentResult> results;
    results.reserve(batch_size);
    for (size_t idx = 0; idx < batch_size; ++idx) {
        SegmentResult result{frames[idx].clone(), {}};

        const float keep_curve_prob = 0.7F;
        if (to_prob(presence_ptr[idx * presence_stride]) >= keep_curve_prob &&
            curve_stride > 0) {
            result.coordinates.resize(img_w);
            for (int column_idx = 0; column_idx < img_w; ++column_idx) {
                size_t src = curve_stride == 1
                                 ? 0
                                 : static_cast<size_t>(std::lround(
                                       (static_cast<double>(column_idx) *
                                        static_cast<double>(curve_stride - 1)) /
                                       denom));
                src = std::min(src, curve_stride - 1);
                int row_val = std::clamp(
                    static_cast<int>(std::lround(
                        curve_ptr[idx * curve_stride + src] * scale_h)),
                    0, img_h - 1);
                result.coordinates[static_cast<size_t>(column_idx)] =
                    cv::Point(column_idx, row_val);
            }
        }
        medianFilter1D(result.coordinates);
        onlineKalmanFilter1D(result.coordinates);
        if (!result.coordinates.empty()) {
            draw_line(result.image, result.coordinates);
        }
        results.emplace_back(std::move(result));
    }

    return results;
}

SegmentResult detect_lines(const cv::Mat &inputImg) {
#ifdef LEGACY_IMG_PIPELINE
    return detect_lines_legacy(inputImg);
#else
    auto results = infer_batch({inputImg});
    return results.empty() ? SegmentResult{inputImg.clone(), {}}
                           : std::move(results.front());
#endif
}

inline std::filesystem::path
prepare_output_dir(const std::filesystem::path &base_dir, bool make_session) {
    std::filesystem::path dir = base_dir;
    if (make_session) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);

        std::tm time_struct{};
        localtime_r(&time_t, &time_struct);
        std::ostringstream oss;
        oss << std::put_time(&time_struct, "%Y-%m-%dT%H-%M-%S");
        dir /= oss.str();
    }
    std::filesystem::create_directories(dir);
    return dir;
}

std::vector<Eigen::Vector3d> lines_3d(const std::vector<cv::Mat> &img_array,
                                      const int interval) {
    bool save_debug = true;
    if (const char *env = std::getenv("OCTA_SAVE_DEBUG"); env != nullptr) {
        std::string env_var = env;
        save_debug =
            (env_var != "0" && env_var != "false" && env_var != "FALSE");
    }

    std::filesystem::path out_dir;
    if (save_debug) {
        const std::filesystem::path base_out_dir = "result";
        const bool create_session_dir = true;
        out_dir = prepare_output_dir(base_out_dir, create_session_dir);
    }

    std::vector<Eigen::Vector3d> pc_3d;
    if (img_array.empty()) {
        return pc_3d;
    }

    int num_frames = interval > 1 ? interval : 2;
    int img_w = img_array.front().cols;
    double increments = (static_cast<double>(img_w) - 1.0) /
                        static_cast<double>(num_frames - 1);

    std::vector<SegmentResult> detections;
#ifdef LEGACY_IMG_PIPELINE
    detections.reserve(img_array.size());
    for (const auto &frame : img_array) {
        detections.emplace_back(detect_lines_legacy(frame));
    }
#else
    detections = infer_batch(img_array);
    CV_Assert(detections.size() == img_array.size());
#endif

    for (size_t i = 0; i < img_array.size(); ++i) {
        const cv::Mat &img = img_array[i];
        const SegmentResult &point_cloud = detections[i];
        if (save_debug) {
            const std::string raw_filename = std::format("raw_image{}.jpg", i);
            const std::string processed_filename =
                std::format("detected_image{}.jpg", i);
            cv::imwrite((out_dir / raw_filename).string(), img);
            cv::imwrite((out_dir / processed_filename).string(),
                        point_cloud.image);
        }
        if (point_cloud.coordinates.empty()) {
            continue;
        }

        int idx = static_cast<int>(i) % interval;
        double z_val = idx * increments;

        for (const auto &coordinate : point_cloud.coordinates) {
            auto x_pts = static_cast<double>(coordinate.x);
            auto y_pts = static_cast<double>(coordinate.y);
            pc_3d.emplace_back(x_pts, z_val, y_pts);
        }
    }

    return pc_3d;
}

} // namespace octa_ros::img
