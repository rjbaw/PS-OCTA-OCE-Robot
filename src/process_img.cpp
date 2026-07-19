/**
 * @file process_img.cpp
 * @author rjbaw
 */

#include "process_img.hpp"
#include "robust_plane_fit.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
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
static std::vector<SegmentResult>
infer_batch(const std::vector<cv::Mat> &frames);

void preload_curve_model(std::size_t batch_size) {
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
}

void warmup_curve_model(const cv::Mat &example_frame,
                        const std::size_t batch_size) {
    if (example_frame.empty()) {
        return;
    }
    const size_t run_batch = batch_size == 0 ? 1 : batch_size;
    std::vector<cv::Mat> frames(run_batch, example_frame);
    (void)infer_batch(frames);
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

static std::vector<SegmentResult>
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
        // medianFilter1D(result.coordinates);
        if (!result.coordinates.empty()) {
            draw_line(result.image, result.coordinates);
        }
        results.emplace_back(std::move(result));
    }

    return results;
}

SegmentResult detect_lines(const cv::Mat &inputImg) {
    auto results = infer_batch({inputImg});
    return results.empty() ? SegmentResult{inputImg.clone(), {}}
                           : std::move(results.front());
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
SurfaceReconstructionResult
reconstruct_surface(const std::vector<cv::Mat> &img_array, const int interval) {

    SurfaceReconstructionResult out;

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

    if (img_array.empty()) {
        return out;
    }

    const int num_frames = interval > 1 ? interval : 2;
    const int img_w = img_array.front().cols;
    const double increments = (static_cast<double>(img_w) - 1.0) /
                              static_cast<double>(num_frames - 1);

    std::vector<SegmentResult> detections = infer_batch(img_array);
    CV_Assert(detections.size() == img_array.size());

    using Points3d = octa_ros::img::Points3d;
    std::vector<Points3d> strips;
    strips.reserve(detections.size());
    int valid_strip_count = 0;
    double min_sweep_pos = std::numeric_limits<double>::infinity();
    double max_sweep_pos = -std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < img_array.size(); ++i) {
        const cv::Mat &img = img_array[i];
        const SegmentResult &det = detections[i];

        if (save_debug) {
            const std::string raw_filename = std::format("raw_image{}.jpg", i);
            const std::string processed_filename =
                std::format("detected_image{}.jpg", i);
            cv::imwrite((out_dir / raw_filename).string(), img);
            cv::imwrite((out_dir / processed_filename).string(), det.image);
        }

        if (det.coordinates.empty()) {
            continue;
        }

        const int idx = static_cast<int>(i) % num_frames;
        const double sweep_pos = idx * increments;
        min_sweep_pos = std::min(min_sweep_pos, sweep_pos);
        max_sweep_pos = std::max(max_sweep_pos, sweep_pos);
        ++valid_strip_count;

        Points3d strip(static_cast<Eigen::Index>(det.coordinates.size()), 3);

        for (size_t k = 0; k < det.coordinates.size(); ++k) {
            const auto &coord = det.coordinates[k];

            const auto x = static_cast<double>(coord.x);
            const auto z = static_cast<double>(coord.y);
            Eigen::Vector3d p(x, sweep_pos, z);

            out.point_cloud.push_back(p);
            strip.row(static_cast<Eigen::Index>(k)) = p.transpose();
        }

        strips.push_back(std::move(strip));
    }

    if (valid_strip_count < 2 || (max_sweep_pos - min_sweep_pos) < 1e-9) {
        return out;
    }

    RobustPlaneOptions opts;
    opts.up_dir = Eigen::Vector3d::UnitZ();
    opts.sweep_direction = Eigen::Vector3d::UnitY();

    const auto fit = fit_robust_plane_tukey_irls(strips, opts);
    if (!fit.ok) {
        return out;
    }

    out.pose_ok = true;
    out.rotation = fit.rotation;
    out.center = fit.center;
    out.normal = fit.normal;
    out.d = fit.d;
    out.robust_scale = fit.scale;

    return out;
}

std::vector<Eigen::Vector3d> lines_3d(const std::vector<cv::Mat> &img_array,
                                      const int interval) {
    return reconstruct_surface(img_array, interval).point_cloud;
}

} // namespace octa_ros::img
