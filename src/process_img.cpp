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
#include <unordered_map>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <onnxruntime/onnxruntime_c_api.h>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <onnxruntime/provider_options.h>
#include <opencv4/opencv2/core/base.hpp>

namespace octa_ros::img {

static Ort::Env &get_ort_env() {
    static auto *env = new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "octa_ros");
    return *env;
}

static Ort::Session &load_session(const std::string &path,
                                  int64_t batch_override) {
    static Ort::Session *session = nullptr;
    static std::string cached_path;
    static int64_t cached_batch = -1;

    auto &env = get_ort_env();

    if (session == nullptr || cached_path != path ||
        cached_batch != batch_override) {
        Ort::SessionOptions opts;
        const OrtApi &api = Ort::GetApi();
        OrtSessionOptions *raw = opts;
        if (batch_override > 0) {
            Ort::ThrowOnError(
                api.AddFreeDimensionOverrideByName(raw, "batch",
                                                   batch_override));
        }
        opts.SetIntraOpNumThreads(1);
        opts.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        for (auto &provider : Ort::GetAvailableProviders()) {
            std::cerr << "[ORT] available EP: " << provider << "\n";
        }

        bool ep_set = false;
        const char *ep_name = "CPU";
        try {
            OrtCUDAProviderOptionsV2 *cuda_opts = nullptr;
            Ort::ThrowOnError(api.CreateCUDAProviderOptions(&cuda_opts));
            Ort::ThrowOnError(api.SessionOptionsAppendExecutionProvider_CUDA_V2(
                opts, cuda_opts));
            api.ReleaseCUDAProviderOptions(cuda_opts);
            ep_set = true;
            ep_name = "CUDA";
        } catch (const Ort::Exception &ex) {
            std::cerr << "[ORT] " << ex.what() << "\n";
        }
        if (!ep_set) {
            try {
                std::unordered_map<std::string, std::string> ov_opts;
                ov_opts["device_type"] = "HETERO:GPU,NPU";
                ov_opts["precision"] = "FP16";
                opts.AppendExecutionProvider_OpenVINO_V2(ov_opts);
                ep_set = true;
                ep_name = "OpenVINO";
            } catch (const Ort::Exception &ex) {
                std::cerr << "[ORT] OpenVINO EP append failed: " << ex.what()
                          << "\n";
            }
        }

        delete session;
        session = new Ort::Session(env, path.c_str(), opts);
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

static std::vector<SegmentResult>
infer_batch(const std::vector<cv::Mat> &frames) {
    if (frames.empty()) {
        return {};
    }

    const size_t batch_size = frames.size();
    const cv::Mat &first_img = frames.front();
    CV_Assert(!first_img.empty());
    CV_Assert(first_img.channels() == 1);
    const int img_h = first_img.rows;
    const int img_w = first_img.cols;
    CV_Assert(img_h > 0 && img_w > 0);

    for (size_t idx = 1; idx < batch_size; ++idx) {
        const cv::Mat &img = frames[idx];
        CV_Assert(!img.empty());
        CV_Assert(img.channels() == 1);
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

    const std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
    const std::array<float, 3> stdev{0.229F, 0.224F, 0.225F};
    std::vector<float> input_data(static_cast<size_t>(batch_size) * 3 * run_h *
                                  run_w);
    for (size_t idx = 0; idx < batch_size; ++idx) {
        const cv::Mat &img = frames[idx];
        cv::Mat rgb_u8;
        cv::cvtColor(img, rgb_u8, cv::COLOR_GRAY2RGB);
        cv::Mat rgb_f32;
        rgb_u8.convertTo(rgb_f32, CV_32F, 1.0 / 255.0);
        CV_Assert(rgb_f32.isContinuous());

        size_t base = idx * static_cast<size_t>(3 * run_h * run_w);
        size_t cursor = base;
        for (int ch = 0; ch < 3; ++ch) {
            for (int py = 0; py < run_h; ++py) {
                const float *row = rgb_f32.ptr<float>(py);
                for (int px = 0; px < run_w; ++px) {
                    const float val = row[px * 3 + ch];
                    input_data[cursor++] =
                        (val - mean[static_cast<size_t>(ch)]) /
                        stdev[static_cast<size_t>(ch)];
                }
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
        if (to_prob(presence_ptr[idx * presence_stride]) >= 0.5F &&
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

    std::vector<SegmentResult> detections = infer_batch(img_array);
    CV_Assert(detections.size() == img_array.size());

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
