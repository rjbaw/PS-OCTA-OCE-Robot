#include "process_img.hpp"
#include <cmath>
#include <filesystem>
#include <format>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <onnxruntime/onnxruntime_c_api.h>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <onnxruntime/provider_options.h>
#include <opencv4/opencv2/core/base.hpp>

namespace octa_ros::img {

static Ort::Env &get_ort_env() {
    static auto *env = new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "octa_ros");
    return *env;
}

static Ort::Session &load_session(const std::string &path) {
    static Ort::Session *session = nullptr;
    static std::string cached_path;

    if (session == nullptr || cached_path != path) {
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        opts.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        bool ep_set = false;
        try {
            OrtCUDAProviderOptionsV2 *cuda_opts = nullptr;
            Ort::ThrowOnError(
                Ort::GetApi().CreateCUDAProviderOptions(&cuda_opts));
            Ort::ThrowOnError(
                Ort::GetApi().SessionOptionsAppendExecutionProvider_CUDA_V2(
                    opts, cuda_opts));
            Ort::GetApi().ReleaseCUDAProviderOptions(cuda_opts);
            ep_set = true;
        } catch (const Ort::Exception &ex) {
            (void)ex;
        }
        if (!ep_set) {
            try {
                OrtOpenVINOProviderOptions ov_opts;
                ov_opts.device_type = "GPU_FP32";
                Ort::ThrowOnError(
                    Ort::GetApi()
                        .SessionOptionsAppendExecutionProvider_OpenVINO(
                            opts, &ov_opts));
                ep_set = true;
            } catch (const Ort::Exception &ex) {
                (void)ex;
            }
        }
        session = new Ort::Session(get_ort_env(), path.c_str(), opts);
        cached_path = path;
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

SegmentResult detect_lines(const cv::Mat &inputImg) {
    CV_Assert(!inputImg.empty());
    const int img_h = inputImg.rows;
    const int img_w = inputImg.cols;
    CV_Assert(img_h > 0 && img_w > 0);
    CV_Assert(inputImg.channels() == 1);

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

    const int orig_h = img_h;
    const int orig_w = img_w;
    cv::Mat img_raw = inputImg;
    cv::Mat rgb_u8;
    cv::cvtColor(img_raw, rgb_u8, cv::COLOR_GRAY2RGB);
    cv::Mat rgb_f32;
    rgb_u8.convertTo(rgb_f32, CV_32F, 1.0 / 255.0);
    CV_Assert(rgb_f32.isContinuous());

    auto &session = load_session(model_path);
    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name_alloc = session.GetInputNameAllocated(0, allocator);
    std::string input_name = input_name_alloc.get();
    auto input_type_info = session.GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> expected_shape = input_tensor_info.GetShape();
    int run_h = img_h;
    int run_w = img_w;
    if (expected_shape.size() == 4) {
        const int64_t exp_h = expected_shape[2];
        const int64_t exp_w = expected_shape[3];
        if (exp_h > 0 && exp_w > 0 && (exp_h != img_h || exp_w != img_w)) {
            cv::Mat resized;
            cv::resize(
                rgb_f32, resized,
                cv::Size(static_cast<int>(exp_w), static_cast<int>(exp_h)), 0,
                0, cv::INTER_LINEAR);
            rgb_f32 = resized;
            run_h = static_cast<int>(exp_h);
            run_w = static_cast<int>(exp_w);
        }
    }

    const std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
    const std::array<float, 3> stdev{0.229F, 0.224F, 0.225F};
    std::vector<float> input_data(static_cast<size_t>(1 * 3 * run_h * run_w));
    size_t idx = 0;
    for (int ch = 0; ch < 3; ++ch) {
        for (int py = 0; py < run_h; ++py) {
            const float *row = rgb_f32.ptr<float>(py);
            for (int px = 0; px < run_w; ++px) {
                const float val = row[px * 3 + ch];
                input_data[idx++] = (val - mean[static_cast<size_t>(ch)]) /
                                    stdev[static_cast<size_t>(ch)];
            }
        }
    }

    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemTypeDefault);
    std::array<int64_t, 4> input_shape{1, 3, run_h, run_w};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem_info, input_data.data(), input_data.size(), input_shape.data(),
        input_shape.size());
    size_t num_outputs = session.GetOutputCount();
    if (num_outputs < 2) {
        throw std::runtime_error("onnx model did not have >=2 outputs");
    }
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

    if (outputs.size() < 2) {
        throw std::runtime_error(
            "ONNX inference returned insufficient outputs");
    }

    const Ort::Value &presence_val = outputs[0];
    const auto *presence_ptr = presence_val.GetTensorData<float>();
    auto presence_shape = presence_val.GetTensorTypeAndShapeInfo().GetShape();
    size_t presence_n = 1;
    for (auto dim : presence_shape) {
        presence_n *= static_cast<size_t>(dim);
    }
    float presence_logit = (presence_n >= 1) ? presence_ptr[0] : 0.0F;
    auto sigmoid = [](float xf) -> float {
        return 1.0F / (1.0F + std::exp(-xf));
    };
    float p_curve = sigmoid(presence_logit);

    const Ort::Value &curve_val = outputs[1];
    const auto *curve_ptr = curve_val.GetTensorData<float>();
    auto curve_shape = curve_val.GetTensorTypeAndShapeInfo().GetShape();
    size_t curve_len = 1;
    for (auto dim : curve_shape) {
        curve_len *= static_cast<size_t>(dim);
    }

    std::vector<cv::Point> ret_coords;
    if (p_curve >= 0.5F) {
        ret_coords.reserve(orig_w);
        const double scale_h =
            static_cast<double>(orig_h) / static_cast<double>(run_h);
        if (curve_len == static_cast<size_t>(run_w)) {
            for (int x_pt = 0; x_pt < orig_w; ++x_pt) {
                auto src_x = static_cast<size_t>(
                    std::lround((static_cast<double>(x_pt) *
                                 static_cast<double>(run_w - 1)) /
                                static_cast<double>(orig_w - 1)));
                src_x = std::min(src_x, static_cast<size_t>(run_w - 1));
                int y_clamped = std::clamp(
                    static_cast<int>(std::lround(curve_ptr[src_x] * scale_h)),
                    0, orig_h - 1);
                ret_coords.emplace_back(x_pt, y_clamped);
            }
        } else if (curve_len > 1) {
            for (int x_pt = 0; x_pt < orig_w; ++x_pt) {
                auto src_x = static_cast<size_t>(
                    std::lround((static_cast<double>(x_pt) *
                                 static_cast<double>(curve_len - 1)) /
                                static_cast<double>(orig_w - 1)));
                src_x = std::min(src_x, curve_len - 1);
                int y_clamped = std::clamp(
                    static_cast<int>(std::lround(curve_ptr[src_x] * scale_h)),
                    0, orig_h - 1);
                ret_coords.emplace_back(x_pt, y_clamped);
            }
        }
    }

    cv::Mat overlay = img_raw.clone();
    if (!ret_coords.empty()) {
        draw_line(overlay, ret_coords);
    }

    SegmentResult result;
    result.image = overlay;
    result.coordinates = std::move(ret_coords);
    return result;
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
    int num_frames = interval > 1 ? interval : 2;
    int img_w = img_array.empty() ? 1 : img_array.front().cols;
    double increments = (static_cast<double>(img_w) - 1.0) /
                        static_cast<double>(num_frames - 1);

    for (size_t i = 0; i < img_array.size(); ++i) {
        const cv::Mat &img = img_array[i];
        SegmentResult point_cloud = detect_lines(img);
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

        for (auto &coordinate : point_cloud.coordinates) {
            auto x_pts = static_cast<double>(coordinate.x);
            auto y_pts = static_cast<double>(coordinate.y);
            pc_3d.emplace_back(x_pts, z_val, y_pts);
        }
    }

    return pc_3d;
}

} // namespace octa_ros::img
