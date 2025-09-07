#include "process_img.hpp"
#include <format>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <c10/core/TensorOptions.h>
#include <torch/script.h>
#include <torch/torch.h>
#include <torch/types.h>

#if __has_include(<c10/xpu/XPUFunctions.h>) && \
    __has_include(<c10/xpu/impl/xpu_cmake_macros.h>)
#include <c10/xpu/XPUFunctions.h>
#ifndef HAS_XPU
#define HAS_XPU 1
#endif
#else
#ifndef HAS_XPU
#define HAS_XPU 0
#endif
#endif

namespace octa_ros::img {

static torch::jit::script::Module &load_model(const std::string &path,
                                              const torch::Device &device) {
    static std::unique_ptr<torch::jit::script::Module> mod_cpu;
    static std::unique_ptr<torch::jit::script::Module> mod_cuda;
    static std::unique_ptr<torch::jit::script::Module> mod_xpu;

    if (device.type() == torch::kCUDA) {
        if (!mod_cuda) {
            mod_cuda = std::make_unique<torch::jit::script::Module>(
                torch::jit::load(path));
            mod_cuda->to(device);
            mod_cuda->eval();
        }
        return *mod_cuda;
    }
    if (device.type() == torch::kXPU) {
        if (!mod_xpu) {
            mod_xpu = std::make_unique<torch::jit::script::Module>(
                torch::jit::load(path));
            mod_xpu->to(device);
            mod_xpu->eval();
        }
        return *mod_xpu;
    }

    if (!mod_cpu) {
        mod_cpu = std::make_unique<torch::jit::script::Module>(
            torch::jit::load(path));
        mod_cpu->to(device);
        mod_cpu->eval();
    }
    return *mod_cpu;
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

SegmentResult detect_lines(const cv::Mat &inputImg) {
    CV_Assert(!inputImg.empty());
    const int img_h = inputImg.rows;
    const int img_w = inputImg.cols;
    CV_Assert(img_h > 0 && img_w > 0);
    CV_Assert(inputImg.channels() == 1);

    const auto share = ament_index_cpp::get_package_share_directory("octa_ros");
    std::filesystem::path model_stdpath =
        std::filesystem::path(share) / "config/curve_model.ts";
    const std::string model_path = model_stdpath.string();

    torch::Device device = torch::kCPU;
    if (torch::cuda::is_available()) {
        device = torch::kCUDA;
    }
#if HAS_XPU
    if (torch::xpu::is_available()) {
        device = torch::kXPU;
    }
#endif

    cv::Mat img_raw = inputImg;
    cv::Mat rgb_u8;
    cv::cvtColor(img_raw, rgb_u8, cv::COLOR_GRAY2RGB);
    cv::Mat rgb_f32;
    rgb_u8.convertTo(rgb_f32, CV_32F, 1.0 / 255.0);
    CV_Assert(rgb_f32.isContinuous());

    auto img_tensor =
        torch::from_blob(rgb_f32.data, {1, img_h, img_w, 3}, torch::kFloat32)
            .permute({0, 3, 1, 2})
            .contiguous(); // (1,3,512,500)

    auto opts = torch::TensorOptions().dtype(torch::kFloat32);
    auto mean =
        torch::tensor({0.485F, 0.456F, 0.406F}, opts).view({1, 3, 1, 1});
    auto stdev =
        torch::tensor({0.229F, 0.224F, 0.225F}, opts).view({1, 3, 1, 1});
    img_tensor = (img_tensor - mean) / stdev;
    img_tensor = img_tensor.to(device);

    auto &model = load_model(model_path, device);
    model.eval();

    torch::NoGradGuard no_grad;
    c10::IValue out_iv = model.forward({img_tensor});

    auto tup = out_iv.toTuple();

    torch::Tensor presence_logits = tup->elements()[0].toTensor(); // (1,)
    torch::Tensor curve_logits = tup->elements()[1].toTensor();    // (1,500)

    auto p_curve = torch::sigmoid(presence_logits).item<float>();

    std::vector<cv::Point> ret_coords;
    if (p_curve >= 0.5F) {
        torch::Tensor y_vec = curve_logits.squeeze(0).to(torch::kFloat32);
        ret_coords.reserve(img_w);
        auto y_cpu = y_vec.to(torch::kCPU).contiguous();
        const float *y_ptr = y_cpu.data_ptr<float>();
        for (int x_pt = 0; x_pt < img_w; ++x_pt) {
            int y_clamped =
                std::clamp((int)std::lround(y_ptr[x_pt]), 0, img_h - 1);
            ret_coords.emplace_back(x_pt, y_clamped);
        }
    }

    cv::Mat overlay = img_raw;
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
            // No curve detected; skip this frame gracefully
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
