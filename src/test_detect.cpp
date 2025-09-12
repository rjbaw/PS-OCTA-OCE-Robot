#include "process_img.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>

static bool has_image_ext(const std::filesystem::path &path) {
    auto ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" ||
           ext == ".tif" || ext == ".tiff";
}

int main(int argc, char **argv) {
    try {
        if (argc < 2) {
            std::cerr
                << "Usage: " << argv[0] << " <input_path> [output_path]\n"
                << "- input_path: image file or directory (recurses)\n"
                << "- output_path: output image or directory (mirrors tree)\n";
            return 1;
        }

        std::filesystem::path in_path(argv[1]);
        std::filesystem::path out_path =
            (argc > 2) ? std::filesystem::path(argv[2])
                       : std::filesystem::path("result.jpg");

        if (std::filesystem::is_directory(in_path)) {
            // Directory mode: out_path must be a directory
            std::filesystem::create_directories(out_path);
            std::vector<std::filesystem::path> files;
            for (const auto &entry :
                 std::filesystem::recursive_directory_iterator(in_path)) {
                if (!entry.is_regular_file()) {
                    continue;
                }
                if (!has_image_ext(entry.path())) {
                    continue;
                }
                files.push_back(entry.path());
            }
            std::sort(files.begin(), files.end());
            if (files.empty()) {
                std::cerr << "No images under: " << in_path << "\n";
                return 1;
            }
            const auto in_root = std::filesystem::absolute(in_path);
            size_t idx_file = 0;
            for (const auto &file_path : files) {
                cv::Mat img =
                    cv::imread(file_path.string(), cv::IMREAD_GRAYSCALE);
                if (img.empty()) {
                    std::cerr << "Skip (cannot open): " << file_path << "\n";
                    continue;
                }
                auto rel = std::filesystem::relative(file_path, in_root);
                auto dst = out_path / rel;
                octa_ros::img::SegmentResult res =
                    octa_ros::img::detect_lines(img);
                std::filesystem::create_directories(dst.parent_path());
                if (!cv::imwrite(dst.string(), res.image)) {
                    std::cerr << "Failed to write: " << dst << "\n";
                } else {
                    std::cout << "Saved: " << dst << "\n";
                }
                if (!res.coordinates.empty()) {
                    const int model_w = 500;
                    std::vector<float> y_vals;
                    y_vals.reserve(model_w);
                    const int width = img.cols;
                    for (int x_idx = 0; x_idx < model_w; ++x_idx) {
                        auto src_x = static_cast<size_t>(
                            std::lround((static_cast<double>(x_idx) *
                                         static_cast<double>(width - 1)) /
                                        static_cast<double>(model_w - 1)));
                        if (src_x >= res.coordinates.size()) {
                            src_x = res.coordinates.size() - 1;
                        }
                        y_vals.push_back(
                            static_cast<float>(res.coordinates[src_x].y));
                    }
                    auto txt = dst;
                    txt.replace_extension(".txt");
                    std::ofstream ofs(txt);
                    ofs.setf(std::ios::fixed);
                    ofs << std::setprecision(1);
                    for (const float value : y_vals) {
                        ofs << value << '\n';
                    }
                }
                ++idx_file;
            }
            return 0;
        }

        // Single-image mode
        cv::Mat inputImage = cv::imread(in_path.string(), cv::IMREAD_GRAYSCALE);
        if (inputImage.empty()) {
            std::cerr << "Cannot open " << in_path << "\n";
            return 1;
        }
        octa_ros::img::SegmentResult result =
            octa_ros::img::detect_lines(inputImage);
        std::filesystem::path dst = out_path;
        if (std::filesystem::is_directory(out_path) ||
            out_path.extension().empty()) {
            std::filesystem::create_directories(out_path);
            dst = out_path / in_path.filename();
            dst.replace_extension(".jpg");
        } else if (!dst.parent_path().empty()) {
            std::filesystem::create_directories(dst.parent_path());
        }
        if (!cv::imwrite(dst.string(), result.image)) {
            std::cerr << "Could not save result to " << dst << "\n";
            return 1;
        }
        std::cout << "Saved result to \"" << dst << "\"\n";
        if (!result.coordinates.empty()) {
            const int model_w = 500;
            std::vector<float> y_vals;
            y_vals.reserve(model_w);
            const int width = inputImage.cols;
            for (int x_idx = 0; x_idx < model_w; ++x_idx) {
                auto src_x = static_cast<size_t>(
                    std::lround((static_cast<double>(x_idx) *
                                 static_cast<double>(width - 1)) /
                                static_cast<double>(model_w - 1)));
                if (src_x >= result.coordinates.size()) {
                    src_x = result.coordinates.size() - 1;
                }
                y_vals.push_back(
                    static_cast<float>(result.coordinates[src_x].y));
            }
            auto txt = dst;
            txt.replace_extension(".txt");
            std::ofstream ofs(txt);
            ofs.setf(std::ios::fixed);
            ofs << std::setprecision(1);
            for (const float value : y_vals) {
                ofs << value << '\n';
            }
        }
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[test_detect] Exception: " << e.what() << "\n";
        return 3;
    } catch (...) {
        std::cerr << "[test_detect] Unknown exception\n";
        return 4;
    }
}
