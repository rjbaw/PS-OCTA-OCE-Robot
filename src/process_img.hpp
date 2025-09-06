/**
 * @file process_img.hpp
 * @brief Image processing functions
 */

#ifndef PROCESS_IMG_HPP
#define PROCESS_IMG_HPP

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <c10/core/TensorOptions.h>
#include <torch/script.h>
#include <torch/torch.h>
#include <torch/types.h>

#if __has_include(                                                             \
    <c10/xpu/XPUFunctions.h>) && __has_include(<c10/xpu/impl/xpu_cmake_macros.h>)
#include <c10/xpu/XPUFunctions.h>
#ifndef HAS_XPU
#define HAS_XPU 1
#endif
#else
#ifndef HAS_XPU
#define HAS_XPU 0
#endif
#endif

struct SegmentResult {
    cv::Mat image;
    std::vector<cv::Point> coordinates;
};

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord);

Eigen::Matrix3d align_to_direction(const Eigen::Matrix3d &rot_matrix);

std::vector<cv::Point> get_max_coor(const cv::Mat &img);

SegmentResult detect_lines(const cv::Mat &inputImg);

std::vector<Eigen::Vector3d> lines_3d(const std::vector<cv::Mat> &img_array,
                                      int interval);

#endif // PROCESS_IMG_HPP
