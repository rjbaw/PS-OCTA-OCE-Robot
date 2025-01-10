#ifndef PROCESS_IMG_HPP
#define PROCESS_IMG_HPP

#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>

struct SegmentResult {
    cv::Mat image;
    std::vector<cv::Point> coordinates;
};

void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord);

std::vector<double> kalmanFilter1D(const std::vector<double> &observations,
                                   double Q = 0.01, double R = 0.5);

void removeOutliers(std::vector<double> &vals, double z_threshold = 0.5);

Eigen::Matrix3d align_to_direction(const Eigen::Matrix3d &rot_matrix);

std::vector<cv::Point> get_max_coor(const cv::Mat &img);

SegmentResult detect_lines(const cv::Mat &inputImg);

std::vector<Eigen::Vector3d> lines_3d(const std::vector<cv::Mat> &img_array,
                                      const int interval,
                                      const bool acq_interval = false);

#endif // PROCESS_IMG_HPP
