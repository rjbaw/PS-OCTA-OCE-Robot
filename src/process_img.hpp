/**
 * @file process_img.hpp
 * @author rjbaw
 * @brief Image processing and curve detection helpers for OCT images.
 *
 * Functions operate on grayscale OCT B-scan frames. Pixel coordinates are in
 * image space (x right, y down). 3D point reconstruction returns points in
 * pixel units with components ordered as (x, z, y) for downstream geometry
 * processing.
 */

#ifndef PROCESS_IMG_HPP
#define PROCESS_IMG_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

/** @addtogroup imaging
 *  @{ */

namespace octa_ros::img {

/**
 * @brief Output of the curve detection pass.
 *
 * When a curve is detected, the overlay image contains a rendered polyline of
 * the curve and the coordinates contain one pixel per x-column. If no curve is
 * found, the coordinates vector is empty and the image is a copy of input.
 */
struct SegmentResult {
    cv::Mat image;                      ///< Associated image.
    std::vector<cv::Point> coordinates; ///< Detected (x,y) curve pixels.
};

/**
 * @brief Draw a polyline onto an image using the provided pixel coordinates.
 * @param image Image to draw into (modified in-place).
 * @param ret_coord Ordered list of pixel coordinates defining the curve.
 */
void draw_line(cv::Mat &image, const std::vector<cv::Point> &ret_coord);

/**
 * @brief Align each column vector to the closest canonical axis with positive
 * sign.
 * @param rot_matrix 3x3 rotation matrix to align.
 * @return Axis-aligned rotation matrix with non-negative diagonal entries.
 */
Eigen::Matrix3d align_to_direction(const Eigen::Matrix3d &rot_matrix);

/**
 * @brief Detect a 2D curve in a grayscale OCT image using an ONNX model.
 *
 * Performs color conversion, normalization, model resizing if needed, and runs
 * the detection network. Returns the overlay image and the per-column pixel
 * coordinates for the detected curve, if present.
 *
 * @param inputImg Single-channel uint8 image. If 3-channel (BGR), it is
 * converted to grayscale.
 * @return SegmentResult with overlay and coordinates (possibly empty).
 * @note Model path resolution: if `set_curve_model_path()` is not called, the
 *       ONNX model is loaded from the package share path
 * `config/curve_model.onnx`.
 */
SegmentResult detect_lines(const cv::Mat &inputImg);

/**
 * @brief Reconstruct a sparse 3D point set from a sequence of detected curves.
 *
 * For a sequence of 2D frames, picks per-column points from each frame and
 * assigns the Z coordinate based on the frame index modulo @p interval.
 * If environment variable `OCTA_SAVE_DEBUG` is set (and not 0/false), raw and
 * overlay frames are saved under `result/` in a timestamped subdirectory.
 *
 * @param img_array Sequence of frames (same size).
 * @param interval Number of frames per Z period (>=2 recommended).
 * @return Vector of 3D points (x,z,y) in pixels.
 */
std::vector<Eigen::Vector3d> lines_3d(const std::vector<cv::Mat> &img_array,
                                      int interval);

/**
 * @brief Preload the ONNX curve model and create an inference session.
 *
 * This forces initialization of the ONNX Runtime environment and session so
 * that subsequent calls to @ref detect_lines() / @ref lines_3d() do not incur
 * one-time model loading latency.
 *
 * @param batch_size Expected batch size for typical inference calls.
 */
void preload_curve_model(std::size_t batch_size = 1);

/**
 * @brief Warm up the ONNX curve model with a dummy inference.
 *
 * Runs a single inference pass using the provided example frame to trigger
 * any one-time runtime initialization (e.g., kernel compilation).
 *
 * @param example_frame Example frame matching the expected input size.
 * @param batch_size Batch size to warm (should match typical inference).
 */
void warmup_curve_model(const cv::Mat &example_frame,
                        std::size_t batch_size = 1);

/**
 * @brief Override the default ONNX model path used for curve detection.
 *
 * If not set, the model is resolved from the package share directory under
 * `config/curve_model.onnx`.
 *
 * @param path Filesystem path to the ONNX model.
 */
void set_curve_model_path(const std::string &path);

} // namespace octa_ros::img

/** @} */

#endif // PROCESS_IMG_HPP
