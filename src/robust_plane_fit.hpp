#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace octa_ros::img {

using Points3d = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

struct RobustPlaneOptions {
    int n_init = 192;
    int subset_size = 4;
    int top_k = 8;
    int max_iter = 50;
    double tuning = 4.685;
    double tol_angle_deg = 5e-4;
    double tol_d = 1e-8;
    double low_weight_threshold = 0.10;
    double zero_weight_eps = 1e-3;
    std::uint32_t seed = 0;

    // Sign convention for the fitted normal.
    Eigen::Vector3d up_dir = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d sweep_direction = Eigen::Vector3d::UnitY();
};

struct PlaneInitCandidate {
    std::string tag;
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double d = 0.0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double score_median_abs = std::numeric_limits<double>::infinity();
};

struct StripDiagnostics {
    std::size_t index = 0;
    double median_abs_residual = 0.0;
    double p90_abs_residual = 0.0;
    double p95_abs_residual = 0.0;
    double mean_weight = 0.0;
    double low_weight_frac = 0.0;
    double zero_weight_frac = 0.0;
};

struct RobustPlaneFitResult {
    bool ok = false;

    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double d = 0.0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();

    Eigen::VectorXd residuals;
    Eigen::VectorXd weights;
    double scale = 0.0;
    double objective = std::numeric_limits<double>::infinity();
    int iterations = 0;

    std::string init_tag;
    double init_score_median_abs = std::numeric_limits<double>::infinity();
    std::vector<PlaneInitCandidate> init_candidates;

    // Right-handed frame [projected sweep_direction, lateral axis, normal].
    Eigen::Vector3d sweep_axis = Eigen::Vector3d::UnitY();
    Eigen::Vector3d lateral_axis = Eigen::Vector3d::UnitX();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

    std::vector<StripDiagnostics> strip_diagnostics;
};

namespace detail {

constexpr double kPi = std::numbers::pi_v<double>;

inline double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

inline Eigen::Vector3d safe_unit(const Eigen::Vector3d &v,
                                 const Eigen::Vector3d &fallback) {
    const double n = v.norm();
    if (n < 1e-12) {
        return fallback.normalized();
    }
    return v / n;
}

inline Points3d stack_strips_impl(const std::vector<Points3d> &strips,
                                  std::vector<int> *strip_sizes = nullptr) {
    Eigen::Index total_rows = 0;
    for (const auto &s : strips) {
        total_rows += s.rows();
    }
    Points3d out(total_rows, 3);
    Eigen::Index start = 0;
    if (strip_sizes != nullptr) {
        strip_sizes->clear();
        strip_sizes->reserve(strips.size());
    }
    for (const auto &s : strips) {
        if (strip_sizes != nullptr) {
            strip_sizes->push_back(static_cast<int>(s.rows()));
        }
        if (s.rows() > 0) {
            out.block(start, 0, s.rows(), 3) = s;
            start += s.rows();
        }
    }
    return out;
}

inline double quantile_sorted(const std::vector<double> &sorted_values,
                              double q) {
    if (sorted_values.empty()) {
        return 0.0;
    }
    const double qq = clamp01(q);
    const double pos = qq * static_cast<double>(sorted_values.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(pos));
    const auto hi = static_cast<std::size_t>(std::ceil(pos));
    const double alpha = pos - static_cast<double>(lo);
    return (1.0 - alpha) * sorted_values[lo] + alpha * sorted_values[hi];
}

inline double quantile(std::vector<double> values, double q) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    return quantile_sorted(values, q);
}

inline double median(std::vector<double> values) {
    return quantile(std::move(values), 0.5);
}

inline std::vector<double> to_std_vector(const Eigen::VectorXd &v) {
    std::vector<double> out(static_cast<std::size_t>(v.size()));
    for (Eigen::Index i = 0; i < v.size(); ++i) {
        out[static_cast<std::size_t>(i)] = v(i);
    }
    return out;
}

inline double median_vec(const Eigen::VectorXd &v) {
    return median(to_std_vector(v));
}

inline double quantile_vec(const Eigen::VectorXd &v, double q) {
    return quantile(to_std_vector(v), q);
}

inline double angle_between_deg(const Eigen::Vector3d &a,
                                const Eigen::Vector3d &b) {
    const Eigen::Vector3d aa = safe_unit(a, Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d bb = safe_unit(b, Eigen::Vector3d::UnitZ());
    const double c = std::clamp(std::abs(aa.dot(bb)), 0.0, 1.0);
    return std::acos(c) * 180.0 / kPi;
}

inline void orient_plane(Eigen::Vector3d &n, double &d,
                         const Eigen::Vector3d &up_dir) {
    Eigen::Vector3d up = safe_unit(up_dir, Eigen::Vector3d::UnitZ());
    n = safe_unit(n, Eigen::Vector3d::UnitZ());
    if (n.dot(up) < 0.0) {
        n = -n;
        d = -d;
    }
}

inline void fit_plane_svd(const Points3d &points,
                          const Eigen::VectorXd *weights,
                          const Eigen::Vector3d &up_dir, Eigen::Vector3d &n,
                          double &d, Eigen::Vector3d &center) {
    if (points.rows() < 3) {
        throw std::invalid_argument(
            "Need at least 3 points for plane fitting.");
    }

    if (weights == nullptr) {
        center = points.colwise().mean().transpose();
        const Points3d q = points.rowwise() - center.transpose();
        const Eigen::Matrix3d scatter =
            (q.transpose() * q) / static_cast<double>(points.rows());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(scatter);
        if (es.info() != Eigen::Success) {
            throw std::runtime_error(
                "Eigen decomposition failed in plane fit.");
        }
        n = es.eigenvectors().col(0);
    } else {
        if (weights->size() != points.rows()) {
            throw std::invalid_argument(
                "Weight vector length does not match point count.");
        }
        const Eigen::VectorXd w = weights->cwiseMax(0.0);
        const double sw = w.sum();
        if (sw <= 1e-12) {
            throw std::invalid_argument(
                "Weighted plane fit received near-zero total weight.");
        }
        center = (points.transpose() * w) / sw;
        const Points3d q = points.rowwise() - center.transpose();
        const Points3d qw = q.array().colwise() * w.array();
        const Eigen::Matrix3d scatter = (q.transpose() * qw) / sw;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(scatter);
        if (es.info() != Eigen::Success) {
            throw std::runtime_error(
                "Eigen decomposition failed in weighted plane fit.");
        }
        n = es.eigenvectors().col(0);
    }

    n = safe_unit(n, Eigen::Vector3d::UnitZ());
    d = -n.dot(center);
    orient_plane(n, d, up_dir);
}

inline Eigen::VectorXd plane_residuals(const Points3d &points,
                                       const Eigen::Vector3d &n, double d) {
    return points * n + Eigen::VectorXd::Constant(points.rows(), d);
}

inline double mad_scale(const Eigen::VectorXd &residuals) {
    const double med = median_vec(residuals);
    const Eigen::VectorXd abs_dev = (residuals.array() - med).abs();
    double s = 1.4826 * median_vec(abs_dev);
    if (!std::isfinite(s) || s < 1e-9) {
        s = median_vec(residuals.cwiseAbs());
    }
    return std::max(s, 1e-9);
}

inline Eigen::VectorXd tukey_weights(const Eigen::VectorXd &residuals,
                                     double scale, double tuning) {
    const double denom = tuning * scale + 1e-12;
    Eigen::VectorXd out = Eigen::VectorXd::Zero(residuals.size());
    for (Eigen::Index i = 0; i < residuals.size(); ++i) {
        const double u = residuals(i) / denom;
        if (std::abs(u) < 1.0) {
            const double t = 1.0 - u * u;
            out(i) = t * t;
        }
    }
    return out;
}

inline double tukey_rho_sum(const Eigen::VectorXd &residuals, double scale,
                            double tuning) {
    const double denom = tuning * scale + 1e-12;
    const double cap = (tuning * tuning) / 6.0;
    double sum = 0.0;
    for (Eigen::Index i = 0; i < residuals.size(); ++i) {
        const double u = std::abs(residuals(i)) / denom;
        if (u < 1.0) {
            const double t = 1.0 - u * u;
            sum += cap * (1.0 - t * t * t);
        } else {
            sum += cap;
        }
    }
    return sum;
}

inline int count_nonzero_weights(const Eigen::VectorXd &w, double eps = 0.0) {
    int count = 0;
    for (Eigen::Index i = 0; i < w.size(); ++i) {
        if (w(i) > eps) {
            ++count;
        }
    }
    return count;
}

inline double global_extent(const Points3d &points) {
    if (points.rows() == 0) {
        return 0.0;
    }
    const Eigen::RowVector3d pmin = points.colwise().minCoeff();
    const Eigen::RowVector3d pmax = points.colwise().maxCoeff();
    return (pmax - pmin).norm();
}

inline Points3d gather_rows(const Points3d &points,
                            const std::vector<int> &idx) {
    Points3d out(static_cast<Eigen::Index>(idx.size()), 3);
    for (std::size_t i = 0; i < idx.size(); ++i) {
        out.row(static_cast<Eigen::Index>(i)) = points.row(idx[i]);
    }
    return out;
}

inline std::vector<int> sample_unique_indices(int n_total, int subset_size,
                                              std::mt19937 &rng) {
    subset_size = std::min(subset_size, n_total);
    std::unordered_set<int> picked;
    picked.reserve(static_cast<std::size_t>(subset_size));
    std::uniform_int_distribution<int> dist(0, n_total - 1);
    while (static_cast<int>(picked.size()) < subset_size) {
        picked.insert(dist(rng));
    }
    return {picked.begin(), picked.end()};
}

inline std::vector<PlaneInitCandidate>
init_candidates(const Points3d &points, const RobustPlaneOptions &opts) {
    std::vector<PlaneInitCandidate> out;
    if (points.rows() < 3) {
        return out;
    }

    PlaneInitCandidate all;
    fit_plane_svd(points, nullptr, opts.up_dir, all.normal, all.d, all.center);
    all.score_median_abs =
        median_vec(plane_residuals(points, all.normal, all.d).cwiseAbs());
    all.tag = "all_point_svd";
    out.push_back(all);

    std::mt19937 rng(opts.seed);
    const int wanted = std::max(opts.n_init, 0);
    const int max_tries =
        std::max(20 * std::max(wanted, 1), std::max(wanted, 1));
    const int subset_size = std::max(opts.subset_size, 3);
    const double extent = std::max(global_extent(points), 1e-12);

    int tries = 0;
    while (static_cast<int>(out.size()) < wanted + 1 && tries < max_tries) {
        ++tries;
        const std::vector<int> idx = sample_unique_indices(
            static_cast<int>(points.rows()), subset_size, rng);
        const Points3d subset = gather_rows(points, idx);
        const Eigen::Vector3d c = subset.colwise().mean().transpose();
        const Points3d q = subset.rowwise() - c.transpose();

        Eigen::JacobiSVD<Points3d> svd(q);
        const Eigen::VectorXd s = svd.singularValues();
        if (s.size() < 2 || s(1) < 1e-4 * extent) {
            continue;
        }

        PlaneInitCandidate cand;
        fit_plane_svd(subset, nullptr, opts.up_dir, cand.normal, cand.d,
                      cand.center);
        cand.score_median_abs =
            median_vec(plane_residuals(points, cand.normal, cand.d).cwiseAbs());
        cand.tag = "random_subset";
        out.push_back(cand);
    }

    std::sort(out.begin(), out.end(),
              [](const PlaneInitCandidate &a, const PlaneInitCandidate &b) {
                  return a.score_median_abs < b.score_median_abs;
              });
    return out;
}

inline RobustPlaneFitResult refine_from_init(const Points3d &points,
                                             const PlaneInitCandidate &init,
                                             const RobustPlaneOptions &opts) {
    RobustPlaneFitResult out;
    Eigen::Vector3d n = init.normal;
    double d = init.d;
    Eigen::Vector3d center = init.center;

    int it_done = 0;
    for (int it = 0; it < opts.max_iter; ++it) {
        ++it_done;
        const Eigen::VectorXd residuals = plane_residuals(points, n, d);
        const double scale = mad_scale(residuals);
        Eigen::VectorXd w = tukey_weights(residuals, scale, opts.tuning);

        if (count_nonzero_weights(w) < 3) {
            w = tukey_weights(residuals, scale, std::max(6.0, opts.tuning));
        }

        Eigen::Vector3d n_new;
        double d_new = 0.0;
        Eigen::Vector3d center_new;
        if (count_nonzero_weights(w) < 3) {
            fit_plane_svd(points, nullptr, opts.up_dir, n_new, d_new,
                          center_new);
        } else {
            fit_plane_svd(points, &w, opts.up_dir, n_new, d_new, center_new);
        }

        const double ang = angle_between_deg(n, n_new);
        const double dd = std::abs(d_new - d);
        n = n_new;
        d = d_new;
        center = center_new;

        if (ang < opts.tol_angle_deg && dd < opts.tol_d) {
            break;
        }
    }

    out.ok = true;
    out.normal = n;
    out.d = d;
    out.center = center;
    out.residuals = plane_residuals(points, out.normal, out.d);
    out.scale = mad_scale(out.residuals);
    out.weights = tukey_weights(out.residuals, out.scale, opts.tuning);

    if (count_nonzero_weights(out.weights) >= 3) {
        fit_plane_svd(points, &out.weights, opts.up_dir, out.normal, out.d,
                      out.center);
        out.residuals = plane_residuals(points, out.normal, out.d);
        out.scale = mad_scale(out.residuals);
        out.weights = tukey_weights(out.residuals, out.scale, opts.tuning);
    }

    out.objective = tukey_rho_sum(out.residuals, out.scale, opts.tuning);
    out.iterations = it_done;
    out.init_tag = init.tag;
    out.init_score_median_abs = init.score_median_abs;
    return out;
}

inline Eigen::Matrix3d
build_frame_impl(const Eigen::Vector3d &normal,
                 const Eigen::Vector3d &sweep_direction) {
    const Eigen::Vector3d n = safe_unit(normal, Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d prefer =
        safe_unit(sweep_direction, Eigen::Vector3d::UnitY());

    Eigen::Vector3d u = prefer - prefer.dot(n) * n;
    if (u.norm() < 1e-8) {
        const Eigen::Vector3d fallback = (std::abs(n.x()) < 0.9)
                                             ? Eigen::Vector3d::UnitX()
                                             : Eigen::Vector3d::UnitY();
        u = fallback - fallback.dot(n) * n;
    }
    u.normalize();

    Eigen::Vector3d v = n.cross(u);
    if (v.norm() < 1e-8) {
        throw std::runtime_error(
            "Failed to build plane frame from fitted normal.");
    }
    v.normalize();
    u = v.cross(n).normalized();

    if (u.dot(prefer) < 0.0) {
        u = -u;
        v = -v;
    }

    Eigen::Matrix3d R;
    R.col(0) = u;
    R.col(1) = v;
    R.col(2) = n;
    return R;
}

inline std::vector<StripDiagnostics> compute_strip_diagnostics(
    const Eigen::VectorXd &residuals, const Eigen::VectorXd &weights,
    const std::vector<int> &strip_sizes, const RobustPlaneOptions &opts) {
    std::vector<StripDiagnostics> out;
    out.reserve(strip_sizes.size());
    Eigen::Index start = 0;
    for (std::size_t i = 0; i < strip_sizes.size(); ++i) {
        const auto count = static_cast<Eigen::Index>(strip_sizes[i]);
        StripDiagnostics row;
        row.index = i;
        if (count > 0) {
            const Eigen::VectorXd r =
                residuals.segment(start, count).cwiseAbs();
            const Eigen::VectorXd w = weights.segment(start, count);
            row.median_abs_residual = median_vec(r);
            row.p90_abs_residual = quantile_vec(r, 0.90);
            row.p95_abs_residual = quantile_vec(r, 0.95);
            row.mean_weight = w.mean();
            int low = 0;
            int zero = 0;
            for (Eigen::Index k = 0; k < w.size(); ++k) {
                if (w(k) < opts.low_weight_threshold) {
                    ++low;
                }
                if (w(k) < opts.zero_weight_eps) {
                    ++zero;
                }
            }
            row.low_weight_frac =
                static_cast<double>(low) / static_cast<double>(w.size());
            row.zero_weight_frac =
                static_cast<double>(zero) / static_cast<double>(w.size());
        }
        out.push_back(row);
        start += count;
    }
    return out;
}

inline void attach_frame(RobustPlaneFitResult &fit,
                         const RobustPlaneOptions &opts) {
    fit.rotation = build_frame_impl(fit.normal, opts.sweep_direction);
    fit.sweep_axis = fit.rotation.col(0);
    fit.lateral_axis = fit.rotation.col(1);
}

} // namespace detail

inline Points3d stack_strips(const std::vector<Points3d> &strips) {
    return detail::stack_strips_impl(strips, nullptr);
}

inline Eigen::Matrix3d
build_frame_from_plane(const Eigen::Vector3d &normal,
                       const Eigen::Vector3d &sweep_direction) {
    return detail::build_frame_impl(normal, sweep_direction);
}

inline RobustPlaneFitResult
fit_robust_plane_tukey_irls(const Points3d &points,
                            const RobustPlaneOptions &opts = {}) {
    RobustPlaneFitResult out;
    if (points.rows() < 3) {
        return out;
    }

    try {
        out.init_candidates = detail::init_candidates(points, opts);
    } catch (const std::exception &) {
        return out;
    }
    if (out.init_candidates.empty()) {
        return out;
    }

    try {
        const int n_refine =
            std::max(1, std::min(opts.top_k,
                                 static_cast<int>(out.init_candidates.size())));
        bool have_best = false;
        RobustPlaneFitResult best;
        for (int i = 0; i < n_refine; ++i) {
            RobustPlaneFitResult cur = detail::refine_from_init(
                points, out.init_candidates[static_cast<std::size_t>(i)], opts);
            if (!have_best || cur.objective < best.objective) {
                best = std::move(cur);
                have_best = true;
            }
        }
        if (!have_best) {
            return out;
        }

        best.init_candidates = out.init_candidates;
        detail::attach_frame(best, opts);
        return best;
    } catch (const std::exception &) {
        return out;
    }
}

inline RobustPlaneFitResult
fit_robust_plane_tukey_irls(const std::vector<Points3d> &strips,
                            const RobustPlaneOptions &opts = {}) {
    RobustPlaneFitResult out;
    std::vector<int> strip_sizes;
    const Points3d points = detail::stack_strips_impl(strips, &strip_sizes);
    out = fit_robust_plane_tukey_irls(points, opts);
    if (out.ok) {
        out.strip_diagnostics = detail::compute_strip_diagnostics(
            out.residuals, out.weights, strip_sizes, opts);
    }
    return out;
}

} // namespace octa_ros::img
