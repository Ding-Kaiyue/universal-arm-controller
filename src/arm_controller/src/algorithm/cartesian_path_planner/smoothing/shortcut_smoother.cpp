#include "algorithm/cartesian_path_planner/smoothing/shortcut_smoother.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::algorithm::cartesian_path_planner {

ShortcutSmoother::ShortcutSmoother(
    const SmoothingConfig& cfg, 
    std::shared_ptr<const DistanceFieldInterface> distance_field)
    : cfg_(cfg), collision_checker_(std::move(distance_field)) {}

CartesianPath ShortcutSmoother::smooth(
    const CartesianPath& raw_path,
    const PathPlanningInput& input) {
    CartesianPath out = raw_path;
    if (out.waypoints.size() < 3) {
        return out;  // Not enough waypoints to smooth
    }
    std::mt19937 rng(42);  // Fixed seed for reproducibility
    for (int trial = 0; trial < cfg_.max_shortcut_trials; ++trial) {
        if (out.waypoints.size() < 3) {
            break;  // No more shortcuts possible
        }

        std::uniform_int_distribution<size_t> dist(0, out.waypoints.size() - 1);
        size_t i = dist(rng);
        size_t j = dist(rng);
        if (i == j) {
            continue;
        }
        if (i > j) {
            std::swap(i, j);
        }
        if (j <= i + 1) {
            continue;
        }
        const auto& p0 = out.waypoints[i].position;
        const auto& p1 = out.waypoints[j].position;

        if (!collision_checker_.isSegmentValid(
                p0, p1, input.hard_clearance, cfg_.collision_check_step)) {
            continue;
        }
        std::vector<CartesianWaypoint> new_wps;
        new_wps.reserve(out.waypoints.size() - (j - i - 1));
        new_wps.insert(new_wps.end(), out.waypoints.begin(), out.waypoints.begin() + i + 1);
        new_wps.insert(new_wps.end(), out.waypoints.begin() + j, out.waypoints.end());
        out.waypoints = std::move(new_wps);
    }

    // Local point-relaxation smoothing:
    // move inner points toward midpoints of neighbors while preserving clearance.
    const int local_iters = std::max(0, cfg_.local_adjust_iterations);
    const double alpha = std::clamp(cfg_.local_adjust_alpha, 0.0, 1.0);
    for (int iter = 0; iter < local_iters; ++iter) {
        if (out.waypoints.size() < 3 || alpha <= 0.0) {
            break;
        }
        bool changed = false;
        for (size_t i = 1; i + 1 < out.waypoints.size(); ++i) {
            const Eigen::Vector3d& p_prev = out.waypoints[i - 1].position;
            const Eigen::Vector3d& p_cur = out.waypoints[i].position;
            const Eigen::Vector3d& p_next = out.waypoints[i + 1].position;

            const Eigen::Vector3d p_mid = 0.5 * (p_prev + p_next);
            const Eigen::Vector3d p_new = p_cur + alpha * (p_mid - p_cur);

            if (!collision_checker_.isStateValid(
                    p_new, input.hard_clearance)) {
                continue;
            }
            if (!collision_checker_.isSegmentValid(
                    p_prev,
                    p_new,
                    input.hard_clearance,
                    cfg_.collision_check_step)) {
                continue;
            }
            if (!collision_checker_.isSegmentValid(
                    p_new,
                    p_next,
                    input.hard_clearance,
                    cfg_.collision_check_step)) {
                continue;
            }

            CartesianWaypoint candidate = out.waypoints[i];
            candidate.position = p_new;
            out.waypoints[i] = candidate;
            changed = true;
        }
        if (!changed) {
            break;
        }
    }

    out.length = 0.0;
    for (size_t k = 1; k < out.waypoints.size(); ++k) {
        out.length += (out.waypoints[k].position - out.waypoints[k - 1].position).norm();
    }
    return out;    
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
