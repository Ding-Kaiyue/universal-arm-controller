#include "algorithm/cartesian_path_planner/core/trajectory_parameterizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "trajectory_interpolator/config.hpp"
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include "trajectory_interpolator/trajectory_interpolator.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

namespace {

constexpr double kTimeEpsilon = 1e-8;

bool nearlyEqualTime(const double a, const double b) {
    return std::abs(a - b) < kTimeEpsilon;
}

}  // namespace

TrajectoryParameterizer::TrajectoryParameterizer(
    const PlannerCommonConfig& cfg,
    std::shared_ptr<const DistanceFieldInterface> distance_field)
    : cfg_(cfg), distance_field_(std::move(distance_field)) {}

TimedCartesianTrajectory TrajectoryParameterizer::parameterize(
    const CartesianPath& path,
    const Eigen::Matrix3d& R_start,
    const Eigen::Matrix3d& R_goal,
    const double hard_clearance) const {
    TimedCartesianTrajectory traj;
    if (path.waypoints.size() < 2) {
        return traj;
    }

    traj.waypoints = path.waypoints;
    traj.segment_durations.reserve(traj.waypoints.size() - 1);
    traj.cumulative_times.reserve(traj.waypoints.size());
    traj.cumulative_times.push_back(0.0);

    double t_sum = 0.0;
    for (size_t i = 0; i + 1 < traj.waypoints.size(); ++i) {
        const double len =
            (traj.waypoints[i + 1].position - traj.waypoints[i].position).norm();
        const double dt = std::max(1e-3, len / cfg_.default_segment_speed);
        traj.segment_durations.push_back(dt);
        t_sum += dt;
        traj.cumulative_times.push_back(t_sum);
    }
    traj.total_duration = t_sum;
    traj.waypoints.front().orientation = R_start;
    traj.waypoints.back().orientation = R_goal;

    if (cfg_.enable_minimum_snap_optimization) {
        if (const auto snap_traj = tryMinimumSnapOptimization(traj, traj, hard_clearance)) {
            return *snap_traj;
        }
        if (cfg_.enable_interpolator_smoothing) {
            return smoothWithInterpolator(traj, hard_clearance);
        }
        return traj;
    }
    if (cfg_.enable_interpolator_smoothing) {
        return smoothWithInterpolator(traj, hard_clearance);
    }
    return traj;
}

TimedCartesianTrajectory TrajectoryParameterizer::smoothWithInterpolator(
    const TimedCartesianTrajectory& in_traj,
    const double hard_clearance) const {
    TimedCartesianTrajectory out = in_traj;
    if (in_traj.waypoints.size() < 3 || in_traj.total_duration <= 1e-9) {
        return out;
    }

    trajectory_interpolator::Trajectory interp_in;
    interp_in.joint_names = {"x", "y", "z"};
    interp_in.points.reserve(in_traj.waypoints.size());

    for (size_t i = 0; i < in_traj.waypoints.size(); ++i) {
        trajectory_interpolator::TrajectoryPoint p;
        p.time_from_start = in_traj.cumulative_times[i];
        p.positions = {
            in_traj.waypoints[i].position.x(),
            in_traj.waypoints[i].position.y(),
            in_traj.waypoints[i].position.z()};
        p.velocities = {0.0, 0.0, 0.0};
        p.accelerations = {0.0, 0.0, 0.0};

        if (i + 1 < in_traj.waypoints.size()) {
            const double dt = std::max(1e-6, in_traj.segment_durations[i]);
            const Eigen::Vector3d v =
                (in_traj.waypoints[i + 1].position - in_traj.waypoints[i].position) / dt;
            p.velocities = {v.x(), v.y(), v.z()};
        } else if (i > 0) {
            const double dt = std::max(1e-6, in_traj.segment_durations[i - 1]);
            const Eigen::Vector3d v =
                (in_traj.waypoints[i].position - in_traj.waypoints[i - 1].position) / dt;
            p.velocities = {v.x(), v.y(), v.z()};
        }
        interp_in.points.push_back(std::move(p));
    }

    TrajectoryInterpolator interpolator;
    trajectory_interpolator::SplineConfig interp_cfg;
    interp_cfg.target_dt = std::max(1e-3, cfg_.interpolator_target_dt);
    interp_cfg.spline_type =
        (cfg_.interpolator_continuity_order <= 1)
            ? trajectory_interpolator::SplineConfig::SplineType::CUBIC_HERMITE
            : trajectory_interpolator::SplineConfig::SplineType::CUBIC_SPLINE;
    interp_cfg.left_boundary =
        trajectory_interpolator::SplineConfig::BoundaryType::FIRST_DERIVATIVE;
    interp_cfg.right_boundary =
        trajectory_interpolator::SplineConfig::BoundaryType::FIRST_DERIVATIVE;
    interpolator.setInterpolationConfig(interp_cfg);

    if (!interpolator.loadTrajectory(interp_in)) {
        return out;
    }

    const trajectory_interpolator::Trajectory interp_out = interpolator.interpolate();
    if (interp_out.points.size() < 2) {
        return out;
    }

    std::vector<double> sample_times;
    sample_times.reserve(
        static_cast<size_t>(std::ceil(in_traj.total_duration / interp_cfg.target_dt)) +
        in_traj.cumulative_times.size() + 2);
    sample_times.push_back(0.0);
    for (double t = interp_cfg.target_dt; t < in_traj.total_duration; t += interp_cfg.target_dt) {
        sample_times.push_back(t);
    }
    sample_times.push_back(in_traj.total_duration);
    for (double t : in_traj.cumulative_times) {
        sample_times.push_back(std::clamp(t, 0.0, in_traj.total_duration));
    }
    std::sort(sample_times.begin(), sample_times.end());
    sample_times.erase(
        std::unique(sample_times.begin(), sample_times.end(),
                    [](const double a, const double b) { return std::abs(a - b) < 1e-9; }),
        sample_times.end());

    TimedCartesianTrajectory candidate;
    candidate.waypoints.reserve(sample_times.size());
    candidate.cumulative_times.reserve(sample_times.size());
    candidate.segment_durations.reserve(sample_times.size() - 1);

    size_t anchor_idx = 0;
    for (size_t i = 0; i < sample_times.size(); ++i) {
        const double t = sample_times[i];
        CartesianWaypoint wp;
        const bool use_anchor =
            (anchor_idx < in_traj.cumulative_times.size() &&
             std::abs(t - in_traj.cumulative_times[anchor_idx]) < 1e-8);
        if (use_anchor) {
            wp.position = in_traj.waypoints[anchor_idx].position;
            wp.orientation = in_traj.waypoints[anchor_idx].orientation;
            ++anchor_idx;
        } else {
            const trajectory_interpolator::TrajectoryPoint p =
                interpolator.getTrajectoryPointAtTime(t);
            if (p.positions.size() < 3) {
                return out;
            }
            wp.position = Eigen::Vector3d(
                p.positions[0],
                p.positions[1],
                p.positions[2]);

            // Keep 6D A* anchor orientations, and only interpolate orientation
            // for newly inserted samples between neighboring anchors.
            const size_t prev_anchor =
                (anchor_idx == 0) ? 0 : (anchor_idx - 1);
            const size_t next_anchor =
                std::min(anchor_idx, in_traj.cumulative_times.size() - 1);
            const double t0 = in_traj.cumulative_times[prev_anchor];
            const double t1 = in_traj.cumulative_times[next_anchor];
            const double s =
                (t1 > t0 + 1e-9) ? std::clamp((t - t0) / (t1 - t0), 0.0, 1.0) : 0.0;

            Eigen::Quaterniond q0(in_traj.waypoints[prev_anchor].orientation);
            Eigen::Quaterniond q1(in_traj.waypoints[next_anchor].orientation);
            q0.normalize();
            q1.normalize();
            wp.orientation = q0.slerp(s, q1).toRotationMatrix();
        }
        candidate.waypoints.push_back(wp);
        candidate.cumulative_times.push_back(t);
        if (i > 0) {
            candidate.segment_durations.push_back(std::max(
                1e-6,
                sample_times[i] - sample_times[i - 1]));
        }
    }

    if (candidate.cumulative_times.front() != 0.0) {
        const double t0 = candidate.cumulative_times.front();
        for (double& t : candidate.cumulative_times) {
            t -= t0;
        }
    }
    candidate.total_duration = std::max(0.0, candidate.cumulative_times.back());

    if (!isTrajectoryCollisionFree(candidate, hard_clearance)) {
        return out;
    }
    return candidate;
}

std::optional<TimedCartesianTrajectory> TrajectoryParameterizer::tryMinimumSnapOptimization(
    const TimedCartesianTrajectory& in_traj,
    const TimedCartesianTrajectory& anchor_traj,
    const double hard_clearance) const {
    if (in_traj.waypoints.size() < 5 || anchor_traj.waypoints.size() < 2) {
        return std::nullopt;
    }
    if (cfg_.minimum_snap_iterations <= 0 || cfg_.minimum_snap_weight <= 0.0) {
        return std::nullopt;
    }

    TimedCartesianTrajectory candidate = buildResampledTrajectory(in_traj, anchor_traj);
    if (candidate.waypoints.size() < 5) {
        return std::nullopt;
    }

    std::vector<Eigen::Vector3d> reference_positions;
    reference_positions.reserve(candidate.waypoints.size());
    std::vector<bool> anchor_mask(candidate.waypoints.size(), false);
    for (size_t i = 0; i < candidate.waypoints.size(); ++i) {
        reference_positions.push_back(candidate.waypoints[i].position);
        for (double anchor_time : anchor_traj.cumulative_times) {
            if (nearlyEqualTime(candidate.cumulative_times[i], anchor_time)) {
                anchor_mask[i] = true;
                break;
            }
        }
    }

    const double data_weight = std::max(0.0, cfg_.minimum_snap_data_weight);
    const double snap_weight = std::max(0.0, cfg_.minimum_snap_weight);
    const double relaxation = std::clamp(cfg_.minimum_snap_relaxation, 1e-3, 1.0);
    if (data_weight <= 0.0 && snap_weight <= 0.0) {
        return std::nullopt;
    }

    std::vector<Eigen::Vector3d> updated_positions(reference_positions.size());
    for (int iter = 0; iter < cfg_.minimum_snap_iterations; ++iter) {
        for (size_t i = 0; i < candidate.waypoints.size(); ++i) {
            if (i < 2 || i + 2 >= candidate.waypoints.size() || anchor_mask[i]) {
                updated_positions[i] = candidate.waypoints[i].position;
                continue;
            }
            // Minimize the squared fourth-order finite difference
            // p[i-2] - 4 p[i-1] + 6 p[i] - 4 p[i+1] + p[i+2].
            const Eigen::Vector3d snap_neighbors =
                candidate.waypoints[i - 2].position -
                4.0 * candidate.waypoints[i - 1].position -
                4.0 * candidate.waypoints[i + 1].position +
                candidate.waypoints[i + 2].position;
            const double denom = data_weight + 36.0 * snap_weight;
            if (denom <= 1e-12) {
                updated_positions[i] = candidate.waypoints[i].position;
                continue;
            }

            const Eigen::Vector3d snap_optimum =
                (data_weight * reference_positions[i] - 6.0 * snap_weight * snap_neighbors) / denom;
            updated_positions[i] =
                (1.0 - relaxation) * candidate.waypoints[i].position +
                relaxation * snap_optimum;
        }
        for (size_t i = 2; i + 2 < candidate.waypoints.size(); ++i) {
            if (!anchor_mask[i]) {
                candidate.waypoints[i].position = updated_positions[i];
            }
        }
    }

    if (!isTrajectoryCollisionFree(candidate, hard_clearance)) {
        return std::nullopt;
    }
    return candidate;
}

TimedCartesianTrajectory TrajectoryParameterizer::buildResampledTrajectory(
    const TimedCartesianTrajectory& source_traj,
    const TimedCartesianTrajectory& anchor_traj) const {
    TimedCartesianTrajectory candidate;
    if (source_traj.waypoints.size() < 2 || anchor_traj.waypoints.size() < 2) {
        return candidate;
    }

    const double target_dt = std::max(1e-3, cfg_.interpolator_target_dt);
    std::vector<double> sample_times;
    sample_times.reserve(
        static_cast<size_t>(std::ceil(source_traj.total_duration / target_dt)) +
        anchor_traj.cumulative_times.size() + 2);
    sample_times.push_back(0.0);
    for (double t = target_dt; t < source_traj.total_duration; t += target_dt) {
        sample_times.push_back(t);
    }
    sample_times.push_back(source_traj.total_duration);
    for (double t : anchor_traj.cumulative_times) {
        sample_times.push_back(std::clamp(t, 0.0, source_traj.total_duration));
    }
    std::sort(sample_times.begin(), sample_times.end());
    sample_times.erase(
        std::unique(
            sample_times.begin(),
            sample_times.end(),
            [](const double a, const double b) { return nearlyEqualTime(a, b); }),
        sample_times.end());

    candidate.waypoints.reserve(sample_times.size());
    candidate.cumulative_times.reserve(sample_times.size());
    candidate.segment_durations.reserve(sample_times.size() - 1);

    size_t anchor_idx = 0;
    for (size_t i = 0; i < sample_times.size(); ++i) {
        const double t = sample_times[i];
        CartesianWaypoint wp;
        const bool use_anchor =
            (anchor_idx < anchor_traj.cumulative_times.size() &&
             nearlyEqualTime(t, anchor_traj.cumulative_times[anchor_idx]));
        if (use_anchor) {
            wp = anchor_traj.waypoints[anchor_idx];
            ++anchor_idx;
        } else {
            wp = sampleWaypointAtTime(source_traj, t);
        }
        candidate.waypoints.push_back(wp);
        candidate.cumulative_times.push_back(t);
        if (i > 0) {
            candidate.segment_durations.push_back(std::max(1e-6, t - sample_times[i - 1]));
        }
    }
    candidate.total_duration = std::max(0.0, candidate.cumulative_times.back());
    return candidate;
}

CartesianWaypoint TrajectoryParameterizer::sampleWaypointAtTime(
    const TimedCartesianTrajectory& traj,
    const double time_from_start) const {
    CartesianWaypoint wp;
    if (traj.waypoints.empty()) {
        return wp;
    }
    if (time_from_start <= 0.0 || traj.waypoints.size() == 1) {
        return traj.waypoints.front();
    }
    if (time_from_start >= traj.total_duration) {
        return traj.waypoints.back();
    }

    const auto upper = std::lower_bound(
        traj.cumulative_times.begin(),
        traj.cumulative_times.end(),
        time_from_start);
    if (upper == traj.cumulative_times.begin()) {
        return traj.waypoints.front();
    }
    if (upper == traj.cumulative_times.end()) {
        return traj.waypoints.back();
    }

    const size_t next_idx = static_cast<size_t>(std::distance(traj.cumulative_times.begin(), upper));
    if (nearlyEqualTime(*upper, time_from_start)) {
        return traj.waypoints[next_idx];
    }
    const size_t prev_idx = next_idx - 1;
    const double t0 = traj.cumulative_times[prev_idx];
    const double t1 = traj.cumulative_times[next_idx];
    const double s =
        (t1 > t0 + kTimeEpsilon) ? std::clamp((time_from_start - t0) / (t1 - t0), 0.0, 1.0) : 0.0;

    wp.position =
        (1.0 - s) * traj.waypoints[prev_idx].position +
        s * traj.waypoints[next_idx].position;

    Eigen::Quaterniond q0(traj.waypoints[prev_idx].orientation);
    Eigen::Quaterniond q1(traj.waypoints[next_idx].orientation);
    q0.normalize();
    q1.normalize();
    wp.orientation = q0.slerp(s, q1).toRotationMatrix();
    return wp;
}

bool TrajectoryParameterizer::isTrajectoryCollisionFree(
    const TimedCartesianTrajectory& traj,
    const double hard_clearance) const {
    if (!distance_field_) {
        return true;
    }
    CartesianCollisionChecker checker(distance_field_);
    const double check_step = std::max(0.001, cfg_.path_resolution * 0.5);

    for (const auto& wp : traj.waypoints) {
        if (!checker.isStateValid(wp.position, hard_clearance)) {
            return false;
        }
    }
    for (size_t i = 0; i + 1 < traj.waypoints.size(); ++i) {
        if (!checker.isSegmentValid(
                traj.waypoints[i].position,
                traj.waypoints[i + 1].position,
                hard_clearance,
                check_step)) {
            return false;
        }
    }
    return true;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
