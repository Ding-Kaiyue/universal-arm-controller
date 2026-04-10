#include "algorithm/cartesian_path_planner/core/trajectory_parameterizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "trajectory_interpolator/config.hpp"
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include "trajectory_interpolator/trajectory_interpolator.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

TrajectoryParameterizer::TrajectoryParameterizer(
    const PlannerCommonConfig& cfg,
    std::shared_ptr<const DistanceFieldInterface> distance_field)
    : cfg_(cfg), distance_field_(std::move(distance_field)) {}

TimedCartesianTrajectory TrajectoryParameterizer::parameterize(
    const CartesianPath& path,
    const Eigen::Matrix3d& R_start,
    const Eigen::Matrix3d& R_goal,
    const double safe_distance) const {
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
    if (!cfg_.enable_interpolator_smoothing) {
        return traj;
    }
    return smoothWithInterpolator(traj, safe_distance);
}

TimedCartesianTrajectory TrajectoryParameterizer::smoothWithInterpolator(
    const TimedCartesianTrajectory& in_traj,
    const double safe_distance) const {
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

    if (!isTrajectoryCollisionFree(candidate, safe_distance)) {
        return out;
    }
    return candidate;
}

bool TrajectoryParameterizer::isTrajectoryCollisionFree(
    const TimedCartesianTrajectory& traj,
    const double safe_distance) const {
    if (!distance_field_) {
        return true;
    }
    CartesianCollisionChecker checker(distance_field_);
    const double check_step = std::max(0.001, cfg_.path_resolution * 0.5);

    for (const auto& wp : traj.waypoints) {
        if (!checker.isStateValid(wp.position, safe_distance)) {
            return false;
        }
    }
    for (size_t i = 0; i + 1 < traj.waypoints.size(); ++i) {
        if (!checker.isSegmentValid(
                traj.waypoints[i].position,
                traj.waypoints[i + 1].position,
                safe_distance,
                check_step)) {
            return false;
        }
    }
    return true;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
