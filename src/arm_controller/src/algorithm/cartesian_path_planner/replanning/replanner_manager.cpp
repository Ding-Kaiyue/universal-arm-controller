#include "algorithm/cartesian_path_planner/replanning/replanner_manager.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::algorithm::cartesian_path_planner {

ReplannerManager::ReplannerManager(std::shared_ptr<CartesianPathPlanner> planner)
    : planner_(std::move(planner)) {}

void ReplannerManager::setConfig(const ReplannerConfig& cfg) {
    cfg_ = cfg;
}

bool ReplannerManager::start(
    const PathPlanningInput& first_input,
    std::string* error) {
    return planSegment(first_input, error);
}

bool ReplannerManager::planSegment(
    const PathPlanningInput& input,
    std::string* error) {
    if (!planner_) {
        if (error != nullptr) {
            *error = "ReplannerManager planSegment failed: planner is null.";
        }
        return false;
    }

    if (cfg_.segment_sample_step_m <= 0.0 ||
        cfg_.replan_every_control_ticks <= 0 ||
        cfg_.control_cycle_sec <= 0.0 ||
        cfg_.prediction_horizon_ticks < 0 ||
        cfg_.planning_latency_sec < 0.0) {
        if (error != nullptr) {
            *error = "ReplannerManager planSegment failed: invalid replanning config.";
        }
        return false;
    }

    const TimedCartesianTrajectory traj = planner_->planTrajectory(input);
    if (traj.empty()) {
        if (error != nullptr) {
            *error = "ReplannerManager planSegment failed: planning returned empty trajectory.";
        }
        return false;
    }

    active_.traj = toDistanceSampledSegment(traj);
    if (active_.traj.empty()) {
        if (error != nullptr) {
            *error = "ReplannerManager planSegment failed: distance sampling conversion failed.";
        }
        return false;
    }
    started_ = true;
    return true;
}

bool ReplannerManager::planFromFeedbackPose(
    const PathPlanningInput& goal_input,
    const Eigen::Isometry3d& T_feedback,
    std::string* error) {
    PathPlanningInput in = goal_input;
    in.p_start = T_feedback.translation();
    in.R_start = T_feedback.linear();
    return planSegment(in, error);
}

bool ReplannerManager::planFromJointFeedback(
    const PathPlanningInput& goal_input,
    const std::vector<double>& q_current,
    const std::function<bool(const std::vector<double>&, Eigen::Isometry3d&)>& fk_callback,
    std::string* error) {
    if (!fk_callback) {
        if (error != nullptr) {
            *error = "planFromJointFeedback failed: fk_callback is null.";
        }
        return false;
    }
    Eigen::Isometry3d T_feedback = Eigen::Isometry3d::Identity();
    if (!fk_callback(q_current, T_feedback)) {
        if (error != nullptr) {
            *error = "planFromJointFeedback failed: fk_callback compute failed.";
        }
        return false;
    }
    return planFromFeedbackPose(goal_input, T_feedback, error);
}

bool ReplannerManager::planFromJointFeedbackProvider(
    const PathPlanningInput& goal_input,
    const std::function<std::vector<double>()>& q_provider,
    const std::function<bool(const std::vector<double>&, Eigen::Isometry3d&)>& fk_callback,
    std::string* error) {
    if (!q_provider) {
        if (error != nullptr) {
            *error = "planFromJointFeedbackProvider failed: q_provider is null.";
        }
        return false;
    }
    const std::vector<double> q_current = q_provider();
    if (q_current.empty()) {
        if (error != nullptr) {
            *error = "planFromJointFeedbackProvider failed: q_provider returned empty joint vector.";
        }
        return false;
    }
    return planFromJointFeedback(goal_input, q_current, fk_callback, error);
}

bool ReplannerManager::shouldReplanAtControlTick(const int control_tick) const {
    if (cfg_.replan_every_control_ticks <= 0) {
        return false;
    }
    if (control_tick <= 0) {
        return false;
    }
    return (control_tick % cfg_.replan_every_control_ticks) == 0;
}

bool ReplannerManager::planFromPreviousResult(
    const PathPlanningInput& next_goal_input,
    std::string* error) {
    if (!hasActiveTrajectory()) {
        if (error != nullptr) {
            *error = "planFromPreviousResult failed: active trajectory unavailable.";
        }
        return false;
    }
    const PathPlanningInput in = makeNextInputFromHandoff(next_goal_input);
    return planSegment(in, error);
}

bool ReplannerManager::planFromPredictedActiveTrajectory(
    const PathPlanningInput& goal_input,
    const int current_point_index,
    std::string* error) {
    if (!hasActiveTrajectory()) {
        if (error != nullptr) {
            *error = "planFromPredictedActiveTrajectory failed: active trajectory unavailable.";
        }
        return false;
    }
    TimedCartesianSample pred{};
    if (!samplePredictedReplanStart(current_point_index, pred)) {
        if (error != nullptr) {
            *error = "planFromPredictedActiveTrajectory failed: cannot sample predicted pose.";
        }
        return false;
    }
    const int pred_idx = predictedPointIndex(current_point_index);
    const Eigen::Vector3d v_handoff =
        estimateTrajectoryVelocityAtPoint(active_.traj, pred_idx);

    PathPlanningInput in = goal_input;
    in.p_start = pred.T_target.translation();
    in.R_start = pred.T_target.linear();
    if (!planSegment(in, error)) {
        return false;
    }
    smoothSegmentStartForHandoff(active_.traj, v_handoff);
    return true;
}

bool ReplannerManager::samplePredictedReplanStart(
    const int current_point_index,
    TimedCartesianSample& out) const {
    if (!hasActiveTrajectory()) {
        return false;
    }
    return sample(predictedPointIndex(current_point_index), out);
}

bool ReplannerManager::hasActiveTrajectory() const {
    return started_ && !active_.traj.empty();
}

int ReplannerManager::activeSegmentPointCount() const {
    if (!hasActiveTrajectory()) {
        return 0;
    }
    return static_cast<int>(active_.traj.waypoints.size());
}

PathPlanningInput ReplannerManager::makeNextInputFromHandoff(
    const PathPlanningInput& goal_input) const {
    PathPlanningInput in = goal_input;

    const TimedCartesianSample handoff =
        sampler_.sample(active_.traj, active_.traj.total_duration);
    in.p_start = handoff.T_target.translation();
    in.R_start = handoff.T_target.linear();
    return in;
}

TimedCartesianTrajectory ReplannerManager::toDistanceSampledSegment(
    const TimedCartesianTrajectory& in_traj) const {
    TimedCartesianTrajectory out;
    if (in_traj.empty()) {
        return out;
    }

    const int raw_n = static_cast<int>(in_traj.waypoints.size());
    if (raw_n < 2) {
        return in_traj;
    }

    double path_length = 0.0;
    for (int i = 1; i < raw_n; ++i) {
        path_length += (in_traj.waypoints[static_cast<std::size_t>(i)].position -
                        in_traj.waypoints[static_cast<std::size_t>(i - 1)].position)
                           .norm();
    }

    if (path_length <= 1e-9) {
        return in_traj;
    }

    const int n =
        static_cast<int>(std::ceil(path_length / cfg_.segment_sample_step_m)) + 1;
    if (n < 2) {
        return out;
    }
    const double total = std::max(1e-6, in_traj.total_duration);

    out.waypoints.reserve(static_cast<std::size_t>(n));
    out.segment_durations.reserve(static_cast<std::size_t>(n - 1));
    out.cumulative_times.reserve(static_cast<std::size_t>(n));
    out.cumulative_times.push_back(0.0);

    const double dt = total / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        const double t = std::min(total, i * dt);
        const TimedCartesianSample s = sampler_.sample(in_traj, t);
        CartesianWaypoint wp;
        wp.position = s.T_target.translation();
        wp.orientation = s.T_target.linear();
        out.waypoints.push_back(wp);
        if (i > 0) {
            out.segment_durations.push_back(dt);
            out.cumulative_times.push_back(i * dt);
        }
    }
    out.total_duration = total;
    return out;
}

bool ReplannerManager::sample(const int point_index, TimedCartesianSample& out) const {
    if (!hasActiveTrajectory()) {
        return false;
    }
    const int n = static_cast<int>(active_.traj.waypoints.size());
    if (n < 2) {
        return false;
    }
    const int idx = std::clamp(point_index, 0, n - 1);
    const double t_query =
        (static_cast<double>(idx) / static_cast<double>(n - 1)) *
        std::max(0.0, active_.traj.total_duration);
    out = sampler_.sample(active_.traj, t_query);
    return true;
}

int ReplannerManager::predictedPointIndex(const int current_point_index) const {
    if (!hasActiveTrajectory()) {
        return 0;
    }
    const int n = static_cast<int>(active_.traj.waypoints.size());
    if (n <= 1) {
        return 0;
    }

    const double control_predict_sec =
        static_cast<double>(cfg_.prediction_horizon_ticks) * cfg_.control_cycle_sec;
    const double total_predict_sec = control_predict_sec + cfg_.planning_latency_sec;

    const double total = std::max(1e-6, active_.traj.total_duration);
    const double dt_point = total / static_cast<double>(n - 1);
    const int delta_points =
        static_cast<int>(std::ceil(total_predict_sec / std::max(1e-6, dt_point)));

    return std::clamp(current_point_index + std::max(0, delta_points), 0, n - 1);
}

Eigen::Vector3d ReplannerManager::estimateTrajectoryVelocityAtPoint(
    const TimedCartesianTrajectory& traj,
    const int point_index) const {
    const int n = static_cast<int>(traj.waypoints.size());
    if (n < 2) {
        return Eigen::Vector3d::Zero();
    }
    const int i = std::clamp(point_index, 0, n - 1);
    if (i <= 0) {
        const double dt = std::max(1e-6, traj.segment_durations[0]);
        return (traj.waypoints[1].position - traj.waypoints[0].position) / dt;
    }
    if (i >= n - 1) {
        const double dt = std::max(1e-6, traj.segment_durations[n - 2]);
        return (traj.waypoints[n - 1].position - traj.waypoints[n - 2].position) / dt;
    }
    const double t_prev = traj.cumulative_times[static_cast<size_t>(i - 1)];
    const double t_next = traj.cumulative_times[static_cast<size_t>(i + 1)];
    const double dt = std::max(1e-6, t_next - t_prev);
    return (traj.waypoints[static_cast<size_t>(i + 1)].position -
            traj.waypoints[static_cast<size_t>(i - 1)].position) /
           dt;
}

void ReplannerManager::smoothSegmentStartForHandoff(
    TimedCartesianTrajectory& traj,
    const Eigen::Vector3d& v_start) const {
    const int n = static_cast<int>(traj.waypoints.size());
    if (n < 4 || cfg_.handoff_blend_points <= 1) {
        return;
    }
    const int m = std::clamp(cfg_.handoff_blend_points, 2, n - 1);
    const double t0 = traj.cumulative_times.front();
    const double tm = traj.cumulative_times[static_cast<size_t>(m)];
    const double T = std::max(1e-6, tm - t0);

    const Eigen::Vector3d p0 = traj.waypoints.front().position;  // hard anchor
    const Eigen::Vector3d pm = traj.waypoints[static_cast<size_t>(m)].position;  // preserve seam neighborhood end
    const Eigen::Vector3d vm = estimateTrajectoryVelocityAtPoint(traj, m);

    for (int i = 1; i < m; ++i) {
        const double ti = traj.cumulative_times[static_cast<size_t>(i)] - t0;
        const double s = std::clamp(ti / T, 0.0, 1.0);
        const double s2 = s * s;
        const double s3 = s2 * s;

        const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        const double h10 = s3 - 2.0 * s2 + s;
        const double h01 = -2.0 * s3 + 3.0 * s2;
        const double h11 = s3 - s2;

        traj.waypoints[static_cast<size_t>(i)].position =
            h00 * p0 + h10 * T * v_start + h01 * pm + h11 * T * vm;
    }
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
