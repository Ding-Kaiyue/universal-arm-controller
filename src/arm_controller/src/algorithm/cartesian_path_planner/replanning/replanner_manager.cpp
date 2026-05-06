#include "algorithm/cartesian_path_planner/replanning/replanner_manager.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace arm_controller::algorithm::cartesian_path_planner {

ReplannerManager::ReplannerManager(std::shared_ptr<CartesianPathPlanner> planner)
    : planner_(std::move(planner)) {}

void ReplannerManager::setConfig(const ReplannerConfig& cfg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
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
    std::shared_ptr<CartesianPathPlanner> planner;
    ReplannerConfig cfg;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        planner = planner_;
        cfg = cfg_;
    }

    if (!planner) {
        if (error != nullptr) {
            *error = "ReplannerManager planSegment failed: planner is null.";
        }
        return false;
    }

    if (cfg.segment_sample_step_m <= 0.0 ||
        cfg.replan_every_control_ticks <= 0 ||
        cfg.control_cycle_sec <= 0.0 ||
        cfg.prediction_horizon_ticks < 0 ||
        cfg.planning_latency_sec < 0.0) {
        if (error != nullptr) {
            *error = "ReplannerManager planSegment failed: invalid replanning config.";
        }
        return false;
    }

    const TimedCartesianTrajectory traj = planner->planTrajectory(input);
    if (traj.empty()) {
        if (error != nullptr) {
            *error = "ReplannerManager planSegment failed: planning returned empty trajectory.";
        }
        return false;
    }

    TimedCartesianTrajectory sampled = toDistanceSampledSegment(traj);
    if (sampled.empty()) {
        if (error != nullptr) {
            *error = "ReplannerManager planSegment failed: distance sampling conversion failed.";
        }
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active_.traj = std::move(sampled);
        pending_.traj = TimedCartesianTrajectory{};
        started_ = true;
    }
    return true;
}

bool ReplannerManager::planFromFeedbackPose(
    const PathPlanningInput& goal_input,
    const Eigen::Isometry3d& T_feedback,
    std::string* error) {
    PathPlanningInput in = goal_input;
    in.p_start = T_feedback.translation();
    in.R_start = T_feedback.linear();
    in.q_start_seed.reset();
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
    int replan_every = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        replan_every = cfg_.replan_every_control_ticks;
    }
    if (replan_every <= 0) {
        return false;
    }
    if (control_tick <= 0) {
        return false;
    }
    return (control_tick % replan_every) == 0;
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
    if (!preparePendingFromPredictedActiveTrajectory(goal_input, current_point_index, error)) {
        return false;
    }
    return commitPendingSegment(error);
}

bool ReplannerManager::preparePendingFromPredictedActiveTrajectory(
    const PathPlanningInput& goal_input,
    const int current_point_index,
    std::string* error) {
    const int pred_idx = predictedPointIndex(current_point_index);
    return preparePendingFromActiveTrajectoryPoint(goal_input, pred_idx, error);
}

bool ReplannerManager::preparePendingFromActiveTrajectoryPoint(
    const PathPlanningInput& goal_input,
    const int start_point_index,
    std::string* error) {
    TimedCartesianTrajectory active_traj;
    std::shared_ptr<CartesianPathPlanner> planner;
    ReplannerConfig cfg;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_ || active_.traj.empty()) {
            if (error != nullptr) {
                *error = "preparePendingFromActiveTrajectoryPoint failed: active trajectory unavailable.";
            }
            return false;
        }
        active_traj = active_.traj;
        planner = planner_;
        cfg = cfg_;
    }

    if (!planner) {
        if (error != nullptr) {
            *error = "preparePendingFromActiveTrajectoryPoint failed: planner is null.";
        }
        return false;
    }
    if (cfg.segment_sample_step_m <= 0.0 ||
        cfg.replan_every_control_ticks <= 0 ||
        cfg.control_cycle_sec <= 0.0 ||
        cfg.prediction_horizon_ticks < 0 ||
        cfg.planning_latency_sec < 0.0) {
        if (error != nullptr) {
            *error = "preparePendingFromActiveTrajectoryPoint failed: invalid replanning config.";
        }
        return false;
    }

    const int n = static_cast<int>(active_traj.waypoints.size());
    if (n < 2) {
        if (error != nullptr) {
            *error = "preparePendingFromActiveTrajectoryPoint failed: active trajectory has insufficient waypoints.";
        }
        return false;
    }

    const int start_idx = std::clamp(start_point_index, 0, n - 1);
    TimedCartesianSample pred{};
    if (!sampleFromTrajectory(active_traj, start_idx, pred)) {
        if (error != nullptr) {
            *error = "preparePendingFromActiveTrajectoryPoint failed: cannot sample start pose.";
        }
        return false;
    }
    const Eigen::Vector3d v_handoff =
        estimateTrajectoryVelocityAtPoint(active_traj, start_idx);

    PathPlanningInput in = goal_input;
    in.p_start = pred.T_target.translation();
    in.R_start = pred.T_target.linear();
    if (pred.has_ik_joint_target &&
        pred.ik_joint_target.size() > 0 &&
        pred.ik_joint_target.allFinite()) {
        in.q_start_seed = pred.ik_joint_target;
    } else {
        in.q_start_seed.reset();
    }

    const TimedCartesianTrajectory traj = planner->planTrajectory(in);
    if (traj.empty()) {
        if (error != nullptr) {
            *error = "preparePendingFromActiveTrajectoryPoint failed: planning returned empty trajectory.";
        }
        return false;
    }

    TimedCartesianTrajectory next = toDistanceSampledSegment(traj);
    if (next.empty()) {
        if (error != nullptr) {
            *error = "preparePendingFromActiveTrajectoryPoint failed: distance sampling conversion failed.";
        }
        return false;
    }

    smoothSegmentStartForHandoff(next, v_handoff);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        pending_.traj = std::move(next);
    }
    return true;
}

bool ReplannerManager::commitPendingSegment(std::string* error) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (pending_.traj.empty()) {
        if (error != nullptr) {
            *error = "commitPendingSegment failed: pending trajectory unavailable.";
        }
        return false;
    }
    active_ = std::move(pending_);
    pending_.traj = TimedCartesianTrajectory{};
    started_ = true;
    return true;
}

bool ReplannerManager::hasPendingTrajectory() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return !pending_.traj.empty();
}

bool ReplannerManager::samplePredictedReplanStart(
    const int current_point_index,
    TimedCartesianSample& out) const {
    TimedCartesianTrajectory active_traj;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_ || active_.traj.empty()) {
            return false;
        }
        active_traj = active_.traj;
    }
    const int idx = predictedPointIndex(active_traj, current_point_index);
    if (!sampleFromTrajectory(active_traj, idx, out)) {
        return false;
    }
    return true;
}

bool ReplannerManager::hasActiveTrajectory() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return started_ && !active_.traj.empty();
}

int ReplannerManager::activeSegmentPointCount() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!started_ || active_.traj.empty()) {
        return 0;
    }
    return static_cast<int>(active_.traj.waypoints.size());
}

PathPlanningInput ReplannerManager::makeNextInputFromHandoff(
    const PathPlanningInput& goal_input) const {
    PathPlanningInput in = goal_input;

    TimedCartesianTrajectory active_traj;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active_traj = active_.traj;
    }
    const TimedCartesianSample handoff =
        sampler_.sample(active_traj, active_traj.total_duration);
    in.p_start = handoff.T_target.translation();
    in.R_start = handoff.T_target.linear();
    if (handoff.has_ik_joint_target &&
        handoff.ik_joint_target.size() > 0 &&
        handoff.ik_joint_target.allFinite()) {
        in.q_start_seed = handoff.ik_joint_target;
    } else {
        in.q_start_seed.reset();
    }
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

    double segment_sample_step_m = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        segment_sample_step_m = cfg_.segment_sample_step_m;
    }
    const int n =
        static_cast<int>(std::ceil(path_length / segment_sample_step_m)) + 1;
    if (n < 2) {
        return out;
    }

    // Preserve planner-side smoothing whenever the trajectory is already sampled
    // at least as densely as the replanner requires. Only densify sparse output.
    if (raw_n >= n) {
        return in_traj;
    }

    const double total = std::max(1e-6, in_traj.total_duration);

    out.waypoints.reserve(static_cast<std::size_t>(n));
    out.waypoint_joint_targets.reserve(static_cast<std::size_t>(n));
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
        if (s.has_ik_joint_target) {
            out.waypoint_joint_targets.push_back(s.ik_joint_target);
        } else {
            out.waypoint_joint_targets.emplace_back();
        }
        if (i > 0) {
            out.segment_durations.push_back(dt);
            out.cumulative_times.push_back(i * dt);
        }
    }
    out.total_duration = total;
    return out;
}

bool ReplannerManager::sample(const int point_index, TimedCartesianSample& out) const {
    TimedCartesianTrajectory active_traj;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_ || active_.traj.empty()) {
            return false;
        }
        active_traj = active_.traj;
    }
    if (!sampleFromTrajectory(active_traj, point_index, out)) {
        return false;
    }
    return true;
}

bool ReplannerManager::sampleByElapsedTime(
    const double elapsed_sec,
    TimedCartesianSample& out) const {
    TimedCartesianTrajectory active_traj;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_ || active_.traj.empty()) {
            return false;
        }
        active_traj = active_.traj;
    }
    const double total = std::max(0.0, active_traj.total_duration);
    const double t_query = std::clamp(elapsed_sec, 0.0, total);
    out = sampler_.sample(active_traj, t_query);
    return true;
}

bool ReplannerManager::sampleFromTrajectory(
    const TimedCartesianTrajectory& traj,
    const int point_index,
    TimedCartesianSample& out) const {
    const int n = static_cast<int>(traj.waypoints.size());
    if (n < 2 || traj.cumulative_times.size() != traj.waypoints.size()) {
        return false;
    }
    const int idx = std::clamp(point_index, 0, n - 1);
    const double t_query = std::clamp(
        traj.cumulative_times[static_cast<std::size_t>(idx)],
        0.0,
        std::max(0.0, traj.total_duration));
    out = sampler_.sample(traj, t_query);
    return true;
}

int ReplannerManager::pointIndexAtTime(const double elapsed_sec) const {
    TimedCartesianTrajectory active_traj;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_ || active_.traj.empty()) {
            return 0;
        }
        active_traj = active_.traj;
    }
    const int n = static_cast<int>(active_traj.waypoints.size());
    if (n <= 1 || active_traj.cumulative_times.size() != active_traj.waypoints.size()) {
        return 0;
    }
    const double total = std::max(1e-6, active_traj.total_duration);
    const double clamped_t = std::clamp(elapsed_sec, 0.0, total);
    const auto it = std::lower_bound(
        active_traj.cumulative_times.begin(),
        active_traj.cumulative_times.end(),
        clamped_t);
    if (it == active_traj.cumulative_times.begin()) {
        return 0;
    }
    if (it == active_traj.cumulative_times.end()) {
        return n - 1;
    }
    const std::size_t upper_idx = static_cast<std::size_t>(it - active_traj.cumulative_times.begin());
    const std::size_t lower_idx = upper_idx - 1;
    const double t_lower = active_traj.cumulative_times[lower_idx];
    const double t_upper = active_traj.cumulative_times[upper_idx];
    return (clamped_t - t_lower <= t_upper - clamped_t)
               ? static_cast<int>(lower_idx)
               : static_cast<int>(upper_idx);
}

double ReplannerManager::activeSegmentTotalDurationSec() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!started_ || active_.traj.empty()) {
        return 0.0;
    }
    return std::max(0.0, active_.traj.total_duration);
}

int ReplannerManager::predictedPointIndex(const int current_point_index) const {
    TimedCartesianTrajectory active_traj;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_ || active_.traj.empty()) {
            return 0;
        }
        active_traj = active_.traj;
    }
    return predictedPointIndex(active_traj, current_point_index);
}

int ReplannerManager::predictedPointIndex(
    const TimedCartesianTrajectory& traj,
    const int current_point_index) const {
    const int n = static_cast<int>(traj.waypoints.size());
    if (n <= 1 || traj.cumulative_times.size() != traj.waypoints.size()) {
        return 0;
    }
    ReplannerConfig cfg;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cfg = cfg_;
    }
    const double control_predict_sec =
        static_cast<double>(cfg.prediction_horizon_ticks) * cfg.control_cycle_sec;
    const double total_predict_sec = control_predict_sec + cfg.planning_latency_sec;

    const int idx = std::clamp(current_point_index, 0, n - 1);
    const double start_t = std::clamp(
        traj.cumulative_times[static_cast<std::size_t>(idx)],
        0.0,
        std::max(0.0, traj.total_duration));
    const double target_t = std::clamp(
        start_t + std::max(0.0, total_predict_sec),
        0.0,
        std::max(0.0, traj.total_duration));
    const auto it = std::lower_bound(
        traj.cumulative_times.begin(),
        traj.cumulative_times.end(),
        target_t);
    if (it == traj.cumulative_times.end()) {
        return n - 1;
    }
    return static_cast<int>(it - traj.cumulative_times.begin());
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
    int handoff_blend_points = 0;
    int prediction_horizon_ticks = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        handoff_blend_points = cfg_.handoff_blend_points;
        prediction_horizon_ticks = cfg_.prediction_horizon_ticks;
    }
    const int blend_points = std::max(handoff_blend_points, prediction_horizon_ticks);
    if (n < 4 || blend_points <= 1) {
        return;
    }
    const int m = std::clamp(blend_points, 2, n - 1);
    const double t0 = traj.cumulative_times.front();
    const double tm = traj.cumulative_times[static_cast<size_t>(m)];
    const double T = std::max(1e-6, tm - t0);

    const Eigen::Vector3d p0 = traj.waypoints.front().position;  // hard anchor
    const Eigen::Vector3d pm = traj.waypoints[static_cast<size_t>(m)].position;  // preserve seam neighborhood end
    const Eigen::Vector3d vm = estimateTrajectoryVelocityAtPoint(traj, m);

    Eigen::Quaterniond q0(traj.waypoints.front().orientation);
    Eigen::Quaterniond qm(traj.waypoints[static_cast<size_t>(m)].orientation);
    q0.normalize();
    qm.normalize();
    if (q0.dot(qm) < 0.0) {
        qm.coeffs() *= -1.0;
    }
    auto quatToVec = [](const Eigen::Quaterniond& q) {
        Eigen::Vector4d out;
        out << q.x(), q.y(), q.z(), q.w();
        return out;
    };
    auto vecToQuat = [](const Eigen::Vector4d& v) {
        Eigen::Quaterniond q(v(3), v(0), v(1), v(2));
        if (q.norm() < 1e-12) {
            return Eigen::Quaterniond::Identity();
        }
        q.normalize();
        return q;
    };
    auto signedQuatAt = [&](const int idx, const Eigen::Quaterniond& ref) {
        Eigen::Quaterniond q(traj.waypoints[static_cast<size_t>(idx)].orientation);
        q.normalize();
        if (q.dot(ref) < 0.0) {
            q.coeffs() *= -1.0;
        }
        return q;
    };

    const Eigen::Vector4d q0v = quatToVec(q0);
    const Eigen::Vector4d qmv = quatToVec(qm);

    Eigen::Vector4d qdot0 = Eigen::Vector4d::Zero();
    if (n >= 2) {
        const Eigen::Quaterniond q1 = signedQuatAt(1, q0);
        const double dt0 = std::max(1e-6, traj.segment_durations.front());
        qdot0 = (quatToVec(q1) - q0v) / dt0;
    }

    Eigen::Vector4d qdotm = Eigen::Vector4d::Zero();
    if (m > 0 && m < n - 1) {
        const Eigen::Quaterniond q_prev = signedQuatAt(m - 1, qm);
        const Eigen::Quaterniond q_next = signedQuatAt(m + 1, qm);
        const double t_prev = traj.cumulative_times[static_cast<size_t>(m - 1)];
        const double t_next = traj.cumulative_times[static_cast<size_t>(m + 1)];
        const double dt = std::max(1e-6, t_next - t_prev);
        qdotm = (quatToVec(q_next) - quatToVec(q_prev)) / dt;
    } else if (m > 0) {
        const Eigen::Quaterniond q_prev = signedQuatAt(m - 1, qm);
        const double dt = std::max(1e-6, traj.segment_durations[static_cast<size_t>(m - 1)]);
        qdotm = (qmv - quatToVec(q_prev)) / dt;
    }

    auto hermiteEval = [T](const double t, const Eigen::VectorXd& p0v, const Eigen::VectorXd& v0v,
                           const Eigen::VectorXd& p1v, const Eigen::VectorXd& v1v) {
        const double tau = std::clamp(t / std::max(1e-9, T), 0.0, 1.0);
        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;
        const double h00 = 2.0 * tau3 - 3.0 * tau2 + 1.0;
        const double h10 = tau3 - 2.0 * tau2 + tau;
        const double h01 = -2.0 * tau3 + 3.0 * tau2;
        const double h11 = tau3 - tau2;
        return h00 * p0v + h10 * T * v0v + h01 * p1v + h11 * T * v1v;
    };

    for (int i = 1; i < m; ++i) {
        const double ti = traj.cumulative_times[static_cast<size_t>(i)] - t0;
        const Eigen::Vector3d pi =
            hermiteEval(ti, p0, v_start, pm, vm);
        const Eigen::Vector4d qv =
            hermiteEval(ti, q0v, qdot0, qmv, qdotm);
        traj.waypoints[static_cast<size_t>(i)].position = pi;
        traj.waypoints[static_cast<size_t>(i)].orientation =
            vecToQuat(qv).toRotationMatrix();
    }
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
