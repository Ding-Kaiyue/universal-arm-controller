#include "algorithm/cartesian_path_planner/global_trajectory/global_trajectory_manager.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace arm_controller::algorithm::cartesian_path_planner {

GlobalTrajectoryManager::GlobalTrajectoryManager(
    std::shared_ptr<arm_controller::algorithm::global_planner::GlobalPlannerInterface> planner)
    : planner_(std::move(planner)) {}

void GlobalTrajectoryManager::setConfig(const GlobalTrajectoryConfig &cfg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  cfg_ = cfg;
}

bool GlobalTrajectoryManager::start(const PathPlanningInput &first_input,
                                    std::string *error) {
  return planSegment(first_input, error);
}

bool GlobalTrajectoryManager::planSegment(const PathPlanningInput &input,
                                          std::string *error) {
  std::shared_ptr<arm_controller::algorithm::global_planner::GlobalPlannerInterface> planner;
  GlobalTrajectoryConfig cfg;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    planner = planner_;
    cfg = cfg_;
  }

  if (!planner) {
    if (error != nullptr) {
      *error = "GlobalTrajectoryManager planSegment failed: planner is null.";
    }
    return false;
  }

  if (cfg.control_cycle_sec <= 0.0 || cfg.planning_latency_sec < 0.0) {
    if (error != nullptr) {
      *error = "GlobalTrajectoryManager planSegment failed: invalid global "
               "trajectory config.";
    }
    return false;
  }

  const TimedJointTrajectory traj = planner->planTrajectory(input);
  if (traj.empty()) {
    if (error != nullptr) {
      *error = "GlobalTrajectoryManager planSegment failed: planning returned "
               "empty trajectory.";
    }
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_.traj = traj;
    active_.joint_to_pose_fn = input.joint_to_pose_fn;
    pending_.traj = TimedJointTrajectory{};
    pending_.joint_to_pose_fn = nullptr;
    started_ = true;
  }
  return true;
}

bool GlobalTrajectoryManager::preparePendingSegment(
    const PathPlanningInput &goal_input, std::string *error) {
  std::shared_ptr<arm_controller::algorithm::global_planner::GlobalPlannerInterface> planner;
  GlobalTrajectoryConfig cfg;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    planner = planner_;
    cfg = cfg_;
  }

  if (!planner) {
    if (error != nullptr) {
      *error = "preparePendingSegment failed: planner is null.";
    }
    return false;
  }
  if (cfg.control_cycle_sec <= 0.0 || cfg.planning_latency_sec < 0.0) {
    if (error != nullptr) {
      *error = "preparePendingSegment failed: invalid global trajectory config.";
    }
    return false;
  }

  const TimedJointTrajectory traj = planner->planTrajectory(goal_input);
  if (traj.empty()) {
    if (error != nullptr) {
      *error =
          "preparePendingSegment failed: planning returned empty trajectory.";
    }
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_.traj = traj;
    pending_.joint_to_pose_fn = goal_input.joint_to_pose_fn;
  }
  return true;
}

bool GlobalTrajectoryManager::commitPendingSegment(std::string *error) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (pending_.traj.empty()) {
    if (error != nullptr) {
      *error = "commitPendingSegment failed: pending trajectory unavailable.";
    }
    return false;
  }
  active_ = std::move(pending_);
  pending_.traj = TimedJointTrajectory{};
  pending_.joint_to_pose_fn = nullptr;
  started_ = true;
  return true;
}

bool GlobalTrajectoryManager::hasActiveTrajectory() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return started_ && !active_.traj.empty();
}

int GlobalTrajectoryManager::activeSegmentPointCount() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!started_ || active_.traj.empty()) {
    return 0;
  }
  return static_cast<int>(active_.traj.joint_targets.size());
}

bool GlobalTrajectoryManager::sample(const int point_index,
                                     TimedCartesianSample &out) const {
  TimedSegment active_segment;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!started_ || active_.traj.empty()) {
      return false;
    }
    active_segment = active_;
  }
  if (!sampleFromSegment(active_segment, point_index, out)) {
    return false;
  }
  return true;
}

bool GlobalTrajectoryManager::sampleByElapsedTime(const double elapsed_sec,
                                                  TimedCartesianSample &out) const {
  TimedSegment active_segment;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!started_ || active_.traj.empty()) {
      return false;
    }
    active_segment = active_;
  }
  const TimedJointTrajectory &active_traj = active_segment.traj;
  const double t_query = std::max(0.0, elapsed_sec);
  out = TimedCartesianSample{};
  out.is_cartesian_tracking_target = false;
  out.time_from_start = t_query;
  if (active_traj.empty() ||
      active_traj.cumulative_times.size() != active_traj.joint_targets.size() ||
      active_traj.segment_durations.size() + 1 != active_traj.joint_targets.size()) {
    return false;
  }

  const int n = static_cast<int>(active_traj.joint_targets.size());
  if (n < 2) {
    return false;
  }

  const double total = std::max(0.0, active_traj.total_duration);
  const double clamped_t = std::clamp(t_query, 0.0, total);
  const auto it =
      std::lower_bound(active_traj.cumulative_times.begin(),
                       active_traj.cumulative_times.end(), clamped_t);
  std::size_t upper_idx = 0u;
  if (it == active_traj.cumulative_times.end()) {
    upper_idx = static_cast<std::size_t>(n - 1);
  } else {
    upper_idx = static_cast<std::size_t>(
        it - active_traj.cumulative_times.begin());
  }
  if (upper_idx == 0u) {
    const Eigen::VectorXd &q = active_traj.joint_targets.front();
    if (q.size() <= 0 || !q.allFinite()) {
      return false;
    }
    out.ik_joint_target = q;
    out.has_ik_joint_target = true;
    out.time_from_start = clamped_t;
    return populateSamplePose(active_segment.joint_to_pose_fn, out);
  }

  const std::size_t lower_idx = upper_idx - 1u;
  const Eigen::VectorXd &q_lower = active_traj.joint_targets[lower_idx];
  const Eigen::VectorXd &q_upper = active_traj.joint_targets[upper_idx];
  if (q_lower.size() <= 0 || q_upper.size() != q_lower.size() ||
      !q_lower.allFinite() || !q_upper.allFinite()) {
    return false;
  }

  const double t_lower = active_traj.cumulative_times[lower_idx];
  const double t_upper = active_traj.cumulative_times[upper_idx];
  const double alpha =
      (t_upper > t_lower + 1e-9)
          ? std::clamp((clamped_t - t_lower) / (t_upper - t_lower), 0.0, 1.0)
          : 1.0;
  const Eigen::VectorXd q = (1.0 - alpha) * q_lower + alpha * q_upper;
  out.ik_joint_target = q;
  out.has_ik_joint_target = true;
  out.time_from_start = clamped_t;
  return populateSamplePose(active_segment.joint_to_pose_fn, out);
}

int GlobalTrajectoryManager::nearestPointIndexByJointTarget(
    const Eigen::VectorXd &q_current, const int hint_index,
    const int search_window) const {
  if (q_current.size() <= 0 || !q_current.allFinite()) {
    return 0;
  }

  TimedJointTrajectory active_traj;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!started_ || active_.traj.empty()) {
      return 0;
    }
    active_traj = active_.traj;
  }

  const int n = static_cast<int>(active_traj.joint_targets.size());
  if (n <= 1) {
    return pointIndexAtTime(0.0);
  }

  const int clamped_hint = std::clamp(hint_index, 0, n - 1);
  const int window = std::max(1, search_window);
  int start_idx = std::max(0, clamped_hint - window);
  int end_idx = std::min(n - 1, clamped_hint + window);

  auto findBestInRange = [&](const int begin, const int end) {
    double best_dist = std::numeric_limits<double>::infinity();
    int best_idx = clamped_hint;
    for (int idx = begin; idx <= end; ++idx) {
      const Eigen::VectorXd &q_ref =
          active_traj.joint_targets[static_cast<std::size_t>(idx)];
      if (q_ref.size() != q_current.size() || !q_ref.allFinite()) {
        continue;
      }
      const double dist = (q_ref - q_current).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = idx;
      }
    }
    return std::make_pair(best_idx, best_dist);
  };

  auto [best_idx, best_dist] = findBestInRange(start_idx, end_idx);
  if (!std::isfinite(best_dist)) {
    std::tie(best_idx, best_dist) = findBestInRange(0, n - 1);
  }
  return best_idx;
}

double GlobalTrajectoryManager::timeAtPointIndex(const int point_index) const {
  TimedJointTrajectory active_traj;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!started_ || active_.traj.empty()) {
      return 0.0;
    }
    active_traj = active_.traj;
  }
  const int n = static_cast<int>(active_traj.joint_targets.size());
  if (n <= 1 ||
      active_traj.cumulative_times.size() != active_traj.joint_targets.size()) {
    return 0.0;
  }
  const int idx = std::clamp(point_index, 0, n - 1);
  return std::clamp(active_traj.cumulative_times[static_cast<std::size_t>(idx)],
                    0.0, std::max(0.0, active_traj.total_duration));
}

bool GlobalTrajectoryManager::populateSamplePose(
    const PathPlanningInput::JointToPoseFn &joint_to_pose_fn,
    TimedCartesianSample &sample) const {
  if (!sample.has_ik_joint_target || sample.ik_joint_target.size() <= 0 ||
      !sample.ik_joint_target.allFinite() || !joint_to_pose_fn) {
    return false;
  }

  CartesianWaypoint waypoint;
  if (!joint_to_pose_fn(sample.ik_joint_target, waypoint) ||
      !waypoint.position.allFinite() || !waypoint.orientation.allFinite()) {
    return false;
  }

  sample.T_target = Eigen::Isometry3d::Identity();
  sample.T_target.linear() = waypoint.orientation;
  sample.T_target.translation() = waypoint.position;
  return sample.T_target.matrix().allFinite();
}

bool GlobalTrajectoryManager::sampleFromSegment(
    const TimedSegment &segment, const int point_index,
    TimedCartesianSample &out) const {
  const TimedJointTrajectory &traj = segment.traj;
  const int n = static_cast<int>(traj.joint_targets.size());
  if (n < 2 || traj.cumulative_times.size() != traj.joint_targets.size()) {
    return false;
  }
  const int idx = std::clamp(point_index, 0, n - 1);
  const double t_query =
      std::clamp(traj.cumulative_times[static_cast<std::size_t>(idx)], 0.0,
                 std::max(0.0, traj.total_duration));
  out = TimedCartesianSample{};
  out.time_from_start = t_query;
  out.is_cartesian_tracking_target = false;
  const Eigen::VectorXd &q = traj.joint_targets[static_cast<std::size_t>(idx)];
  if (q.size() <= 0 || !q.allFinite()) {
    return false;
  }
  out.ik_joint_target = q;
  out.has_ik_joint_target = true;
  return populateSamplePose(segment.joint_to_pose_fn, out);
}

int GlobalTrajectoryManager::pointIndexAtTime(const double elapsed_sec) const {
  TimedJointTrajectory active_traj;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!started_ || active_.traj.empty()) {
      return 0;
    }
    active_traj = active_.traj;
  }
  const int n = static_cast<int>(active_traj.joint_targets.size());
  if (n <= 1 ||
      active_traj.cumulative_times.size() != active_traj.joint_targets.size()) {
    return 0;
  }
  const double total = std::max(1e-6, active_traj.total_duration);
  const double clamped_t = std::clamp(elapsed_sec, 0.0, total);
  const auto it =
      std::lower_bound(active_traj.cumulative_times.begin(),
                       active_traj.cumulative_times.end(), clamped_t);
  if (it == active_traj.cumulative_times.begin()) {
    return 0;
  }
  if (it == active_traj.cumulative_times.end()) {
    return n - 1;
  }
  const std::size_t upper_idx =
      static_cast<std::size_t>(it - active_traj.cumulative_times.begin());
  const std::size_t lower_idx = upper_idx - 1;
  const double t_lower = active_traj.cumulative_times[lower_idx];
  const double t_upper = active_traj.cumulative_times[upper_idx];
  return (clamped_t - t_lower <= t_upper - clamped_t)
             ? static_cast<int>(lower_idx)
             : static_cast<int>(upper_idx);
}

double GlobalTrajectoryManager::activeSegmentTotalDurationSec() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!started_ || active_.traj.empty()) {
    return 0.0;
  }
  return std::max(0.0, active_.traj.total_duration);
}

PlannedSegmentKind GlobalTrajectoryManager::activeSegmentKind() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!started_ || active_.traj.empty()) {
    return PlannedSegmentKind::Goal;
  }
  return active_.traj.segment_kind;
}

} // namespace arm_controller::algorithm::cartesian_path_planner
