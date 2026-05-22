#include "controller/reactive_task/local_planner/reactive_task_local_planner_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace arm_controller::controller::reactive_task {

namespace {

std::string formatVector(const Eigen::VectorXd& v) {
  std::ostringstream oss;
  oss << v.transpose().format(Eigen::IOFormat(4, 0, ", ", ", ", "[", "]"));
  return oss.str();
}

}  // namespace

void LocalPlannerRuntime::clearTracking() {
  last_output_valid = false;
  trajectory_valid = false;
  target_poses.clear();
  target_twists.clear();
  joint_targets.clear();
  start_index = 0u;
}

void LocalPlannerRuntime::clearCachedTarget() {
  last_output_valid = false;
  last_joint_target.resize(0);
  last_joint_target_dt_sec = 0.0;
}

LocalPlannerRuntime::ApplyOutputResult LocalPlannerRuntime::applyOutput(
    ApplyOutputInput input) {
  ApplyOutputResult result;

  const double output_age_sec = input.now_sec - input.request_time_sec;
  if (std::isfinite(output_age_sec) &&
      output_age_sec > input.max_usable_age_sec) {
    result.stale_age = true;
    result.reason = "stale age";
    RCLCPP_DEBUG(input.logger,
                 "[%s] local_trajopt async result dropped: stale age=%.3f max=%.3f",
                 input.mapping.c_str(),
                 output_age_sec,
                 input.max_usable_age_sec);
    return result;
  }

  last_target_pose = input.output.target_pose;
  last_target_twist = input.output.target_twist;
  clearCachedTarget();
  if (input.output.has_optimized_joint_target &&
      input.output.optimized_joint_target.size() == input.planner_start_state.size() &&
      input.output.optimized_joint_target.allFinite()) {
    last_joint_target = input.output.optimized_joint_target;
    last_joint_target_dt_sec = input.output.optimized_joint_target_time_sec;
  }
  last_update_time_sec = input.now_sec;
  last_output_valid = true;

  const double local_trajectory_dt_sec =
      std::max(1e-3, input.output.trajectory_dt_sec);
  result.nominal_twist = input.output.nominal_twist;
  target_poses = std::move(input.output.target_poses);
  target_twists = std::move(input.output.target_twists);
  joint_targets = std::move(input.output.optimized_joint_trajectory);
  start_index = 0u;

  if (!joint_targets.empty()) {
    double best_distance_sq = std::numeric_limits<double>::infinity();
    const bool has_current_velocity =
        input.current_qdot_reference_valid &&
        input.current_qdot_reference.size() == input.planner_start_state.size() &&
        input.current_qdot_reference.norm() > 1e-4;
    for (std::size_t i = 0u; i < joint_targets.size(); ++i) {
      const Eigen::VectorXd& q_ref = joint_targets[i];
      if (q_ref.size() != input.planner_start_state.size() || !q_ref.allFinite()) {
        continue;
      }
      const Eigen::VectorXd delta = q_ref - input.planner_start_state;
      double score = delta.squaredNorm();
      if (has_current_velocity) {
        const double along_velocity = delta.dot(input.current_qdot_reference);
        score *= along_velocity > 0.0 ? 0.75 : 1.35;
      }
      if (score < best_distance_sq) {
        best_distance_sq = score;
        start_index = i;
      }
    }
  }

  const std::size_t remaining_local_samples =
      joint_targets.size() > start_index ? joint_targets.size() - start_index
                                         : 0u;
  if (remaining_local_samples < 8u && joint_targets.size() >= 8u) {
    clearCachedTarget();
    clearTracking();
    result.stale_handoff = true;
    result.reason = "stale handoff";
    RCLCPP_DEBUG(input.logger,
                 "[%s] local_trajopt async result dropped: stale handoff remaining=%zu",
                 input.mapping.c_str(),
                 remaining_local_samples);
    return result;
  }

  start_time_sec = input.now_sec;
  dt_sec = local_trajectory_dt_sec;
  trajectory_valid = !target_poses.empty() &&
                     target_poses.size() == target_twists.size() &&
                     target_poses.size() == joint_targets.size();

  if (!joint_targets.empty() && input.clock) {
    RCLCPP_INFO_THROTTLE(
        input.logger, *input.clock, 1000,
        "[%s] local_trajopt joint trace tick=%d steps=%zu dt=%.4f start_index=%zu q_now=%s q_first=%s q_last=%s q_goal=%s",
        input.mapping.c_str(),
        input.planner_tick,
        joint_targets.size(),
        dt_sec,
        start_index,
        formatVector(input.planner_start_state).c_str(),
        formatVector(joint_targets.front()).c_str(),
        formatVector(joint_targets.back()).c_str(),
        formatVector(input.q_goal).c_str());
  }

  result.accepted = true;
  return result;
}

}  // namespace arm_controller::controller::reactive_task
