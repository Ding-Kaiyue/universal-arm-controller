#include "controller/reactive_task/controller/reactive_task_control_target_builder.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::controller::reactive_task {

ControlTarget ReactiveTaskControlTargetBuilder::build(
    const Input& input) const {
  ControlTarget target;
  target.setPoseTwist(input.sample.T_target, input.sample.target_twist);

  if (!input.phase_flags.path_follow_active) {
    return target;
  }

  if (input.terminal_goal_tracking) {
    Eigen::Isometry3d terminal_pose = Eigen::Isometry3d::Identity();
    terminal_pose.linear() = input.goal_rotation;
    terminal_pose.translation() = input.goal_position;
    target.setPoseTwist(terminal_pose, Eigen::Matrix<double, 6, 1>::Zero());
    target.local_planner_tracking = false;
    if (input.terminal_joint_anchor_sample_valid) {
      target.setLocalPlannerJointTarget(
          input.terminal_joint_anchor_sample.ik_joint_target, 1.0);
    }
    return target;
  }

  if (applyLocalPlannerTrajectory(input, &target)) {
    return target;
  }

  (void)applyLocalPlannerFallback(input, &target);
  return target;
}

bool ReactiveTaskControlTargetBuilder::applyLocalPlannerTrajectory(
    const Input& input,
    ControlTarget* target) const {
  if (target == nullptr || input.local_planner == nullptr ||
      !input.local_planner->trajectory_valid ||
      input.local_planner->target_poses.empty()) {
    return false;
  }

  const LocalPlannerRuntime& local_planner = *input.local_planner;
  const double local_elapsed =
      std::max(0.0,
               input.tracked_reference_time_sec - local_planner.start_time_sec);
  const std::size_t elapsed_index =
      static_cast<std::size_t>(
          std::floor(local_elapsed / std::max(1e-3, local_planner.dt_sec))) +
      1u;
  const std::size_t sample_index = std::min<std::size_t>(
      local_planner.target_poses.size() - 1u,
      local_planner.start_index + elapsed_index);
  const bool target_accepted =
      local_planner.target_poses[sample_index].matrix().allFinite();
  if (!target_accepted) {
    if (input.clock) {
      RCLCPP_WARN_THROTTLE(
          input.logger, *input.clock, 1000,
          "[%s] local_trajopt target rejected: non-finite optimized pose",
          input.mapping.c_str());
    }
    return false;
  }

  target->setPoseTwist(local_planner.target_poses[sample_index],
                       local_planner.target_twists[sample_index]);
  target->local_planner_tracking = true;
  if (sample_index < local_planner.joint_targets.size() &&
      local_planner.joint_targets[sample_index].size() == input.arm_state.q.size() &&
      local_planner.joint_targets[sample_index].allFinite()) {
    target->setLocalPlannerJointTarget(
        local_planner.joint_targets[sample_index],
        std::max(1e-3, local_planner.dt_sec));
  }
  return true;
}

bool ReactiveTaskControlTargetBuilder::applyLocalPlannerFallback(
    const Input& input,
    ControlTarget* target) const {
  if (target == nullptr || input.local_planner == nullptr ||
      !input.local_planner->last_output_valid) {
    return false;
  }

  const LocalPlannerRuntime& local_planner = *input.local_planner;
  const bool target_accepted =
      local_planner.last_target_pose.matrix().allFinite();
  if (!target_accepted) {
    if (input.clock) {
      RCLCPP_WARN_THROTTLE(
          input.logger, *input.clock, 1000,
          "[%s] local_trajopt fallback target rejected: non-finite optimized pose",
          input.mapping.c_str());
    }
    return false;
  }

  target->setPoseTwist(local_planner.last_target_pose,
                       local_planner.last_target_twist);
  target->local_planner_tracking = true;
  if (local_planner.last_joint_target.size() == input.arm_state.q.size() &&
      local_planner.last_joint_target.allFinite()) {
    target->setLocalPlannerJointTarget(
        local_planner.last_joint_target,
        local_planner.last_joint_target_dt_sec);
  }
  return true;
}

}  // namespace arm_controller::controller::reactive_task
