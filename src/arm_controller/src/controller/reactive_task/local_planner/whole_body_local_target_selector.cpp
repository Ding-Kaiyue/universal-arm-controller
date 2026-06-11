#include "controller/reactive_task/local_planner/whole_body_local_target_selector.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::controller::reactive_task {

namespace {

constexpr int kBaseDof = 3;

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Eigen::VectorXd interpolateState(
    const Eigen::VectorXd& q0,
    const Eigen::VectorXd& q1,
    const double ratio) {
  Eigen::VectorXd q = (1.0 - ratio) * q0 + ratio * q1;
  if (q.size() >= kBaseDof) {
    q[2] = normalizeAngle(q0[2] + ratio * normalizeAngle(q1[2] - q0[2]));
  }
  return q;
}

Eigen::VectorXd clampState(
    Eigen::VectorXd q,
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max) {
  if (q.size() != q_min.size() || q.size() != q_max.size()) {
    return q;
  }
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    q[i] = std::clamp(q[i], q_min[i], q_max[i]);
  }
  if (q.size() >= kBaseDof) {
    q[2] = normalizeAngle(q[2]);
  }
  return q;
}

bool sampleTimedJointTrajectory(
    const WholeBodyLocalTargetSelector::TimedJointTrajectory& trajectory,
    const double t_query,
    Eigen::VectorXd* q_ref) {
  if (q_ref == nullptr || trajectory.empty() ||
      trajectory.joint_targets.size() != trajectory.cumulative_times.size()) {
    return false;
  }
  if (t_query <= 0.0) {
    *q_ref = trajectory.joint_targets.front();
    return true;
  }
  if (t_query >= trajectory.total_duration) {
    *q_ref = trajectory.joint_targets.back();
    return true;
  }
  const auto upper = std::upper_bound(
      trajectory.cumulative_times.begin(),
      trajectory.cumulative_times.end(),
      t_query);
  if (upper == trajectory.cumulative_times.begin()) {
    *q_ref = trajectory.joint_targets.front();
    return true;
  }
  if (upper == trajectory.cumulative_times.end()) {
    *q_ref = trajectory.joint_targets.back();
    return true;
  }
  const auto i1 = static_cast<std::size_t>(
      std::distance(trajectory.cumulative_times.begin(), upper));
  const std::size_t i0 = i1 - 1u;
  const double t0 = trajectory.cumulative_times[i0];
  const double t1 = trajectory.cumulative_times[i1];
  const double ratio = (t1 > t0 + 1.0e-9)
                           ? std::clamp((t_query - t0) / (t1 - t0), 0.0, 1.0)
                           : 0.0;
  *q_ref = interpolateState(
      trajectory.joint_targets[i0], trajectory.joint_targets[i1], ratio);
  return true;
}

}  // namespace

WholeBodyLocalTargetSelector::WholeBodyLocalTargetSelector()
    : WholeBodyLocalTargetSelector(Config{}) {}

WholeBodyLocalTargetSelector::WholeBodyLocalTargetSelector(Config config)
    : config_(config) {}

bool WholeBodyLocalTargetSelector::select(
    const Input& input,
    Output* output) const {
  if (output == nullptr) {
    return false;
  }
  *output = Output{};
  if (input.q_current.size() <= 0 || !input.q_current.allFinite() ||
      input.global_reference.empty()) {
    return false;
  }

  const int horizon_steps = std::max(3, config_.horizon_steps);
  const double dt = std::max(1.0e-3, config_.dt_sec);
  output->references.reserve(static_cast<std::size_t>(horizon_steps));
  output->references.push_back(clampState(input.q_current, input.q_min, input.q_max));

  for (int i = 1; i < horizon_steps; ++i) {
    Eigen::VectorXd q_ref;
    if (!sampleTimedJointTrajectory(
            input.global_reference,
            input.global_time_sec + dt * static_cast<double>(i),
            &q_ref) ||
        q_ref.size() != input.q_current.size() || !q_ref.allFinite()) {
      return false;
    }
    output->references.push_back(clampState(q_ref, input.q_min, input.q_max));
  }
  output->target = output->references.back();
  output->dt_sec = dt;
  output->ok = true;
  return true;
}

}  // namespace arm_controller::controller::reactive_task
