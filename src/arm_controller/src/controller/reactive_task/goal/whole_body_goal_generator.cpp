#include "controller/reactive_task/goal/whole_body_goal_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arm_controller::controller::reactive_task {

namespace {

double normalizeAngle(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double orientationErrorNorm(const Eigen::Matrix3d& current,
                            const Eigen::Matrix3d& target) {
  Eigen::Quaterniond q_current(current);
  Eigen::Quaterniond q_target(target);
  q_current.normalize();
  q_target.normalize();
  Eigen::Quaterniond q_err = q_current.conjugate() * q_target;
  q_err.normalize();
  return std::abs(Eigen::AngleAxisd(q_err).angle());
}

Eigen::VectorXd makeFullState(
    const WholeBodyGoalGenerator::BaseState& base,
    const Eigen::VectorXd& arm) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(WholeBodyGoalGenerator::kFullDof);
  q[0] = base.x;
  q[1] = base.y;
  q[2] = base.yaw;
  if (arm.size() == WholeBodyGoalGenerator::kArmDof) {
    q.segment(WholeBodyGoalGenerator::kBaseDof,
              WholeBodyGoalGenerator::kArmDof) = arm;
  }
  return q;
}

}  // namespace

WholeBodyGoalGenerator::WholeBodyGoalGenerator()
    : WholeBodyGoalGenerator(Config{}) {}

WholeBodyGoalGenerator::WholeBodyGoalGenerator(Config config)
    : config_(std::move(config)) {}

WholeBodyGoalGenerator::Output WholeBodyGoalGenerator::generate(
    const Input& input) const {
  Output output;
  if (input.q_start.size() != kFullDof || input.q_min.size() != kFullDof ||
      input.q_max.size() != kFullDof || !input.full_state_fk) {
    Diagnostic diag;
    diag.reason = "invalid_generator_input";
    output.diagnostics.push_back(std::move(diag));
    return output;
  }

  for (const Seed& seed : input.seeds) {
    Eigen::VectorXd q_seed = makeFullState(seed.base, seed.arm_seed);
    if (input.seed_projector) {
      Eigen::VectorXd projected_arm;
      if (input.seed_projector(seed.base, seed.arm_seed, &projected_arm) &&
          projected_arm.size() == kArmDof && projected_arm.allFinite()) {
        q_seed.segment(kBaseDof, kArmDof) = projected_arm;
      }
    }

    q_seed = clampToBounds(input, q_seed);
    Eigen::VectorXd q_optimized = optimizeSeed(input, q_seed);
    Diagnostic diag = diagnose(input, q_optimized);
    output.diagnostics.push_back(diag);

    if (!diag.valid || isDuplicate(output.q_goal_candidates, q_optimized)) {
      continue;
    }
    output.q_goal_candidates.push_back(std::move(q_optimized));
    if (output.q_goal_candidates.size() >= config_.max_goal_candidates) {
      break;
    }
  }

  return output;
}

WholeBodyGoalGenerator::Diagnostic WholeBodyGoalGenerator::diagnose(
    const Input& input, const Eigen::VectorXd& q) const {
  Diagnostic diag;
  diag.q_goal = q;
  diag.valid = false;
  diag.collision_free = false;
  diag.reason = "unchecked";

  if (q.size() != kFullDof || !q.allFinite() || !input.full_state_fk) {
    diag.reason = "invalid_q_goal";
    return diag;
  }

  Eigen::Isometry3d left_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d right_pose = Eigen::Isometry3d::Identity();
  if (!input.full_state_fk(q, &left_pose, &right_pose) ||
      !left_pose.matrix().allFinite() || !right_pose.matrix().allFinite()) {
    diag.reason = "fk_failed";
    return diag;
  }

  diag.left_position_error_m =
      (left_pose.translation() - input.left_target.translation()).norm();
  diag.right_position_error_m =
      (right_pose.translation() - input.right_target.translation()).norm();
  diag.left_orientation_error_rad =
      orientationErrorNorm(left_pose.linear(), input.left_target.linear());
  diag.right_orientation_error_rad =
      orientationErrorNorm(right_pose.linear(), input.right_target.linear());

  const bool position_ok =
      diag.left_position_error_m <= config_.position_tolerance_m &&
      diag.right_position_error_m <= config_.position_tolerance_m;
  const bool orientation_ok =
      !config_.require_strict_orientation ||
      (diag.left_orientation_error_rad <= config_.orientation_tolerance_rad &&
       diag.right_orientation_error_rad <= config_.orientation_tolerance_rad);
  if (!position_ok || !orientation_ok) {
    diag.reason = "pose_error_too_large";
    return diag;
  }

  if (input.joint_state_validator) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic collision_diag;
    diag.collision_free =
        input.joint_state_validator(q, config_.hard_clearance, &collision_diag);
    diag.min_margin = collision_diag.min_margin;
    if (!diag.collision_free) {
      diag.reason = collision_diag.reason.empty() ? "collision_fail"
                                                  : collision_diag.reason;
      return diag;
    }
  } else {
    diag.collision_free = true;
    diag.min_margin = std::numeric_limits<double>::infinity();
  }

  diag.valid = true;
  diag.reason = "ok";
  return diag;
}

Eigen::VectorXd WholeBodyGoalGenerator::optimizeSeed(
    const Input& input, const Eigen::VectorXd& seed) const {
  Eigen::VectorXd q = clampToBounds(input, seed);
  if (q.size() != kFullDof || !q.allFinite()) {
    return seed;
  }

  double step = std::max(1e-4, config_.optimizer_step);
  double best_cost = objective(input, q);
  for (int iter = 0; iter < config_.optimizer_iterations; ++iter) {
    const Eigen::VectorXd grad = finiteDifferenceGradient(input, q);
    if (grad.size() != q.size() || !grad.allFinite() || grad.norm() < 1e-8) {
      break;
    }

    bool accepted = false;
    for (int line = 0; line < 8; ++line) {
      Eigen::VectorXd candidate = clampToBounds(input, q - step * grad);
      candidate[2] = normalizeAngle(candidate[2]);
      const double candidate_cost = objective(input, candidate);
      if (std::isfinite(candidate_cost) && candidate_cost < best_cost) {
        q = std::move(candidate);
        best_cost = candidate_cost;
        accepted = true;
        step = std::min(0.20, step * 1.15);
        break;
      }
      step *= 0.5;
    }
    if (!accepted && step < 1e-5) {
      break;
    }
  }
  return clampToBounds(input, q);
}

double WholeBodyGoalGenerator::objective(const Input& input,
                                         const Eigen::VectorXd& q) const {
  if (q.size() != kFullDof || !q.allFinite() || !input.full_state_fk) {
    return std::numeric_limits<double>::infinity();
  }

  Eigen::Isometry3d left_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d right_pose = Eigen::Isometry3d::Identity();
  if (!input.full_state_fk(q, &left_pose, &right_pose)) {
    return std::numeric_limits<double>::infinity();
  }

  const double left_pos =
      (left_pose.translation() - input.left_target.translation()).squaredNorm();
  const double right_pos =
      (right_pose.translation() - input.right_target.translation()).squaredNorm();
  const double left_rot =
      std::pow(orientationErrorNorm(left_pose.linear(), input.left_target.linear()),
               2);
  const double right_rot =
      std::pow(orientationErrorNorm(right_pose.linear(), input.right_target.linear()),
               2);

  double cost =
      config_.position_weight * (left_pos + right_pos) +
      config_.orientation_weight * (left_rot + right_rot);

  if (input.q_start.size() == kFullDof) {
    cost += config_.base_xy_weight *
            (q.head<2>() - input.q_start.head<2>()).squaredNorm();
    cost += config_.base_yaw_weight *
            std::pow(normalizeAngle(q[2] - input.q_start[2]), 2);
    cost += config_.arm_motion_weight *
            (q.segment(kBaseDof, kArmDof) -
             input.q_start.segment(kBaseDof, kArmDof))
                .squaredNorm();
  }
  if (input.q_nominal.size() == kFullDof) {
    cost += config_.posture_weight *
            (q.segment(kBaseDof, kArmDof) -
             input.q_nominal.segment(kBaseDof, kArmDof))
                .squaredNorm();
  }

  if (input.joint_state_validator) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
    const bool ok =
        input.joint_state_validator(q, config_.hard_clearance, &diag);
    if (!ok) {
      const double margin =
          std::isfinite(diag.min_margin) ? diag.min_margin : -0.05;
      cost += config_.collision_weight * std::pow(std::min(0.0, margin), 2);
    }
  }

  return cost;
}

Eigen::VectorXd WholeBodyGoalGenerator::finiteDifferenceGradient(
    const Input& input, const Eigen::VectorXd& q) const {
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());
  const double base_cost = objective(input, q);
  if (!std::isfinite(base_cost)) {
    return grad;
  }

  for (Eigen::Index i = 0; i < q.size(); ++i) {
    const double eps = (i < kBaseDof) ? 1e-4 : 1e-5;
    Eigen::VectorXd q_plus = q;
    Eigen::VectorXd q_minus = q;
    q_plus[i] += eps;
    q_minus[i] -= eps;
    if (i == 2) {
      q_plus[i] = normalizeAngle(q_plus[i]);
      q_minus[i] = normalizeAngle(q_minus[i]);
    }
    q_plus = clampToBounds(input, q_plus);
    q_minus = clampToBounds(input, q_minus);
    const double c_plus = objective(input, q_plus);
    const double c_minus = objective(input, q_minus);
    if (std::isfinite(c_plus) && std::isfinite(c_minus)) {
      grad[i] = (c_plus - c_minus) / (2.0 * eps);
    }
  }
  return grad;
}

Eigen::VectorXd WholeBodyGoalGenerator::clampToBounds(
    const Input& input, Eigen::VectorXd q) const {
  if (q.size() != input.q_min.size() || q.size() != input.q_max.size()) {
    return q;
  }
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    const double lo = std::min(input.q_min[i], input.q_max[i]);
    const double hi = std::max(input.q_min[i], input.q_max[i]);
    if (std::isfinite(lo) && std::isfinite(hi)) {
      q[i] = std::clamp(q[i], lo, hi);
    }
  }
  if (q.size() >= kBaseDof) {
    q[2] = normalizeAngle(q[2]);
  }
  return q;
}

bool WholeBodyGoalGenerator::isDuplicate(
    const std::vector<Eigen::VectorXd>& goals, const Eigen::VectorXd& q) const {
  for (const Eigen::VectorXd& existing : goals) {
    if (existing.size() == q.size() &&
        (existing - q).cwiseAbs().maxCoeff() < 1e-3) {
      return true;
    }
  }
  return false;
}

}  // namespace arm_controller::controller::reactive_task
