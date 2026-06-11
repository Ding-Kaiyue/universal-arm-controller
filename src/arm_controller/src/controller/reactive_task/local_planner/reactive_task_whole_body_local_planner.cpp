#include "controller/reactive_task/local_planner/reactive_task_whole_body_local_planner.hpp"

#include "algorithm/global_planner/layer_gap_rrt_connector.hpp"
#include "controller/reactive_task/local_planner/whole_body_frontend_initializer.hpp"
#include "controller/reactive_task/local_planner/whole_body_lbfgs_optimizer.hpp"
#include "controller/reactive_task/local_planner/whole_body_local_target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <utility>

namespace arm_controller::controller::reactive_task {

namespace {

constexpr int kBaseDof = 3;
constexpr double kMinDtSec = 1.0e-3;
constexpr std::size_t kFrontendLayerStride = 4u;

namespace gp = arm_controller::algorithm::global_planner;

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Eigen::VectorXd clampState(
    Eigen::VectorXd q,
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max) {
  if (q.size() != q_min.size() || q.size() != q_max.size()) {
    return q;
  }
  for (int i = 0; i < q.size(); ++i) {
    q[i] = std::clamp(q[i], q_min[i], q_max[i]);
  }
  if (q.size() >= kBaseDof) {
    q[2] = normalizeAngle(q[2]);
  }
  return q;
}

Eigen::VectorXd interpolateState(
    const Eigen::VectorXd& from,
    const Eigen::VectorXd& to,
    const double ratio) {
  Eigen::VectorXd q = (1.0 - ratio) * from + ratio * to;
  if (q.size() >= kBaseDof) {
    q[2] = normalizeAngle(from[2] + ratio * normalizeAngle(to[2] - from[2]));
  }
  return q;
}

Eigen::VectorXd clampStepByDynamics(
    const Eigen::VectorXd& from,
    Eigen::VectorXd target,
    const WholeBodyLocalPlanner::Config& cfg,
    const double dt) {
  if (from.size() != target.size()) {
    return target;
  }
  const double safe_dt = std::max(kMinDtSec, dt);
  if (target.size() >= kBaseDof) {
    target[0] = from[0] + std::clamp(
                              target[0] - from[0],
                              -std::max(0.0, cfg.max_base_vx) * safe_dt,
                              std::max(0.0, cfg.max_base_vx) * safe_dt);
    target[1] = from[1] + std::clamp(
                              target[1] - from[1],
                              -std::max(0.0, cfg.max_base_vy) * safe_dt,
                              std::max(0.0, cfg.max_base_vy) * safe_dt);
    const double yaw_delta = std::clamp(
        normalizeAngle(target[2] - from[2]),
        -std::max(0.0, cfg.max_base_wz) * safe_dt,
        std::max(0.0, cfg.max_base_wz) * safe_dt);
    target[2] = normalizeAngle(from[2] + yaw_delta);
  }
  const double arm_step =
      std::max(0.0, cfg.max_arm_qdot) * safe_dt;
  for (int i = kBaseDof; i < target.size(); ++i) {
    target[i] = from[i] + std::clamp(target[i] - from[i], -arm_step, arm_step);
  }
  return target;
}

bool validateState(
    const cp::PathPlanningInput::JointStateValidatorFn& validator,
    const Eigen::VectorXd& q,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
  if (!validator) {
    return true;
  }
  cp::PathPlanningInput::WholeBodyPoseDiagnostic local_diag;
  const bool ok = validator(q, safe_distance, &local_diag);
  if (diag != nullptr) {
    *diag = local_diag;
  }
  return ok;
}

bool validateSegment(
    const cp::PathPlanningInput::JointSegmentValidatorFn& validator,
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
  if (!validator) {
    return true;
  }
  cp::PathPlanningInput::WholeBodyPoseDiagnostic local_diag;
  const bool ok = validator(q_from, q_to, safe_distance, &local_diag);
  if (diag != nullptr) {
    *diag = local_diag;
  }
  return ok;
}

bool validateForwardSegment(
    const WholeBodyLocalPlanner::Input& input,
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
  cp::PathPlanningInput::WholeBodyPoseDiagnostic local_diag;
  if (validateSegment(
          input.joint_segment_validator,
          q_from,
          q_to,
          input.safe_distance,
          &local_diag)) {
    if (diag != nullptr) {
      *diag = local_diag;
    }
    return true;
  }
  if (local_diag.failed_on_segment_sample &&
      local_diag.failed_segment_t <= 1.0e-9) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
    if (validateState(
            input.joint_state_validator,
            q_from,
            input.safe_distance,
            &state_diag)) {
      if (diag != nullptr) {
        *diag = local_diag;
      }
      return true;
    }
  }
  if (diag != nullptr) {
    *diag = local_diag;
  }
  return false;
}

std::string diagnosticSummary(
    const cp::PathPlanningInput::WholeBodyPoseDiagnostic& diag) {
  std::ostringstream out;
  out << (diag.reason.empty() ? "invalid" : diag.reason);
  if (!diag.worst_link_name.empty()) {
    out << ":" << diag.worst_link_name;
  }
  out << " margin=" << diag.min_margin
      << " safe=" << diag.safe_distance_used
      << " dist=" << diag.worst_distance
      << " req=" << diag.required_clearance;
  if (diag.failed_on_segment_sample) {
    out << " t=" << diag.failed_segment_t;
  }
  return out.str();
}

bool populateOutputFromStates(
    WholeBodyLocalPlanner::JointVectorList states,
    const Eigen::VectorXd& q_current,
    const double dt,
    const int repaired_states,
    const bool truncated_by_collision,
    const bool replanned_window,
    const bool used_optimized_trajectory,
    const bool used_frontend_fallback,
    WholeBodyLocalPlanner::Output* output) {
  if (output == nullptr || states.size() < 2u) {
    return false;
  }
  output->ok = true;
  output->used = true;
  output->joint_targets = std::move(states);
  output->dt_sec = dt;
  output->sampled_steps = static_cast<int>(output->joint_targets.size());
  output->repaired_states = repaired_states;
  output->truncated_by_collision = truncated_by_collision;
  output->replanned_window = replanned_window;
  output->used_optimized_trajectory = used_optimized_trajectory;
  output->used_frontend_fallback = used_frontend_fallback;
  const std::size_t next_index =
      std::min<std::size_t>(1u, output->joint_targets.size() - 1u);
  output->next_joint_target = output->joint_targets[next_index];
  output->next_joint_velocity =
      (output->next_joint_target - q_current) / dt;
  if (output->next_joint_velocity.size() >= kBaseDof) {
    output->next_joint_velocity[2] =
        normalizeAngle(output->next_joint_target[2] - q_current[2]) / dt;
  }
  return true;
}

bool validateWindowPath(
    const WholeBodyLocalPlanner::Input& input,
    const WholeBodyLocalPlanner::JointVectorList& states,
    std::string* reason = nullptr) {
  if (states.size() < 2u) {
    if (reason != nullptr) {
      *reason = "window_path_too_short";
    }
    return false;
  }
  for (std::size_t i = 0; i < states.size(); ++i) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
    const double state_safe_distance = i == 0u ? 0.0 : input.safe_distance;
    if (!validateState(
            input.joint_state_validator,
            states[i],
            state_safe_distance,
            &state_diag)) {
      if (reason != nullptr) {
        *reason = "state[" + std::to_string(i) + "]:" +
                  diagnosticSummary(state_diag);
        if (states[i].size() >= kBaseDof) {
          *reason += " base=(" + std::to_string(states[i][0]) + "," +
                     std::to_string(states[i][1]) + "," +
                     std::to_string(states[i][2]) + ")";
        }
        if (states[i].size() > kBaseDof) {
          *reason += " arm_norm=" +
                     std::to_string(states[i].tail(states[i].size() - kBaseDof).norm());
        }
      }
      return false;
    }
    if (i > 0u) {
      cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
      if (!validateForwardSegment(
              input, states[i - 1u], states[i], &segment_diag)) {
        if (reason != nullptr) {
          *reason = "segment[" + std::to_string(i - 1u) + "->" +
                    std::to_string(i) + "]:" +
                    diagnosticSummary(segment_diag);
          if (states[i - 1u].size() >= kBaseDof &&
              states[i].size() >= kBaseDof) {
            *reason += " from_base=(" +
                       std::to_string(states[i - 1u][0]) + "," +
                       std::to_string(states[i - 1u][1]) + "," +
                       std::to_string(states[i - 1u][2]) + ") to_base=(" +
                       std::to_string(states[i][0]) + "," +
                       std::to_string(states[i][1]) + "," +
                       std::to_string(states[i][2]) + ")";
          }
        }
        return false;
      }
    }
  }
  return true;
}

bool fillMissingLayerStates(
    const WholeBodyLocalPlanner::Input& input,
    const WholeBodyLocalPlanner::JointVectorList& refs,
    WholeBodyLocalPlanner::JointVectorList* states,
    std::vector<bool>* has_layer,
    int* repaired_states) {
  if (states == nullptr || has_layer == nullptr ||
      states->size() != refs.size() || has_layer->size() != refs.size()) {
    return false;
  }
  int repaired = 0;
  for (std::size_t layer = 0; layer < refs.size(); ++layer) {
    if ((*has_layer)[layer]) {
      continue;
    }

    std::size_t lower = layer;
    while (lower > 0u && !(*has_layer)[lower]) {
      --lower;
    }
    std::size_t upper = layer;
    while (upper + 1u < refs.size() && !(*has_layer)[upper]) {
      ++upper;
    }
    if (lower == layer || upper == layer || !(*has_layer)[lower] ||
        !(*has_layer)[upper] || upper <= lower) {
      return false;
    }

    const double ratio =
        static_cast<double>(layer - lower) / static_cast<double>(upper - lower);
    Eigen::VectorXd q =
        interpolateState((*states)[lower], (*states)[upper], ratio);
    q[0] = refs[layer][0];
    q[1] = refs[layer][1];
    q[2] = refs[layer][2];
    q = clampState(q, input.q_min, input.q_max);

    cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
    if (!validateState(input.joint_state_validator, q, input.safe_distance, &diag)) {
      return false;
    }
    (*states)[layer] = std::move(q);
    (*has_layer)[layer] = true;
    ++repaired;
  }
  if (repaired_states != nullptr) {
    *repaired_states += repaired;
  }
  return true;
}

WholeBodyLocalPlanner::JointVectorList sampleTrajectoryForValidation(
    const WholeBodyPolynomialTrajectory& trajectory,
    const std::size_t waypoint_count) {
  const std::size_t sample_count =
      std::max<std::size_t>(waypoint_count, std::max<std::size_t>(16u, waypoint_count * 3u));
  return trajectory.sampleUniform(sample_count);
}

bool validateTrajectoryWindow(
    const WholeBodyLocalPlanner::Input& input,
    const WholeBodyPolynomialTrajectory& trajectory,
    const std::size_t waypoint_count,
    std::string* reason = nullptr) {
  return validateWindowPath(
      input,
      sampleTrajectoryForValidation(trajectory, waypoint_count),
      reason);
}

WholeBodyLocalPlanner::JointVectorList sampleTrajectoryForOutput(
    const WholeBodyPolynomialTrajectory& trajectory,
    const std::size_t output_count) {
  WholeBodyLocalPlanner::JointVectorList samples =
      trajectory.sampleUniform(output_count);
  return samples;
}

bool trajectoryUsableForOutput(
    const WholeBodyLocalPlanner::Input& input,
    const WholeBodyPolynomialTrajectory& trajectory,
    const std::size_t output_count,
    std::string* reason = nullptr) {
  if (!validateTrajectoryWindow(input, trajectory, output_count, reason)) {
    return false;
  }
  return true;
}

Eigen::VectorXd armFromFullState(const Eigen::VectorXd& q) {
  return q.segment(kBaseDof, q.size() - kBaseDof);
}

bool buildFrontendWindow(
    const WholeBodyLocalPlanner::JointVectorList& dense_refs,
    const double dense_dt,
    WholeBodyLocalPlanner::JointVectorList* frontend_refs,
    std::vector<double>* frontend_segment_times) {
  if (frontend_refs == nullptr || frontend_segment_times == nullptr ||
      dense_refs.size() < 2u || !(dense_dt > 0.0)) {
    return false;
  }

  frontend_refs->clear();
  frontend_segment_times->clear();
  std::vector<std::size_t> selected_indices;
  selected_indices.reserve(dense_refs.size());
  for (std::size_t i = 0; i < dense_refs.size(); i += kFrontendLayerStride) {
    selected_indices.push_back(i);
  }
  if (selected_indices.empty() || selected_indices.back() != dense_refs.size() - 1u) {
    selected_indices.push_back(dense_refs.size() - 1u);
  }
  if (selected_indices.size() < 2u) {
    return false;
  }

  frontend_refs->reserve(selected_indices.size());
  for (const std::size_t index : selected_indices) {
    frontend_refs->push_back(dense_refs[index]);
  }
  frontend_segment_times->reserve(selected_indices.size() - 1u);
  for (std::size_t i = 1u; i < selected_indices.size(); ++i) {
    const std::size_t delta_index = selected_indices[i] - selected_indices[i - 1u];
    frontend_segment_times->push_back(
        dense_dt * static_cast<double>(std::max<std::size_t>(1u, delta_index)));
  }
  return frontend_segment_times->size() + 1u == frontend_refs->size();
}

bool buildLocalGapInput(
    const WholeBodyLocalPlanner::Input& input,
    const WholeBodyLocalPlanner::JointVectorList& refs,
    const std::vector<double>& segment_times,
    cp::PathPlanningInput* planning_input,
    gp::LayerGapRrtConnector::Input* gap_input) {
  if (planning_input == nullptr || gap_input == nullptr ||
      refs.size() < 2u || refs.front().size() <= kBaseDof ||
      segment_times.size() + 1u != refs.size()) {
    return false;
  }

  *planning_input = cp::PathPlanningInput{};
  planning_input->q_min = input.q_min;
  planning_input->q_max = input.q_max;
  planning_input->joint_state_validator = input.joint_state_validator;
  planning_input->joint_segment_validator = input.joint_segment_validator;
  planning_input->manipulator_state_validator = input.joint_state_validator;
  planning_input->manipulator_segment_validator = input.joint_segment_validator;

  *gap_input = gp::LayerGapRrtConnector::Input{};
  gap_input->planning_input = planning_input;
  gap_input->arm_start = armFromFullState(refs.front());
  gap_input->arm_goal = armFromFullState(refs.back());
  gap_input->safe_distance = input.safe_distance;
  gap_input->route.reserve(refs.size());
  gap_input->segment_times.reserve(refs.size() - 1u);
  gap_input->segment_check_states.reserve(refs.size() - 1u);
  for (const Eigen::VectorXd& q : refs) {
    gp::LayerGapRrtConnector::BaseState base;
    base.x = q[0];
    base.y = q[1];
    base.yaw = q[2];
    gap_input->route.push_back(base);
  }
  for (std::size_t i = 1u; i < refs.size(); ++i) {
    gap_input->segment_times.push_back(
        std::max(kMinDtSec, segment_times[i - 1u]));
    std::vector<gp::LayerGapRrtConnector::BaseState> checks;
    checks.reserve(3u);
    checks.push_back(gap_input->route[i - 1u]);
    gp::LayerGapRrtConnector::BaseState mid;
    mid.x = 0.5 * (gap_input->route[i - 1u].x + gap_input->route[i].x);
    mid.y = 0.5 * (gap_input->route[i - 1u].y + gap_input->route[i].y);
    mid.yaw = normalizeAngle(
        gap_input->route[i - 1u].yaw +
        0.5 * normalizeAngle(gap_input->route[i].yaw -
                             gap_input->route[i - 1u].yaw));
    checks.push_back(mid);
    checks.push_back(gap_input->route[i]);
    gap_input->segment_check_states.push_back(std::move(checks));
  }
  return true;
}

bool findRepairSeedLayers(
    const WholeBodyLocalPlanner::Input& input,
    const WholeBodyLocalPlanner::JointVectorList& refs,
    int* prefix_end,
    int* suffix_start,
    std::string* reason) {
  if (prefix_end == nullptr || suffix_start == nullptr || refs.size() < 2u) {
    return false;
  }

  *prefix_end = -1;
  for (std::size_t i = 0; i < refs.size(); ++i) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
    const double state_safe_distance = i == 0u ? 0.0 : input.safe_distance;
    if (!validateState(
            input.joint_state_validator,
            refs[i],
            state_safe_distance,
            &state_diag)) {
      if (reason != nullptr && reason->empty()) {
        *reason = "prefix state[" + std::to_string(i) + "] " +
                  diagnosticSummary(state_diag);
      }
      break;
    }
    if (i > 0u) {
      cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
      if (!validateForwardSegment(input, refs[i - 1u], refs[i], &segment_diag)) {
        if (reason != nullptr && reason->empty()) {
          *reason = "prefix segment[" + std::to_string(i - 1u) + "->" +
                    std::to_string(i) + "] " +
                    diagnosticSummary(segment_diag);
        }
        break;
      }
    }
    *prefix_end = static_cast<int>(i);
  }

  *suffix_start = static_cast<int>(refs.size());
  for (int i = static_cast<int>(refs.size()) - 1; i >= 0; --i) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
    const double state_safe_distance = i == 0 ? 0.0 : input.safe_distance;
    if (!validateState(
            input.joint_state_validator,
            refs[static_cast<std::size_t>(i)],
            state_safe_distance,
            &state_diag)) {
      if (reason != nullptr && reason->empty()) {
        *reason = "suffix state[" + std::to_string(i) + "] " +
                  diagnosticSummary(state_diag);
      }
      break;
    }
    if (i + 1 < static_cast<int>(refs.size())) {
      cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
      if (!validateForwardSegment(
              input,
              refs[static_cast<std::size_t>(i)],
              refs[static_cast<std::size_t>(i + 1)],
              &segment_diag)) {
        if (reason != nullptr && reason->empty()) {
          *reason = "suffix segment[" + std::to_string(i) + "->" +
                    std::to_string(i + 1) + "] " +
                    diagnosticSummary(segment_diag);
        }
        break;
      }
    }
    *suffix_start = i;
  }

  if (*prefix_end < 0 ||
      *suffix_start >= static_cast<int>(refs.size()) ||
      *prefix_end >= *suffix_start) {
    if (reason != nullptr && reason->empty()) {
      *reason = "no separated valid prefix/suffix";
    }
    return false;
  }
  return true;
}

bool repairLocalWindowWithGapRrt(
    const WholeBodyLocalPlanner::Input& input,
    const WholeBodyLocalPlanner::Config& cfg,
    const WholeBodyLocalPlanner::JointVectorList& refs,
    const std::vector<double>& segment_times,
    WholeBodyLocalPlanner::JointVectorList* repaired_states,
    int* repaired_count,
    std::string* reason) {
  if (repaired_states == nullptr || refs.size() < 3u ||
      refs.front().size() <= kBaseDof ||
      segment_times.size() + 1u != refs.size()) {
    if (reason != nullptr) {
      *reason = "repair input invalid";
    }
    return false;
  }

  int prefix_end = -1;
  int suffix_start = static_cast<int>(refs.size());
  std::string seed_reason;
  if (!findRepairSeedLayers(
          input, refs, &prefix_end, &suffix_start, &seed_reason)) {
    if (reason != nullptr) {
      *reason = seed_reason;
    }
    return false;
  }

  cp::PathPlanningInput planning_input;
  gp::LayerGapRrtConnector::Input gap_input;
  if (!buildLocalGapInput(input, refs, segment_times, &planning_input, &gap_input)) {
    if (reason != nullptr) {
      *reason = "repair gap input build failed";
    }
    return false;
  }

  for (int i = 0; i <= prefix_end; ++i) {
    gap_input.start_seeds.push_back(
        {armFromFullState(refs[static_cast<std::size_t>(i)]), i});
  }
  for (int i = suffix_start; i < static_cast<int>(refs.size()); ++i) {
    gap_input.goal_seeds.push_back(
        {armFromFullState(refs[static_cast<std::size_t>(i)]), i});
  }

  gp::LayerGapRrtConnector::Config repair_cfg;
  repair_cfg.max_iterations = 1600;
  repair_cfg.goal_bias = 0.40;
  repair_cfg.max_joint_velocity_rad_per_sec =
      std::max(0.0, cfg.max_arm_qdot);
  repair_cfg.time_scale = 3.0;
  repair_cfg.meet_tolerance_rad = 1.0e-4;
  repair_cfg.require_complete_endpoint_path = false;
  gp::LayerGapRrtConnector connector(repair_cfg);
  const gp::LayerGapRrtConnector::Result gap_result =
      connector.connect(gap_input);
  if (!gap_result.success || gap_result.path.size() < 2u ||
      gap_result.path.size() != gap_result.path_layers.size()) {
    if (reason != nullptr) {
      *reason = "repair gap-RRT failed: iterations=" +
                std::to_string(gap_result.iterations) +
                " nodes=" + std::to_string(gap_result.node_count) +
                " extend_rejects=" +
                std::to_string(gap_result.extend_rejects) +
                " top_extend=" + gap_result.top_extend_reject_reason +
                " prefix_end=" + std::to_string(prefix_end) +
                " suffix_start=" + std::to_string(suffix_start);
    }
    return false;
  }

  const int gap_start_layer = gap_result.path_layers.front();
  const int gap_end_layer = gap_result.path_layers.back();
  if (gap_start_layer < 0 || gap_end_layer < gap_start_layer ||
      gap_start_layer > prefix_end || gap_end_layer < suffix_start ||
      gap_end_layer >= static_cast<int>(refs.size())) {
    if (reason != nullptr) {
      *reason = "repair gap-RRT produced inconsistent layers start=" +
                std::to_string(gap_start_layer) +
                " end=" + std::to_string(gap_end_layer) +
                " prefix_end=" + std::to_string(prefix_end) +
                " suffix_start=" + std::to_string(suffix_start);
    }
    return false;
  }

  WholeBodyLocalPlanner::JointVectorList states(
      refs.size(), Eigen::VectorXd{});
  std::vector<bool> has_layer(refs.size(), false);
  for (int i = 0; i < gap_start_layer; ++i) {
    states[static_cast<std::size_t>(i)] = refs[static_cast<std::size_t>(i)];
    has_layer[static_cast<std::size_t>(i)] = true;
  }
  for (std::size_t path_index = 0; path_index < gap_result.path.size();
       ++path_index) {
    const int layer = gap_result.path_layers[path_index];
    if (layer < 0 || layer >= static_cast<int>(refs.size()) ||
        gap_result.path[path_index].size() + kBaseDof != refs.front().size()) {
      if (reason != nullptr) {
        *reason = "repair gap-RRT returned invalid path state";
      }
      return false;
    }
    Eigen::VectorXd q = refs[static_cast<std::size_t>(layer)];
    q.segment(kBaseDof, gap_result.path[path_index].size()) =
        gap_result.path[path_index];
    states[static_cast<std::size_t>(layer)] =
        clampState(q, input.q_min, input.q_max);
    has_layer[static_cast<std::size_t>(layer)] = true;
  }
  for (int i = gap_end_layer + 1; i < static_cast<int>(refs.size()); ++i) {
    states[static_cast<std::size_t>(i)] = refs[static_cast<std::size_t>(i)];
    has_layer[static_cast<std::size_t>(i)] = true;
  }

  int interpolated_repairs = 0;
  if (!fillMissingLayerStates(
          input, refs, &states, &has_layer, &interpolated_repairs)) {
    if (reason != nullptr) {
      *reason = "repair path missing unrecoverable layer";
    }
    return false;
  }

  std::string validation_reason;
  if (!validateWindowPath(input, states, &validation_reason)) {
    if (reason != nullptr) {
      *reason = "repair window invalid (" + validation_reason + ")";
    }
    return false;
  }

  if (repaired_count != nullptr) {
    *repaired_count += std::max(0, gap_end_layer - gap_start_layer - 1) +
                       interpolated_repairs;
  }
  *repaired_states = std::move(states);
  return true;
}

Eigen::VectorXd withArmAtBase(
    const Eigen::VectorXd& base_state,
    const Eigen::VectorXd& arm_source) {
  Eigen::VectorXd q = base_state;
  if (q.size() == arm_source.size() && q.size() > kBaseDof) {
    q.segment(kBaseDof, q.size() - kBaseDof) =
        arm_source.segment(kBaseDof, arm_source.size() - kBaseDof);
  }
  return q;
}

bool findSafeWindowGoal(
    const WholeBodyLocalPlanner::Input& input,
    const WholeBodyLocalPlanner::Config& cfg,
    const WholeBodyLocalPlanner::JointVectorList& refs,
    Eigen::VectorXd* goal) {
  if (goal == nullptr || refs.empty() || input.q_current.size() <= kBaseDof) {
    return false;
  }

  const Eigen::VectorXd base_goal = refs.back();
  auto tryCandidate = [&](Eigen::VectorXd candidate) {
    if (candidate.size() != base_goal.size()) {
      return false;
    }
    candidate[0] = base_goal[0];
    candidate[1] = base_goal[1];
    candidate[2] = base_goal[2];
    candidate = clampState(candidate, input.q_min, input.q_max);
    if (validateState(input.joint_state_validator, candidate, input.safe_distance)) {
      *goal = std::move(candidate);
      return true;
    }
    return false;
  };

  if (tryCandidate(base_goal)) {
    return true;
  }

  if (tryCandidate(withArmAtBase(base_goal, input.q_current))) {
    return true;
  }

  for (const Eigen::VectorXd& ref : refs) {
    if (tryCandidate(withArmAtBase(base_goal, ref))) {
      return true;
    }
  }

  const int samples = std::max(12, 4 * std::max(1, cfg.collision_repair_samples));
  for (int i = 1; i <= samples; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples);
    Eigen::VectorXd candidate = base_goal;
    candidate.segment(kBaseDof, candidate.size() - kBaseDof) =
        (1.0 - ratio) *
            input.q_current.segment(kBaseDof, input.q_current.size() - kBaseDof) +
        ratio * base_goal.segment(kBaseDof, base_goal.size() - kBaseDof);
    if (tryCandidate(candidate)) {
      return true;
    }
  }

  std::uint32_t seed = 2166136261u;
  for (Eigen::Index i = 0; i < input.q_current.size(); ++i) {
    seed ^= static_cast<std::uint32_t>(
        std::llround((input.q_current[i] + 13.0) * 10000.0));
    seed *= 16777619u;
    seed ^= static_cast<std::uint32_t>(
        std::llround((base_goal[i] + 17.0) * 10000.0));
    seed *= 16777619u;
  }
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
  const double max_window_arm_delta =
      std::max(0.0, cfg.max_arm_qdot) *
      std::max(kMinDtSec, cfg.dt_sec) *
      static_cast<double>(std::max<std::size_t>(1u, refs.size() - 1u));
  for (int attempt = 0; attempt < 80; ++attempt) {
    Eigen::VectorXd candidate = base_goal;
    for (Eigen::Index joint = kBaseDof; joint < candidate.size(); ++joint) {
      const bool has_bounds =
          input.q_min.size() == candidate.size() &&
          input.q_max.size() == candidate.size();
      if (has_bounds) {
        const double lo = std::min(input.q_min[joint], input.q_max[joint]);
        const double hi = std::max(input.q_min[joint], input.q_max[joint]);
        if (std::isfinite(lo) && std::isfinite(hi) && hi > lo) {
          candidate[joint] = lo + unit_dist(rng) * (hi - lo);
          continue;
        }
      }
      candidate[joint] =
          0.5 * (input.q_current[joint] + base_goal[joint]) +
          (unit_dist(rng) - 0.5) * 1.0;
      candidate[joint] = std::clamp(
          candidate[joint],
          input.q_current[joint] - max_window_arm_delta,
          input.q_current[joint] + max_window_arm_delta);
    }
    if (tryCandidate(candidate)) {
      return true;
    }
  }

  return false;
}

bool computeWindowReplan(
    const WholeBodyLocalPlanner::Input& input,
    const WholeBodyLocalPlanner::Config& cfg,
    WholeBodyLocalPlanner::Output* output) {
  if (output == nullptr || input.q_current.size() <= kBaseDof ||
      input.global_reference.empty() ||
      !input.joint_state_validator ||
      !input.joint_segment_validator) {
    return false;
  }

  WholeBodyLocalTargetSelector target_selector(
      WholeBodyLocalTargetSelector::Config{
          std::max(3, cfg.horizon_steps),
          std::max(kMinDtSec, cfg.dt_sec)});
  WholeBodyLocalTargetSelector::Input target_input;
  target_input.q_current = input.q_current;
  target_input.global_reference = input.global_reference;
  target_input.global_time_sec = input.global_time_sec;
  target_input.q_min = input.q_min;
  target_input.q_max = input.q_max;
  WholeBodyLocalTargetSelector::Output target_output;
  if (!target_selector.select(target_input, &target_output) ||
      target_output.references.size() < 3u) {
    output->error = "local target selection failed";
    return false;
  }
  const double dt = target_output.dt_sec;
  WholeBodyLocalPlanner::JointVectorList dense_refs;
  dense_refs.reserve(target_output.references.size());
  dense_refs.push_back(clampState(
      target_output.references.front(), input.q_min, input.q_max));
  for (std::size_t i = 1u; i < target_output.references.size(); ++i) {
    dense_refs.push_back(clampState(
        clampStepByDynamics(dense_refs.back(), target_output.references[i], cfg, dt),
        input.q_min,
        input.q_max));
  }

  WholeBodyLocalPlanner::JointVectorList refs;
  std::vector<double> frontend_segment_times;
  if (!buildFrontendWindow(
          dense_refs, dt, &refs, &frontend_segment_times)) {
    output->error = "local frontend window construction failed";
    return false;
  }

  cp::PathPlanningInput::WholeBodyPoseDiagnostic start_diag;
  if (!validateState(
          input.joint_state_validator,
          refs.front(),
          input.safe_distance,
          &start_diag)) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic hard_start_diag;
    if (!validateState(
            input.joint_state_validator,
            refs.front(),
            0.0,
            &hard_start_diag)) {
      output->error =
          "local start reference invalid: " +
          (hard_start_diag.reason.empty() ? std::string("invalid")
                                          : hard_start_diag.reason);
      return false;
    }
  }
  Eigen::VectorXd window_goal = refs.back();
  if (!findSafeWindowGoal(input, cfg, refs, &window_goal)) {
    output->error = "no safe local window goal";
    return false;
  }
  const double goal_segment_dt =
      frontend_segment_times.empty() ? dt : frontend_segment_times.back();
  refs.back() =
      clampStepByDynamics(refs[refs.size() - 2u], window_goal, cfg, goal_segment_dt);
  refs.back() = clampState(refs.back(), input.q_min, input.q_max);
  if (!validateState(input.joint_state_validator, refs.back(), input.safe_distance) &&
      !findSafeWindowGoal(input, cfg, refs, &refs.back())) {
    output->error = "dynamic-clamped local window goal invalid";
    return false;
  }

  gp::LayerGapRrtConnector::Config gap_cfg;
  gap_cfg.max_iterations = 800;
  gap_cfg.goal_bias = 0.50;
  gap_cfg.max_joint_velocity_rad_per_sec =
      std::max(0.0, cfg.max_arm_qdot);
  gap_cfg.time_scale = 3.0;
  gap_cfg.meet_tolerance_rad = 1.0e-4;
  gap_cfg.require_complete_endpoint_path = true;
  gp::LayerGapRrtConnector connector(gap_cfg);

  cp::PathPlanningInput planning_input;
  gp::LayerGapRrtConnector::Input gap_input;
  if (!buildLocalGapInput(
          input, refs, frontend_segment_times, &planning_input, &gap_input)) {
    output->error = "local LayerGapRRT input build failed";
    return false;
  }

  gap_input.start_seeds.push_back({gap_input.arm_start, 0});
  gap_input.goal_seeds.push_back(
      {gap_input.arm_goal, static_cast<int>(refs.size() - 1u)});

  const gp::LayerGapRrtConnector::Result result =
      connector.connect(gap_input);
  WholeBodyLocalPlanner::JointVectorList states;
  int repaired_states = 0;
  auto repairFromRefs = [&](const std::string& error) {
    std::string repair_reason;
    if (!repairLocalWindowWithGapRrt(
            input,
            cfg,
            refs,
            frontend_segment_times,
            &states,
            &repaired_states,
            &repair_reason)) {
      output->error = error + "; REMANI-style gap repair failed (" +
                      repair_reason + ")";
      return false;
    }
    return true;
  };

  if (!result.success || result.path.size() < 2u ||
      result.path.size() != result.path_layers.size()) {
    const std::string error =
        (result.success ? "local LayerGapRRT returned inconsistent path"
                        : "local LayerGapRRT failed") +
        std::string(": iterations=") +
        std::to_string(result.iterations) +
        " nodes=" + std::to_string(result.node_count) +
        " extend_rejects=" + std::to_string(result.extend_rejects) +
        " top_extend=" + result.top_extend_reject_reason;
    if (!repairFromRefs(error)) {
      return false;
    }
  } else {
    states.assign(refs.size(), Eigen::VectorXd{});
    std::vector<bool> has_layer(refs.size(), false);
    bool path_ok = true;
    for (std::size_t path_index = 0; path_index < result.path.size();
         ++path_index) {
      const Eigen::VectorXd& arm = result.path[path_index];
      const int layer = result.path_layers[path_index];
      if (arm.size() + kBaseDof != input.q_current.size() ||
          layer < 0 || layer >= static_cast<int>(refs.size())) {
        path_ok = false;
        break;
      }
      Eigen::VectorXd q = refs[static_cast<std::size_t>(layer)];
      q.segment(kBaseDof, arm.size()) = arm;
      states[static_cast<std::size_t>(layer)] = std::move(q);
      has_layer[static_cast<std::size_t>(layer)] = true;
    }
    if (!path_ok ||
        !fillMissingLayerStates(
            input, refs, &states, &has_layer, &repaired_states)) {
      if (!repairFromRefs(
              path_ok
                  ? "local LayerGapRRT path missing unrecoverable layer"
                  : "local LayerGapRRT returned dimension mismatch")) {
        return false;
      }
    }
  }
  for (std::size_t i = 0; i < states.size(); ++i) {
    states[i][0] = refs[i][0];
    states[i][1] = refs[i][1];
    states[i][2] = refs[i][2];
    states[i] = clampState(states[i], input.q_min, input.q_max);
  }
  std::string validation_reason;
  if (!validateWindowPath(input, states, &validation_reason)) {
    output->error =
        "repaired local LayerGapRRT window invalid (" + validation_reason + ")";
    return false;
  }

  WholeBodyPolynomialTrajectory initial_trajectory;
  WholeBodyFrontendInitializer initializer;
  if (!initializer.initialize(refs, states, dt, &initial_trajectory)) {
    if (!initial_trajectory.reset(states, frontend_segment_times)) {
      output->error = "whole-body frontend initializer failed";
      return false;
    }
  } else if (!initial_trajectory.reset(states, frontend_segment_times)) {
    output->error = "whole-body frontend initializer retiming failed";
    return false;
  }

  WholeBodyLbfgsOptimizer::Config opt_cfg;
  opt_cfg.max_iterations = std::max(0, cfg.optimization_iterations);
  opt_cfg.reference_weight = cfg.reference_weight;
  opt_cfg.base_reference_weight = cfg.base_reference_weight;
  opt_cfg.yaw_reference_weight = cfg.yaw_reference_weight;
  opt_cfg.base_progress_weight = cfg.base_progress_weight;
  opt_cfg.smoothness_weight = cfg.smoothness_weight;
  opt_cfg.velocity_weight = cfg.current_state_weight;
  opt_cfg.collision_weight = cfg.collision_weight;
  opt_cfg.time_weight = cfg.time_weight;
  opt_cfg.obstacle_safe_margin = cfg.obstacle_safe_margin;
  opt_cfg.constrain_points_per_piece = cfg.constrain_points_per_piece;
  opt_cfg.max_base_vx = cfg.max_base_vx;
  opt_cfg.max_base_vy = cfg.max_base_vy;
  opt_cfg.max_base_wz = cfg.max_base_wz;
  opt_cfg.max_arm_qdot = cfg.max_arm_qdot;
  WholeBodyLbfgsOptimizer optimizer(opt_cfg);

  WholeBodyLbfgsOptimizer::Input opt_input;
  opt_input.initial_trajectory = initial_trajectory;
  opt_input.references = refs;
  if (refs.size() >= 2u && refs.front().size() >= kBaseDof &&
      refs.back().size() >= kBaseDof) {
    opt_input.base_progress_direction = refs.back().head<2>() -
                                        refs.front().head<2>();
  }
  opt_input.base_progress_tolerance =
      0.5 * std::max(0.0, cfg.max_base_vx) * std::max(kMinDtSec, dt);
  opt_input.q_min = input.q_min;
  opt_input.q_max = input.q_max;
  opt_input.safe_distance = input.safe_distance;
  opt_input.state_validator = input.joint_state_validator;
  opt_input.segment_validator = input.joint_segment_validator;
  opt_input.collision_cost_gradient_fn = input.collision_cost_gradient_fn;
  WholeBodyLbfgsOptimizer::Output opt_output;
  if (!optimizer.optimize(opt_input, &opt_output)) {
    output->error =
        "MINCO/LBFGS optimization failed after REMANI-style frontend repair";
    return false;
  }

  WholeBodyLocalPlanner::JointVectorList optimized_states =
      sampleTrajectoryForOutput(opt_output.trajectory, dense_refs.size());
  std::string optimized_reason;
  if (!trajectoryUsableForOutput(
          input, opt_output.trajectory, dense_refs.size(), &optimized_reason)) {
    output->error =
        "optimized MINCO trajectory invalid after REMANI-style frontend repair (" +
        optimized_reason + ")";
    return false;
  }

  return populateOutputFromStates(
      std::move(optimized_states),
      input.q_current,
      opt_output.trajectory.duration() /
          static_cast<double>(
              std::max<std::size_t>(1u, dense_refs.size() - 1u)),
      repaired_states,
      false,
      true,
      true,
      false,
      output);
}

}  // namespace

WholeBodyLocalPlanner::WholeBodyLocalPlanner()
    : WholeBodyLocalPlanner(Config{}) {}

WholeBodyLocalPlanner::WholeBodyLocalPlanner(Config cfg)
    : cfg_(cfg) {}

void WholeBodyLocalPlanner::configure(Config cfg) {
  cfg_ = cfg;
}

bool WholeBodyLocalPlanner::compute(const Input& input, Output* output) const {
  if (output == nullptr) {
    return false;
  }
  *output = Output{};

  if (input.q_current.size() <= 0 || !input.q_current.allFinite()) {
    output->error = "invalid current whole-body state";
    return false;
  }
  if (input.global_reference.empty()) {
    output->error = "empty global whole-body reference";
    return false;
  }
  if (input.q_min.size() != input.q_current.size() ||
      input.q_max.size() != input.q_current.size()) {
    output->error = "whole-body bounds dimension mismatch";
    return false;
  }

  if (computeWindowReplan(input, cfg_, output)) {
    return true;
  }
  const std::string primary_error = output->error.empty()
      ? std::string(
            "whole-body REMANI-style local planner failed during target "
            "selection, frontend initialization, or LBFGS polynomial optimization")
      : output->error;

  Config recovery_cfg = cfg_;
  recovery_cfg.horizon_steps =
      std::max(cfg_.horizon_steps + 8, 2 * std::max(3, cfg_.horizon_steps));
  recovery_cfg.optimization_iterations =
      std::max(cfg_.optimization_iterations + 16, cfg_.optimization_iterations * 2);
  recovery_cfg.reference_weight = std::min(cfg_.reference_weight, 0.005);
  recovery_cfg.base_reference_weight = std::min(cfg_.base_reference_weight, 1.5);
  recovery_cfg.yaw_reference_weight = std::min(cfg_.yaw_reference_weight, 0.3);
  recovery_cfg.base_progress_weight = cfg_.base_progress_weight;
  recovery_cfg.collision_weight = std::max(cfg_.collision_weight, 650000.0);
  recovery_cfg.obstacle_safe_margin =
      std::max(cfg_.obstacle_safe_margin, 0.20);
  recovery_cfg.constrain_points_per_piece =
      std::max(cfg_.constrain_points_per_piece, 24);

  Output recovery_output;
  if (computeWindowReplan(input, recovery_cfg, &recovery_output)) {
    *output = std::move(recovery_output);
    return true;
  }
  output->error = primary_error + "; long-horizon recovery failed (" +
                  (recovery_output.error.empty()
                       ? std::string("no detailed recovery error")
                       : recovery_output.error) +
                  ")";
  return false;
}

}  // namespace arm_controller::controller::reactive_task
