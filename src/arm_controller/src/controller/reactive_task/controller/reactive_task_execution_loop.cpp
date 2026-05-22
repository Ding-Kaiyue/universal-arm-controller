#include "controller/reactive_task/reactive_task_controller.hpp"

#include "controller/reactive_task/global_planner/reactive_task_planning_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <sstream>
#include <thread>
#include <utility>

namespace rq = arm_controller::algorithm::reactive_qp;
namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace rt = arm_controller::controller::reactive_task;
namespace planning_detail = arm_controller::controller::reactive_task::planning_detail;

namespace {
constexpr int kHoldRetryTicks = 8;
constexpr bool kReactiveObstacleAvoidanceEnabled = true;
constexpr double kTrackQdotBlend = 0.20;
constexpr double kRecoveryQdotBlend = 0.65;
constexpr double kTerminalQdotBlend = 0.12;
constexpr double kTrackQdotDeltaLimit = 0.16;
constexpr double kRecoveryQdotDeltaLimit = 0.22;
constexpr double kTerminalQdotDeltaLimit = 0.06;
constexpr double kRecoveryMinJointSpeed = 0.14;
constexpr double kReferenceFullSpeedPositionError = 0.035;
constexpr double kReferenceFreezePositionError = 0.090;
constexpr double kReferenceFullSpeedOrientationError = 0.12;
constexpr double kReferenceFreezeOrientationError = 0.35;
constexpr double kObstacleStallReplanProgressLimit = 0.985;
constexpr double kObstacleStallReplanGuidanceGate = 0.85;

std::string formatVector(const Eigen::VectorXd &v) {
  std::ostringstream oss;
  oss << v.transpose().format(Eigen::IOFormat(4, 0, ", ", ", ", "[", "]"));
  return oss.str();
}

double progressGateFromError(const double error,
                             const double full_speed_error,
                             const double freeze_error) {
  if (!std::isfinite(error)) {
    return 0.0;
  }
  if (error <= full_speed_error) {
    return 1.0;
  }
  if (error >= freeze_error) {
    return 0.0;
  }
  const double x =
      (freeze_error - error) / std::max(1e-6, freeze_error - full_speed_error);
  return std::clamp(x, 0.0, 1.0);
}

struct LocalPlannerStartState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::VectorXd q;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};

struct LocalPlannerBuildResult {
  rt::ArmLocalPlanner::Input input;
  rt::ObstacleSelectionOutput obstacle_selection;
};

LocalPlannerStartState makeLocalPlannerStartState(
    const Eigen::VectorXd &q_now,
    const Eigen::Isometry3d &current_pose,
    const bool previous_qdot_reference_valid,
    const Eigen::VectorXd &previous_qdot_reference,
    const double prediction_dt,
    const rq::JointLimitData &joint_limits,
    const std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics>
        &fk_provider) {
  LocalPlannerStartState start;
  start.q = q_now;
  start.pose = current_pose;

  if (previous_qdot_reference_valid &&
      previous_qdot_reference.size() == q_now.size()) {
    start.q = q_now + prediction_dt * previous_qdot_reference;
    start.q = start.q.cwiseMax(joint_limits.q_min).cwiseMin(joint_limits.q_max);
  }

  if (!fk_provider) {
    return start;
  }

  static const std::vector<std::string> kNoLinkPoseQueries;
  arm_controller::kinematics::LinkPoseResultList predicted_link_poses;
  Eigen::Isometry3d predicted_pose = Eigen::Isometry3d::Identity();
  if (fk_provider->computeLinkPoses(
          start.q,
          kNoLinkPoseQueries,
          predicted_link_poses,
          nullptr,
          nullptr,
          &predicted_pose) &&
      predicted_pose.matrix().allFinite()) {
    start.pose = predicted_pose;
  } else {
    start.q = q_now;
    start.pose = current_pose;
  }

  return start;
}

cp::TimedCartesianSampleList sampleLocalPlannerReference(
    const cp::GlobalTrajectoryManager *global_trajectory,
    const double continuous_sample_time_sec,
    const double prediction_dt,
    const int horizon_steps,
    const double dt_sec) {
  cp::TimedCartesianSampleList samples;
  if (global_trajectory == nullptr) {
    return samples;
  }

  const int sample_count = std::max(1, horizon_steps);
  samples.reserve(static_cast<std::size_t>(sample_count));
  for (int k = 0; k < sample_count; ++k) {
    const double sample_time =
        continuous_sample_time_sec + prediction_dt +
        dt_sec * static_cast<double>(k + 1);
    cp::TimedCartesianSample sample;
    if (!global_trajectory->sampleByElapsedTime(sample_time, sample)) {
      break;
    }
    samples.push_back(sample);
  }
  return samples;
}

rt::ArmLocalPlanner::Input::JointToPoseFn makeJointToPoseFn(
    const std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics>
        &fk_provider) {
  return [fk = fk_provider](const Eigen::VectorXd &q,
                            Eigen::Isometry3d *pose) -> bool {
    if (!fk || pose == nullptr) {
      return false;
    }
    static const std::vector<std::string> kNoLinkPoseQueries;
    arm_controller::kinematics::LinkPoseResultList link_poses;
    Eigen::Isometry3d ee_pose = Eigen::Isometry3d::Identity();
    if (!fk->computeLinkPoses(
            q,
            kNoLinkPoseQueries,
            link_poses,
            nullptr,
            nullptr,
            &ee_pose)) {
      return false;
    }
    *pose = ee_pose;
    return true;
  };
}

LocalPlannerBuildResult buildLocalPlannerInput(
    const std::string &mapping,
    const cp::GlobalTrajectoryManager *global_trajectory,
    const rt::ArmLocalPlanner::Config &local_planner_cfg,
    const rt::ArmState &arm_state,
    const arm_controller::kinematics::ForwardKinematicsOutput &fk_now,
    const rt::LocalReferenceOutput &local_reference_output,
    const bool reference_finished,
    const double prediction_dt,
    const bool previous_qdot_reference_valid,
    const Eigen::VectorXd &previous_qdot_reference,
    const std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics>
        &fk_provider,
    const rq::LinkCollisionEllipsoidList &collision_ellipsoids,
    const std::string &robot_type,
    const std::string &planning_group,
    const std::string &base_link,
    const std::string &tip_link,
    const std::string &urdf_path,
    const std::string &srdf_path,
    const rt::ReactiveTaskObstacleSelector &obstacle_selector,
    const std::shared_ptr<cp::CameraDriverPointcloudMapAdapter>
        &pointcloud_map) {
  LocalPlannerBuildResult result;
  const Eigen::VectorXd &q_now = arm_state.q;

  const LocalPlannerStartState start = makeLocalPlannerStartState(
      q_now,
      fk_now.ee_pose,
      previous_qdot_reference_valid,
      previous_qdot_reference,
      prediction_dt,
      arm_state.joint_limits,
      fk_provider);

  result.input.current_pose = start.pose;
  result.input.reference_samples = sampleLocalPlannerReference(
      global_trajectory,
      local_reference_output.continuous_sample_time_sec,
      prediction_dt,
      local_planner_cfg.horizon_steps,
      local_planner_cfg.dt_sec);
  result.input.reference_finished = reference_finished;
  result.input.joint_names = arm_state.joint_names;
  result.input.q_current = start.q;
  result.input.q_goal = start.q;
  result.input.robot_type = robot_type;
  result.input.planning_group = planning_group;
  result.input.base_link = base_link;
  result.input.tip_link = tip_link;
  result.input.urdf_path = urdf_path;
  result.input.srdf_path = srdf_path;
  result.input.joint_to_pose = makeJointToPoseFn(fk_provider);

  rt::ObstacleSelectionInput obstacle_input;
  obstacle_input.enabled = kReactiveObstacleAvoidanceEnabled;
  obstacle_input.obstacle_name_prefix = "camera_obstacle_" + mapping;
  obstacle_input.pointcloud_map = pointcloud_map;
  obstacle_input.obstacle_padding_m = local_planner_cfg.obstacle_padding_m;
  obstacle_input.obstacle_selection_radius_m =
      local_planner_cfg.obstacle_selection_radius_m;
  obstacle_input.max_obstacle_spheres =
      local_planner_cfg.max_obstacle_spheres;
  obstacle_input.current_pose = fk_now.ee_pose;
  obstacle_input.reference_samples = result.input.reference_samples;
  obstacle_input.fk_provider = fk_provider;
  obstacle_input.collision_ellipsoids = &collision_ellipsoids;
  obstacle_input.current_link_poses = &fk_now.link_poses;
  result.obstacle_selection = obstacle_selector.select(obstacle_input);
  result.input.sphere_obstacles =
      std::move(result.obstacle_selection.obstacles);

  if (!result.input.reference_samples.empty()) {
    const cp::TimedCartesianSample &lookahead_sample =
        result.input.reference_samples.back();
    if (lookahead_sample.has_ik_joint_target &&
        lookahead_sample.ik_joint_target.size() == q_now.size() &&
        lookahead_sample.ik_joint_target.allFinite()) {
      result.input.q_goal = lookahead_sample.ik_joint_target;
      result.input.q_goal_valid = true;
    }
  }

  return result;
}
} // namespace

using planning_detail::orientationErrorRad;

bool ReactiveTaskController::runPlanningControlLoop(
    const std::string &mapping, const PlanningSession &session,
    PlanningRuntime *runtime) {
  if (runtime == nullptr || !runtime->global_trajectory) {
    return false;
  }

  const int max_neo_iterations =
      std::max(runtime->exec_ctx.effective_max_planner_ticks * 4, 1000);
  while (rclcpp::ok() && runtime->sample_ok &&
         runtime->exec_ctx.neo_iter < max_neo_iterations) {
    const auto cycle_start_time = std::chrono::steady_clock::now();

    if (!is_active(mapping)) {
      break;
    }

    const std::vector<double> q_now_vec =
        hardware_manager_->get_current_joint_positions_lockfree(mapping);
    if (q_now_vec.size() != session.ctx->joint_names.size()) {
      break;
    }

    const double feedback_age_sec =
        hardware_manager_->get_joint_feedback_age_sec(mapping);
    if (!std::isfinite(feedback_age_sec) ||
        feedback_age_sec > runtime->feedback_stale_threshold_sec) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] reactive_task abort: stale joint feedback age=%.3f s "
                   "(threshold=%.3f s).",
                   mapping.c_str(), feedback_age_sec,
                   runtime->feedback_stale_threshold_sec);
      break;
    }

    const Eigen::VectorXd q_now = Eigen::Map<const Eigen::VectorXd>(
        q_now_vec.data(), static_cast<Eigen::Index>(q_now_vec.size()));

    arm_controller::kinematics::ForwardKinematicsOutput fk_now;
    if (!session.ctx->fk_provider->compute(q_now, fk_now)) {
      break;
    }

    const double pos_err_goal =
        (fk_now.ee_position - session.request.p_goal).norm();
    const double ori_err_goal =
        orientationErrorRad(fk_now.ee_rotation, session.request.R_goal);

    rt::ExecutionStatusInput preview_status_input;
    preview_status_input.current_phase = runtime->exec_ctx.phase;
    preview_status_input.pos_err_goal = pos_err_goal;
    preview_status_input.ori_err_goal = ori_err_goal;
    preview_status_input.path_progress = runtime->exec_ctx.last_path_progress;
    preview_status_input.task_residual_norm =
        runtime->exec_ctx.last_task_residual_norm;
    preview_status_input.whole_body_min_margin =
        runtime->exec_ctx.last_whole_body_status.min_margin;
    preview_status_input.whole_body_collision_free =
        runtime->exec_ctx.last_whole_body_status.collision_free;
    preview_status_input.whole_body_state =
        runtime->exec_ctx.last_whole_body_status.state;
    preview_status_input.reference_finished =
        runtime->exec_ctx.last_reference_finished;
    preview_status_input.hold_ready =
        runtime->exec_ctx.hold_ready;
    preview_status_input.no_progress_cycles =
        runtime->exec_ctx.no_progress_cycles;
    preview_status_input.no_motion_cycles = runtime->exec_ctx.no_motion_cycles;
    preview_status_input.qdot_limit_violation =
        runtime->exec_ctx.last_qdot_limit_violation;
    preview_status_input.phase_ticks = runtime->exec_ctx.phase_ticks;
    const rt::ExecutionStatusOutput phase_policy =
        execution_state_machine_.describe(preview_status_input);

    rt::LocalReferenceInput local_reference_input;
    local_reference_input.phase = runtime->exec_ctx.phase;
    local_reference_input.freeze_reference_progress =
        phase_policy.freeze_reference_progress;
    local_reference_input.allow_lookahead = phase_policy.allow_lookahead;
    local_reference_input.use_anchor_pose_only =
        phase_policy.use_anchor_pose_only;
    local_reference_input.global_trajectory = runtime->global_trajectory.get();
    local_reference_input.q_now = q_now;
    local_reference_input.ee_position = fk_now.ee_position;
    local_reference_input.ee_pose = fk_now.ee_pose;
    local_reference_input.planner_tick_sec = runtime->planner_tick_sec;
    local_reference_input.planner_tick_accumulator =
        runtime->exec_ctx.planner_tick_accumulator;
    local_reference_input.tracked_reference_time_sec =
        runtime->exec_ctx.tracked_reference_time_sec;
    local_reference_input.path_follow_joint_anchor_index_state =
        runtime->exec_ctx.path_follow_joint_anchor_index_state;
    local_reference_input.whole_body_collision_free =
        runtime->exec_ctx.last_whole_body_status.collision_free;

    rt::LocalReferenceOutput local_reference_output;
    if (!local_reference_manager_.compute(local_reference_input,
                                          &local_reference_output) ||
        !local_reference_output.ok) {
      RCLCPP_WARN(
          node_->get_logger(),
          "[%s] local_reference failed at tick %d: %s",
          mapping.c_str(), runtime->exec_ctx.planner_tick,
          local_reference_output.error.c_str());
      runtime->sample_ok = false;
      break;
    }
    if (local_reference_output.tracked_reference_time_sec <
        runtime->exec_ctx.tracked_reference_time_sec - 1e-9) {
      runtime->exec_ctx.tracked_reference_time_sec =
          local_reference_output.tracked_reference_time_sec;
      runtime->exec_ctx.planner_time_sec =
          runtime->exec_ctx.tracked_reference_time_sec;
      runtime->exec_ctx.planner_tick_accumulator = 0.0;
    }

    runtime->sample = local_reference_output.current_sample;
    runtime->exec_ctx.path_follow_joint_anchor_index_state =
        local_reference_output.next_anchor_index_state;
    const cp::TimedCartesianSample &path_follow_joint_anchor_sample =
        local_reference_output.anchor_sample_valid
            ? local_reference_output.anchor_sample
            : local_reference_output.current_sample;
    const bool path_follow_joint_anchor_sample_valid =
        local_reference_output.anchor_sample_valid;
    const bool reference_finished = local_reference_output.reference_finished;
    const double path_progress = local_reference_output.path_progress;
    cp::TimedCartesianSample terminal_joint_anchor_sample;
    bool terminal_joint_anchor_sample_valid = false;
    if (reference_finished) {
      const double total_duration =
          runtime->global_trajectory->activeSegmentTotalDurationSec();
      terminal_joint_anchor_sample_valid =
          runtime->global_trajectory->sampleByElapsedTime(
              total_duration, terminal_joint_anchor_sample) &&
          terminal_joint_anchor_sample.has_ik_joint_target &&
          terminal_joint_anchor_sample.ik_joint_target.size() == q_now.size() &&
          terminal_joint_anchor_sample.ik_joint_target.allFinite();
    }

    rt::ReactiveTaskEnvironmentProbe::Input env_input;
    env_input.mapping = mapping;
    env_input.whole_body_validator = session.whole_body_validator.get();
    env_input.q_now = &q_now;
    env_input.safe_distance = session.control_safe_distance;
    env_input.map = session.collision_map;
    env_input.ee_position = fk_now.ee_position;
    env_input.map_config.enable_dummy_obstacle =
        runtime_cfg_.enable_dummy_obstacle;
    env_input.map_config.dummy_obstacle_radius =
        runtime_cfg_.dummy_obstacle_radius;
    env_input.map_config.dummy_obstacle_center_left_arm =
        runtime_cfg_.dummy_obstacle_center_left_arm;
    env_input.map_config.dummy_obstacle_center_right_arm =
        runtime_cfg_.dummy_obstacle_center_right_arm;
    rt::ReactiveTaskEnvironmentProbe::Output env_output;
    environment_probe_.probe(env_input, &env_output);
    const rt::WholeBodyStatusSnapshot &whole_body_status =
        env_output.whole_body_status;
    constexpr double kHardCollisionEnterClearance = -0.030;
    constexpr double kHardCollisionNoiseClearance = -0.010;
    const double whole_body_signed_clearance =
        whole_body_status.worst_distance -
        whole_body_status.worst_effective_radius;
    if (std::isfinite(whole_body_signed_clearance)) {
      if (whole_body_signed_clearance <= kHardCollisionEnterClearance) {
        ++runtime->exec_ctx.hard_collision_margin_cycles;
      } else if (whole_body_signed_clearance >= kHardCollisionNoiseClearance) {
        runtime->exec_ctx.hard_collision_margin_cycles = 0;
      }
    } else {
      runtime->exec_ctx.hard_collision_margin_cycles = 0;
    }
    Eigen::Vector3d clearance_direction = Eigen::Vector3d::Zero();
    bool clearance_direction_valid = false;
    if (env_output.ee_escape_gradient_valid) {
      clearance_direction = env_output.ee_escape_gradient;
      clearance_direction_valid = true;
    } else if (whole_body_status.worst_gradient_norm > 1e-6 &&
               whole_body_status.worst_gradient_world.allFinite()) {
      clearance_direction = whole_body_status.worst_gradient_world /
                            whole_body_status.worst_gradient_norm;
      clearance_direction_valid = clearance_direction.allFinite();
    }

    rt::ExecutionStatusInput phase_input;
    phase_input.current_phase = runtime->exec_ctx.phase;
    phase_input.pos_err_goal = pos_err_goal;
    phase_input.ori_err_goal = ori_err_goal;
    phase_input.path_progress = path_progress;
    phase_input.task_residual_norm = runtime->exec_ctx.last_task_residual_norm;
    phase_input.whole_body_min_margin = whole_body_status.min_margin;
    phase_input.whole_body_collision_free = whole_body_status.collision_free;
    phase_input.whole_body_state = whole_body_status.state;
    phase_input.reference_finished = reference_finished;
    if (runtime->exec_ctx.active_segment_is_recovery && reference_finished) {
      phase_input.reference_finished = true;
      phase_input.pos_err_goal = std::numeric_limits<double>::infinity();
    }
    phase_input.hold_ready = runtime->exec_ctx.hold_ready;
    phase_input.no_progress_cycles = runtime->exec_ctx.no_progress_cycles;
    phase_input.no_motion_cycles = runtime->exec_ctx.no_motion_cycles;
    phase_input.qdot_limit_violation = false;
    phase_input.phase_ticks = runtime->exec_ctx.phase_ticks;
    phase_input.hard_collision_margin_cycles =
        runtime->exec_ctx.hard_collision_margin_cycles;

    rt::ExecutionStatusOutput phase_decision =
        execution_state_machine_.evaluate(phase_input);
    const bool goal_reached =
        reference_finished &&
        pos_err_goal <= runtime_cfg_.goal_position_tolerance &&
        ori_err_goal <= runtime_cfg_.goal_orientation_tolerance_rad &&
        !runtime->exec_ctx.active_segment_is_recovery;
    const int obstacle_stall_replan_cycle_limit =
        std::max(50, runtime->no_progress_cycle_limit / 2);
    const double obstacle_stall_replan_margin =
        std::min(0.010, std::max(0.003, 0.15 * session.control_safe_distance));
    const bool close_to_obstacle_boundary =
        (std::isfinite(whole_body_status.min_margin) &&
         whole_body_status.min_margin <= obstacle_stall_replan_margin) ||
        runtime->exec_ctx.last_obstacle_guidance_gate >=
            kObstacleStallReplanGuidanceGate;
    const bool blocked_before_goal =
        runtime->exec_ctx.phase == rt::ExecutionPhase::Track &&
        !reference_finished &&
        path_progress < kObstacleStallReplanProgressLimit &&
        runtime->exec_ctx.no_progress_cycles >= obstacle_stall_replan_cycle_limit &&
        close_to_obstacle_boundary;
    if (blocked_before_goal && phase_decision.phase == rt::ExecutionPhase::Track) {
      phase_decision.phase = rt::ExecutionPhase::Hold;
      phase_decision.freeze_reference_progress = true;
      phase_decision.allow_lookahead = false;
      phase_decision.use_anchor_pose_only = true;
      phase_decision.transition_reason = "obstacle_stall_replan";
      phase_decision.phase_changed = true;
      RCLCPP_WARN(
          node_->get_logger(),
          "[%s] global_replan_trigger: stalled near obstacle for %d cycles "
          "(path=%.3f min_margin=%.4f gate=%.3f); stopping arm before RRTConnect",
          mapping.c_str(), runtime->exec_ctx.no_progress_cycles, path_progress,
          whole_body_status.min_margin,
          runtime->exec_ctx.last_obstacle_guidance_gate);
    }
    if (phase_decision.phase_changed) {
      const rt::ExecutionPhase previous_phase = runtime->exec_ctx.phase;
      if (phase_decision.phase == rt::ExecutionPhase::Hold &&
          previous_phase != rt::ExecutionPhase::Hold) {
        runtime->exec_ctx.hold_pose = fk_now.ee_pose;
        runtime->exec_ctx.hold_pose_valid = true;
        runtime->exec_ctx.previous_nominal_twist.setZero();
        runtime->exec_ctx.previous_nominal_twist_valid = true;
        runtime->exec_ctx.hold_ready = false;
        runtime->exec_ctx.no_progress_cycles = 0;
        runtime->exec_ctx.no_motion_cycles = 0;
        runtime->exec_ctx.hard_collision_margin_cycles = 0;
        runtime->exec_ctx.recovery_boost_ticks = 0;
      } else if (previous_phase == rt::ExecutionPhase::Hold &&
                 phase_decision.phase != rt::ExecutionPhase::Hold) {
        runtime->exec_ctx.hold_pose = Eigen::Isometry3d::Identity();
        runtime->exec_ctx.hold_pose_valid = false;
        runtime->exec_ctx.hold_ready = false;
        runtime->exec_ctx.recovery_boost_ticks = std::max(
            8, static_cast<int>(std::llround(0.8 / runtime->neo_tick_sec)));
      }
      diagnostics_publisher_.logPhaseTransition(mapping, phase_decision, path_progress,
                                         pos_err_goal, ori_err_goal);
      runtime->exec_ctx.phase = phase_decision.phase;
      if (runtime->exec_ctx.phase != previous_phase) {
        runtime->exec_ctx.phase_ticks = 0;
      }
    }

    if (goal_reached) {
      if (!send_joint_velocities(
              mapping, std::vector<double>(
                           static_cast<std::size_t>(q_now.size()), 0.0))) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s] goal reached but failed to send hold command.",
                     mapping.c_str());
        break;
      }
      RCLCPP_INFO(node_->get_logger(),
                  "[%s] reactive_task reached goal: pos_err=%.5f ori_err=%.5f path_progress=%.3f",
                  mapping.c_str(), pos_err_goal, ori_err_goal, path_progress);
      return true;
    }

    // Local replanning is currently synchronous and can block for seconds.
    // Let one control cycle send a zero/hold command after entering the phase
    // before starting the planner, so stale tracking commands are not held
    // while the planner is busy.
    const bool hold_attempt_tick =
        runtime->exec_ctx.phase_ticks >= 1 &&
        ((runtime->exec_ctx.phase_ticks - 1) % kHoldRetryTicks) == 0;
    if (runtime->exec_ctx.phase == rt::ExecutionPhase::Hold &&
        !runtime->exec_ctx.hold_ready && hold_attempt_tick) {
      const double prediction_dt =
          std::max(0.0, runtime_cfg_.global_trajectory.planning_latency_sec);
      Eigen::VectorXd q_replan_start = q_now;
      if (runtime->exec_ctx.previous_qdot_reference_valid &&
          runtime->exec_ctx.previous_qdot_reference.size() == q_now.size()) {
        q_replan_start =
            q_now + prediction_dt * runtime->exec_ctx.previous_qdot_reference;
        q_replan_start =
            q_replan_start.cwiseMax(session.ctx->joint_limits.q_min)
                .cwiseMin(session.ctx->joint_limits.q_max);
      }

      Eigen::Vector3d replan_start_position = Eigen::Vector3d::Zero();
      Eigen::Matrix3d replan_start_rotation = Eigen::Matrix3d::Identity();
      static const std::vector<std::string> kNoLinkPoseQueries;
      arm_controller::kinematics::LinkPoseResultList replan_link_poses;
      if (!session.ctx->fk_provider->computeLinkPoses(
              q_replan_start,
              kNoLinkPoseQueries,
              replan_link_poses,
              &replan_start_position,
              &replan_start_rotation)) {
        replan_start_position = fk_now.ee_position;
        replan_start_rotation = fk_now.ee_rotation;
        q_replan_start = q_now;
      }

      cp::PathPlanningInput hold_input = session.request;
      hold_input.p_start = replan_start_position;
      hold_input.R_start = replan_start_rotation;
      hold_input.q_start_seed = q_replan_start;

      std::string hold_error;
      if (runtime->global_trajectory->preparePendingSegment(hold_input,
                                                    &hold_error) &&
          runtime->global_trajectory->commitPendingSegment(&hold_error)) {
        runtime->active_duration_sec =
            runtime->global_trajectory->activeSegmentTotalDurationSec();
        const bool committed_segment_is_recovery_only =
            runtime->global_trajectory->activeSegmentKind() ==
            cp::PlannedSegmentKind::Recovery;
        runtime->exec_ctx.active_duration_sec = runtime->active_duration_sec;
        runtime->exec_ctx.tracked_reference_time_sec = 0.0;
        runtime->exec_ctx.planner_time_sec = 0.0;
        runtime->exec_ctx.planner_tick_accumulator = 0.0;
        runtime->exec_ctx.path_follow_joint_anchor_index_state = 0;
        runtime->exec_ctx.best_pos_err = pos_err_goal;
        runtime->exec_ctx.best_ori_err = ori_err_goal;
        runtime->exec_ctx.best_path_progress = 0.0;
        runtime->exec_ctx.best_obstacle_min_distance =
            runtime->exec_ctx.last_obstacle_min_distance;
        runtime->exec_ctx.best_obstacle_guidance_gate =
            runtime->exec_ctx.last_obstacle_guidance_gate;
        runtime->exec_ctx.no_progress_cycles = 0;
        runtime->exec_ctx.no_motion_cycles = 0;
        runtime->exec_ctx.hard_collision_margin_cycles = 0;
        runtime->exec_ctx.hold_ready = false;
        runtime->exec_ctx.active_segment_is_recovery =
            committed_segment_is_recovery_only;
        runtime->exec_ctx.previous_qdot_reference =
            Eigen::VectorXd::Zero(q_now.size());
        runtime->exec_ctx.previous_qdot_reference_valid =
            runtime->exec_ctx.previous_qdot_reference.size() == q_now.size() &&
            runtime->exec_ctx.previous_qdot_reference.allFinite();
        runtime->exec_ctx.local_planner.clearTracking();
        runtime->exec_ctx.local_planner_runner.advanceGeneration();
        if (!runtime->global_trajectory->sampleByElapsedTime(0.0, runtime->sample)) {
          RCLCPP_WARN(node_->get_logger(),
                      "[%s] Hold committed but initial sample failed",
                      mapping.c_str());
          runtime->sample_ok = false;
          break;
        }
        diagnostics_publisher_.publishTrajectory(
            mapping, *runtime->global_trajectory, runtime->sample, fk_now.ee_position);
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] global_fallback: committed %s joint-reference segment",
                    mapping.c_str(),
                    committed_segment_is_recovery_only ? "start-recovery"
                                                       : "goal");
        runtime->exec_ctx.hold_pose = Eigen::Isometry3d::Identity();
        runtime->exec_ctx.hold_pose_valid = false;
        runtime->exec_ctx.phase = rt::ExecutionPhase::Track;
        runtime->exec_ctx.phase_ticks = 0;
        if (committed_segment_is_recovery_only) {
          runtime->exec_ctx.recovery_boost_ticks = std::max(
              8, static_cast<int>(std::llround(0.8 / runtime->neo_tick_sec)));
        }
      } else if (((runtime->exec_ctx.phase_ticks - 1) %
                  (4 * kHoldRetryTicks)) == 0) {
          RCLCPP_WARN(node_->get_logger(),
                    "[%s] global_fallback not ready: %s",
                    mapping.c_str(), hold_error.c_str());
      }
    }

    if (runtime->exec_ctx.phase == rt::ExecutionPhase::Hold &&
        !runtime->exec_ctx.hold_ready) {
      runtime->exec_ctx.previous_qdot_reference =
          Eigen::VectorXd::Zero(q_now.size());
      runtime->exec_ctx.previous_qdot_reference_valid =
          runtime->exec_ctx.previous_qdot_reference.size() == q_now.size() &&
          runtime->exec_ctx.previous_qdot_reference.allFinite();

      if (!send_joint_velocities(
              mapping, std::vector<double>(
                           static_cast<std::size_t>(q_now.size()), 0.0))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] Hold abort: failed to send hold command.",
                     mapping.c_str());
        break;
      }

      runtime->exec_ctx.last_task_residual_norm = 0.0;
      runtime->exec_ctx.last_path_progress = path_progress;
      runtime->exec_ctx.last_reference_finished = reference_finished;
      runtime->exec_ctx.last_whole_body_status = whole_body_status;
      runtime->exec_ctx.last_clearance_direction = clearance_direction;
      runtime->exec_ctx.last_clearance_direction_valid =
          clearance_direction_valid;

      const auto cycle_elapsed =
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - cycle_start_time)
              .count();
      const double cycle_dt_sec = std::max(runtime->neo_tick_sec, cycle_elapsed);
      if (cycle_elapsed < runtime->neo_tick_sec) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(runtime->neo_tick_sec -
                                          cycle_elapsed));
      }
      ++runtime->exec_ctx.neo_iter;
      ++runtime->exec_ctx.phase_ticks;
      runtime->exec_ctx.planner_tick_accumulator += cycle_dt_sec;
      continue;
    }

    rt::ReactiveTaskTerminalPolicy::PhaseFlags phase_flags =
        terminal_policy_.describePhase(runtime->exec_ctx.phase);
    const bool terminal_goal_tracking =
        phase_flags.path_follow_active && reference_finished;
    phase_flags.terminal_goal_tracking = terminal_goal_tracking;
    auto clear_local_planner_tracking = [&]() {
      runtime->exec_ctx.previous_nominal_twist.setZero();
      runtime->exec_ctx.previous_nominal_twist_valid = true;
      runtime->exec_ctx.local_planner.clearTracking();
    };
    rt::ArmState current_arm_state;
    current_arm_state.joint_names = session.ctx->joint_names;
    current_arm_state.q = q_now;
    current_arm_state.qd_min = session.ctx->qd_min;
    current_arm_state.qd_max = session.ctx->qd_max;
    current_arm_state.joint_limits = session.ctx->joint_limits;
    if (runtime->exec_ctx.previous_qdot_reference_valid &&
        runtime->exec_ctx.previous_qdot_reference.size() == q_now.size()) {
      current_arm_state.qd = runtime->exec_ctx.previous_qdot_reference;
    }
    if (phase_flags.path_follow_active) {
      if (terminal_goal_tracking) {
        runtime->exec_ctx.local_planner_runner.handleTerminalTracking(
            &runtime->exec_ctx.local_planner,
            !runtime->exec_ctx.last_reference_finished,
            mapping,
            node_->get_logger());
      } else {
        rt::AsyncLocalPlannerRunner::ApplyReadyInput apply_ready_input;
        apply_ready_input.mapping = mapping;
        apply_ready_input.logger = node_->get_logger();
        apply_ready_input.clock = node_->get_clock();
        apply_ready_input.current_qdot_reference =
            runtime->exec_ctx.previous_qdot_reference;
        apply_ready_input.current_qdot_reference_valid =
            runtime->exec_ctx.previous_qdot_reference_valid;
        apply_ready_input.now_sec = runtime->exec_ctx.tracked_reference_time_sec;
        apply_ready_input.max_usable_age_sec =
            std::max(0.12,
                     2.0 * runtime_cfg_.local_planner.update_period_sec);
        const rt::AsyncLocalPlannerRunner::ApplyReadyResult apply_result =
            runtime->exec_ctx.local_planner_runner.applyReady(
                &runtime->exec_ctx.local_planner, std::move(apply_ready_input));
        if (apply_result.accepted) {
          runtime->exec_ctx.previous_nominal_twist =
              apply_result.nominal_twist;
          runtime->exec_ctx.previous_nominal_twist_valid = true;
        }
      }

      const double local_planner_period_sec =
          std::max(1e-3, runtime_cfg_.local_planner.update_period_sec);
      const bool local_planner_due =
          !terminal_goal_tracking &&
          runtime->exec_ctx.local_planner_runner.dueForRequest(
              runtime->exec_ctx.local_planner,
              runtime->exec_ctx.tracked_reference_time_sec,
              local_planner_period_sec);
      if (local_planner_due) {
        const double local_planner_prediction_dt = std::clamp(
            std::max(runtime_cfg_.global_trajectory.planning_latency_sec,
                     runtime_cfg_.local_planner.update_period_sec),
            0.0,
            0.15);
        std::shared_ptr<cp::CameraDriverPointcloudMapAdapter> pointcloud_map;
        {
          std::lock_guard<std::mutex> lock(live_distance_field_mutex_);
          pointcloud_map = camera_driver_pointcloud_map_;
        }
        LocalPlannerBuildResult local_planner_build = buildLocalPlannerInput(
            mapping,
            runtime->global_trajectory.get(),
            runtime_cfg_.local_planner,
            current_arm_state,
            fk_now,
            local_reference_output,
            reference_finished,
            local_planner_prediction_dt,
            runtime->exec_ctx.previous_qdot_reference_valid,
            runtime->exec_ctx.previous_qdot_reference,
            session.ctx->fk_provider,
            session.ctx->collision_ellipsoids,
            session.ctx->robot_type,
            session.ctx->planning_group,
            session.ctx->base_link,
            session.ctx->tip_link,
            session.ctx->urdf_path,
            session.ctx->srdf_path,
            obstacle_selector_,
            pointcloud_map);
        rt::ArmLocalPlanner::Input &local_planner_input =
            local_planner_build.input;
        if (pointcloud_map) {
          const rt::ObstacleSelectionOutput &obstacle_selection =
              local_planner_build.obstacle_selection;
          RCLCPP_INFO_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[%s] local_trajopt obstacles: selected=%zu candidates=%zu cells=%zu radius=%.3f voxel_radius=%.3f padding=%.3f selection_radius=%.3f",
              mapping.c_str(), local_planner_input.sphere_obstacles.size(),
              obstacle_selection.candidate_count, obstacle_selection.cell_count,
              obstacle_selection.obstacle_radius,
              obstacle_selection.voxel_radius,
              obstacle_selection.padding,
              obstacle_selection.selection_radius);
        }

        if (local_planner_input.q_goal_valid) {
          runtime->exec_ctx.local_planner_runner.start(
              local_planner_,
              std::move(local_planner_input),
              runtime->exec_ctx.planner_tick,
              runtime->exec_ctx.tracked_reference_time_sec);
        }
      }
    } else {
      clear_local_planner_tracking();
      runtime->exec_ctx.local_planner_runner.advanceGeneration();
    }

    rt::ReactiveTaskControlTargetBuilder::Input control_target_input;
    control_target_input.mapping = mapping;
    control_target_input.logger = node_->get_logger();
    control_target_input.clock = node_->get_clock();
    control_target_input.phase_flags = phase_flags;
    control_target_input.terminal_goal_tracking = terminal_goal_tracking;
    control_target_input.sample = runtime->sample;
    control_target_input.goal_position = session.request.p_goal;
    control_target_input.goal_rotation = session.request.R_goal;
    control_target_input.terminal_joint_anchor_sample =
        terminal_joint_anchor_sample;
    control_target_input.terminal_joint_anchor_sample_valid =
        terminal_joint_anchor_sample_valid;
    control_target_input.arm_state = current_arm_state;
    control_target_input.tracked_reference_time_sec =
        runtime->exec_ctx.tracked_reference_time_sec;
    control_target_input.local_planner = &runtime->exec_ctx.local_planner;
    const rt::ControlTarget control_target =
        control_target_builder_.build(control_target_input);
    phase_flags.local_trajopt_tracking = control_target.local_planner_tracking;
    const rt::ReactiveTaskTerminalPolicy::CommandTargetOutput command_target =
        terminal_policy_.buildCommandTarget(
            runtime->exec_ctx, phase_flags, session.request.p_goal,
            session.request.R_goal, runtime->sample, control_target.pose,
            control_target.twist, fk_now.ee_pose, reference_finished,
            clearance_direction, clearance_direction_valid,
            whole_body_status.min_margin);
    rq::TaskVelocityInput task_in;
    task_in.T_current = fk_now.ee_pose;
    task_in.has_target_pose = true;
    task_in.T_target = command_target.commanded_target_pose;
    task_in.has_target_twist = true;
    task_in.target_twist = command_target.commanded_target_twist;
    rq::TaskVelocityOutput task_out = runtime->task_velocity_generator.compute(
        task_in, reactive_cfg_.task_velocity);
    terminal_policy_.shapeTaskVelocity(phase_flags, task_out,
                                       reactive_cfg_.task_velocity);
    rt::ReactiveTaskNeoPipeline::PrepareInput neo_pipeline_input;
    neo_pipeline_input.arm_state = current_arm_state;
    neo_pipeline_input.jacobian_provider = session.ctx->jacobian_provider.get();
    neo_pipeline_input.task_out = &task_out;
    neo_pipeline_input.phase_flags = &phase_flags;
    neo_pipeline_input.exec_ctx = &runtime->exec_ctx;
    neo_pipeline_input.joint_preference_cfg = &session.ctx->joint_preference_cfg;
    neo_pipeline_input.path_follow_joint_anchor_sample =
        path_follow_joint_anchor_sample_valid ? &path_follow_joint_anchor_sample
                                              : nullptr;
    neo_pipeline_input.path_follow_joint_anchor_sample_valid =
        path_follow_joint_anchor_sample_valid;
    neo_pipeline_input.current_sample = &runtime->sample;
    neo_pipeline_input.local_planner_joint_target =
        control_target.localPlannerJointTarget(q_now.size());
    neo_pipeline_input.local_planner_joint_target_dt_sec =
        control_target.local_planner_joint_target_dt_sec;
    neo_pipeline_input.manipulability_gradient =
        session.ctx->manipulability_gradient.get();
    neo_pipeline_input.manipulability_cfg = &reactive_cfg_.manipulability;
    rt::ReactiveTaskNeoPipeline::PrepareOutput neo_pipeline_output;
    if (!neo_pipeline_.prepare(neo_pipeline_input, &neo_pipeline_output) ||
        !neo_pipeline_output.ok) {
      break;
    }
    rq::ReactiveQpBuildInput qp_input = neo_pipeline_output.qp_input;
    rq::ReactiveQpBuildConfig qp_build_cfg = reactive_cfg_.qp_build;
    rt::ReactiveTaskSafetyPolicy::Input safety_input;
    safety_input.enable_obstacle_constraints =
        kReactiveObstacleAvoidanceEnabled && session.enable_obstacle_constraints;
    safety_input.planner_tick = runtime->exec_ctx.planner_tick;
    safety_input.reference_finished = reference_finished;
    safety_input.path_progress = path_progress;
    safety_input.hard_clearance = session.request.hard_clearance;
    safety_input.safe_distance = session.control_safe_distance;
    safety_input.whole_body_min_margin = whole_body_status.min_margin;
    safety_input.exec_ctx = &runtime->exec_ctx;
    safety_input.base_qp_build_cfg = qp_build_cfg;
    safety_input.qp_input = qp_input;
    safety_input.arm_state = current_arm_state;
    safety_input.link_poses = &fk_now.link_poses;
    safety_input.collision_ellipsoids = &session.ctx->collision_ellipsoids;
    safety_input.jacobian_provider = session.ctx->jacobian_provider.get();
    safety_input.map = session.collision_map;
    safety_input.phase_flags = &phase_flags;
    safety_input.logger = node_->get_logger();
    safety_input.clock = node_->get_clock();
    safety_input.mapping = mapping;

    rt::ReactiveTaskSafetyPolicy::Output safety_output;
    if (!safety_policy_.apply(safety_input, &safety_output)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] safety abort: policy application failed.",
                   mapping.c_str());
      break;
    }
    qp_input = std::move(safety_output.qp_input);
    qp_build_cfg = std::move(safety_output.qp_build_cfg);
    const double obstacle_guidance_gate = safety_output.obstacle_guidance_gate;
    const double obstacle_min_distance = safety_output.obstacle_min_distance;
    const double active_safety_distance = safety_output.active_safety_distance;

    rt::ReactiveTaskNeoPipeline::SolveInput neo_solve_input;
    neo_solve_input.mapping = mapping;
    neo_solve_input.planner_tick = runtime->exec_ctx.planner_tick;
    neo_solve_input.qp_input = &qp_input;
    neo_solve_input.qp_build_cfg = &qp_build_cfg;
    if (!runtime->solver) {
      runtime->solver =
          std::make_unique<arm_controller::algorithm::reactive_qp::ReactiveQpSolver>();
    }
    neo_solve_input.solver = runtime->solver.get();
    neo_solve_input.arm_state = current_arm_state;
    neo_solve_input.task_jacobian = &neo_pipeline_output.J_task;
    neo_solve_input.desired_twist = &task_out.v_des;
    neo_solve_input.logger = node_->get_logger();
    rt::ReactiveTaskNeoPipeline::SolveOutput neo_solve_output;
    if (!neo_pipeline_.solve(neo_solve_input, &neo_solve_output) ||
        !neo_solve_output.ok) {
      break;
    }

    const int dof = static_cast<int>(q_now.size());
    std::vector<double> qdot_cmd = std::move(neo_solve_output.qdot_cmd);
    Eigen::VectorXd qdot_eigen = std::move(neo_solve_output.qdot_eigen);
    const Eigen::VectorXd &task_pred = neo_solve_output.task_pred;
    const Eigen::VectorXd &task_residual = neo_solve_output.task_residual;
    const double task_residual_norm = neo_solve_output.task_residual_norm;
    const bool waiting_for_first_local_plan = false;
    if (waiting_for_first_local_plan) {
      qdot_eigen = Eigen::VectorXd::Zero(dof);
      qdot_cmd.assign(static_cast<std::size_t>(dof), 0.0);
      runtime->exec_ctx.previous_qdot_reference = qdot_eigen;
      runtime->exec_ctx.previous_qdot_reference_valid =
          qdot_eigen.size() == q_now.size() && qdot_eigen.allFinite();
      runtime->exec_ctx.q_prev_feedback = q_now;
      runtime->exec_ctx.q_prev_feedback_valid = q_now.allFinite();
      runtime->exec_ctx.previous_pos_err = pos_err_goal;
      runtime->exec_ctx.previous_ori_err = ori_err_goal;
      runtime->exec_ctx.previous_task_error_valid =
          std::isfinite(pos_err_goal) && std::isfinite(ori_err_goal);
      runtime->exec_ctx.last_task_residual_norm = 0.0;
      runtime->exec_ctx.last_obstacle_guidance_gate = obstacle_guidance_gate;
      runtime->exec_ctx.last_obstacle_min_distance = obstacle_min_distance;
      runtime->exec_ctx.last_path_progress = path_progress;
      runtime->exec_ctx.last_reference_finished = reference_finished;
      runtime->exec_ctx.last_whole_body_status = whole_body_status;
      runtime->exec_ctx.last_clearance_direction = clearance_direction;
      runtime->exec_ctx.last_clearance_direction_valid =
          clearance_direction_valid;
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[%s] local_trajopt warmup: waiting for first async window at tick=%d progress=%.3f",
          mapping.c_str(), runtime->exec_ctx.planner_tick, path_progress);

      if (!send_joint_velocities(mapping, qdot_cmd)) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s] warmup abort: failed to send hold command.",
                     mapping.c_str());
        break;
      }

      const auto cycle_elapsed =
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - cycle_start_time)
              .count();
      if (cycle_elapsed < runtime->neo_tick_sec) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(runtime->neo_tick_sec -
                                          cycle_elapsed));
      }
      ++runtime->exec_ctx.neo_iter;
      ++runtime->exec_ctx.phase_ticks;
      runtime->exec_ctx.planner_tick_accumulator +=
          std::max(runtime->neo_tick_sec, cycle_elapsed);
      while (runtime->exec_ctx.planner_tick_accumulator + 1e-12 >=
             runtime->planner_tick_sec) {
        runtime->exec_ctx.planner_tick_accumulator -= runtime->planner_tick_sec;
        ++runtime->exec_ctx.planner_tick;
      }
      continue;
    }

    rt::ReactiveTaskWatchdog::UpdateInput watchdog_input;
    watchdog_input.q_now = q_now;
    watchdog_input.qdot_eigen = qdot_eigen;
    watchdog_input.qd_min = session.ctx->qd_min;
    watchdog_input.qd_max = session.ctx->qd_max;
    watchdog_input.phase = runtime->exec_ctx.phase;
    watchdog_input.pos_err_goal = pos_err_goal;
    watchdog_input.ori_err_goal = ori_err_goal;
    watchdog_input.path_progress = path_progress;
    watchdog_input.obstacle_guidance_gate = obstacle_guidance_gate;
    watchdog_input.obstacle_min_distance = obstacle_min_distance;
    watchdog_input.control_dt_sec = runtime->neo_tick_sec;
    rt::ReactiveTaskWatchdog::UpdateOutput watchdog_output;
    watchdog_.update(runtime->exec_ctx, watchdog_input,
                     runtime->no_motion_cycle_limit,
                     runtime->no_progress_cycle_limit, &watchdog_output);
    const bool recovery_boost_active =
        runtime->exec_ctx.recovery_boost_ticks > 0;
    const bool conservative_qdot_phase =
        phase_flags.hold_active || phase_flags.terminal_goal_tracking;
    const double qdot_smoothing_alpha =
        recovery_boost_active
            ? kRecoveryQdotBlend
            : (phase_flags.terminal_goal_tracking
                   ? kTerminalQdotBlend
                   : (conservative_qdot_phase ? kRecoveryQdotBlend
                                                : kTrackQdotBlend));
    if (runtime->exec_ctx.previous_qdot_reference_valid &&
        runtime->exec_ctx.previous_qdot_reference.size() == qdot_eigen.size()) {
      const Eigen::VectorXd previous_qdot =
          runtime->exec_ctx.previous_qdot_reference;
      qdot_eigen = (1.0 - qdot_smoothing_alpha) * previous_qdot +
                   qdot_smoothing_alpha * qdot_eigen;
      const double qdot_delta_limit =
          recovery_boost_active
              ? kRecoveryQdotDeltaLimit
              : (phase_flags.terminal_goal_tracking
                     ? kTerminalQdotDeltaLimit
                     : (conservative_qdot_phase ? kRecoveryQdotDeltaLimit
                                                  : kTrackQdotDeltaLimit));
      for (int i = 0; i < qdot_eigen.size(); ++i) {
        qdot_eigen(i) =
            std::clamp(qdot_eigen(i), previous_qdot(i) - qdot_delta_limit,
                       previous_qdot(i) + qdot_delta_limit);
      }
    }
    for (int i = 0; i < qdot_eigen.size(); ++i) {
      qdot_eigen(i) = std::clamp(qdot_eigen(i), session.ctx->qd_min(i),
                                 session.ctx->qd_max(i));
    }
    if (recovery_boost_active) {
      for (int i = 0; i < qdot_eigen.size(); ++i) {
        const double abs_qdot = std::abs(qdot_eigen(i));
        if (abs_qdot < 1e-5 || abs_qdot >= kRecoveryMinJointSpeed) {
          continue;
        }
        qdot_eigen(i) = std::copysign(
            std::min(std::abs(session.ctx->qd_max(i)), kRecoveryMinJointSpeed),
            qdot_eigen(i));
        qdot_eigen(i) = std::clamp(qdot_eigen(i), session.ctx->qd_min(i),
                                   session.ctx->qd_max(i));
      }
    }
    const bool primary_recovery_active = watchdog_.shouldApplyPrimaryRecovery(
        runtime->exec_ctx, pos_err_goal, ori_err_goal,
        runtime->no_motion_cycle_limit, runtime->no_progress_cycle_limit) &&
        !phase_flags.terminal_goal_tracking;
    watchdog_.applyPrimaryRecoveryBoost(primary_recovery_active,
                                        session.ctx->qd_max, qdot_eigen);
    runtime->exec_ctx.previous_qdot_reference = qdot_eigen;
    runtime->exec_ctx.previous_qdot_reference_valid =
        runtime->exec_ctx.previous_qdot_reference.size() == q_now.size() &&
        runtime->exec_ctx.previous_qdot_reference.allFinite();
    for (int i = 0; i < dof; ++i) {
      qdot_cmd[static_cast<std::size_t>(i)] = qdot_eigen(i);
    }

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[%s] neo trace tick=%d phase=%d q_now=%s qdot=%s pos_err=%.4f ori_err=%.4f path=%.3f target_xyz=(%.3f, %.3f, %.3f)",
        mapping.c_str(), runtime->exec_ctx.planner_tick,
        static_cast<int>(runtime->exec_ctx.phase),
        formatVector(q_now).c_str(), formatVector(qdot_eigen).c_str(),
        pos_err_goal, ori_err_goal, path_progress,
        command_target.commanded_target_pose.translation().x(),
        command_target.commanded_target_pose.translation().y(),
        command_target.commanded_target_pose.translation().z());

    const double qdot_max_abs = qdot_eigen.cwiseAbs().maxCoeff();
    const double qdot_norm = qdot_eigen.norm();
    const double qdot_delta_norm = watchdog_output.qdot_delta_norm;
    const bool qdot_limit_violation = watchdog_output.qdot_limit_violation;
    const double joint_limit_margin_min =
        (q_now - session.ctx->joint_limits.q_min)
            .cwiseMin(session.ctx->joint_limits.q_max - q_now)
            .minCoeff();
    const double reference_pos_error =
        (runtime->sample.T_target.translation() -
         fk_now.ee_pose.translation()).norm();
    const double reference_ori_error = orientationErrorRad(
        fk_now.ee_rotation, runtime->sample.T_target.linear());
    const double reference_tracking_gate =
        waiting_for_first_local_plan
            ? 0.0
            : phase_flags.path_follow_active && !reference_finished
            ? progressGateFromError(reference_pos_error,
                                    kReferenceFullSpeedPositionError,
                                    kReferenceFreezePositionError)
            : 1.0;
    (void)reference_ori_error;

    runtime->exec_ctx.last_task_residual_norm = task_residual_norm;
    runtime->exec_ctx.last_obstacle_guidance_gate = obstacle_guidance_gate;
    runtime->exec_ctx.last_obstacle_min_distance = obstacle_min_distance;
    runtime->exec_ctx.last_path_progress = path_progress;
    runtime->exec_ctx.last_reference_finished = reference_finished;
    runtime->exec_ctx.last_whole_body_status = whole_body_status;
    runtime->exec_ctx.last_clearance_direction = clearance_direction;
    runtime->exec_ctx.last_clearance_direction_valid =
        clearance_direction_valid;

    if (watchdog_output.no_motion_abort) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] watchdog stop: no joint motion for %d "
                   "cycles while qdot_norm=%.4f (pos_err=%.5f ori_err=%.5f)",
                   mapping.c_str(), runtime->exec_ctx.no_motion_cycles,
                   qdot_norm, pos_err_goal, ori_err_goal);
      break;
    }
    if (watchdog_output.no_progress_abort) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s] watchdog stop: no effective progress for "
                   "%d cycles (best_pos_err=%.5f best_ori_err=%.5f "
                   "best_path_progress=%.3f current_pos_err=%.5f "
                   "current_ori_err=%.5f current_path_progress=%.3f)",
                   mapping.c_str(), runtime->exec_ctx.no_progress_cycles,
                   runtime->exec_ctx.best_pos_err,
                   runtime->exec_ctx.best_ori_err,
                   runtime->exec_ctx.best_path_progress, pos_err_goal,
                   ori_err_goal, path_progress);
      break;
    }

    if ((runtime->exec_ctx.neo_iter % runtime->exec_ctx.safety_log_stride) ==
        0) {
      rt::ReactiveTaskDiagnosticsPublisher::RuntimeVisualizationInput
          diag_input;
      diag_input.mapping = mapping;
      diag_input.fk_provider = session.ctx->fk_provider.get();
      diag_input.collision_ellipsoids = &session.ctx->collision_ellipsoids;
      diag_input.q_now = &q_now;
      diag_input.global_trajectory = runtime->global_trajectory.get();
      diag_input.sample = &runtime->sample;
      diag_input.ee_position = fk_now.ee_position;
      diag_input.phase = runtime->exec_ctx.phase;
      diag_input.planner_tick = runtime->exec_ctx.planner_tick;
      diag_input.neo_iter = runtime->exec_ctx.neo_iter;
      diag_input.path_progress = path_progress;
      diag_input.pos_err_goal = pos_err_goal;
      diag_input.ori_err_goal = ori_err_goal;
      diag_input.qdot_norm = qdot_norm;
      diag_input.qdot_max_abs = qdot_max_abs;
      diag_input.qdot_delta_norm = qdot_delta_norm;
      diag_input.qdot_limit_violation = qdot_limit_violation;
      diag_input.joint_limit_margin_min = joint_limit_margin_min;
      diag_input.task_residual_norm = task_residual_norm;
      diag_input.ee_clearance = env_output.ee_clearance;
      diag_input.whole_body_status = whole_body_status;
      diag_input.active_safety_distance = active_safety_distance;
      diag_input.obstacle_guidance_gate = obstacle_guidance_gate;
      diag_input.obstacle_min_distance = obstacle_min_distance;
      diag_input.local_plan_sampled_steps =
          runtime->exec_ctx.local_planner.sampledSteps();
      diag_input.local_plan_dt_sec =
          runtime->exec_ctx.local_planner.diagnosticDtSec();
      diag_input.qdot = &qdot_eigen;
      diag_input.v_des = &task_out.v_des;
      diag_input.task_pred = &task_pred;
      diag_input.task_residual = &task_residual;
      diagnostics_publisher_.publishRuntimeCycle(diag_input);
      diagnostics_publisher_.publishExecutionTrace(mapping, fk_now.ee_position);
    }

    if (!send_joint_velocities(mapping, qdot_cmd)) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "[%s] neo abort: failed to send joint velocity command.",
          mapping.c_str());
      break;
    }

    if (pos_err_goal <= runtime_cfg_.goal_position_tolerance &&
        ori_err_goal <= runtime_cfg_.goal_orientation_tolerance_rad) {
      return true;
    }

    const auto cycle_elapsed =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - cycle_start_time)
            .count();
    const double cycle_dt_sec = std::max(runtime->neo_tick_sec, cycle_elapsed);
    if (cycle_elapsed < runtime->neo_tick_sec) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(runtime->neo_tick_sec -
                                        cycle_elapsed));
    }
    ++runtime->exec_ctx.neo_iter;
    if (runtime->exec_ctx.recovery_boost_ticks > 0) {
      --runtime->exec_ctx.recovery_boost_ticks;
    }
    ++runtime->exec_ctx.phase_ticks;
    runtime->exec_ctx.planner_tick_accumulator += cycle_dt_sec;

    while (runtime->exec_ctx.planner_tick_accumulator + 1e-12 >=
           runtime->planner_tick_sec) {
      runtime->exec_ctx.planner_tick_accumulator -= runtime->planner_tick_sec;
      ++runtime->exec_ctx.planner_tick;

      runtime->exec_ctx.tracked_reference_time_sec = std::min(
          runtime->exec_ctx.tracked_reference_time_sec +
              runtime->planner_tick_sec *
                  local_reference_output.progress_scale *
                  reference_tracking_gate,
          runtime->global_trajectory->activeSegmentTotalDurationSec());
      runtime->exec_ctx.planner_time_sec =
          runtime->exec_ctx.tracked_reference_time_sec;
      if (!runtime->global_trajectory->sampleByElapsedTime(
              runtime->exec_ctx.planner_time_sec, runtime->sample)) {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] global_reference sample failed at tick %d",
                    mapping.c_str(), runtime->exec_ctx.planner_tick);
        runtime->sample_ok = false;
        break;
      }
    }
  }

  return false;
}
