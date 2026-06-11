#include "controller/reactive_task/reactive_task_controller.hpp"

#include "controller/reactive_task/global_planner/reactive_task_planning_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace rt = arm_controller::controller::reactive_task;
namespace planning_detail = arm_controller::controller::reactive_task::planning_detail;

using planning_detail::orientationErrorRad;

bool ReactiveTaskController::initializePlanningRuntime(
    const std::string &mapping, const PlanningSession &session,
    PlanningRuntime *runtime) {
  if (runtime == nullptr || !session.planner) {
    return false;
  }

  runtime->global_trajectory = std::make_unique<cp::GlobalTrajectoryManager>(session.planner);
  const cp::GlobalTrajectoryConfig global_trajectory_cfg =
      runtime_cfg_.global_trajectory;
  runtime->global_trajectory->setConfig(global_trajectory_cfg);
  RCLCPP_INFO(node_->get_logger(),
              "[%s] global_planner init: manager_ready=true goals=%zu qdim=%ld",
              mapping.c_str(), session.request.q_goal_candidates.size(),
              static_cast<long>(session.q_start.size()));

  runtime->planner_tick_sec =
      std::max(1e-4, global_trajectory_cfg.control_cycle_sec);
  runtime->neo_tick_sec = std::max(1e-4, runtime_cfg_.neo_control_cycle_sec);
  runtime->exec_ctx.safety_log_stride =
      std::max(1, static_cast<int>(std::llround(0.1 / runtime->neo_tick_sec)));

  std::string error;
  RCLCPP_INFO(node_->get_logger(), "[%s] global_planner start: entering",
              mapping.c_str());
  if (!runtime->global_trajectory->start(session.request, &error)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] global_planner start failed: %s", mapping.c_str(),
                 error.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "[%s] global_planner start: complete",
              mapping.c_str());

  std::ostringstream map_summary;
  int processed_frames = 0;
  std::size_t active_cells = 0u;
  std::size_t successful_queries = 0u;
  std::size_t failed_queries = 0u;
  bool esdf_ready = false;
  int esdf_frames = 0;
  {
    std::lock_guard<std::mutex> lock(live_distance_field_mutex_);
    if (camera_driver_pointcloud_map_) {
      processed_frames = camera_driver_pointcloud_map_->processedFrames();
      active_cells = camera_driver_pointcloud_map_->activeCellCount();
    }
    if (camera_driver_esdf_map_) {
      successful_queries = camera_driver_esdf_map_->successfulQueries();
      failed_queries = camera_driver_esdf_map_->failedQueries();
      esdf_ready = camera_driver_esdf_map_->isMapReady();
      esdf_frames = camera_driver_esdf_map_->processedFrames();
    }
  }
  map_summary << "distance_field=" << runtime_cfg_.distance_field_source;
  if (runtime_cfg_.distance_field_source == "camera_driver_esdf") {
    map_summary << "(shm_ready=" << (esdf_ready ? "true" : "false")
                << " frames=" << esdf_frames << " ok=" << successful_queries
                << " fail=" << failed_queries << ")";
  } else if (runtime_cfg_.distance_field_source == "camera_driver_pointcloud") {
    map_summary << "(frames=" << processed_frames << " cells=" << active_cells
                << ")";
  }
  map_summary << " collision_map=" << runtime_cfg_.collision_map_source
              << "(frames=" << processed_frames << " cells=" << active_cells
              << ")";

  RCLCPP_INFO(
      node_->get_logger(),
      "[%s] reactive_task: maps=%s pipeline=global_rrt_connect->local_trajopt->neo "
      "freq(local/neo)=%.2f/%.2fHz dt(local/neo)=%.3f/%.3f max_ticks=%d "
      "clr(plan/control/hard)=%.3f/%.3f/%.3f tol=%.3f/%.3f "
      "start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f)",
      mapping.c_str(), map_summary.str().c_str(),
      1.0 / std::max(1e-3, runtime_cfg_.local_planner.update_period_sec),
      1.0 / runtime->neo_tick_sec,
      runtime_cfg_.local_planner.update_period_sec, runtime->neo_tick_sec,
      runtime_cfg_.max_control_ticks, session.request.safe_distance,
      session.control_safe_distance, session.request.hard_clearance,
      runtime_cfg_.goal_position_tolerance,
      runtime_cfg_.goal_orientation_tolerance_rad, session.request.p_start.x(),
      session.request.p_start.y(), session.request.p_start.z(),
      session.request.p_goal.x(), session.request.p_goal.y(),
      session.request.p_goal.z());

  if (!runtime->global_trajectory->sampleByElapsedTime(
          runtime->exec_ctx.planner_time_sec, runtime->sample)) {
    RCLCPP_WARN(node_->get_logger(), "[%s] global_plan initial sample failed",
                mapping.c_str());
    runtime->sample_ok = false;
  }

  runtime->active_duration_sec =
      runtime->global_trajectory->activeSegmentTotalDurationSec();
  runtime->configured_max_planner_ticks =
      std::max(1, runtime_cfg_.max_control_ticks);
  const int duration_limited_min_ticks = std::max(
      1, static_cast<int>(std::ceil((runtime->active_duration_sec + 1.0) /
                                    runtime->planner_tick_sec)));
  runtime->exec_ctx.effective_max_planner_ticks = std::max(
      runtime->configured_max_planner_ticks, duration_limited_min_ticks);
  runtime->exec_ctx.active_duration_sec = runtime->active_duration_sec;

  if (runtime->sample_ok) {
    RCLCPP_INFO(node_->get_logger(),
                "[%s] global_plan: duration=%.3f waypoints=%d ticks=%d/%d sampling=continuous_joint_interpolation",
                mapping.c_str(), runtime->active_duration_sec,
                runtime->global_trajectory->activeSegmentPointCount(),
                runtime->configured_max_planner_ticks,
                runtime->exec_ctx.effective_max_planner_ticks);
  }

  runtime->exec_ctx.phase = rt::ExecutionPhase::Track;
  runtime->exec_ctx.ik_seed_q = session.q_start;
  runtime->exec_ctx.hold_ready = false;
  runtime->exec_ctx.active_segment_is_recovery =
      runtime->global_trajectory->activeSegmentKind() ==
      cp::PlannedSegmentKind::Recovery;
  runtime->exec_ctx.phase_ticks = 0;
  runtime->exec_ctx.q_prev_feedback = session.q_start;
  runtime->exec_ctx.q_prev_feedback_valid =
      (runtime->exec_ctx.q_prev_feedback.size() == session.q_start.size()) &&
      session.q_start.allFinite();
  runtime->exec_ctx.previous_qdot_reference =
      Eigen::VectorXd::Zero(session.q_start.size());
  runtime->exec_ctx.previous_qdot_reference_valid =
      runtime->exec_ctx.previous_qdot_reference.size() ==
          session.q_start.size() &&
      runtime->exec_ctx.previous_qdot_reference.allFinite();
  runtime->exec_ctx.previous_nominal_twist.setZero();
  runtime->exec_ctx.previous_nominal_twist_valid = true;
  runtime->exec_ctx.path_follow_joint_anchor_index_state = std::max(
      0,
      runtime->global_trajectory->pointIndexAtTime(runtime->exec_ctx.planner_time_sec));
  runtime->exec_ctx.best_pos_err =
      (session.request.p_start - session.request.p_goal).norm();
  runtime->exec_ctx.best_ori_err =
      orientationErrorRad(session.request.R_start, session.request.R_goal);
  runtime->exec_ctx.best_path_progress = 0.0;
  runtime->exec_ctx.previous_pos_err = runtime->exec_ctx.best_pos_err;
  runtime->exec_ctx.previous_ori_err = runtime->exec_ctx.best_ori_err;
  runtime->exec_ctx.previous_task_error_valid = false;
  runtime->exec_ctx.no_motion_cycles = 0;
  runtime->exec_ctx.no_progress_cycles = 0;
  runtime->exec_ctx.hard_collision_margin_cycles = 0;

  runtime->no_motion_cycle_limit =
      std::max(20, static_cast<int>(std::llround(0.6 / runtime->neo_tick_sec)));
  runtime->no_progress_cycle_limit =
      std::max(50, static_cast<int>(std::llround(2.0 / runtime->neo_tick_sec)));
  runtime->feedback_stale_threshold_sec =
      std::max(0.2, 5.0 * runtime->neo_tick_sec);

  return true;
}
