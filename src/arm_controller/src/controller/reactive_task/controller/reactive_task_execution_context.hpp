#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>

#include "controller/reactive_task/local_planner/reactive_task_async_local_planner_runner.hpp"
#include "controller/reactive_task/local_planner/reactive_task_local_planner_runtime.hpp"
#include "controller/reactive_task/controller/reactive_task_types.hpp"

namespace arm_controller::controller::reactive_task {

struct ReactiveTaskExecutionContext {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double planner_time_sec{0.0};
  double tracked_reference_time_sec{0.0};
  double planner_tick_accumulator{0.0};
  int planner_tick{0};
  int neo_iter{0};
  int safety_log_stride{1};
  int effective_max_planner_ticks{1};
  double active_duration_sec{0.0};

  ExecutionPhase phase{ExecutionPhase::Track};

  Eigen::VectorXd ik_seed_q;
  Eigen::Isometry3d hold_pose{Eigen::Isometry3d::Identity()};
  bool hold_pose_valid{false};
  bool hold_ready{false};
  bool active_segment_is_recovery{false};
  int recovery_boost_ticks{0};
  int phase_ticks{0};
  int hard_collision_margin_cycles{0};

  Eigen::VectorXd q_prev_feedback;
  bool q_prev_feedback_valid{false};
  Eigen::VectorXd previous_qdot_reference;
  bool previous_qdot_reference_valid{false};
  Eigen::Matrix<double, 6, 1> previous_nominal_twist{
      Eigen::Matrix<double, 6, 1>::Zero()};
  bool previous_nominal_twist_valid{false};
  LocalPlannerRuntime local_planner;
  AsyncLocalPlannerRunner local_planner_runner;

  int path_follow_joint_anchor_index_state{0};

  double best_pos_err{std::numeric_limits<double>::infinity()};
  double best_ori_err{std::numeric_limits<double>::infinity()};
  double best_path_progress{0.0};
  double best_obstacle_min_distance{-std::numeric_limits<double>::infinity()};
  double best_obstacle_guidance_gate{std::numeric_limits<double>::infinity()};
  double previous_pos_err{std::numeric_limits<double>::infinity()};
  double previous_ori_err{std::numeric_limits<double>::infinity()};
  bool previous_task_error_valid{false};

  int no_motion_cycles{0};
  int no_progress_cycles{0};

  double last_task_residual_norm{0.0};
  double last_obstacle_guidance_gate{0.0};
  double last_obstacle_min_distance{std::numeric_limits<double>::quiet_NaN()};
  double last_path_progress{0.0};
  bool last_reference_finished{false};
  bool last_qdot_limit_violation{false};
  WholeBodyStatusSnapshot last_whole_body_status{};
  Eigen::Vector3d last_clearance_direction{Eigen::Vector3d::Zero()};
  bool last_clearance_direction_valid{false};
};

} // namespace arm_controller::controller::reactive_task
