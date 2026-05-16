#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <future>
#include <limits>
#include <vector>

#include "reactive_task_local_planner.hpp"
#include "reactive_task_types.hpp"

namespace arm_controller::controller::reactive_task {

using JointVectorList =
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>;

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
  Eigen::Isometry3d last_local_planner_target_pose{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 1> last_local_planner_target_twist{
      Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::VectorXd last_local_planner_joint_target;
  double last_local_planner_joint_target_dt_sec{0.0};
  double last_local_planner_update_time_sec{-std::numeric_limits<double>::infinity()};
  bool last_local_planner_output_valid{false};
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
      local_planner_target_poses;
  std::vector<
      Eigen::Matrix<double, 6, 1>,
      Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>>
      local_planner_target_twists;
  JointVectorList local_planner_joint_targets;
  double local_planner_start_time_sec{0.0};
  std::size_t local_planner_start_index{0};
  double local_planner_dt_sec{0.05};
  bool local_planner_trajectory_valid{false};
  std::future<ReactiveTaskLocalPlanner::Output> pending_local_planner_future;
  bool pending_local_planner_valid{false};
  double pending_local_planner_request_time_sec{
      -std::numeric_limits<double>::infinity()};
  int pending_local_planner_tick{0};
  int local_planner_generation{0};
  int pending_local_planner_generation{0};
  Eigen::VectorXd pending_local_planner_q_start;
  Eigen::VectorXd pending_local_planner_q_goal;

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
