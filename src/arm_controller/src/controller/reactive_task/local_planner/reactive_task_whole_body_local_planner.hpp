#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "algorithm/cartesian_path_planner/types.hpp"
#include "controller/reactive_task/local_planner/reactive_task_local_planner.hpp"
#include "controller/reactive_task/local_planner/whole_body_lbfgs_optimizer.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;

class WholeBodyLocalPlanner final {
public:
  using JointVectorList = LocalPlannerJointVectorList;

  struct Config {
    int horizon_steps{10};
    double dt_sec{0.05};
    int optimization_iterations{4};
    double reference_weight{0.0};
    double base_reference_weight{0.0};
    double yaw_reference_weight{0.0};
    double base_progress_weight{0.0};
    double smoothness_weight{0.20};
    double current_state_weight{0.40};
    double collision_weight{50000.0};
    double time_weight{1.0};
    double obstacle_safe_margin{0.05};
    int constrain_points_per_piece{16};
    double collision_repair_step{0.04};
    int collision_repair_samples{6};
    double max_base_vx{0.20};
    double max_base_vy{0.20};
    double max_base_wz{0.45};
    double max_arm_qdot{0.45};
  };

  struct Input {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::VectorXd q_current;
    cp::TimedJointTrajectory global_reference;
    double global_time_sec{0.0};
    double safe_distance{0.05};
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;
    cp::PathPlanningInput::JointStateValidatorFn joint_state_validator;
    cp::PathPlanningInput::JointSegmentValidatorFn joint_segment_validator;
    WholeBodyLbfgsOptimizer::CollisionCostGradientFn collision_cost_gradient_fn;
  };

  struct Output {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool ok{false};
    bool used{false};
    std::string error;
    JointVectorList joint_targets;
    double dt_sec{0.0};
    Eigen::VectorXd next_joint_target;
    Eigen::VectorXd next_joint_velocity;
    int sampled_steps{0};
    int repaired_states{0};
    bool truncated_by_collision{false};
    bool replanned_window{false};
    bool used_optimized_trajectory{false};
    bool used_frontend_fallback{false};
  };

  WholeBodyLocalPlanner();
  explicit WholeBodyLocalPlanner(Config cfg);

  void configure(Config cfg);
  bool compute(const Input& input, Output* output) const;

private:
  Config cfg_;
};

}  // namespace arm_controller::controller::reactive_task
