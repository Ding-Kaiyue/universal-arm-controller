#pragma once

#include "algorithm/cartesian_path_planner/types.hpp"
#include "controller/reactive_task/local_planner/whole_body_polynomial_trajectory.hpp"

#include <functional>

namespace arm_controller::controller::reactive_task {

class WholeBodyLbfgsOptimizer {
public:
  using StateList = WholeBodyPolynomialTrajectory::StateList;
  using JointStateValidatorFn =
      arm_controller::algorithm::cartesian_path_planner::PathPlanningInput::
          JointStateValidatorFn;
  using JointSegmentValidatorFn =
      arm_controller::algorithm::cartesian_path_planner::PathPlanningInput::
          JointSegmentValidatorFn;
  struct CollisionCostGradient {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double cost{0.0};
    Eigen::VectorXd gradient;
    bool valid{false};
  };

  using CollisionCostGradientFn = std::function<CollisionCostGradient(
      const Eigen::VectorXd& /*q*/,
      double /*safe_distance*/)>;

  struct Config {
    int max_iterations{12};
    double initial_step_size{0.20};
    double min_step_size{1.0e-4};
    double reference_weight{0.0};
    double base_reference_weight{0.0};
    double yaw_reference_weight{0.0};
    double base_progress_weight{0.0};
    double smoothness_weight{0.20};
    double velocity_weight{0.20};
    double collision_weight{50000.0};
    double time_weight{1.0};
    double obstacle_safe_margin{0.05};
    int constrain_points_per_piece{16};
    double max_base_vx{0.20};
    double max_base_vy{0.20};
    double max_base_wz{0.45};
    double max_arm_qdot{0.45};
  };

  struct Input {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    WholeBodyPolynomialTrajectory initial_trajectory;
    StateList references;
    Eigen::Vector2d base_progress_direction{Eigen::Vector2d::Zero()};
    double base_progress_tolerance{0.02};
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;
    double safe_distance{0.05};
    JointStateValidatorFn state_validator;
    JointSegmentValidatorFn segment_validator;
    CollisionCostGradientFn collision_cost_gradient_fn;
  };

  struct Output {
    bool ok{false};
    WholeBodyPolynomialTrajectory trajectory;
    double initial_cost{0.0};
    double final_cost{0.0};
    int iterations{0};
  };

  WholeBodyLbfgsOptimizer();
  explicit WholeBodyLbfgsOptimizer(Config config);

  bool optimize(const Input& input, Output* output) const;

private:
  Config config_;
};

}  // namespace arm_controller::controller::reactive_task
