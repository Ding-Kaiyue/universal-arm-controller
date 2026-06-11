#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "algorithm/cartesian_path_planner/types.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;

class WholeBodyGoalGenerator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kBaseDof = 3;
  static constexpr int kArmDof = 12;
  static constexpr int kFullDof = kBaseDof + kArmDof;

  struct Config {
    double position_tolerance_m{0.02};
    double orientation_tolerance_rad{0.20};
    double safe_distance{0.025};
    double hard_clearance{0.0};
    std::size_t max_goal_candidates{12};
    int optimizer_iterations{80};
    double optimizer_step{0.05};
    double base_xy_weight{0.08};
    double base_yaw_weight{0.04};
    double arm_motion_weight{0.02};
    double posture_weight{0.01};
    double position_weight{120.0};
    double orientation_weight{12.0};
    double collision_weight{80.0};
    bool require_strict_orientation{true};
  };

  struct BaseState {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct Seed {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BaseState base;
    Eigen::VectorXd arm_seed;
  };

  struct Diagnostic {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    bool collision_free{false};
    double left_position_error_m{0.0};
    double right_position_error_m{0.0};
    double left_orientation_error_rad{0.0};
    double right_orientation_error_rad{0.0};
    double min_margin{0.0};
    std::string reason;
    Eigen::VectorXd q_goal;
  };

  struct Input {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::VectorXd q_start;
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;
    Eigen::VectorXd q_nominal;
    Eigen::Isometry3d left_target{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d right_target{Eigen::Isometry3d::Identity()};
    std::vector<Seed, Eigen::aligned_allocator<Seed>> seeds;

    // Given a full 15D state, return left/right tip poses in world frame.
    std::function<bool(const Eigen::VectorXd&, Eigen::Isometry3d*,
                       Eigen::Isometry3d*)>
        full_state_fk;

    // Optional seed projector. The generator uses this as the first pass for
    // each base seed; it is typically implemented with dual-arm IK.
    std::function<bool(const BaseState&, const Eigen::VectorXd&,
                       Eigen::VectorXd*)>
        seed_projector;

    // Full-state collision / validity diagnostic.
    cp::PathPlanningInput::JointStateValidatorFn joint_state_validator;
  };

  struct Output {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<Eigen::VectorXd> q_goal_candidates;
    std::vector<Diagnostic, Eigen::aligned_allocator<Diagnostic>> diagnostics;
  };

  WholeBodyGoalGenerator();
  explicit WholeBodyGoalGenerator(Config config);

  Output generate(const Input& input) const;

  const Config& config() const { return config_; }

private:
  Diagnostic diagnose(const Input& input, const Eigen::VectorXd& q) const;
  Eigen::VectorXd optimizeSeed(const Input& input, const Eigen::VectorXd& seed) const;
  double objective(const Input& input, const Eigen::VectorXd& q) const;
  Eigen::VectorXd finiteDifferenceGradient(const Input& input, const Eigen::VectorXd& q) const;
  Eigen::VectorXd clampToBounds(const Input& input, Eigen::VectorXd q) const;
  bool isDuplicate(const std::vector<Eigen::VectorXd>& goals, const Eigen::VectorXd& q) const;

  Config config_;
};

}  // namespace arm_controller::controller::reactive_task
