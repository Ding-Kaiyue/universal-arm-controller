#pragma once

#include <Eigen/Core>

#include <vector>

#include "algorithm/cartesian_path_planner/types.hpp"

namespace arm_controller::controller::reactive_task {

class WholeBodyLocalTargetSelector {
public:
  using TimedJointTrajectory =
      arm_controller::algorithm::cartesian_path_planner::TimedJointTrajectory;

  struct Config {
    int horizon_steps{10};
    double dt_sec{0.05};
  };

  struct Input {
    Eigen::VectorXd q_current;
    TimedJointTrajectory global_reference;
    double global_time_sec{0.0};
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;
  };

  struct Output {
    using StateList =
        std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>;

    bool ok{false};
    StateList references;
    Eigen::VectorXd target;
    double dt_sec{0.0};
  };

  WholeBodyLocalTargetSelector();
  explicit WholeBodyLocalTargetSelector(Config config);

  bool select(const Input& input, Output* output) const;

private:
  Config config_;
};

}  // namespace arm_controller::controller::reactive_task
