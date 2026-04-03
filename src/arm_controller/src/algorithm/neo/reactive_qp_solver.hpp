#pragma once

#include <Eigen/Core>
#include <OsqpEigen/OsqpEigen.h>
#include <string>

#include "reactive_qp_problem.hpp"

namespace arm_controller::algorithm::reactive_qp {

struct ReactiveQpSolverConfig {
  bool warm_start{true};
  bool verbose{false};
  int max_iterations{4000};
  double absolute_tolerance{1e-5};
  double relative_tolerance{1e-5};
};

class ReactiveQpSolver {
 public:
  explicit ReactiveQpSolver(
      ReactiveQpSolverConfig config = ReactiveQpSolverConfig{});

  bool solve(
      const ReactiveQpProblem& problem,
      Eigen::VectorXd& out_solution,
      std::string* error = nullptr);

 private:
  bool configureSolver(const ReactiveQpProblem& problem);

  ReactiveQpSolverConfig config_;
  OsqpEigen::Solver solver_;
  int cached_num_variables_{-1};
  int cached_num_constraints_{-1};
  bool initialized_{false};
};

}  // namespace arm_controller::algorithm::reactive_qp
