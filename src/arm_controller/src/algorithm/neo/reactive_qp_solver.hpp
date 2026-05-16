#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <osqp.h>
#include <string>
#include <vector>

#include "reactive_qp_problem.hpp"

namespace arm_controller::algorithm::reactive_qp {

#if defined(OSQP_VERSION)
#define ARM_CONTROLLER_OSQP_LEGACY_API 1
using OsqpInt = c_int;
using OsqpFloat = c_float;
using OsqpCscMatrix = csc;
using OsqpSolverHandle = OSQPWorkspace;
#else
#define ARM_CONTROLLER_OSQP_LEGACY_API 0
using OsqpInt = OSQPInt;
using OsqpFloat = OSQPFloat;
using OsqpCscMatrix = OSQPCscMatrix;
using OsqpSolverHandle = OSQPSolver;
#endif

struct ReactiveQpSolverConfig {
  bool warm_start{true};
  bool verbose{false};
  int max_iterations{4000};
  double absolute_tolerance{1e-5};
  double relative_tolerance{1e-5};
};

class ReactiveQpSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ReactiveQpSolver(
      ReactiveQpSolverConfig config = ReactiveQpSolverConfig{});
  ~ReactiveQpSolver();

  bool solve(
      const ReactiveQpProblem& problem,
      Eigen::VectorXd& out_solution,
      std::string* error = nullptr);

 private:
  struct OwnedCscMatrix {
    OsqpCscMatrix view{};
    std::vector<OsqpInt> outer_index;
    std::vector<OsqpInt> inner_index;
    std::vector<OsqpFloat> values;
  };

  bool configureSolver(const ReactiveQpProblem& problem);
  void clearWorkspace();
  static bool fillCscMatrix(
      const Eigen::SparseMatrix<double>& matrix,
      OwnedCscMatrix& storage);
#if ARM_CONTROLLER_OSQP_LEGACY_API
  static bool diagnoseOsqpData(const OSQPData& data);
#else
  static bool diagnoseOsqpData(
      const OwnedCscMatrix& hessian,
      const OwnedCscMatrix& constraints,
      const std::vector<OsqpFloat>& gradient,
      const std::vector<OsqpFloat>& lower_bound,
      const std::vector<OsqpFloat>& upper_bound,
      int num_variables,
      int num_constraints);
#endif

  ReactiveQpSolverConfig config_;
  OsqpSolverHandle* workspace_{nullptr};
#if ARM_CONTROLLER_OSQP_LEGACY_API
  OSQPData data_{};
#endif
  OSQPSettings settings_{};
  OwnedCscMatrix hessian_csc_;
  OwnedCscMatrix constraint_csc_;
  std::vector<OsqpFloat> gradient_storage_;
  std::vector<OsqpFloat> lower_bound_storage_;
  std::vector<OsqpFloat> upper_bound_storage_;
  int cached_num_variables_{-1};
  int cached_num_constraints_{-1};
  bool initialized_{false};
};

}  // namespace arm_controller::algorithm::reactive_qp
