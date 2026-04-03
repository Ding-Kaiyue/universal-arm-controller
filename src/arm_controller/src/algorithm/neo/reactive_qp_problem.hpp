#pragma once

#include <Eigen/Core>
#include <string>

namespace arm_controller::algorithm::reactive_qp {

struct ReactiveQpProblem {
  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  Eigen::MatrixXd constraint_matrix;
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;

  [[nodiscard]] int numVariables() const { return static_cast<int>(gradient.size()); }
  [[nodiscard]] int numConstraints() const { return static_cast<int>(lower_bound.size()); }

  [[nodiscard]] bool isWellFormed(std::string* error = nullptr) const;
};

}  // namespace arm_controller::algorithm::reactive_qp
