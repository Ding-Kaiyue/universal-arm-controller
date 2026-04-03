#pragma once

#include <Eigen/Core>
#include <string>

#include "reactive_qp_problem.hpp"

namespace arm_controller::algorithm::reactive_qp {

class ReactiveQpValidator {
 public:
  static bool validateTaskInput(
      const Eigen::MatrixXd& jacobian_task,
      const Eigen::VectorXd& desired_twist,
      const Eigen::VectorXd& q_current,
      const Eigen::VectorXd& qd_min,
      const Eigen::VectorXd& qd_max,
      const Eigen::VectorXd& q_min,
      const Eigen::VectorXd& q_max,
      std::string* error = nullptr);

  static bool validateProblem(
      const ReactiveQpProblem& problem,
      std::string* error = nullptr);
};

}  // namespace arm_controller::algorithm::reactive_qp
