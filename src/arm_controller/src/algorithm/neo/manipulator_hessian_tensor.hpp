#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace arm_controller::algorithm::reactive_qp {

// Builds manipulator Hessian tensor slices dJ/dq_i from a 6xn geometric Jacobian.
//
// Jacobian row convention in this project:
//   J.topRows(3)    -> translational part J_v
//   J.bottomRows(3) -> rotational part   J_w
//
// Output layout:
//   out_dJ_dq[i] is a 6xn matrix representing dJ/dq_i.
//
// Assumption:
//   Serial chain of revolute joints.
class ManipulatorHessianTensorBuilder {
public:
    static bool buildFromJacobian(
        const Eigen::MatrixXd& jacobian_6xn,
        std::vector<Eigen::MatrixXd>& out_dJ_dq,
        std::string* error = nullptr);
};

}  // namespace arm_controller::algorithm::reactive_qp

