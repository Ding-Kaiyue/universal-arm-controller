#include "manipulator_hessian_tensor.hpp"

#include <Eigen/Geometry>

namespace arm_controller::algorithm::reactive_qp {

namespace {

bool fail(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
    return false;
}

}  // namespace

bool ManipulatorHessianTensorBuilder::buildFromJacobian(
    const Eigen::MatrixXd& jacobian_6xn,
    std::vector<Eigen::MatrixXd>& out_dJ_dq,
    std::string* error) {
    if (jacobian_6xn.rows() != 6 || jacobian_6xn.cols() <= 0) {
        return fail(error, "Expected 6xn Jacobian with n > 0.");
    }
    if (!jacobian_6xn.allFinite()) {
        return fail(error, "Jacobian contains non-finite values.");
    }

    const int dof = static_cast<int>(jacobian_6xn.cols());
    out_dJ_dq.clear();
    out_dJ_dq.resize(static_cast<std::size_t>(dof), Eigen::MatrixXd::Zero(6, dof));

    for (int i = 0; i < dof; ++i) {
        Eigen::MatrixXd& dJ_dqi = out_dJ_dq[static_cast<std::size_t>(i)];

        for (int j = 0; j < dof; ++j) {
            // Translational Hessian component:
            //   H_aij = Jw_a x Jv_b, with a=min(i,j), b=max(i,j).
            const int a = (i < j) ? i : j;
            const int b = (i < j) ? j : i;
            const Eigen::Vector3d jw_a = jacobian_6xn.block<3, 1>(3, a);
            const Eigen::Vector3d jv_b = jacobian_6xn.block<3, 1>(0, b);
            dJ_dqi.block<3, 1>(0, j) = jw_a.cross(jv_b);

            // Rotational Hessian component for serial revolute chains:
            //   dJw_j/dq_i = Jw_i x Jw_j, when i < j, else 0.
            if (i < j) {
                const Eigen::Vector3d jw_i = jacobian_6xn.block<3, 1>(3, i);
                const Eigen::Vector3d jw_j = jacobian_6xn.block<3, 1>(3, j);
                dJ_dqi.block<3, 1>(3, j) = jw_i.cross(jw_j);
            }
        }
    }

    return true;
}

}  // namespace arm_controller::algorithm::reactive_qp
