#include "arm_controller/utils/velocity_strict_solver.hpp"
#include <Eigen/SVD>
#include <cmath>

namespace arm_controller::utils {

/* ============================================================
 * Damped pseudo inverse (numerical safety only)
 * ============================================================ */
Eigen::MatrixXd VelocityStrictSolver::dampedPseudoInverse(
    const Eigen::MatrixXd& J,
    double lambda)
{
    Eigen::MatrixXd JJt = J * J.transpose();
    Eigen::MatrixXd I =
        Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());

    Eigen::MatrixXd A = JJt + lambda * lambda * I;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    if (ldlt.info() != Eigen::Success) {
        return Eigen::MatrixXd::Zero(J.cols(), J.rows());
    }

    return J.transpose() * ldlt.solve(
        Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols()));
}

/* ============================================================
 * Main solver
 * ============================================================ */
bool VelocityStrictSolver::solve(
    const Eigen::MatrixXd& J,
    const Eigen::VectorXd& v_ee,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_min_pos,
    const Eigen::VectorXd& q_max_pos,
    const Eigen::VectorXd& qd_max,
    Eigen::VectorXd& qd_out,
    const rclcpp::Logger& logger)
{
    const int dof = J.cols();

    if (v_ee.norm() < 1e-9) {
        qd_out.setZero(dof);
        return true;
    }

    /* ---------- 1. Damped least squares ---------- */
    constexpr double LAMBDA = 1e-4;
    Eigen::MatrixXd J_pinv = dampedPseudoInverse(J, LAMBDA);

    if (J_pinv.size() == 0) {
        return false;
    }

    qd_out = J_pinv * v_ee;

    /* ---------- 2. Velocity feasibility check ---------- */
    Eigen::VectorXd v_check = J * qd_out;

    if ((v_check - v_ee).norm() > 1e-3) {
        RCLCPP_WARN(
            logger,
            "Cartesian velocity unreachable (projection error %.4e)",
            (v_check - v_ee).norm());
        return false;
    }

    /* ---------- 3. Joint velocity limits ---------- */
    double scale = 1.0;
    for (int i = 0; i < dof; ++i) {
        if (std::abs(qd_out(i)) > qd_max(i)) {
            scale = std::min(scale, qd_max(i) / std::abs(qd_out(i)));
        }
    }
    qd_out *= scale;

    /* ---------- 4. Joint position bounds ---------- */
    for (int i = 0; i < dof; ++i) {
        if (q(i) <= q_min_pos(i) && qd_out(i) < 0.0) {
            return false;
        }
        if (q(i) >= q_max_pos(i) && qd_out(i) > 0.0) {
            return false;
        }
    }

    return true;
}

/* ============================================================
 * Workspace boundary (unchanged)
 * ============================================================ */
bool VelocityStrictSolver::check_workspace_boundary(
    const std::vector<double>& joint_positions,
    const Eigen::VectorXd& qd,
    const std::vector<std::string>& joint_names,
    const std::shared_ptr<HardwareManager>& hardware_manager,
    double dt)
{
    if (!hardware_manager) {
        return false;
    }

    for (size_t i = 0; i < joint_names.size(); ++i) {
        JointLimits lim;
        hardware_manager->get_joint_limits(joint_names[i], lim);
        if (!lim.has_position_limits) continue;

        double q_next = joint_positions[i] + qd[i] * dt;
        if (q_next < lim.min_position || q_next > lim.max_position) {
            return false;
        }
    }
    return true;
}

}  // namespace arm_controller::utils
