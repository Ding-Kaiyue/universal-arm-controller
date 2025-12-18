#include "arm_controller/utils/velocity_abb_solver.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

namespace arm_controller::utils {

/* =====================================================================
 * Singularity Metric
 * ===================================================================== */

double VelocityABBSolver::minSingularValue(const Eigen::MatrixXd& J)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        J, Eigen::ComputeThinU | Eigen::ComputeThinV);

    if (svd.singularValues().size() == 0)
        return 0.0;

    return svd.singularValues().tail(1)(0);
}

bool VelocityABBSolver::check_workspace_boundary(
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

/* =====================================================================
 * ABB Velocity Scaling Law
 *
 *  - Full speed in safe region
 *  - Smooth quadratic decay near singularity
 *  - Never reaches zero (crawl speed)
 * ===================================================================== */
double VelocityABBSolver::abbVelocityScale(double sigma_min)
{
    constexpr double SIGMA_FULL  = 0.10;  // no limitation
    constexpr double SIGMA_SLOW  = 0.02;  // strong singularity
    constexpr double SCALE_FLOOR = 0.05;  // 5% minimum speed

    if (sigma_min >= SIGMA_FULL)
        return 1.0;

    if (sigma_min <= SIGMA_SLOW)
        return SCALE_FLOOR;

    double x = (sigma_min - SIGMA_SLOW) / (SIGMA_FULL - SIGMA_SLOW);
    return SCALE_FLOOR + (1.0 - SCALE_FLOOR) * x * x;
}

/* =====================================================================
 * ABB Adaptive Damped Pseudoinverse
 *
 *  - Zero damping far from singularity
 *  - Quadratic damping near singularity
 * ===================================================================== */

Eigen::MatrixXd VelocityABBSolver::abbDampedPseudoInverse(
    const Eigen::MatrixXd& J,
    double sigma_min)
{
    constexpr double SIGMA_DAMP = 0.05;
    constexpr double LAMBDA_MAX = 0.1;

    double lambda = 0.0;
    if (sigma_min < SIGMA_DAMP) {
        double r = 1.0 - sigma_min / SIGMA_DAMP;
        lambda = LAMBDA_MAX * r * r;
    }

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


/* =====================================================================
 * Main Solver
 * ===================================================================== */

bool VelocityABBSolver::solve(
    const Eigen::MatrixXd& J,
    const Eigen::VectorXd& v_ee,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_min_pos,
    const Eigen::VectorXd& q_max_pos,
    Eigen::VectorXd& qd_out,
    const Eigen::VectorXd& qd_max,
    const rclcpp::Logger&)
{
    const int dof = J.cols();

    if (v_ee.norm() < 1e-9) {
        qd_out.setZero(dof);
        scale_state_ = 1.0;
        return true;
    }

    /* 1. Direction */
    const double v_mag = v_ee.norm();
    const Eigen::VectorXd v_dir = v_ee / v_mag;

    /* 2. Singularity */
    const double sigma_min = minSingularValue(J);

    /* 3. ABB speed scaling */
    constexpr double ALPHA = 0.9;
    const double scale_target = abbVelocityScale(sigma_min);
    scale_state_ = ALPHA * scale_state_ + (1.0 - ALPHA) * scale_target;
    const double v_cmd = v_mag * scale_state_;

    /* 4. DLS */
    Eigen::MatrixXd J_pinv = abbDampedPseudoInverse(J, sigma_min);

    /* 5. Primary task */
    Eigen::VectorXd qd_task = J_pinv * (v_cmd * v_dir);

    /* 6. Nullspace projector */
    Eigen::MatrixXd N =
        Eigen::MatrixXd::Identity(dof, dof) - J_pinv * J;

    /* 7. Joint limit avoidance (ABB-style adaptive) */
    Eigen::VectorXd grad =
        jointLimitGradient(q, q_min_pos, q_max_pos);

    // proximity-based gain
    double proximity = grad.cwiseAbs().maxCoeff();  // [0, ~1]
    constexpr double K_NULL_MAX = 0.6;
    double k_null = K_NULL_MAX * proximity;

    Eigen::VectorXd qd_null = -k_null * grad;

    /* 8. Final joint velocity */
    qd_out = qd_task + N * qd_null;

    /* 9. Uniform velocity scaling (NO joint clamp) */
    double scale = 1.0;
    for (int i = 0; i < dof; ++i) {
        if (std::abs(qd_out(i)) > qd_max(i)) {
            scale = std::min(scale, qd_max(i) / std::abs(qd_out(i)));
        }
    }
    qd_out *= scale;

    return true;
}


Eigen::VectorXd VelocityABBSolver::jointLimitGradient(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max)
{
    const int n = q.size();
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(n);

    for (int i = 0; i < n; ++i) {
        const double q_mid = 0.5 * (q_min(i) + q_max(i));
        const double range = q_max(i) - q_min(i);

        if (range > 1e-6) {
            double x = (q(i) - q_mid) / range;
            grad(i) = 2.0 * x;  // ∂/∂q
        }
    }
    return grad;
}


}  // namespace arm_controller::utils
