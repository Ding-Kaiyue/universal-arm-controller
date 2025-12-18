#include "arm_controller/utils/velocity_qp_solver.hpp"
#include <Eigen/SVD>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>

namespace arm_controller::utils {

static constexpr double W_TASK   = 1.0;
static constexpr double W_QDOT   = 1e-4;
static constexpr double W_LAMBDA = 1e-2;
static constexpr double W_SMOOTH = 0.2;
static constexpr double W_POSTURE = 0.05;

static constexpr double DT = 0.01;
static constexpr double DAMPING = 0.01;


bool VelocityQPSolver::check_workspace_boundary(
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

Eigen::MatrixXd VelocityQPSolver::dampedPseudoInverse(
    const Eigen::MatrixXd& J, double lambda)
{
    Eigen::MatrixXd JJt = J * J.transpose();
    Eigen::MatrixXd I =
        Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());
    return J.transpose() * (JJt + lambda * lambda * I).inverse();
}

double VelocityQPSolver::computeMinSingularValue(const Eigen::MatrixXd& J)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd sv = svd.singularValues();
    if (sv.size() == 0) return 0.0;
    return sv(sv.size() - 1);
}

bool VelocityQPSolver::solve_velocity_qp(
    const Eigen::MatrixXd &J,
    const Eigen::VectorXd &v_ee,
    Eigen::VectorXd &qd_out,
    const Eigen::VectorXd &qd_min,
    const Eigen::VectorXd &qd_max,
    const rclcpp::Logger& logger)
{
    const int n = J.cols();
    // const int m = J.rows();  // Not used in this legacy solver

    if (qd_prev_.size() != n) {
        qd_prev_ = Eigen::VectorXd::Zero(n);
    }

    if (v_ee.norm() < 1e-9) {
        qd_out = Eigen::VectorXd::Zero(n);
        qd_prev_ = qd_out;
        return true;
    }

    const int nv = n + 1;   // qdot + lambda

    // ============== Hessian matrix ==============
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nv, nv);

    H.topLeftCorner(n, n) =
        2.0 * (W_TASK * J.transpose() * J +
               W_QDOT * Eigen::MatrixXd::Identity(n, n) +
               W_SMOOTH * Eigen::MatrixXd::Identity(n, n));

    H.topRightCorner(n, 1) = -2.0 * W_TASK * J.transpose() * v_ee;
    H.bottomLeftCorner(1, n) = H.topRightCorner(n, 1).transpose();
    H(n, n) = 2.0 * (W_TASK * v_ee.squaredNorm() + W_LAMBDA);

    // ============== Gradient vector ==============
    Eigen::VectorXd f = Eigen::VectorXd::Zero(nv);
    f.head(n) = -2.0 * W_SMOOTH * qd_prev_;
    f(n) = -2.0 * W_LAMBDA;

    // ============== Constraints ==============
    const int nc = 4 * n + 2;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nc, nv);
    Eigen::VectorXd lb = Eigen::VectorXd::Zero(nc);
    Eigen::VectorXd ub = Eigen::VectorXd::Zero(nc);

    int r = 0;

    // qdot limits
    A.block(r, 0, n, n).setIdentity();
    lb.segment(r, n) = qd_min;
    ub.segment(r, n) = qd_max;
    r += n;

    // position prediction
    A.block(r, 0, n, n) = DT * Eigen::MatrixXd::Identity(n, n);
    lb.segment(r, n).setConstant(-1e3);
    ub.segment(r, n).setConstant(1e3);
    r += n;

    // lambda
    A(r, n) = 1.0;
    lb(r) = 0.0;
    ub(r) = 1.0;
    r += 1;

    // ============== Solve QP ==============
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(nv);
    solver.data()->setNumberOfConstraints(r);

    // 转换为compressed sparse format (CSR)
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    Eigen::SparseMatrix<double> A_sparse = A.topRows(r).sparseView();

    solver.data()->setHessianMatrix(H_sparse);
    solver.data()->setGradient(f);
    solver.data()->setLinearConstraintsMatrix(A_sparse);
    solver.data()->setLowerBound(lb.head(r));
    solver.data()->setUpperBound(ub.head(r));

    if (!solver.initSolver()) {
        RCLCPP_ERROR(logger, "❎ QP init failed");
        return false;
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        RCLCPP_WARN(logger, "❎ QP solve failed, forcing stop");
        qd_out = Eigen::VectorXd::Zero(n);
        return false;
    }

    Eigen::VectorXd sol = solver.getSolution();
    qd_out = sol.head(n);
    qd_prev_ = qd_out;

    return true;
}


bool VelocityQPSolver::solve_velocity_hqp(
    const Eigen::MatrixXd& J,
    const Eigen::VectorXd& v_ee,
    const Eigen::VectorXd& q,
    Eigen::VectorXd& qd_out,
    const Eigen::VectorXd& qd_min,
    const Eigen::VectorXd& qd_max,
    const std::vector<std::string>& joint_names,
    const std::shared_ptr<HardwareManager>& hardware_manager,
    const rclcpp::Logger& logger)
{
    const int n = J.cols();
    const int task_dim = J.rows();

    if (qd_prev_.size() != n)
        qd_prev_ = Eigen::VectorXd::Zero(n);

    if (v_ee.norm() < 1e-9) {
        qd_out.setZero();
        qd_prev_ = qd_out;
        return true;
    }

    double v_mag = v_ee.norm();
    Eigen::VectorXd v_dir = v_ee / v_mag;

    /* ===================================================================
     * FOUR-STAGE INDUSTRIAL JOG CONTROL
     * Philosophy: "宁可不动，也绝不允许偏离这个方向"
     * ===================================================================
     */

    /* ========== STAGE 1: Compute Singularity Metrics ==========
     *
     * Compute σ_min for later use in velocity scaling
     * No hard stop logic - continuous deceleration handles all singularity levels
     */

    double sigma_min = computeMinSingularValue(J);

    /* ========== STAGE 2: Direction-Preserving QP ==========
     *
     * CRITICAL: Must minimize ||q̇|| to prevent "joint cancellation" artifacts
     * Without this, QP would use large opposite velocities to satisfy constraints
     *
     * Objective:
     *   min  EPS_QDOT * ||q̇||² − λ
     * s.t. J·q̇ = λ·v_dir
     *      q̇_min ≤ q̇ ≤ q̇_max
     *      0 ≤ λ ≤ v_mag
     */

    const int nv = n + 1;
    static constexpr double EPS_QDOT = 1e-3;  // Regularization to prevent joint explosion (increased for high-DOF systems)

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nv, nv);
    // q̇ regularization: prevents QP from using massive opposite joint velocities
    H.topLeftCorner(n, n) = EPS_QDOT * Eigen::MatrixXd::Identity(n, n);
    // Note: H(n,n) = 0 for λ because we want to maximize it via linear term

    Eigen::VectorXd f = Eigen::VectorXd::Zero(nv);
    f(n) = -1.0;  // Maximize λ (negative coefficient to minimize -λ)

    const int nc = task_dim + n + 1;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nc, nv);
    Eigen::VectorXd lb(nc), ub(nc);

    int r = 0;

    A.block(r, 0, task_dim, n) = J;
    A.block(r, n, task_dim, 1) = -v_dir;
    lb.segment(r, task_dim).setZero();
    ub.segment(r, task_dim).setZero();
    r += task_dim;

    A.block(r, 0, n, n).setIdentity();
    lb.segment(r, n) = qd_min;
    ub.segment(r, n) = qd_max;
    r += n;

    // Lambda bounds: allow full range
    // QP will naturally reduce lambda when constraints cannot be satisfied
    // No pre-emptive scaling based on singularity
    A(r, n) = 1.0;
    lb(r) = 0.0;
    ub(r) = v_mag;
    r++;

    hqp_solver_.settings()->setWarmStart(true);
    hqp_solver_.settings()->setVerbosity(false);
    hqp_solver_.settings()->setMaxIteration(100);
    hqp_solver_.settings()->setAbsoluteTolerance(1e-4);  // Tighten tolerance
    hqp_solver_.settings()->setRelativeTolerance(1e-4);

    hqp_solver_.data()->setNumberOfVariables(nv);
    hqp_solver_.data()->setNumberOfConstraints(r);

    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    Eigen::SparseMatrix<double> A_sparse = A.topRows(r).sparseView();

    // Clear previous matrices if solver was already initialized (warm start mode)
    if (solver_initialized_) {
        hqp_solver_.data()->clearHessianMatrix();
        hqp_solver_.data()->clearLinearConstraintsMatrix();
    }

    hqp_solver_.data()->setHessianMatrix(H_sparse);
    hqp_solver_.data()->setGradient(f);
    hqp_solver_.data()->setLinearConstraintsMatrix(A_sparse);
    hqp_solver_.data()->setLowerBound(lb.head(r));
    hqp_solver_.data()->setUpperBound(ub.head(r));

    // Debug: Print QP setup
    RCLCPP_DEBUG(logger, "[HQP-S2] QP setup: nv=%d, nc=%d, v_mag=%.4f", nv, r, v_mag);
    RCLCPP_DEBUG(logger, "[HQP-S2] H(n,n)=%.2e, f(n)=%.4f", H(n,n), f(n));

    if (!solver_initialized_) {
        if (!hqp_solver_.initSolver()) {
            RCLCPP_ERROR(logger, "[HQP] OSQP init failed");
            qd_out.setZero();
            return false;
        }
        solver_initialized_ = true;
    }

    if (hqp_solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        RCLCPP_WARN(logger, "[HQP-S2] QP solve failed");
        qd_out.setZero();
        qd_prev_ = qd_out;
        return true;
    }

    Eigen::VectorXd sol = hqp_solver_.getSolution();
    x_prev_ = sol;

    Eigen::VectorXd qd1 = sol.head(n);
    double lambda = sol(n);

    RCLCPP_INFO(logger, "[HQP-S2] QP solved: λ=%.4f, |qd1|=%.4f, σ=%.4f",
                lambda, qd1.norm(), sigma_min);

    // Near singularity warning only
    if (sigma_min < 0.03) {
        RCLCPP_WARN(logger, "[HQP-S2] Near singularity (σ=%.4f). Lambda reduced to %.1f%%.",
                    sigma_min, (lambda / v_mag) * 100.0);
    }

    /* ========== STAGE 3: Null-Space Optimization (Conditional) ==========
     *
     * Only apply if λ is near maximum
     * Otherwise trust the QP constraint solution
     */

    const double lambda_ratio = (v_mag > 1e-9) ? (lambda / v_mag) : 0.0;

    // Softer threshold (0.6 instead of 0.8) allows null-space to participate more
    // This helps with configuration improvement near singularities
    if (lambda_ratio < 0.6) {
        qd_out = qd1;
    } else {
        Eigen::MatrixXd J_pinv = dampedPseudoInverse(J, DAMPING);
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - J_pinv * J;

        Eigen::VectorXd q_nominal = q;
        if (hardware_manager) {
            for (int i = 0; i < n && i < (int)joint_names.size(); ++i) {
                JointLimits lim;
                hardware_manager->get_joint_limits(joint_names[i], lim);
                if (lim.has_position_limits) {
                    q_nominal(i) = 0.5 * (lim.min_position + lim.max_position);
                }
            }
        }

        static constexpr double Kp_posture = 0.5;
        Eigen::VectorXd grad =
            W_SMOOTH * (qd_prev_ - qd1) +
            W_POSTURE * Kp_posture * (q_nominal - q);

        Eigen::VectorXd delta_qd = N * grad;

        // Null-space injection magnitude limit
        // Prevent null-space from becoming the dominant force
        static constexpr double NS_MAX_RATIO = 0.2;
        double max_ns = NS_MAX_RATIO * qd1.norm();

        if (delta_qd.norm() > max_ns && max_ns > 1e-9) {
            delta_qd *= max_ns / delta_qd.norm();
        }

        qd_out = qd1 + delta_qd;

        for (int i = 0; i < n; ++i) {
            qd_out(i) = std::clamp(qd_out(i), qd_min(i), qd_max(i));
        }
    }

    /* ========== STAGE 4: Direction Integrity Check ==========
     *
     * Final safety guard: verify direction hasn't drifted
     * If violated, EMERGENCY STOP
     */

    /* ========== STAGE 4: Direction Integrity Monitoring (No Hard Stop) ==========
     *
     * Monitor direction error but don't hard-stop
     * Velocity is already naturally reduced via lambda_max scaling
     * Direction constraint is satisfied by QP equality constraint
     */

    Eigen::VectorXd v_actual = J * qd_out;

    // Use angle error instead of norm error - more physically intuitive
    // Measures actual deviation from desired direction
    double v_actual_norm = v_actual.norm();
    double cos_angle = (v_actual_norm > 1e-9) ?
        (v_actual.dot(v_dir) / v_actual_norm) : 1.0;

    // Clamp to [-1, 1] to avoid numerical issues with acos
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double angle_error_rad = std::acos(cos_angle);
    double angle_error_deg = angle_error_rad * 180.0 / M_PI;

    // Warn if direction deviates significantly
    // ~11° threshold corresponds to cos(11°) ≈ 0.98
    double cos_threshold = 0.98;  // ~11 degree tolerance
    if (cos_angle < cos_threshold && lambda > 1e-6) {
        RCLCPP_WARN(logger, "[HQP-S4] Direction error: %.1f°. Accepting reduced control authority.",
                    angle_error_deg);
    }

    qd_prev_ = qd_out;

    RCLCPP_DEBUG(
        logger,
        "[HQP] λ=%.3f(%.0f%%) | σ=%.4f | |qd|=%.3f | dir_err=%.1f°",
        lambda, lambda_ratio*100, sigma_min, qd_out.norm(), angle_error_deg);

    return true;
}

} // namespace arm_controller::utils
