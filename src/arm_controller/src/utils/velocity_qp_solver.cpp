#include "arm_controller/utils/velocity_qp_solver.hpp"
#include <Eigen/SVD>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>

namespace arm_controller::utils {

void VelocityQPSolver::check_direction_feasibility(
    const Eigen::MatrixXd &J,
    const Eigen::VectorXd &v_ee,
    double &out_feasibility_ratio,
    double &out_residual_norm,
    double &out_alignment,
    double &out_cond)
{
    // SVD 分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd sv = svd.singularValues();
    double min_sv = sv.size() ? sv(sv.size()-1) : 0.0;
    double max_sv = sv.size() ? sv(0) : 0.0;

    // 条件数
    out_cond = (min_sv > EPSILON_SV) ? (max_sv / min_sv) : 1e12;

    // 投影到 Jacobian 列空间
    Eigen::VectorXd v_proj = Eigen::VectorXd::Zero(v_ee.size());
    if (svd.matrixU().cols() > 0) {
        v_proj = svd.matrixU() * (svd.matrixU().transpose() * v_ee);
    }

    // 计算可行性指标
    out_residual_norm = (v_ee - v_proj).norm();
    out_feasibility_ratio = (v_proj.norm() / std::max(EPSILON_SV, v_ee.norm()));
    out_alignment = (v_proj.norm() > EPSILON_SV) ? (v_proj.dot(v_ee) / (v_proj.norm() * v_ee.norm())) : 0.0;
}

bool VelocityQPSolver::post_verify_direction(
    const Eigen::MatrixXd &J,
    const Eigen::VectorXd &qd_solution,
    const Eigen::VectorXd &v_ee,
    double allowed_angle_deg,
    const rclcpp::Logger& logger)
{
    Eigen::VectorXd v_actual = J * qd_solution;
    double v_actual_norm = v_actual.norm();
    double v_des_norm = v_ee.norm();

    // 如果速度太小，认为是停的（正常）
    if (v_actual_norm < 1e-9 || v_des_norm < 1e-9) {
        return true;  // 视为安全
    }

    // 计算夹角余弦
    double cos_thresh = std::cos(allowed_angle_deg * M_PI / 180.0);
    double cos_val = v_actual.dot(v_ee) / (v_actual_norm * v_des_norm);

    if (cos_val < cos_thresh) {
        RCLCPP_WARN(logger,
            "❗ Direction mismatch after QP: cos=%.6f < %.6f (angle=%.2f°). Stopping motion.",
            cos_val, cos_thresh,
            std::acos(std::min(1.0, std::max(-1.0, cos_val))) * 180.0 / M_PI);
        return false;  // 方向偏离过大，不允许运动
    }

    return true;  // 方向一致，允许运动
}

bool VelocityQPSolver::check_workspace_boundary(
    const std::vector<double>& joint_positions,
    const Eigen::VectorXd& qd_command,
    const std::vector<std::string>& joint_names,
    const std::shared_ptr<HardwareManager>& hardware_manager,
    double dt)
{
    if (!hardware_manager) {
        return false;
    }

    // 预测下一个关节位置
    Eigen::VectorXd q_current = Eigen::Map<const Eigen::VectorXd>(joint_positions.data(), joint_positions.size());
    Eigen::VectorXd q_next = q_current + qd_command * dt;

    std::vector<double> q_next_vec(q_next.data(), q_next.data() + q_next.size());

    // 检查预测的关节位置是否在关节限制范围内
    bool would_violate_limits = false;

    for (size_t i = 0; i < q_next_vec.size() && i < joint_names.size(); ++i) {
        JointLimits limits;
        hardware_manager->get_joint_limits(joint_names[i], limits);

        if (limits.has_position_limits) {
            // 检查当前位置是否已经超出限制
            bool current_exceeds_min = joint_positions[i] < limits.min_position;
            bool current_exceeds_max = joint_positions[i] > limits.max_position;

            // 检查下一个位置是否会超出限制
            bool next_exceeds_min = q_next_vec[i] < limits.min_position;
            bool next_exceeds_max = q_next_vec[i] > limits.max_position;

            // 只在以下情况下阻止：
            // 1. 当前已经超出，但命令会继续违反（进一步超出）
            if ((current_exceeds_min && next_exceeds_min && q_next_vec[i] < joint_positions[i]) ||
                (current_exceeds_max && next_exceeds_max && q_next_vec[i] > joint_positions[i])) {
                would_violate_limits = true;
                break;
            }

            // 2. 当前在限制内，但命令会导致超出
            if (!current_exceeds_min && !current_exceeds_max &&
                (next_exceeds_min || next_exceeds_max)) {
                would_violate_limits = true;
                break;
            }
        }
    }

    return !would_violate_limits;
}

bool VelocityQPSolver::solve_velocity_qp(
    const Eigen::MatrixXd &J,
    const Eigen::VectorXd &v_ee,
    Eigen::VectorXd &qd_solution,
    const Eigen::VectorXd &qd_min,
    const Eigen::VectorXd &qd_max,
    const rclcpp::Logger& logger)
{
    const int dof = J.cols();
    const int task_dim = J.rows();

    double v_magnitude = v_ee.norm();
    if (v_magnitude < 1e-9) {
        qd_solution = Eigen::VectorXd::Zero(dof);
        return true;
    }
    Eigen::VectorXd v_direction = v_ee / v_magnitude;

    // 前置几何可行性检测
    double feasibility_ratio, residual_norm, alignment, cond;
    check_direction_feasibility(J, v_ee, feasibility_ratio, residual_norm, alignment, cond);

    if (feasibility_ratio < FEASIBILITY_RATIO_THRESHOLD ||
        residual_norm > RESIDUAL_RATIO_THRESHOLD * v_magnitude ||
        cond > CONDITION_NUMBER_THRESHOLD) {
        RCLCPP_WARN(logger,
            "❎ Direction not geometrically feasible (feasibility=%.3f, residual=%.3f, cond=%.3g). Stopping motion.",
            feasibility_ratio, residual_norm, cond);
        qd_solution = Eigen::VectorXd::Zero(dof);
        return true;  // 返回 true 表示"安全地停止了"
    }

    // ===== QP 求解 =====
    const int n_var = dof + 1;
    const int n_constr = task_dim + dof + 1;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_var, n_var);
    H.topLeftCorner(dof, dof) = QDOT_REGULARIZATION * Eigen::MatrixXd::Identity(dof, dof);
    H(dof, dof) = EPSILON_S;

    Eigen::VectorXd f = Eigen::VectorXd::Zero(n_var);
    f(dof) = -1.0;  // Coefficient for s variable (negative to maximize)

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constr, n_var);

    // Task equality constraint: J·q̇ - s·v = 0
    A.topLeftCorner(task_dim, dof) = J;
    A.topRightCorner(task_dim, 1) = -v_ee;

    // Joint velocity box constraint matrix
    A.block(task_dim, 0, dof, dof).setIdentity();

    // Scaling variable constraint coefficient
    A(task_dim + dof, dof) = 1.0;

    // Setup bounds
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_constr, -OsqpEigen::INFTY);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_constr, OsqpEigen::INFTY);

    lb.head(task_dim).setZero();
    ub.head(task_dim).setZero();

    lb.segment(task_dim, dof) = qd_min;
    ub.segment(task_dim, dof) = qd_max;

    lb(task_dim + dof) = 0.0;
    ub(task_dim + dof) = 1.0;

    // Setup and solve QP
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(n_var);
    solver.data()->setNumberOfConstraints(n_constr);

    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    Eigen::SparseMatrix<double> A_sparse = A.sparseView();

    if (!solver.data()->setHessianMatrix(H_sparse) ||
        !solver.data()->setGradient(f) ||
        !solver.data()->setLinearConstraintsMatrix(A_sparse) ||
        !solver.data()->setLowerBound(lb) ||
        !solver.data()->setUpperBound(ub)) {
        RCLCPP_ERROR(logger, "❎ QP setup failed");
        return false;
    }

    if (!solver.initSolver()) {
        RCLCPP_ERROR(logger, "❎ QP init failed");
        return false;
    }

    auto exitflag = solver.solveProblem();
    if (exitflag != OsqpEigen::ErrorExitFlag::NoError) {
        RCLCPP_WARN(logger, "⚠️ QP failed (flag=%d). Trying geometric scaling fallback...", static_cast<int>(exitflag));

        // Fallback: geometric scaling
        const double scale = 0.9;
        bool solved = false;

        for (int iter = 0; iter < 10; ++iter) {
            Eigen::VectorXd v_scaled = v_ee * std::pow(scale, iter + 1);
            A.topRightCorner(task_dim, 1) = -v_scaled;
            Eigen::SparseMatrix<double> A2 = A.sparseView();

            if (!solver.updateLinearConstraintsMatrix(A2)) break;

            lb.head(task_dim).setZero();
            ub.head(task_dim).setZero();

            if (!solver.updateLowerBound(lb) || !solver.updateUpperBound(ub)) break;

            exitflag = solver.solveProblem();
            if (exitflag == OsqpEigen::ErrorExitFlag::NoError) {
                solved = true;
                RCLCPP_INFO(logger, "Fallback solved at iter=%d, scale=%.3f", iter, std::pow(scale, iter + 1));
                break;
            }
        }

        if (!solved) {
            RCLCPP_ERROR(logger, "❎ Fallback scaling also failed");
            return false;
        }
    }

    Eigen::VectorXd solution = solver.getSolution();
    qd_solution = solution.head(dof);
    double s_opt = solution(dof);

    // 后置方向验证
    Eigen::VectorXd v_actual = J * qd_solution;
    double v_actual_norm = v_actual.norm();
    double dir_err = 0.0;

    if (v_actual_norm > 1e-9) {
        dir_err = (v_actual / v_actual_norm - v_direction).norm();

        // Safeguard: rescale if magnitude drifted
        double expected_velocity = s_opt * v_magnitude;
        if (expected_velocity > 1e-9 && v_actual_norm > 1e-9) {
            double velocity_scale = expected_velocity / v_actual_norm;
            if (velocity_scale > 0.5 && velocity_scale < 2.0) {
                qd_solution *= velocity_scale;
                v_actual *= velocity_scale;
            }
        }
    }

    // Final safety check: if direction misalignment exceeds threshold, force stop
    if (!post_verify_direction(J, qd_solution, v_ee, ALLOWED_ANGLE_DEG, logger)) {
        qd_solution = Eigen::VectorXd::Zero(dof);
    }

    return true;
}

} // namespace arm_controller::utils
