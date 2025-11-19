#include "cartesian_velocity_controller.hpp"
#include "controller_interface.hpp"
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <chrono>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'CartesianVelocity', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/cartesian_velocity_action/single_arm geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

CartesianVelocityController::CartesianVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<geometry_msgs::msg::TwistStamped>("CartesianVelocity", node),
      base_frame_("base_link")
{
    // 获取HardwareManager实例
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化TF2缓冲和监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // 从配置文件读取基座坐标系（可选）
    node_->get_parameter("controllers.CartesianVelocity.base_frame", base_frame_);
    RCLCPP_INFO(node_->get_logger(), "[CartesianVelocity] Base frame: %s", base_frame_.c_str());

    // 注意：话题订阅在 init_subscriptions() 中创建，当 controller 被激活时调用

    // 预初始化所有mapping的moveit服务
    initialize_moveit_service();
}


void CartesianVelocityController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] CartesianVelocity: not found in hardware configuration."
        );
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "Hardware driver not initialized");
        return;
    }

    // 保存当前激活的 mapping
    active_mapping_ = mapping;
    is_active_ = true;

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    // ════════════════════════════════════════════════════════════════════════════════
    // 在启动时将世界坐标系（world）设置为参考坐标系
    // 后续所有速度命令都相对于全局固定的世界坐标系计算，不受机械臂自身旋转影响
    // ════════════════════════════════════════════════════════════════════════════════

    try {
        // 获取启动时刻的 world 坐标系作为参考
        // 这样所有速度命令都相对于全局世界坐标系，不受机械臂旋转影响
        rclcpp::Time now = node_->get_clock()->now();

        // 在 world 坐标系中，参考矩阵就是单位矩阵（identity）
        // 因为 world 本身就是固定的
        R_world_to_ref_ = Eigen::Matrix3d::Identity();
        reference_frame_ = "world";
        has_reference_frame_ = true;

        RCLCPP_INFO(node_->get_logger(),
            "[%s] ✓ Reference frame set to world. All velocities will be relative to world coordinates (fixed).",
            mapping.c_str());
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] ⚠️ Failed to set world reference frame: %s.",
            mapping.c_str(), ex.what());
        has_reference_frame_ = false;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] CartesianVelocityController activated", mapping.c_str());
}


bool CartesianVelocityController::stop(const std::string& mapping) {
    // 停止处理消息
    is_active_ = false;
    has_reference_frame_ = false;

    // 发送零速度命令停止机械臂
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    if (!joint_names.empty()) {
        std::vector<double> zero_velocities(joint_names.size(), 0.0);
        send_joint_velocities(mapping, zero_velocities);
    }

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] CartesianVelocityController deactivated", mapping.c_str());
    return true;  // 需要钩子状态来安全停止
}

void CartesianVelocityController::initialize_moveit_service() {
    try {
        // 获取所有mapping
        auto all_mappings = hardware_manager_->get_all_mappings();
        if (all_mappings.empty()) {
            RCLCPP_WARN(node_->get_logger(), "❎ CartesianVelocity: No mappings configured");
            return;
        }

        // 为每个mapping初始化规划服务
        for (const auto& mapping : all_mappings) {
            std::string planning_group = hardware_manager_->get_planning_group(mapping);

            if (planning_group.empty()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ❎ CartesianVelocity: No planning group configured, skipping...", mapping.c_str());
                continue;
            }

            try {
                // 创建 MoveItAdapter
                auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, planning_group);

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ CartesianVelocity: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ CartesianVelocity: Exception: %s", mapping.c_str(), e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ CartesianVelocity: Failed to initialize MoveIt services: %s", e.what());
    }
}
void CartesianVelocityController::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    // 只在激活时才处理消息
    if (!is_active_) return;

    auto joint_positions = hardware_manager_->get_current_joint_positions(active_mapping_);
    auto joint_names = hardware_manager_->get_joint_names(active_mapping_);
    if (joint_positions.empty() || joint_names.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ No joint states or names available", active_mapping_.c_str());
        return;
    }

    // Get Jacobian using MoveItAdapter
    if (moveit_adapters_.find(active_mapping_) == moveit_adapters_.end() || !moveit_adapters_[active_mapping_]) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ MoveItAdapter not initialized", active_mapping_.c_str());
        return;
    }

    Eigen::MatrixXd J = moveit_adapters_[active_mapping_]->computeJacobian(joint_positions);
    if (J.rows() == 0 || J.cols() == 0) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ Invalid Jacobian matrix (empty)", active_mapping_.c_str());
        return;
    }

    // Validate Jacobian dimensions
    if (J.rows() < 6) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] ⚠️ Jacobian has only %ld rows (<6 task dimensions). "
            "Check planning group configuration - may be constrained motion.",
            active_mapping_.c_str(), J.rows());
    }
    if (J.cols() < 6) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] ⚠️ Robot has only %ld DoF. Some Cartesian velocities may not be achievable.",
            active_mapping_.c_str(), J.cols());
    }

    // ════════════════════════════════════════════════════════════════════════════════
    // 坐标系处理：所有速度命令都相对于世界坐标系（固定全局坐标系）
    // ════════════════════════════════════════════════════════════════════════════════
    //
    // 工作流程：
    // 1. start() 时：将世界坐标系设置为参考坐标系
    // 2. 每条命令到达时：
    //    - 用户输入的速度被理解为相对于世界坐标系
    //    - 获取当前 base_link 相对于 world 的姿态
    //    - 将世界坐标系中的速度变换到当前 base_link 坐标系
    //    - 基于变换后的速度进行 IK 求解
    //
    // 效果：用户的速度命令始终相对于全局固定的世界坐标系，不会因机械臂旋转而改变
    //      例如：[0.1, 0, 0] 总是沿着世界的 X 轴方向，无论机械臂朝向如何

    Eigen::Vector3d v_linear_raw(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    Eigen::Vector3d v_angular_raw(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);

    Eigen::VectorXd v_ee(6);

    // 如果参考坐标系已锁定为 world，进行坐标变换
    if (has_reference_frame_) {
        try {
            // 获取当前时刻的 world → base_link 变换
            // 需要这个变换来将世界坐标系中的速度转换到 base_link 坐标系
            rclcpp::Time now = node_->get_clock()->now();
            geometry_msgs::msg::TransformStamped tf_world_to_base_current =
                tf_buffer_->lookupTransform("world", base_frame_, now, std::chrono::milliseconds(100));

            Eigen::Quaterniond q_current(
                tf_world_to_base_current.transform.rotation.w,
                tf_world_to_base_current.transform.rotation.x,
                tf_world_to_base_current.transform.rotation.y,
                tf_world_to_base_current.transform.rotation.z
            );
            Eigen::Matrix3d R_world_to_base = q_current.toRotationMatrix().transpose();

            // 直接使用 R_world_to_base 将世界坐标系的速度转换到 base_link 坐标系
            // 因为参考坐标系就是 world，所以 R_world_to_ref_ 是单位矩阵
            Eigen::Vector3d v_linear_base = R_world_to_base * v_linear_raw;
            Eigen::Vector3d v_angular_base = R_world_to_base * v_angular_raw;

            v_ee << v_linear_base(0), v_linear_base(1), v_linear_base(2),
                    v_angular_base(0), v_angular_base(1), v_angular_base(2);

            RCLCPP_DEBUG(node_->get_logger(),
                "[%s] Velocity transformed: world[%.6f,%.6f,%.6f] → base[%.6f,%.6f,%.6f]",
                active_mapping_.c_str(),
                v_linear_raw(0), v_linear_raw(1), v_linear_raw(2),
                v_linear_base(0), v_linear_base(1), v_linear_base(2));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(),
                "[%s] ⚠️ TF lookup failed: %s. Using raw velocity.",
                active_mapping_.c_str(), ex.what());
            v_ee << v_linear_raw(0), v_linear_raw(1), v_linear_raw(2),
                    v_angular_raw(0), v_angular_raw(1), v_angular_raw(2);
        }
    } else {
        // 如果参考坐标系未锁定，直接使用原始速度（备用模式）
        RCLCPP_DEBUG(node_->get_logger(), "[%s] Reference frame not locked, using raw velocity",
            active_mapping_.c_str());
        v_ee << v_linear_raw(0), v_linear_raw(1), v_linear_raw(2),
                v_angular_raw(0), v_angular_raw(1), v_angular_raw(2);
    }

    RCLCPP_DEBUG(node_->get_logger(), "[%s] Final velocity for IK: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        active_mapping_.c_str(), v_ee(0), v_ee(1), v_ee(2), v_ee(3), v_ee(4), v_ee(5));

    Eigen::VectorXd qd(J.cols());
    if (!solve_velocity_qp(active_mapping_, J, v_ee, qd, joint_names)) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ Failed to solve velocity QP. Sending zero velocities to stop.", active_mapping_.c_str());
        // 发送零速度命令来停止机械臂（与工作空间边界检查逻辑一致）
        auto joint_names_list = hardware_manager_->get_joint_names(active_mapping_);
        if (!joint_names_list.empty()) {
            std::vector<double> zero_velocities(joint_names_list.size(), 0.0);
            send_joint_velocities(active_mapping_, zero_velocities);
        }
        return;
    }

    // 检查工作空间边界 - 如果会超出工作空间，停止运动
    if (!check_workspace_boundary(active_mapping_, joint_positions, qd)) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ Workspace boundary violation detected. Sending zero velocities.", active_mapping_.c_str());
        // 发送零速度命令来停止机械臂
        auto joint_names_list = hardware_manager_->get_joint_names(active_mapping_);
        if (!joint_names_list.empty()) {
            std::vector<double> zero_velocities(joint_names_list.size(), 0.0);
            send_joint_velocities(active_mapping_, zero_velocities);
        }
        return;
    }

    std::vector<double> velocities(qd.data(), qd.data() + qd.size());
    send_joint_velocities(active_mapping_, velocities);
}

bool CartesianVelocityController::solve_velocity_qp(
    const std::string& mapping,
    const Eigen::MatrixXd &J,
    const Eigen::VectorXd &v_ee,
    Eigen::VectorXd &qd_solution,
    const std::vector<std::string> &joint_names)
{
    const int dof = J.cols();
    const int task_dim = J.rows();

    double v_magnitude = v_ee.norm();
    if (v_magnitude < 1e-9) {
        qd_solution = Eigen::VectorXd::Zero(dof);
        return true;
    }
    Eigen::VectorXd v_direction = v_ee / v_magnitude;

    // ========== 前置几何可行性检测（第1层防护）==========
    // 目的：在QP求解前快速判断请求的速度方向是否可达
    //
    // 原理：通过SVD分解 J = U·Σ·Vᵀ，检测v_ee是否在J的列空间内
    //       U的列向量构成所有可达末端速度方向的空间
    //
    // 三个关键指标：
    // 1. feasibility_ratio：v_ee中有多大比例可达（0~1）
    //    - 如果 < 95%，说明用户要求的速度中>5%无法达到
    //    - 继续QP会产生"错误方向"，因此直接停止
    //
    // 2. residual_norm：不可达的速度分量幅度
    //    - 如果 > 10% × ||v_ee||，说明有太大部分无法达到
    //    - 即使QP成功，也只能执行极端扭曲的轨迹，不安全
    //
    // 3. cond (condition number)：κ(J) = σ_max / σ_min
    //    - > 1e6：雅可比矩阵病态（接近奇异点）
    //    - QP Hessian会严重失条件数，数值误差放大
    //    - 求解器可能失败或给出垃圾结果
    double feasibility_ratio, residual_norm, alignment, cond;
    check_direction_feasibility(J, v_ee, feasibility_ratio, residual_norm, alignment, cond);

    // 判断方向是否可行
    if (feasibility_ratio < FEASIBILITY_RATIO_THRESHOLD ||
        residual_norm > RESIDUAL_RATIO_THRESHOLD * v_magnitude ||
        cond > CONDITION_NUMBER_THRESHOLD) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] ❎ Direction not geometrically feasible (feasibility=%.3f, residual=%.3f, cond=%.3g). Stopping motion.",
            mapping.c_str(), feasibility_ratio, residual_norm, cond);
        qd_solution = Eigen::VectorXd::Zero(dof);
        return true;  // 返回 true 表示"安全地停止了"
    }

    // Joint limits
    Eigen::VectorXd qd_min(dof), qd_max(dof);
    for (int i = 0; i < dof; ++i) {
        JointLimits limits;
        hardware_manager_->get_joint_limits(joint_names[i], limits);
        double vmax = limits.has_velocity_limits ? limits.max_velocity : 1.0;
        qd_max(i) = vmax;
        qd_min(i) = -vmax;
    }

    // ===== QP 求解：软约束IK with 缩放变量（第2层防护）=====
    // 数学模型：
    //   min  (1/2)ε·||q̇||² - s            (目标函数)
    //   s.t. J·q̇ - s·v_ee = 0            (任务约束-软化)
    //        q̇_min ≤ q̇ ≤ q̇_max         (关节速度约束-硬约束)
    //        0 ≤ s ≤ 1                    (缩放因子约束)
    //
    // 关键思想：引入缩放变量 s
    //   - s=1.0：能完全执行命令速度 v_ee
    //   - 0<s<1：由于约束（如关节限制、奇异点），只能执行 s×v_ee
    //   - s=0：无法执行任何速度（完全被约束）
    //
    // 目标 min(-s) = max(s) 使得优化器自动寻找最大可行速度
    //   即使不能达到100%，也要尽量达到最大的s
    //
    // 正则化项 (1/2)ε·||q̇||²：
    //   - 防止QP在多解时选择无限大的关节速度
    //   - ε=1e-4 在数值稳定性和精度间的平衡点
    //   - 接近奇异点时保证Hessian矩阵有界
    const int n_var = dof + 1;
    const int n_constr = task_dim + dof + 1;

    // Numeric stability parameters
    // eps: Regularization weight for joint velocities in Hessian matrix
    //      - Increased to 1e-4 for better numerical stability in OSQP solver
    //      - Prevents ill-conditioning near singularities and reduces solver failures
    //      - Trade-off: solution may deviate slightly from pseudoinverse but much more stable
    //      - For low-DoF (6 joints): 1e-4 provides good balance; can reduce to 1e-5 for more precision
    //      - For high-DoF (>7 joints): 1e-4 is recommended minimum
    // eps_s: Tiny regularization for scaling variable s
    //        Kept very small (1e-8) to ensure s maximization takes priority
    const double eps = 1e-4;     // Regularization weight on q̇ (Hessian) - increased for stability
    const double eps_s = 1e-8;   // Tiny regularization on s for numerical stability

    // Objective function: min(f^T x + 0.5 x^T H x) = min(0.5 ε ||q̇||² - s)
    // Since OSQP minimizes, using f(dof) = -1 means min(-s) = max(s) ✓
    // This ensures scaling variable s is maximized to achieve full commanded velocity when feasible

    // Hessian: diag(ε I_dof, eps_s)
    // Provides weak regularization on joint velocities while keeping s as primary variable
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_var, n_var);
    H.topLeftCorner(dof, dof) = eps * Eigen::MatrixXd::Identity(dof, dof);
    H(dof, dof) = eps_s;

    // Gradient: [0, ..., 0, -1] for min(-s) = max(s)
    Eigen::VectorXd f = Eigen::VectorXd::Zero(n_var);
    f(dof) = -1.0;  // Coefficient for s variable (negative to maximize)

    // Build constraint matrix A with clear segment organization:
    // Row indices:
    //   [0 : task_dim)           → Task constraint: J·q̇ - s·v = 0
    //   [task_dim : task_dim+dof) → Joint velocity box constraints: q̇_min ≤ q̇ ≤ q̇_max
    //   [task_dim+dof)            → Scaling variable constraint: 0 ≤ s ≤ 1
    // Variable indices: [q̇ (0:dof), s (dof:dof+1)]

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constr, n_var);

    // [1] Task equality constraint: J·q̇ - s·v = 0
    A.topLeftCorner(task_dim, dof) = J;
    A.topRightCorner(task_dim, 1) = -v_ee;

    // [2] Joint velocity box constraint matrix (identity for q̇ coefficients)
    A.block(task_dim, 0, dof, dof).setIdentity();

    // [3] Scaling variable constraint coefficient
    A(task_dim + dof, dof) = 1.0;

    // Setup bounds (initialize to ±infinity, then set specific constraints)
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_constr, -OsqpEigen::INFTY);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_constr, OsqpEigen::INFTY);

    // [1] Equality constraint: J·q̇ - s·v = 0 → set both bounds to 0
    lb.head(task_dim).setZero();
    ub.head(task_dim).setZero();

    // [2] Joint velocity bounds: q̇_min ≤ q̇ ≤ q̇_max
    lb.segment(task_dim, dof) = qd_min;
    ub.segment(task_dim, dof) = qd_max;

    // [3] Scaling variable bound: 0 ≤ s ≤ 1
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
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ QP setup failed", mapping.c_str());
        return false;
    }

    if (!solver.initSolver()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ QP init failed", mapping.c_str());
        return false;
    }

    auto exitflag = solver.solveProblem();
    if (exitflag != OsqpEigen::ErrorExitFlag::NoError) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ QP failed (flag=%d). Trying geometric scaling fallback...",
            mapping.c_str(), static_cast<int>(exitflag));

        // Fallback: geometric scaling of v_ee (direction preserved)
        // Try scaling velocity by 0.9^(iter+1) to make velocity constraint less aggressive
        const double scale = 0.9;
        bool solved = false;

        for (int iter = 0; iter < 10; ++iter) {
            // Explicit exponential scaling: v_scaled = v_ee * 0.9^(iter+1)
            Eigen::VectorXd v_scaled = v_ee * std::pow(scale, iter + 1);

            // Update A's top-right column: -v_scaled
            A.topRightCorner(task_dim, 1) = -v_scaled;
            Eigen::SparseMatrix<double> A2 = A.sparseView();

            if (!solver.updateLinearConstraintsMatrix(A2)) break;

            // Reset equality bounds (already 0, but be explicit)
            lb.head(task_dim).setZero();
            ub.head(task_dim).setZero();

            if (!solver.updateLowerBound(lb) || !solver.updateUpperBound(ub)) break;

            exitflag = solver.solveProblem();
            if (exitflag == OsqpEigen::ErrorExitFlag::NoError) {
                solved = true;
                RCLCPP_INFO(node_->get_logger(), "[%s] Fallback solved at iter=%d, scale=%.3f",
                    mapping.c_str(), iter, std::pow(scale, iter + 1));
                break;
            }
        }

        if (!solved) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Fallback scaling also failed", mapping.c_str());
            return false;
        }
    }

    Eigen::VectorXd solution = solver.getSolution();
    qd_solution = solution.head(dof);
    double s_opt = solution(dof);

    // ========== 后置方向一致性验证 ==========
    // Verify direction preservation and numerically stabilize the solution
    Eigen::VectorXd v_actual = J * qd_solution;
    double v_actual_norm = v_actual.norm();
    double dir_err = 0.0;

    if (v_actual_norm > 1e-9) {
        dir_err = (v_actual / v_actual_norm - v_direction).norm();

        // Safeguard: Near singularities, numerical errors can amplify direction errors.
        // If direction is acceptable but magnitude drifted, rescale to target velocity.
        // This ensures ||J·q̇|| ≈ s_opt * ||v_ee|| while preserving direction.
        double expected_velocity = s_opt * v_magnitude;
        if (expected_velocity > 1e-9 && v_actual_norm > 1e-9) {
            double velocity_scale = expected_velocity / v_actual_norm;
            // Apply scale back only if it's reasonably close (within 2x or 0.5x)
            // to avoid amplifying numerical instabilities
            if (velocity_scale > 0.5 && velocity_scale < 2.0) {
                qd_solution *= velocity_scale;
                v_actual *= velocity_scale;
                RCLCPP_DEBUG(node_->get_logger(),
                    "[%s] Velocity rescaled: %.3f → %.3f (direction preserved)",
                    mapping.c_str(), 1.0 / velocity_scale, 1.0);
            }
        }
    }

    RCLCPP_DEBUG(node_->get_logger(),
        "[%s] s=%.4f, ||Jq̇||=%.6f, dir_err=%.6f, feasibility=%.3f, cond=%.3g",
        mapping.c_str(), s_opt, v_actual_norm, dir_err, feasibility_ratio, cond);

    // Final safety check: if direction misalignment exceeds threshold, force stop
    // ========== 后置方向一致性验证（第3层防护）==========
    // 即使QP求解成功，仍需验证实际末端速度方向是否与期望相符
    //
    // 原因：
    //   1. QP求解的数值精度有限
    //   2. 关节约束与方向约束可能无法完全同时满足
    //   3. 在工作空间边界附近，约束极其紧张
    //
    // 检验方法：计算实际与期望方向间的夹角
    //   v_actual = J·q̇_solution       (实际末端速度)
    //   v_desired = v_ee               (期望末端速度)
    //
    //   cos(θ) = (v_actual · v_desired) / (||v_actual|| × ||v_desired||)
    //
    // 如果 cos(θ) < cos(5°) ≈ 0.99619：
    //   → 方向偏差 > 5°，认为"方向已失效"
    //   → 发送零速度停止，而不是执行"错误方向"的运动
    //
    // 为什么选5°？
    //   - 人类感知：< 5° 的偏差很难被察觉
    //   - 安全裕度：足以检测出实质的轨迹偏离
    //   - 工业标准：大多数应用使用 3~10° 范围

    if (!post_verify_direction(mapping, J, qd_solution, v_ee, ALLOWED_ANGLE_DEG)) {
        // Direction drifted too much (>5°), force stop to prevent corrupted motion
        qd_solution = Eigen::VectorXd::Zero(dof);
    }

    return true;
}


bool CartesianVelocityController::send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities) {

    if (!hardware_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware manager not initialized");
        return false;
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return false;
    }

    const std::string& interface = hardware_manager_->get_interface(mapping);
    const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);


    for (size_t i = 0; i < motor_ids.size(); ++i) {
        double vel_deg = joint_velocities[i] * 180.0 / M_PI;  // 转为度/秒
        hardware_driver->control_motor_in_velocity_mode(interface, motor_ids[i], vel_deg);
    }

    return true;
}

// ==================== 几何可行性检测（前置） ====================
void CartesianVelocityController::check_direction_feasibility(
    const Eigen::MatrixXd &J,
    const Eigen::VectorXd &v_ee,
    double &out_feasibility_ratio,
    double &out_residual_norm,
    double &out_alignment,
    double &out_cond) const
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

// ==================== 方向一致性验证（后置） ====================
bool CartesianVelocityController::post_verify_direction(
    const std::string& mapping,
    const Eigen::MatrixXd &J,
    const Eigen::VectorXd &qd_solution,
    const Eigen::VectorXd &v_ee,
    double allowed_angle_deg) const
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
        RCLCPP_WARN(node_->get_logger(),
            "[%s] ❗ Direction mismatch after QP: cos=%.6f < %.6f (angle=%.2f°). Stopping motion.",
            mapping.c_str(), cos_val, cos_thresh,
            std::acos(std::min(1.0, std::max(-1.0, cos_val))) * 180.0 / M_PI);
        return false;  // 方向偏离过大，不允许运动
    }

    return true;  // 方向一致，允许运动
}

bool CartesianVelocityController::check_workspace_boundary(
    const std::string& mapping,
    const std::vector<double>& joint_positions,
    const Eigen::VectorXd& qd_command,
    double dt)
{
    // 预测下一个关节位置
    Eigen::VectorXd q_current = Eigen::Map<const Eigen::VectorXd>(joint_positions.data(), joint_positions.size());
    Eigen::VectorXd q_next = q_current + qd_command * dt;

    std::vector<double> q_next_vec(q_next.data(), q_next.data() + q_next.size());

    // 检查预测的关节位置是否在关节限制范围内
    // 关键：只阻止进一步违反限制的运动，但允许离开限制的运动（恢复）
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    bool would_violate_limits = false;

    for (size_t i = 0; i < q_next_vec.size() && i < joint_names.size(); ++i) {
        JointLimits limits;
        hardware_manager_->get_joint_limits(joint_names[i], limits);

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
                RCLCPP_WARN(node_->get_logger(),
                    "[%s] ⚠️ Joint '%s' out of limits, motion blocked: curr=%.4f, cmd=%.4f (limits: [%.4f, %.4f])",
                    mapping.c_str(), joint_names[i].c_str(),
                    joint_positions[i], q_next_vec[i], limits.min_position, limits.max_position);
                would_violate_limits = true;
                break;
            }

            // 2. 当前在限制内，但命令会导致超出
            if (!current_exceeds_min && !current_exceeds_max &&
                (next_exceeds_min || next_exceeds_max)) {
                RCLCPP_WARN(node_->get_logger(),
                    "[%s] ⚠️ Joint '%s' would exceed limits: curr=%.4f, cmd=%.4f (limits: [%.4f, %.4f])",
                    mapping.c_str(), joint_names[i].c_str(),
                    joint_positions[i], q_next_vec[i], limits.min_position, limits.max_position);
                would_violate_limits = true;
                break;
            }

            // 允许的情况：当前已超出但命令要恢复（回到限制内）
            if ((current_exceeds_min && !next_exceeds_min) ||
                (current_exceeds_max && !next_exceeds_max)) {
                RCLCPP_DEBUG(node_->get_logger(),
                    "[%s] Joint '%s' recovering from limit violation: %.4f → %.4f",
                    mapping.c_str(), joint_names[i].c_str(),
                    joint_positions[i], q_next_vec[i]);
            }
        }
    }

    if (would_violate_limits) {
        RCLCPP_DEBUG(node_->get_logger(),
            "[%s] Joint limit violation detected. Motion blocked.", mapping.c_str());
        return false;  // 停止运动
    }

    return true;  // 允许运动
}

