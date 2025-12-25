#include "cartesian_velocity_controller.hpp"
#include "controller_interface.hpp"
#include <stdexcept>
#include <algorithm>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'CartesianVelocity', mapping: 'single_arm'}"
// ros2 topic pub /controller_api/cartesian_velocity_action/single_arm geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"

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

    // 保存当前激活的mapping
    active_mapping_ = mapping;
    is_active_ = true;

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    // ✅ 创建 10ms 控制定时器（Jog 实时循环）
    control_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CartesianVelocityController::control_loop, this));

    RCLCPP_INFO(node_->get_logger(),
        "[%s] ✓ CartesianVelocityController activated. Reference frame will be determined from each TwistStamped message.",
        mapping.c_str());
    RCLCPP_INFO(node_->get_logger(),
        "[%s] ✓ 10ms control loop started (Cartesian Jog mode)",
        mapping.c_str());
}


bool CartesianVelocityController::stop(const std::string& mapping) {
    is_active_ = false;

    // ✅ 销毁 10ms 控制定时器
    if (control_timer_) {
        control_timer_.reset();
        RCLCPP_INFO(node_->get_logger(), "[%s] Control loop stopped", mapping.c_str());
    }

    // 发送零速度命令停止机械臂
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    if (!joint_names.empty()) {
        std::vector<double> zero_velocities(joint_names.size(), 0.0);
        send_joint_velocities(mapping, zero_velocities);
    }

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] CartesianVelocityController deactivated", mapping.c_str());
    return true;
}

void CartesianVelocityController::initialize_moveit_service() {
    try {
        // 获取所有mapping
        auto all_mappings = hardware_manager_->get_all_mappings();
        if (all_mappings.empty()) {
            RCLCPP_WARN(node_->get_logger(), "❎ CartesianVelocity: No mappings configured");
            return;
        }

        // 为每个mapping初始化MoveIt服务
        for (const auto& mapping : all_mappings) {
            std::string planning_group = hardware_manager_->get_planning_group(mapping);

            if (planning_group.empty()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ❎ CartesianVelocity: No planning group configured", mapping.c_str());
                continue;
            }

            try {
                auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, planning_group);

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception: %s", mapping.c_str(), e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to initialize MoveIt services: %s", e.what());
    }
}

void CartesianVelocityController::velocity_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    // ✅ Twist latch 模式：仅缓存最新命令
    // 实际的 IK / 关节速度计算在 control_loop 中以 10ms 频率进行

    if (!node_) return;

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_twist_ = *msg;
        // ✅ 使用 steady_clock（单调时间）确保工业级时间源一致
        last_twist_time_ = steady_clock_.now();
    }

    RCLCPP_INFO(node_->get_logger(),
        "[%s] ✓ Twist received: linear[%.3f, %.3f, %.3f] angular[%.3f, %.3f, %.3f] frame='%s'",
        active_mapping_.c_str(),
        msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
        msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z,
        msg->header.frame_id.c_str());
}

void CartesianVelocityController::control_loop()
{
    /* ========================================
     * 10ms 实时控制循环
     * 这是睿尔曼风格的"真正的 Jog 控制"
     * 关键特性：
     * - 每 10ms 重新计算 Jacobian（机器人动态更新）
     * - 命令 latch（缓存最新 Twist）
     * - 100ms 超时保护
     * ======================================== */

    if (!is_active_ || active_mapping_.empty()) {
        return;
    }

    // 获取最新的 Twist 命令（带超时检查）
    geometry_msgs::msg::TwistStamped cmd;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        // ✅ 使用 steady_clock 进行超时检查（单调时间，不受 use_sim_time 影响）
        if ((steady_clock_.now() - last_twist_time_).seconds() > 0.1) {
            // 命令超时 → 紧急停止
            auto joint_names = hardware_manager_->get_joint_names(active_mapping_);
            std::vector<double> zero(joint_names.size(), 0.0);
            send_joint_velocities(active_mapping_, zero);
            return;
        }
        cmd = last_twist_;
    }

    /* ===============================
     * 1. Joint state (每周期读取)
     * =============================== */

    auto joint_positions = hardware_manager_->get_current_joint_positions(active_mapping_);
    auto joint_names     = hardware_manager_->get_joint_names(active_mapping_);

    if (joint_positions.empty() || joint_names.empty()) {
        return;
    }

    auto it = moveit_adapters_.find(active_mapping_);
    if (it == moveit_adapters_.end() || !it->second) {
        return;
    }

    /* ===============================
     * 2. Jacobian (BASE / WORLD) - 每周期重新计算 ✨ 关键
     * =============================== */

    Eigen::MatrixXd J =
        it->second->computeJacobian(joint_positions);

    if (J.rows() == 0 || J.cols() == 0 || J.hasNaN()) {
        std::vector<double> zero(joint_names.size(), 0.0);
        send_joint_velocities(active_mapping_, zero);
        return;
    }

    /* ===============================
     * 3. Cartesian velocity (STRICT)
     * =============================== */

    Eigen::Vector3d v_linear(
        cmd.twist.linear.x,
        cmd.twist.linear.y,
        cmd.twist.linear.z);

    Eigen::Vector3d v_angular(
        cmd.twist.angular.x,
        cmd.twist.angular.y,
        cmd.twist.angular.z);

    // 世界 / 基坐标系速度（默认）
    std::string user_frame =
        cmd.header.frame_id.empty() ? base_frame_ : cmd.header.frame_id;

    if (user_frame != base_frame_) {
        try {
            // ✅ 使用 tf2::TimePointZero 获取最新可用的变换（TF 官方推荐）
            auto tf =
                tf_buffer_->lookupTransform(
                    base_frame_, user_frame,
                    tf2::TimePointZero,
                    std::chrono::milliseconds(50));

            Eigen::Quaterniond q(
                tf.transform.rotation.w,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z);

            Eigen::Matrix3d R = q.toRotationMatrix();
            v_linear  = R * v_linear;
            v_angular = R * v_angular;

        } catch (...) {
            std::vector<double> zero(joint_names.size(), 0.0);
            send_joint_velocities(active_mapping_, zero);
            return;
        }
    }

    /* ========================================
     * ✅ 完整 6D 笛卡尔速度控制
     * 支持位置和姿态（位置 + 旋转）
     * ======================================== */

    // 使用完整 6D Jacobian（6×n）：3个线速度 + 3个角速度
    Eigen::MatrixXd J_task = J;
    Eigen::VectorXd v_task(6);
    v_task << v_linear(0), v_linear(1), v_linear(2),
              v_angular(0), v_angular(1), v_angular(2);

    RCLCPP_INFO(node_->get_logger(),
        "[%s] Target velocity - linear[%.3f, %.3f, %.3f] angular[%.3f, %.3f, %.3f]",
        active_mapping_.c_str(),
        v_linear(0), v_linear(1), v_linear(2),
        v_angular(0), v_angular(1), v_angular(2));

    if (v_task.norm() < 1e-8) {
        std::vector<double> zero(joint_names.size(), 0.0);
        send_joint_velocities(active_mapping_, zero);
        return;
    }

    /* ===============================
     * 4. Singularity Metric & Scaling
     * =============================== */

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_task);
    double sigma_min = svd.singularValues().minCoeff();

    // ✅ 工业策略：奇异区缩速，不拒绝
    // σ 小 → scale ↓，方向保持
    double scale = 1.0;
    if (sigma_min < 0.05) {
        // 平滑缩速函数
        if (sigma_min <= 0.01) {
            scale = 0.0;  // 极度奇异 → 停止
        } else {
            // σ ∈ [0.01, 0.05) 线性缩速
            scale = (sigma_min - 0.01) / (0.05 - 0.01);
        }
    }

    v_task *= scale;  // 应用缩速

    /* ===============================
     * 5. Joint limits & limits info
     * =============================== */

    const int dof = J.cols();

    Eigen::VectorXd q_current =
        Eigen::Map<Eigen::VectorXd>(
            joint_positions.data(), joint_positions.size());

    Eigen::VectorXd qd_max(dof);
    Eigen::VectorXd q_min_pos(dof), q_max_pos(dof);

    for (int i = 0; i < dof; ++i) {
        JointLimits limits;
        hardware_manager_->get_joint_limits(joint_names[i], limits);

        double vmax = limits.has_velocity_limits ? limits.max_velocity : 1.0;
        qd_max(i) = vmax;

        q_min_pos(i) = limits.min_position;
        q_max_pos(i) = limits.max_position;
    }

    /* ===============================
     * 6. 6D 笛卡尔速度求解（位置 + 姿态）
     * =============================== */

    Eigen::VectorXd qd(dof);

    // 调用 solver（传入完整 6D Jacobian 和 6D 任务速度）
    bool ok = solver_.solve(
        J_task,
        v_task,
        q_current,
        q_min_pos,
        q_max_pos,
        qd_max,
        qd,
        node_->get_logger());

    if (!ok) {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] Jog velocity unreachable",
            active_mapping_.c_str());

        std::vector<double> zero(joint_names.size(), 0.0);
        send_joint_velocities(active_mapping_, zero);
        return;
    }

    /* ========================================
     * 7. ✅ 工业级方向一致性检验（6D）
     * 检查方向，不检查模长
     * ======================================== */

    Eigen::VectorXd v_reconstructed = J_task * qd;

    // 检查是否接近零
    if (v_reconstructed.norm() < 1e-6) {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] Velocity reconstruction near zero",
            active_mapping_.c_str());
        std::vector<double> zero(joint_names.size(), 0.0);
        send_joint_velocities(active_mapping_, zero);
        return;
    }

    // ✅ 方向一致性检验（6D 速度空间）
    double cos_angle = v_reconstructed.normalized().dot(v_task.normalized());
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);

    // ~11° 容限
    if (cos_angle < 0.98) {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] Direction error: %.1f° > 11°, rejecting",
            active_mapping_.c_str(),
            std::acos(cos_angle) * 180.0 / M_PI);

        std::vector<double> zero(joint_names.size(), 0.0);
        send_joint_velocities(active_mapping_, zero);
        return;
    }

    /* ===============================
     * 7. Send joint velocity
     * =============================== */

    std::vector<double> velocities(qd.data(), qd.data() + qd.size());
    send_joint_velocities(active_mapping_, velocities);
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

    try {
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);
        const std::vector<std::string>& joint_names = hardware_manager_->get_joint_names(mapping);

        // MIT模式速度控制参数
        const double kp_velocity = 0.0;      // 速度模式：kp=0.0
        const double kd_velocity = 0.01;     // 速度模式：kd=0.01

        const double position = 0.0;         // 位置在纯速度模式下不使用

        // 获取当前关节位置对应的重力矩
        std::vector<double> gravity_torques = hardware_manager_->compute_gravity_torques(mapping);

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            const auto& joint_name = joint_names[i];
            uint32_t motor_id = motor_ids[i];
            double vel = joint_velocities[i] * 180.0 / M_PI;  // 转为度/秒
            double effort = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;  // 重力补偿力矩

            if (vel != 0.0) {
                RCLCPP_INFO(node_->get_logger(),
                    "[%s] Motor %u (joint %s): vel_rad=%.6f, vel_deg=%.6f",
                    mapping.c_str(), motor_id, joint_name.c_str(), joint_velocities[i], vel);
            }

            int violation_dir = hardware_manager_->get_joint_violation_direction(joint_name);
            if (hardware_manager_->is_joint_emergency_stopped(joint_name) &&
                ((violation_dir < 0 && vel < 0.0) || (violation_dir > 0 && vel > 0.0))) {
                // 不允许继续违规，发送零速度
                hardware_driver->control_motor_in_mit_mode(interface, motor_id, position, 0.0, effort, kp_velocity, kd_velocity);
                auto clock = node_->get_clock();
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *clock,
                    2000,
                    "[%s] Joint '%s' emergency stopped (dir=%d), unsafe velocity %.3f -> skipping.",
                    mapping.c_str(), joint_name.c_str(), violation_dir, vel);
            } else {
                hardware_driver->control_motor_in_mit_mode(interface, motor_id, position, vel, effort, kp_velocity, kd_velocity);
            }
        }

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to send joint velocities: %s", mapping.c_str(), e.what());
        return false;
    }
}