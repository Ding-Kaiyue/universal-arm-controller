#include "cartesian_velocity_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/utils/velocity_qp_solver.hpp"
#include <stdexcept>
#include <algorithm>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'CartesianVelocity', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/cartesian_velocity_action/single_arm geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

CartesianVelocityController::CartesianVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<geometry_msgs::msg::TwistStamped>("CartesianVelocity", node),
      base_frame_("Link6")
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

    RCLCPP_INFO(node_->get_logger(),
        "[%s] ✓ CartesianVelocityController activated. Reference frame will be determined from each TwistStamped message.",
        mapping.c_str());
}


bool CartesianVelocityController::stop(const std::string& mapping) {
    // 停止处理消息
    is_active_ = false;

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

    // 坐标系处理：每条消息可以指定不同的参考坐标系
    Eigen::Vector3d v_linear_raw(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    Eigen::Vector3d v_angular_raw(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);

    Eigen::VectorXd v_ee(6);

    // 获取用户指定的参考坐标系，如果未指定则使用Link6
    std::string user_frame = msg->header.frame_id.empty() ? base_frame_ : msg->header.frame_id;

    try {
        rclcpp::Time now = node_->get_clock()->now();

        // 获取从用户指定的坐标系到base_link的变换
        geometry_msgs::msg::TransformStamped tf_user_to_base =
            tf_buffer_->lookupTransform(base_frame_, user_frame, now, std::chrono::milliseconds(100));

        // 从四元数提取旋转矩阵
        Eigen::Quaterniond q_user_to_base(
            tf_user_to_base.transform.rotation.w,
            tf_user_to_base.transform.rotation.x,
            tf_user_to_base.transform.rotation.y,
            tf_user_to_base.transform.rotation.z
        );
        Eigen::Matrix3d R_user_to_base = q_user_to_base.toRotationMatrix();

        // 将速度从用户坐标系变换到base_link坐标系
        Eigen::Vector3d v_linear_base = R_user_to_base * v_linear_raw;
        Eigen::Vector3d v_angular_base = R_user_to_base * v_angular_raw;

        v_ee << v_linear_base(0), v_linear_base(1), v_linear_base(2),
                v_angular_base(0), v_angular_base(1), v_angular_base(2);

        RCLCPP_DEBUG(node_->get_logger(),
            "[%s] Velocity transformed: %s[%.6f,%.6f,%.6f] → %s[%.6f,%.6f,%.6f]",
            active_mapping_.c_str(),
            user_frame.c_str(), v_linear_raw(0), v_linear_raw(1), v_linear_raw(2),
            base_frame_.c_str(), v_linear_base(0), v_linear_base(1), v_linear_base(2));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] ⚠️ TF lookup failed for frame '%s': %s. Stopping motion for safety.",
            active_mapping_.c_str(), user_frame.c_str(), ex.what());
        // 安全停止：发送零速度而不是使用原始速度
        auto joint_names_list = hardware_manager_->get_joint_names(active_mapping_);
        if (!joint_names_list.empty()) {
            std::vector<double> zero_velocities(joint_names_list.size(), 0.0);
            send_joint_velocities(active_mapping_, zero_velocities);
        }
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "[%s] Final velocity for IK: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        active_mapping_.c_str(), v_ee(0), v_ee(1), v_ee(2), v_ee(3), v_ee(4), v_ee(5));

    // 获取关节速度限制
    Eigen::VectorXd qd_min(J.cols()), qd_max(J.cols());
    for (int i = 0; i < J.cols(); ++i) {
        JointLimits limits;
        hardware_manager_->get_joint_limits(joint_names[i], limits);
        double vmax_abs = limits.has_velocity_limits ? limits.max_velocity : 1.0;
        qd_max(i) = vmax_abs;
        qd_min(i) = -vmax_abs;
    }

    // 使用 VelocityQPSolver 求解
    Eigen::VectorXd qd(J.cols());
    if (!arm_controller::utils::VelocityQPSolver::solve_velocity_qp(
            J, v_ee, qd, qd_min, qd_max, node_->get_logger())) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ Failed to solve velocity QP. Sending zero velocities to stop.", active_mapping_.c_str());
        // 发送零速度命令来停止机械臂
        auto joint_names_list = hardware_manager_->get_joint_names(active_mapping_);
        if (!joint_names_list.empty()) {
            std::vector<double> zero_velocities(joint_names_list.size(), 0.0);
            send_joint_velocities(active_mapping_, zero_velocities);
        }
        return;
    }

    // 检查工作空间边界 - 如果会超出工作空间，停止运动
    if (!arm_controller::utils::VelocityQPSolver::check_workspace_boundary(
            joint_positions, qd, joint_names, hardware_manager_)) {
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
        const double kd_velocity = 0.1;     // 速度模式：kd=0.1
        const double effort = 0.0;           // 力矩在纯速度模式下不使用
        const double position = 0.0;         // 位置在纯速度模式下不使用

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            const auto& joint_name = joint_names[i];
            uint32_t motor_id = motor_ids[i];
            double vel = joint_velocities[i] * 180.0 / M_PI;  // 转为度/秒

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
