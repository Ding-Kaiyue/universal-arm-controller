#include "cartesian_velocity_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include "arm_controller/utils/velocity_qp_solver.hpp"
#include <stdexcept>
#include <algorithm>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'CartesianVelocity', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/cartesian_velocity_action/single_arm geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

CartesianVelocityController::CartesianVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<geometry_msgs::msg::TwistStamped>("CartesianVelocity", node),
      base_frame_("base_link")
{
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化TF2缓冲和监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // 从配置文件读取基座坐标系
    node_->get_parameter("controllers.CartesianVelocity.base_frame", base_frame_);
    RCLCPP_INFO(node_->get_logger(), "[CartesianVelocity] Base frame: %s", base_frame_.c_str());

    // 初始化MoveIt服务
    initialize_moveit_service();

    // 启动IPC命令队列消费线程（早期启动以接收API发送的命令）
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&CartesianVelocityController::command_queue_consumer_thread, this);
        RCLCPP_INFO(node_->get_logger(), "✅ CartesianVelocity: IPC queue consumer thread started early");
    }
}


void CartesianVelocityController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] CartesianVelocity: not found in hardware configuration."
        );
    }

    // 调用基类 start() 设置 per-mapping 的 is_active_[mapping] = true
    VelocityControllerImpl::start(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] CartesianVelocityController activated", mapping.c_str());
}


bool CartesianVelocityController::stop(const std::string& mapping) {
    // 发送零速度命令停止机械臂
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    if (!joint_names.empty()) {
        std::vector<double> zero_velocities(joint_names.size(), 0.0);
        send_joint_velocities(mapping, zero_velocities);
    }

    // 调用基类 stop() 设置 per-mapping 的 is_active_[mapping] = false
    VelocityControllerImpl::stop(mapping);

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
        const double effort = 0.0;           // 力矩在纯速度模式下不使用
        const double position = 0.0;         // 位置在纯速度模式下不使用

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            const auto& joint_name = joint_names[i];
            uint32_t motor_id = motor_ids[i];
            double vel = joint_velocities[i] * 180.0 / M_PI;  // 转为度/秒

            int violation_dir = hardware_manager_->get_joint_violation_direction(joint_name);
            if (hardware_manager_->is_joint_emergency_stopped(joint_name) && ((violation_dir < 0 && vel < 0.0) || (violation_dir > 0 && vel > 0.0))) {
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

bool CartesianVelocityController::move(const std::string& mapping, const std::vector<double>& parameters) {
    // parameters 为 [vx, vy, vz, wx, wy, wz]
    if (parameters.size() != 6) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ CartesianVelocity: Expected 6 parameters (vx,vy,vz,wx,wy,wz), got %zu",
                   mapping.c_str(), parameters.size());
        return false;
    }

    auto joint_positions = hardware_manager_->get_current_joint_positions(mapping);
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    if (joint_positions.empty() || joint_names.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ No joint states or names available", mapping.c_str());
        return false;
    }

    // Get Jacobian
    if (moveit_adapters_.find(mapping) == moveit_adapters_.end() || !moveit_adapters_[mapping]) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ MoveItAdapter not initialized", mapping.c_str());
        return false;
    }

    Eigen::MatrixXd J = moveit_adapters_[mapping]->computeJacobian(joint_positions);
    if (J.rows() == 0 || J.cols() == 0) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ Invalid Jacobian matrix", mapping.c_str());
        return false;
    }

    // Convert parameters to task velocity
    Eigen::Vector3d v_linear_raw(parameters[0], parameters[1], parameters[2]);
    Eigen::Vector3d v_angular_raw(parameters[3], parameters[4], parameters[5]);

    Eigen::VectorXd v_ee(6);
    try {
        rclcpp::Time now = node_->get_clock()->now();
        geometry_msgs::msg::TransformStamped tf_world_to_base =
            tf_buffer_->lookupTransform("world", base_frame_, now, std::chrono::milliseconds(100));

        Eigen::Quaterniond q(
            tf_world_to_base.transform.rotation.w,
            tf_world_to_base.transform.rotation.x,
            tf_world_to_base.transform.rotation.y,
            tf_world_to_base.transform.rotation.z
        );
        Eigen::Matrix3d R_world_to_base = q.toRotationMatrix().transpose();

        Eigen::Vector3d v_linear_base = R_world_to_base * v_linear_raw;
        Eigen::Vector3d v_angular_base = R_world_to_base * v_angular_raw;

        v_ee << v_linear_base(0), v_linear_base(1), v_linear_base(2),
                v_angular_base(0), v_angular_base(1), v_angular_base(2);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ TF lookup failed: %s. Using raw velocity.",
                   mapping.c_str(), ex.what());
        v_ee << v_linear_raw(0), v_linear_raw(1), v_linear_raw(2),
                v_angular_raw(0), v_angular_raw(1), v_angular_raw(2);
    }

    // Build joint velocity limits
    Eigen::VectorXd qd_min(J.cols()), qd_max(J.cols());
    for (int i = 0; i < J.cols(); ++i) {
        JointLimits limits;
        hardware_manager_->get_joint_limits(joint_names[i], limits);
        double vmax = limits.has_velocity_limits ? limits.max_velocity : 1.0;
        qd_max(i) = vmax;
        qd_min(i) = -vmax;
    }

    // Solve velocity QP
    Eigen::VectorXd qd(J.cols());
    if (!arm_controller::utils::VelocityQPSolver::solve_velocity_qp(J, v_ee, qd, qd_min, qd_max, node_->get_logger())) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ Failed to solve velocity QP", mapping.c_str());
        return false;
    }

    // Check workspace boundary
    if (!arm_controller::utils::VelocityQPSolver::check_workspace_boundary(
            joint_positions, qd, joint_names, hardware_manager_)) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ❎ Workspace boundary violation", mapping.c_str());
        return false;
    }

    std::vector<double> velocities(qd.data(), qd.data() + qd.size());
    return send_joint_velocities(mapping, velocities);
}

void CartesianVelocityController::command_queue_consumer_thread() {
    arm_controller::TrajectoryCommandIPC cmd;
    std::map<std::string, arm_controller::ipc::ExecutionState> last_state;

    while (consumer_running_) {
        // 使用带过滤的 pop，只获取 CartesianVelocity 命令
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "CartesianVelocity")) {
            continue;
        }

        std::string mapping = cmd.get_mapping();
        std::string cmd_id = cmd.get_command_id();

        RCLCPP_INFO(node_->get_logger(), "[%s] CartesianVelocity: Received IPC command (ID: %s)",
                   mapping.c_str(), cmd_id.c_str());

        // 获取 per-mapping 的互斥锁
        std::lock_guard<std::mutex> execution_lock(arm_controller::CommandQueueIPC::getMappingExecutionMutex(mapping));

        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        try {
            // 更新为执行中
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::EXECUTING);
                last_state[mapping] = arm_controller::ipc::ExecutionState::EXECUTING;
            }

            auto params = cmd.get_parameters();
            bool success = move(mapping, params);

            if (success) {
                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ CartesianVelocity command executed successfully (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::SUCCESS);
                    last_state[mapping] = arm_controller::ipc::ExecutionState::SUCCESS;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ CartesianVelocity command execution failed (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                    last_state[mapping] = arm_controller::ipc::ExecutionState::FAILED;
                }
            }

            // 延迟后恢复到 IDLE
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception in CartesianVelocity command execution: %s",
                        mapping.c_str(), e.what());
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                last_state[mapping] = arm_controller::ipc::ExecutionState::FAILED;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);
            }
        }
    }
}
