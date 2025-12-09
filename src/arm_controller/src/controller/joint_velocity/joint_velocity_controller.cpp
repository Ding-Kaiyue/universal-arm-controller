#include "joint_velocity_controller.hpp"
#include "controller_interface.hpp"
#include "hardware/hardware_manager.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <stdexcept>
#include <algorithm>
#include <controller_interfaces/srv/work_mode.hpp>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'JointVelocity', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/joint_velocity_action/single_arm sensor_msgs/msg/JointState "{velocity: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"

JointVelocityController::JointVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<sensor_msgs::msg::JointState>("JointVelocity", node)
{
    // 获取HardwareManager实例
    hardware_manager_ = HardwareManager::getInstance();

    // 注意：话题订阅在 init_subscriptions() 中创建，当 controller 被激活时调用
    RCLCPP_INFO(node_->get_logger(), "JointVelocityController initialized");

    // 启动IPC命令队列消费线程（早期启动以接收API发送的命令）
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&JointVelocityController::command_queue_consumer_thread, this);
        RCLCPP_INFO(node_->get_logger(), "✅ JointVelocity: IPC queue consumer thread started early");
    }
}


void JointVelocityController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] JointVelocity: not found in hardware configuration."
        );
    }
    // 调用基类 start() 设置 per-mapping 的 is_active_[mapping] = true
    VelocityControllerImpl::start(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] JointVelocityController activated", mapping.c_str());
}

bool JointVelocityController::stop(const std::string& mapping) {
    // 调用基类 stop() 设置 per-mapping 的 is_active_[mapping] = false
    VelocityControllerImpl::stop(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] JointVelocityController deactivated", mapping.c_str());
    return true;  // 需要钩子状态来安全停止
    
}

bool JointVelocityController::move(const std::string& mapping, const std::vector<double>& parameters) {
    // 获取期望的关节数
    size_t expected_joint_count = hardware_manager_->get_joint_count(mapping);

    // 处理参数长度：自动填充或裁短
    std::vector<double> joint_velocities = parameters;
    if (joint_velocities.size() < expected_joint_count) {
        // 用0填充不足的部分
        joint_velocities.resize(expected_joint_count, 0.0);
        RCLCPP_WARN(node_->get_logger(), "[%s] JointVelocity: Parameters padded to %zu joints",
                   mapping.c_str(), expected_joint_count);
    } else if (joint_velocities.size() > expected_joint_count) {
        // 裁短多余的部分
        joint_velocities.resize(expected_joint_count);
        RCLCPP_WARN(node_->get_logger(), "[%s] JointVelocity: Parameters truncated to %zu joints",
                   mapping.c_str(), expected_joint_count);
    }

    // 调用原有的 send_joint_velocities
    return send_joint_velocities(mapping, joint_velocities);
}

bool JointVelocityController::send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities) {
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

void JointVelocityController::command_queue_consumer_thread() {
    arm_controller::TrajectoryCommandIPC cmd;
    std::map<std::string, std::string> current_mode;
    std::map<std::string, arm_controller::ipc::ExecutionState> last_state;  // Track last execution state per mapping

    while (consumer_running_) {
        // 使用带过滤的 pop，只获取 JointVelocity 命令
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "JointVelocity")) {
            continue;
        }

        std::string mode = cmd.get_mode();
        std::string mapping = cmd.get_mapping();
        std::string cmd_id = cmd.get_command_id();

        RCLCPP_INFO(node_->get_logger(), "[%s] JointVelocity: Received IPC command (ID: %s)",
                   mapping.c_str(), cmd_id.c_str());

        // 获取 per-mapping 的互斥锁，确保同一手臂的命令串行执行
        std::lock_guard<std::mutex> execution_lock(arm_controller::CommandQueueIPC::getMappingExecutionMutex(mapping));

        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        try {
            // 获取状态管理器并更新为执行中
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::EXECUTING);
                last_state[mapping] = arm_controller::ipc::ExecutionState::EXECUTING;
            }

            auto params = cmd.get_parameters();
            bool success = move(mapping, params);

            if (success) {
                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ JointVelocity command executed successfully (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::SUCCESS);
                    last_state[mapping] = arm_controller::ipc::ExecutionState::SUCCESS;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ JointVelocity command execution failed (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                    last_state[mapping] = arm_controller::ipc::ExecutionState::FAILED;
                }
            }

            // 延迟后恢复到 IDLE，给下一条命令足够的时间看到最终状态
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception in JointVelocity command execution: %s",
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