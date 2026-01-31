#include "movej_controller.hpp"
#include "controller_interface.hpp"
#include "hardware/hardware_manager.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <controller_interfaces/srv/work_mode.hpp>
#include <set>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'MoveJ', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/movej_action/single_arm sensor_msgs/msg/JointState "{position: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"
// ros2 topic pub --once /trajectory_control controller_interfaces/msg/TrajectoryControl "{mapping: 'single_arm', action: 'Cancel'}"

MoveJController::MoveJController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<sensor_msgs::msg::JointState>("MoveJ", node)
{
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化轨迹插值器
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

    // ✅ 加载插值器配置
    load_interpolator_config(*trajectory_interpolator_);

    // 初始化轨迹规划服务
    initialize_planning_services();

    // 启动IPC命令队列消费线程（早期启动以接收API发送的命令）
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&MoveJController::command_queue_consumer_thread, this);
        RCLCPP_INFO(node_->get_logger(), "✅ MoveJ: IPC queue consumer thread started early");
    }
}

void MoveJController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] MoveJ: not found in hardware configuration."
        );
    }

    // 调用基类 start() 设置 per-mapping 的 is_active_[mapping] = true
    TrajectoryControllerImpl::start(mapping);

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveJController activated", mapping.c_str());

}

bool MoveJController::stop(const std::string& mapping) {
    // 调用基类 stop() 设置 per-mapping 的 is_active_[mapping] = false
    TrajectoryControllerImpl::stop(mapping);

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveJController deactivated", mapping.c_str());
    return true;
}

void MoveJController::trajectory_callback(const std::string& mapping, const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 使用传入的 mapping 进行规划和执行
    plan_and_execute(mapping, msg);
}

void MoveJController::initialize_planning_services() {
    try {
        // 获取所有mapping
        auto all_mappings = hardware_manager_->get_all_mappings();
        if (all_mappings.empty()) {
            RCLCPP_WARN(node_->get_logger(), "❎ MoveJ: No mappings configured");
            return;
        }

        // 为每个mapping初始化规划服务
        for (const auto& mapping : all_mappings) {
            std::string planning_group = hardware_manager_->get_planning_group(mapping);

            if (planning_group.empty()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ❎ MoveJ: No planning group configured, skipping...", mapping.c_str());
                continue;
            }

            try {
                // 创建 MoveItAdapter
                auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, planning_group, "movej");

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;

                // 创建 TracIKAdapter（MoveJ 不使用，但需要为 MotionPlanningService 创建）
                auto tracik_adapter = std::make_shared<trajectory_planning::infrastructure::integration::TracIKAdapter>(
                    node_, planning_group);

                if (!tracik_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Failed to create TracIKAdapter", mapping.c_str());
                    continue;
                }

                // 创建轨迹规划服务
                auto motion_planning_service = std::make_shared<trajectory_planning::application::services::MotionPlanningService>(
                    moveit_adapter,
                    tracik_adapter,
                    node_,
                    hardware_manager_->get_robot_type(mapping));

                if (!motion_planning_service) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Failed to create MotionPlanningService", mapping.c_str());
                    continue;
                }

                // 注册MoveJ策略
                motion_planning_service->registerMoveJStrategy();   // 用到了TrajectoryPlanning的MoveJ策略

                motion_planning_services_[mapping] = motion_planning_service;
                mapping_to_planning_group_[mapping] = planning_group;

                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveJ: Planning service initialized (planning group: '%s')",
                           mapping.c_str(), planning_group.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "❎ MoveJ: Failed to initialize planning services: %s", e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ MoveJ: Failed to initialize planning services: %s", e.what());
    }
}

void MoveJController::plan_and_execute(const std::string& mapping, const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 查找mapping
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() ||
        !motion_planning_services_[mapping]) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ❎ MoveJ: Planning service not found. This strategy must be registered first.", mapping.c_str());
        return;
    }

    // 检查目标关节状态
    if (msg->position.size() != hardware_manager_->get_joint_count(mapping)) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Invalid joint state size", mapping.c_str());
        return;
    }

    // 在规划前同步 MoveIt 状态到当前机械臂位置，确保从正确的起始位置规划
    if (moveit_adapters_.find(mapping) != moveit_adapters_.end() && moveit_adapters_[mapping]) {
        auto current_positions = hardware_manager_->get_current_joint_positions(mapping);
        if (!current_positions.empty()) {
            moveit_adapters_[mapping]->setStartState(current_positions);
            RCLCPP_DEBUG(node_->get_logger(), "[%s] MoveJ: Synced MoveIt state to current position before planning", mapping.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] MoveJ: Failed to get current positions for pre-planning sync", mapping.c_str());
        }
    }

    // 进行轨迹规划
    auto planning_result = motion_planning_services_[mapping]->planJointMotion(*msg);
    if (!planning_result.success) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Planning failed", mapping.c_str());
        last_planning_success_[mapping] = false;
        return;
    }

    last_planning_success_[mapping] = true;

    // 检查轨迹点数
    if (planning_result.trajectory.size() < 3) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveJ: Already at target position, no movement needed",
                   mapping.c_str());
        return;
    }

    // 转换轨迹格式
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    trajectory_interpolator::Trajectory interpolator_trajectory =
        arm_controller::utils::TrajectoryConverter::convertPlanningToInterpolator(
            planning_result.trajectory, joint_names);

    // 分析轨迹动力学参数
    auto dynamics = arm_controller::utils::TrajectoryConverter::analyzeTrajectoryDynamics(planning_result.trajectory);
    auto safe_params = arm_controller::utils::TrajectoryConverter::calculateSafeInterpolationParams(dynamics);

    // 插值轨迹
    trajectory_interpolator::Trajectory final_trajectory = interpolate_trajectory(
        interpolator_trajectory,
        safe_params.max_velocity,
        safe_params.max_acceleration,
        safe_params.max_jerk,
        mapping
    );

    // 执行轨迹
    execute_trajectory(final_trajectory, mapping);
}

bool MoveJController::execute(const std::string& mapping, const std::vector<double>& parameters) {
    // 检查 mapping 和规划服务
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() ||
        !motion_planning_services_[mapping]) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Planning service not found.", mapping.c_str());
        return false;
    }

    // 参数检查
    if (parameters.size() != hardware_manager_->get_joint_count(mapping)) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Invalid parameters size for execute()", mapping.c_str());
        return false;
    }

    // 构造 JointState 消息
    auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state_msg->position = parameters;

    // 初始化规划状态为未尝试
    last_planning_success_[mapping] = true;

    // 调用原有的 plan_and_execute，它会更新 last_planning_success_
    plan_and_execute(mapping, joint_state_msg);

    // 根据规划结果返回
    return last_planning_success_[mapping];
}

trajectory_interpolator::Trajectory MoveJController::interpolate_trajectory(
    const trajectory_interpolator::Trajectory& interpolator_trajectory,
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    const std::string& mapping) {

    if (!trajectory_interpolator_) {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] ⚠️ MoveJ: No interpolator available, using original trajectory", mapping.c_str());
        return interpolator_trajectory;
    }

    try {
        // 使用提供的动力学参数加载轨迹
        if (trajectory_interpolator_->loadTrajectoryWithDynamicConfig(interpolator_trajectory,
                                                                     max_velocity,
                                                                     max_acceleration,
                                                                     max_jerk)) {
            return trajectory_interpolator_->interpolate();
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ MoveJ: Interpolation failed, using original trajectory", mapping.c_str());
            return interpolator_trajectory;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ MoveJ: Interpolation exception: %s, using original trajectory",
                   mapping.c_str(), e.what());
        return interpolator_trajectory;
    }
}

void MoveJController::execute_trajectory(
    const trajectory_interpolator::Trajectory& trajectory,
    const std::string& mapping) {
    try {
        // 使用转换工具将轨迹转换为硬件驱动格式
        Trajectory hw_trajectory = arm_controller::utils::TrajectoryConverter::convertInterpolatorToHardwareDriver(trajectory);

        // 使用异步执行轨迹（不阻塞，并保存 execution_id 以支持暂停/恢复/取消）
        std::string execution_id = hardware_manager_->execute_trajectory_async(mapping, hw_trajectory, true);
        if (execution_id.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Failed to execute trajectory",
                        mapping.c_str());
            last_planning_success_[mapping] = false;
            return;
        }

        bool wait_success = hardware_manager_->wait_for_trajectory_completion(mapping);
        last_planning_success_[mapping] = wait_success;

        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveJ: Trajectory execution started (ID: %s)",
                   mapping.c_str(), execution_id.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Exception during trajectory execution: %s",
                    mapping.c_str(), e.what());
        last_planning_success_[mapping] = false;
        return;
    }
}

void MoveJController::command_queue_consumer_thread() {
    arm_controller::TrajectoryCommandIPC cmd;
    std::map<std::string, std::string> current_mode;
    std::map<std::string, arm_controller::ipc::ExecutionState> last_state;  // Track last execution state per mapping

    while (consumer_running_) {
        // 使用带过滤的 pop，只获取 MoveJ 命令
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "MoveJ")) {
            continue;
        }

        std::string mode = cmd.get_mode();
        std::string mapping = cmd.get_mapping();
        std::string cmd_id = cmd.get_command_id();

        RCLCPP_INFO(node_->get_logger(), "[%s] MoveJ: Received IPC command (ID: %s)",
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
            bool success = execute(mapping, params);

            if (success) {
                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveJ command executed successfully (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::SUCCESS);
                    last_state[mapping] = arm_controller::ipc::ExecutionState::SUCCESS;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ command execution failed (ID: %s)",
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
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception in MoveJ command execution: %s",
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