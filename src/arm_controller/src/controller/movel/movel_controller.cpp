#include "movel_controller.hpp"
#include "controller_interface.hpp"
#include "hardware/hardware_manager.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <controller_interfaces/srv/work_mode.hpp>
#include <set>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'MoveL', mapping: 'left_arm'}"
// ros2 topic pub --once /controller_api/movel_action/left_arm geometry_msgs/msg/Pose "{position: {x: 0.19, y: 0.0, z: 0.63}, orientation: {x: -0.4546, y: 0.4546, z: -0.5417, w: 0.5417}}"

MoveLController::MoveLController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<geometry_msgs::msg::Pose>("MoveL", node)
{
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化轨迹插值器
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

    // ✅ 加载插值器配置
    load_interpolator_config(*trajectory_interpolator_);

    // 初始化轨迹规划服务
    initialize_planning_services();

    // 启动后台规划工作线程（暂时禁用以调试 IPC 问题）
    if (!planning_worker_running_) {
        planning_worker_running_ = true;
        planning_worker_ = std::make_unique<std::thread>(&MoveLController::planning_worker_thread, this);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 启动IPC命令队列消费线程（早期启动以接收API发送的命令）
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&MoveLController::command_queue_consumer_thread, this);
    }
}

void MoveLController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] MoveL: not found in hardware configuration."
        );
    }

    // 调用基类 start() 设置 per-mapping 的 active_mappings_[mapping] = true
    TrajectoryControllerImpl::start(mapping);

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] MoveLController activated", mapping.c_str());

}

bool MoveLController::stop(const std::string& mapping) {
    // 调用基类 stop() 设置 per-mapping 的 active_mappings_[mapping] = false
    TrajectoryControllerImpl::stop(mapping);

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveLController deactivated", mapping.c_str());
    return true;
}

void MoveLController::trajectory_callback(const std::string& mapping, const geometry_msgs::msg::Pose::SharedPtr msg) {
    // 规划和执行在后台工作线程中进行
    {
        std::lock_guard<std::mutex> lock(planning_queue_mutex_);
        planning_queue_.push({mapping, msg});
    }
    planning_queue_cv_.notify_one();
}

void MoveLController::initialize_planning_services() {
    try {
        // 获取所有mapping
        auto all_mappings = hardware_manager_->get_all_mappings();
        if (all_mappings.empty()) {
            RCLCPP_WARN(node_->get_logger(), "❎ MoveL: No mappings configured");
            return;
        }

        // 为每个mapping初始化规划服务
        for (const auto& mapping : all_mappings) {
            std::string planning_group = hardware_manager_->get_planning_group(mapping);

            if (planning_group.empty()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ❎ MoveL: No planning group configured, skipping...", mapping.c_str());
                continue;
            }

            try {
                // 创建 MoveItAdapter
                auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, planning_group, "movel");

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;

                // 创建 TracIKAdapter
                auto tracik_adapter = std::make_shared<trajectory_planning::infrastructure::integration::TracIKAdapter>(
                    node_, planning_group);

                if (!tracik_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Failed to create TracIKAdapter", mapping.c_str());
                    continue;
                }

                tracik_adapters_[mapping] = tracik_adapter;

                // 创建轨迹规划服务
                auto motion_planning_service = std::make_shared<trajectory_planning::application::services::MotionPlanningService>(
                    moveit_adapter, tracik_adapter, node_, hardware_manager_->get_robot_type(mapping));

                if (!motion_planning_service) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Failed to create MotionPlanningService", mapping.c_str());
                    continue;
                }

                // 注册MoveL策略
                motion_planning_service->registerMoveLStrategy();   // 用到了TrajectoryPlanning的MoveL策略

                motion_planning_services_[mapping] = motion_planning_service;
                mapping_to_planning_group_[mapping] = planning_group;

            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "❎ MoveL: Failed to initialize planning services: %s", e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ MoveL: Failed to initialize planning services: %s", e.what());
    }
}

void MoveLController::plan_and_execute(const std::string& mapping, const geometry_msgs::msg::Pose::SharedPtr msg) {
    // 查找mapping
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() ||
        !motion_planning_services_[mapping]) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ❎ MoveL: Planning service not found. This strategy must be registered first.", mapping.c_str());
        return;
    }

    // 进行轨迹规划
    auto planning_result = motion_planning_services_[mapping]->planLinearMotion(*msg);
    if (!planning_result.success) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Planning failed", mapping.c_str());
        last_planning_success_[mapping] = false;
        return;
    }

    last_planning_success_[mapping] = true;

    RCLCPP_INFO(node_->get_logger(), "[%s] [MoveL] Planned %zu pts, duration=%.2f s",
                mapping.c_str(), planning_result.trajectory.size(),
                planning_result.trajectory.total_duration().seconds());

    // 检查轨迹点数
    if (planning_result.trajectory.size() < 3) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveL: Already at target position, no movement needed",
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

bool MoveLController::execute(const std::string& mapping, const std::vector<double>& parameters) {
    // 检查mapping和规划服务
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() ||
        !motion_planning_services_[mapping]) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Planning service not found", mapping.c_str());
        return false;
    }

    // 参数检查
    if (parameters.size() != 7) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Expected 7 parameters (Pose), got %zu", mapping.c_str(), parameters.size());
        return false;
    }

    // 构建 Pose 消息
    auto pose_state = std::make_shared<geometry_msgs::msg::Pose>();
    pose_state->position.x = parameters[0];
    pose_state->position.y = parameters[1];
    pose_state->position.z = parameters[2];
    pose_state->orientation.x = parameters[3];
    pose_state->orientation.y = parameters[4];
    pose_state->orientation.z = parameters[5];
    pose_state->orientation.w = parameters[6];

    // 初始化规划状态为失败（未尝试），只有规划成功才会设为true
    last_planning_success_[mapping] = false;

    // 调用原有的 plan_and_execute，它会更新 last_planning_success_
    plan_and_execute(mapping, pose_state);

    // 根据规划结果返回
    return last_planning_success_[mapping];
}

trajectory_interpolator::Trajectory MoveLController::interpolate_trajectory(
    const trajectory_interpolator::Trajectory& interpolator_trajectory,
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    const std::string& mapping) {

    if (!trajectory_interpolator_) {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] ⚠️ MoveL: No interpolator available, using original trajectory", mapping.c_str());
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
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ MoveL: Interpolation failed, using original trajectory", mapping.c_str());
            return interpolator_trajectory;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ MoveL: Interpolation exception: %s, using original trajectory",
                   mapping.c_str(), e.what());
        return interpolator_trajectory;
    }
}

void MoveLController::execute_trajectory(
    const trajectory_interpolator::Trajectory& trajectory,
    const std::string& mapping) {
    try {
        // 使用转换工具将轨迹转换为硬件驱动格式
        Trajectory hw_trajectory = arm_controller::utils::TrajectoryConverter::convertInterpolatorToHardwareDriver(trajectory);

        // 使用异步执行轨迹（不阻塞，并保存 execution_id 以支持暂停/恢复/取消）
        std::string execution_id = hardware_manager_->execute_trajectory_async(mapping, hw_trajectory, true);
        if (execution_id.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Failed to execute trajectory on mapping: %s",
                        mapping.c_str(), mapping.c_str());
            last_planning_success_[mapping] = false;
            return;
        }

        bool wait_success = hardware_manager_->wait_for_trajectory_completion(mapping);
        last_planning_success_[mapping] = wait_success;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Exception during trajectory execution: %s",
                    mapping.c_str(), e.what());
        last_planning_success_[mapping] = false;
        return;
    }
}

void MoveLController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;
    std::map<std::string, std::string> current_mode;
    std::map<std::string, arm_controller::ipc::ExecutionState> last_state;  // Track last execution state per mapping

    while (consumer_running_) {
        // 使用带过滤的 pop，只获取 MoveL 命令
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "MoveL", 10)) {
            continue;
        }

        std::string mode = cmd.get_mode();
        std::string mapping = cmd.get_mapping();
        std::string cmd_id = cmd.get_command_id();

        // 获取 per-mapping 的互斥锁，确保同一手臂的命令串行执行
        std::lock_guard<std::mutex> execution_lock(arm_controller::CommandQueueIPC::getMappingExecutionMutex(mapping));

        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        try {
            // ✅ 检查和处理 IPC 侧的模式过渡（包括 hook 检测）
            if (state_mgr) {
                bool transition_ok = state_mgr->transitionToMode("MoveL");
                if (!transition_ok) {
                    // 需要进入 hook 状态来安全切换
                    if (state_mgr->isInHookState()) {
                        RCLCPP_DEBUG(node_->get_logger(), "[%s] 🛑 MoveL in hook state - requesting HoldState transition",
                                     mapping.c_str());
                        // 暂停这条命令的处理，让 hook 完成
                        // 通知其他 consumers 继续处理
                        arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
                        continue;
                    }
                }
            }

            // 获取状态管理器并更新为执行中
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::EXECUTING);
                last_state[mapping] = arm_controller::ipc::ExecutionState::EXECUTING;
            }

            auto params = cmd.get_parameters();
            bool success = execute(mapping, params);

            if (success) {
                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveL command executed successfully (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::SUCCESS);
                    last_state[mapping] = arm_controller::ipc::ExecutionState::SUCCESS;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL command execution failed (ID: %s)",
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
                // 命令执行完成后
                arm_controller::ipc::ExecutorControllerState executor_state;
                strncpy(executor_state.current_mode, mode.c_str(), sizeof(executor_state.current_mode) - 1);
                executor_state.current_mode[sizeof(executor_state.current_mode) - 1] = '\0';  // 确保字符串以空字符结尾
                executor_state.execution_state = (int)arm_controller::ipc::ExecutionState::IDLE;
                state_mgr->updateFromExecutor(executor_state);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception in MoveL command execution: %s",
                        mapping.c_str(), e.what());
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                last_state[mapping] = arm_controller::ipc::ExecutionState::FAILED;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);
            }


        }
        // 通知其他 consumers
        arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
    }
}

void MoveLController::planning_worker_thread() {
    while (planning_worker_running_) {
        PlanningTask task;
        {
            std::unique_lock<std::mutex> lock(planning_queue_mutex_);
            planning_queue_cv_.wait(lock, [this] {
                return !planning_queue_.empty() || !planning_worker_running_;
            });

            if (!planning_worker_running_) break;

            if (planning_queue_.empty()) continue;

            task = planning_queue_.front();
            planning_queue_.pop();
        }

        // 在队列外执行规划和执行（释放队列锁）
        plan_and_execute(task.mapping, task.msg);
    }
}
