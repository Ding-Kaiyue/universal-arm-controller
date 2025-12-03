#include "movel_controller.hpp"
#include "controller_interface.hpp"
#include "hardware/hardware_manager.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <controller_interfaces/srv/work_mode.hpp>
#include <set>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'MoveL', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/movel_action/single_arm geometry_msgs/msg/Pose "{position: {x: 0.19, y: 0.0, z: 0.63}, orientation: {x: -0.4546, y: 0.4546, z: -0.5417, w: 0.5417}}"

MoveLController::MoveLController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<geometry_msgs::msg::Pose>("MoveL", node)
{
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化轨迹插值器
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

    // 初始化轨迹规划服务
    initialize_planning_services();

    // 启动IPC命令队列消费线程（早期启动以接收API发送的命令）
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&MoveLController::command_queue_consumer_thread, this);
        RCLCPP_INFO(node_->get_logger(), "✅ MoveL: IPC queue consumer thread started early");
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

    // 仅在首次启动时创建队列消费线程
    // if (!consumer_running_) {
    //     consumer_running_ = true;
    //     queue_consumer_ = std::make_unique<std::thread>(&MoveLController::command_queue_consumer_thread, this);
    //     RCLCPP_INFO(node_->get_logger(), "✅ MoveL: Command queue consumer thread started");
    // }

    // 同步 MoveIt 状态到当前机械臂位置，防止规划从错误的起始位置开始
    if (moveit_adapters_.find(mapping) != moveit_adapters_.end() && moveit_adapters_[mapping]) {
        auto current_positions = hardware_manager_->get_current_joint_positions(mapping);
        if (!current_positions.empty()) {
            moveit_adapters_[mapping]->setStartState(current_positions);
            RCLCPP_DEBUG(node_->get_logger(), "[%s] MoveL: Synced MoveIt state to current position", mapping.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] MoveL: Failed to get current positions for state sync", mapping.c_str());
        }
    }

    // 调用基类 start() 设置 per-mapping 的 is_active_[mapping] = true
    // 订阅已在 ControllerNode::init_controllers() 时提前创建，Lambda 会直接调用 plan_and_execute
    TrajectoryControllerImpl::start(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveLController activated", mapping.c_str());
}

bool MoveLController::stop(const std::string& mapping) {
    // 调用基类 stop() 设置 per-mapping 的 is_active_[mapping] = false
    TrajectoryControllerImpl::stop(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveLController deactivated", mapping.c_str());
    return true;
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
                    node_, planning_group);

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;

                // 创建轨迹规划服务
                auto motion_planning_service = std::make_shared<trajectory_planning::application::services::MotionPlanningService>(
                    moveit_adapter, 
                    node_);

                if (!motion_planning_service) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Failed to create MotionPlanningService", mapping.c_str());
                    continue;
                }

                // 注册MoveL策略
                motion_planning_service->registerMoveLStrategy();   // 用到了TrajectoryPlanning的MoveL策略

                motion_planning_services_[mapping] = motion_planning_service;
                mapping_to_planning_group_[mapping] = planning_group;

                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveL: Planning service initialized (planning group: '%s')",
                           mapping.c_str(), planning_group.c_str());
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

    // 进行轨迹规划（可选JOINT/CARTESIAN/INTELLIGENT）
    auto planning_result = motion_planning_services_[mapping]->planLinearMotion(
        *msg,
        trajectory_planning::infrastructure::planning::MoveLPlanningStrategy::PlanningType::INTELLIGENT
    );
    if (!planning_result.success) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Planning failed", mapping.c_str());
        last_planning_success_[mapping] = false;
        return;
    }
    last_planning_success_[mapping] = true;

    // 检查轨迹点数
    if (planning_result.trajectory.size() < 2) {
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

bool MoveLController::move(const std::string& mapping, const std::vector<double>& parameters) {
    // 检查mapping和规划服务
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() ||
        !motion_planning_services_[mapping]) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Planning service not found", mapping.c_str());
        return false;
    }

    // 处理参数长度：自动填充或裁短
    std::vector<double> pose = parameters;
    if (pose.size() < 7) {
        // 用0填充不足的部分
        pose.resize(7, 0.0);
        RCLCPP_WARN(node_->get_logger(), "[%s] MoveL: Parameters padded to %d pose parameters: x, y, z, qx, qy, qz, qw",
                   mapping.c_str(), 7);
    } else if (pose.size() > 7) {
        // 裁短多余的部分
        pose.resize(7);
        RCLCPP_WARN(node_->get_logger(), "[%s] MoveL: Parameters truncated to %d pose parameters: x, y, z, qx, qy, qz, qw",
                   mapping.c_str(), 7);
    }

    // 构建 Pose 消息
    auto pose_state = std::make_shared<geometry_msgs::msg::Pose>();
    pose_state->position.x = pose[0];
    pose_state->position.y = pose[1];
    pose_state->position.z = pose[2];
    pose_state->orientation.x = pose[3];
    pose_state->orientation.y = pose[4];
    pose_state->orientation.z = pose[5];
    pose_state->orientation.w = pose[6];

    // 初始化规划状态为未尝试
    last_planning_success_[mapping] = true;

    // 调用原有的 plan_and_execute，它会更新 last_planning_success_
    plan_and_execute(mapping, pose_state);

    // 根据规划结果返回
    return last_planning_success_[mapping];
}

void MoveLController::execute_trajectory(
    const trajectory_interpolator::Trajectory& trajectory,
    const std::string& mapping) {
    try {
        Trajectory hw_trajectory = arm_controller::utils::TrajectoryConverter::convertInterpolatorToHardwareDriver(trajectory);

        std::string execution_id = hardware_manager_->execute_trajectory_async(mapping, hw_trajectory, true);
        if (execution_id.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Failed to execute trajectory",
                        mapping.c_str());
            last_planning_success_[mapping] = false;
            return;
        }

        bool wait_success = hardware_manager_->wait_for_trajectory_completion(mapping, 0);
        last_planning_success_[mapping] = wait_success;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Exception during trajectory execution: %s",
                    mapping.c_str(), e.what());
        last_planning_success_[mapping] = false;
        return;
    }
}

void MoveLController::command_queue_consumer_thread() {
    RCLCPP_INFO(node_->get_logger(), "🔄 MoveL: IPC queue consumer thread running");

    arm_controller::TrajectoryCommandIPC cmd;
    std::map<std::string, std::string> current_mode;
    std::map<std::string, arm_controller::ipc::ExecutionState> last_state;  // Track last execution state per mapping

    while (consumer_running_) {
        // 使用带过滤的 pop，只获取 MoveL 命令
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "MoveL", 1000)) {
            continue;
        }

        std::string mode = cmd.get_mode();
        std::string mapping = cmd.get_mapping();
        std::string cmd_id = cmd.get_command_id();

        RCLCPP_INFO(node_->get_logger(), "[%s] MoveL: Received IPC command (ID: %s)",
                   mapping.c_str(), cmd_id.c_str());

        // 等待相同 arm 的所有命令执行完成（无论是哪个 controller 的命令）
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
        if (state_mgr) {
            auto current = state_mgr->getExecutionState();
            // 如果上一条命令还在执行中，等待其完成
            if (current == arm_controller::ipc::ExecutionState::EXECUTING) {
                RCLCPP_INFO(node_->get_logger(), "[%s] ⏳ Waiting for previous command to complete before executing next (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
                // 无限等待直到前一条命令完成
                while (consumer_running_) {
                    current = state_mgr->getExecutionState();
                    if (current != arm_controller::ipc::ExecutionState::EXECUTING) {
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }

        // Now execute the MoveL command
        try {
            RCLCPP_INFO(node_->get_logger(), "[%s] 🚀 Executing MoveL command (ID: %s)",
                       mapping.c_str(), cmd_id.c_str());

            // 获取状态管理器并更新为执行中
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::EXECUTING);
                last_state[mapping] = arm_controller::ipc::ExecutionState::EXECUTING;
            }

            auto params = cmd.get_parameters();
            bool success = move(mapping, params);

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
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception in MoveL command execution: %s",
                        mapping.c_str(), e.what());
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                last_state[mapping] = arm_controller::ipc::ExecutionState::FAILED;
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(), "🔄 MoveL: IPC queue consumer thread stopped");
}
