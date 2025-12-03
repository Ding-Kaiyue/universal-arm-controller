#include "movej_controller.hpp"
#include "controller_interface.hpp"
#include "hardware/hardware_manager.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"
#include <controller_interfaces/srv/work_mode.hpp>
#include <set>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'MoveJ', mapping: 'single_arm'}"
// ros2 topic pub --once --w 1 /controller_api/movej_action/left_arm sensor_msgs/msg/JointState "{position: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"
// ros2 topic pub --rate 1 /controller_api/movej_action/left_arm sensor_msgs/msg/JointState "{position: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"
// ros2 topic pub --once /trajectory_control controller_interfaces/msg/TrajectoryControl "{mapping: 'single_arm', action: 'Cancel'}"

MoveJController::MoveJController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<sensor_msgs::msg::JointState>("MoveJ", node)
{
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化轨迹插值器
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

    // 初始化轨迹规划服务
    initialize_planning_services();

    // 启动IPC命令队列消费线程（早期启动以接收API发送的命令）
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&MoveJController::command_queue_consumer_thread, this);
        RCLCPP_INFO(node_->get_logger(), "✅ MoveJ: IPC queue consumer thread started early");
    }

    // 注意：话题订阅在 init_subscriptions() 中创建，以支持 {mapping} 占位符
}

void MoveJController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] MoveJ: not found in hardware configuration."
        );
    }

    // 仅在首次启动时创建队列消费线程
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&MoveJController::command_queue_consumer_thread, this);
        RCLCPP_INFO(node_->get_logger(), "✅ MoveJ: Command queue consumer thread started");
    }

    // 调用基类 start() 设置 per-mapping 的 is_active_[mapping] = true
    // 订阅已在 ControllerNode::init_controllers() 时提前创建，Lambda 会直接调用 plan_and_execute
    TrajectoryControllerImpl::start(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveJController activated", mapping.c_str());
}

bool MoveJController::stop(const std::string& mapping) {
    // 清理该 mapping 的话题订阅
    // cleanup_subscriptions(mapping);

    // 调用基类 stop() 设置 per-mapping 的 is_active_[mapping] = false
    TrajectoryControllerImpl::stop(mapping);

    // 注意：轨迹执行是在 ROS2 callback 中同步进行的
    // execute_trajectory 是阻塞调用，stop() 被调用时表示上一个轨迹已执行完毕
    // 或模式切换已等待轨迹完成

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveJController deactivated", mapping.c_str());
    return true;
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
                    node_, planning_group);

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;

                // 创建轨迹规划服务
                auto motion_planning_service = std::make_shared<trajectory_planning::application::services::MotionPlanningService>(
                    moveit_adapter,
                    node_);

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
    
    // 进行轨迹规划
    auto planning_result = motion_planning_services_[mapping]->planJointMotion(*msg);
    if (!planning_result.success) {
        return;
    }

    // 检查轨迹点数
    if (planning_result.trajectory.size() < 2) {
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

bool MoveJController::move(const std::string& mapping, const std::vector<double>& parameters) {
    // 检查mapping和规划服务
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() ||
        !motion_planning_services_[mapping]) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Planning service not found", mapping.c_str());
        return false;
    }

    // 获取期望的关节数
    size_t expected_joint_count = hardware_manager_->get_joint_count(mapping);

    // 处理参数长度：自动填充或裁短
    std::vector<double> joint_positions = parameters;
    if (joint_positions.size() < expected_joint_count) {
        // 用0填充不足的部分
        joint_positions.resize(expected_joint_count, 0.0);
        RCLCPP_WARN(node_->get_logger(), "[%s] MoveJ: Parameters padded to %zu joints",
                   mapping.c_str(), expected_joint_count);
    } else if (joint_positions.size() > expected_joint_count) {
        // 裁短多余的部分
        joint_positions.resize(expected_joint_count);
        RCLCPP_WARN(node_->get_logger(), "[%s] MoveJ: Parameters truncated to %zu joints",
                   mapping.c_str(), expected_joint_count);
    }

    // 构建 JointState 消息
    auto joint_state = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state->position = joint_positions;

    // 调用原有的 plan_and_execute
    plan_and_execute(mapping, joint_state);
    return true;
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
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Failed to execute trajectory on mapping: %s",
                        mapping.c_str(), mapping.c_str());
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveJ: Trajectory execution started (ID: %s)",
                   mapping.c_str(), execution_id.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ: Exception during trajectory execution: %s",
                    mapping.c_str(), e.what());
        return;
    }
}

void MoveJController::command_queue_consumer_thread() {
    RCLCPP_INFO(node_->get_logger(), "🔄 MoveJ: IPC queue consumer thread running");

    arm_controller::TrajectoryCommandIPC cmd;
    std::map<std::string, std::string> current_mode;  // Track current mode per mapping

    while (consumer_running_) {
        if (!arm_controller::CommandQueueIPC::getInstance().pop(cmd, 1000)) {
            continue;
        }

        std::string mode = cmd.get_mode();
        std::string mapping = cmd.get_mapping();
        std::string cmd_id = cmd.get_command_id();

        RCLCPP_INFO(node_->get_logger(), "📥 MoveJ: Received IPC command (ID: %s, mode: %s, mapping: %s)",
                   cmd_id.c_str(), mode.c_str(), mapping.c_str());

        // 只处理 MoveJ 命令，其他模式的命令由对应的控制器处理
        if (mode != "MoveJ") {
            RCLCPP_DEBUG(node_->get_logger(), "[%s] ⏭️  Skipping non-MoveJ command (mode: %s, ID: %s)",
                        mapping.c_str(), mode.c_str(), cmd_id.c_str());
            continue;
        }

        // Now execute the MoveJ command
        try {
            RCLCPP_INFO(node_->get_logger(), "[%s] 🚀 Executing MoveJ command (ID: %s)",
                       mapping.c_str(), cmd_id.c_str());

            auto params = cmd.get_parameters();
            bool success = move(mapping, params);

            if (success) {
                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveJ command executed successfully (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveJ command execution failed (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception in MoveJ command execution: %s",
                        mapping.c_str(), e.what());
        }
    }

    RCLCPP_INFO(node_->get_logger(), "🔄 MoveJ: IPC queue consumer thread stopped");
}
