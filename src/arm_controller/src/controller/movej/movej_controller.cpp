#include "movej_controller.hpp"
#include "controller_interface.hpp"
#include "hardware/hardware_manager.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"
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

    std::string input_topic;
    node_->get_parameter("controllers.MoveJ.input_topic", input_topic);
    
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        input_topic, rclcpp::QoS(10).reliable(),
        std::bind(&MoveJController::trajectory_callback, this, std::placeholders::_1)
    );


    // 初始化轨迹规划服务
    initialize_planning_services();

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

    // 保存当前激活的mapping
    active_mapping_ = mapping;
    is_active_ = true;

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveJController activated", mapping.c_str());

}

bool MoveJController::stop(const std::string& mapping) {
    is_active_ = false;

    // 清理资源
    active_mapping_.clear();

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    // 注意：轨迹执行是在 ROS2 callback 中同步进行的
    // execute_trajectory 是阻塞调用，stop() 被调用时表示上一个轨迹已执行完毕
    // 或模式切换已等待轨迹完成

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveJController deactivated", mapping.c_str());
    return true;
}

void MoveJController::trajectory_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!is_active_) return;

    // 使用保存的 mapping 进行规划和执行
    plan_and_execute(active_mapping_, msg);
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
