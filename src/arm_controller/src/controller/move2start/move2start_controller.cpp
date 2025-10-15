#include "move2start_controller.hpp"
#include "controller_interface.hpp"
#include <limits>
#include <algorithm>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'Move2Start'}"

Move2StartController::Move2StartController(const rclcpp::Node::SharedPtr& node)
    : UtilityControllerBase("Move2Start", node)
{
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化轨迹插值器
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

    // 预初始化所有mapping的规划服务
    initialize_planning_services();
}

void Move2StartController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] Move2Start: not found in hardware configuration."
        );
    }
    // 执行回启动位置的规划和运动
    if (!move_to_start_position(mapping)) {
        throw std::runtime_error(
            "❎ [" + mapping + "] Move2Start: Failed to move to initial position"
        );
    }

    is_active_ = true;
}

bool Move2StartController::stop(const std::string& mapping) {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] Move2StartController deactivated", mapping.c_str());
    return true;
}

void Move2StartController::initialize_planning_services() {
    try {
        // 获取所有mapping
        auto all_mappings = hardware_manager_->get_all_mappings();
        if (all_mappings.empty()) {
            RCLCPP_WARN(node_->get_logger(), "❎ Move2Start: No mappings configured");
            return;
        }

        // 为每个mapping初始化规划服务
        for (const auto& mapping : all_mappings) {
            std::string planning_group = hardware_manager_->get_planning_group(mapping);

            if (planning_group.empty()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ❎ Move2Start: No planning group configured, skipping...", mapping.c_str());
                continue;
            }

            try {
                // 创建 MoveItAdapter
                auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, planning_group);

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Move2Start: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;

                // 创建轨迹规划服务
                auto motion_planning_service = std::make_shared<trajectory_planning::application::services::MotionPlanningService>(
                    moveit_adapter, node_);

                if (!motion_planning_service) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Move2Start: Failed to create MotionPlanningService", mapping.c_str());
                    continue;
                }

                // 注册MoveJ策略
                motion_planning_service->registerMoveJStrategy();   // 用到了TrajectoryPlanning的MoveJ策略

                motion_planning_services_[mapping] = motion_planning_service;
                mapping_to_planning_group_[mapping] = planning_group;

                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Move2Start: Planning service initialized (planning group: '%s')",
                           mapping.c_str(), planning_group.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Move2Start: Exception while initializing planning service: %s",
                           mapping.c_str(), e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Move2Start: Failed to initialize planning services: %s", e.what());
    }
}

bool Move2StartController::move_to_start_position(const std::string& mapping) {
    // 检查前置状态
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() 
        || !motion_planning_services_[mapping]) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ❎ Planning service not found. This strategy must be registered first.", mapping.c_str());
        return false;
    }

    // 获取启动位置配置
    auto start_position = hardware_manager_->get_start_position(mapping);
    if (start_position.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Move2Start: No start position configured", mapping.c_str());
        return false;
    }

    // 创建目标关节状态
    sensor_msgs::msg::JointState target_joint_state;
    target_joint_state.header.stamp = node_->now();
    target_joint_state.header.frame_id = hardware_manager_->get_frame_id(mapping);
    target_joint_state.name = hardware_manager_->get_joint_names(mapping);
    target_joint_state.position = start_position;

    // 验证关节数量
    if (start_position.size() != target_joint_state.name.size()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Move2Start: Start position size (%zu) doesn't match joint count (%zu)",
                     mapping.c_str(), start_position.size(), target_joint_state.name.size());
        return false;
    }

    // 进行轨迹规划
    auto planning_result = motion_planning_services_[mapping]->planJointMotion(target_joint_state);
    
    // 检查轨迹点数
    if (planning_result.trajectory.size() < 2) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Move2Start: Already at target position, no movement needed",
                   mapping.c_str());
        return true;
    }

    if (!planning_result.success) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Move2Start: Planning failed: %s",
                     mapping.c_str(), planning_result.error_message.c_str());
        return false;
    }

    // 转换轨迹格式
    trajectory_interpolator::Trajectory interpolator_trajectory =
        arm_controller::utils::TrajectoryConverter::convertPlanningToInterpolator(
            planning_result.trajectory, target_joint_state.name);

    // 分析轨迹动力学参数
    auto dynamics = arm_controller::utils::TrajectoryConverter::analyzeTrajectoryDynamics(planning_result.trajectory);
    auto safe_params = arm_controller::utils::TrajectoryConverter::calculateSafeInterpolationParams(dynamics);
        
    // 插值轨迹
    trajectory_interpolator::Trajectory final_trajectory = interpolate_trajectory(
        interpolator_trajectory,
        safe_params.max_velocity,
        safe_params.max_acceleration,
        safe_params.max_jerk,
        mapping);

    // 执行轨迹
    return execute_trajectory(final_trajectory, mapping);
}

trajectory_interpolator::Trajectory Move2StartController::interpolate_trajectory(
    const trajectory_interpolator::Trajectory& interpolator_trajectory,
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    const std::string& mapping) {

    if (!trajectory_interpolator_) {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] ⚠️ Move2Start: No interpolator available, using original trajectory", mapping.c_str());
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
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ Move2Start: Interpolation failed, using original trajectory", mapping.c_str());
            return interpolator_trajectory;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ Move2Start: Interpolation exception: %s, using original trajectory",
                   mapping.c_str(), e.what());
        return interpolator_trajectory;
    }
}

bool Move2StartController::execute_trajectory(
    const trajectory_interpolator::Trajectory& trajectory,
    const std::string& mapping) {
    try {
        // 获取对应的interface
        std::string interface = hardware_manager_->get_interface(mapping);
        if (interface.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Move2Start: No interface found for mapping", mapping.c_str());
            return false;
        }

        // 执行轨迹
        if (!hardware_manager_->executeTrajectory(interface, trajectory)) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Failed to execute trajectory on interface: %s",
                        mapping.c_str(), interface.c_str());
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception during trajectory execution: %s",
                    mapping.c_str(), e.what());
        return false;
    }
}

