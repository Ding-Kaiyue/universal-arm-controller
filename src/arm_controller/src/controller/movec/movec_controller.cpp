#include "movec_controller.hpp"
#include "controller_interface.hpp"
#include "hardware/hardware_manager.hpp"
#include "trajectory_planning_interfaces/msg/move_c_request.hpp"
#include <set>

// 使用步骤:
// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'MoveC', mapping: 'single_arm'}"

// 注意: planArcMotion 会自动使用当前位置作为起点，所以只需要提供 via_point 和 goal_point
/* ros2 topic pub --once /controller_api/movec_action/single_arm geometry_msgs/msg/PoseArray "{poses: [
   {position: {x: 0.30, y: 0.0, z: 0.55}, orientation: {x: -0.5, y: 0.5, z: -0.5, w: 0.5}},
   {position: {x: 0.25, y: 0.0, z: 0.60}, orientation: {x: -0.4777, y: 0.4777, z: -0.5213, w: 0.5213}}
   ]}"
*/

MoveCController::MoveCController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<geometry_msgs::msg::PoseArray>("MoveC", node)
{
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化轨迹插值器
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

    // 初始化轨迹规划服务
    initialize_planning_services();
}

void MoveCController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] MoveC: not found in hardware configuration."
        );
    }

    // 保存当前激活的 mapping
    active_mapping_ = mapping;
    is_active_ = true;

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    // 同步 MoveIt 状态到当前机械臂位置，防止规划从错误的起始位置开始
    if (moveit_adapters_.find(mapping) != moveit_adapters_.end() && moveit_adapters_[mapping]) {
        auto current_positions = hardware_manager_->get_current_joint_positions(mapping);
        if (!current_positions.empty()) {
            moveit_adapters_[mapping]->setStartState(current_positions);
            RCLCPP_DEBUG(node_->get_logger(), "[%s] MoveC: Synced MoveIt state to current position", mapping.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] MoveC: Failed to get current positions for state sync", mapping.c_str());
        }
    }
}

bool MoveCController::stop(const std::string& mapping) {
    is_active_ = false;

    // 清理资源
    active_mapping_.clear();

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveCController deactivated", mapping.c_str());
    return true;
}

void MoveCController::trajectory_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (!is_active_) return;

    // 使用mapping进行规划和执行
    plan_and_execute(active_mapping_, msg);
}

void MoveCController::initialize_planning_services() {
    try {
        // 获取所有mapping
        auto all_mappings = hardware_manager_->get_all_mappings();
        if (all_mappings.empty()) {
            RCLCPP_WARN(node_->get_logger(), "❎ MoveC: No mappings configured");
            return;
        }

        // 为每个mapping初始化规划服务
        for (const auto& mapping : all_mappings) {
            std::string planning_group = hardware_manager_->get_planning_group(mapping);

            if (planning_group.empty()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ❎ MoveC: No planning group configured, skipping...", mapping.c_str());
                continue;
            }

            try {
                // 创建 MoveItAdapter
                auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, planning_group);

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveC: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;

                // 创建轨迹规划服务
                auto motion_planning_service = std::make_shared<trajectory_planning::application::services::MotionPlanningService>(
                    moveit_adapter, node_);

                if (!motion_planning_service) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveC: Failed to create MotionPlanningService", mapping.c_str());
                    continue;
                }

                // 注册MoveC策略
                motion_planning_service->registerMoveCStrategy();   // 用到了TrajectoryPlanning的MoveC策略

                motion_planning_services_[mapping] = motion_planning_service;
                mapping_to_planning_group_[mapping] = planning_group;

                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveC: Planning service initialized (planning group: '%s')",
                           mapping.c_str(), planning_group.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "❎ MoveC: Failed to initialize planning services: %s", e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ MoveC: Failed to initialize planning services: %s", e.what());
    }
}

void MoveCController::plan_and_execute(const std::string& mapping, const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // 查找mapping
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() ||
        !motion_planning_services_[mapping]) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ❎ MoveC: Planning service not found. This strategy must be registered first.", mapping.c_str());
        return;
    }

    // 将 PoseArray 转换为 MoveCRequest
    trajectory_planning_interfaces::msg::MoveCRequest movec_request;

    // 默认使用 ARC 类型，用户可以根据需要修改
    movec_request.route_type = 0;  // 0: ARC, 1: BEZIER, 2: CIRCLE, 3: CIRCLE3PT
    movec_request.route_name = "arc_trajectory";

    // 复制所有途径点
    movec_request.waypoints = msg->poses;

    // 进行轨迹规划
    auto planning_result = motion_planning_services_[mapping]->planArcMotion(movec_request);

    if (!planning_result.success) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveC: Planning failed", mapping.c_str());
        return;
    }

    // 检查轨迹点数
    if (planning_result.trajectory.size() < 2) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveC: Current pose and via pose and end pose are the same, no movement needed",
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

trajectory_interpolator::Trajectory MoveCController::interpolate_trajectory(
    const trajectory_interpolator::Trajectory& interpolator_trajectory,
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    const std::string& mapping) {

    if (!trajectory_interpolator_) {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] ⚠️ MoveC: No interpolator available, using original trajectory", mapping.c_str());
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
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ MoveC: Interpolation failed, using original trajectory", mapping.c_str());
            return interpolator_trajectory;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ MoveC: Interpolation exception: %s, using original trajectory",
                   mapping.c_str(), e.what());
        return interpolator_trajectory;
    }
}

void MoveCController::execute_trajectory(
    const trajectory_interpolator::Trajectory& trajectory,
    const std::string& mapping) {
    try {
        // 使用转换工具将轨迹转换为硬件驱动格式
        Trajectory hw_trajectory = arm_controller::utils::TrajectoryConverter::convertInterpolatorToHardwareDriver(trajectory);

        // 使用异步执行轨迹（不阻塞，并保存 execution_id 以支持暂停/恢复/取消）
        std::string execution_id = hardware_manager_->execute_trajectory_async(mapping, hw_trajectory, true);
        if (execution_id.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveC: Failed to execute trajectory on mapping: %s",
                        mapping.c_str(), mapping.c_str());
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MoveC: Trajectory execution started (ID: %s)",
                   mapping.c_str(), execution_id.c_str());


    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveC: Exception during trajectory execution: %s",
                    mapping.c_str(), e.what());
        return;
    }
}
