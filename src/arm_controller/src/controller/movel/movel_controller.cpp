#include "movel_controller.hpp"
#include "controller_interface.hpp"
#include "hardware/hardware_manager.hpp"
#include <set>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'MoveL', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/movel_action geometry_msgs/msg/Pose "{position: {x: 0.19, y: 0.0, z: 0.63}, orientation: {x: -0.4546, y: 0.4546, z: -0.5417, w: 0.5417}}"

MoveLController::MoveLController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<geometry_msgs::msg::Pose>("MoveL", node)
{
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化轨迹插值器
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

    std::string input_topic;
    node_->get_parameter("controllers.MoveL.input_topic", input_topic);

    sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
        input_topic, rclcpp::QoS(10).reliable(),
        std::bind(&MoveLController::trajectory_callback, this, std::placeholders::_1)
    );

    // 初始化轨迹规划服务
    initialize_planning_services();
}

void MoveLController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] MoveL: not found in hardware configuration."
        );
    }

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

    // 保存当前激活的mapping
    active_mapping_ = mapping;
    is_active_ = true;
}

bool MoveLController::stop(const std::string& mapping) {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] MoveLController deactivated", mapping.c_str());
    return true;
}

void MoveLController::trajectory_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!is_active_) return;

    // 使用mapping进行规划和执行
    plan_and_execute(active_mapping_, msg);
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
                    moveit_adapter, node_);

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
        return;
    }

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

void MoveLController::execute_trajectory(
    const trajectory_interpolator::Trajectory& trajectory,
    const std::string& mapping) {
    try {
        // 获取对应的interface
        std::string interface = hardware_manager_->get_interface(mapping);
        if (interface.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: No interface found for mapping", mapping.c_str());
            return;
        }

        // 执行轨迹
        if (!hardware_manager_->executeTrajectory(interface, trajectory)) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Failed to execute trajectory on interface: %s",
                        mapping.c_str(), interface.c_str());
            return;
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MoveL: Exception during trajectory execution: %s",
                    mapping.c_str(), e.what());
        return;
    }
}
