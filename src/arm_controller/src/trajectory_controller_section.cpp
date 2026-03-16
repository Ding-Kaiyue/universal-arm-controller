#include "arm_controller/trajectory_controller_section.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

TrajectoryControllerNode::TrajectoryControllerNode()
    : Node("trajectory_controller_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing Trajectory Controller Node");
}

void TrajectoryControllerNode::post_init() {
    RCLCPP_INFO(this->get_logger(), "Starting post-initialization");

    init_hardware();
    init_trajectory_interpolator();
    init_event_publisher();
    init_action_servers();
}

void TrajectoryControllerNode::init_hardware() {
    // 获取已初始化的硬件管理器实例（由controller_manager负责初始化）
    hardware_manager_ = HardwareManager::getInstance();
    if (!hardware_manager_) {
        RCLCPP_FATAL(this->get_logger(), "Failed to get HardwareManager instance");
        rclcpp::shutdown();
        return;
    }
}

void TrajectoryControllerNode::init_trajectory_interpolator() {
    try {
        trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

        // 加载插值器配置
        std::string pkg_path = ament_index_cpp::get_package_share_directory("arm_controller");
        std::string config_path = pkg_path + "/config/interpolator_config.yaml";
        YAML::Node config = YAML::LoadFile(config_path);

        if (config["interpolation"]["default"]) {
            auto default_config = config["interpolation"]["default"];
            trajectory_interpolator::SplineConfig spline_config;

            if (default_config["target_dt"]) {
                spline_config.target_dt = default_config["target_dt"].as<double>();
            }
            if (default_config["max_velocity"]) {
                spline_config.max_velocity = default_config["max_velocity"].as<double>();
            }
            if (default_config["max_acceleration"]) {
                spline_config.max_acceleration = default_config["max_acceleration"].as<double>();
            }
            if (default_config["max_jerk"]) {
                spline_config.max_jerk = default_config["max_jerk"].as<double>();
            }

            trajectory_interpolator_->setInterpolationConfig(spline_config);
            RCLCPP_INFO(this->get_logger(), "Loaded interpolator config - dt:%.3fs vel:%.2f acc:%.2f jerk:%.2f",
                        spline_config.target_dt, spline_config.max_velocity,
                        spline_config.max_acceleration, spline_config.max_jerk);
        }

        RCLCPP_INFO(this->get_logger(), "Trajectory interpolator initialized");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize trajectory interpolator: %s", e.what());
        rclcpp::shutdown();
    }
}

void TrajectoryControllerNode::init_action_servers() {
    if (!action_servers_.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Action servers already initialized");
        return;
    }

    // 获取所有可用的mapping
    std::vector<std::string> mappings = hardware_manager_->get_all_mappings();
    
    if (mappings.empty()) {
        RCLCPP_FATAL(this->get_logger(), 
            "No mappings available from HardwareManager - this should not happen with sequential initialization");
        rclcpp::shutdown();
        return;
    }
    
    for (const std::string& mapping : mappings) {
        std::string controller_name = hardware_manager_->get_controller_name(mapping);
        std::string action_server_name = "/" + controller_name + "/follow_joint_trajectory";

        RCLCPP_INFO(this->get_logger(), "Creating action server: %s for mapping: %s", 
                   action_server_name.c_str(), mapping.c_str());

        auto action_server = rclcpp_action::create_server<FollowJointTrajectory>(
            this, action_server_name,
            [this, mapping](const rclcpp_action::GoalUUID& uuid, 
                           std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
                return this->handle_goal(uuid, goal, mapping);
            },
            [this, mapping](const std::shared_ptr<GoalHandle> goal_handle) {
                return this->handle_cancel(goal_handle, mapping);
            },
            [this, mapping](const std::shared_ptr<GoalHandle> goal_handle) {
                this->handle_accepted(goal_handle, mapping);
            }
        );

        action_servers_[mapping] = action_server;
    }

    RCLCPP_INFO(this->get_logger(), "✅ Created %zu action servers for all mappings", action_servers_.size());
}

void TrajectoryControllerNode::init_event_publisher() {
    // 创建事件发布器，用于通知状态管理器
    action_event_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/action_controller_events", rclcpp::QoS(10).reliable());

    RCLCPP_INFO(this->get_logger(), "Action event publisher initialized");
}

void TrajectoryControllerNode::publish_action_event(const std::string& event_type, const std::string& mapping) {
    if (!action_event_publisher_) {
        RCLCPP_WARN(this->get_logger(), "Action event publisher not initialized");
        return;
    }

    auto event_msg = std_msgs::msg::String();
    // 格式: "event_type:mapping"
    event_msg.data = event_type + ":" + mapping;
    action_event_publisher_->publish(event_msg);
}

void TrajectoryControllerNode::on_trajectory_completed(const std::string& mapping) {
    // 发布轨迹完成事件
    publish_action_event("action_succeeded", mapping);
}

rclcpp_action::GoalResponse TrajectoryControllerNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal,
    const std::string& mapping) {

    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "✅ Trajectory goal accepted with %zu points for mapping: %s",
                goal->trajectory.points.size(), mapping.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryControllerNode::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle,
    const std::string& mapping) {

    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "❎ Received request to cancel trajectory goal for mapping: %s", mapping.c_str());

    // 发布取消事件
    publish_action_event("action_cancelled", mapping);

    // TODO: 实现轨迹取消逻辑
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryControllerNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle,
                                               const std::string& mapping) {
    // 发布事件通知状态管理器切换到ROS2ActionControl模式
    publish_action_event("action_goal_accepted", mapping);

    // 在新线程中执行轨迹，避免阻塞action server
    std::thread{std::bind(&TrajectoryControllerNode::execute_trajectory, this, goal_handle, mapping)}.detach();
}

void TrajectoryControllerNode::execute_trajectory(const std::shared_ptr<GoalHandle> goal_handle, const std::string& mapping) {
    const auto goal = goal_handle->get_goal();
    current_goal_handle_ = goal_handle;

    auto result = std::make_shared<FollowJointTrajectory::Result>();

    try {
        // 纯软件映射（如无反馈夹爪）：直接调用 gripper 控制接口
        if (hardware_manager_->get_motors_id(mapping).empty()) {
            bool success = execute_gripper_command(*goal, mapping);
            if (!success) {
                result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
                result->error_string = "Gripper command execution failed";
                goal_handle->abort(result);
                publish_action_event("action_failed", mapping);
                current_goal_handle_.reset();
                return;
            }

            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            result->error_string = "Software trajectory executed successfully";
            goal_handle->succeed(result);
            on_trajectory_completed(mapping);
            current_goal_handle_.reset();
            return;
        }

        // 将ROS轨迹转换为插值器格式
        auto interpolator_trajectory = arm_controller::utils::TrajectoryConverter::convertRosToInterpolator(goal->trajectory);

        // 检查轨迹点数：如果少于3个点，认为已在目标位置，直接执行不进行插值
        trajectory_interpolator::Trajectory final_trajectory = interpolator_trajectory;

        if (goal->trajectory.points.size() < 3) {
            RCLCPP_INFO(this->get_logger(), "⚠️ Trajectory has < 3 points (%zu), skipping interpolation and executing directly",
                        goal->trajectory.points.size());
        } else {
            // 分析轨迹动力学参数并应用动态配置
            auto dynamics = arm_controller::utils::TrajectoryConverter::analyzeTrajectoryDynamics(goal->trajectory);
            auto safe_params = arm_controller::utils::TrajectoryConverter::calculateSafeInterpolationParams(dynamics);

            RCLCPP_INFO(this->get_logger(), "Using dynamic interpolation params - vel:%.3f acc:%.3f jerk:%.3f",
                        safe_params.max_velocity, safe_params.max_acceleration, safe_params.max_jerk);

            // 使用动态配置加载轨迹
            if (!trajectory_interpolator_->loadTrajectoryWithDynamicConfig(interpolator_trajectory,
                                                                          safe_params.max_velocity,
                                                                          safe_params.max_acceleration,
                                                                          safe_params.max_jerk)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory with dynamic config");
                result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
                result->error_string = "Failed to load trajectory";
                goal_handle->abort(result);
                publish_action_event("action_aborted", mapping);
                return;
            }

            final_trajectory = trajectory_interpolator_->interpolate();
            RCLCPP_INFO(this->get_logger(), "Interpolation successful, trajectory has %zu points (was %zu)",
                        final_trajectory.points.size(), goal->trajectory.points.size());
        }

        // 使用异步执行轨迹以支持暂停/恢复/取消（不显示硬件驱动的进度条，避免重复显示）
        Trajectory hw_trajectory = arm_controller::utils::TrajectoryConverter::convertInterpolatorToHardwareDriver(final_trajectory);
        std::string execution_id = hardware_manager_->execute_trajectory_async(mapping, hw_trajectory, true);

        if (execution_id.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❎ Failed to start trajectory execution on mapping: %s", mapping.c_str());
            result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
            result->error_string = "Trajectory execution failed";
            goal_handle->abort(result);
            publish_action_event("action_failed", mapping);
            return;
        }

        // 等待轨迹执行完成
        if (!hardware_manager_->wait_for_trajectory_completion(mapping)) {
            RCLCPP_ERROR(this->get_logger(), "❎ Failed to wait for trajectory completion");
            result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
            result->error_string = "Trajectory execution failed";
            goal_handle->abort(result);
            publish_action_event("action_failed", mapping);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "Trajectory executed successfully";
        goal_handle->succeed(result);

        // 轨迹完成回调
        on_trajectory_completed(mapping);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during trajectory execution: %s", e.what());
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = std::string("Exception: ") + e.what();
        goal_handle->abort(result);
        publish_action_event("action_failed", mapping);
    }

    current_goal_handle_.reset();
}

bool TrajectoryControllerNode::execute_gripper_command(
    const FollowJointTrajectory::Goal& goal,
    const std::string& mapping) {
    RCLCPP_INFO(this->get_logger(),
                "[%s] Executing software gripper trajectory with %zu points",
                mapping.c_str(), goal.trajectory.points.size());

    const auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Gripper command failed: hardware driver not initialized", mapping.c_str());
        return false;
    }

    const std::string interface = hardware_manager_->get_interface_for_mapping(mapping);
    if (interface.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Gripper command failed: interface not configured", mapping.c_str());
        return false;
    }

    const auto& trajectory = goal.trajectory;
    if (trajectory.points.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Gripper command failed: empty trajectory points", mapping.c_str());
        return false;
    }

    // 使用最后一个轨迹点作为 slider 目标值
    const auto& point = trajectory.points.back();
    if (point.positions.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Gripper command failed: empty positions", mapping.c_str());
        return false;
    }

    int finger_idx = -1;
    if (!trajectory.joint_names.empty()) {
        auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), "left_gripper_finger1_joint");
        if (it != trajectory.joint_names.end()) {
            finger_idx = static_cast<int>(std::distance(trajectory.joint_names.begin(), it));
        }
    }
    if (finger_idx < 0 || finger_idx >= static_cast<int>(point.positions.size())) {
        finger_idx = 0;
    }

    const std::string finger_name = "left_gripper_finger1_joint";
    JointLimits limits;
    hardware_manager_->get_joint_limits(finger_name, limits);
    double min_pos = limits.has_position_limits ? limits.min_position : 0.0;
    double max_pos = limits.has_position_limits ? limits.max_position : 0.025;
    double target_pos = point.positions[static_cast<size_t>(finger_idx)];

    double ratio = 0.0;
    if (max_pos > min_pos + 1e-9) {
        ratio = (target_pos - min_pos) / (max_pos - min_pos);
    }
    ratio = std::clamp(ratio, 0.0, 1.0);
    // PGC 实际方向与 RViz 关节值相反：关节越小越夹紧
    // 因此将关节归一化值做反向映射到协议百分比
    uint8_t position_percent = static_cast<uint8_t>(std::lround((1.0 - ratio) * 100.0));

    // 协议约束：velocity [1,100], effort [20,100]
    uint8_t velocity_percent = 50;
    if (finger_idx >= 0 && finger_idx < static_cast<int>(point.velocities.size())) {
        double vel = std::abs(point.velocities[static_cast<size_t>(finger_idx)]);
        double max_vel = (limits.has_velocity_limits && limits.max_velocity > 1e-6) ? limits.max_velocity : 1.0;
        velocity_percent = static_cast<uint8_t>(std::lround(std::clamp(vel / max_vel, 0.0, 1.0) * 100.0));
    }
    velocity_percent = std::clamp<uint8_t>(velocity_percent, 1, 100);

    uint8_t effort_percent = 50;
    if (finger_idx >= 0 && finger_idx < static_cast<int>(point.effort.size())) {
        double effort = std::abs(point.effort[static_cast<size_t>(finger_idx)]);
        effort_percent = static_cast<uint8_t>(std::lround(std::clamp(effort, 0.0, 100.0)));
    }
    effort_percent = std::clamp<uint8_t>(effort_percent, 20, 100);

    // 1 = PGC_Gripper
    hardware_driver->control_gripper(interface, 1, position_percent, velocity_percent, effort_percent);

    // 为 RViz/MoveIt 同步软件状态：finger2 与 finger1 同步（与 URDF mimic 一致）
    const auto& mapping_joint_names = hardware_manager_->get_joint_names(mapping);
    if (!mapping_joint_names.empty()) {
        std::vector<double> positions(mapping_joint_names.size(), 0.0);
        std::vector<double> velocities(mapping_joint_names.size(), 0.0);
        for (size_t i = 0; i < mapping_joint_names.size(); ++i) {
            if (mapping_joint_names[i] == "left_gripper_finger1_joint" ||
                mapping_joint_names[i] == "left_gripper_finger2_joint") {
                positions[i] = target_pos;
                if (finger_idx >= 0 && finger_idx < static_cast<int>(point.velocities.size())) {
                    velocities[i] = point.velocities[static_cast<size_t>(finger_idx)];
                }
            }
        }
        hardware_manager_->update_software_joint_state(mapping, positions, velocities);
    }

    RCLCPP_INFO(this->get_logger(),
                "[%s] Sent gripper command via %s: pos=%u vel=%u effort=%u",
                mapping.c_str(), interface.c_str(),
                position_percent, velocity_percent, effort_percent);

    return true;
}
