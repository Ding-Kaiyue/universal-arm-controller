#include "arm_controller/hardware/hardware_manager.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <set>
#include <limits>

std::shared_ptr<HardwareManager> HardwareManager::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = std::shared_ptr<HardwareManager>(new HardwareManager());
    }
    return instance_;
}

bool HardwareManager::initialize(rclcpp::Node::SharedPtr node) {
    node_ = node;

    // 加载硬件配置
    load_hardware_config();

    try {
        // 创建CANFD电机驱动
        std::vector<std::string> interface_names;
        std::set<std::string> unique_interfaces;
        RCLCPP_INFO(node_->get_logger(), "DEBUG: mapping_to_interface_ contains %zu entries", mapping_to_interface_.size());
        for (const auto& [mapping, interface] : mapping_to_interface_) {
            RCLCPP_INFO(node_->get_logger(), "  - mapping='%s', interface='%s'", mapping.c_str(), interface.c_str());
            if (interface.empty()) {
                RCLCPP_WARN(node_->get_logger(), "    ⚠️  WARNING: Empty interface for mapping '%s'!", mapping.c_str());
            }
            if (!interface.empty()) {
                unique_interfaces.insert(interface);
            }
        }
        // 软件映射（如夹爪）也可能需要总线接口
        for (const auto& [mapping, interface] : software_mapping_to_interface_) {
            if (!interface.empty()) {
                unique_interfaces.insert(interface);
            }
        }
        interface_names.assign(unique_interfaces.begin(), unique_interfaces.end());
        if (interface_names.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No valid CAN interfaces configured");
        }

        auto motor_driver = hardware_driver::createCanFdMotorDriver(interface_names);

        // 将mapping->motor_id转换为interface->motor_id
        // 仅为有电机的映射创建配置（跳过纯软件映射如夹爪）
        std::map<std::string, std::vector<uint32_t>> interface_motor_config;
        for (const auto& [mapping, motor_ids] : motor_config_) {
            if (!motor_ids.empty()) {  // 仅处理有电机的映射
                std::string interface = get_interface(mapping);
                interface_motor_config[interface] = motor_ids;
            }
        }

        // 创建RobotHardware实例，使用观察者模式
        hardware_driver_ = std::make_shared<RobotHardware>(motor_driver, interface_motor_config,
                                                          shared_from_this());

        // 初始化按键驱动 (不需要独立的总线，通过button_driver转发数据包)
        auto button_driver = hardware_driver::createCanFdButtonDriver(nullptr);
        hardware_driver_->set_button_driver(button_driver);

        // 初始化夹爪驱动（按配置选择型号：omnipicker/pgc/auto）
        std::string gripper_model = "auto";
        for (const auto& [mapping, model_cfg] : gripper_model_config_) {
            if (mapping.find("gripper") == std::string::npos || model_cfg.empty()) {
                continue;
            }
            gripper_model = model_cfg;
            break;
        }
        auto gripper_driver = hardware_driver::createCanFdGripperDriver(interface_names, gripper_model);
        if (gripper_driver) {
            hardware_driver_->set_gripper_driver(gripper_driver);
            RCLCPP_INFO(node_->get_logger(), "✅ Gripper driver initialized");
        } else {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Failed to initialize gripper driver");
        }

        // 创建关节状态发布器
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // 加载关节限位配置
        load_joint_limits_config();

        // 初始化时重置系统状态（对所有mapping）
        for (const auto& [mapping, interface] : mapping_to_interface_) {
            reset_system_health(mapping);
            clear_emergency_stops(mapping);
        }

        RCLCPP_INFO(node_->get_logger(), "✅ HardwareManager initialized successfully");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to initialize HardwareManager: %s", e.what());
        return false;
    }
}

std::shared_ptr<RobotHardware> HardwareManager::get_hardware_driver() const {
    return hardware_driver_;
}

bool HardwareManager::register_motor_recorder(std::shared_ptr<hardware_driver::motor_driver::MotorStatusObserver> recorder) {
    if (!recorder) {
        return false;
    }

    // 保存 recorder 供 on_motor_status_update 转发使用
    motor_recorder_ = recorder;
    return true;
}

bool HardwareManager::unregister_motor_recorder() {
    motor_recorder_.reset();
    return true;
}

bool HardwareManager::is_robot_stopped(const std::string& mapping) const {
    const double velocity_threshold = 0.01; // rad/s

    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    // 从mapping获取对应的joint state
    auto it = mapping_joint_states_.find(mapping);
    const auto& joint_state = it != mapping_joint_states_.end() ?
                             it->second : sensor_msgs::msg::JointState{};

    // 调试：打印velocity大小和内容
    RCLCPP_DEBUG(node_->get_logger(), "[%s] is_robot_stopped check: velocity.size()=%zu",
                 mapping.c_str(), joint_state.velocity.size());

    if (joint_state.velocity.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  is_robot_stopped: velocity vector is EMPTY! Returning TRUE",
                    mapping.c_str());
    } else {
        for (size_t i = 0; i < std::min(size_t(3), joint_state.velocity.size()); ++i) {
            RCLCPP_DEBUG(node_->get_logger(), "[%s] velocity[%zu]=%.6f rad/s",
                         mapping.c_str(), i, joint_state.velocity[i]);
        }
    }

    // 检查该mapping的所有关节速度是否接近零
    for (size_t i = 0; i < joint_state.velocity.size(); ++i) {
        bool velocity_too_high = std::abs(joint_state.velocity[i]) > velocity_threshold;
        if (velocity_too_high) {
            RCLCPP_DEBUG(node_->get_logger(), "[%s] velocity[%zu] too high (%.6f > %.6f), returning FALSE",
                         mapping.c_str(), i, std::abs(joint_state.velocity[i]), velocity_threshold);
            return false;
        }
    }

    return true;
}

bool HardwareManager::are_joints_within_limits(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    
    // 从mapping获取对应的joint state
    auto it = mapping_joint_states_.find(mapping);
    const auto& joint_state = it != mapping_joint_states_.end() ? 
                             it->second : sensor_msgs::msg::JointState{};
    
    const auto& joint_names = get_joint_names(mapping);
    
    // 检查该mapping的每个关节是否在限位内
    for (size_t i = 0; i < joint_names.size() && i < joint_state.position.size(); ++i) {
        const std::string& joint_name = joint_names[i];
        double position = joint_state.position[i];
        double velocity = i < joint_state.velocity.size() ? joint_state.velocity[i] : 0.0;
        
        auto limit_it = joint_limits_config_.find(joint_name);
        const JointLimits& limits = limit_it != joint_limits_config_.end() ? 
                                   limit_it->second : JointLimits{};
        
        // 检查位置限位
        bool position_violation = limits.has_position_limits && 
                                (position < limits.min_position || position > limits.max_position);
        
        bool velocity_violation = limits.has_velocity_limits && 
                                (std::abs(velocity) > limits.max_velocity);
        
        if (position_violation) {
            RCLCPP_WARN(node_->get_logger(),
                       "[%s] Joint %s position %.3f is outside limits [%.3f, %.3f]",
                       mapping.c_str(), joint_name.c_str(), position, 
                       limits.min_position, limits.max_position);
            return false;
        }
        
        if (velocity_violation) {
            RCLCPP_WARN(node_->get_logger(),
                       "[%s] Joint %s velocity %.3f exceeds limit %.3f",
                       mapping.c_str(), joint_name.c_str(), 
                       std::abs(velocity), limits.max_velocity);
            return false;
        }
    }
    
    return true;
}

bool HardwareManager::is_system_healthy(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    // 检查该mapping的健康状态
    auto mapping_health_it = mapping_health_status_.find(mapping);
    bool mapping_healthy = mapping_health_it != mapping_health_status_.end() ? 
                          mapping_health_it->second : true;
    
    bool overall_healthy = system_healthy_ && hardware_driver_ != nullptr && mapping_healthy;

    // 如果系统不健康，打印原因
    if (!overall_healthy) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "[%s] System not healthy - system_healthy_: %s, hardware_driver_: %s, mapping_healthy_: %s",
                             mapping.c_str(),
                             system_healthy_ ? "true" : "false",
                             hardware_driver_ ? "available" : "null",
                             mapping_healthy ? "true" : "false");
    }

    return overall_healthy;
}

// ============= 工具模板 =============
template<typename T>
const T& get_config_value(const std::map<std::string, T>& config_map,
                        const std::string& mapping,
                        const std::string& config_name,
                        rclcpp::Logger logger)
{
    static const T empty_value{};
    auto it = config_map.find(mapping);
    if (it != config_map.end()) {
        return it->second;
    }

    // 不在这里打印错误，让调用者决定是否需要打印
    (void)logger; (void)config_name; // 避免未使用参数警告
    return empty_value;
}

// ============ 配置信息访问 ============
const std::string& HardwareManager::get_interface(const std::string& mapping) const {
    return get_config_value(mapping_to_interface_, mapping, "interface", node_->get_logger());
}

std::string HardwareManager::get_interface_for_mapping(const std::string& mapping) const {
    auto hw_it = mapping_to_interface_.find(mapping);
    if (hw_it != mapping_to_interface_.end()) {
        return hw_it->second;
    }
    auto sw_it = software_mapping_to_interface_.find(mapping);
    if (sw_it != software_mapping_to_interface_.end()) {
        return sw_it->second;
    }
    return "";
}

std::string HardwareManager::get_mapping_by_interface(const std::string& interface) const {
    auto it = interface_to_mapping_.find(interface);
    if (it != interface_to_mapping_.end()) {
        return it->second;
    }
    RCLCPP_WARN(node_->get_logger(), "❎ Interface '%s' not found in configuration", interface.c_str());
    return "";
}

std::vector<std::string> HardwareManager::get_all_mappings() const {
    std::vector<std::string> mappings;
    std::set<std::string> mapping_set;
    for (const auto& [mapping, interface] : mapping_to_interface_) {
        mapping_set.insert(mapping);
    }
    for (const auto& [mapping, interface] : software_mapping_to_interface_) {
        mapping_set.insert(mapping);
    }
    for (const auto& mapping : mapping_set) {
        mappings.push_back(mapping);
    }
    return mappings;
}

const std::vector<uint32_t>& HardwareManager::get_motors_id(const std::string& mapping) const {
    return get_config_value(motor_config_, mapping, "motor", node_->get_logger());
}

const std::vector<std::string>& HardwareManager::get_joint_names(const std::string& mapping) const {
    return get_config_value(joint_names_config_, mapping, "joint_names", node_->get_logger());
}

const std::string& HardwareManager::get_frame_id(const std::string& mapping) const {
    return get_config_value(frame_id_config_, mapping, "frame_id", node_->get_logger());
}

uint8_t HardwareManager::get_joint_count(const std::string& mapping) const {
    auto& joint_names = get_joint_names(mapping);
    return static_cast<uint8_t>(joint_names.size());
}

const std::string& HardwareManager::get_controller_name(const std::string& mapping) const {
    return get_config_value(controller_name_config_, mapping, "controller_name", node_->get_logger());
}

const std::string& HardwareManager::get_planning_group(const std::string& mapping) const {
    return get_config_value(planning_group_config_, mapping, "planning_group", node_->get_logger());
}

const std::string& HardwareManager::get_gripper_model(const std::string& mapping) const {
    return get_config_value(gripper_model_config_, mapping, "gripper_model", node_->get_logger());
}

const std::vector<double>& HardwareManager::get_initial_position(const std::string& mapping) const {
    return get_config_value(initial_position_config_, mapping, "initial_position", node_->get_logger());
}

const std::vector<double>& HardwareManager::get_start_position(const std::string& mapping) const {
    return get_config_value(start_position_config_, mapping, "start_position", node_->get_logger());
}

const std::string& HardwareManager::get_robot_type(const std::string& mapping) const {
    return get_config_value(robot_type_config_, mapping, "robot_type", node_->get_logger());
}

// ============ 轨迹执行 ============
bool HardwareManager::executeTrajectory(const std::string& interface, const trajectory_interpolator::Trajectory& trajectory) {
    if (!hardware_driver_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return false;
    }

    // 通过 interface 获取 mapping
    std::string mapping;
    auto it = interface_to_mapping_.find(interface);
    if (it != interface_to_mapping_.end()) {
        mapping = it->second;
    }

    // 转换 trajectory_interpolator::Trajectory 到 ::Trajectory
    ::Trajectory hw_trajectory;
    hw_trajectory.joint_names = trajectory.joint_names;

    hw_trajectory.points.reserve(trajectory.points.size());
    for (const auto& point : trajectory.points) {
        ::TrajectoryPoint hw_point;
        hw_point.time_from_start = point.time_from_start;
        hw_point.positions = point.positions;
        hw_point.velocities = point.velocities;
        hw_point.accelerations = point.accelerations;

        // 预计算重力矩 (如果有 mapping)
        if (!mapping.empty()) {
            // 位置是度数，需要转换为弧度
            std::vector<double> positions_rad;
            positions_rad.reserve(point.positions.size());
            for (double pos_deg : point.positions) {
                positions_rad.push_back(pos_deg * M_PI / 180.0);
            }
            hw_point.efforts = compute_gravity_torques(mapping, positions_rad);
        }

        hw_trajectory.points.push_back(hw_point);
    }

    return hardware_driver_->execute_trajectory(interface, hw_trajectory);
}

// ============ 关节状态获取和控制 ============
std::vector<double> HardwareManager::get_current_joint_positions(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    auto it = mapping_joint_states_.find(mapping);
    if (it != mapping_joint_states_.end()) {
        return it->second.position;
    }

    RCLCPP_WARN(node_->get_logger(),
                "[%s] Joint state not found, returning empty position vector",
                mapping.c_str());
    return std::vector<double>{};
}


std::vector<double> HardwareManager::get_current_joint_velocities(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    auto it = mapping_joint_states_.find(mapping);
    if (it != mapping_joint_states_.end()) {
        return it->second.velocity;
    }

    RCLCPP_WARN(node_->get_logger(),
                "[%s] Joint state not found, returning empty velocity vector",
                mapping.c_str());
    return std::vector<double>{};
}

std::vector<double> HardwareManager::get_current_joint_efforts(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    auto it = mapping_joint_states_.find(mapping);
    if (it != mapping_joint_states_.end()) {
        return it->second.effort;
    }

    RCLCPP_WARN(node_->get_logger(),
                "[%s] Joint state not found, returning empty effort vector",
                mapping.c_str());
    return std::vector<double>{};
}

double HardwareManager::get_joint_feedback_age_sec(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    auto it = mapping_joint_states_.find(mapping);
    if (it == mapping_joint_states_.end()) {
        return std::numeric_limits<double>::infinity();
    }

    const auto& stamp = it->second.header.stamp;
    if (stamp.nanosec == 0 && stamp.sec == 0) {
        return std::numeric_limits<double>::infinity();
    }

    const auto now = node_->now();
    const auto dt = now - stamp;
    return std::max(0.0, dt.seconds());
}

bool HardwareManager::update_software_joint_state(
    const std::string& mapping,
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    const std::vector<double>& efforts) {
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        auto it = mapping_joint_states_.find(mapping);
        if (it == mapping_joint_states_.end()) {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] Software joint state update failed: mapping not found",
                        mapping.c_str());
            return false;
        }

        auto& joint_state = it->second;
        if (positions.size() != joint_state.name.size()) {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] Software joint state update failed: position size mismatch (%zu != %zu)",
                        mapping.c_str(), positions.size(), joint_state.name.size());
            return false;
        }

        joint_state.header.stamp = node_->now();
        joint_state.position = positions;

        if (!velocities.empty()) {
            if (velocities.size() != joint_state.name.size()) {
                RCLCPP_WARN(node_->get_logger(),
                            "[%s] Software joint state update failed: velocity size mismatch (%zu != %zu)",
                            mapping.c_str(), velocities.size(), joint_state.name.size());
                return false;
            }
            joint_state.velocity = velocities;
        } else {
            joint_state.velocity.assign(joint_state.name.size(), 0.0);
        }

        if (!efforts.empty()) {
            if (efforts.size() != joint_state.name.size()) {
                RCLCPP_WARN(node_->get_logger(),
                            "[%s] Software joint state update failed: effort size mismatch (%zu != %zu)",
                            mapping.c_str(), efforts.size(), joint_state.name.size());
                return false;
            }
            joint_state.effort = efforts;
        } else {
            joint_state.effort.assign(joint_state.name.size(), 0.0);
        }

        // 同步 lock-free 缓存
        auto cache_it = joint_positions_cache_.find(mapping);
        if (cache_it != joint_positions_cache_.end() && cache_it->second.size() == joint_state.position.size()) {
            for (size_t i = 0; i < joint_state.position.size(); ++i) {
                if (cache_it->second[i]) {
                    cache_it->second[i]->store(joint_state.position[i], std::memory_order_release);
                }
            }
        }
    }

    // 在锁外发布，避免阻塞热路径
    publish_joint_state();
    return true;
}

// ✅ Lock-free 版本：用于实时计算线程，避免竞争 joint_state_mutex_
std::vector<double> HardwareManager::get_current_joint_positions_lockfree(const std::string& mapping) const {
    auto it = joint_positions_cache_.find(mapping);
    if (it == joint_positions_cache_.end() || it->second.empty()) {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Joint position cache not found, returning empty vector",
                    mapping.c_str());
        return std::vector<double>{};
    }

    std::vector<double> result;
    result.reserve(it->second.size());
    for (const auto& atomic_ptr : it->second) {
        if (atomic_ptr) {
            result.push_back(atomic_ptr->load(std::memory_order_acquire));
        }
    }
    return result;
}

bool HardwareManager::send_hold_state_command(const std::string& mapping,
                                                  const std::vector<double>& positions) {
    if (!hardware_driver_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return false;
    }

    // 软件映射（如夹爪）没有电机反馈，不应发送 MIT hold 命令
    if (get_motors_id(mapping).empty()) {
        RCLCPP_DEBUG(node_->get_logger(),
                    "[%s] Skip hold-state MIT command for software-only mapping",
                    mapping.c_str());
        return true;
    }

    const std::string interface = get_interface_for_mapping(mapping);
    if (interface.empty() || interface == "unknown_interface") {
        RCLCPP_ERROR(node_->get_logger(),
                    "[%s] ❎ Invalid interface for sending hold position command",
                    mapping.c_str());
        return false;
    }

    // 使用MIT模式发送位置保持命令
    // 位置模式参数: kp=0.05, kd=0.005, effort=0
    std::array<double, 6> positions_deg = {};
    std::array<double, 6> velocities_deg = {};
    std::array<double, 6> efforts = {};
    std::array<double, 6> kps = {};
    std::array<double, 6> kds = {};

    for (size_t i = 0; i < positions.size(); ++i) {
        positions_deg[i] = positions[i] * 180.0 / M_PI;
        velocities_deg[i] = 0.0;
        efforts[i] = compute_gravity_torques(mapping)[i]; // 计算重力补偿力矩
        kps[i] = 0.05;
        kds[i] = 0.005;
    }

    bool success = hardware_driver_->send_realtime_mit_command(interface, positions_deg, velocities_deg, efforts, kps, kds);

    if (!success) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "[%s] ❎ Failed to send hold position command",
                            mapping.c_str());
    }

    return success;
}

void HardwareManager::on_motor_status_update(const std::string& interface,
                                            uint32_t motor_id,
                                            const hardware_driver::motor_driver::Motor_Status& status) {
    // ✅ 转发给录制观察者（如果已注册）
    if (motor_recorder_) {
        motor_recorder_->on_motor_status_update(interface, motor_id, status);
    }

    update_joint_state(interface, motor_id, status);

    // 实时安全检查
    if (safety_enabled_) {
        check_safety_limits(interface, motor_id, status);
    }

    // ✅ CRITICAL FIX: 在锁外发布！
    // publish_joint_state() 会尝试获取 joint_state_mutex_，
    // 而 update_joint_state() 已经释放了锁，所以这里是安全的
    publish_joint_state();
}

void HardwareManager::update_joint_state(const std::string& interface, uint32_t motor_id,
                                        const hardware_driver::motor_driver::Motor_Status& status) {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    // 通过interface查找对应的mapping
    auto mapping_it = interface_to_mapping_.find(interface);
    if (mapping_it == interface_to_mapping_.end()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Unknown interface: %s", interface.c_str());
        return;
    }

    const std::string& mapping = mapping_it->second;
    const auto& motor_ids = get_motors_id(mapping);

    // 查找motor_id在该mapping中的位置
    auto motor_it = std::find(motor_ids.begin(), motor_ids.end(), motor_id);
    if (motor_it == motor_ids.end()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Motor ID %u not found in mapping %s", motor_id, mapping.c_str());
        return;
    }

    int local_index = std::distance(motor_ids.begin(), motor_it);

    // 确保该mapping的JointState已初始化
    auto joint_state_it = mapping_joint_states_.find(mapping);
    if (joint_state_it == mapping_joint_states_.end()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                             "Joint state not initialized for mapping %s", mapping.c_str());
        return;
    }

    // 更新该mapping的关节状态
    sensor_msgs::msg::JointState& joint_state = joint_state_it->second;
    joint_state.header.stamp = node_->now();

    // 转换单位：度数 → 弧度 (硬件返回度数，ROS需要弧度)
    double position_rad = status.position * M_PI / 180.0;
    joint_state.position[local_index] = position_rad;
    joint_state.velocity[local_index] = status.velocity * M_PI / 180.0;

    joint_state.effort[local_index] = status.effort;

    // ✅ CRITICAL FIX: 同时更新 lock-free 缓存（供计算线程读取，避免竞争）
    auto cache_it = joint_positions_cache_.find(mapping);
    if (cache_it != joint_positions_cache_.end() && local_index < static_cast<int>(cache_it->second.size())) {
        if (cache_it->second[local_index]) {
            cache_it->second[local_index]->store(position_rad, std::memory_order_release);
        }
    }

    // 记录最新温度用于调试
    std::string motor_key = mapping + "_motor" + std::to_string(motor_id);
    motor_temperatures_[motor_key] = status.temperature;

    // 每20条状态更新才执行一次系统健康检查
    if (++health_check_counter_ >= 20) {
        health_check_counter_ = 0;
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        system_healthy_ = (status.temperature < 850); // 简单的健康检查, 温度低于85℃认为正常
    }
}

void HardwareManager::publish_joint_state() {
    if (!joint_state_pub_) return;

    // ✅ CRITICAL FIX: 分离数据复制和发布操作
    // 1. 在锁内复制数据（快速操作）
    sensor_msgs::msg::JointState combined_state;
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        combined_state.header.stamp = node_->now();
        combined_state.header.frame_id = "world";  // 全局坐标系

        // 遍历所有 mapping，合并关节名称和状态
        for (const auto& [mapping, joint_state] : mapping_joint_states_) {
            // 添加该mapping的所有关节名称、位置、速度、力矩
            for (size_t i = 0; i < joint_state.name.size(); ++i) {
                combined_state.name.push_back(joint_state.name[i]);
                if (i < joint_state.position.size()) {
                    combined_state.position.push_back(joint_state.position[i]);
                }
                if (i < joint_state.velocity.size()) {
                    combined_state.velocity.push_back(joint_state.velocity[i]);
                }
                if (i < joint_state.effort.size()) {
                    combined_state.effort.push_back(joint_state.effort[i]);
                }
            }
        }
    }  // ✅ 锁释放

    // 2. 在锁外发布（可能阻塞，但不持有关键锁）
    if (!combined_state.name.empty()) {
        joint_state_pub_->publish(combined_state);
    }
}

// =========== 配置文件加载 ===========
bool HardwareManager::load_joint_limits_config() {
    try {
        if (mapping_to_interface_.empty()) {
            RCLCPP_WARN_ONCE(node_->get_logger(), "No mapping available before loading joint limits.");
            return false;
        }
        // 清空旧配置
        joint_limits_config_.clear();

        // 为每个 mapping 加载对应的 joint limit 文件
        for (const auto& [mapping_name, interface] : mapping_to_interface_) {
            const std::string& robot_type = get_robot_type(mapping_name);
            bool robot_type_undefined = robot_type.empty() || robot_type == "unknown_robot";
            
            if (robot_type_undefined) {
                RCLCPP_WARN_ONCE(node_->get_logger(), "[%s] robot_type undefined, skipping joint limits.", mapping_name.c_str());
                continue;
            }

            // 拼接文件路径
            std::string pkg_path = ament_index_cpp::get_package_share_directory("arm_controller");
            std::string config_file = pkg_path + "/config/" + robot_type + "_joint_limits.yaml";

            RCLCPP_DEBUG(node_->get_logger(), "[%s] Loading joint limits: %s", mapping_name.c_str(), config_file.c_str());
            YAML::Node config = YAML::LoadFile(config_file);

            auto joint_limits_node = config["joint_limits"];
            bool missing_joint_limits = !joint_limits_node;
            
            if (missing_joint_limits) {
                RCLCPP_ERROR_ONCE(node_->get_logger(), "❎ [%s] Missing 'joint_limits' section in %s",
                                 mapping_name.c_str(), config_file.c_str());
                continue;
            }

            // 解析 joint_limits 节点
            for (const auto& joint : joint_limits_node) {
                std::string joint_name = joint.first.as<std::string>();
                const YAML::Node& limits = joint.second;
                JointLimits jl;

                jl.has_position_limits      = limits["has_position_limits"] ? limits["has_position_limits"].as<bool>() : false;
                jl.min_position             = limits["min_position"]        ? limits["min_position"].as<double>() : -3.14;
                jl.max_position             = limits["max_position"]        ? limits["max_position"].as<double>() : 3.14;

                jl.has_velocity_limits      = limits["has_velocity_limits"] ? limits["has_velocity_limits"].as<bool>() : false;
                jl.max_velocity             = limits["max_velocity"]        ? limits["max_velocity"].as<double>() : 3.14;

                jl.has_acceleration_limits  = limits["has_acceleration_limits"] ? limits["has_acceleration_limits"].as<bool>() : false;
                jl.max_acceleration         = limits["max_acceleration"]        ? limits["max_acceleration"].as<double>() : 3.14;

                joint_limits_config_[joint_name] = jl;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "✅ All joint limits loaded successfully.");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to load joint limits config: %s", e.what());
        return false;
    }
}

bool HardwareManager::load_hardware_config() {
    try {
        // 1. 加载配置文件
        std::string package_path = ament_index_cpp::get_package_share_directory("arm_controller");
        std::string config_file = package_path + "/config/hardware_config.yaml";
        YAML::Node config = YAML::LoadFile(config_file);

        // 2. 读取hardware节点
        auto hardware_node = config["hardware"];
        if (!hardware_node) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Missing 'hardware' section in YAML.");
            return false;
        }

        // 3. 清空旧配置
        clear_mappings();

        // 4. 第一阶段：解析所有配置但不注册 gravity mapping，同时加载 URDF
        std::map<std::string, YAML::Node> mapping_configs;
        for (auto it = hardware_node.begin(); it != hardware_node.end(); ++it) {
            std::string key = it->first.as<std::string>();

            std::string mapping_name;
            if (key.find("_mapping") != std::string::npos) {
                mapping_name = key.substr(0, key.find("_mapping"));
            } else {
                mapping_name = key;
            }

            mapping_configs[mapping_name] = it->second;

            // 在第一阶段就加载 URDF（只加载一次）
            if (!parse_mapping(mapping_name, it->second, true)) {  // true = skip_gravity_registration
                RCLCPP_ERROR(node_->get_logger(), "❎ Failed to parse mapping: %s", mapping_name.c_str());
                return false;
            }
        }

        // 5. 第二阶段：为所有 mapping 注册 gravity compensation
        // 首先确保重力补偿器已创建并加载了 URDF
        if (!gravity_compensator_) {
            RCLCPP_INFO(node_->get_logger(), "Creating GravityCompensator in second phase...");
            gravity_compensator_ = std::make_shared<arm_controller::dynamics::GravityCompensator>();

            // 使用第一阶段加载的 URDF 路径
            if (!loaded_urdf_path_.empty()) {
                RCLCPP_INFO(node_->get_logger(), "Loading URDF from first phase: %s", loaded_urdf_path_.c_str());
                if (!gravity_compensator_->loadUrdf(loaded_urdf_path_)) {
                    RCLCPP_WARN(node_->get_logger(), "Failed to load URDF in second phase: %s", loaded_urdf_path_.c_str());
                } else {
                    RCLCPP_INFO(node_->get_logger(), "✅ URDF loaded in second phase: %s", loaded_urdf_path_.c_str());
                }
            } else {
                RCLCPP_WARN(node_->get_logger(), "No URDF path available from first phase");
            }
        }

        for (const auto& [mapping_name, mapping_node] : mapping_configs) {
            // 仅为有电机的 mapping 注册重力补偿
            const auto& motors = motor_config_[mapping_name];
            if (!motors.empty() && mapping_node["joint_names"]) {
                std::vector<std::string> joint_names = mapping_node["joint_names"].as<std::vector<std::string>>();
                if (gravity_compensator_ && !gravity_compensator_->registerMapping(mapping_name, joint_names)) {
                    RCLCPP_WARN(node_->get_logger(), "[%s] Failed to register gravity mapping", mapping_name.c_str());
                } else if (gravity_compensator_) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] Gravity mapping registered with %zu joints",
                        mapping_name.c_str(), joint_names.size());
                }
            } else if (motors.empty()) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Skipping gravity mapping for software-only mapping (gripper mass included in parent arm)", mapping_name.c_str());
            }
            initialize_joint_state(mapping_name);
        }

        RCLCPP_INFO(node_->get_logger(), "✅ Hardware config loaded successfully.");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node_->get_logger(), "❎ Failed to load hardware config: %s", e.what());
        rclcpp::shutdown();
        return false;
    }
}

void HardwareManager::clear_mappings() {
    mapping_to_interface_.clear();
    software_mapping_to_interface_.clear();
    interface_to_mapping_.clear();
    mapping_joint_states_.clear();

    motor_config_.clear();
    joint_names_config_.clear();
    controller_name_config_.clear();
    planning_group_config_.clear();
    gripper_model_config_.clear();
    frame_id_config_.clear();
    initial_position_config_.clear();
    start_position_config_.clear();
    joint_limits_config_.clear();
    robot_type_config_.clear();

    // 重置重力补偿计算器
    gravity_compensator_.reset();
    loaded_urdf_path_.clear();
}

bool HardwareManager::parse_mapping(const std::string& mapping_name, const YAML::Node& mapping_node, bool skip_gravity_registration) {
    RCLCPP_INFO(node_->get_logger(), "[PARSE_MAPPING] Starting for mapping: %s (skip_gravity=%d)",
                mapping_name.c_str(), skip_gravity_registration);

    if (!mapping_node) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Mapping node is null for '%s'", mapping_name.c_str());
        return false;
    }
    // ===== 机器人类型 =====
    robot_type_config_[mapping_name] =
        mapping_node["robot_type"] ? mapping_node["robot_type"].as<std::string>() : "unknown_robot";

    // ===== 电机ID列表 =====
    motor_config_[mapping_name] =
        mapping_node["motors"] ? mapping_node["motors"].as<std::vector<uint32_t>>() : std::vector<uint32_t>{};

    // ===== 接口映射 (仅当有电机时) =====
    // 如果 motors 列表为空，说明这是一个纯软件 mapping（如无反馈的夹爪），不需要硬件初始化
    std::string interface =
        mapping_node["interface"] ? mapping_node["interface"].as<std::string>() : "unknown_interface";

    if (!motor_config_[mapping_name].empty()) {
        // 有电机，添加到硬件映射
        mapping_to_interface_[mapping_name] = interface;
        interface_to_mapping_[interface] = mapping_name;
    } else {
        // 无电机，仅用于软件层面（如发布 joint_state）
        software_mapping_to_interface_[mapping_name] = interface;
        RCLCPP_INFO(node_->get_logger(), "[%s] No motors configured - pure software mapping (no hardware initialization)",
                    mapping_name.c_str());
    }

    // ===== 关节名称列表 =====
    joint_names_config_[mapping_name] =
        mapping_node["joint_names"] ? mapping_node["joint_names"].as<std::vector<std::string>>() : std::vector<std::string>{};

    // ===== 控制器名称 =====
    controller_name_config_[mapping_name] =
        mapping_node["controller_name"] ? mapping_node["controller_name"].as<std::string>() : "";

    // ===== 规划组名称 =====
    planning_group_config_[mapping_name] =
        mapping_node["planning_group"] ? mapping_node["planning_group"].as<std::string>() : "";

    // ===== 夹爪型号 =====
    // 优先读取显式 gripper_model；未配置时从 robot_type 进行推断
    if (mapping_node["gripper_model"]) {
        gripper_model_config_[mapping_name] = mapping_node["gripper_model"].as<std::string>();
    } else {
        std::string inferred_model = "auto";
        const std::string robot_type = robot_type_config_[mapping_name];
        if (robot_type.find("pgc") != std::string::npos || robot_type.find("PGC") != std::string::npos) {
            inferred_model = "pgc";
        } else if (robot_type.find("omnipicker") != std::string::npos || robot_type.find("OmniPicker") != std::string::npos) {
            inferred_model = "omnipicker";
        }
        gripper_model_config_[mapping_name] = inferred_model;
    }

    // ===== 坐标系 =====
    frame_id_config_[mapping_name] =
        mapping_node["frame_id"] ? mapping_node["frame_id"].as<std::string>() : "unknown_frame_id";

    // ===== 初始位置 =====
    initial_position_config_[mapping_name] =
        mapping_node["initial_position"] ? mapping_node["initial_position"].as<std::vector<double>>() : std::vector<double>{};

    // ===== 启动位置 =====
    start_position_config_[mapping_name] =
        mapping_node["start_position"] ? mapping_node["start_position"].as<std::vector<double>>() : std::vector<double>{};

    // 获取接口名称（如果不在mapping_to_interface_中则使用配置中的值）
    // 对于纯软件映射（无电机），interface会被配置但不会添加到mapping_to_interface_
    auto interface_it = mapping_to_interface_.find(mapping_name);
    std::string interface_display = (interface_it != mapping_to_interface_.end()) ? interface_it->second : interface;

    RCLCPP_INFO(node_->get_logger(), "✅ Loaded mapping [%s]: interface=%s, controller=%s, group=%s, frame_id=%s, joints=%zu",
        mapping_name.c_str(),
        interface_display.c_str(),
        controller_name_config_[mapping_name].c_str(),
        planning_group_config_[mapping_name].c_str(),
        frame_id_config_[mapping_name].c_str(),
        joint_names_config_[mapping_name].size()
    );

    // ===== URDF路径 (根据robot_type自动查找) =====
    std::string urdf_path;
    try {
        std::string robot_desc_path = ament_index_cpp::get_package_share_directory("robot_description");
        urdf_path = robot_desc_path + "/urdf/" + robot_type_config_[mapping_name] + ".urdf";
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Could not find robot_description package: %s",
            mapping_name.c_str(), e.what());
    }

    // ===== 加载重力补偿模型 URDF (只在第一阶段加载) =====
    // 第一阶段：记录 URDF 路径，供第二阶段使用
    // 第二阶段：使用 URDF 路径创建和初始化重力补偿器
    if (!urdf_path.empty()) {
        if (loaded_urdf_path_ != urdf_path) {
            RCLCPP_INFO(node_->get_logger(), "[%s] URDF path: %s", mapping_name.c_str(), urdf_path.c_str());
            loaded_urdf_path_ = urdf_path;
        }
    }

    return true;
}

void HardwareManager::initialize_joint_state(const std::string& mapping_name) {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.frame_id = frame_id_config_[mapping_name];
    joint_state.name = joint_names_config_[mapping_name];
    joint_state.position = initial_position_config_[mapping_name];
    joint_state.velocity.resize(joint_names_config_[mapping_name].size(), 0.0);
    joint_state.effort.resize(joint_names_config_[mapping_name].size(), 0.0);

    mapping_joint_states_[mapping_name] = joint_state;

    // ✅ 同时初始化 lock-free 缓存（用于计算线程，避免竞争）
    {
        std::lock_guard<std::mutex> lock(cache_init_mutex_);
        std::vector<std::shared_ptr<std::atomic<double>>> cache;
        // 为每个关节位置创建一个独立的 atomic<double>
        for (double pos : joint_state.position) {
            cache.push_back(std::make_shared<std::atomic<double>>(pos));
        }
        joint_positions_cache_[mapping_name] = cache;
    }
}

const std::map<std::string, JointLimits>& HardwareManager::get_joint_limits() const {
    return joint_limits_config_;
}

void HardwareManager::get_joint_limits(const std::string& joint_name, JointLimits& limits) {
    auto it = joint_limits_config_.find(joint_name);
    limits = it != joint_limits_config_.end() ? it->second : JointLimits{};
}

void HardwareManager::check_safety_limits(const std::string& interface, uint32_t motor_id,
                                         const hardware_driver::motor_driver::Motor_Status& status) {
    // ⚠️ 示教模式下跳过安全限位检查
    if (teaching_mode_enabled_) {
        return;
    }

    // 通过interface查找对应的mapping
    auto mapping_it = interface_to_mapping_.find(interface);
    if (mapping_it == interface_to_mapping_.end()) {
        return;
    }
    
    const std::string& mapping = mapping_it->second;
    const auto& joint_names = get_joint_names(mapping);
    const auto& motor_ids = get_motors_id(mapping);
    
    // 查找motor_id在该mapping中的位置
    auto motor_it = std::find(motor_ids.begin(), motor_ids.end(), motor_id);
    if (motor_it == motor_ids.end()) {
        return;
    }
    
    int joint_index = std::distance(motor_ids.begin(), motor_it);
    if (joint_index >= static_cast<int>(joint_names.size())) {
        return;
    }
    
    const std::string& joint_name = joint_names[joint_index];

    // 转换单位：度数 → 弧度
    double position = status.position * M_PI / 180.0;
    double velocity = status.velocity * M_PI / 180.0;

    // 获取位置限位
    JointLimits limits;
    get_joint_limits(joint_name, limits);

    bool safety_violation = false;
    std::string violation_reason;
    int violation_dir = 0;

    constexpr double SAFETY_LOOKAHEAD_TIME = 0.2; // 秒
    double predicted_position = position + velocity * SAFETY_LOOKAHEAD_TIME;

    // 检查位置限位
    bool near_min_limit = limits.has_position_limits && (position <= limits.min_position + POSITION_MARGIN);
    bool near_max_limit = limits.has_position_limits && (position >= limits.max_position - POSITION_MARGIN);
    // bool hard_limit_exceeded = limits.has_position_limits &&
    //                            (position < limits.min_position || position > limits.max_position);

    // 提前预测急停
    if (limits.has_position_limits) {
        if (predicted_position < limits.min_position) {
            safety_violation = true;
            violation_dir = -1;
            violation_reason = "approaching min position limit (predicted)";
        } else if (predicted_position > limits.max_position) {
            safety_violation = true;
            violation_dir = 1;
            violation_reason = "approaching max position limit (predicted)";
        }
    }

    // 检查速度限位
    bool velocity_limit_exceeded = limits.has_velocity_limits &&
                                   (std::abs(velocity) > limits.max_velocity);
    if (velocity_limit_exceeded) {
        safety_violation = true;
        violation_dir = (velocity > 0) ? 1 : -1;
        violation_reason = "velocity limit exceeded";
    }

    bool currently_emergency = joint_emergency_stop_[joint_name];

    // 未触发急停但接近软限位，打印预警
    if (!currently_emergency && (near_min_limit || near_max_limit)) {
        // RCLCPP_WARN(node_->get_logger(),
        //             "[%s] Joint %s is near position limit: %.3f rad",
        //             mapping.c_str(), joint_name.c_str(), position);
    }

    if (safety_violation) {
        if (!currently_emergency) {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] ❗ Safety violation detected for joint %s (motor %u): %s. Emergency stopping motor.",
                        mapping.c_str(), joint_name.c_str(), motor_id, violation_reason.c_str());

            emergency_stop_joint(interface, motor_id);
        }
        joint_emergency_stop_[joint_name] = true;
        joint_violation_direction_[joint_name] = violation_dir;
    } else if (currently_emergency) {
        // 关节处于紧急停止状态时，只有回到安全区域才解除急停
        if (position > limits.min_position + POSITION_MARGIN &&
            position < limits.max_position - POSITION_MARGIN) {
            joint_emergency_stop_[joint_name] = false;
            joint_violation_direction_[joint_name] = 0;
            RCLCPP_INFO(node_->get_logger(),
                        "[%s] Joint %s has recovered from safety violation. Clearing emergency stop.",
                        mapping.c_str(), joint_name.c_str());
        }
    }
}

bool HardwareManager::is_joint_emergency_stopped(const std::string& joint_name) const {
    auto it = joint_emergency_stop_.find(joint_name);
    return (it != joint_emergency_stop_.end() && it->second);
}

int HardwareManager::get_joint_violation_direction(const std::string& joint_name) const {
    auto it = joint_violation_direction_.find(joint_name);
    return (it != joint_violation_direction_.end()) ? it->second : 0;
}

void HardwareManager::emergency_stop_joint(const std::string& interface, uint32_t motor_id) {
    bool hardware_not_available = !hardware_driver_;
    
    if (hardware_not_available) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not available for emergency stop");
        return;
    }

    try {
        // TODO: 🌟 修改急停策略, hardware_driver提供一个急停的方法, 当前发送零速度命令来停止电机
        hardware_driver_->control_motor_in_mit_mode(interface, motor_id, 0.0, 0.0, 0.0, 0.0, 0.01);

        RCLCPP_WARN(node_->get_logger(),
                   "Emergency stop executed for motor %u on interface %s",
                   motor_id, interface.c_str());

        // 更新系统健康状态
        std::lock_guard<std::mutex> lock(status_mutex_);
        system_healthy_ = false;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                    "❎ Failed to execute emergency stop for motor %u: %s",
                    motor_id, e.what());
    }
}

void HardwareManager::print_system_status() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    std::lock_guard<std::mutex> joint_lock(joint_state_mutex_);

    RCLCPP_INFO(node_->get_logger(), "─────────────── SYSTEM STATUS ───────────────");
    RCLCPP_INFO(node_->get_logger(), "Health: %s | Safety: %s | Hardware: %s",
                system_healthy_ ? "OK" : "FAULT",
                safety_enabled_ ? "ON" : "OFF",
                hardware_driver_ ? "READY" : "MISSING");


    bool has_emergency_stops = !joint_emergency_stop_.empty();
    
    if (has_emergency_stops) {
        RCLCPP_WARN(node_->get_logger(), "Active emergency stops:");
        for (auto& [joint, active] : joint_emergency_stop_) {
            if (active) RCLCPP_WARN(node_->get_logger(), " - %s", joint.c_str());
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Motor temperatures (°C):");
    for (auto& [key, temp_raw] : motor_temperatures_) {
        double temp_c = temp_raw / 10.0;
        RCLCPP_INFO(node_->get_logger(), " - %-15s : %.1f°C", key.c_str(), temp_c);
    }

    RCLCPP_INFO(node_->get_logger(), "─────────────── END STATUS ───────────────");
}

void HardwareManager::reset_system_health(const std::string& mapping) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    // 重置全局健康状态
    system_healthy_ = true;
    
    // 重置该mapping的健康状态
    mapping_health_status_[mapping] = true;
    
    RCLCPP_DEBUG(node_->get_logger(), "✅ [%s] System health reset", mapping.c_str());
}

void HardwareManager::clear_emergency_stops(const std::string& mapping) {
    joint_emergency_stop_[mapping] = false;
}

bool HardwareManager::enable_motors(const std::string& mapping, uint8_t mode) {
    if (!hardware_driver_) {
        RCLCPP_ERROR(node_->get_logger(), "Hardware driver not initialized");
        return false;
    }

    try {
        const std::string& interface = get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = get_motors_id(mapping);

        // 检查mapping是否有效
        if (interface.empty() || motor_ids.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Mapping '%s' not found in configuration", mapping.c_str());
            return false;
        }

        hardware_driver_->enable_motors(interface, motor_ids, mode);

        // 重置系统安全状态
        clear_emergency_stops(mapping);
        reset_system_health(mapping);

        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ All motors enabled successfully (mode: %u)", mapping.c_str(), mode);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Failed to enable motors: %s", mapping.c_str(), e.what());
        return false;
    }
}

bool HardwareManager::disable_motors(const std::string& mapping, uint8_t mode) {
    if (!hardware_driver_) {
        RCLCPP_ERROR(node_->get_logger(), "Hardware driver not initialized");
        return false;
    }

    try {
        const std::string& interface = get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = get_motors_id(mapping);

        // 检查mapping是否有效
        if (interface.empty() || motor_ids.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Mapping '%s' not found in configuration", mapping.c_str());
            return false;
        }

        hardware_driver_->disable_motors(interface, motor_ids, mode);

        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ All motors disabled successfully (mode: %u)", mapping.c_str(), mode);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Failed to disable motors: %s", mapping.c_str(), e.what());
        return false;
    }
}

// ============= 异步轨迹执行和控制接口实现 =============

std::string HardwareManager::execute_trajectory_async(
    const std::string& mapping,
    const Trajectory& trajectory,
    bool show_progress) {
    try {
        // 检查 hardware_driver_ 是否初始化
        if (!hardware_driver_) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
            return "";
        }

        if (mapping_to_interface_.find(mapping) == mapping_to_interface_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Mapping '%s' not found", mapping.c_str());
            return "";
        }

        const std::string& interface = mapping_to_interface_.at(mapping);

        // 预计算每个轨迹点的重力矩
        Trajectory trajectory_with_efforts = trajectory;
        for (size_t idx = 0; idx < trajectory_with_efforts.points.size(); ++idx) {
            auto& point = trajectory_with_efforts.points[idx];
            // 使用轨迹点的位置计算重力矩 (位置是度数，需要转换为弧度)
            std::vector<double> positions_rad;
            positions_rad.reserve(point.positions.size());
            for (double pos_deg : point.positions) {
                positions_rad.push_back(pos_deg * M_PI / 180.0);
            }
            point.efforts = compute_gravity_torques(mapping, positions_rad);
        }

        // 调用硬件驱动的异步执行方法
        std::string execution_id = hardware_driver_->execute_trajectory_async(interface, trajectory_with_efforts, show_progress);

        if (!execution_id.empty()) {
            // 记录 mapping -> execution_id 的对应关系
            {
                std::lock_guard<std::mutex> lock(execution_mutex_);
                mapping_to_execution_id_[mapping] = execution_id;
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Failed to start trajectory execution", mapping.c_str());
        }

        return execution_id;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception during trajectory execution: %s", mapping.c_str(), e.what());
        return "";
    }
}

bool HardwareManager::pause_trajectory(const std::string& mapping) {
    try {
        if (!hardware_driver_) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
            return false;
        }

        // 获取该 mapping 当前的 execution_id
        std::string execution_id;
        {
            std::lock_guard<std::mutex> lock(execution_mutex_);
            auto it = mapping_to_execution_id_.find(mapping);
            if (it == mapping_to_execution_id_.end()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  No active trajectory execution found for mapping", mapping.c_str());
                return false;
            }
            execution_id = it->second;
        }

        bool success = hardware_driver_->pause_trajectory(execution_id);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Trajectory paused (ID: %s)", mapping.c_str(), execution_id.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  Failed to pause trajectory (ID: %s)", mapping.c_str(), execution_id.c_str());
        }
        return success;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception during trajectory pause: %s", mapping.c_str(), e.what());
        return false;
    }
}

bool HardwareManager::resume_trajectory(const std::string& mapping) {
    try {
        if (!hardware_driver_) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
            return false;
        }

        // 获取该 mapping 当前的 execution_id
        std::string execution_id;
        {
            std::lock_guard<std::mutex> lock(execution_mutex_);
            auto it = mapping_to_execution_id_.find(mapping);
            if (it == mapping_to_execution_id_.end()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  No active trajectory execution found for mapping", mapping.c_str());
                return false;
            }
            execution_id = it->second;
        }

        bool success = hardware_driver_->resume_trajectory(execution_id);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Trajectory resumed (ID: %s)", mapping.c_str(), execution_id.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  Failed to resume trajectory (ID: %s)", mapping.c_str(), execution_id.c_str());
        }
        return success;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception during trajectory resume: %s", mapping.c_str(), e.what());
        return false;
    }
}

bool HardwareManager::cancel_trajectory(const std::string& mapping) {
    try {
        if (!hardware_driver_) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
            return false;
        }

        // 获取该 mapping 当前的 execution_id
        std::string execution_id;
        {
            std::lock_guard<std::mutex> lock(execution_mutex_);
            auto it = mapping_to_execution_id_.find(mapping);
            if (it == mapping_to_execution_id_.end()) {
                return false;
            }
            execution_id = it->second;
        }

        bool success = hardware_driver_->cancel_trajectory(execution_id);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Trajectory cancelled (ID: %s)", mapping.c_str(), execution_id.c_str());

            // 清理执行ID映射
            {
                std::lock_guard<std::mutex> lock(execution_mutex_);
                mapping_to_execution_id_.erase(mapping);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  Failed to cancel trajectory (ID: %s)", mapping.c_str(), execution_id.c_str());
        }
        return success;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception during trajectory cancel: %s", mapping.c_str(), e.what());
        return false;
    }
}

double HardwareManager::get_execution_progress(const std::string& execution_id) {
    try {
        if (!hardware_driver_) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
            return -1.0;
        }

        // 从硬件驱动查询执行进度
        TrajectoryExecutionProgress progress;
        if (hardware_driver_->get_execution_progress(execution_id, progress)) {
            // 计算进度百分比 [0, 1]
            if (progress.total_points > 0) {
                double progress_ratio = static_cast<double>(progress.current_point_index) / progress.total_points;
                return std::clamp(progress_ratio, 0.0, 1.0);
            }
            return 0.0;
        } else {
            // 查询失败（执行可能已完成或出错）
            return -1.0;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️  Failed to get execution progress for execution_id %s: %s",
                   execution_id.c_str(), e.what());
        return -1.0;
    }
}

bool HardwareManager::wait_for_trajectory_completion(const std::string& mapping, int timeout_ms) {
    try {
        if (!hardware_driver_) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
            return false;
        }

        // 获取该 mapping 当前的 execution_id
        std::string execution_id;
        {
            std::lock_guard<std::mutex> lock(execution_mutex_);
            auto it = mapping_to_execution_id_.find(mapping);
            if (it == mapping_to_execution_id_.end()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  No active trajectory execution found for mapping - trajectory may not have been started", mapping.c_str());
                return false;  // 改为返回 false：没有活跃轨迹意味着轨迹未开始或已被清理
            }
            execution_id = it->second;
        }

        // 等待轨迹执行完成
        bool completed = hardware_driver_->wait_for_completion(execution_id, timeout_ms);

        if (completed) {
            // 清理执行ID映射
            std::lock_guard<std::mutex> lock(execution_mutex_);
            mapping_to_execution_id_.erase(mapping);
        }

        return completed;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Exception waiting for trajectory completion: %s", e.what());
        return false;
    }
}

// ============= 重力矩计算实现 =============

std::vector<double> HardwareManager::compute_gravity_torques(const std::string& mapping) {
    // 获取当前关节位置并计算重力矩
    std::vector<double> joint_positions = get_current_joint_positions(mapping);
    return compute_gravity_torques(mapping, joint_positions);
}

std::vector<double> HardwareManager::compute_gravity_torques(const std::string& mapping, const std::vector<double>& joint_positions) {
    // 检查重力补偿计算器是否存在
    if (!gravity_compensator_) {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] gravity_compensator_ is null, returning zero torques", mapping.c_str());
        return std::vector<double>(joint_positions.size(), 0.0);
    }

    if (!gravity_compensator_->hasMapping(mapping)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[%s] Gravity mapping not registered, returning zero torques", mapping.c_str());
        return std::vector<double>(joint_positions.size(), 0.0);
    }

    // 检查关节数量是否匹配
    size_t expected_dof = gravity_compensator_->getDof(mapping);
    if (joint_positions.size() != expected_dof) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[%s] Joint position size mismatch: expect %zu, got %zu",
            mapping.c_str(), expected_dof, joint_positions.size());
        return std::vector<double>(joint_positions.size(), 0.0);
    }

    // 调用 GravityCompensator 计算重力矩
    RCLCPP_DEBUG(node_->get_logger(), "[%s] Computing gravity torques for %zu joints",
                 mapping.c_str(), joint_positions.size());
    std::vector<double> gravity_torques = gravity_compensator_->computeGravity(mapping, joint_positions);

    if (gravity_torques.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[%s] Failed to compute gravity torques", mapping.c_str());
        return std::vector<double>(joint_positions.size(), 0.0);
    }

    return gravity_torques;
}
