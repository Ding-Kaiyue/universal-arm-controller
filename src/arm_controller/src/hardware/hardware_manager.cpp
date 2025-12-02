#include "arm_controller/hardware/hardware_manager.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

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
        for (const auto& [mapping, interface] : mapping_to_interface_) {
            interface_names.push_back(interface);
        }

        auto motor_driver = hardware_driver::createCanFdMotorDriver(interface_names);

        // 将mapping->motor_id转换为interface->motor_id
        std::map<std::string, std::vector<uint32_t>> interface_motor_config;
        for (const auto& [mapping, motor_ids] : motor_config_) {
            std::string interface = get_interface(mapping);
            interface_motor_config[interface] = motor_ids;
        }

        // 创建RobotHardware实例，使用观察者模式
        hardware_driver_ = std::make_shared<RobotHardware>(motor_driver, interface_motor_config,
                                                          shared_from_this());

        // 初始化按键驱动 (不需要独立的总线，通过motor_driver转发数据包)
        auto button_driver = hardware_driver::createCanFdButtonDriver(nullptr);
        hardware_driver_->set_button_driver(button_driver);
        RCLCPP_INFO(node_->get_logger(), "按键驱动初始化完成");

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


bool HardwareManager::is_robot_stopped(const std::string& mapping) const {
    const double velocity_threshold = 0.01; // rad/s
    
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    
    // 从mapping获取对应的joint state
    auto it = mapping_joint_states_.find(mapping);
    const auto& joint_state = it != mapping_joint_states_.end() ? 
                             it->second : sensor_msgs::msg::JointState{};
    
    // 检查该mapping的所有关节速度是否接近零
    for (size_t i = 0; i < joint_state.velocity.size(); ++i) {
        bool velocity_too_high = std::abs(joint_state.velocity[i]) > velocity_threshold;
        if (velocity_too_high) return false;
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

std::vector<std::string> HardwareManager::get_all_mappings() const {
    std::vector<std::string> mappings;
    for (const auto& [mapping, interface] : mapping_to_interface_) {
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

bool HardwareManager::send_hold_position_command(const std::string& mapping,
                                                  const std::vector<double>& positions) {
    if (!hardware_driver_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return false;
    }

    const std::string& interface = get_interface(mapping);
    if (interface.empty() || interface == "unknown_interface") {
        RCLCPP_ERROR(node_->get_logger(),
                    "[%s] ❎ Invalid interface for sending hold position command",
                    mapping.c_str());
        return false;
    }

    // 检查关节数量，最多支持6个关节
    if (positions.size() > 6) {
        RCLCPP_ERROR(node_->get_logger(),
                    "[%s] ❎ Too many joints (%zu), maximum is 6",
                    mapping.c_str(), positions.size());
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
        efforts[i] = 0.0;
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

bool HardwareManager::send_hold_velocity_command(const std::string& mapping) {
    if (!hardware_driver_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return false;
    }

    const std::string& interface = get_interface(mapping);
    if (interface.empty() || interface == "unknown_interface") {
        RCLCPP_ERROR(node_->get_logger(),
                    "[%s] ❎ Invalid interface for sending hold velocity command",
                    mapping.c_str());
        return false;
    }

    // 创建零速度数组
    std::array<double, 6> zero_velocities{};

    // 使用实时速度控制接口发送零速度命令
    bool success = hardware_driver_->send_realtime_velocity_command(interface, zero_velocities);

    if (!success) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "[%s] ❎ Failed to send hold velocity command",
                            mapping.c_str());
    }

    return success;
}

void HardwareManager::on_motor_status_update(const std::string& interface,
                                            uint32_t motor_id,
                                            const hardware_driver::motor_driver::Motor_Status& status) {
    update_joint_state(interface, motor_id, status);

    // 实时安全检查
    if (safety_enabled_) {
        check_safety_limits(interface, motor_id, status);
    }

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
    joint_state.position[local_index] = status.position * M_PI / 180.0;
    joint_state.velocity[local_index] = status.velocity * M_PI / 180.0;
    joint_state.effort[local_index] = status.effort;

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

    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    // 遍历所有 mapping → joint_state
    for (const auto& [mapping, joint_state] : mapping_joint_states_) {
        auto state_copy = joint_state;  // 拷贝一份，以免修改 map 中原始数据
        
        // 填充 header 信息
        state_copy.header.stamp = node_->now();
        state_copy.header.frame_id = get_frame_id(mapping);

        // 发布当前 mapping 的 JointState
        joint_state_pub_->publish(state_copy);

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

        // 4. 遍历并解析所有mapping节点
        for (auto it = hardware_node.begin(); it != hardware_node.end(); ++it) {
            std::string key = it->first.as<std::string>();
            
            std::string mapping_name;
            if (key.find("_mapping") != std::string::npos) {
                // 支持传统的 *_mapping 格式，提取mapping名称（去掉_mapping后缀）
                mapping_name = key.substr(0, key.find("_mapping"));
            } else {
                // 支持直接的mapping名称
                mapping_name = key;
            }
            
            if (!parse_mapping(mapping_name, it->second)) {
                RCLCPP_ERROR(node_->get_logger(), "❎ Failed to parse mapping: %s", mapping_name.c_str());
                return false; // 某个mapping错误则终止
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
    interface_to_mapping_.clear();
    mapping_joint_states_.clear();

    motor_config_.clear();
    joint_names_config_.clear();
    controller_name_config_.clear();
    planning_group_config_.clear();
    frame_id_config_.clear();
    initial_position_config_.clear();
    start_position_config_.clear();
    joint_limits_config_.clear();
    robot_type_config_.clear();
}

bool HardwareManager::parse_mapping(const std::string& mapping_name, const YAML::Node& mapping_node) {
    if (!mapping_node) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Mapping node is null for '%s'", mapping_name.c_str());
        return false;
    }
    // ===== 机器人类型 =====
    robot_type_config_[mapping_name] =
        mapping_node["robot_type"] ? mapping_node["robot_type"].as<std::string>() : "unknown_robot";

    // ===== 接口映射 =====
    std::string interface = 
        mapping_node["interface"] ? mapping_node["interface"].as<std::string>() : "unknown_interface";
    mapping_to_interface_[mapping_name] = interface;
    interface_to_mapping_[interface] = mapping_name;

    // ===== 电机ID列表 =====
    motor_config_[mapping_name] =
        mapping_node["motors"] ? mapping_node["motors"].as<std::vector<uint32_t>>() : std::vector<uint32_t>{};

    // ===== 关节名称列表 =====
    joint_names_config_[mapping_name] =
        mapping_node["joint_names"] ? mapping_node["joint_names"].as<std::vector<std::string>>() : std::vector<std::string>{};

    // ===== 控制器名称 =====
    controller_name_config_[mapping_name] =
        mapping_node["controller_name"] ? mapping_node["controller_name"].as<std::string>() : "unknown_controller";

    // ===== 规划组名称 =====
    planning_group_config_[mapping_name] =
        mapping_node["planning_group"] ? mapping_node["planning_group"].as<std::string>() : "unknown_group";
    
    // ===== 坐标系 =====
    frame_id_config_[mapping_name] =
        mapping_node["frame_id"] ? mapping_node["frame_id"].as<std::string>() : "unknown_frame_id";
    
    // ===== 初始位置 =====
    initial_position_config_[mapping_name] =
        mapping_node["initial_position"] ? mapping_node["initial_position"].as<std::vector<double>>() : std::vector<double>{};

    // ===== 启动位置 =====
    start_position_config_[mapping_name] =
        mapping_node["start_position"] ? mapping_node["start_position"].as<std::vector<double>>() : std::vector<double>{};
    
    RCLCPP_INFO(node_->get_logger(), "✅ Loaded mapping [%s]: interface=%s, controller=%s, group=%s, frame_id=%s, joints=%zu",
        mapping_name.c_str(),
        mapping_to_interface_[mapping_name].c_str(),
        controller_name_config_[mapping_name].c_str(),
        planning_group_config_[mapping_name].c_str(),
        frame_id_config_[mapping_name].c_str(),
        joint_names_config_[mapping_name].size()
    );
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
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Joint %s is near position limit: %.3f rad",
                    mapping.c_str(), joint_name.c_str(), position);
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
        hardware_driver_->control_motor_in_velocity_mode(interface, motor_id, 0.0);

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
