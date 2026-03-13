#include "controller_manager_section.hpp"
#include "controller/hold_state/hold_state_controller.hpp"
#include "controller_base/utility_controller_base.hpp"
#include "controller_base/trajectory_controller_base.hpp"
#include "controller_base/velocity_controller_base.hpp"
#include "controller_base/teach_controller_base.hpp"
#include "controller/controller_registry.hpp"
#include "controller_interface.hpp"
#include "ipc/ipc_context.hpp"
#include <algorithm>
#include <thread>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

ControllerManagerNode::ControllerManagerNode()
    : Node("controller_manager_node")
{
    // 在构造函数中声明参数，确保参数立即可用
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = false;

    this->declare_parameter("velocity_scaling_factor", 1.0, descriptor);
    this->declare_parameter("acceleration_scaling_factor", 1.0, descriptor);
    this->declare_parameter("movej.velocity_scaling_factor", 0.8, descriptor);
    this->declare_parameter("movej.acceleration_scaling_factor", 0.8, descriptor);
    this->declare_parameter("movel.velocity_scaling_factor", 0.5, descriptor);
    this->declare_parameter("movel.acceleration_scaling_factor", 0.5, descriptor);
    this->declare_parameter("movec.velocity_scaling_factor", 0.3, descriptor);
    this->declare_parameter("movec.acceleration_scaling_factor", 0.3, descriptor);

    // 只加载配置，其他初始化延迟到post_init
    load_config();
}

void ControllerManagerNode::post_init() {
    // 现在可以安全使用shared_from_this()
    init_hardware();
    load_motion_planning_parameters();

    // 为所有 mapping 初始化默认模式状态
    std::vector<std::string> mappings;
    if (hardware_manager_) {
        mappings = hardware_manager_->get_all_mappings();
    } else {
        // 硬件不可用时，使用默认 mappings 以支持 IPC 测试
        mappings = {"left_arm", "right_arm"};
        RCLCPP_INFO(this->get_logger(), "Using default mappings for NO-HARDWARE mode");
    }

    init_commons();
    init_action_event_listener();
    init_controllers();

    // 启动默认控制器 - 初始化 MIT 模式
    for (const auto& mapping : mappings) {
        auto key_pair = std::make_pair("SystemStart", mapping);
        auto it = controller_map_.find(key_pair);
        if (it != controller_map_.end()) {
            it->second->start(mapping);
            {
                std::lock_guard<std::mutex> lock(mapping_state_mutex_);
                mapping_to_mode_[mapping] = "SystemStart";
            }

            // ✅ 同步 IPC 侧的状态
            auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
            if (state_mgr) {
                state_mgr->initializeCurrentMode("SystemStart");
            }
        }
    }

    // 然后切换到 HoldState 保持当前位置（为每个mapping都启动）
    for (const auto& mapping : mappings) {
        start_working_controller("HoldState", mapping);

        // ✅ 同步初始化 IPC 侧的状态，确保 current_mode 被正确设置
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
        if (state_mgr) {
            state_mgr->initializeCurrentMode("HoldState");
            RCLCPP_DEBUG(this->get_logger(), "[%s] ✅ IPC state initialized: current_mode=HoldState", mapping.c_str());
        }
    }

    // ✅ Initialize state tracking for IPC - SystemStart mode is already set above
    {
        std::lock_guard<std::mutex> lock(mapping_state_mutex_);
        // At this point, all mappings have been set to "HoldState" by start_working_controller
        // SystemStart was briefly set, then overwritten with HoldState, which is correct
    }
}

void ControllerManagerNode::load_config() {
    try {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("arm_controller");
        std::string yaml_path = pkg_path + "/config/config.yaml";
        yaml_config_ = YAML::LoadFile(yaml_path);
        RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully from %s", yaml_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load configuration: %s", e.what());
        rclcpp::shutdown();
    }
}

void ControllerManagerNode::init_hardware() {
    // 初始化硬件管理器
    hardware_manager_ = HardwareManager::getInstance();
    if (!hardware_manager_) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to get HardwareManager instance - running in NO-HARDWARE mode");
        return;
    }

    // 关键：初始化硬件管理器以启用电机通信
    if (!hardware_manager_->initialize(this->shared_from_this())) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to initialize HardwareManager - running in NO-HARDWARE mode");
        // 不调用 rclcpp::shutdown()，允许系统继续运行以支持 IPC 测试
        hardware_manager_ = nullptr;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "✅ Hardware manager initialized successfully");
}

void ControllerManagerNode::init_commons() {
    if (!yaml_config_["common"]) {
        RCLCPP_ERROR(this->get_logger(), "No 'common' field in config YAML");
        return;
    }

    // 解析common配置 - 使用扁平结构
    for (const auto& item : yaml_config_["common"]) {
        std::string key = item["key"].as<std::string>();
        std::string kind = item["kind"].as<std::string>();
        std::string name = item["name"].as<std::string>();
        std::string type = item["type"].as<std::string>();

        common_topics_[key] = TopicInfo{key, name, type, kind};
    }

    // 创建ROS接口
    auto get_topic_name = [&](const std::string& key) -> std::string {
        auto it = common_topics_.find(key);
        return (it != common_topics_.end()) ? it->second.name : "";
    };

    // 工作模式切换服务
    working_mode_service_ = this->create_service<controller_interfaces::srv::WorkMode>(
        get_topic_name("controller_mode_service"),
        std::bind(&ControllerManagerNode::handle_work_mode, this,
                 std::placeholders::_1, std::placeholders::_2));

    // 电机控制服务
    motor_control_service_ = this->create_service<controller_interfaces::srv::MotorControl>(
        get_topic_name("motor_control_service"),
        std::bind(&ControllerManagerNode::handle_motor_control, this,
                 std::placeholders::_1, std::placeholders::_2));

    // 状态发布器
    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        get_topic_name("running_status"), 10);

    // 状态发布定时器（1Hz）
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ControllerManagerNode::status_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Common topics and ROS interfaces initialized");
}

void ControllerManagerNode::init_controllers() {
    try {
        if (!yaml_config_["controllers"]) {
            RCLCPP_ERROR(this->get_logger(), "No 'controllers' field in config YAML");
            return;
        }
        auto available = get_available_controllers();

        // 获取所有 mapping
        auto all_mappings = hardware_manager_->get_all_mappings();

        for (const auto& entry : yaml_config_["controllers"]) {
            std::string key = entry["key"].as<std::string>();
            std::string class_name = entry["class"].as<std::string>();

            // 从 YAML 提取默认 topic 值
            std::string default_input_topic, default_output_topic;

            // ✅ 支持 input_topic（单个）或 input_topics（多个数组）
            if (entry["input_topic"] && entry["input_topic"]["name"]) {
                default_input_topic = entry["input_topic"]["name"].as<std::string>();
                if (!default_input_topic.empty()) {
                    this->declare_parameter("controllers." + key + ".input_topic", default_input_topic);
                }
            } else if (entry["input_topics"] && entry["input_topics"].IsSequence()) {
                // 处理 input_topics 数组 - 声明为 input_topic0, input_topic1 等（不带点号）
                int idx = 0;
                for (const auto& topic_entry : entry["input_topics"]) {
                    if (topic_entry["name"]) {
                        std::string topic_name = topic_entry["name"].as<std::string>();
                        this->declare_parameter("controllers." + key + ".input_topic" + std::to_string(idx), topic_name);
                        RCLCPP_DEBUG(this->get_logger(), "[controllers] Declared input_topic%d for '%s': %s",
                                    idx, key.c_str(), topic_name.c_str());
                        idx++;
                    }
                }
            }

            if (entry["output_topic"] && entry["output_topic"]["name"]) {
                default_output_topic = entry["output_topic"]["name"].as<std::string>();
            }
            if (!default_output_topic.empty()) {
                this->declare_parameter("controllers." + key + ".output_topic", default_output_topic);
            }

            auto it = available.find(class_name);
            if (it != available.end()) {
                ControllerInterface::instance().register_class(key, it->second);

                // ✅ 创建一个共享的 controller 实例，服务所有 mapping
                // 每个 controller 已经通过 map<string, XXX> 来管理各 mapping 的状态
                auto shared_controller = it->second(this->shared_from_this());
                for (const auto& mapping : all_mappings) {
                    auto key_pair = std::make_pair(key, mapping);
                    controller_map_[key_pair] = shared_controller;
                    RCLCPP_DEBUG(this->get_logger(), "[controllers] Registered mapping: %s for controller: %s (class: %s)",
                                mapping.c_str(), key.c_str(), class_name.c_str());
                }

                // ✅ 为 velocity controllers 注册 hook 请求回调
                if (key == "JointVelocity" || key == "CartesianVelocity") {
                    auto velocity_ctrl = std::dynamic_pointer_cast<VelocityControllerBase>(shared_controller);
                    if (velocity_ctrl) {
                        velocity_ctrl->set_hook_request_callback(
                            [this](const std::string& mapping, const std::string& target_mode) {
                                start_working_controller(target_mode, mapping);
                            });
                    }
                }

                // ✅ 为 TrajectoryRecord / TrajectoryReplay 注册模式切换请求回调
                if (key == "TrajectoryRecord" || key == "TrajectoryReplay") {
                    auto teach_ctrl = std::dynamic_pointer_cast<TeachControllerBase>(shared_controller);
                    if (teach_ctrl) {
                        teach_ctrl->set_hook_request_callback(
                            [this](const std::string& mapping, const std::string& target_mode) {
                                start_working_controller(target_mode, mapping);
                            });
                    }
                }

                RCLCPP_INFO(this->get_logger(), "[controllers] ✅ Created shared controller: %s (class: %s) for %zu mappings (consumer threads: 1)",
                            key.c_str(), class_name.c_str(), all_mappings.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "[controllers] Controller class '%s' not found for key '%s'",
                            class_name.c_str(), key.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Initialized %zu controller instances", controller_map_.size());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize controllers: %s", e.what());
        rclcpp::shutdown();
    }
}


void ControllerManagerNode::handle_work_mode(
    const std::shared_ptr<controller_interfaces::srv::WorkMode::Request> request,
    std::shared_ptr<controller_interfaces::srv::WorkMode::Response> response) {

    std::string new_mode = request->mode;
    std::string mapping = request->mapping.empty() ? "single_arm" : request->mapping;

    RCLCPP_INFO(this->get_logger(), "Request to switch controller mode to %s with mapping: %s",
                new_mode.c_str(), mapping.c_str());

    bool success = start_working_controller(new_mode, mapping);

    if (success) {
        response->success = true;
        response->message = "✅ Switched to mode " + request->mode + " successfully.";
    } else {
        response->success = false;
        response->message = "❎ Failed to switch to mode " + request->mode + ".";
    }
}

bool ControllerManagerNode::start_working_controller(const std::string& mode_name, const std::string& mapping) {
    // 立即取消任何正在执行的轨迹（所有模式切换都需要这样做）
    if (hardware_manager_) {
        hardware_manager_->cancel_trajectory(mapping);
    }

    // 使用 (mode_name, mapping) 对查找 controller 实例
    auto key_pair = std::make_pair(mode_name, mapping);
    auto it = controller_map_.find(key_pair);
    if (it == controller_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid work mode %s for mapping %s", mode_name.c_str(), mapping.c_str());
        return false;
    }

    // 对于Disable和EmergencyStop模式，总是强制执行，即使已经在该模式（确保真正失能）
    if (mode_name == "Disable" || mode_name == "EmergencyStop") {
        // 强制停止当前控制器，不管需不需要钩子状态
        {
            std::lock_guard<std::mutex> lock(mapping_state_mutex_);
            auto current_mode_it = mapping_to_mode_.find(mapping);
            if (current_mode_it != mapping_to_mode_.end()) {
                auto current_key_pair = std::make_pair(current_mode_it->second, mapping);
                auto current_it = controller_map_.find(current_key_pair);
                if (current_it != controller_map_.end()) {
                    current_it->second->stop(mapping);
                    mapping_to_mode_.erase(mapping);
                }
            }
        }
        return switch_to_mode(mode_name, mapping);
    }

    // 如果已经在目标模式（且不是Disable/EmergencyStop），直接返回成功
    {
        std::lock_guard<std::mutex> lock(mapping_state_mutex_);
        auto mapping_mode_it = mapping_to_mode_.find(mapping);
        if (mapping_mode_it != mapping_to_mode_.end() && mapping_mode_it->second == mode_name) {
            auto hook_state_it = mapping_in_hook_state_.find(mapping);
            bool is_in_hook = (hook_state_it != mapping_in_hook_state_.end()) ? hook_state_it->second : false;
            if (!is_in_hook) {
                RCLCPP_INFO(this->get_logger(), "[%s] Already in mode %s", mapping.c_str(), mode_name.c_str());
                return true;
            }
        }

        // 如果当前处于钩子状态，记录请求
        auto hook_state_it = mapping_in_hook_state_.find(mapping);
        bool is_in_hook = (hook_state_it != mapping_in_hook_state_.end()) ? hook_state_it->second : false;
        if (is_in_hook) {
            RCLCPP_INFO(this->get_logger(), "[%s] Currently in hook state, updating target mode to %s",
                        mapping.c_str(), mode_name.c_str());
            // 轨迹已在上面取消，更新目标模式，让持续检查机制自动处理转换
            mapping_target_mode_[mapping] = mode_name;
            return true;
        }
    }

    // 停止当前控制器
    bool need_hook = false;
    if (!stop_working_controller(need_hook, mapping)) {
        RCLCPP_WARN(this->get_logger(), "Failed to stop current controller");
        return false;
    }

    // 如果需要钩子状态，进入钩子状态并开始持续监控
    // ⚠️ 例外：如果目标模式是 HoldState，不需要 hook，直接切换
    // 因为 HoldState 本身就是安全状态，用来作为过渡点
    if (need_hook && mode_name != "HoldState") {
        enter_hook_state(mode_name, mapping);
        return true;    // 进入等待状态，持续监控会处理实际转换
    }

    // 直接切换到目标模式
    return switch_to_mode(mode_name, mapping);
}

bool ControllerManagerNode::stop_working_controller(bool& need_hook, const std::string& mapping) {
    need_hook = false;

    std::string current_mode;
    {
        std::lock_guard<std::mutex> lock(mapping_state_mutex_);
        auto current_mode_it = mapping_to_mode_.find(mapping);
        if (current_mode_it == mapping_to_mode_.end()) {
            RCLCPP_WARN(this->get_logger(), "[%s] No active mode found for mapping", mapping.c_str());
            return true;  // 没有活跃模式，无需停止
        }
        current_mode = current_mode_it->second;
    }

    // 使用 (mode, mapping) 对查找 controller 实例
    auto key_pair = std::make_pair(current_mode, mapping);
    auto it = controller_map_.find(key_pair);
    if (it != controller_map_.end()) {
        // 获取该 controller 的 per-mapping hook 状态
        std::unordered_map<std::string, bool> hook_state_map = it->second->needs_hook_state();
        auto hook_it = hook_state_map.find(mapping);
        need_hook = (hook_it != hook_state_map.end()) ? hook_it->second : false;
        it->second->stop(mapping);

        // ✅ 清除 mapping_to_mode_[mapping]，标记该 mapping 已无活跃模式
        {
            std::lock_guard<std::mutex> lock(mapping_state_mutex_);
            mapping_to_mode_.erase(mapping);
        }

        return true;
    }
    return false;
}

bool ControllerManagerNode::enter_hook_state(const std::string& target_mode, const std::string& mapping) {
    // 设置目标模式（per-mapping）
    {
        std::lock_guard<std::mutex> lock(mapping_state_mutex_);
        mapping_target_mode_[mapping] = target_mode;
        mapping_in_hook_state_[mapping] = true;
    }

    auto hook_key = std::make_pair("HoldState", mapping);
    auto hook_it = controller_map_.find(hook_key);
    if (hook_it != controller_map_.end()) {
        auto hold_controller = std::dynamic_pointer_cast<HoldStateController>(hook_it->second);
        if (hold_controller) {
            // ⚠️ 如果 HoldState 已经为该 mapping 激活（比如来自 post_init 或之前的 hook），
            // 需要先 stop() 再 start() 来重新配置定时器和回调关联
            // 这样才能让新的回调生效
            if (hold_controller->is_active(mapping)) {
                RCLCPP_DEBUG(this->get_logger(), "[%s] Re-entering hook state - restarting HoldState to apply new callback", mapping.c_str());
                hold_controller->stop(mapping);
            }

            // 设置目标状态（per-mapping）
            hold_controller->set_target_mode(mapping, target_mode);

            // 设置转换就绪回调 - 现在是 per-mapping 的
            hold_controller->set_transition_ready_callback(mapping, [this](const std::string& ready_mapping) {
                // 条件满足时，自动执行状态转换
                this->on_transition_ready(ready_mapping);
            });

            // 启动 HoldState控制器（自动持续检查）
            hold_controller->start(mapping);
            {
                std::lock_guard<std::mutex> lock(mapping_state_mutex_);
                mapping_to_mode_[mapping] = "HoldState";
            }

            // ✅ 通知 IPC 侧进入 hook 状态
            auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
            if (state_mgr) {
                // 通过 transitionToMode 通知 IPC 侧需要 hook
                state_mgr->transitionToMode(target_mode);
                RCLCPP_DEBUG(this->get_logger(), "[%s] ✅ IPC notified: entering hook for target mode %s", mapping.c_str(), target_mode.c_str());
            }

            // RCLCPP_INFO(this->get_logger(), "Entered hook state, target mode: %s", target_mode.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to cast HoldState controller");
            {
                std::lock_guard<std::mutex> lock(mapping_state_mutex_);
                mapping_in_hook_state_[mapping] = false;
            }
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "HoldState controller not found in controller map");
        {
            std::lock_guard<std::mutex> lock(mapping_state_mutex_);
            mapping_in_hook_state_[mapping] = false;
        }
        return false;
    }
}

void ControllerManagerNode::on_transition_ready(const std::string& mapping) {
    std::string target;
    {
        std::lock_guard<std::mutex> lock(mapping_state_mutex_);
        auto hook_state_it = mapping_in_hook_state_.find(mapping);
        bool is_in_hook = (hook_state_it != mapping_in_hook_state_.end()) ? hook_state_it->second : false;
        if (!is_in_hook) {
            RCLCPP_WARN(this->get_logger(), "[%s] Transition ready callback called but not in hook state", mapping.c_str());
            return;
        }

        // 保存目标模式，因为exit_hook_state会清空它
        auto target_it = mapping_target_mode_.find(mapping);
        target = (target_it != mapping_target_mode_.end()) ? target_it->second : "";
    }

    // ✅ 先更新 IPC 状态，告知 IPC consumer hook 已完成
    auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
    if (state_mgr) {
        state_mgr->initializeCurrentMode(target);  // 清除 in_hook_state，设置当前模式为目标模式
        RCLCPP_INFO(this->get_logger(), "[%s] ✅ IPC state updated: hook cleared, mode set to %s", mapping.c_str(), target.c_str());
    }

    // 执行实际的状态转换
    if (exit_hook_state(mapping)) {
        RCLCPP_INFO(this->get_logger(), "Successfully transitioned to %s", target.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to transition to %s", target.c_str());
    }
}


bool ControllerManagerNode::exit_hook_state(const std::string& mapping) {
    std::string target_mode;
    {
        std::lock_guard<std::mutex> lock(mapping_state_mutex_);
        auto hook_state_it = mapping_in_hook_state_.find(mapping);
        bool is_in_hook = (hook_state_it != mapping_in_hook_state_.end()) ? hook_state_it->second : false;
        if (!is_in_hook) {
            RCLCPP_WARN(this->get_logger(), "[%s] Not in hook state", mapping.c_str());
            return false;
        }

        // 检查目标模式是否有效
        auto target_it = mapping_target_mode_.find(mapping);
        if (target_it == mapping_target_mode_.end() || target_it->second.empty()) {
            RCLCPP_ERROR(this->get_logger(), "[%s] Target mode is empty when exiting hook state", mapping.c_str());
            return false;
        }
        target_mode = target_it->second;
    }

    // 如果目标模式不是HoldState，则停止当前的HoldState控制器
    // 如果目标就是HoldState，则不需要停止（避免竞态条件）
    if (target_mode != "HoldState") {
        auto hook_key = std::make_pair("HoldState", mapping);
        auto hook_it = controller_map_.find(hook_key);
        if (hook_it != controller_map_.end()) {
            RCLCPP_DEBUG(this->get_logger(), "[%s] Stopping HoldState controller before switching to %s", mapping.c_str(), target_mode.c_str());
            hook_it->second->stop(mapping);
        }
    }

    // 切换到目标模式
    bool success = switch_to_mode(target_mode, mapping);
    if (success) {
        {
            std::lock_guard<std::mutex> lock(mapping_state_mutex_);
            mapping_in_hook_state_[mapping] = false;
            mapping_target_mode_.erase(mapping);
        }
        RCLCPP_INFO(this->get_logger(), "[%s] Exited hook state, switched to %s", mapping.c_str(), target_mode.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "[%s] Failed to switch to target mode: %s", mapping.c_str(), target_mode.c_str());
    }

    return success;
}

bool ControllerManagerNode::switch_to_mode(const std::string& mode_name, const std::string& mapping) {
    // 检查输入参数
    if (mode_name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Mode name is empty");
        return false;
    }

    // 使用 (mode_name, mapping) 对查找 controller 实例
    auto key_pair = std::make_pair(mode_name, mapping);
    auto it = controller_map_.find(key_pair);
    if (it == controller_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Controller not found for mode: %s, mapping: %s", mode_name.c_str(), mapping.c_str());
        return false;
    }

    if (!it->second) {
        RCLCPP_ERROR(this->get_logger(), "Controller pointer is null for mode: %s, mapping: %s", mode_name.c_str(), mapping.c_str());
        return false;
    }

    try {
        // 启动新控制器
        it->second->start(mapping);
        {
            std::lock_guard<std::mutex> lock(mapping_state_mutex_);
            mapping_to_mode_[mapping] = mode_name;
        }

        // ✅ 同步 IPC 侧的状态，防止 IPC consumer 的旧命令重新启动已被 stop 的控制器
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
        if (state_mgr) {
            state_mgr->initializeCurrentMode(mode_name);
            RCLCPP_DEBUG(this->get_logger(), "[%s] ✅ IPC state synchronized: mode set to %s", mapping.c_str(), mode_name.c_str());
        }

        // 清空缓存的消息（避免切换控制器时执行旧消息导致意外运动）
        // 注意：禁用自动投递缓存消息，因为这会导致切换控制器时机械臂意外运动
        auto cached = cached_messages_.find(mode_name);
        if (cached != cached_messages_.end()) {
            RCLCPP_INFO(this->get_logger(), "Clearing cached messages for controller %s (auto-delivery disabled for safety)", mode_name.c_str());
            cached_messages_.erase(cached);
        }
        RCLCPP_INFO(this->get_logger(), "✅ Switched to mode %s", mode_name.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❎ Failed to switch to mode %s: %s", mode_name.c_str(), e.what());
        return false;
    }
}

bool ControllerManagerNode::check_work_mode(const std::string& target_mode, const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(mapping_state_mutex_);
    auto mode_it = mapping_to_mode_.find(mapping);
    if (mode_it != mapping_to_mode_.end()) {
        return mode_it->second == target_mode;
    }
    return false;
}

void ControllerManagerNode::status_timer_callback() {
    publish_status();
}

void ControllerManagerNode::publish_status() {
    std_msgs::msg::String status_msg;
    // 构建包含所有mapping状态的字符串
    std::string status_str;
    {
        std::lock_guard<std::mutex> lock(mapping_state_mutex_);
        for (const auto& [mapping, mode] : mapping_to_mode_) {
            if (!status_str.empty()) {
                status_str += " | ";
            }
            status_str += mapping + ":" + mode;
        }
    }
    if (status_str.empty()) {
        status_str = "Uninitialized";
    }
    status_msg.data = status_str;
    status_publisher_->publish(status_msg);
}

void ControllerManagerNode::init_action_event_listener() {
    // 创建动作事件订阅器
    action_event_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/action_controller_events", rclcpp::QoS(10).reliable(),
        std::bind(&ControllerManagerNode::handle_action_event, this, std::placeholders::_1));

    // 创建轨迹控制命令订阅器
    trajectory_control_subscriber_ = this->create_subscription<controller_interfaces::msg::TrajectoryControl>(
        "/trajectory_control", rclcpp::QoS(10).reliable(),
        std::bind(&ControllerManagerNode::handle_trajectory_control, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action event listener and trajectory control listener initialized");
}

void ControllerManagerNode::handle_action_event(const std_msgs::msg::String::SharedPtr msg) {
    // 安全检查
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null action event message");
        return;
    }

    // 解析事件消息格式: "event_type:mapping"
    std::string event_data = msg->data;

    if (event_data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty action event message");
        return;
    }

    size_t delimiter_pos = event_data.find(':');

    std::string event_type = event_data;
    std::string mapping = "single_arm";  // 默认mapping

    if (delimiter_pos != std::string::npos) {
        event_type = event_data.substr(0, delimiter_pos);
        mapping = event_data.substr(delimiter_pos + 1);
    }

    RCLCPP_INFO(this->get_logger(), "Received action event: %s (mapping: %s)", event_type.c_str(), mapping.c_str());

    // 如果已经在钩子状态中（用户已主动请求切换到某个模式），不应该被action事件改变
    {
        std::lock_guard<std::mutex> lock(mapping_state_mutex_);
        auto hook_state_it = mapping_in_hook_state_.find(mapping);
        bool is_in_hook = (hook_state_it != mapping_in_hook_state_.end()) ? hook_state_it->second : false;
        if (is_in_hook) {
            RCLCPP_DEBUG(this->get_logger(), "[%s] Already in hook state, ignoring action event", mapping.c_str());
            return;
        }
    }

    if (event_type == "action_goal_accepted") {
        // 自动切换到ROS2ActionControl模式（无论之前在哪个模式）
        RCLCPP_INFO(this->get_logger(), "Action goal accepted for mapping: %s, switching to ROS2ActionControl mode", mapping.c_str());
        start_working_controller("ROS2ActionControl", mapping);
    }
    else if (event_type == "action_goal_rejected") {
        // 自动切换到HoldState模式
        RCLCPP_INFO(this->get_logger(), "Action goal rejected for mapping: %s, switching to HoldState mode", mapping.c_str());
        start_working_controller("HoldState", mapping);
    }
    else if (event_type == "action_cancelled") {
        // 自动切换到HoldState模式
        RCLCPP_INFO(this->get_logger(), "Action cancelled for mapping: %s, switching to HoldState mode", mapping.c_str());
        start_working_controller("HoldState", mapping);
    }
    else if (event_type == "action_aborted") {
        // 自动切换到HoldState模式
        RCLCPP_INFO(this->get_logger(), "Action aborted for mapping: %s, switching to HoldState mode", mapping.c_str());
        start_working_controller("HoldState", mapping);
    }
    else if (event_type == "action_succeeded") {
        // 自动切换到HoldState模式
        RCLCPP_INFO(this->get_logger(), "Action succeeded for mapping: %s, switching to HoldState mode", mapping.c_str());
        start_working_controller("HoldState", mapping);
    }
    else if (event_type == "action_failed") {
        // 自动切换到HoldState模式
        RCLCPP_INFO(this->get_logger(), "Action failed for mapping: %s, switching to HoldState mode", mapping.c_str());
        start_working_controller("HoldState", mapping);
    }
}

void ControllerManagerNode::handle_motor_control(
    const std::shared_ptr<controller_interfaces::srv::MotorControl::Request> request,
    std::shared_ptr<controller_interfaces::srv::MotorControl::Response> response) {

    std::string mapping = request->mapping.empty() ? "single_arm" : request->mapping;
    std::string action = request->action;

    RCLCPP_INFO(this->get_logger(), "Motor control request: action=%s, mapping=%s", action.c_str(), mapping.c_str());

    if (!hardware_manager_) {
        response->success = false;
        response->message = "Hardware manager not initialized";
        return;
    }

    // 电机模式
    uint8_t mode = request->mode;

    if (action == "Enable") {
        bool success = hardware_manager_->enable_motors(mapping, mode);
        response->success = success;
        response->message = success ? "✅ Motors enabled successfully" : "❎ Failed to enable motors";
    } else if (action == "Disable") {
        // 失能电机
        bool success = hardware_manager_->disable_motors(mapping, mode);
        response->success = success;
        response->message = success ? "✅ Motors disabled successfully" : "❎ Failed to disable motors";
    } else {
        response->success = false;
        response->message = "Invalid action: " + action + ". Use 'Enable' or 'Disable'";
    }
}

void ControllerManagerNode::handle_trajectory_control(const controller_interfaces::msg::TrajectoryControl::SharedPtr msg) {
    // 安全检查
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null trajectory control message");
        return;
    }

    if (!hardware_manager_) {
        RCLCPP_ERROR(this->get_logger(), "Hardware manager not initialized");
        return;
    }

    std::string action = msg->action;
    std::string mapping = msg->mapping.empty() ? "single_arm" : msg->mapping;

    RCLCPP_INFO(this->get_logger(), "Received trajectory control command: action=%s, mapping=%s",
                action.c_str(), mapping.c_str());

    if (action == "Pause") {
        if (hardware_manager_->pause_trajectory(mapping)) {
            RCLCPP_INFO(this->get_logger(), "✅ Trajectory paused successfully for mapping: %s", mapping.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️  Failed to pause trajectory for mapping: %s", mapping.c_str());
        }
    }
    else if (action == "Resume") {
        if (hardware_manager_->resume_trajectory(mapping)) {
            RCLCPP_INFO(this->get_logger(), "✅ Trajectory resumed successfully for mapping: %s", mapping.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️  Failed to resume trajectory for mapping: %s", mapping.c_str());
        }
    }
    else if (action == "Cancel") {
        if (hardware_manager_->cancel_trajectory(mapping)) {
            RCLCPP_INFO(this->get_logger(), "✅ Trajectory cancelled successfully for mapping: %s", mapping.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️  Failed to cancel trajectory for mapping: %s", mapping.c_str());
        }
    }
    else {
        RCLCPP_WARN(this->get_logger(), "⚠️  Unknown trajectory control action: %s", action.c_str());
    }
}

void ControllerManagerNode::load_motion_planning_parameters() {
    try {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("arm_controller");
        std::string config_path = pkg_path + "/config/config.yaml";
        YAML::Node config = YAML::LoadFile(config_path);

        // 从配置文件读取并设置参数
        if (config["velocity_scaling_factor"]) {
            this->set_parameter(rclcpp::Parameter("velocity_scaling_factor",
                config["velocity_scaling_factor"].as<double>()));
        }
        if (config["acceleration_scaling_factor"]) {
            this->set_parameter(rclcpp::Parameter("acceleration_scaling_factor",
                config["acceleration_scaling_factor"].as<double>()));
        }

        // 加载各控制器专用参数
        for (const auto& controller : {"movej", "movel", "movec"}) {
            if (config[controller]) {
                if (config[controller]["velocity_scaling_factor"]) {
                    std::string param_name = std::string(controller) + ".velocity_scaling_factor";
                    this->set_parameter(rclcpp::Parameter(param_name,
                        config[controller]["velocity_scaling_factor"].as<double>()));
                }
                if (config[controller]["acceleration_scaling_factor"]) {
                    std::string param_name = std::string(controller) + ".acceleration_scaling_factor";
                    this->set_parameter(rclcpp::Parameter(param_name,
                        config[controller]["acceleration_scaling_factor"].as<double>()));
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "✅ Motion planning parameters loaded from config.yaml");

    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to load motion planning parameters: %s", e.what());
    }
}
