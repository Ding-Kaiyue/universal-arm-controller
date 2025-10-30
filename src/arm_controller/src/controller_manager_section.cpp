#include "controller_manager_section.hpp"
#include "controller/hold_state/hold_state_controller.hpp"
#include "controller_base/utility_controller_base.hpp"
#include "controller/controller_registry.hpp"
#include "controller_interface.hpp"
// #include "controller/move2start/move2start_controller.hpp"
// #include "controller/move2initial/move2initial_controller.hpp"

ControllerManagerNode::ControllerManagerNode()
    : Node("controller_manager")
    , current_mode_("HoldState")
    , in_hook_state_(false)
    , emergency_stop_active_(false)
    , safety_zone_violation_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Controller Manager Node");

    // 只加载配置，其他初始化延迟到post_init
    load_config();

    RCLCPP_INFO(this->get_logger(), "Controller Manager Node basic initialization complete");
}

void ControllerManagerNode::post_init() {
    RCLCPP_INFO(this->get_logger(), "Starting post-initialization");

    // 现在可以安全使用shared_from_this()
    init_hardware();
    init_commons();
    init_action_event_listener();
    init_controllers();

    // 启动默认控制器
    start_working_controller("HoldState");

    RCLCPP_INFO(this->get_logger(), "Controller Manager Node post-initialization complete");
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
        RCLCPP_FATAL(this->get_logger(), "Failed to get HardwareManager instance");
        rclcpp::shutdown();
        return;
    }

    // 关键：初始化硬件管理器以启用电机通信
    if (!hardware_manager_->initialize(this->shared_from_this())) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize HardwareManager");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Hardware manager initialized successfully");

    // 打印可用接口信息
    // auto interfaces = hardware_manager_->get_interfaces();
    // RCLCPP_INFO(this->get_logger(), "Available interfaces: %zu", interfaces.size());
    // for (const auto& interface : interfaces) {
    //     RCLCPP_INFO(this->get_logger(), "  - Interface: %s", interface.c_str());
    // }
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
        RCLCPP_INFO(this->get_logger(), "[common] key=%s kind=%s name=%s type=%s",
            key.c_str(), kind.c_str(), name.c_str(), type.c_str());
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

        for (const auto& entry : yaml_config_["controllers"]) {
            std::string key = entry["key"].as<std::string>();
            std::string class_name = entry["class"].as<std::string>();

            // 从 YAML 提取默认 topic 值
            std::string default_input_topic, default_output_topic;
            if (entry["input_topic"] && entry["input_topic"]["name"]) {
                default_input_topic = entry["input_topic"]["name"].as<std::string>();
            }
            if (!default_input_topic.empty()) {
                this->declare_parameter("controllers." + key + ".input_topic", default_input_topic);
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
                auto controller = it->second(this->shared_from_this());
                controller_map_[key] = controller;
                RCLCPP_INFO(this->get_logger(), "[controllers] Registered controller: %s (class: %s)",
                            key.c_str(), class_name.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "[controllers] Controller class '%s' not found for key '%s'",
                            class_name.c_str(), key.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Initialized %zu controllers", controller_map_.size());
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
        current_mode_ = new_mode;
        response->success = true;
        response->message = "✅ Switched to mode " + request->mode + " successfully.";
    } else {
        response->success = false;
        response->message = "❎ Failed to switch to mode " + request->mode + ".";
    }
}

bool ControllerManagerNode::start_working_controller(const std::string& mode_name, const std::string& mapping) {
    auto it = controller_map_.find(mode_name);
    if (it == controller_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid work mode %s", mode_name.c_str());
        return false;
    }

    // 对于Disable和EmergencyStop模式，总是强制执行，即使已经在该模式（确保真正失能）
    if (mode_name == "Disable" || mode_name == "EmergencyStop") {
        // 强制停止当前控制器，不管需不需要钩子状态
        auto current_it = controller_map_.find(current_mode_);
        if (current_it != controller_map_.end()) {
            current_it->second->stop(mapping);
            RCLCPP_INFO(this->get_logger(), "Force stopped controller for mode: %s", current_mode_.c_str());
        }
        return switch_to_mode(mode_name, mapping);
    }

    // 如果已经在目标模式（且不是Disable/EmergencyStop），直接返回成功
    if (current_mode_ == mode_name && !in_hook_state_) {
        RCLCPP_INFO(this->get_logger(), "Already in mode %s", mode_name.c_str());
        return true;
    }

    // 如果当前处于钩子状态，记录请求但不做任何处理，让持续检查机制自动处理转换
    if (in_hook_state_) {
        RCLCPP_INFO(this->get_logger(), "Currently in hook state, continous safety monitoring will handle transition to mode %s",
                    mode_name.c_str());
        return true;
    }

    // 停止当前控制器
    bool need_hook = false;
    if (!stop_working_controller(need_hook, mapping)) {
        RCLCPP_WARN(this->get_logger(), "Failed to stop current controller");
        return false;
    }

    // 如果需要钩子状态，进入钩子状态并开始持续监控
    if (need_hook) {
        RCLCPP_INFO(this->get_logger(), "Need hook state for safe transition to %s", mode_name.c_str());
        enter_hook_state(mode_name, mapping);
        return true;    // 进入等待状态，持续监控会处理实际转换
    }

    // 直接切换到目标模式
    return switch_to_mode(mode_name, mapping);
}

bool ControllerManagerNode::stop_working_controller(bool& need_hook, const std::string& mapping) {
    need_hook = false;
    auto it = controller_map_.find(current_mode_);
    if (it != controller_map_.end()) {
        need_hook = it->second->needs_hook_state();
        it->second->stop(mapping);
        RCLCPP_INFO(this->get_logger(), "[%s] Stopped controller for mode: %s, needs_hook: %s",
                    mapping.c_str(), current_mode_.c_str(), need_hook ? "true" : "false");
        return true;
    }
    return false;
}

bool ControllerManagerNode::enter_hook_state(const std::string& target_mode, const std::string& mapping) {
    // 设置目标模式
    target_mode_ = target_mode;
    in_hook_state_ = true;

    auto hook_it = controller_map_.find("HoldState");
    if (hook_it != controller_map_.end()) {
        auto hold_controller = std::dynamic_pointer_cast<HoldStateController>(hook_it->second);
        if (hold_controller) {
            // 设置前一个模式（从当前模式切换到 HoldState）
            hold_controller->set_previous_mode(current_mode_);
            RCLCPP_DEBUG(this->get_logger(), "Set previous_mode to '%s' for HoldState", current_mode_.c_str());

            // 设置目标状态
            hold_controller->set_target_mode(target_mode);

            // 设置转换就绪回调
            hold_controller->set_transition_ready_callback([this, mapping]() {
                // 条件满足时，自动执行状态转换
                this->on_transition_ready(mapping);
            });

            // 启动 HoldState控制器（自动持续检查）
            hold_controller->start(mapping);
            current_mode_ = "HoldState";

            RCLCPP_INFO(this->get_logger(), "Entered hook state, target mode: %s", target_mode.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to cast HoldState controller");
            in_hook_state_ = false;
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "HoldState controller not found in controller map");
        in_hook_state_ = false;
        return false;
    }
}

void ControllerManagerNode::on_transition_ready(const std::string& mapping) {
    if (!in_hook_state_) {
        RCLCPP_WARN(this->get_logger(), "Transition ready callback called but not in hook state");
        return;
    }

    // 保存目标模式，因为exit_hook_state会清空它
    std::string target = target_mode_;

    // 执行实际的状态转换
    if (exit_hook_state(mapping)) {
        RCLCPP_INFO(this->get_logger(), "Successfully transitioned to %s", target.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to transition to %s", target.c_str());
    }
}


bool ControllerManagerNode::exit_hook_state(const std::string& mapping) {
    if (!in_hook_state_) {
        RCLCPP_WARN(this->get_logger(), "Not in hook state");
        return false;
    }

    // 检查目标模式是否有效
    if (target_mode_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Target mode is empty when exiting hook state");
        return false;
    }

    // 如果目标模式不是HoldState，则停止当前的HoldState控制器
    // 如果目标就是HoldState，则不需要停止（避免竞态条件）
    if (target_mode_ != "HoldState") {
        auto hook_it = controller_map_.find("HoldState");
        if (hook_it != controller_map_.end()) {
            RCLCPP_DEBUG(this->get_logger(), "Stopping HoldState controller before switching to %s", target_mode_.c_str());
            hook_it->second->stop(mapping);
        }
    }

    // 切换到目标模式
    bool success = switch_to_mode(target_mode_, mapping);
    if (success) {
        in_hook_state_ = false;
        RCLCPP_INFO(this->get_logger(), "Exited hook state, switched to %s", target_mode_.c_str());
        target_mode_.clear();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch to target mode: %s", target_mode_.c_str());
    }

    return success;
}

bool ControllerManagerNode::switch_to_mode(const std::string& mode_name, const std::string& mapping) {
    // 检查输入参数
    if (mode_name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Mode name is empty");
        return false;
    }

    auto it = controller_map_.find(mode_name);
    if (it == controller_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Controller not found for mode: %s", mode_name.c_str());
        return false;
    }

    if (!it->second) {
        RCLCPP_ERROR(this->get_logger(), "Controller pointer is null for mode: %s", mode_name.c_str());
        return false;
    }

    try {
        // 如果切换到 HoldState，传递前一个模式
        if (mode_name == "HoldState") {
            auto hold_controller = std::dynamic_pointer_cast<HoldStateController>(it->second);
            if (hold_controller) {
                std::string prev_mode = current_mode_;  // 创建本地副本，避免竞态条件
                hold_controller->set_previous_mode(prev_mode);
                RCLCPP_DEBUG(this->get_logger(), "Set previous_mode to '%s' for HoldState", prev_mode.c_str());
            }
        }

        // 启动新控制器
        it->second->start(mapping);
        current_mode_ = mode_name;

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

bool ControllerManagerNode::check_work_mode(const std::string& target_mode) const {
    return current_mode_ == target_mode;
}

void ControllerManagerNode::status_timer_callback() {
    publish_status();
}

void ControllerManagerNode::publish_status() {
    std_msgs::msg::String status_msg;
    status_msg.data = current_mode_;
    status_publisher_->publish(status_msg);
}

void ControllerManagerNode::init_action_event_listener() {
    // 创建动作事件订阅器
    action_event_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/action_controller_events", rclcpp::QoS(10).reliable(),
        std::bind(&ControllerManagerNode::handle_action_event, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action event listener initialized");
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

// bool ControllerManagerNode::execute_move_to_start(const std::string& mapping) {
//     RCLCPP_INFO(this->get_logger(), "Executing move to start for mapping: %s", mapping.c_str());

//     // 获取Move2Start控制器
//     auto it = controller_map_.find("Move2StartController");
//     if (it == controller_map_.end()) {
//         RCLCPP_ERROR(this->get_logger(), "Move2StartController not found");
//         return false;
//     }

//     // 转换为Move2StartController类型
//     auto move2start_controller = std::dynamic_pointer_cast<Move2StartController>(it->second);
//     if (!move2start_controller) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to cast to Move2StartController");
//         return false;
//     }

//     try {
//         // 调用plan_and_execute_to_start方法
//         move2start_controller->plan_and_execute_to_start(mapping);
//         RCLCPP_INFO(this->get_logger(), "Move to start completed for mapping: %s", mapping.c_str());
//         return true;
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to execute move to start for mapping %s: %s",
//                      mapping.c_str(), e.what());
//         return false;
//     }
// }

// bool ControllerManagerNode::execute_move_to_initial(const std::string& mapping) {
//     RCLCPP_INFO(this->get_logger(), "Executing move to initial for mapping: %s", mapping.c_str());

//     // 获取Move2Initial控制器
//     auto it = controller_map_.find("Move2InitialController");
//     if (it == controller_map_.end()) {
//         RCLCPP_ERROR(this->get_logger(), "Move2InitialController not found");
//         return false;
//     }

//     // 转换为Move2InitialController类型
//     auto move2initial_controller = std::dynamic_pointer_cast<Move2InitialController>(it->second);
//     if (!move2initial_controller) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to cast to Move2InitialController");
//         return false;
//     }

//     try {
//         // 调用plan_and_execute_to_initial方法
//         move2initial_controller->plan_and_execute_to_initial(mapping);
//         RCLCPP_INFO(this->get_logger(), "Move to initial completed for mapping: %s", mapping.c_str());
//         return true;
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to execute move to initial for mapping %s: %s",
//                      mapping.c_str(), e.what());
//         return false;
//     }
// }