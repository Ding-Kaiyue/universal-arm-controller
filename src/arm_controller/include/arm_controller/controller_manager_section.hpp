#ifndef __CONTROLLER_MANAGER_SECTION_HPP__
#define __CONTROLLER_MANAGER_SECTION_HPP__

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <set>
#include <map>
#include <unordered_map>
#include <any>
#include <controller_interfaces/srv/work_mode.hpp>
#include <std_msgs/msg/string.hpp>
#include "arm_controller/controller_base/mode_controller_base.hpp"
#include "arm_controller/hardware/hardware_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

class ControllerManagerNode : public rclcpp::Node {
public:
    ControllerManagerNode();
    ~ControllerManagerNode() = default;

    // 延迟初始化 - 在构造函数外调用
    void post_init();

private:
    // 配置加载和初始化
    void init_hardware();
    void init_controllers();

    // 控制器管理
    bool start_working_controller(const std::string& mode_name, const std::string& mapping = "");
    bool stop_working_controller(bool& need_hook, const std::string& mapping = "");
    bool check_work_mode(const std::string& target_mode) const;

    // 高级状态管理
    bool enter_hook_state(const std::string& target_mode, const std::string& mapping = "");
    void on_transition_ready(const std::string& mapping = "");
    bool exit_hook_state(const std::string& mapping = "");
    bool switch_to_mode(const std::string& mode_name, const std::string& mapping = "");

    // 配置加载
    void load_config();
    void init_commons();

    // 服务回调
    void handle_work_mode(
        const std::shared_ptr<controller_interfaces::srv::WorkMode::Request> request,
        std::shared_ptr<controller_interfaces::srv::WorkMode::Response> response);

    // 特殊控制器执行方法
    // bool execute_move_to_start(const std::string& mapping);
    // bool execute_move_to_initial(const std::string& mapping);

    // 状态监控
    void publish_status();
    void status_timer_callback();

    // 动作事件处理
    void init_action_event_listener();
    void handle_action_event(const std_msgs::msg::String::SharedPtr msg);

    // 成员变量
    std::map<std::string, std::shared_ptr<ModeControllerBase>> controller_map_;
    std::string current_mode_;
    std::string target_mode_;
    bool in_hook_state_;
    bool emergency_stop_active_;
    bool safety_zone_violation_;

    // 配置和缓存
    YAML::Node yaml_config_;
    std::unordered_map<std::string, std::any> cached_messages_;

    struct TopicInfo {
        std::string key;
        std::string name;
        std::string type;
        std::string kind;
    };
    std::unordered_map<std::string, TopicInfo> common_topics_;

    // ROS接口
    rclcpp::Service<controller_interfaces::srv::WorkMode>::SharedPtr working_mode_service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 动作事件监听器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_event_subscriber_;

    // 硬件管理
    std::shared_ptr<HardwareManager> hardware_manager_;

    // 配置管理
    // Configuration is loaded via YAML files directly
};

#endif // __CONTROLLER_MANAGER_SECTION_HPP__