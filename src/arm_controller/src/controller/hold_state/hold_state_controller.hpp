#ifndef __HOLD_STATE_CONTROLLER_HPP__
#define __HOLD_STATE_CONTROLLER_HPP__

#include "controller_base/utility_controller_base.hpp"
#include "std_msgs/msg/string.hpp"
#include "hardware/hardware_manager.hpp"
#include <functional>
#include <string>
#include <unordered_map>
#include <mutex>

class HoldStateController final : public UtilityControllerBase {
public:
    // 转换就绪回调函数类型 - 现在包含 mapping 参数
    using TransitionReadyCallback = std::function<void(const std::string&)>;

    explicit HoldStateController(const rclcpp::Node::SharedPtr& node);
    ~HoldStateController() override = default;

    void start(const std::string& mapping) override;
    bool stop(const std::string& mapping) override;

    // 设置目标状态，当钩子状态完成时切换到该状态
    void set_target_mode(const std::string& mapping, const std::string& target_mode) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        mapping_target_modes_[mapping] = target_mode;
    }
    std::string get_target_state(const std::string& mapping) const {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        auto it = mapping_target_modes_.find(mapping);
        return it != mapping_target_modes_.end() ? it->second : "";
    }

    // 设置转换就绪回调函数 - 现在是 per-mapping 的
    void set_transition_ready_callback(const std::string& mapping, TransitionReadyCallback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        mapping_callbacks_[mapping] = callback;
    }

    // 检查是否可以安全切换到目标状态
    bool can_transition_to_target(const std::string& mapping);

    /**
     * @brief 启动安全过渡流程
     *
     * 直接启动 HoldState 并设置过渡完成时的回调
     * 当安全检查完成后，回调会被触发以启动目标模式
     */
    void start_safe_transition(const std::string& mapping, const std::string& target_mode,
                              TransitionReadyCallback on_ready) {
        set_target_mode(mapping, target_mode);
        set_transition_ready_callback(mapping, on_ready);
        if (!is_active(mapping)) {
            start(mapping);
        }
    }

private:
    struct MappingContext {
        rclcpp::TimerBase::SharedPtr safety_timer;  // 定时器：仅用于安全检查，不发送保持命令
        bool transition_ready = false;
        bool system_health_check_paused = false;
        std::vector<double> hold_positions;  // 记录当前关节位置以保持状态
    };

    std::unordered_map<std::string, MappingContext> mapping_contexts_;
    std::shared_ptr<HardwareManager> hardware_manager_;

    // Per-mapping target modes and callbacks
    std::unordered_map<std::string, std::string> mapping_target_modes_;
    std::unordered_map<std::string, TransitionReadyCallback> mapping_callbacks_;
    mutable std::mutex callback_mutex_;  // ✅ 保护 mapping_callbacks_ 和 mapping_target_modes_

    // internal helpers
    void safety_check_timer_callback(const std::string& mapping);
};

#endif  // __HOLD_STATE_CONTROLLER_HPP__ 