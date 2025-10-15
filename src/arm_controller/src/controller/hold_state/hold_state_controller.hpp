#ifndef __HOLD_STATE_CONTROLLER_HPP__
#define __HOLD_STATE_CONTROLLER_HPP__

#include "controller_base/utility_controller_base.hpp"
#include "std_msgs/msg/string.hpp"
#include "hardware/hardware_manager.hpp"
#include <functional>
#include <string>
#include <unordered_map>

class HoldStateController final : public UtilityControllerBase {
public:
    // 转换就绪回调函数类型
    using TransitionReadyCallback = std::function<void()>;

    explicit HoldStateController(const rclcpp::Node::SharedPtr& node);
    ~HoldStateController() override = default;

    void start(const std::string& mapping) override;
    bool stop(const std::string& mapping) override;

    // 设置目标状态，当钩子状态完成时切换到该状态
    void set_target_mode(const std::string& target_mode) { target_mode_ = target_mode; }
    std::string get_target_state() const { return target_mode_; }

    // 设置前一个模式，用于选择合适的保持策略
    void set_previous_mode(const std::string& previous_mode) { previous_mode_ = previous_mode; }

    // 设置转换就绪回调函数
    void set_transition_ready_callback(TransitionReadyCallback callback) {
        transition_ready_callback_ = callback;
    }

    // 检查是否可以安全切换到目标状态
    bool can_transition_to_target(const std::string& mapping);

private:
    struct MappingContext {
        rclcpp::TimerBase::SharedPtr safety_timer;
        bool transition_ready = false;
        bool system_health_check_paused = false;
        std::vector<double> hold_positions;  // 记录当前关节位置以保持状态
    };

    std::string target_mode_;
    std::string previous_mode_;  // 前一个模式，用于选择保持策略
    std::unordered_map<std::string, MappingContext> mapping_contexts_;
    std::shared_ptr<HardwareManager> hardware_manager_;

    // 全局回调（调用者会由 controller manager 设置）
    TransitionReadyCallback transition_ready_callback_;

    // internal helpers
    void safety_check_timer_callback(const std::string& mapping);
};

#endif  // __HOLD_STATE_CONTROLLER_HPP__ 