#ifndef __UTILITY_CONTROLLER_BASE_HPP__
#define __UTILITY_CONTROLLER_BASE_HPP__

#include "arm_controller/controller_base/mode_controller_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <any>

class UtilityControllerBase : public ModeControllerBase {
public:
    explicit UtilityControllerBase(std::string mode, rclcpp::Node::SharedPtr node)
        : ModeControllerBase(mode), node_(node) {}
    virtual ~UtilityControllerBase() = default;

    // 子类必须实现这两个方法，执行自己的初始化/清理逻辑
    // 最后需要调用 ModeControllerBase::start/stop() 来更新 active_mappings_ 状态
    virtual void start(const std::string& mapping) = 0;
    virtual bool stop(const std::string& mapping) = 0;

    // 进入模式 - 用户通过 IPC 调用此方法进入当前 mode
    virtual bool enter_mode(const std::string& mapping = "") {
        start(mapping);
        return is_active(mapping);
    }

    // 退出模式 - 用户通过 IPC 调用此方法退出当前 mode
    virtual bool exit_mode(const std::string& mapping = "") {
        return stop(mapping);
    }

protected:
    rclcpp::Node::SharedPtr node_;
};

#endif // __UTILITY_CONTROLLER_BASE_HPP__
