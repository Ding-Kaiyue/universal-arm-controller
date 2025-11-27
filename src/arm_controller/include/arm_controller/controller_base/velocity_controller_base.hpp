#ifndef __VELOCITY_CONTROLLER_BASE_HPP__
#define __VELOCITY_CONTROLLER_BASE_HPP__

#include "arm_controller/controller_base/mode_controller_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <any>

class VelocityControllerBase : public ModeControllerBase {
public:
    explicit VelocityControllerBase(std::string mode) : ModeControllerBase(mode) {}
    virtual ~VelocityControllerBase() = default;
    virtual void handle_message(std::any msg) override = 0;
};

template<typename T>
class VelocityControllerImpl : public VelocityControllerBase {
public:
    explicit VelocityControllerImpl(std::string mode, rclcpp::Node::SharedPtr node)
        : VelocityControllerBase(mode), node_(node) {}
    virtual ~VelocityControllerImpl() = default;

    virtual void velocity_callback(const typename T::SharedPtr msg) = 0;

    void handle_message(std::any msg) override final{
        try {
            auto typed_msg = std::any_cast<typename T::SharedPtr>(msg);
            velocity_callback(typed_msg);
        } catch (const std::bad_any_cast& e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("VelocityControllerImpl"), "Failed to cast message to type " << typeid(T).name());
        }
    }

    void start(const std::string& mapping) override = 0;
    bool stop(const std::string& mapping) override = 0;

    // 速度控制器通常需要钩子状态来安全停止
    bool needs_hook_state() const override { return true; }

protected:
    rclcpp::Node::SharedPtr node_;

    // 子类可以重写此方法以从消息中提取 mapping 信息
    virtual std::string get_mapping_from_message([[maybe_unused]] const typename T::SharedPtr msg) {
        return "";  // 默认实现
    }
};

#endif
