#ifndef __VELOCITY_CONTROLLER_BASE_HPP__
#define __VELOCITY_CONTROLLER_BASE_HPP__

#include "arm_controller/controller_base/mode_controller_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <any>
#include <map>

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

    // 初始化订阅 - 为指定的 mapping 创建话题订阅
    void init_subscriptions(const std::string& mapping) {
        if (mapping.empty()) return;

        // 从配置获取话题名称
        std::string input_topic;
        node_->get_parameter("controllers." + get_mode() + ".input_topic", input_topic);

        if (input_topic.empty()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] No input_topic configured for controller", get_mode().c_str());
            return;
        }

        // 替换 {mapping} 占位符
        size_t pos = input_topic.find("{mapping}");
        if (pos != std::string::npos) {
            input_topic.replace(pos, 9, mapping);
        }

        // 创建订阅
        subscriptions_[mapping] = node_->create_subscription<T>(
            input_topic, rclcpp::QoS(10).reliable(),
            [this](const typename T::SharedPtr msg) {
                if (!is_active_) return;
                velocity_callback(msg);
            }
        );

        RCLCPP_INFO(node_->get_logger(), "[%s] Subscribed to topic: %s (mapping: %s)",
                   get_mode().c_str(), input_topic.c_str(), mapping.c_str());
    }

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
    std::map<std::string, typename rclcpp::Subscription<T>::SharedPtr> subscriptions_;

    // 清理指定 mapping 的订阅 - 在 stop() 时调用
    void cleanup_subscriptions(const std::string& mapping) {
        auto it = subscriptions_.find(mapping);
        if (it != subscriptions_.end()) {
            it->second.reset();
            subscriptions_.erase(it);
            RCLCPP_INFO(node_->get_logger(), "[%s] Cleaned up subscription for mapping: %s",
                       get_mode().c_str(), mapping.c_str());
        }
    }

    // 子类可以重写此方法以从消息中提取 mapping 信息
    virtual std::string get_mapping_from_message([[maybe_unused]] const typename T::SharedPtr msg) {
        return "";  // 默认实现
    }
};

#endif