#ifndef __VELOCITY_CONTROLLER_BASE_HPP__
#define __VELOCITY_CONTROLLER_BASE_HPP__

#include "arm_controller/controller_base/mode_controller_base.hpp"
#include "hardware/hardware_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <any>
#include <map>

class VelocityControllerBase : public ModeControllerBase {
public:
    explicit VelocityControllerBase(std::string mode) : ModeControllerBase(mode) {}
    virtual ~VelocityControllerBase() = default;
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
            [this, mapping](const typename T::SharedPtr msg) {
                if (!is_active(mapping)) return;
                velocity_callback(mapping, msg);
            }
        );

        RCLCPP_INFO(node_->get_logger(), "[%s] Subscribed to topic: %s (mapping: %s)",
                   get_mode().c_str(), input_topic.c_str(), mapping.c_str());
    }

    virtual void velocity_callback(const std::string& mapping, const typename T::SharedPtr msg) = 0;

    // 直接发送速度命令 - 通过 IPC 命令队列消费线程调用
    // 参数会自动填充/裁短以匹配控制器要求的数据格式
    virtual bool send_velocity(const std::string& mapping, const std::vector<double>& velocity) = 0;

    virtual void command_queue_consumer_thread() = 0;
    
    // 速度控制器通常需要钩子状态来安全停止
    std::unordered_map<std::string, bool> needs_hook_state() const override { return {}; }

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

};

#endif
