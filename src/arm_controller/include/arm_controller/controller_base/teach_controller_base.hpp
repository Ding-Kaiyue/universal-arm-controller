#ifndef __TEACH_CONTROLLER_BASE_HPP__
#define __TEACH_CONTROLLER_BASE_HPP__

#include "arm_controller/controller_base/mode_controller_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <any>

class TeachControllerBase : public ModeControllerBase {
public:
    explicit TeachControllerBase(std::string mode, rclcpp::Node::SharedPtr node)
        : ModeControllerBase(mode), node_(node) {}
    virtual ~TeachControllerBase() = default;

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
        subscriptions_[mapping] = node_->create_subscription<std_msgs::msg::String>(
            input_topic, rclcpp::QoS(10).reliable(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (!is_active_) return;
                teach_callback(msg);
            }
        );

        RCLCPP_INFO(node_->get_logger(), "[%s] Subscribed to topic: %s (mapping: %s)",
                   get_mode().c_str(), input_topic.c_str(), mapping.c_str());
    }

    virtual void teach_callback(const std_msgs::msg::String::SharedPtr msg) = 0;
    // void handle_message(std::any /*msg*/) override final {}

    void start(const std::string& mapping = "") override = 0;
    bool stop(const std::string& mapping = "") override = 0;

    // 记录复现功能默认需要钩子状态来安全停止
    bool needs_hook_state() const override { return true; }
protected:
    rclcpp::Node::SharedPtr node_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;

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

#endif // __TEACH_CONTROLLER_BASE_HPP__
