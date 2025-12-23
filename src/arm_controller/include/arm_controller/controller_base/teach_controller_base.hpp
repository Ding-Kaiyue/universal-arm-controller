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

        // ✅ TeachControllerBase 总是有两个 input_topic：
        // - input_topic0: 录制命令（开始/停止）
        // - input_topic1: 控制命令（暂停/恢复/取消/完成）

        std::string input_topic0, input_topic1;
        node_->get_parameter("controllers." + get_mode() + ".input_topic0", input_topic0);
        node_->get_parameter("controllers." + get_mode() + ".input_topic1", input_topic1);

        // 替换 {mapping} 占位符
        size_t pos = input_topic0.find("{mapping}");
        if (pos != std::string::npos) {
            input_topic0.replace(pos, 9, mapping);
        }

        pos = input_topic1.find("{mapping}");
        if (pos != std::string::npos) {
            input_topic1.replace(pos, 9, mapping);
        }

        // 创建订阅 - 两个话题都使用同一个 callback
        subscriptions_[mapping + "0"] = node_->create_subscription<std_msgs::msg::String>(
            input_topic0, rclcpp::QoS(10).reliable(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (!is_active_) return;
                teach_callback(msg);
            }
        );

        subscriptions_[mapping + "1"] = node_->create_subscription<std_msgs::msg::String>(
            input_topic1, rclcpp::QoS(10).reliable(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (!is_active_) return;
                on_teaching_control(msg);
            }
        );

        RCLCPP_INFO(node_->get_logger(), "[%s] Subscribed to filename input topic: %s (mapping: %s)",
                   get_mode().c_str(), input_topic0.c_str(), mapping.c_str());
        RCLCPP_INFO(node_->get_logger(), "[%s] Subscribed to intermediate control topic: %s (mapping: %s)",
                   get_mode().c_str(), input_topic1.c_str(), mapping.c_str());
    }

    virtual void teach_callback(const std_msgs::msg::String::SharedPtr msg) = 0;
    virtual void on_teaching_control(const std_msgs::msg::String::SharedPtr msg) = 0;

    void start(const std::string& mapping = "") override = 0;
    bool stop(const std::string& mapping = "") override = 0;

    virtual void pause(const std::string& mapping = "") = 0;
    virtual void resume(const std::string& mapping = "") = 0;
    virtual void cancel(const std::string& mapping = "") = 0;
    virtual void complete(const std::string& mapping = "") = 0;
    
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
