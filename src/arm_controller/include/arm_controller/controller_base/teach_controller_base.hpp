#ifndef __TEACH_CONTROLLER_BASE_HPP__
#define __TEACH_CONTROLLER_BASE_HPP__

#include "arm_controller/controller_base/mode_controller_base.hpp"
#include <trajectory_interpolator/trajectory_interpolator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_interfaces/msg/teaching_control.hpp>
#include <any>
#include <functional>
#include "arm_controller/hardware/hardware_manager.hpp"

class TeachControllerBase : public ModeControllerBase {
public:
    explicit TeachControllerBase(std::string mode, rclcpp::Node::SharedPtr node)
        : ModeControllerBase(mode), node_(node) {}
    virtual ~TeachControllerBase() = default;

    void init_subscriptions(const std::string& mapping) {
        if (mapping.empty()) return;

        // ✅ TeachControllerBase 只有一个 input_topic，使用 TeachingControl 消息
        // TeachingControl 包含：mapping、action、filename 所有信息

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

        // 创建订阅 - 单个话题订阅 TeachingControl 消息
        subscriptions_[mapping] = node_->create_subscription<controller_interfaces::msg::TeachingControl>(
            input_topic, rclcpp::QoS(10).reliable(),
            [this, mapping](const controller_interfaces::msg::TeachingControl::SharedPtr msg) {
                if (!is_active(mapping)) return;
                teach_callback(msg);
            }
        );

        RCLCPP_INFO(node_->get_logger(), "[%s] Subscribed to teaching control topic: %s (mapping: %s)",
                   get_mode().c_str(), input_topic.c_str(), mapping.c_str());
    }

    virtual void teach_callback(const controller_interfaces::msg::TeachingControl::SharedPtr msg) = 0;

    // 直接执行示教命令 - 通过teach_callback调用
    // command: "start"、"pause"、"resume"、"cancel"、"complete"
    virtual bool execute(const std::string& mapping, const std::string& command, const std::string& filename) = 0;

    virtual void pause() = 0;
    virtual void resume() = 0;
    virtual void cancel() = 0;
    virtual void complete() = 0;

    virtual void command_queue_consumer_thread() = 0;

    // 与 VelocityController 一致：请求 ControllerManager 发起模式切换
    void set_hook_request_callback(
        std::function<void(const std::string&, const std::string&)> callback) {
        hook_request_callback_ = std::move(callback);
    }
    
    // 只有轨迹复现（TrajectoryRecord/Replay）需要钩子状态来安全停止
    std::unordered_map<std::string, bool> needs_hook_state() const override {
        std::unordered_map<std::string, bool> result;
        std::string mode = get_mode();

        // 仅这两个模式需要钩子状态
        if (mode != "TrajectoryRecord" && mode != "TrajectoryReplay") {
            return result;  // 返回空 map
        }

        std::lock_guard<std::mutex> lock(active_mappings_mutex_);
        for (const auto& [mapping, is_active] : active_mappings_) {
            if (is_active) {
                result[mapping] = true;
            }
        }
        return result;
    }

protected:
    // 设置示教模式标志 - 防止安全限位检查触发急停
    void enable_teaching_mode() {
        auto hw_manager = HardwareManager::getInstance();
        if (hw_manager) {
            hw_manager->set_teaching_mode(true);
            RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Teaching mode enabled - safety checks disabled",
                       get_mode().c_str());
        }
    }

    void disable_teaching_mode() {
        auto hw_manager = HardwareManager::getInstance();
        if (hw_manager) {
            hw_manager->set_teaching_mode(false);
            RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Teaching mode disabled - safety checks re-enabled",
                       get_mode().c_str());
        }
    }
    rclcpp::Node::SharedPtr node_;
    std::function<void(const std::string&, const std::string&)> hook_request_callback_;
    std::map<std::string, rclcpp::Subscription<controller_interfaces::msg::TeachingControl>::SharedPtr> subscriptions_;

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

    // ✅ 加载插值器配置的辅助方法
    void load_interpolator_config(TrajectoryInterpolator& interpolator) {
        try {
            std::string pkg_path = ament_index_cpp::get_package_share_directory("arm_controller");
            std::string config_path = pkg_path + "/config/interpolator_config.yaml";
            YAML::Node config = YAML::LoadFile(config_path);

            if (config["interpolation"]["default"]) {
                auto default_config = config["interpolation"]["default"];
                trajectory_interpolator::SplineConfig spline_config;

                if (default_config["target_dt"]) {
                    spline_config.target_dt = default_config["target_dt"].as<double>();
                }
                if (default_config["max_velocity"]) {
                    spline_config.max_velocity = default_config["max_velocity"].as<double>();
                }
                if (default_config["max_acceleration"]) {
                    spline_config.max_acceleration = default_config["max_acceleration"].as<double>();
                }
                if (default_config["max_jerk"]) {
                    spline_config.max_jerk = default_config["max_jerk"].as<double>();
                }

                interpolator.setInterpolationConfig(spline_config);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  Failed to load interpolator config: %s, using defaults",
                       get_mode().c_str(), e.what());
        }
    }
};

#endif // __TEACH_CONTROLLER_BASE_HPP__
