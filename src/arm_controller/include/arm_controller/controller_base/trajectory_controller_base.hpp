#ifndef __TRAJECTORY_CONTROLLER_BASE_HPP__
#define __TRAJECTORY_CONTROLLER_BASE_HPP__

#include <arm_controller/controller_base/mode_controller_base.hpp>
#include <trajectory_interpolator/trajectory_interpolator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <any>
#include <vector>
#include <string>
#include <map>
#include <arm_controller/hardware/hardware_manager.hpp>

class TrajectoryControllerBase : public ModeControllerBase {
public:
    explicit TrajectoryControllerBase(std::string mode) : ModeControllerBase(mode) {}
    virtual ~TrajectoryControllerBase() = default;
};

template<typename T>
class TrajectoryControllerImpl : public TrajectoryControllerBase {
public:
    explicit TrajectoryControllerImpl(std::string mode, rclcpp::Node::SharedPtr node) 
        : TrajectoryControllerBase(mode), node_(node) {}
    virtual ~TrajectoryControllerImpl() override = default;

    // 初始化订阅 - 为指定的 mapping 创建话题订阅
    // 在 controller 创建后、start() 调用前执行
    void init_subscriptions(const std::string& mapping) override {
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

        // 创建订阅（使用映射作为键存储多个订阅）
        subscriptions_[mapping] = node_->create_subscription<T>(
            input_topic, rclcpp::QoS(10).reliable(),
            [this, mapping](const typename T::SharedPtr msg) {
                if (!is_active(mapping)) return;
                trajectory_callback(mapping, msg);
            }
        );

        RCLCPP_INFO(node_->get_logger(), "[%s] Subscribed to topic: %s (mapping: %s)",
                   get_mode().c_str(), input_topic.c_str(), mapping.c_str());
    }

    virtual void plan_and_execute(const std::string& mapping, const typename T::SharedPtr msg) = 0;

    virtual void trajectory_callback(const std::string& mapping, const typename T::SharedPtr msg) = 0;

    // 直接执行轨迹命令 - 通过 IPC 命令队列消费线程调用
    // 参数会自动填充/裁短以匹配控制器要求的数据格式
    virtual bool execute(const std::string& mapping, const std::vector<double>& parameters) = 0;
    
    virtual void command_queue_consumer_thread() = 0;
    
    // ✅ 轨迹控制器需要钩子状态来安全停止 - 返回所有活跃的 mapping
    // 如果轨迹正在执行并被要求停止，应该进入 hook 状态确保安全停止
    std::unordered_map<std::string, bool> needs_hook_state() const override {
        std::unordered_map<std::string, bool> result;
        std::lock_guard<std::mutex> lock(active_mappings_mutex_);
        for (const auto& [mapping, is_active] : active_mappings_) {
            if (is_active) {
                result[mapping] = true;
            }
        }
        return result;
    }

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

    // 加载插值器配置的辅助方法
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

#endif // __TRAJECTORY_CONTROLLER_BASE_HPP__