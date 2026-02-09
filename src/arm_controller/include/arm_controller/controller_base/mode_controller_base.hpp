#ifndef __MODE_CONTROLLER_HPP__
#define __MODE_CONTROLLER_HPP__

#include <any>
#include <string>
#include <unordered_map>


class ModeControllerBase {
public:
    explicit ModeControllerBase(std::string mode) : mode_(mode) {}
    virtual ~ModeControllerBase() = default;

    // 初始化订阅 - 子类可以重写以创建话题订阅
    virtual void init_subscriptions(const std::string& mapping = "") { (void)mapping; }

    virtual void start(const std::string& mapping = "") {
        std::string normalized = normalize_mapping(mapping);
        active_mappings_[normalized] = true;
    }
    virtual bool stop(const std::string& mapping = "") {
        std::string normalized = normalize_mapping(mapping);
        active_mappings_[normalized] = false;
        return !active_mappings_[normalized];
    }

    bool is_active(const std::string& mapping = "") const {
        std::string normalized = normalize_mapping(mapping);
        auto it = active_mappings_.find(normalized);
        bool result = (it != active_mappings_.end()) ? it->second : false;
        // 追踪 is_active 的调用（仅在非DEBUG时）
        // RCLCPP_DEBUG(...)  可能无法访问 logger，所以注释掉
        return result;
    }
    
    // 获取控制器模式名称
    std::string get_mode() const { return mode_; }
    
    // 检查某 mapping 是否需要钩子状态进行安全转移
    virtual std::unordered_map<std::string, bool> needs_hook_state() const { return {}; }

    // helper: normalize mapping ("" -> "single_arm")
    static std::string normalize_mapping(const std::string& mapping) {
        return mapping.empty() ? std::string("single_arm") : mapping;
    }
    
protected:
    // 记录每个 mapping 的活跃状态: key=mapping名称, value=是否活跃
    std::unordered_map<std::string, bool> active_mappings_;

private:
    std::string mode_;

    
};


#endif // __MODE_CONTROLLER_HPP__

