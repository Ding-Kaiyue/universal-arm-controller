#ifndef __MODE_CONTROLLER_HPP__
#define __MODE_CONTROLLER_HPP__

#include <any>
#include <string>
#include <unordered_map>


class ModeControllerBase {
public:
    explicit ModeControllerBase(std::string mode) : mode_(mode) {}
    virtual ~ModeControllerBase() = default;

    virtual void start(const std::string& mapping = "") {
        std::string normalized = normalize_mapping(mapping);
        is_active_[normalized] = true;
    }
    virtual bool stop(const std::string& mapping = "") {
        std::string normalized = normalize_mapping(mapping);
        is_active_[normalized] = false;
        return !is_active_[normalized];
    }

    // virtual void handle_message(std::any msg) = 0;

    bool is_active(const std::string& mapping = "") const {
        std::string normalized = normalize_mapping(mapping);
        auto it = is_active_.find(normalized);
        return it != is_active_.end() ? it->second : false;
    }

    // 获取控制器模式名称
    std::string get_mode() const { return mode_; }

    // 检查是否需要钩子状态进行安全转移
    virtual bool needs_hook_state() const { return false; }

    // helper: normalize mapping ("" -> "single_arm")
    static std::string normalize_mapping(const std::string& mapping) {
        return mapping.empty() ? std::string("single_arm") : mapping;
    }

protected:
    std::unordered_map<std::string, bool> is_active_;

private:
    std::string mode_;


};


#endif // __MODE_CONTROLLER_HPP__

