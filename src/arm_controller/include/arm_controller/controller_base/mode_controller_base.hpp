#ifndef __MODE_CONTROLLER_HPP__
#define __MODE_CONTROLLER_HPP__

#include <any>
#include <string>


class ModeControllerBase {
public:
    explicit ModeControllerBase(std::string mode) : mode_(mode) {}
    virtual ~ModeControllerBase() = default;

    virtual void start(const std::string& mapping = "") { (void)mapping; is_active_ = true; }
    virtual bool stop(const std::string& mapping = "") { (void)mapping; is_active_ = false; return is_active_;}
    
    virtual void handle_message(std::any msg) = 0;

    bool is_active() const { return is_active_; }
    
    // 获取控制器模式名称
    std::string get_mode() const { return mode_; }
    
    // 检查是否需要钩子状态进行安全转移
    virtual bool needs_hook_state() const { return false; }

    // helper: normalize mapping ("" -> "single_arm")
    static std::string normalize_mapping(const std::string& mapping) {
        return mapping.empty() ? std::string("single_arm") : mapping;
    }
    
protected:
    bool is_active_ = false;

private:
    std::string mode_;

    
};


#endif // __MODE_CONTROLLER_HPP__

