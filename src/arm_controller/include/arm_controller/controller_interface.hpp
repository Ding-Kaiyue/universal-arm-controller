#ifndef __CONTROLLER_INTERFACE_HPP__
#define __CONTROLLER_INTERFACE_HPP__

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "arm_controller/controller_base/mode_controller_base.hpp"

class ModeControllerBase;

class ControllerInterface {
public:
    using Creator = std::function<std::shared_ptr<ModeControllerBase>(rclcpp::Node::SharedPtr)>;

    // 单例获取接口
    static ControllerInterface& instance() {
        static ControllerInterface instance;
        return instance;
    }

    // 注册控制器类，返回是否成功（false表示已存在）
    bool register_class(const std::string& name, Creator creator) {
        return registry_.emplace(name, creator).second;
    }

    // 通过名称创建控制器实例，找不到返回nullptr
    std::shared_ptr<ModeControllerBase> create(const std::string& name, rclcpp::Node::SharedPtr node) {
        auto it = registry_.find(name);
        if (it != registry_.end()) {
            return it->second(node);
        }
        return nullptr;
    }

    // 获取所有注册类的map，只读访问
    const std::unordered_map<std::string, Creator>& get_all() const {
        return registry_;
    }

private:
    ControllerInterface() = default;
    ~ControllerInterface() = default;

    ControllerInterface(const ControllerInterface&) = delete;
    ControllerInterface& operator=(const ControllerInterface&) = delete;

    std::unordered_map<std::string, Creator> registry_;
};


#endif    // __CONTROLLER_INTERFACE_HPP__
