#ifndef __VELOCITY_CONTROLLER_BASE_HPP__
#define __VELOCITY_CONTROLLER_BASE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <any>
#include <vector>
#include <string>
#include <map>
#include "arm_controller/hardware/hardware_manager.hpp"
#include "arm_controller/controller_base/mode_controller_base.hpp"


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
    virtual ~VelocityControllerImpl() override = default;
    
    // 直接执行轨迹命令 - 通过 IPC 命令队列消费线程调用
    // 参数会自动填充/裁短以匹配控制器要求的数据格式
    virtual bool move(const std::string& mapping, const std::vector<double>& parameters) = 0;

    // 速度控制器通常需要钩子状态来安全停止
    bool needs_hook_state() const override { return true; }

protected:
    rclcpp::Node::SharedPtr node_;

};

#endif  // VELOCITY_CONTROLLER_BASE_HPP__