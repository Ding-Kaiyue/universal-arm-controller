#ifndef __TRAJECTORY_CONTROLLER_BASE_HPP__
#define __TRAJECTORY_CONTROLLER_BASE_HPP__

#include <arm_controller/controller_base/mode_controller_base.hpp>
#include <rclcpp/rclcpp.hpp>
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

    // 直接执行轨迹命令 - 通过 IPC 命令队列消费线程调用
    // 参数会自动填充/裁短以匹配控制器要求的数据格式
    virtual bool move(const std::string& mapping, const std::vector<double>& parameters) = 0;

    // 轨迹控制器通常需要钩子状态来安全停止
    bool needs_hook_state() const override { return true; }
    
protected:
    // 通过订阅话题接收轨迹命令
    virtual void plan_and_execute(const std::string& mapping, const typename T::SharedPtr msg) = 0;

protected:
    rclcpp::Node::SharedPtr node_;
};

#endif  // TRAJECTORY_CONTROLLER_BASE_HPP_
