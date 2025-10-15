#ifndef __ROS2_ACTION_CONTROL_CONTROLLER_HPP__
#define __ROS2_ACTION_CONTROL_CONTROLLER_HPP__

#include "controller_base/utility_controller_base.hpp"
#include "hardware/hardware_manager.hpp"

class ROS2ActionControlController final : public UtilityControllerBase {
public:
    explicit ROS2ActionControlController(const rclcpp::Node::SharedPtr& node);
    ~ROS2ActionControlController() override;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

    // 重写需要hook状态的方法 - ROS2动作控制需要安全停止
    bool needs_hook_state() const override { return true; }

private:
    std::shared_ptr<HardwareManager> hardware_manager_;
};

#endif  // __ROS2_ACTION_CONTROL_CONTROLLER_HPP__