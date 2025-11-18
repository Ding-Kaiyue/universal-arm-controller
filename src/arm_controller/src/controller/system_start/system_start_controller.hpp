#ifndef __SYSTEM_START_CONTROLLER_HPP__
#define __SYSTEM_START_CONTROLLER_HPP__

#include "controller_base/utility_controller_base.hpp"
#include "hardware/hardware_manager.hpp"
#include "utils/motor_mode.hpp"

class SystemStartController final : public UtilityControllerBase {
public:
    explicit SystemStartController(const rclcpp::Node::SharedPtr& node);
    ~SystemStartController() override;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

    // 不需要重写hook状态方法 - 系统启动控制器不需要安全停止（本身就是保持状态）
    
private:
    std::shared_ptr<HardwareManager> hardware_manager_;
};  // class SystemStartController

#endif // __SYSTEM_START_CONTROLLER_HPP__
