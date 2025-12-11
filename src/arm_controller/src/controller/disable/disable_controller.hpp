#ifndef __DISABLE_CONTROLLER_HPP__
#define __DISABLE_CONTROLLER_HPP__

#include "controller_base/utility_controller_base.hpp"
#include "hardware/hardware_manager.hpp"

class DisableController final : public UtilityControllerBase {
public: 
    explicit DisableController(const rclcpp::Node::SharedPtr& node);
    ~DisableController() override;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

private:
    bool disable_motors(const std::string& mapping);
    std::shared_ptr<HardwareManager> hardware_manager_;

};

#endif  // __DISABLE_CONTROLLER_HPP__
