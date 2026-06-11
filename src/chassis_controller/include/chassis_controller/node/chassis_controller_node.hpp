#pragma once

#include <rclcpp/rclcpp.hpp>

#include "chassis_controller/controller/chassis_cmd_vel_controller.hpp"

namespace chassis_controller {

class ChassisControllerNode final : public rclcpp::Node {
public:
    ChassisControllerNode();
    ~ChassisControllerNode() override;

private:
    ChassisController controller_;
};

}  // namespace chassis_controller
