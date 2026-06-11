#include "chassis_controller/node/chassis_controller_node.hpp"

#include <memory>

#include "chassis_controller/config/chassis_controller_config.hpp"

namespace chassis_controller {

ChassisControllerNode::ChassisControllerNode()
    : rclcpp::Node("chassis_controller_node"),
      controller_(*this, loadChassisControllerConfig(*this)) {
    RCLCPP_INFO(this->get_logger(), "Initializing Chassis Controller Node");
    controller_.start();
}

ChassisControllerNode::~ChassisControllerNode() = default;

}  // namespace chassis_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<chassis_controller::ChassisControllerNode>());
    rclcpp::shutdown();
    return 0;
}
