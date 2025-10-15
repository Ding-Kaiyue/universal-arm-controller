#include "movel_controller.hpp"
#include "controller_interface/movel_interface.hpp"

std::unique_ptr<TrajectoryControllerBase> createMoveLController(const rclcpp::Node::SharedPtr& node) {
    return std::make_unique<MoveLController>(node);
}
