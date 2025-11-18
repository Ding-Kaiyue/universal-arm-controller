#include "movec_controller.hpp"
#include "controller_interface/movec_interface.hpp"

std::unique_ptr<TrajectoryControllerBase> createMoveCController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<MoveCController>(node);
}
