#include "movej_controller.hpp"
#include "controller_interface/movej_interface.hpp"

std::unique_ptr<TrajectoryControllerBase> createMoveJController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<MoveJController>(node);
}
