#include "point_replay_controller.hpp"
#include "controller_interface/point_replay_interface.hpp"

std::unique_ptr<TeachControllerBase> createPointReplayController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<PointReplayController>(node);
}