#include "trajectory_replay_controller.hpp"
#include "controller_interface/trajectory_replay_interface.hpp"

std::unique_ptr<TeachControllerBase> createTrajectoryReplayController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<TrajectoryReplayController>(node);
}
