#include "reactive_task_controller.hpp"
#include "controller_interface/reactive_task_interface.hpp"

std::unique_ptr<TrajectoryControllerBase> createReactiveTaskController(
    const rclcpp::Node::SharedPtr& node) {
    return std::make_unique<ReactiveTaskController>(node);
}
