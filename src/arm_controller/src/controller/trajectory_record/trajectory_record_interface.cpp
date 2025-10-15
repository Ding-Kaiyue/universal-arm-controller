#include "trajectory_record_controller.hpp"
#include "controller_interface/trajectory_record_interface.hpp"

std::unique_ptr<RecordControllerBase> createTrajectoryRecordController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<TrajectoryRecordController>(node);
}
