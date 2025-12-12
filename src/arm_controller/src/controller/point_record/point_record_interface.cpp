#include "point_record_controller.hpp"
#include "controller_interface/point_record_interface.hpp"

std::unique_ptr<TeachControllerBase> createPointRecordController(const rclcpp::Node::SharedPtr &node) {
    return std::make_unique<PointRecordController>(node);
}
