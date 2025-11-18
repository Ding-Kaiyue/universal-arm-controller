#ifndef __POINT_RECORD_INTERFACE_HPP__
#define __POINT_RECORD_INTERFACE_HPP__

#include <controller_base/record_controller_base.hpp>
#include <memory>

std::unique_ptr<RecordControllerBase> createPointRecordController(const rclcpp::Node::SharedPtr &node);

#endif    // __POINT_RECORD_INTERFACE_HPP__
