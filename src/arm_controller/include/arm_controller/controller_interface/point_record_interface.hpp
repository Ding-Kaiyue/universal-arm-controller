#ifndef __POINT_RECORD_INTERFACE_HPP__
#define __POINT_RECORD_INTERFACE_HPP__

#include <controller_base/teach_controller_base.hpp>
#include <memory>

std::unique_ptr<TeachControllerBase> createPointRecordController(const rclcpp::Node::SharedPtr &node);

#endif    // __POINT_RECORD_INTERFACE_HPP__
