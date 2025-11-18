#ifndef __TRAJECTORY_RECORD_INTERFACE_HPP__
#define __TRAJECTORY_RECORD_INTERFACE_HPP__

#include <controller_base/record_controller_base.hpp>
#include <memory>

std::unique_ptr<RecordControllerBase> createTrajectoryRecordController(const rclcpp::Node::SharedPtr &node);

#endif    // __TRAJECTORY_RECORD_INTERFACE_HPP__
