#ifndef __POINT_REPLAY_INTERFACE_HPP__
#define __POINT_REPLAY_INTERFACE_HPP__

#include <controller_base/record_controller_base.hpp>
#include <memory>

std::unique_ptr<RecordControllerBase> createPointReplayController(const rclcpp::Node::SharedPtr &node);

#endif    // __POINT_REPLAY_INTERFACE_HPP__
