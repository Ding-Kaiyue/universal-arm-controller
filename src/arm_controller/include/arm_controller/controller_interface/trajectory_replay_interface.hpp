#ifndef __TRAJECTORY_REPLAY_INTERFACE_HPP__
#define __TRAJECTORY_REPLAY_INTERFACE_HPP__

#include <controller_base/record_controller_base.hpp>
#include <memory>

std::unique_ptr<RecordControllerBase> createTrajectoryReplayController(const rclcpp::Node::SharedPtr &node);

#endif    // __TRAJECTORY_REPLAY_INTERFACE_HPP__
