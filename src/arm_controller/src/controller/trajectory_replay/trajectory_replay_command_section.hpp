#ifndef __TRAJECTORY_REPLAY_COMMAND_SECTION_HPP__
#define __TRAJECTORY_REPLAY_COMMAND_SECTION_HPP__

#include "arm_controller/hardware/hardware_manager.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace trajectory_replay_command_section {

// Shared command-loop timing constants for TrajectoryReplay command processing.
constexpr int kQueuePopTimeoutMs = 10;
constexpr int kPendingRetrySleepMs = 50;
constexpr int kResultStateHoldMs = 100;

std::vector<std::string> resolve_target_mappings(
    const std::string& mapping,
    const std::string& action,
    const std::shared_ptr<HardwareManager>& hardware_manager,
    std::mutex& state_mutex,
    const std::map<std::string, bool>& replaying_mappings);

bool ensure_mode_ready_for_start(
    const std::vector<std::string>& target_mappings,
    const rclcpp::Logger& logger,
    const std::function<void(const std::string&, const std::string&)>& hook_request_callback);

void set_execution_state_for_mappings(
    const std::vector<std::string>& target_mappings,
    arm_controller::ipc::ExecutionState state);

void finalize_command_states(
    const std::vector<std::string>& target_mappings,
    bool success,
    const std::string& action);

}  // namespace trajectory_replay_command_section

#endif  // __TRAJECTORY_REPLAY_COMMAND_SECTION_HPP__
