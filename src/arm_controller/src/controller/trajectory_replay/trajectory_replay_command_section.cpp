#include "trajectory_replay_controller.hpp"
#include "trajectory_replay_command_section.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <filesystem>
#include <cstring>

namespace trajectory_replay_command_section {

std::vector<std::string> resolve_target_mappings(
    const std::string& mapping,
    const std::string& action,
    const std::shared_ptr<HardwareManager>& hardware_manager,
    std::mutex& state_mutex,
    const std::map<std::string, bool>& replaying_mappings) {
    std::vector<std::string> target_mappings;

    if (action == "start") {
        if (mapping.empty() || mapping == "*") {
            if (hardware_manager) {
                target_mappings = hardware_manager->get_all_mappings();
            }
        } else {
            target_mappings.push_back(mapping);
        }
        return target_mappings;
    }

    if (!mapping.empty() && mapping != "*") {
        target_mappings.push_back(mapping);
        return target_mappings;
    }

    std::lock_guard<std::mutex> lock(state_mutex);
    for (const auto& [m, _] : replaying_mappings) {
        target_mappings.push_back(m);
    }
    return target_mappings;
}

bool ensure_mode_ready_for_start(
    const std::vector<std::string>& target_mappings,
    const rclcpp::Logger& logger,
    const std::function<void(const std::string&, const std::string&)>& hook_request_callback) {
    bool has_hook = false;

    for (const auto& target_mapping : target_mappings) {
        std::lock_guard<std::mutex> execution_lock(
            arm_controller::CommandQueueIPC::getMappingExecutionMutex(target_mapping));
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(target_mapping);

        if (!state_mgr) {
            continue;
        }

        state_mgr->transitionToMode("TrajectoryReplay");
        if (state_mgr->isInHookState()) {
            RCLCPP_DEBUG(logger,
                         "[%s] 🛑 TrajectoryReplay in hook state - waiting for transition",
                         target_mapping.c_str());
            if (hook_request_callback) {
                hook_request_callback(target_mapping, "TrajectoryReplay");
            }
            has_hook = true;
            break;
        }
    }

    if (has_hook) {
        arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
        return false;
    }

    return true;
}

void set_execution_state_for_mappings(
    const std::vector<std::string>& target_mappings,
    arm_controller::ipc::ExecutionState state) {
    for (const auto& target_mapping : target_mappings) {
        std::lock_guard<std::mutex> execution_lock(
            arm_controller::CommandQueueIPC::getMappingExecutionMutex(target_mapping));
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(target_mapping);
        if (state_mgr) {
            state_mgr->setExecutionState(state);
        }
    }
}

void finalize_command_states(
    const std::vector<std::string>& target_mappings,
    bool success,
    const std::string& action) {
    std::this_thread::sleep_for(std::chrono::milliseconds(kResultStateHoldMs));
    bool should_idle = (action == "cancel" || action == "complete");

    for (const auto& target_mapping : target_mappings) {
        std::lock_guard<std::mutex> execution_lock(
            arm_controller::CommandQueueIPC::getMappingExecutionMutex(target_mapping));
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(target_mapping);
        if (!state_mgr) {
            continue;
        }

        state_mgr->setExecutionState(
            success ? arm_controller::ipc::ExecutionState::SUCCESS
                    : arm_controller::ipc::ExecutionState::FAILED);

        if (should_idle) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kResultStateHoldMs));
            state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);

            arm_controller::ipc::ExecutorControllerState executor_state;
            strncpy(executor_state.current_mode, "TrajectoryReplay",
                    sizeof(executor_state.current_mode) - 1);
            executor_state.current_mode[sizeof(executor_state.current_mode) - 1] = '\0';
            executor_state.execution_state = (int)arm_controller::ipc::ExecutionState::IDLE;
            state_mgr->updateFromExecutor(executor_state);
        }
    }
}

}  // namespace trajectory_replay_command_section

void TrajectoryReplayController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;

    while (consumer_running_) {
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(
                cmd, "TrajectoryReplay", trajectory_replay_command_section::kQueuePopTimeoutMs)) {
            continue;
        }

        std::string mapping = cmd.get_mapping();
        std::string filename = cmd.get_filename();
        std::string action = cmd.get_action();

        auto target_mappings = trajectory_replay_command_section::resolve_target_mappings(
            mapping, action, hardware_manager_, state_mutex_, replaying_mappings_);
        if (target_mappings.empty()) {
            if (action == "start") {
                RCLCPP_WARN(node_->get_logger(), "❎ TrajectoryReplay: No mappings to process for start");
            } else {
                RCLCPP_WARN(node_->get_logger(), "❎ TrajectoryReplay: No active replay session");
            }
            continue;
        }

        try {
            if (action == "start" &&
                !trajectory_replay_command_section::ensure_mode_ready_for_start(
                    target_mappings, node_->get_logger(), hook_request_callback_)) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(trajectory_replay_command_section::kPendingRetrySleepMs));
                continue;
            }

            trajectory_replay_command_section::set_execution_state_for_mappings(
                target_mappings, arm_controller::ipc::ExecutionState::EXECUTING);

            bool success = true;
            if (action == "start") {
                std::string file_path = replay_dir_ + "/" + filename + "_smooth.csv";
                if (!std::filesystem::exists(file_path)) {
                    RCLCPP_ERROR(node_->get_logger(), "❎ File not found: %s", file_path.c_str());
                    success = false;
                } else if (replaying_) {
                    RCLCPP_WARN(node_->get_logger(), "⚠️ Already replaying, ignoring new command");
                    success = false;
                } else {
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        replaying_mappings_.clear();
                        for (const auto& target_mapping : target_mappings) {
                            replaying_mappings_[target_mapping] = true;
                        }
                    }

                    replaying_ = true;
                    paused_ = false;

                    if (replay_thread_ && replay_thread_->joinable()) {
                        replaying_ = false;
                        replay_thread_->join();
                    }

                    replay_thread_ = std::make_unique<std::thread>([this, file_path]() {
                        this->replay_thread_func(file_path);
                    });

                    RCLCPP_INFO(node_->get_logger(), "✅ Started replay for %zu mappings: %s",
                                target_mappings.size(), file_path.c_str());
                }
            } else if (!execute(target_mappings[0], action, filename)) {
                RCLCPP_WARN(node_->get_logger(), "⚠️ TrajectoryReplay: %s command failed", action.c_str());
                success = false;
            }

            trajectory_replay_command_section::finalize_command_states(
                target_mappings, success, action);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Exception in TrajectoryReplay command execution: %s", e.what());
        }

        arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
    }
}
