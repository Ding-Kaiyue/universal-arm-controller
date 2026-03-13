#include "trajectory_replay_ipc_interface.hpp"
#include "arm_controller/ipc/command_producer.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"

namespace arm_controller::trajectory_replay {

bool TrajectoryReplayIPCInterface::startReplay(const std::string& file_name, const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    // 验证文件名不为空
    if (file_name.empty()) {
        setLastError("File name cannot be empty");
        return false;
    }

    // 支持 "*" 通配符表示所有 active mappings
    std::string target_mapping = mapping;
    if (!mapping.empty() && mapping != "*") {
        auto val_result = ipc::CommandValidator::validateMapping(mapping);
        if (!val_result.valid) {
            setLastError(val_result.error_message);
            return false;
        }
    }

    // 与 MoveJ/MoveL/MoveC 对齐：先表达模式切换需求，并置为 PENDING
    if (!mapping.empty() && mapping != "*") {
        auto state_mgr = getStateManager(mapping);
        if (!state_mgr) {
            setLastError("Failed to get state manager");
            return false;
        }
        state_mgr->transitionToMode("TrajectoryReplay");
        state_mgr->setExecutionState(ipc::ExecutionState::PENDING);
    } else {
        // "*" 场景下按双臂常见映射同步状态（兼容现有 left/right 体系）
        for (const auto& m : {"left_arm", "right_arm"}) {
            auto state_mgr = getStateManager(m);
            if (state_mgr) {
                state_mgr->transitionToMode("TrajectoryReplay");
                state_mgr->setExecutionState(ipc::ExecutionState::PENDING);
            }
        }
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    // 如果是 "*"，转换为空字符串在 IPC 中表示"所有mappings"
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryReplay")
        .withMapping(target_mapping)
        .build();

    // 设置文件名和 start 动作
    cmd.set_filename(file_name);
    cmd.set_action("start");

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push command: " + producer->getLastError());
        return false;
    }

    return true;
}

bool TrajectoryReplayIPCInterface::pauseReplay(const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    // 支持 "*" 通配符
    std::string target_mapping = mapping;
    if (!mapping.empty() && mapping != "*") {
        auto val_result = ipc::CommandValidator::validateMapping(mapping);
        if (!val_result.valid) {
            setLastError(val_result.error_message);
            return false;
        }
    }
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryReplay")
        .withMapping(target_mapping)
        .build();
    cmd.set_action("pause");

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push pause command: " + producer->getLastError());
        return false;
    }

    return true;
}

bool TrajectoryReplayIPCInterface::resumeReplay(const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    // 支持 "*" 通配符
    std::string target_mapping = mapping;
    if (!mapping.empty() && mapping != "*") {
        auto val_result = ipc::CommandValidator::validateMapping(mapping);
        if (!val_result.valid) {
            setLastError(val_result.error_message);
            return false;
        }
    }
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryReplay")
        .withMapping(target_mapping)
        .build();
    cmd.set_action("resume");

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push resume command: " + producer->getLastError());
        return false;
    }

    return true;
}

bool TrajectoryReplayIPCInterface::stopReplay(const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    // 支持 "*" 通配符
    std::string target_mapping = mapping;
    if (!mapping.empty() && mapping != "*") {
        auto val_result = ipc::CommandValidator::validateMapping(mapping);
        if (!val_result.valid) {
            setLastError(val_result.error_message);
            return false;
        }
    }
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryReplay")
        .withMapping(target_mapping)
        .build();
    cmd.set_action("complete");

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push complete command: " + producer->getLastError());
        return false;
    }

    return true;
}

bool TrajectoryReplayIPCInterface::cancelReplay(const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    // 支持 "*" 通配符
    std::string target_mapping = mapping;
    if (!mapping.empty() && mapping != "*") {
        auto val_result = ipc::CommandValidator::validateMapping(mapping);
        if (!val_result.valid) {
            setLastError(val_result.error_message);
            return false;
        }
    }
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryReplay")
        .withMapping(target_mapping)
        .build();
    cmd.set_action("cancel");

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push cancel command: " + producer->getLastError());
        return false;
    }

    return true;
}

std::string TrajectoryReplayIPCInterface::getCurrentMode(const std::string& mapping) const {
    auto state_mgr = getStateManager(mapping);
    return state_mgr ? state_mgr->getCurrentMode() : "";
}

ipc::ExecutionState TrajectoryReplayIPCInterface::getExecutionState(const std::string& mapping) const {
    auto state_mgr = getStateManager(mapping);
    return state_mgr ? state_mgr->getExecutionState() : ipc::ExecutionState::IDLE;
}

bool TrajectoryReplayIPCInterface::isReplaying(const std::string& mapping) const {
    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) return false;
    return state_mgr->getCurrentMode() == "TrajectoryReplay" &&
           state_mgr->getExecutionState() == ipc::ExecutionState::EXECUTING;
}

}   // namespace arm_controller::trajectory_replay
