#include "trajectory_record_ipc_interface.hpp"
#include "arm_controller/ipc/command_producer.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"

namespace arm_controller::trajectory_record {

bool TrajectoryRecordIPCInterface::startRecording(const std::string& file_name, const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    // 验证文件名不为空
    if (file_name.empty()) {
        setLastError("File name cannot be empty");
        return false;
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    // 构建命令
    // mapping 可以是：
    // - "*" 或 "" : 对所有 active mappings 操作
    // - "left_arm", "right_arm" 等: 对特定 mapping 操作
    std::string target_mapping = mapping;
    if (!mapping.empty() && mapping != "*") {
        auto val_result = ipc::CommandValidator::validateMapping(mapping);
        if (!val_result.valid) {
            setLastError(val_result.error_message);
            return false;
        }
    }

    // 如果是 "*"，转换为空字符串在 IPC 中表示"所有mappings"
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryRecord")
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

bool TrajectoryRecordIPCInterface::pauseRecording(const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
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
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryRecord")
        .withMapping(target_mapping)
        .build();
    cmd.set_action("pause");

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push pause command: " + producer->getLastError());
        return false;
    }

    return true;
}

bool TrajectoryRecordIPCInterface::resumeRecording(const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
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
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryRecord")
        .withMapping(target_mapping)
        .build();
    cmd.set_action("resume");

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push resume command: " + producer->getLastError());
        return false;
    }

    return true;
}

bool TrajectoryRecordIPCInterface::stopRecording(const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
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
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryRecord")
        .withMapping(target_mapping)
        .build();
    cmd.set_action("complete");

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push complete command: " + producer->getLastError());
        return false;
    }

    return true;
}

bool TrajectoryRecordIPCInterface::cancelRecording(const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
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
    if (mapping == "*") {
        target_mapping = "";
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("TrajectoryRecord")
        .withMapping(target_mapping)
        .build();
    cmd.set_action("cancel");

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push cancel command: " + producer->getLastError());
        return false;
    }

    return true;
}

std::string TrajectoryRecordIPCInterface::getCurrentMode(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return "";
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return "";
    }

    return state_mgr->getCurrentMode();
}

ipc::ExecutionState TrajectoryRecordIPCInterface::getExecutionState() const {
    if (!ensureInitialized()) {
        return ipc::ExecutionState::FAILED;
    }

    // 查询所有active mappings中的任意一个执行状态
    auto left_state = getStateManager("left_arm");
    if (left_state) {
        return left_state->getExecutionState();
    }

    auto right_state = getStateManager("right_arm");
    if (right_state) {
        return right_state->getExecutionState();
    }

    return ipc::ExecutionState::FAILED;
}

bool TrajectoryRecordIPCInterface::isRecording() const {
    if (!ensureInitialized()) {
        return false;
    }

    // 检查是否有任何mapping在录制
    auto left_state = getStateManager("left_arm");
    if (left_state && left_state->getCurrentMode() == "TrajectoryRecord" &&
        left_state->getExecutionState() == ipc::ExecutionState::EXECUTING) {
        return true;
    }

    auto right_state = getStateManager("right_arm");
    if (right_state && right_state->getCurrentMode() == "TrajectoryRecord" &&
        right_state->getExecutionState() == ipc::ExecutionState::EXECUTING) {
        return true;
    }

    return false;
}

}   // namespace arm_controller::trajectory_record
