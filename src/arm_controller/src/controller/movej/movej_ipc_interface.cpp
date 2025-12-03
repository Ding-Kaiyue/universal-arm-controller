#include "movej_ipc_interface.hpp"
#include "arm_controller/ipc/command_producer.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"

namespace arm_controller::movej {

bool MoveJIPCInterface::execute(const std::vector<double>& joint_positions,
                                const std::string& mapping) {
    if (!ensureInitialized()) {
        return false;
    }

    auto val_result = ipc::CommandValidator::validateMapping(mapping);
    if (!val_result.valid) {
        setLastError(val_result.error_message);
        return false;
    }

    val_result = ipc::CommandValidator::validateJointPositions(joint_positions);
    if (!val_result.valid) {
        setLastError(val_result.error_message);
        return false;
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        setLastError("Failed to get state manager");
        return false;
    }

    // 转移到 MoveJ 模式
    state_mgr->transitionToMode("MoveJ");
    // 设置执行状态为待执行
    state_mgr->setExecutionState(ipc::ExecutionState::PENDING);

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("MoveJ")
        .withMapping(mapping)
        .withJointPositions(joint_positions)
        .build();

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push command: " + producer->getLastError());
        return false;
    }

    return true;
}

std::string MoveJIPCInterface::getCurrentMode(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return "";
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return "";
    }

    return state_mgr->getCurrentMode();
}

ipc::ExecutionState MoveJIPCInterface::getExecutionState(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return ipc::ExecutionState::FAILED;
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return ipc::ExecutionState::FAILED;
    }

    return state_mgr->getExecutionState();
}

std::vector<double> MoveJIPCInterface::getCurrentJointPositions(
    const std::string& mapping [[maybe_unused]]) const {
    if (!ensureInitialized()) {
        return {};
    }

    return {};
}

}  // namespace arm_controller::movej
