#include "joint_velocity_ipc_interface.hpp"
#include "arm_controller/ipc/command_producer.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"

namespace arm_controller::joint_velocity {

bool JointVelocityIPCInterface::execute(const std::vector<double>& joint_velocities,
                                        const std::string& mapping) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto val_result = ipc::CommandValidator::validateMapping(mapping);
    if (!val_result.valid) {
        setLastError(val_result.error_message);
        return false;
    }

    val_result = ipc::CommandValidator::validateJointVelocities(joint_velocities);
    if (!val_result.valid) {
        setLastError(val_result.error_message);
        return false;
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        setLastError("Failed to get state manager");
        return false;
    }

    // 直接初始化状态（清除 hook_state，设置当前模式）
    state_mgr->initializeCurrentMode("JointVelocity");
    // 设置执行状态为待执行
    state_mgr->setExecutionState(ipc::ExecutionState::PENDING);

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("JointVelocity")
        .withMapping(mapping)
        .withJointVelocities(joint_velocities)
        .build();

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push command: " + producer->getLastError());
        return false;
    }

    return true;
}

std::string JointVelocityIPCInterface::getCurrentMode(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return "";
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return "";
    }

    return state_mgr->getCurrentMode();
}

ipc::ExecutionState JointVelocityIPCInterface::getExecutionState(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return ipc::ExecutionState::FAILED;
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return ipc::ExecutionState::FAILED;
    }

    return state_mgr->getExecutionState();
}

}   // namespace arm_controller::joint_velocity
