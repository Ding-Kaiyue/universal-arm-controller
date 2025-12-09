#include "cartesian_velocity_ipc_interface.hpp"
#include "arm_controller/ipc/command_producer.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"

namespace arm_controller::cartesian_velocity {

bool CartesianVelocityIPCInterface::execute(const std::vector<double>& cartesian_velocities,
                                            const std::string& mapping) {
    if (!ensureInitialized()) {
        return false;
    }

    auto val_result = ipc::CommandValidator::validateMapping(mapping);
    if (!val_result.valid) {
        setLastError(val_result.error_message);
        return false;
    }

    // 验证笛卡尔速度参数长度（应为6）
    if (cartesian_velocities.size() != 6) {
        setLastError("Cartesian velocities must have exactly 6 elements");
        return false;
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        setLastError("Failed to get state manager");
        return false;
    }

    // 转移到 CartesianVelocity 模式
    state_mgr->transitionToMode("CartesianVelocity");
    // 设置执行状态为待执行
    state_mgr->setExecutionState(ipc::ExecutionState::PENDING);

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    auto cmd = ipc::CommandBuilder()
        .withMode("CartesianVelocity")
        .withMapping(mapping)
        .withCartesianVelocities(cartesian_velocities)
        .build();

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push command: " + producer->getLastError());
        return false;
    }

    return true;
}

std::string CartesianVelocityIPCInterface::getCurrentMode(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return "";
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return "";
    }

    return state_mgr->getCurrentMode();
}

ipc::ExecutionState CartesianVelocityIPCInterface::getExecutionState(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return ipc::ExecutionState::FAILED;
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return ipc::ExecutionState::FAILED;
    }

    return state_mgr->getExecutionState();  
}

}  // namespace arm_controller::cartesian_velocity