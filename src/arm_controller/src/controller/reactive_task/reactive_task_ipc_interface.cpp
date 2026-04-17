#include "reactive_task_ipc_interface.hpp"

#include "arm_controller/ipc/command_producer.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"

namespace arm_controller::reactive_task {

bool ReactiveTaskIPCInterface::execute(
    const std::vector<double>& target_pose,
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

    if (target_pose.size() != 7) {
        setLastError("target_pose must have 7 elements: [x,y,z,qx,qy,qz,qw]");
        return false;
    }

    val_result = ipc::CommandValidator::validatePose(
        target_pose[0],
        target_pose[1],
        target_pose[2],
        target_pose[3],
        target_pose[4],
        target_pose[5],
        target_pose[6]);
    if (!val_result.valid) {
        setLastError(val_result.error_message);
        return false;
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        setLastError("Failed to get state manager");
        return false;
    }

    state_mgr->initializeCurrentMode("ReactiveTask");
    state_mgr->setExecutionState(ipc::ExecutionState::PENDING);

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    auto cmd = ipc::CommandBuilder()
                   .withMode("ReactiveTask")
                   .withMapping(mapping)
                   .withPose(
                       target_pose[0],
                       target_pose[1],
                       target_pose[2],
                       target_pose[3],
                       target_pose[4],
                       target_pose[5],
                       target_pose[6])
                   .build();

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push command: " + producer->getLastError());
        return false;
    }
    return true;
}

std::string ReactiveTaskIPCInterface::getCurrentMode(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return "";
    }
    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return "";
    }
    return state_mgr->getCurrentMode();
}

ipc::ExecutionState ReactiveTaskIPCInterface::getExecutionState(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return ipc::ExecutionState::FAILED;
    }
    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return ipc::ExecutionState::FAILED;
    }
    return state_mgr->getExecutionState();
}

}  // namespace arm_controller::reactive_task
