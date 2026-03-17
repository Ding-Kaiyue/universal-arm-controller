#include "basic_ops_ipc_interface.hpp"
#include "arm_controller/ipc/command_producer.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"
#include "hardware_driver/driver/gripper_driver_interface.hpp"
#include <algorithm>
#include <vector>

namespace arm_controller::basic_ops {

bool BasicOpsIPCInterface::push_basic_ops_command(const std::string& op_mode,
                                                  const std::string& mapping,
                                                  const std::vector<double>& params) {
    if (!ensureInitialized()) {
        setLastError("IPC not initialized");
        return false;
    }

    auto map_result = ipc::CommandValidator::validateMapping(mapping);
    if (!map_result.valid) {
        setLastError(map_result.error_message);
        return false;
    }

    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        setLastError("Failed to get state manager");
        return false;
    }

    // BasicOps 不触发模式切换，只更新执行状态
    state_mgr->setExecutionState(ipc::ExecutionState::PENDING);

    auto producer = getCommandProducer();
    if (!producer) {
        setLastError("Failed to get command producer");
        return false;
    }

    auto cmd = ipc::CommandBuilder()
        .withMode(op_mode)
        .withMapping(mapping)
        .withJointPositions(params)
        .build();

    if (!producer->pushCommand(cmd)) {
        setLastError("Failed to push command: " + producer->getLastError());
        return false;
    }

    return true;
}

bool BasicOpsIPCInterface::gripper_control(int position,
                                           const std::string& mapping,
                                           int velocity,
                                           int effort,
                                           int gripper_type) {
    // 外部参数统一按 0~255 输入；实际按夹爪类型的映射在服务端执行。
    position = std::clamp(position, 0, 255);
    velocity = std::clamp(velocity, 0, 255);
    effort   = std::clamp(effort, 0, 255);

    return push_basic_ops_command(
        "GripperControl",
        mapping,
        {
            static_cast<double>(gripper_type),
            static_cast<double>(position),
            static_cast<double>(velocity),
            static_cast<double>(effort)
        });
}

bool BasicOpsIPCInterface::enable_motors(const std::string& mapping, int mode) {
    mode = std::clamp(mode, 0, 255);
    return push_basic_ops_command("MotorEnable", mapping, {static_cast<double>(mode)});
}

bool BasicOpsIPCInterface::disable_motors(const std::string& mapping, int mode) {
    mode = std::clamp(mode, 0, 255);
    return push_basic_ops_command("MotorDisable", mapping, {static_cast<double>(mode)});
}

std::string BasicOpsIPCInterface::getCurrentMode(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return "";
    }
    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return "";
    }
    return state_mgr->getCurrentMode();
}

ipc::ExecutionState BasicOpsIPCInterface::getExecutionState(const std::string& mapping) const {
    if (!ensureInitialized()) {
        return ipc::ExecutionState::FAILED;
    }
    auto state_mgr = getStateManager(mapping);
    if (!state_mgr) {
        return ipc::ExecutionState::FAILED;
    }
    return state_mgr->getExecutionState();
}

}  // namespace arm_controller::basic_ops
