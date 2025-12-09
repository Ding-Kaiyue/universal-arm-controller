#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <vector>
#include <string>

namespace arm_controller::joint_velocity {

class JointVelocityIPCInterface : public ipc::ModuleIPCInterface {
public:
    JointVelocityIPCInterface() = default;

    std::string getModuleName() const override {
        return "JointVelocity";
    }

    // 执行 JointVelocity 命令
    bool execute(const std::vector<double>& joint_velocities,
                 const std::string& mapping);

    // 获取当前模式名
    std::string getCurrentMode(const std::string& mapping) const;

    // 获取执行状态
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;
};

} // namespace arm_controller::joint_velocity