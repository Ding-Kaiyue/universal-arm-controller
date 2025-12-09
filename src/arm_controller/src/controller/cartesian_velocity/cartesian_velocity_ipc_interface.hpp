#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <vector>
#include <string>

namespace arm_controller::cartesian_velocity {

class CartesianVelocityIPCInterface : public ipc::ModuleIPCInterface {
public:
    CartesianVelocityIPCInterface() = default;

    std::string getModuleName() const override {
        return "CartesianVelocity";
    }

    // 执行 CartesianVelocity 命令
    bool execute(const std::vector<double>& cartesian_velocities,
                 const std::string& mapping);

    // 获取当前模式名
    std::string getCurrentMode(const std::string& mapping) const;

    // 获取执行状态
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;
};

} // namespace arm_controller::cartesian_velocity