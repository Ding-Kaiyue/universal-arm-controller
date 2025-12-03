#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <vector>
#include <string>

namespace arm_controller::movej {

class MoveJIPCInterface : public ipc::ModuleIPCInterface {
public:
    MoveJIPCInterface() = default;

    std::string getModuleName() const override {
        return "MoveJ";
    }

    // 执行 MoveJ 命令
    bool execute(const std::vector<double>& joint_positions,
                 const std::string& mapping);

    // 获取当前模式名
    std::string getCurrentMode(const std::string& mapping) const;

    // 获取执行状态
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;

    // 获取当前关节位置
    std::vector<double> getCurrentJointPositions(const std::string& mapping) const;
};

}  // namespace arm_controller::movej
