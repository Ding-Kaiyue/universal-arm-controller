#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <vector>
#include <string>

namespace arm_controller::movec {

class MoveCIPCInterface : public ipc::ModuleIPCInterface {
public:
    MoveCIPCInterface() = default;

    std::string getModuleName() const override {
        return "MoveC";
    }

    // 执行 MoveC 命令
    bool execute(const std::vector<double>& waypoints,
                 const std::string& mapping);

    // 获取当前模式名
    std::string getCurrentMode(const std::string& mapping) const;

    // 获取执行状态
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;

    // 获取当前位置
    std::vector<double> getCurrentPosition(const std::string& mapping) const;
};

}  // namespace arm_controller::movec
