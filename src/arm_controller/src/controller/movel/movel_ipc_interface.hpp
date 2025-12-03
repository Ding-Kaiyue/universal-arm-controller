#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <vector>
#include <string>

namespace arm_controller::movel {

class MoveLIPCInterface : public ipc::ModuleIPCInterface {
public:
    MoveLIPCInterface() = default;

    std::string getModuleName() const override {
        return "MoveL";
    }

    // 执行 MoveL 命令
    bool execute(double x, double y, double z,
                 double qx, double qy, double qz, double qw,
                 const std::string& mapping);

    // 获取当前模式名
    std::string getCurrentMode(const std::string& mapping) const;

    // 获取执行状态
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;
};

}  // namespace arm_controller::movel
