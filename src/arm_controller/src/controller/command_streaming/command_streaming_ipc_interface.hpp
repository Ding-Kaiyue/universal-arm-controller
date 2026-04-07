#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <vector>
#include <string>

namespace arm_controller::command_streaming {

class CommandStreamingIPCInterface : public ipc::ModuleIPCInterface {
public:
    CommandStreamingIPCInterface() = default;

    std::string getModuleName() const override {
        return "CommandStreaming";
    }

    // 执行 CommandStreaming 命令
    bool execute(const std::vector<double>& packed_command,
                 const std::string& mapping);

    // 获取当前模式名
    std::string getCurrentMode(const std::string& mapping) const;

    // 获取执行状态
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;
};

}  // namespace arm_controller::command_streaming
