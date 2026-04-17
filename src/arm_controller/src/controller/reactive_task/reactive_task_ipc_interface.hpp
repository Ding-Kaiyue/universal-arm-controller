#pragma once

#include <string>
#include <vector>

#include "arm_controller/ipc/module_ipc_interface.hpp"

namespace arm_controller::reactive_task {

class ReactiveTaskIPCInterface : public ipc::ModuleIPCInterface {
public:
    ReactiveTaskIPCInterface() = default;

    std::string getModuleName() const override {
        return "ReactiveTask";
    }

    bool execute(const std::vector<double>& target_pose, const std::string& mapping);
    std::string getCurrentMode(const std::string& mapping) const;
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;
};

}  // namespace arm_controller::reactive_task
