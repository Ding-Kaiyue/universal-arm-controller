#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <vector>
#include <string>

namespace arm_controller::mink_servo {

class MinkServoIPCInterface : public ipc::ModuleIPCInterface {
public:
    MinkServoIPCInterface() = default;

    std::string getModuleName() const override {
        return "MinkServo";
    }

    bool execute(const std::vector<double>& target_pose, const std::string& mapping);
    std::string getCurrentMode(const std::string& mapping) const;
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;
};

}  // namespace arm_controller::mink_servo

