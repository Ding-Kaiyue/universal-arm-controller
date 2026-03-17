#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <string>

namespace arm_controller::basic_ops {

class BasicOpsIPCInterface : public ipc::ModuleIPCInterface {
public:
    BasicOpsIPCInterface() = default;

    std::string getModuleName() const override {
        return "BasicOps";
    }

    // 任意时刻调用夹爪控制（外部统一使用 0~255 原始刻度）
    bool gripper_control(int position,
                        const std::string& mapping,
                        int velocity,
                        int effort,
                        int gripper_type = -1);

    // 电机使能 / 失能（mode: 例如 MIT 模式）
    bool enable_motors(const std::string& mapping, int mode);
    bool disable_motors(const std::string& mapping, int mode);

    std::string getCurrentMode(const std::string& mapping) const;
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;

private:
    bool push_basic_ops_command(const std::string& op_mode,
                                const std::string& mapping,
                                const std::vector<double>& params);
};

}  // namespace arm_controller::basic_ops
