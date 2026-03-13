#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <string>

namespace arm_controller::trajectory_replay {

class TrajectoryReplayIPCInterface : public ipc::ModuleIPCInterface {
public:
    TrajectoryReplayIPCInterface() = default;

    std::string getModuleName() const override {
        return "TrajectoryReplay";
    }

    // 开始执行（播放）轨迹 - 支持"*"通配符表示所有active mappings
    bool startReplay(const std::string& file_name, const std::string& mapping = "");

    // 暂停执行 - 支持"*"通配符
    bool pauseReplay(const std::string& mapping = "");

    // 恢复执行 - 支持"*"通配符
    bool resumeReplay(const std::string& mapping = "");

    // 停止执行 - 支持"*"通配符
    bool stopReplay(const std::string& mapping = "");

    // 取消执行（强制停止） - 支持"*"通配符
    bool cancelReplay(const std::string& mapping = "");

    // 获取当前模式名
    std::string getCurrentMode(const std::string& mapping) const;

    // 获取执行状态
    ipc::ExecutionState getExecutionState(const std::string& mapping) const;

    // 获取回放状态
    bool isReplaying(const std::string& mapping) const;
};

}   // namespace arm_controller::trajectory_replay