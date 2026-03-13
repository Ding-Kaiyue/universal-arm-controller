#pragma once

#include "arm_controller/ipc/module_ipc_interface.hpp"
#include <string>

namespace arm_controller::trajectory_record {

class TrajectoryRecordIPCInterface : public ipc::ModuleIPCInterface {
public:
    TrajectoryRecordIPCInterface() = default;

    std::string getModuleName() const override {
        return "TrajectoryRecord";
    }

    // 开始记录轨迹（mapping 为空表示对所有 active mappings）
    bool startRecording(const std::string& file_name, const std::string& mapping = "");

    // 暂停记录（mapping 为空表示暂停所有 recorders）
    bool pauseRecording(const std::string& mapping = "");

    // 恢复记录（mapping 为空表示恢复所有 recorders）
    bool resumeRecording(const std::string& mapping = "");

    // 停止记录（mapping 为空表示停止所有 recorders）
    bool stopRecording(const std::string& mapping = "");

    // 取消记录（mapping 为空表示取消所有 recorders）
    bool cancelRecording(const std::string& mapping = "");

    // 获取当前模式名
    std::string getCurrentMode(const std::string& mapping) const;

    // 获取执行状态（针对第一个active mapping）
    ipc::ExecutionState getExecutionState() const;

    // 获取记录状态（是否有任何mapping在录制中）
    bool isRecording() const;
};

}   // namespace arm_controller::trajectory_record