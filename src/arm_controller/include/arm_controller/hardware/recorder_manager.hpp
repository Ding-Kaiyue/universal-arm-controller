#pragma once

#include "arm_controller/hardware/motor_data_recorder.hpp"
#include <memory>
#include <string>
#include <mutex>

/**
 * RecorderManager - 管理 MotorDataRecorder 的生命周期
 *
 * 职责：
 * - 文件级别的 recorder 生命周期管理
 * - 状态转换保护（通过 mutex）
 * - 向硬件层提供 recorder 指针（仅供观察者注册）
 *
 * 不管 mapping：mapping 只用于初始化控制器，不涉及录制状态
 */
class RecorderManager {
public:
    enum class State {
        IDLE,           // 没有录制
        RECORDING,      // 正在录制
        PAUSED,         // 暂停中
    };

    RecorderManager() = default;
    ~RecorderManager();

    /**
     * 启动新的录制
     * @param file_path 输出文件路径
     * @return 成功返回 true，失败返回 false（e.g. 前一个录制未完成）
     */
    bool startRecording(const std::string& file_path);

    /**
     * 暂停当前录制
     * @return 成功返回 true（state 从 RECORDING → PAUSED）
     */
    bool pauseRecording();

    /**
     * 恢复已暂停的录制
     * @return 成功返回 true（state 从 PAUSED → RECORDING）
     */
    bool resumeRecording();

    /**
     * 正常停止录制（flush + stop）
     * @return 成功返回 true，state 变为 IDLE
     */
    bool stopRecording();

    /**
     * 异常取消录制（不 flush，直接停止）
     * @return 成功返回 true，state 变为 IDLE
     */
    bool cancelRecording();

    /**
     * 获取当前录制状态
     * @return State::IDLE / RECORDING / PAUSED
     */
    State getState() const;

    /**
     * 获取当前文件路径
     * @return 文件路径（在 IDLE 状态下返回空字符串）
     */
    std::string getFilePath() const;

    /**
     * 获取 recorder 指针（供硬件层注册观察者）
     * @return MotorDataRecorder 指针，IDLE 时返回 nullptr
     */
    std::shared_ptr<MotorDataRecorder> getRecorder() const;

private:
    std::shared_ptr<MotorDataRecorder> recorder_;
    State state_ = State::IDLE;
    std::string file_path_;
    mutable std::mutex mutex_;  // 保护所有成员变量
};
