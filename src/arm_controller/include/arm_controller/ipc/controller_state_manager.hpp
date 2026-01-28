#pragma once

#include "ipc_types.hpp"
#include <string>
#include <unordered_map>
#include <mutex>

namespace arm_controller::ipc {

// ============================================================================
// 执行状态（在当前模式下的执行进度）
// 与具体的模式名称无关，只表示执行的生命周期
// ============================================================================
enum class ExecutionState : int32_t {
    IDLE = 0,             // 已进入模式，未发送轨迹
    PENDING = 1,          // 轨迹已发送，未开始执行
    EXECUTING = 2,        // 轨迹执行中
    SUCCESS = 3,          // 轨迹执行成功
    FAILED = 4,           // 轨迹执行失败
};

// ============================================================================
// IPC 执行进程的状态信息结构
// ============================================================================
struct alignas(16) ExecutorControllerState {
    char current_mode[32];      // 当前激活的模式名 (如: "SystemStart", "MoveJ", "MoveL")
    int32_t execution_state;    // ExecutionState
    uint64_t cmd_seq;           // 当前执行的命令序号
    uint64_t timestamp_ns;      // 最后更新时间
    int32_t progress;           // 执行进度 0-100
    char error_message[256];    // 错误信息

    ExecutorControllerState()
        : execution_state((int32_t)ExecutionState::IDLE),
          cmd_seq(0),
          timestamp_ns(0),
          progress(0) {
        std::memset(current_mode, 0, 32);
        std::memset(error_message, 0, 256);
    }
};

// ============================================================================
// 客户端（API）侧的状态管理器
// 职责：
// 1. 追踪当前激活的模式名称（字符串）
// 2. 追踪执行状态（未发送、待执行、执行中、成功、失败）
// 3. 非阻塞式模式转移
// ============================================================================
class ControllerStateManager {
public:
    explicit ControllerStateManager(const std::string& mapping = "left_arm")
        : mapping_(mapping),
          current_mode_(""),           // 初始化为空（系统启动时为 SystemStart）
          target_mode_(""),
          execution_state_(ExecutionState::IDLE),
          in_hook_state_(false) {}

    // 状态查询
    std::string getCurrentMode() const;      // 当前激活的模式名
    std::string getTargetMode() const;       // 目标模式名
    ExecutionState getExecutionState() const; // 当前执行状态
    bool isInHookState() const;              // 是否在 hook 状态

    // 非阻塞式模式转移
    // 立即返回，不等待执行进程反馈
    bool transitionToMode(const std::string& target_mode);

    // 设置执行状态
    void setExecutionState(ExecutionState state);

    // 初始化当前模式（用于启动时与执行进程同步）
    void initializeCurrentMode(const std::string& mode);

    // 从共享内存更新执行进程的状态
    void updateFromExecutor(const ExecutorControllerState& executor_state);

private:
    std::string mapping_;
    std::string current_mode_;       // 当前模式名（如 "SystemStart", "MoveJ"）
    std::string target_mode_;        // 目标模式名
    ExecutionState execution_state_; // 当前执行状态
    mutable std::mutex state_mutex_;
    bool in_hook_state_;

    bool need_stop_before_transition_(
        const std::string& from,
        const std::string& to);
};

}  // namespace arm_controller::ipc