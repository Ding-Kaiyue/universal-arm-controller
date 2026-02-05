#include "arm_controller/ipc/controller_state_manager.hpp"
#include <iostream>

namespace arm_controller::ipc {

// 定义需要hook状态才能安全停止的模式
// 这些是轨迹规划模式，需要在切换前进入HoldState
const std::unordered_set<std::string> ControllerStateManager::modes_requiring_hook_ = {
    "MoveJ",
    "MoveL",
    "MoveC",
    "JointVelocity",
    "CartesianVelocity",
    "TrajectoryReplay",
    "PointReplay"
};

std::string ControllerStateManager::getCurrentMode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_mode_;
}

std::string ControllerStateManager::getTargetMode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return target_mode_;
}

ExecutionState ControllerStateManager::getExecutionState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return execution_state_;
}

bool ControllerStateManager::isInHookState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return in_hook_state_;
}

void ControllerStateManager::setExecutionState(ExecutionState state) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        execution_state_ = state;
    }
    std::string state_str;
    switch (state) {
        case ExecutionState::IDLE: state_str = "IDLE"; break;
        case ExecutionState::PENDING: state_str = "PENDING"; break;
        case ExecutionState::EXECUTING: state_str = "EXECUTING"; break;
        case ExecutionState::SUCCESS: state_str = "SUCCESS"; break;
        case ExecutionState::FAILED: state_str = "FAILED"; break;
        default: state_str = "UNKNOWN"; break;
    }
}

void ControllerStateManager::initializeCurrentMode(const std::string& mode) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_mode_ = mode;
    target_mode_ = mode;
    execution_state_ = ExecutionState::IDLE;
}

bool ControllerStateManager::need_stop_before_transition_(
    const std::string& from,
    const std::string& to) {
    // 如果已经在目标模式，不需要停止
    if (from == to) {
        return false;
    }

    // 检查当前模式是否在需要hook的模式列表中
    // 如果是，则需要停止并进入HoldState
    if (modes_requiring_hook_.find(from) != modes_requiring_hook_.end()) {
        return true;
    }

    // 其他情况不需要停止
    return false;
}

bool ControllerStateManager::transitionToMode(const std::string& target_mode) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 如果已经是目标模式且不在 hook，直接返回
    if (current_mode_ == target_mode && !in_hook_state_) {
        return true;
    }

    // 如果在 hook 状态，只更新目标模式
    if (in_hook_state_) {
        target_mode_ = target_mode;
        std::cout << "ℹ️  [" << mapping_ << "] In hook state, updated target mode to: "
                  << target_mode << std::endl;
        return true;
    }

    // 检查是否需要停止当前运动
    if (need_stop_before_transition_(current_mode_, target_mode)) {
        // 进入 hook 状态（HOLDING）
        in_hook_state_ = true;
        target_mode_ = target_mode;
        execution_state_ = ExecutionState::IDLE;
        std::cout << "⏹️  [" << mapping_ << "] Transitioning to hook before: "
                  << target_mode << std::endl;
        return true;
    }

    // 可以直接转移
    target_mode_ = target_mode;
    current_mode_ = target_mode;
    execution_state_ = ExecutionState::IDLE;
    std::cout << "✅ [" << mapping_ << "] Transitioning to mode: " << target_mode << std::endl;
    return true;
}

void ControllerStateManager::updateFromExecutor(
    const ExecutorControllerState& executor_state) {

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 如果在 hook 状态且执行进程已返回到之前的模式（hook 完成）
        if (in_hook_state_ &&
            std::string(executor_state.current_mode) != "Holding" &&
            std::string(executor_state.current_mode) != "") {

            // 退出 hook 状态，转移到目标模式
            in_hook_state_ = false;
            current_mode_ = target_mode_;
            execution_state_ = ExecutionState::IDLE;

            std::cout << "✅ [" << mapping_ << "] Hook transition completed, now in mode: "
                      << current_mode_ << std::endl;
            return;
        }

        // 更新当前模式和执行状态（从执行进程反馈）
        if (!in_hook_state_) {
            current_mode_ = executor_state.current_mode;
            execution_state_ = (ExecutionState)executor_state.execution_state;
        }
    }
}

}  // namespace arm_controller::ipc