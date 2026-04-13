#include "arm_controller/ipc/controller_state_manager.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <chrono>
#include <cstring>
#include <iostream>

namespace arm_controller::ipc {
namespace {

uint64_t now_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

SharedExecutionStateEntry* find_or_alloc_entry(SharedExecutionStateTable* table,
                                               const std::string& mapping) {
    if (!table) {
        return nullptr;
    }

    SharedExecutionStateEntry* empty = nullptr;
    for (size_t i = 0; i < MAX_STATE_MAPPINGS; ++i) {
        auto& entry = table->entries[i];
        if (entry.occupied != 0 && std::string(entry.mapping) == mapping) {
            return &entry;
        }
        if (!empty && entry.occupied == 0) {
            empty = &entry;
        }
    }

    if (!empty) {
        return nullptr;
    }

    empty->occupied = 1;
    std::strncpy(empty->mapping, mapping.c_str(), MAX_STATE_MAPPING_LEN - 1);
    empty->mapping[MAX_STATE_MAPPING_LEN - 1] = '\0';
    return empty;
}

const SharedExecutionStateEntry* find_entry(const SharedExecutionStateTable* table,
                                            const std::string& mapping) {
    if (!table) {
        return nullptr;
    }
    for (size_t i = 0; i < MAX_STATE_MAPPINGS; ++i) {
        const auto& entry = table->entries[i];
        if (entry.occupied != 0 && std::string(entry.mapping) == mapping) {
            return &entry;
        }
    }
    return nullptr;
}

bool read_shared_state(const std::string& mapping, std::string& mode, ExecutionState& state) {
    auto shm_manager = IPCContext::getInstance().getSharedMemoryManager();
    if (!shm_manager || !shm_manager->isValid()) {
        return false;
    }

    auto* mutex = shm_manager->getMutex();
    auto* table = shm_manager->getStateTable();
    if (!mutex || !table) {
        return false;
    }

    try {
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex);
        const auto* entry = find_entry(table, mapping);
        if (!entry) {
            return false;
        }
        mode = std::string(entry->current_mode);
        state = static_cast<ExecutionState>(entry->execution_state);
        return true;
    } catch (...) {
        return false;
    }
}

void write_shared_state(const std::string& mapping,
                        const std::string& mode,
                        ExecutionState state) {
    auto shm_manager = IPCContext::getInstance().getSharedMemoryManager();
    if (!shm_manager || !shm_manager->isValid()) {
        return;
    }

    auto* mutex = shm_manager->getMutex();
    auto* table = shm_manager->getStateTable();
    if (!mutex || !table) {
        return;
    }

    try {
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex);
        auto* entry = find_or_alloc_entry(table, mapping);
        if (!entry) {
            return;
        }
        std::strncpy(entry->current_mode, mode.c_str(), MAX_STATE_MODE_LEN - 1);
        entry->current_mode[MAX_STATE_MODE_LEN - 1] = '\0';
        entry->execution_state = static_cast<int32_t>(state);
        entry->timestamp_ns = now_ns();
    } catch (...) {
        // 共享状态写入失败时不抛异常，保留本地状态作为退化路径
    }
}

}  // namespace

// 定义需要hook状态才能安全停止的模式
// 这些是轨迹规划模式，需要在切换前进入HoldState
const std::unordered_set<std::string> ControllerStateManager::modes_requiring_hook_ = {
    "MoveJ",
    "MoveL",
    "MoveC",
    "JointVelocity",
    "CartesianVelocity",
    "MinkServo",
    "TrajectoryRecord",
    "TrajectoryReplay",
    "PointRecord",
    "PointReplay"
};

std::string ControllerStateManager::getCurrentMode() const {
    std::string shared_mode;
    ExecutionState shared_state = ExecutionState::IDLE;
    if (read_shared_state(mapping_, shared_mode, shared_state)) {
        return shared_mode;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_mode_;
}

std::string ControllerStateManager::getTargetMode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return target_mode_;
}

ExecutionState ControllerStateManager::getExecutionState() const {
    std::string shared_mode;
    ExecutionState shared_state = ExecutionState::IDLE;
    if (read_shared_state(mapping_, shared_mode, shared_state)) {
        return shared_state;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    return execution_state_;
}

bool ControllerStateManager::isInHookState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return in_hook_state_;
}

void ControllerStateManager::setExecutionState(ExecutionState state) {
    std::string mode_to_write;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        execution_state_ = state;
        mode_to_write = current_mode_;
    }
    write_shared_state(mapping_, mode_to_write, state);
}

void ControllerStateManager::initializeCurrentMode(const std::string& mode) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_mode_ = mode;
        target_mode_ = mode;
        execution_state_ = ExecutionState::IDLE;
        in_hook_state_ = false;  // ✅ 清除 hook_state 标志
    }
    if (IPCContext::getInstance().isOwnerRole()) {
        write_shared_state(mapping_, mode, ExecutionState::IDLE);
    }
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
        return true;
    }

    // 检查是否需要停止当前运动
    if (need_stop_before_transition_(current_mode_, target_mode)) {
        // 进入 hook 状态（HOLDING）
        in_hook_state_ = true;
        target_mode_ = target_mode;
        execution_state_ = ExecutionState::IDLE;
        return true;
    }

    // 可以直接转移（仅记录“目标模式请求”）
    // 注意：current_mode_ 只由执行侧反馈（initializeCurrentMode/updateFromExecutor）更新，
    // 避免 IPC 侧出现“已切换”假象，与 ControllerManager 实际模式保持一致。
    target_mode_ = target_mode;
    execution_state_ = ExecutionState::IDLE;
    std::cout << "➡️ [" << mapping_ << "] Requested target mode: " << target_mode
              << " (current: " << current_mode_ << ")" << std::endl;
    return true;
}

void ControllerStateManager::updateFromExecutor(
    const ExecutorControllerState& executor_state) {
    bool need_write = false;
    std::string mode_to_write;
    ExecutionState state_to_write = ExecutionState::IDLE;

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
            need_write = true;
            mode_to_write = current_mode_;
            state_to_write = execution_state_;

            std::cout << "✅ [" << mapping_ << "] Hook transition completed, now in mode: "
                      << current_mode_ << std::endl;
        } else if (!in_hook_state_) {
            // 更新当前模式和执行状态（从执行进程反馈）
            current_mode_ = executor_state.current_mode;
            execution_state_ = (ExecutionState)executor_state.execution_state;
            need_write = true;
            mode_to_write = current_mode_;
            state_to_write = execution_state_;
        }
    }
    if (need_write) {
        write_shared_state(mapping_, mode_to_write, state_to_write);
    }
}

}  // namespace arm_controller::ipc
