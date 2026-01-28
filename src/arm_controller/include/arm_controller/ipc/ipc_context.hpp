#pragma once

#include "shm_manager.hpp"
#include "command_producer.hpp"
#include "controller_state_manager.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>

namespace arm_controller::ipc {

// ============================================================================
// IPCContext：全局IPC上下文管理器
// 职责：
//   1. 管理共享内存、命令生产者等基础设施的生命周期
//   2. 提供单例访问点
//   3. 为各模式接口提供统一的IPC资源访问接口
// 原则：单一职责（SRP）- 仅负责基础设施管理
// ============================================================================
class IPCContext {
public:
    // 获取单例实例
    static IPCContext& getInstance();

    // 禁止拷贝
    IPCContext(const IPCContext&) = delete;
    IPCContext& operator=(const IPCContext&) = delete;

    // 初始化IPC基础设施
    bool initialize(int argc = 0, char** argv = nullptr);

    // 关闭IPC基础设施
    void shutdown();

    // 检查初始化状态
    bool isInitialized() const { return initialized_; }

    // ========================================================================
    // 访问器：供模式接口使用
    // ========================================================================

    std::shared_ptr<SharedMemoryManager> getSharedMemoryManager() const {
        return shm_manager_;
    }

    std::shared_ptr<CommandProducer> getCommandProducer() const {
        return command_producer_;
    }

    // 获取或创建指定mapping的状态管理器
    ControllerStateManager* getStateManager(const std::string& mapping);

    // 获取错误信息
    std::string getLastError() const { return last_error_; }

private:
    IPCContext() = default;
    ~IPCContext();

    // IPC基础设施
    std::shared_ptr<SharedMemoryManager> shm_manager_;
    std::shared_ptr<CommandProducer> command_producer_;

    // 状态管理器池（每个mapping一个）
    std::unordered_map<std::string, std::shared_ptr<ControllerStateManager>> state_managers_;
    mutable std::mutex state_managers_mutex_;

    // 初始化状态
    bool initialized_ = false;
    std::string last_error_;
};

}  // namespace arm_controller::ipc