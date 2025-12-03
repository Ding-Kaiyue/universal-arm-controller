#pragma once

#include "arm_controller/ipc/ipc_context.hpp"
#include "arm_controller/ipc/command_producer.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"
#include <string>
#include <memory>

namespace arm_controller::ipc {

// ============================================================================
// ModuleIPCInterface：IPC模式接口的基类
// 职责：
//   1. 定义模式接口的规范
//   2. 提供通用的验证和状态管理功能
// 原则：开闭原则（OCP）- 通过继承扩展功能，不修改基类
// ============================================================================
class ModuleIPCInterface {
public:
    virtual ~ModuleIPCInterface() = default;

    // 禁止拷贝
    ModuleIPCInterface(const ModuleIPCInterface&) = delete;
    ModuleIPCInterface& operator=(const ModuleIPCInterface&) = delete;

    // 获取模式名称
    virtual std::string getModuleName() const = 0;

    // 获取错误信息
    std::string getLastError() const { return last_error_; }

protected:
    ModuleIPCInterface() = default;

    // 受保护的访问器：供派生类使用
    IPCContext& getIPCContext() const {
        return IPCContext::getInstance();
    }

    std::shared_ptr<CommandProducer> getCommandProducer() const {
        return getIPCContext().getCommandProducer();
    }

    std::shared_ptr<SharedMemoryManager> getSharedMemoryManager() const {
        return getIPCContext().getSharedMemoryManager();
    }

    ControllerStateManager* getStateManager(const std::string& mapping) const {
        return getIPCContext().getStateManager(mapping);
    }

    // 设置错误信息
    void setLastError(const std::string& error) {
        last_error_ = error;
    }

    // 检查初始化状态
    bool ensureInitialized() const {
        if (!getIPCContext().isInitialized()) {
            const_cast<ModuleIPCInterface*>(this)->setLastError(
                "IPC not initialized. Call IPCContext::initialize() first.");
            return false;
        }
        return true;
    }

private:
    std::string last_error_;
};

}  // namespace arm_controller::ipc
