#pragma once

namespace arm_controller {

// ============================================================================
// IPCLifecycle：IPC 生命周期管理
// 职责：仅负责初始化和关闭 IPC 系统
// 原则：单一职责（SRP）- 最小化职责范围
// ============================================================================
class IPCLifecycle {
public:
    // 初始化 IPC 系统
    static bool initialize(int argc = 0, char** argv = nullptr);

    // 关闭 IPC 系统
    static void shutdown();

    // 检查是否已初始化
    static bool isInitialized();
};

}  // namespace arm_controller
