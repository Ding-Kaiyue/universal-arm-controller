#pragma once

namespace arm_controller {

// ============================================================================
// IPCLifecycle：IPC 生命周期管理
// 职责：仅负责初始化和关闭 IPC 系统
// 原则：单一职责（SRP）- 最小化职责范围
// ============================================================================
class IPCLifecycle {
public:
    // Producer 初始化（只能 attach 到现有共享内存）
    static bool initialize(int argc = 0, char** argv = nullptr);

    // ✅ NEW: Consumer 初始化（有权创建和清理共享内存）
    static bool initializeAsConsumer(int argc = 0, char** argv = nullptr);

    // 关闭 IPC 系统
    static void shutdown();

    // 检查是否已初始化
    static bool isInitialized();
};

}  // namespace arm_controller