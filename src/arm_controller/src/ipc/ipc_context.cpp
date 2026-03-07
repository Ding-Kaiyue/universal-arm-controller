#include "arm_controller/ipc/ipc_context.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace arm_controller::ipc {

IPCContext& IPCContext::getInstance() {
    static IPCContext instance;
    return instance;
}

IPCContext::~IPCContext() {
    shutdown();
}

bool IPCContext::initialize(int argc, char** argv) {
    // ✅ 参数保留以保持 API 一致性，但不使用（ROS 初始化由各自的进程负责）
    (void)argc;   // 标记为意图未使用，避免编译警告
    (void)argv;

    try {
        // ✅ 不调用 rclcpp::init() - Producer 进程（example_velocity_control）应自己负责
        // IPCContext::initialize() 仅负责 IPC 基础设施初始化

        // 初始化共享内存管理器（Producer 模式：Participant 角色）
        shm_manager_ = std::make_shared<SharedMemoryManager>();

        // ✅ CRITICAL FIX: Producer 只能 open()，不能 initialize()
        // 原因：只有 Consumer（main.cpp）有权创建和清理共享内存
        // Producer（example_velocity_control）只能 attach 到现有的共享内存
        // Participant 角色禁止调用 initialize()

        if (!shm_manager_->open()) {
            std::cout << "⚠️  Shared memory not found. Check if Consumer (main) has initialized first!" << std::endl;
            std::cout << "   Producer should only attach to existing SHM, not create new one." << std::endl;
            last_error_ = "Failed to open existing shared memory";
            return false;
        }

        // ✅ 设置为 Participant 角色（Producer）
        role_ = ipc::Role::Participant;

        // 创建命令生产者
        command_producer_ = std::make_shared<CommandProducer>(shm_manager_, 0);

        initialized_ = true;
        last_error_ = "";

        std::cout << "✅ IPCContext initialized successfully (Producer mode - Participant role)" << std::endl;
        return true;

    } catch (const std::exception& e) {
        last_error_ = std::string("IPCContext initialize failed: ") + e.what();
        std::cerr << "❌ " << last_error_ << std::endl;
        return false;
    }
}

// ✅ NEW: Consumer 专用初始化（有权创建和清理共享内存）
bool IPCContext::initializeAsConsumer(int argc, char** argv) {
    // ✅ 参数保留以保持 API 一致性，但不使用（ROS 初始化由 main.cpp 负责）
    (void)argc;   // 标记为意图未使用，避免编译警告
    (void)argv;

    try {
        // ✅ CRITICAL: 不调用 rclcpp::init() - main.cpp 已经负责
        // initializeAsConsumer() 仅负责 IPC 基础设施初始化
        // 让 main.cpp 单独管理 ROS 生命周期，避免竞态和初始化顺序问题

        // 初始化共享内存管理器为 Owner 角色（有权创建和清理）
        shm_manager_ = std::make_shared<SharedMemoryManager>();

        // ✅ Consumer 模式：尝试打开，不存在就创建
        // 只有 Owner 有权调用 initialize()，才能清理和创建新的 SHM
        if (!shm_manager_->open()) {
            std::cout << "Shared memory not found, creating new one as Consumer (Owner)..." << std::endl;
            if (!shm_manager_->initialize(ipc::Role::Owner)) {
                last_error_ = "Failed to initialize shared memory as Owner";
                return false;
            }
        }

        // ✅ 设置为 Owner 角色（Consumer）
        role_ = ipc::Role::Owner;

        // 创建命令生产者
        command_producer_ = std::make_shared<CommandProducer>(shm_manager_, 0);

        initialized_ = true;
        last_error_ = "";

        std::cout << "✅ IPCContext initialized successfully (Consumer mode - Owner role)" << std::endl;
        return true;

    } catch (const std::exception& e) {
        last_error_ = std::string("IPCContext::initializeAsConsumer() failed: ") + e.what();
        std::cerr << "❌ " << last_error_ << std::endl;
        return false;
    }
}

void IPCContext::shutdown() {
    if (initialized_) {
        // ✅ Step 1: 信号所有 IPC 消费者线程停止
        // 这会设置 shutdown_ flag，让 popWithFilter() 立即返回
        CommandQueueIPC::getInstance().shutdown();

        // ✅ Step 2: 清理 IPC 资源
        command_producer_.reset();

        // ✅ CRITICAL: 只有 Owner 才能调用 cleanup() 删除系统资源
        // Participant 不能清理共享内存，防止 Producer 销毁 Consumer 还在用的资源
        if (shm_manager_) {
            if (role_ == ipc::Role::Owner) {
                // Owner（Consumer）有权清理共享内存：
                // 1. cleanup() 删除系统资源（SHM、mutex、condition）
                // 2. reset() 释放本地指针引用
                shm_manager_->cleanup();
                shm_manager_.reset();
                std::cout << "✅ IPCContext shutdown completed (Owner cleaned up and removed SHM)" << std::endl;
            } else {
                // Participant（Producer）只清理本地引用，不清理共享内存
                shm_manager_.reset();
                std::cout << "✅ IPCContext shutdown completed (Participant, SHM preserved)" << std::endl;
            }
        }
        state_managers_.clear();

        // ✅ CRITICAL FIX: 移除 rclcpp::shutdown() 调用
        // 原因：rclcpp::shutdown() 不是线程安全的，ROS signal handler 和业务代码
        // 可能会同时调用，导致 ROS executor 内部互斥锁竞争
        // 让 main.cpp 和 ROS signal handler 单独管理 ROS 生命周期
        initialized_ = false;
    }
}

ControllerStateManager* IPCContext::getStateManager(const std::string& mapping) {
    std::lock_guard<std::mutex> lock(state_managers_mutex_);

    auto it = state_managers_.find(mapping);
    if (it != state_managers_.end()) {
        return it->second.get();
    }

    // 创建新的状态管理器
    auto manager = std::make_shared<ControllerStateManager>(mapping);

    // 初始化当前模式为 HoldState（系统启动后的实际默认模式）
    // 在 ControllerManagerNode::post_init() 中：
    //   1. 先启动 SystemStart 进行初始化
    //   2. 然后立即启动 HoldState 作为实际的默认保持状态
    // 因此 IPC 客户端应该同步到 HoldState
    manager->initializeCurrentMode("HoldState");

    state_managers_[mapping] = manager;
    return manager.get();
}

}  // namespace arm_controller::ipc