#include "arm_controller/ipc/ipc_context.hpp"
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
    try {
        // 初始化 ROS2（如果需要）
        if (!rclcpp::ok()) {
            rclcpp::init(argc, argv);
        }

        // 初始化共享内存管理器
        shm_manager_ = std::make_shared<SharedMemoryManager>();

        // 尝试打开现有共享内存，如果不存在则创建
        if (!shm_manager_->open()) {
            std::cout << "Shared memory not found, creating new one..." << std::endl;
            if (!shm_manager_->initialize()) {
                last_error_ = "Failed to initialize shared memory";
                return false;
            }
        }

        // 创建命令生产者
        command_producer_ = std::make_shared<CommandProducer>(shm_manager_, 0);

        initialized_ = true;
        last_error_ = "";

        std::cout << "✅ IPCContext initialized successfully" << std::endl;
        return true;

    } catch (const std::exception& e) {
        last_error_ = std::string("IPCContext initialize failed: ") + e.what();
        std::cerr << "❌ " << last_error_ << std::endl;
        return false;
    }
}

void IPCContext::shutdown() {
    if (initialized_) {
        command_producer_.reset();
        if (shm_manager_) {
            shm_manager_->close();
        }
        state_managers_.clear();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        initialized_ = false;
        std::cout << "✅ IPCContext shutdown completed" << std::endl;
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

    // 初始化当前模式为 SystemStart（系统启动时的默认模式）
    // 这样可以正确地追踪 SystemStart -> MoveJ/MoveL 等的转移
    manager->initializeCurrentMode("SystemStart");

    state_managers_[mapping] = manager;
    return manager.get();
}

}  // namespace arm_controller::ipc