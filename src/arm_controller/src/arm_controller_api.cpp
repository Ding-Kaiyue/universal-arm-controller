#include "arm_controller/arm_controller_api.hpp"
#include "arm_controller/ipc/shm_manager.hpp"
#include "arm_controller/ipc/command_producer.hpp"
#include "arm_controller/ipc/ipc_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>

namespace arm_controller {

// ============================================================================
// Impl 结构体：隐藏实现细节
// ============================================================================
struct ArmControllerAPI::Impl {
    std::shared_ptr<ipc::SharedMemoryManager> shm_manager;
    std::shared_ptr<ipc::CommandProducer> producer;
    std::string last_error;
    bool initialized = false;

    Impl() = default;
};

// ============================================================================
// ArmControllerAPI 实现
// ============================================================================

ArmControllerAPI& ArmControllerAPI::getInstance() {
    static ArmControllerAPI instance;
    return instance;
}

ArmControllerAPI::ArmControllerAPI() : impl_(std::make_unique<Impl>()) {}

ArmControllerAPI::~ArmControllerAPI() {
    shutdown();
}

bool ArmControllerAPI::initialize(int argc, char** argv) {
    try {
        // 初始化 ROS2（如果需要）
        if (!rclcpp::ok()) {
            rclcpp::init(argc, argv);
        }

        // 初始化 IPC 基础设施
        impl_->shm_manager = std::make_shared<ipc::SharedMemoryManager>();

        // 尝试打开现有共享内存，如果不存在则创建
        if (!impl_->shm_manager->open()) {
            std::cout << "Shared memory not found, creating new one..." << std::endl;
            if (!impl_->shm_manager->initialize()) {
                impl_->last_error = "Failed to initialize shared memory";
                return false;
            }
        }

        // 创建命令生产者
        impl_->producer = std::make_shared<ipc::CommandProducer>(
            impl_->shm_manager,
            0);  // producer_id = 0

        impl_->initialized = true;
        impl_->last_error = "";

        std::cout << "✅ ArmControllerAPI initialized successfully" << std::endl;
        return true;

    } catch (const std::exception& e) {
        impl_->last_error = std::string("Initialize failed: ") + e.what();
        std::cerr << "❌ " << impl_->last_error << std::endl;
        return false;
    }
}

void ArmControllerAPI::shutdown() {
    if (impl_->initialized) {
        impl_->producer.reset();
        if (impl_->shm_manager) {
            impl_->shm_manager->close();
        }
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        impl_->initialized = false;
        std::cout << "✅ ArmControllerAPI shutdown completed" << std::endl;
    }
}

bool ArmControllerAPI::ensureInitialized() {
    if (!impl_->initialized) {
        impl_->last_error = "API not initialized. Call initialize() first.";
        return false;
    }
    return true;
}

bool ArmControllerAPI::moveJ(const std::vector<double>& joint_positions,
                              const std::string& mapping) {
    if (!ensureInitialized()) return false;

    // 验证参数
    auto val_result = ipc::CommandValidator::validateMapping(mapping);
    if (!val_result.valid) {
        impl_->last_error = val_result.error_message;
        return false;
    }

    val_result = ipc::CommandValidator::validateJointPositions(joint_positions);
    if (!val_result.valid) {
        impl_->last_error = val_result.error_message;
        return false;
    }

    // 构建命令
    auto cmd = ipc::CommandBuilder()
        .withMode("MoveJ")
        .withMapping(mapping)
        .withJointPositions(joint_positions)
        .build();

    // 发送命令
    if (!impl_->producer->pushCommand(cmd)) {
        impl_->last_error = impl_->producer->getLastError();
        return false;
    }

    impl_->last_error = "";
    return true;
}

bool ArmControllerAPI::moveL(double x, double y, double z,
                              double qx, double qy, double qz, double qw,
                              const std::string& mapping) {
    if (!ensureInitialized()) return false;

    // 验证参数
    auto val_result = ipc::CommandValidator::validateMapping(mapping);
    if (!val_result.valid) {
        impl_->last_error = val_result.error_message;
        return false;
    }

    val_result = ipc::CommandValidator::validatePose(x, y, z, qx, qy, qz, qw);
    if (!val_result.valid) {
        impl_->last_error = val_result.error_message;
        return false;
    }

    // 构建命令
    auto cmd = ipc::CommandBuilder()
        .withMode("MoveL")
        .withMapping(mapping)
        .withPose(x, y, z, qx, qy, qz, qw)
        .build();

    // 发送命令
    if (!impl_->producer->pushCommand(cmd)) {
        impl_->last_error = impl_->producer->getLastError();
        return false;
    }

    impl_->last_error = "";
    return true;
}

bool ArmControllerAPI::moveC(const std::vector<double>& waypoints,
                              const std::string& mapping) {
    if (!ensureInitialized()) return false;

    // 验证参数
    auto val_result = ipc::CommandValidator::validateMapping(mapping);
    if (!val_result.valid) {
        impl_->last_error = val_result.error_message;
        return false;
    }

    val_result = ipc::CommandValidator::validateWaypoints(waypoints);
    if (!val_result.valid) {
        impl_->last_error = val_result.error_message;
        return false;
    }

    // 构建命令
    auto cmd = ipc::CommandBuilder()
        .withMode("MoveC")
        .withMapping(mapping)
        .withWaypoints(waypoints)
        .build();

    // 发送命令
    if (!impl_->producer->pushCommand(cmd)) {
        impl_->last_error = impl_->producer->getLastError();
        return false;
    }

    impl_->last_error = "";
    return true;
}

bool ArmControllerAPI::setJointVelocity(
    const std::vector<double>& velocities [[maybe_unused]],
    const std::string& mapping [[maybe_unused]]) {
    impl_->last_error = "Not implemented";
    return false;
}

bool ArmControllerAPI::pauseTrajectory(const std::string& mapping [[maybe_unused]]) {
    impl_->last_error = "Not implemented";
    return false;
}

bool ArmControllerAPI::resumeTrajectory(const std::string& mapping [[maybe_unused]]) {
    impl_->last_error = "Not implemented";
    return false;
}

bool ArmControllerAPI::cancelTrajectory(const std::string& mapping [[maybe_unused]]) {
    impl_->last_error = "Not implemented";
    return false;
}

std::vector<double> ArmControllerAPI::getCurrentJointPositions(
    const std::string& mapping [[maybe_unused]]) {
    return {};
}

bool ArmControllerAPI::isRobotStopped(const std::string& mapping [[maybe_unused]]) {
    return true;
}

std::string ArmControllerAPI::getLastError() const {
    return impl_->last_error;
}

}  // namespace arm_controller
