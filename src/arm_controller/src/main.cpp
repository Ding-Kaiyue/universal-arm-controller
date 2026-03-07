#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "controller_manager_section.hpp"
#include "trajectory_controller_section.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/arm_controller_api.hpp"
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        // ✅ 使用 Consumer 专用初始化（有权清理和创建 SHM）
        // 这是唯一有权调用 cleanup() 的地方
        arm_controller::CommandQueueIPC::cleanup();
        if (!arm_controller::IPCLifecycle::initializeAsConsumer(argc, argv)) {
            RCLCPP_FATAL(rclcpp::get_logger("main"), "Failed to initialize IPC as Consumer");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("main"), "✅ IPC Consumer initialized successfully");

        // 创建多线程执行器
        rclcpp::executors::MultiThreadedExecutor executor;

        // 1. 首先创建并初始化 ControllerManagerNode（负责硬件初始化）
        auto controller_manager = std::make_shared<ControllerManagerNode>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing Controller Manager Node...");
        controller_manager->post_init();
        executor.add_node(controller_manager);

        // 2. 短暂延迟确保硬件管理器完全初始化
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 3. 然后创建并初始化 TrajectoryControllerNode（使用已初始化的硬件管理器）
        auto trajectory_controller = std::make_shared<TrajectoryControllerNode>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing Trajectory Controller Node...");
        trajectory_controller->post_init();
        executor.add_node(trajectory_controller);

        RCLCPP_INFO(rclcpp::get_logger("main"), "Both nodes initialized successfully, starting execution...");

        // 4. 运行多线程执行器
        executor.spin();

        // 5. ✅ CRITICAL: 节点完全销毁后再关闭 IPC
        // 顺序很重要：先销毁节点 → 消费者线程停止 → 再清理 IPC 资源
        // 这样可以避免消费者线程在 IPC 资源被删除时仍在访问它们
        RCLCPP_INFO(rclcpp::get_logger("main"), "Executor stopped, destroying nodes...");
        controller_manager.reset();  // 显式销毁节点
        trajectory_controller.reset();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Nodes destroyed, shutting down IPC...");

        // 等待一小段时间确保所有线程完全停止
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        arm_controller::IPCLifecycle::shutdown();
        RCLCPP_INFO(rclcpp::get_logger("main"), "IPC shutdown completed");

    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Arm Controller failed: %s", e.what());
        // ✅ 异常时也要清理 IPC
        // try {
            arm_controller::IPCLifecycle::shutdown();
            rclcpp::shutdown();
            return 1;
        // } catch (...) {}
        // return 1;
    }

    rclcpp::shutdown();
    return 0;
}