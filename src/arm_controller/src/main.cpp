#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "controller_manager_section.hpp"
#include "trajectory_controller_section.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Context valid: %d",
            rclcpp::contexts::get_global_default_context()->is_valid());

    try {
        // 初始化IPC命令队列，带超时和重试机制
        RCLCPP_INFO(rclcpp::get_logger("main"), "Attempting to clean up old IPC resources...");
        try {
            arm_controller::CommandQueueIPC::cleanup();
            RCLCPP_INFO(rclcpp::get_logger("main"), "Old IPC resources cleaned up");
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("main"), "Warning during IPC cleanup: %s", e.what());
        }

        // 初始化IPC，带重试
        int max_retries = 3;
        bool ipc_initialized = false;
        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            RCLCPP_INFO(rclcpp::get_logger("main"), "IPC initialization attempt %d/%d", attempt, max_retries);
            if (arm_controller::CommandQueueIPC::getInstance().initialize()) {
                ipc_initialized = true;
                RCLCPP_INFO(rclcpp::get_logger("main"), "✅ IPC command queue initialized successfully");
                break;
            }
            if (attempt < max_retries) {
                RCLCPP_WARN(rclcpp::get_logger("main"), "IPC initialization failed, retrying in 500ms...");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }

        if (!ipc_initialized) {
            RCLCPP_WARN(rclcpp::get_logger("main"), "⚠️  Failed to initialize IPC command queue after %d attempts. Continuing without IPC support.", max_retries);
        }

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
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Arm Controller failed: %s", e.what());
        // return 1;
    }

    rclcpp::shutdown();
    return 0;
}