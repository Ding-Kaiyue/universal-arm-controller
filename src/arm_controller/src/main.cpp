#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "controller_manager_section.hpp"
#include "trajectory_controller_section.hpp"
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
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
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}