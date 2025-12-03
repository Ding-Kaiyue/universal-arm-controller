#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace arm_controller;

int main() {
    std::cout << "===============================================================\n"
              << "ARM Controller 双臂协调运动示例\n"
              << "===============================================================\n\n";

    // 初始化 IPC
    std::cout << "初始化 IPC...\n";
    if (!IPCLifecycle::initialize()) {
        std::cerr << "初始化失败\n";
        return 1;
    }
    std::cout << "初始化成功\n\n";

    movej::MoveJIPCInterface movej;

    // 示例 1: 两臂顺序移动
    std::cout << "[示例 1] 两臂顺序移动\n";
    std::cout << "  左臂移动到 {0.0, 0.5, 1.0, 0.2, 0.3, 0.4}\n";
    if (movej.execute({0.0, 0.5, 1.0, 0.2, 0.3, 0.4}, "left_arm")) {
        std::cout << "    成功\n";
    } else {
        std::cout << "    失败: " << movej.getLastError() << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "  右臂移动到 {0.0, -0.5, -1.0, 0.2, 0.3, 0.4}\n";
    if (movej.execute({0.0, -0.5, -1.0, 0.2, 0.3, 0.4}, "right_arm")) {
        std::cout << "    成功\n\n";
    } else {
        std::cout << "    失败: " << movej.getLastError() << "\n\n";
    }

    // 示例 2: 两臂并行移动
    std::cout << "[示例 2] 两臂并行移动\n";
    std::cout << "  左臂移动到 {0.5, 0.5, 0.5, 0.0, 0.0, 0.0}\n";
    movej.execute({0.5, 0.5, 0.5, 0.0, 0.0, 0.0}, "left_arm");

    std::cout << "  右臂同时移动到 {0.5, -0.5, -0.5, 0.0, 0.0, 0.0}\n";
    movej.execute({0.5, -0.5, -0.5, 0.0, 0.0, 0.0}, "right_arm");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "  两臂并行移动完成\n\n";

    // 关闭 IPC
    std::cout << "关闭 IPC...\n";
    IPCLifecycle::shutdown();

    std::cout << "===============================================================\n"
              << "双臂示例结束！\n"
              << "===============================================================\n";
    return 0;
}
