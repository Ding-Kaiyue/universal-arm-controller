#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/movel/movel_ipc_interface.hpp"
#include "controller/movec/movec_ipc_interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace arm_controller;

int main() {
    std::cout << "===============================================================\n"
              << "ARM Controller IPC 演示\n"
              << "===============================================================\n\n";

    // 初始化 IPC
    std::cout << "📍 初始化 IPC...\n";
    if (!IPCLifecycle::initialize()) {
        std::cerr << "❌ 初始化失败\n";
        return 1;
    }
    std::cout << "✅ 初始化成功\n\n";

    // 创建接口实例
    movej::MoveJIPCInterface movej;
    // movel::MoveLIPCInterface movel;
    // movec::MoveCIPCInterface movec;

    // MoveJ 演示
    std::cout << "========== MoveJ 演示 ==========\n";
    std::cout << "发送 MoveJ 命令 -> left_arm ...\n";
    if (!movej.execute({0.0, 0.5, 1.0, 0.2, 0.3, 0.4}, "left_arm")) {
        std::cerr << "❌ 失败: " << movej.getLastError() << "\n";
    } else {
        std::cout << "✅ 已入队\n";
    }
    std::cout << "\n";

    // MoveJ 演示
    std::cout << "========== MoveJ 演示 ==========\n";
    std::cout << "发送 MoveJ 命令 -> right_arm ...\n";
    if (!movej.execute({0.0, 0.5, 1.0, 0.2, 0.3, 0.4}, "right_arm")) {
        std::cerr << "❌ 失败: " << movej.getLastError() << "\n";
    } else {
        std::cout << "✅ 已入队\n";
    }
    std::cout << "\n";

    // MoveL 演示
    // std::cout << "========== MoveL 演示 ==========\n";
    // std::cout << "发送 MoveL 命令...\n";
    // if (!movel.execute(0.19, -0.5, 0.63, -0.4546, 0.4546, -0.5417, 0.5417, "left_arm")) {
    //     std::cerr << "❌ 失败: " << movel.getLastError() << "\n";
    // } else {
    //     std::cout << "✅ 已入队\n";
    // }
    // std::cout << "\n";

    // // MoveC 演示
    // std::cout << "========== MoveC 演示 ==========\n";
    // std::cout << "发送 MoveC 命令...\n";
    // if (!movec.execute({
    //     0.4, 0.35, 0.45, 0.0, 0.0, 0.707, 0.707,
    //     0.5, 0.3, 0.4, 0.0, 0.0, 0.707, 0.707
    // }, "left_arm")) {
    //     std::cerr << "❌ 失败: " << movec.getLastError() << "\n";
    // } else {
    //     std::cout << "✅ 已入队\n";
    // }
    // std::cout << "\n";

    // 关闭 IPC
    std::cout << "关闭 IPC...\n";
    IPCLifecycle::shutdown();

    std::cout << "===============================================================\n"
              << "演示结束！\n"
              << "===============================================================\n";
    return 0;
}
