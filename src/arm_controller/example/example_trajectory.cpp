#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/movel/movel_ipc_interface.hpp"
#include "controller/movec/movec_ipc_interface.hpp"
#include <iostream>
#include <vector>
#include <array>

using namespace arm_controller;

int main() {
    std::cout << "===============================================================\n"
              << "ARM Controller 轨迹执行示例\n"
              << "===============================================================\n\n";

    // 初始化 IPC
    std::cout << "初始化 IPC...\n";
    if (!IPCLifecycle::initialize()) {
        std::cerr << "初始化失败\n";
        return 1;
    }
    std::cout << "初始化成功\n\n";

    movej::MoveJIPCInterface movej;
    movel::MoveLIPCInterface movel;
    movec::MoveCIPCInterface movec;

    // MoveJ 轨迹
    std::cout << "========== MoveJ 轨迹 ==========\n";
    std::vector<std::vector<double>> movej_trajectory = {
        {0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
        {0.2, 0.3, 0.4, 0.5, 0.6, 0.7},
        {0.3, 0.4, 0.5, 0.6, 0.7, 0.8}
    };

    for (size_t i = 0; i < movej_trajectory.size(); ++i) {
        std::cout << "  点 " << (i + 1) << ": ";
        if (movej.execute(movej_trajectory[i], "left_arm")) {
            std::cout << "✅\n";
        } else {
            std::cout << "❌ " << movej.getLastError() << "\n";
        }
    }
    std::cout << "\n";

    // MoveL 轨迹
    std::cout << "========== MoveL 轨迹 ==========\n";
    std::vector<std::array<double, 7>> movel_trajectory = {
        {0.3, 0.4, 0.5, 0.0, 0.0, 0.707, 0.707},
        {0.4, 0.5, 0.6, 0.0, 0.0, 0.707, 0.707},
        {0.5, 0.6, 0.7, 0.0, 0.0, 0.707, 0.707}
    };

    for (size_t i = 0; i < movel_trajectory.size(); ++i) {
        std::cout << "  点 " << (i + 1) << ": ";
        auto& pose = movel_trajectory[i];
        if (movel.execute(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], "left_arm")) {
            std::cout << "✅\n";
        } else {
            std::cout << "❌ " << movel.getLastError() << "\n";
        }
    }
    std::cout << "\n";

    // 关闭 IPC
    std::cout << "关闭 IPC...\n";
    IPCLifecycle::shutdown();

    std::cout << "===============================================================\n"
              << "轨迹示例结束！\n"
              << "===============================================================\n";
    return 0;
}
