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
    if (!IPCLifecycle::initialize()) {
        std::cerr << "❌ 初始化失败\n";
        return 1;
    }
    std::cout << "✅ 初始化成功\n\n";

    // 创建接口实例
    movej::MoveJIPCInterface movej;
    movel::MoveLIPCInterface movel;
    movec::MoveCIPCInterface movec;

    // 1. MoveJ 演示 - left_arm 先到达关节目标
    std::cout << "========== MoveJ 演示 ==========\n";
    std::cout << "发送 MoveJ 命令 -> left_arm ...\n";
    if (!movej.execute({0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0}, "left_arm")) {
        std::cerr << "❌ 失败: " << movej.getLastError() << "\n";
    } else {
        std::cout << "✅ 已入队\n";
    }
    std::cout << "\n";

    // 2. MoveL 演示 - left_arm 从关节空间目标直线到笛卡尔目标 (0.19, -0.5, 0.63)
    std::cout << "========== MoveL 演示 ==========\n";
    std::cout << "发送 MoveL 命令 -> left_arm ...\n";
    if (!movel.execute(0.19, -0.5, 0.63, -0.4546, 0.4546, -0.5417, 0.5417, "left_arm")) {
        std::cerr << "❌ 失败: " << movel.getLastError() << "\n";
    } else {
        std::cout << "✅ 已入队\n";
    }
    std::cout << "\n";

    // 3. MoveJ 演示
    std::cout << "========== MoveJ 演示 ==========\n";
    std::cout << "发送 MoveJ 命令 -> right_arm ...\n";
    if (!movej.execute({0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0}, "right_arm")) {
        std::cerr << "❌ 失败: " << movej.getLastError() << "\n";
    } else {
        std::cout << "✅ 已入队\n";
    }
    std::cout << "\n";

    // 4. MoveL 演示
    std::cout << "========== MoveL 演示 ==========\n";
    std::cout << "发送 MoveL 命令 -> right_arm ...\n";
    if (!movel.execute(0.19, 0.5, 0.63, -0.4546, 0.4546, -0.5417, 0.5417, "right_arm")) {
        std::cerr << "❌ 失败: " << movel.getLastError() << "\n";
    } else {
        std::cout << "✅ 已入队\n";
    }
    std::cout << "\n";

    // 3. MoveC 演示 - 从 MoveL 目标位置出发的小范围弧形轨迹
    std::cout << "========== MoveC 演示 ==========\n";
    std::cout << "发送 MoveC 命令 (从 MoveL 终点出发)... -> left_arm \n";
    // 起点：MoveL 目标位置 (0.19, -0.5, 0.63)
    // 中间点：向内下方偏移 (+4cm, 0cm, -5cm) = (0.23, -0.5, 0.58)
    // 目标点：继续偏移 (+6cm, 0cm, -8cm) = (0.25, -0.5, 0.55)
    // 形成一个小范围的圆弧，方便规划
    if (!movec.execute({
        0.2300, -0.5000, 0.5800, -0.4546, 0.4546, -0.5417, 0.5417,  // via_point
        0.2500, -0.5000, 0.5500, -0.4546, 0.4546, -0.5417, 0.5417   // goal_point
    }, "left_arm")) {
        std::cerr << "❌ 失败: " << movec.getLastError() << "\n";
    } else {
        std::cout << "✅ 已入队\n";
    }
    std::cout << "\n";

    // 关闭 IPC
    std::cout << "关闭 IPC...\n";
    IPCLifecycle::shutdown();

    std::cout << "===============================================================\n"
              << "演示结束！\n"
              << "===============================================================\n";
    return 0;
}