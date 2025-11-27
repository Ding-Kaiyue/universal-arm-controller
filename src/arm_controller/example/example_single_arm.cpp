#include "arm_controller/arm_controller_api.hpp"
#include <iostream>

using namespace arm_controller;

int main() {
    std::cout << "===============================================================\n"
              << "ArmControllerAPI C++ 示例 - 单臂基础命令\n"
              << "===============================================================\n\n";

    ArmControllerAPI& api = ArmControllerAPI::getInstance();

    // 初始化API
    std::cout << "初始化 API...\n";
    if (!api.initialize()) {
        std::cerr << "初始化失败: " << api.getLastError() << "\n";
        return 1;
    }
    std::cout << "初始化成功\n\n";

    // 示例 1: MoveJ - 关节空间运动（模式自动切换）
    std::cout << "[示例 1] MoveJ - 关节空间运动\n";
    std::cout << "  执行: api.moveJ({0.0, 0.5, 1.0, 0.2, 0.3, 0.4}, \"left_arm\")\n";
    if (api.moveJ({0.0, 0.5, 1.0, 0.2, 0.3, 0.4}, "left_arm")) {
        std::cout << "  结果: 成功\n\n";
    } else {
        std::cout << "  结果: 失败 - " << api.getLastError() << "\n\n";
    }

    // // 示例 3: 另一个MoveJ位置
    // std::cout << "[示例 3] MoveJ - 其他位置\n";
    // std::cout << "  执行: api.moveJ({0.5, 1.0, 0.5, 0.1, 0.2, 0.3}, \"left_arm\")\n";
    // if (api.moveJ({0.5, 1.0, 0.5, 0.1, 0.2, 0.3}, "left_arm")) {
    //     std::cout << "  结果: 成功\n\n";
    // } else {
    //     std::cout << "  结果: 失败 - " << api.getLastError() << "\n\n";
    // }

    // // 示例 4: 切换到 MoveL 模式
    // std::cout << "[示例 4] 切换到 MoveL 模式\n";
    // if (api.setMode("MoveL", "left_arm")) {
    //     std::cout << "  成功: 模式切换完成\n\n";
    // } else {
    //     std::cout << "  失败: " << api.getLastError() << "\n\n";
    // }

    // // 示例 5: MoveL - 直线运动
    // std::cout << "[示例 5] MoveL - 直线运动\n";
    // std::cout << "  执行: api.moveL(0.3, 0.4, 0.5, 0.0, 0.0, 0.707, 0.707, \"left_arm\")\n";
    // if (api.moveL(0.3, 0.4, 0.5, 0.0, 0.0, 0.707, 0.707, "left_arm")) {
    //     std::cout << "  结果: 成功\n\n";
    // } else {
    //     std::cout << "  结果: 失败 - " << api.getLastError() << "\n\n";
    // }

    // // 示例 6: 切换到 MoveC 模式
    // std::cout << "[示例 6] 切换到 MoveC 模式\n";
    // if (api.setMode("MoveC", "left_arm")) {
    //     std::cout << "  成功: 模式切换完成\n\n";
    // } else {
    //     std::cout << "  失败: " << api.getLastError() << "\n\n";
    // }

    // // 示例 7: MoveC - 圆弧运动
    // std::cout << "[示例 7] MoveC - 圆弧运动 (中间点+终点)\n";
    // std::cout << "  执行: api.moveC({...}, \"left_arm\")\n";
    // if (api.moveC({
    //     0.4, 0.35, 0.45, 0.0, 0.0, 0.707, 0.707,
    //     0.5, 0.3, 0.4, 0.0, 0.0, 0.707, 0.707
    // }, "left_arm")) {
    //     std::cout << "  结果: 成功\n\n";
    // } else {
    //     std::cout << "  结果: 失败 - " << api.getLastError() << "\n\n";
    // }

    // 关闭API
    std::cout << "关闭 API...\n";
    api.shutdown();

    std::cout << "===============================================================\n"
              << "示例完成！\n"
              << "===============================================================\n";
    return 0;
}
