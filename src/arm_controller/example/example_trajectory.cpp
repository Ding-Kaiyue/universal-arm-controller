#include "arm_controller/arm_controller_api.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace arm_controller;

int main() {
    std::cout << "===============================================================\n"
              << "ArmControllerAPI C++ 示例 - 多路点轨迹执行\n"
              << "===============================================================\n\n";

    ArmControllerAPI& api = ArmControllerAPI::getInstance();

    // 初始化API
    std::cout << "初始化 API...\n";
    if (!api.initialize()) {
        std::cerr << "初始化失败: " << api.getLastError() << "\n";
        return 1;
    }
    std::cout << "初始化成功\n\n";

    // 示例 1: MoveJ多路点轨迹（模式自动切换）
    std::cout << "[示例 1] MoveJ 多路点轨迹\n";
    {
        std::vector<std::vector<double>> movej_trajectory = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.5, 0.5, 0.5, 0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0, 0.0, 0.0, 0.0},
            {1.5, 0.5, -0.5, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        };

        for (size_t i = 0; i < movej_trajectory.size(); ++i) {
            std::cout << "  路点 " << (i + 1) << "/" << movej_trajectory.size() << ": ";
            if (api.moveJ(movej_trajectory[i], "left_arm")) {
                std::cout << "成功\n";
            } else {
                std::cout << "失败 - " << api.getLastError() << "\n";
            }

            if (i < movej_trajectory.size() - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        std::cout << "  MoveJ 轨迹执行完成\n\n";
    }

    // 示例 2: MoveL多路点轨迹（模式自动切换）
    std::cout << "[示例 2] MoveL 多路点轨迹\n";
    {
        std::vector<std::vector<double>> movel_trajectory = {
            {0.3, 0.4, 0.5, 0.0, 0.0, 0.707, 0.707},
            {0.35, 0.42, 0.52, 0.0, 0.0, 0.707, 0.707},
            {0.4, 0.44, 0.54, 0.0, 0.0, 0.707, 0.707},
            {0.3, 0.4, 0.5, 0.0, 0.0, 0.707, 0.707},
        };

        for (size_t i = 0; i < movel_trajectory.size(); ++i) {
            std::cout << "  路点 " << (i + 1) << "/" << movel_trajectory.size() << ": ";
            const auto& wp = movel_trajectory[i];
            if (api.moveL(wp[0], wp[1], wp[2], wp[3], wp[4], wp[5], wp[6], "left_arm")) {
                std::cout << "成功\n";
            } else {
                std::cout << "失败 - " << api.getLastError() << "\n";
            }

            if (i < movel_trajectory.size() - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        std::cout << "  MoveL 轨迹执行完成\n\n";
    }

    // 示例 3: MoveC多路点轨迹（模式自动切换）
    std::cout << "[示例 3] MoveC 多路点轨迹\n";
    {
        std::vector<std::vector<double>> movec_trajectory = {
            {
                0.4, 0.35, 0.45, 0.0, 0.0, 0.707, 0.707,
                0.5, 0.3, 0.4, 0.0, 0.0, 0.707, 0.707
            },
            {
                0.5, 0.25, 0.35, 0.0, 0.0, 0.707, 0.707,
                0.4, 0.3, 0.4, 0.0, 0.0, 0.707, 0.707
            },
        };

        for (size_t i = 0; i < movec_trajectory.size(); ++i) {
            std::cout << "  路点 " << (i + 1) << "/" << movec_trajectory.size() << ": ";
            if (api.moveC(movec_trajectory[i], "left_arm")) {
                std::cout << "成功\n";
            } else {
                std::cout << "失败 - " << api.getLastError() << "\n";
            }

            if (i < movec_trajectory.size() - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        std::cout << "  MoveC 轨迹执行完成\n\n";
    }

    std::cout << "关闭 API...\n";
    api.shutdown();

    std::cout << "===============================================================\n"
              << "示例完成！\n"
              << "===============================================================\n";
    return 0;
}
