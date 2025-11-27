#include "arm_controller/arm_controller_api.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace arm_controller;

int main() {
    std::cout << "===============================================================\n"
              << "ArmControllerAPI C++ 示例 - 双臂协调运动\n"
              << "===============================================================\n\n";

    ArmControllerAPI& api = ArmControllerAPI::getInstance();

    // 初始化API
    std::cout << "初始化 API...\n";
    if (!api.initialize()) {
        std::cerr << "初始化失败: " << api.getLastError() << "\n";
        return 1;
    }
    std::cout << "初始化成功\n\n";

    // 示例 1: 两臂顺序移动
    std::cout << "[示例 1] 两臂顺序移动\n";
    std::cout << "  左臂移动到 {0.0, 0.5, 1.0, 0.2, 0.3, 0.4}\n";
    if (api.moveJ({0.0, 0.5, 1.0, 0.2, 0.3, 0.4}, "left_arm")) {
        std::cout << "    成功\n";
    } else {
        std::cout << "    失败: " << api.getLastError() << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "  右臂移动到 {0.0, -0.5, -1.0, 0.2, 0.3, 0.4}\n";
    if (api.moveJ({0.0, -0.5, -1.0, 0.2, 0.3, 0.4}, "right_arm")) {
        std::cout << "    成功\n\n";
    } else {
        std::cout << "    失败: " << api.getLastError() << "\n\n";
    }

    // 示例 2: 两臂并行移动
    std::cout << "[示例 2] 两臂并行移动\n";
    std::cout << "  左臂移动到 {0.5, 0.5, 0.5, 0.0, 0.0, 0.0}\n";
    api.moveJ({0.5, 0.5, 0.5, 0.0, 0.0, 0.0}, "left_arm");

    std::cout << "  右臂同时移动到 {0.5, -0.5, -0.5, 0.0, 0.0, 0.0}\n";
    api.moveJ({0.5, -0.5, -0.5, 0.0, 0.0, 0.0}, "right_arm");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "  两臂并行移动完成\n\n";

    // 示例 3: 双臂轨迹执行
    std::cout << "[示例 3] 双臂轨迹执行 (多个路点)\n";

    std::vector<std::vector<double>> left_trajectory = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, 0.5, 0.5, 0.0, 0.0, 0.0},
        {1.0, 1.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    };

    std::vector<std::vector<double>> right_trajectory = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, -0.5, -0.5, 0.0, 0.0, 0.0},
        {1.0, -1.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    };

    size_t max_waypoints = std::max(left_trajectory.size(), right_trajectory.size());
    for (size_t i = 0; i < max_waypoints; ++i) {
        std::cout << "  路点 " << (i + 1) << "/" << max_waypoints << ":\n";

        if (i < left_trajectory.size()) {
            api.moveJ(left_trajectory[i], "left_arm");
        }

        if (i < right_trajectory.size()) {
            api.moveJ(right_trajectory[i], "right_arm");
        }

        if (i < max_waypoints - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    std::cout << "  轨迹执行完成\n\n";

    // 示例 4: 双臂复杂动作序列
    std::cout << "[示例 4] 双臂复杂动作序列\n";

    std::cout << "  步骤1: 两臂移动到准备位置\n";
    api.moveJ({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, "left_arm");
    api.moveJ({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, "right_arm");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "  步骤2: 左臂靠近\n";
    api.moveJ({0.3, 0.3, 0.3, 0.0, 0.0, 0.0}, "left_arm");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "  步骤3: 右臂靠近\n";
    api.moveJ({0.3, -0.3, -0.3, 0.0, 0.0, 0.0}, "right_arm");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "  步骤4: 两臂并行上升\n";
    api.moveJ({0.3, 0.5, 0.5, 0.0, 0.0, 0.0}, "left_arm");
    api.moveJ({0.3, -0.5, -0.5, 0.0, 0.0, 0.0}, "right_arm");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "  步骤5: 两臂回到初始位置\n";
    api.moveJ({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, "left_arm");
    api.moveJ({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, "right_arm");

    std::cout << "\n关闭 API...\n";
    api.shutdown();

    std::cout << "===============================================================\n"
              << "示例完成！\n"
              << "===============================================================\n";
    return 0;
}