#include "arm_controller/arm_controller_api.hpp"
#include "controller/joint_velocity/joint_velocity_ipc_interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace arm_controller;

void move_arm(const std::string& mapping, const std::vector<double>& velocity, int duration_ms) {
    joint_velocity::JointVelocityIPCInterface joint_velocity;
    const int interval_ms = 50;  // 50ms 间隔（< 100ms 超时）

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < duration_ms) {
        if (!joint_velocity.execute(velocity, mapping)) {
            std::cerr << "❌ [" << mapping << "] 失败: " << joint_velocity.getLastError() << "\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }

    // 停止运动
    std::vector<double> zero(velocity.size(), 0.0);
    joint_velocity.execute(zero, mapping);
}

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

    const int duration_ms = 3000;  // 3 秒

    // JointVelocity 演示 - left_arm 和 right_arm 同时运动
    std::cout << "========== JointVelocity 双臂同时演示 (3秒) ==========\n";
    std::cout << "发送持续 JointVelocity 命令 -> left_arm 和 right_arm ...\n";

    // 创建两个线程，分别控制左右臂
    std::thread left_thread(move_arm, "left_arm", std::vector<double>{0.2, 0.0, 0.0, 0.0, 0.0, 0.0}, duration_ms);
    std::thread right_thread(move_arm, "right_arm", std::vector<double>{-0.2, 0.0, 0.0, 0.0, 0.0, 0.0}, duration_ms);

    // 等待两个线程完成
    left_thread.join();
    right_thread.join();

    std::cout << "✅ 双臂运动完成\n\n";

    // 关闭 IPC
    std::cout << "关闭 IPC ...\n";
    IPCLifecycle::shutdown();
    std::cout << "===============================================================\n"
              << "演示结束！\n"
              << "===============================================================\n";

    return 0;
}