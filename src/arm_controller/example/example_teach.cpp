#include "arm_controller/arm_controller_api.hpp"
#include "controller/trajectory_record/trajectory_record_ipc_interface.hpp"
#include "controller/trajectory_replay/trajectory_replay_ipc_interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace arm_controller;

int main() {
    std::cout << "===============================================================\n"
              << "ARM Controller TrajectoryRecord 示教模式 IPC 演示\n"
              << "===============================================================\n\n";

    // 初始化 IPC
    if (!IPCLifecycle::initialize()) {
        std::cerr << "❌ 初始化失败\n";
        return 1;
    }
    std::cout << "✅ IPC 初始化成功\n\n";

    // 创建接口实例
    trajectory_record::TrajectoryRecordIPCInterface trajectory_record;

    const int duration_ms = 10000;  // 10 秒录制时间
    std::string filename = "trajectory_demo";

    // ============ 1️⃣ 开始记录 ============
    // std::cout << "========== 开始轨迹录制 (所有 active mappings) ==========\n";
    // std::cout << "发送 startRecording 命令...\n";

    // if (!trajectory_record.startRecording(filename, "*")) {
    //     std::cerr << "❌ 失败: " << trajectory_record.getLastError() << "\n";
    // } else {
    //     std::cout << "✅ 已开始录制所有 mappings\n";
    // }
    // std::cout << "\n";

    // // ============ 2️⃣ 等待 10 秒进行录制 ============
    // std::cout << "⏱️  开始记录 (10秒)...\n";
    // auto start_time = std::chrono::steady_clock::now();

    // while (true) {
    //     auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    //         std::chrono::steady_clock::now() - start_time).count();

    //     if (elapsed >= duration_ms) {
    //         break;
    //     }

    //     // 每秒输出进度
    //     int remaining = (duration_ms - elapsed) / 1000;
    //     if (elapsed % 1000 < 50) {  // 每秒输出一次
    //         std::cout << "\r   剩余时间: " << remaining << "s   " << std::flush;
    //     }

    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    // std::cout << "\n✅ 录制完成\n\n";

    // // ============ 3️⃣ 停止记录 ============
    // std::cout << "========== 完成轨迹录制 ==========\n";
    // std::cout << "发送 stopRecording 命令 -> left_arm 和 right_arm ...\n";

    // if (!trajectory_record.stopRecording("*")) {
    //     std::cerr << "❌ 失败: " << trajectory_record.getLastError() << "\n";
    // } else {
    //     std::cout << "✅ 已保存\n";
    // }

    // // ============ 4️⃣ 等待处理完成 ============
    // std::cout << "⏳ 等待轨迹平滑处理...\n";
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // std::cout << "✅ 处理完成\n\n";

    // // ============ 5️⃣ 等待 1 秒后开始复现 ============
    // std::cout << "⏱️  等待 1 秒后开始轨迹复现...\n";
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 创建回放接口实例
    trajectory_replay::TrajectoryReplayIPCInterface trajectory_replay;

    std::cout << "========== 开始轨迹复现 (所有 active mappings) ==========\n";
    std::cout << "发送 startReplay 命令...\n";

    if (!trajectory_replay.startReplay(filename, "*")) {
        std::cerr << "❌ 失败: " << trajectory_replay.getLastError() << "\n";
    } else {
        std::cout << "✅ 已开始复现所有 mappings\n";
    }
    std::cout << "\n";

    // ============ 6️⃣ 等待复现完成 ============
    std::cout << "⏳ 轨迹复现中...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
    std::cout << "✅ 复现完成\n\n";

    // ============ 7️⃣ 清理资源 ============
    std::cout << "关闭 IPC...\n";
    IPCLifecycle::shutdown();

    std::cout << "===============================================================\n"
              << "演示结束！\n"
              << "===============================================================\n";

    return 0;
}
