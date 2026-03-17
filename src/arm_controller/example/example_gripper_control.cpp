#include "arm_controller/arm_controller_api.hpp"
#include "controller/basic_ops/basic_ops_ipc_interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace arm_controller;

int main(int argc, char** argv) {
    std::cout << "===============================================================\n"
              << "ARM Controller Gripper IPC Example\n"
              << "===============================================================\n\n";

    if (!IPCLifecycle::initialize(argc, argv)) {
        std::cerr << "❌ IPC initialize failed\n";
        return 1;
    }
    std::cout << "✅ IPC initialized\n";

    basic_ops::BasicOpsIPCInterface gripper;
    const std::string mapping = "left_gripper";  // 可改成你的 gripper mapping
    const int velocity_raw = 128;
    const int effort_raw = 128;
    const int gripper_type = -1; // auto: 按 mapping 推断

    // open
    if (!gripper.gripper_control(255, mapping, velocity_raw, effort_raw, gripper_type)) {
        std::cerr << "❌ open failed: " << gripper.getLastError() << "\n";
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    // close
    if (!gripper.gripper_control(0, mapping, velocity_raw, effort_raw, gripper_type)) {
        std::cerr << "❌ close failed: " << gripper.getLastError() << "\n";
        return 1;
    }
    std::cout << "✅ Gripper demo finished.\n";
    return 0;
}
