#include "arm_controller/arm_controller_api.hpp"
#include "controller/mink_servo/mink_servo_ipc_interface.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

using namespace arm_controller;

int main() {
    std::cout << "===============================================================\n"
              << "MinkServo IPC Example\n"
              << "===============================================================\n";

    if (!IPCLifecycle::initialize()) {
        std::cerr << "ERROR: IPC initialize failed\n";
        return 1;
    }

    mink_servo::MinkServoIPCInterface mink;
    const std::string mapping = "left_arm";
    // constexpr int interval_ms = 10;

    // [x, y, z, qx, qy, qz, qw] in world frame
    std::vector<double> target = {0.249, -0.496, 0.652, -0.454, 0.45, -0.539, 0.549};

    // 1) Warmup: send one target to enter MinkServo and start tracking.
    if (!mink.execute(target, mapping)) {
        std::cerr << "ERROR: warmup failed: " << mink.getLastError() << "\n";
        return 1;
    }
    std::cout << "Warmup sent.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 2) Move down in 5mm steps: 20 steps -> 10cm total.
    std::cout << "Moving down 10cm in 5mm steps...\n";
    std::vector<double> target_down = target;
    constexpr int step_count = 20;
    constexpr double dz_step = 0.005;  // -5mm each step
    for (int step = 0; step < step_count; ++step) {
        target_down[2] += dz_step;
        if (!mink.execute(target_down, mapping)) {
            std::cerr << "ERROR: step send failed: " << mink.getLastError() << "\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    // 3) Final hold target.
    if (!mink.execute(target_down, mapping)) {
        std::cerr << "WARN: final hold send failed: " << mink.getLastError() << "\n";
    }

    std::cout << "Done.\n";
    return 0;
}
