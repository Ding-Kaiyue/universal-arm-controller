#include "arm_controller/arm_controller_api.hpp"
#include "controller/reactive_task/reactive_task_ipc_interface.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace arm_controller;

namespace {

int toInt(const ipc::ExecutionState s) {
    return static_cast<int>(s);
}

bool isTerminalState(const ipc::ExecutionState s) {
    return s == ipc::ExecutionState::SUCCESS ||
           s == ipc::ExecutionState::FAILED ||
           s == ipc::ExecutionState::IDLE;
}

void printState(
    const std::string& mapping,
    const std::string& mode,
    const ipc::ExecutionState exec_state,
    const std::string& tag) {
    std::cout << "[ReactiveTask][" << tag << "] mapping=" << mapping
              << " mode=" << mode
              << " exec_state=" << toInt(exec_state) << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::cout << "===============================================================\n"
              << "ReactiveTask Consumer IPC Demo\n"
              << "===============================================================\n\n";

    if (!IPCLifecycle::initialize(argc, argv)) {
        std::cerr << "❌ initializeAsConsumer failed\n";
        return 1;
    }
    std::cout << "✅ 初始化成功\n\n";

    reactive_task::ReactiveTaskIPCInterface reactive_task;

    const std::string mapping = "left_arm";

    // target_pose = [x, y, z, qx, qy, qz, qw]
    const std::vector<double> target = {0.13, -0.50, 0.63, -0.4546, 0.4546, -0.5417, 0.5417};

    std::cout << "========== ReactiveTask (single command safety run) ==========\n";
    std::cout << "mapping: " << mapping << "\n";
    if (!reactive_task.execute(target, mapping)) {
        std::cerr << "❌ enqueue failed: " << reactive_task.getLastError() << "\n";
    } else {
        std::cout << "✅ command queued\n";
    }
    std::string last_mode = reactive_task.getCurrentMode(mapping);
    ipc::ExecutionState last_exec_state = reactive_task.getExecutionState(mapping);
    printState(mapping, last_mode, last_exec_state, "after_enqueue");

    std::cout << "\nPolling state until terminal execution state...\n";
    while (true) {
        const std::string mode = reactive_task.getCurrentMode(mapping);
        const ipc::ExecutionState exec_state = reactive_task.getExecutionState(mapping);

        if (mode != last_mode || exec_state != last_exec_state) {
            printState(mapping, mode, exec_state, "poll");
            last_mode = mode;
            last_exec_state = exec_state;
        }

        if (isTerminalState(exec_state)) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nShutting down IPC...\n";
    IPCLifecycle::shutdown();

    std::cout << "===============================================================\n"
              << "ReactiveTask Consumer demo finished.\n"
              << "===============================================================\n";
    return 0;
}
