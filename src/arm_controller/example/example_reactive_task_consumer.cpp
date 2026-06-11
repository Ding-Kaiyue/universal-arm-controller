#include "arm_controller/arm_controller_api.hpp"
#include "controller_interfaces/srv/query_distance_field.hpp"
#include "controller/reactive_task/reactive_task_ipc_interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

using namespace arm_controller;

namespace {

constexpr const char* kPointcloudTopic = "/camera_driver/obstacle_pointcloud";
constexpr const char* kEsdfServiceName = "/camera_driver/query_distance_field";

struct RuntimeMapConfig {
    std::string distance_field_source{"camera_driver_esdf"};
    std::string collision_map_source{"camera_driver_pointcloud"};
    std::string pointcloud_topic{kPointcloudTopic};
    std::string esdf_service_name{kEsdfServiceName};
};

int toInt(const ipc::ExecutionState s) {
    return static_cast<int>(s);
}

const char* toString(const ipc::ExecutionState s) {
    switch (s) {
        case ipc::ExecutionState::IDLE:
            return "IDLE";
        case ipc::ExecutionState::PENDING:
            return "PENDING";
        case ipc::ExecutionState::EXECUTING:
            return "EXECUTING";
        case ipc::ExecutionState::SUCCESS:
            return "SUCCESS";
        case ipc::ExecutionState::FAILED:
            return "FAILED";
        default:
            return "UNKNOWN";
    }
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
              << " exec_state=" << toString(exec_state)
              << "(" << toInt(exec_state) << ")\n";
}

bool waitForPublishers(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<std::string>& topics,
    const std::chrono::seconds timeout) {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        bool all_ready = true;
        std::cout << "[Pointcloud Precheck]";
        for (const std::string& topic : topics) {
            const size_t pub_count = node->count_publishers(topic);
            std::cout << " " << topic << "=" << pub_count;
            if (pub_count == 0) {
                all_ready = false;
            }
        }
        std::cout << "\n";

        if (all_ready) {
            return true;
        }

        if (std::chrono::steady_clock::now() - start >= timeout) {
            return false;
        }

        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
}

bool waitForEsdfService(
    const rclcpp::Node::SharedPtr& node,
    const std::string& service_name,
    const std::chrono::seconds timeout) {
    auto client =
        node->create_client<controller_interfaces::srv::QueryDistanceField>(service_name);
    const auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        const bool ready = client->service_is_ready();
        std::cout << "[ESDF Precheck] " << service_name
                  << " ready=" << (ready ? "true" : "false") << "\n";
        if (ready) {
            return true;
        }
        if (std::chrono::steady_clock::now() - start >= timeout) {
            return false;
        }
        client->wait_for_service(std::chrono::milliseconds(500));
        rclcpp::spin_some(node);
    }
    return false;
}

RuntimeMapConfig loadRuntimeMapConfig() {
    RuntimeMapConfig cfg;

    try {
        const std::string config_path =
            ament_index_cpp::get_package_share_directory("arm_controller") +
            "/config/reactive_task_config.yaml";
        const YAML::Node root = YAML::LoadFile(config_path);
        const YAML::Node rtc = root["reactive_task_controller"];
        if (!rtc || !rtc.IsMap()) {
            return cfg;
        }

        if (rtc["distance_field_source"]) {
            cfg.distance_field_source = rtc["distance_field_source"].as<std::string>();
        }
        if (rtc["collision_map_source"]) {
            cfg.collision_map_source = rtc["collision_map_source"].as<std::string>();
        }
        if (const YAML::Node pointcloud = rtc["camera_driver_pointcloud"];
            pointcloud && pointcloud.IsMap() && pointcloud["pointcloud_topic"]) {
            cfg.pointcloud_topic = pointcloud["pointcloud_topic"].as<std::string>();
        }
        if (const YAML::Node esdf = rtc["camera_driver_esdf"];
            esdf && esdf.IsMap() && esdf["service_name"]) {
            cfg.esdf_service_name = esdf["service_name"].as<std::string>();
        }
    } catch (const std::exception& e) {
        std::cerr << "[WARN] failed to load reactive_task_config.yaml, using defaults: "
                  << e.what() << "\n";
    }

    return cfg;
}

bool runMapPrecheck(
    const rclcpp::Node::SharedPtr& node,
    const RuntimeMapConfig& cfg,
    const std::chrono::seconds timeout) {
    bool ok = true;
    if (cfg.distance_field_source == "camera_driver_esdf") {
        std::cout << "ReactiveTask distance_field_source=camera_driver_esdf\n";
        std::cout << "Waiting for ESDF query service:\n"
                  << "  - " << cfg.esdf_service_name << "\n";
        ok = waitForEsdfService(node, cfg.esdf_service_name, timeout) && ok;
    } else if (cfg.distance_field_source == "camera_driver_pointcloud") {
        std::cout << "ReactiveTask distance_field_source=camera_driver_pointcloud\n";
    } else if (cfg.distance_field_source == "dummy") {
        std::cout << "ReactiveTask distance_field_source=dummy\n";
    } else {
        std::cerr << "[WARN] unknown distance_field_source='"
                  << cfg.distance_field_source << "', skipping ESDF precheck.\n";
    }

    if (cfg.collision_map_source == "camera_driver_pointcloud") {
        std::cout << "ReactiveTask collision_map_source=camera_driver_pointcloud\n";
        std::cout << "Waiting for realtime obstacle pointcloud topic:\n"
                  << "  - " << cfg.pointcloud_topic << "\n";
        ok = waitForPublishers(node, {cfg.pointcloud_topic}, timeout) && ok;
    } else if (cfg.collision_map_source == "dummy") {
        std::cout << "ReactiveTask collision_map_source=dummy\n";
    } else {
        std::cerr << "[WARN] unknown collision_map_source='"
                  << cfg.collision_map_source
                  << "', skipping pointcloud precheck.\n";
    }
    return ok;
}

std::string timeoutHint(const RuntimeMapConfig& cfg) {
    if (cfg.collision_map_source == "camera_driver_pointcloud") {
        return "Check controller log for collision_map_source=camera_driver_pointcloud, active_cells, local_trajopt obstacle count, and camera_driver pointcloud publishing.";
    }
    if (cfg.distance_field_source == "camera_driver_pointcloud") {
        return "Check controller log for distance_field_source=camera_driver_pointcloud, frames/cells, local_trajopt obstacle count, and camera_driver pointcloud publishing.";
    }
    if (cfg.distance_field_source == "camera_driver_esdf") {
        return "Check controller log for distance_field_source=camera_driver_esdf and service_ready.";
    }
    return "Check controller log for ReactiveTask map initialization.";
}

}  // namespace

int main(int argc, char** argv) {
    std::cout << "===============================================================\n"
              << "ReactiveTask Consumer Demo\n"
              << "===============================================================\n\n";

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("example_reactive_task_consumer");
    const RuntimeMapConfig map_cfg = loadRuntimeMapConfig();

    std::cout << "Loaded reactive_task_config.yaml distance_field_source="
              << map_cfg.distance_field_source
              << " collision_map_source=" << map_cfg.collision_map_source
              << "\n";
    if (!runMapPrecheck(node, map_cfg, std::chrono::seconds(8))) {
        std::cerr << "[ERROR] map precheck failed within timeout.\n"
                  << "Please start universial_arm_controller_node and camera_driver first.\n";
        rclcpp::shutdown();
        return 1;
    }

    if (!IPCLifecycle::initialize(argc, argv)) {
        std::cerr << "[ERROR] initialize failed\n";
        rclcpp::shutdown();
        return 1;
    }
    std::cout << "[OK] IPC initialized\n\n";

    reactive_task::ReactiveTaskIPCInterface reactive_task;

    const std::string mapping = "left_arm";

    // target_pose = [x, y, z, qx, qy, qz, qw]
    // const std::vector<double> target = {
    //     -0.1, -0.6, 0.58, -0.4546, 0.4546, -0.5417, 0.5417};
    // const std::vector<double> target = {
    //     -0.093, -0.559, 0.555, -0.520, 0.289, -0.388, 0.704};
    // const std::vector<double> target = {
    //     -0.179, -0.815, 0.502, -0.159, 0.484, -0.593, 0.623};
    // const std::vector<double> target = {
    //     0.206, -0.256, 0.544, -0.276, 0.152, -0.250, 0.916};
    const std::vector<double> target = {
         0.270, -0.286, 0.647, -0.200, 0.159, -0.329, 0.909};

    std::cout << "========== ReactiveTask safety run ==========\n";
    std::cout << "mapping: " << mapping << "\n";
    if (!reactive_task.execute(target, mapping)) {
        std::cerr << "[ERROR] enqueue failed: " << reactive_task.getLastError() << "\n";
        IPCLifecycle::shutdown();
        rclcpp::shutdown();
        return 1;
    }
    std::cout << "[OK] command queued\n";

    std::string last_mode = reactive_task.getCurrentMode(mapping);
    ipc::ExecutionState last_exec_state = reactive_task.getExecutionState(mapping);
    printState(mapping, last_mode, last_exec_state, "after_enqueue");

    std::cout << "\nPolling state until terminal execution state...\n";
    const auto poll_start = std::chrono::steady_clock::now();
    constexpr auto kPollTimeout = std::chrono::seconds(45);

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

        if (std::chrono::steady_clock::now() - poll_start > kPollTimeout) {
            std::cerr << "[ERROR] timeout waiting for terminal state.\n"
                      << timeoutHint(map_cfg) << "\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nShutting down IPC...\n";
    IPCLifecycle::shutdown();
    rclcpp::shutdown();

    std::cout << "===============================================================\n"
              << "ReactiveTask Consumer demo finished.\n"
              << "===============================================================\n";
    return 0;
}
