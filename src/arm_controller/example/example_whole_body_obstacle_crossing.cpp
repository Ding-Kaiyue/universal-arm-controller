#include "arm_controller/arm_controller_api.hpp"
#include "controller/reactive_task/reactive_task_ipc_interface.hpp"
#include "controller_interfaces/srv/query_distance_field.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <cstdlib>
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

const char* toString(const ipc::ExecutionState state) {
    switch (state) {
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

bool waitForPublishers(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<std::string>& topics,
    const std::chrono::seconds timeout) {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        bool all_ready = true;
        std::cout << "[Pointcloud Precheck]";
        for (const std::string& topic : topics) {
            const std::size_t pub_count = node->count_publishers(topic);
            std::cout << " " << topic << "=" << pub_count;
            all_ready = all_ready && pub_count > 0;
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
        std::cerr << "[WARN] failed to load reactive_task_config.yaml: "
                  << e.what() << "\n";
    }
    return cfg;
}

bool runMapPrecheck(
    const rclcpp::Node::SharedPtr& node,
    const RuntimeMapConfig& cfg,
    const std::chrono::seconds timeout) {
    bool ok = true;
    if (cfg.distance_field_source == "camera_driver_esdf" ||
        cfg.collision_map_source == "camera_driver_esdf") {
        ok = waitForEsdfService(node, cfg.esdf_service_name, timeout) && ok;
    }
    if (cfg.collision_map_source == "camera_driver_pointcloud") {
        ok = waitForPublishers(node, {cfg.pointcloud_topic}, timeout) && ok;
    }
    return ok;
}

void printPose(const char* name, const std::vector<double>& pose) {
    std::cout << "  " << name << ": [";
    for (std::size_t i = 0; i < pose.size(); ++i) {
        std::cout << pose[i] << (i + 1u == pose.size() ? "" : ", ");
    }
    std::cout << "]\n";
}

void printState(
    const reactive_task::ReactiveTaskIPCInterface& reactive_task,
    const std::string& mapping,
    const char* tag) {
    const ipc::ExecutionState state = reactive_task.getExecutionState(mapping);
    std::cout << "[WholeBodyCrossing][" << tag << "] mapping=" << mapping
              << " mode=" << reactive_task.getCurrentMode(mapping)
              << " state=" << toString(state)
              << "(" << static_cast<int>(state) << ")\n";
}

bool waitForGoal(
    const reactive_task::ReactiveTaskIPCInterface& reactive_task,
    const std::string& mapping,
    const std::chrono::seconds timeout) {
    const auto start = std::chrono::steady_clock::now();
    ipc::ExecutionState last_state = ipc::ExecutionState::IDLE;
    while (rclcpp::ok()) {
        const ipc::ExecutionState state = reactive_task.getExecutionState(mapping);
        if (state != last_state) {
            printState(reactive_task, mapping, "poll");
            last_state = state;
        }
        if (state == ipc::ExecutionState::SUCCESS) {
            return true;
        }
        if (state == ipc::ExecutionState::FAILED) {
            return false;
        }
        if (std::chrono::steady_clock::now() - start > timeout) {
            std::cerr << "[ERROR] timeout waiting for " << mapping << ".\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

double parseDoubleOrDefault(char** argv, const int index, const double fallback) {
    if (argv == nullptr || argv[index] == nullptr) {
        return fallback;
    }
    char* end = nullptr;
    const double value = std::strtod(argv[index], &end);
    return (end != argv[index]) ? value : fallback;
}

}  // namespace

int main(int argc, char** argv) {
    std::cout << "===============================================================\n"
              << "Whole-Body Obstacle Crossing Demo\n"
              << "===============================================================\n\n";

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("example_whole_body_obstacle_crossing");
    const RuntimeMapConfig map_cfg = loadRuntimeMapConfig();

    std::cout << "Map config: distance_field_source="
              << map_cfg.distance_field_source
              << " collision_map_source=" << map_cfg.collision_map_source
              << "\n";
    if (!runMapPrecheck(node, map_cfg, std::chrono::seconds(8))) {
        std::cerr << "[ERROR] map precheck failed. Start robotic_arm_sim.launch.py "
                  << "with pillar_obstacles.world and wait for camera_driver_sim first.\n";
        rclcpp::shutdown();
        return 1;
    }

    if (!IPCLifecycle::initialize(argc, argv)) {
        std::cerr << "[ERROR] IPC initialize failed\n";
        rclcpp::shutdown();
        return 1;
    }

    reactive_task::ReactiveTaskIPCInterface reactive_task;

    const double target_x = (argc > 1) ? parseDoubleOrDefault(argv, 1, 3.95) : 3.95;
    const double target_y = (argc > 2) ? parseDoubleOrDefault(argv, 2, 0.0) : 0.0;
    const double hand_spacing = (argc > 3) ? parseDoubleOrDefault(argv, 3, 1.234) : 1.234;

    // Target poses are in the simulation world/odom frame:
    // [x, y, z, qx, qy, qz, qw].
    //
    // The defaults place the hands beyond the pillar field in
    // robot_simulation/worlds/pillar_obstacles.world, forcing the 15D planner
    // to move the omnidirectional base while keeping the dual-arm goal
    // synchronized. robotic_arm_sim.launch.py uses the sim ESDF config so this
    // global target is inside the Gazebo planning map from startup.
    const std::vector<double> left_target =
        {target_x, target_y + 0.5 * hand_spacing, 0.960,
         0.653, -0.271, -0.653, -0.271};
    const std::vector<double> right_target =
        {target_x, target_y - 0.5 * hand_spacing, 0.960,
         0.653, 0.271, -0.653, 0.271};

    std::cout << "Queueing synchronized whole-body dual-arm goal.\n";
    std::cout << "Usage override: ros2 run arm_controller "
              << "example_whole_body_obstacle_crossing [target_x target_y hand_spacing]\n";
    printPose("left", left_target);
    printPose("right", right_target);

    if (!reactive_task.executeDualArm(left_target, right_target, "dual_arm")) {
        std::cerr << "[ERROR] enqueue failed for dual_arm: "
                  << reactive_task.getLastError() << "\n";
        IPCLifecycle::shutdown();
        rclcpp::shutdown();
        return 1;
    }
    printState(reactive_task, "dual_arm", "after_enqueue");

    const bool success = waitForGoal(
        reactive_task, "dual_arm", std::chrono::seconds(180));

    IPCLifecycle::shutdown();
    rclcpp::shutdown();

    std::cout << "===============================================================\n"
              << "Whole-Body Obstacle Crossing Demo "
              << (success ? "finished successfully" : "failed")
              << ".\n"
              << "===============================================================\n";
    return success ? 0 : 2;
}
