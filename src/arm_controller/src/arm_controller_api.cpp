#include "arm_controller/arm_controller_api.hpp"
#include "arm_controller/command_queue_ipc.hpp"
#include <rclcpp/rclcpp.hpp>
#include <controller_interfaces/srv/work_mode.hpp>
#include <chrono>
#include <sstream>
#include <iostream>
#include <thread>

namespace arm_controller {

struct ArmControllerAPI::Impl {
    std::string last_error;
    bool initialized = false;
    std::shared_ptr<rclcpp::Node> node;
};

ArmControllerAPI& ArmControllerAPI::getInstance() {
    static ArmControllerAPI instance;
    return instance;
}

ArmControllerAPI::ArmControllerAPI() : impl_(std::make_unique<Impl>()) {}

ArmControllerAPI::~ArmControllerAPI() {
    shutdown();
}

bool ArmControllerAPI::initialize(int argc, char** argv) {
    try {
        if (!rclcpp::ok()) {
            rclcpp::init(argc, argv);
        }
        if (!impl_->node) {
            impl_->node = rclcpp::Node::make_shared("arm_controller_api");
        }

        if (!CommandQueueIPC::getInstance().open()) {
            impl_->last_error = "Failed to open IPC queue";
            return false;
        }

        impl_->initialized = true;
        impl_->last_error = "";
        return true;
    } catch (const std::exception& e) {
        impl_->last_error = std::string("Initialize failed: ") + e.what();
        return false;
    }
}

void ArmControllerAPI::shutdown() {
    if (impl_->initialized) {
        impl_->node.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        impl_->initialized = false;
    }
}

bool ArmControllerAPI::moveJ(const std::vector<double>& joint_positions, const std::string& mapping) {
    try {
        if (!impl_->initialized) {
            impl_->last_error = "API not initialized. Call initialize() first.";
            return false;
        }

        if (mapping.empty()) {
            impl_->last_error = "Mapping cannot be empty";
            return false;
        }

        if (joint_positions.empty()) {
            impl_->last_error = "Joint positions cannot be empty";
            return false;
        }

        TrajectoryCommandIPC cmd;
        cmd.set_mode("MoveJ");
        cmd.set_mapping(mapping);
        cmd.set_parameters(joint_positions);

        std::stringstream ss;
        ss << "movej_" << std::chrono::high_resolution_clock::now().time_since_epoch().count();
        cmd.set_command_id(ss.str());
        cmd.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();

        CommandQueueIPC::getInstance().push(cmd);

        impl_->last_error = "";
        return true;
    } catch (const std::exception& e) {
        impl_->last_error = std::string("Exception: ") + e.what();
        return false;
    }
}

bool ArmControllerAPI::moveL(double x, double y, double z, double qx, double qy, double qz, double qw,
                              const std::string& mapping) {
    try {
        if (!impl_->initialized) {
            impl_->last_error = "API not initialized. Call initialize() first.";
            return false;
        }

        if (mapping.empty()) {
            impl_->last_error = "Mapping cannot be empty";
            return false;
        }

        TrajectoryCommandIPC cmd;
        cmd.set_mode("MoveL");
        cmd.set_mapping(mapping);
        cmd.set_parameters({x, y, z, qx, qy, qz, qw});

        std::stringstream ss;
        ss << "movel_" << std::chrono::high_resolution_clock::now().time_since_epoch().count();
        cmd.set_command_id(ss.str());
        cmd.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();

        CommandQueueIPC::getInstance().push(cmd);

        impl_->last_error = "";
        return true;
    } catch (const std::exception& e) {
        impl_->last_error = std::string("Exception: ") + e.what();
        return false;
    }
}

bool ArmControllerAPI::moveC(const std::vector<double>& waypoints, const std::string& mapping) {
    try {
        if (!impl_->initialized) {
            impl_->last_error = "API not initialized. Call initialize() first.";
            return false;
        }

        if (mapping.empty()) {
            impl_->last_error = "Mapping cannot be empty";
            return false;
        }

        if (waypoints.empty() || waypoints.size() % 7 != 0) {
            impl_->last_error = "Waypoints must be multiple of 7 (each: x, y, z, qx, qy, qz, qw)";
            return false;
        }

        TrajectoryCommandIPC cmd;
        cmd.set_mode("MoveC");
        cmd.set_mapping(mapping);
        cmd.set_parameters(waypoints);

        std::stringstream ss;
        ss << "movec_" << std::chrono::high_resolution_clock::now().time_since_epoch().count();
        cmd.set_command_id(ss.str());
        cmd.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();

        CommandQueueIPC::getInstance().push(cmd);

        impl_->last_error = "";
        return true;
    } catch (const std::exception& e) {
        impl_->last_error = std::string("Exception: ") + e.what();
        return false;
    }
}

bool ArmControllerAPI::setJointVelocity(const std::vector<double>& velocities [[maybe_unused]], const std::string& mapping [[maybe_unused]]) {
    impl_->last_error = "Not implemented";
    return false;
}

bool ArmControllerAPI::pauseTrajectory(const std::string& mapping [[maybe_unused]]) {
    impl_->last_error = "Not implemented";
    return false;
}

bool ArmControllerAPI::resumeTrajectory(const std::string& mapping [[maybe_unused]]) {
    impl_->last_error = "Not implemented";
    return false;
}

bool ArmControllerAPI::cancelTrajectory(const std::string& mapping [[maybe_unused]]) {
    impl_->last_error = "Not implemented";
    return false;
}

std::vector<double> ArmControllerAPI::getCurrentJointPositions(const std::string& mapping [[maybe_unused]]) {
    return {};
}

bool ArmControllerAPI::isRobotStopped(const std::string& mapping [[maybe_unused]]) {
    return true;
}

std::string ArmControllerAPI::getLastError() const {
    return impl_->last_error;
}

} // namespace arm_controller
