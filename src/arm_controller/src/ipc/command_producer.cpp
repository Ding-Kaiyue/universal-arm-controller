#include "arm_controller/ipc/command_producer.hpp"
#include <chrono>
#include <sstream>
#include <cstring>
#include <cmath>

namespace arm_controller::ipc {

// ============================================================================
// CommandValidator 实现
// ============================================================================

CommandValidator::ValidationResult
CommandValidator::validateMapping(const std::string& mapping) {
    if (mapping.empty()) {
        return {false, "Mapping cannot be empty"};
    }
    if (mapping.length() > TrajectoryCommand::MAX_MAPPING_LEN - 1) {
        return {false, "Mapping too long"};
    }
    return {true, ""};
}

CommandValidator::ValidationResult
CommandValidator::validateJointPositions(
    const std::vector<double>& positions) {
    if (positions.empty()) {
        return {false, "Joint positions cannot be empty"};
    }
    if (positions.size() > MAX_JOINTS) {
        return {false, "Joint count exceeds maximum"};
    }
    return {true, ""};
}

CommandValidator::ValidationResult
CommandValidator::validateJointVelocities(
    const std::vector<double>& velocities) {
    if (velocities.empty()) {
        return {false, "Joint velocities cannot be empty"};
    }
    if (velocities.size() > MAX_JOINTS) {
        return {false, "Joint count exceeds maximum"};
    }
    return {true, ""};
}

CommandValidator::ValidationResult
CommandValidator::validateCartesianVelocities(
    const std::vector<double>& velocities) {
    if (velocities.size() != 6) {
        return {false, "Cartesian velocities must have exactly 6 elements"};
    }
    return {true, ""};
}

CommandValidator::ValidationResult
CommandValidator::validatePose(
    double x, double y, double z,
    double qx, double qy, double qz, double qw) {
    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        return {false, "Position contains NaN values"};
    }
    double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (qnorm < 1e-6) {
        return {false, "Quaternion norm is zero"};
    }
    return {true, ""};
}

CommandValidator::ValidationResult
CommandValidator::validateWaypoints(
    const std::vector<double>& waypoints) {
    if (waypoints.empty()) {
        return {false, "Waypoints cannot be empty"};
    }
    if (waypoints.size() % 7 != 0) {
        return {false, "Waypoints size must be multiple of 7 (x,y,z,qx,qy,qz,qw)"};
    }
    if (waypoints.size() / 7 > MAX_JOINTS) {
        return {false, "Waypoint count exceeds maximum"};
    }
    return {true, ""};
}

// ============================================================================
// CommandBuilder 实现
// ============================================================================

CommandBuilder::CommandBuilder(uint32_t producer_id)
    : cmd_() {
    cmd_.producer_id = producer_id;
    updateTimestamp();
    generateCommandId();
}

CommandBuilder& CommandBuilder::withMode(const std::string& mode) {
    cmd_.set_mode(mode);
    return *this;
}

CommandBuilder& CommandBuilder::withMapping(const std::string& mapping) {
    cmd_.set_mapping(mapping);
    return *this;
}

CommandBuilder& CommandBuilder::withJointPositions(
    const std::vector<double>& positions) {
    cmd_.set_parameters(positions);
    return *this;
}

CommandBuilder& CommandBuilder::withJointVelocities(
    const std::vector<double>& velocities) {
    cmd_.set_parameters(velocities);
    return *this;
}

CommandBuilder& CommandBuilder::withCartesianVelocities(
    const std::vector<double>& velocities) {
    cmd_.set_parameters(velocities);
    return *this;
}

CommandBuilder& CommandBuilder::withPose(
    double x, double y, double z,
    double qx, double qy, double qz, double qw) {
    std::vector<double> pose_data = {x, y, z, qx, qy, qz, qw};
    cmd_.set_parameters(pose_data);
    return *this;
}

CommandBuilder& CommandBuilder::withWaypoints(
    const std::vector<double>& waypoints) {
    cmd_.set_parameters(waypoints);
    return *this;
}

TrajectoryCommand CommandBuilder::build() {
    updateTimestamp();
    cmd_.calculateCrc();
    return cmd_;
}

void CommandBuilder::generateCommandId() {
    std::stringstream ss;
    auto now = std::chrono::high_resolution_clock::now();
    auto ns = now.time_since_epoch().count();
    ss << "cmd_" << ns;
    std::string id = ss.str();
    std::strncpy(cmd_.command_id, id.c_str(),
                 TrajectoryCommand::MAX_COMMAND_ID_LEN - 1);
    cmd_.command_id[TrajectoryCommand::MAX_COMMAND_ID_LEN - 1] = '\0';
}

void CommandBuilder::updateTimestamp() {
    auto now = std::chrono::high_resolution_clock::now();
    cmd_.timestamp_ns = now.time_since_epoch().count();
}

// ============================================================================
// CommandProducer 实现
// ============================================================================

CommandProducer::CommandProducer(
    std::shared_ptr<SharedMemoryManager> shm_manager,
    uint32_t producer_id)
    : shm_manager_(shm_manager),
      producer_id_(producer_id),
      initialized_(false) {
}

bool CommandProducer::pushCommand(const TrajectoryCommand& cmd) {
    if (!ensureInitialized()) {
        return false;
    }

    try {
        auto mutex = shm_manager_->getMutex();
        auto condition = shm_manager_->getCondition();
        auto queue = shm_manager_->getQueue();

        if (!mutex || !condition || !queue) {
            last_error_ = "SharedMemoryManager not properly initialized";
            return false;
        }

        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex);
        queue->push_back(cmd);
        condition->notify_one();

        last_error_ = "";
        return true;

    } catch (const std::exception& e) {
        last_error_ = std::string("Exception in pushCommand: ") + e.what();
        return false;
    }
}

bool CommandProducer::ensureInitialized() {
    if (initialized_) {
        return true;
    }

    if (!shm_manager_ || !shm_manager_->isValid()) {
        last_error_ = "SharedMemoryManager is not valid";
        return false;
    }

    initialized_ = true;
    return true;
}

}  // namespace arm_controller::ipc
