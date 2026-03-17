#include "basic_ops_ipc_service.hpp"

#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::basic_ops {

BasicOpsIPCService::BasicOpsIPCService(const rclcpp::Node::SharedPtr& node,
                                       const std::shared_ptr<HardwareManager>& hardware_manager)
    : node_(node), hardware_manager_(hardware_manager) {}

BasicOpsIPCService::~BasicOpsIPCService() {
    stop();
}

void BasicOpsIPCService::start() {
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        return;
    }

    gripper_thread_ = std::make_unique<std::thread>(&BasicOpsIPCService::gripper_consumer_loop, this);
    motor_enable_thread_ = std::make_unique<std::thread>(&BasicOpsIPCService::motor_enable_consumer_loop, this);
    motor_disable_thread_ = std::make_unique<std::thread>(&BasicOpsIPCService::motor_disable_consumer_loop, this);

    RCLCPP_INFO(node_->get_logger(), "[BasicOpsIPC] ✅ Started Gripper/Motor IPC consumers");
}

void BasicOpsIPCService::stop() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }

    auto join_thread = [](std::unique_ptr<std::thread>& t) {
        if (t && t->joinable()) {
            t->join();
        }
        t.reset();
    };

    join_thread(gripper_thread_);
    join_thread(motor_enable_thread_);
    join_thread(motor_disable_thread_);

    RCLCPP_INFO(node_->get_logger(), "[BasicOpsIPC] Stopped");
}

void BasicOpsIPCService::update_execution_state(const std::string& mapping, bool success) {
    auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
    if (!state_mgr) {
        return;
    }
    state_mgr->setExecutionState(
        success ? arm_controller::ipc::ExecutionState::SUCCESS
                : arm_controller::ipc::ExecutionState::FAILED);
}

bool BasicOpsIPCService::execute_gripper_position_command(const std::string& mapping,
                                                          int gripper_type,
                                                          uint8_t position_raw,
                                                          uint8_t velocity_raw,
                                                          uint8_t effort_raw) {
    if (!hardware_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Gripper command failed: hardware manager unavailable", mapping.c_str());
        return false;
    }

    const auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Gripper command failed: hardware driver not initialized", mapping.c_str());
        return false;
    }

    const std::string interface = hardware_manager_->get_interface_for_mapping(mapping);
    if (interface.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Gripper command failed: interface not configured", mapping.c_str());
        return false;
    }

    const auto& mapping_joint_names = hardware_manager_->get_joint_names(mapping);
    if (mapping_joint_names.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Gripper command failed: no joint names configured", mapping.c_str());
        return false;
    }

    if (gripper_type < 0) {
        // 未显式指定时，根据 mapping 的 robot_type 自动推断。
        const auto& robot_type = hardware_manager_->get_robot_type(mapping);
        if (robot_type.find("omnipicker") != std::string::npos ||
            robot_type.find("OmniPicker") != std::string::npos) {
            gripper_type = 0;
        } else {
            gripper_type = 1;
        }
    }

    const double ratio = std::clamp(static_cast<double>(position_raw) / 255.0, 0.0, 1.0);

    uint8_t cmd_position = 0;
    uint8_t cmd_velocity = 0;
    uint8_t cmd_effort = 0;
    double target_pos = 0.0;

    if (gripper_type == 0) {
        // OmniPicker: 0~255 直传
        cmd_position = position_raw;
        cmd_velocity = velocity_raw;
        cmd_effort = effort_raw;
    } else {
        // 优先寻找 finger1 作为主动关节；找不到则退化为第一个关节
        std::string finger1_name = mapping_joint_names.front();
        for (const auto& jn : mapping_joint_names) {
            if (jn.find("finger1") != std::string::npos) {
                finger1_name = jn;
                break;
            }
        }
        JointLimits limits;
        hardware_manager_->get_joint_limits(finger1_name, limits);
        const double min_pos = limits.has_position_limits ? limits.min_position : 0.0;
        const double max_pos = limits.has_position_limits ? limits.max_position : 0.025;
        target_pos = std::clamp(min_pos + ratio * (max_pos - min_pos), min_pos, max_pos);

        // PGC: 映射到 0~100，并保持既有下发语义。
        // position: PGC 0=张开,100=夹紧；而 ratio 中 0=夹紧,1=张开，因此取 (1-ratio)
        cmd_position = static_cast<uint8_t>(std::lround((1.0 - ratio) * 100.0));
        cmd_velocity = static_cast<uint8_t>(
            std::clamp(static_cast<int>(std::lround((static_cast<double>(velocity_raw) / 255.0) * 100.0)), 1, 100));
        cmd_effort = static_cast<uint8_t>(
            std::clamp(static_cast<int>(std::lround((static_cast<double>(effort_raw) / 255.0) * 100.0)), 20, 100));
    }

    hardware_driver->control_gripper(interface, static_cast<uint8_t>(gripper_type), cmd_position, cmd_velocity, cmd_effort);

    {
        std::vector<double> positions(mapping_joint_names.size(), 0.0);
        std::vector<double> velocities(mapping_joint_names.size(), 0.0);
        if (gripper_type == 0) {
            // OmniPicker 仅做软件状态占位，使用归一化值(0~1)避免引入米制语义
            std::fill(positions.begin(), positions.end(), ratio);
        } else {
            // PGC 仅更新 finger1/finger2 两个关节的物理位置(米)
            std::string finger1_name = mapping_joint_names.front();
            std::string finger2_name;
            for (const auto& jn : mapping_joint_names) {
                if (jn.find("finger1") != std::string::npos) {
                    finger1_name = jn;
                } else if (jn.find("finger2") != std::string::npos) {
                    finger2_name = jn;
                }
            }
            for (size_t i = 0; i < mapping_joint_names.size(); ++i) {
                if (mapping_joint_names[i] == finger1_name ||
                    (!finger2_name.empty() && mapping_joint_names[i] == finger2_name)) {
                    positions[i] = target_pos;
                }
            }
        }
        hardware_manager_->update_software_joint_state(mapping, positions, velocities);
    }

    if (gripper_type == 0) {
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] ✅ GripperControl via %s: raw=%u type=%d cmd(pos=%u vel=%u effort=%u)",
                    mapping.c_str(), interface.c_str(),
                    position_raw, gripper_type, cmd_position, cmd_velocity, cmd_effort);
    } else {
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] ✅ GripperControl via %s: target=%.5f raw=%u type=%d cmd(pos=%u vel=%u effort=%u)",
                    mapping.c_str(), interface.c_str(), target_pos,
                    position_raw, gripper_type, cmd_position, cmd_velocity, cmd_effort);
    }
    return true;
}

void BasicOpsIPCService::gripper_consumer_loop() {
    while (rclcpp::ok() && running_.load(std::memory_order_acquire)) {
        arm_controller::CommandIPC cmd;
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "GripperControl", 10)) {
            continue;
        }

        const std::string mapping = cmd.get_mapping();
        const auto params = cmd.get_parameters();
        bool success = false;
        if (params.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ GripperControl: empty parameters", mapping.c_str());
        } else {
            // 新格式: [gripper_type, position_raw(0~255), velocity_raw(0~255), effort_raw(0~255)]
            // 兼容旧格式: [position_m, velocity_percent, effort_percent]
            int gripper_type = -1;
            uint8_t position_raw = 0;
            uint8_t velocity_raw = 128;
            uint8_t effort_raw = 128;

            if (params.size() >= 4) {
                gripper_type = static_cast<int>(std::lround(params[0]));
                position_raw = static_cast<uint8_t>(std::clamp(static_cast<int>(std::lround(params[1])), 0, 255));
                velocity_raw = static_cast<uint8_t>(std::clamp(static_cast<int>(std::lround(params[2])), 0, 255));
                effort_raw = static_cast<uint8_t>(std::clamp(static_cast<int>(std::lround(params[3])), 0, 255));
            } else {
                // 旧格式回退：position 米制 -> raw，百分比 -> raw
                const double target_pos_legacy = params[0];
                const int velocity_percent = std::clamp(
                    (params.size() > 1) ? static_cast<int>(std::lround(params[1])) : 50, 0, 100);
                const int effort_percent = std::clamp(
                    (params.size() > 2) ? static_cast<int>(std::lround(params[2])) : 50, 0, 100);

                const auto& mapping_joint_names = hardware_manager_ ? hardware_manager_->get_joint_names(mapping)
                                                                     : std::vector<std::string>{};
                if (!mapping_joint_names.empty()) {
                    JointLimits limits;
                    hardware_manager_->get_joint_limits(mapping_joint_names.front(), limits);
                    const double min_pos = limits.has_position_limits ? limits.min_position : 0.0;
                    const double max_pos = limits.has_position_limits ? limits.max_position : 0.025;
                    double ratio = 0.0;
                    if (max_pos > min_pos + 1e-9) {
                        ratio = (target_pos_legacy - min_pos) / (max_pos - min_pos);
                    }
                    ratio = std::clamp(ratio, 0.0, 1.0);
                    position_raw = static_cast<uint8_t>(std::lround(ratio * 255.0));
                }
                velocity_raw = static_cast<uint8_t>(std::lround((velocity_percent / 100.0) * 255.0));
                effort_raw = static_cast<uint8_t>(std::lround((effort_percent / 100.0) * 255.0));
            }

            success = execute_gripper_position_command(mapping, gripper_type, position_raw, velocity_raw, effort_raw);
        }
        update_execution_state(mapping, success);
    }
}

void BasicOpsIPCService::motor_enable_consumer_loop() {
    while (rclcpp::ok() && running_.load(std::memory_order_acquire)) {
        arm_controller::CommandIPC cmd;
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "MotorEnable", 10)) {
            continue;
        }

        const std::string mapping = cmd.get_mapping();
        const auto params = cmd.get_parameters();
        const uint8_t mode = static_cast<uint8_t>(
            std::clamp((params.empty() ? 0 : static_cast<int>(std::lround(params[0]))), 0, 255));

        const bool ok = hardware_manager_ && hardware_manager_->enable_motors(mapping, mode);
        if (!ok) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MotorEnable failed (mode=%u)", mapping.c_str(), mode);
        } else {
            RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MotorEnable success (mode=%u)", mapping.c_str(), mode);
        }
        update_execution_state(mapping, ok);
    }
}

void BasicOpsIPCService::motor_disable_consumer_loop() {
    while (rclcpp::ok() && running_.load(std::memory_order_acquire)) {
        arm_controller::CommandIPC cmd;
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "MotorDisable", 10)) {
            continue;
        }

        const std::string mapping = cmd.get_mapping();
        const auto params = cmd.get_parameters();
        const uint8_t mode = static_cast<uint8_t>(
            std::clamp((params.empty() ? 0 : static_cast<int>(std::lround(params[0]))), 0, 255));

        const bool ok = hardware_manager_ && hardware_manager_->disable_motors(mapping, mode);
        if (!ok) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ MotorDisable failed (mode=%u)", mapping.c_str(), mode);
        } else {
            RCLCPP_INFO(node_->get_logger(), "[%s] ✅ MotorDisable success (mode=%u)", mapping.c_str(), mode);
        }
        update_execution_state(mapping, ok);
    }
}

}  // namespace arm_controller::basic_ops
