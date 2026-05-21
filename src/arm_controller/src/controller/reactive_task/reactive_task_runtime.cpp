#include "reactive_task_controller.hpp"

#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/ipc_context.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <thread>

ReactiveTaskController::~ReactiveTaskController() {
    planning_worker_running_ = false;
    planning_queue_cv_.notify_all();
    if (planning_worker_ && planning_worker_->joinable()) {
        planning_worker_->join();
    }

    consumer_running_ = false;
    arm_controller::CommandQueueIPC::getInstance().shutdown();
    if (queue_consumer_ && queue_consumer_->joinable()) {
        queue_consumer_->join();
    }
}

bool ReactiveTaskController::send_joint_velocities(
    const std::string& mapping,
    const std::vector<double>& joint_velocities) const {
    if (!hardware_manager_) {
        return false;
    }
    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        return false;
    }

    try {
        const std::string interface = hardware_manager_->get_interface(mapping);
        const auto motor_ids = hardware_manager_->get_motors_id(mapping);
        const auto joint_names = hardware_manager_->get_joint_names(mapping);

        if (motor_ids.empty() || joint_names.empty()) {
            return false;
        }

        std::array<double, 6> batch_positions = {};
        std::array<double, 6> batch_velocities = {};
        std::array<double, 6> batch_efforts = {};
        std::array<double, 6> batch_kps = {};
        std::array<double, 6> batch_kds = {};
        batch_kps.fill(runtime_cfg_.mit_kp);
        batch_kds.fill(runtime_cfg_.mit_kd);

        const auto q_current = hardware_manager_->get_current_joint_positions_lockfree(mapping);
        auto gravity_torques = hardware_manager_->compute_gravity_torques(mapping, q_current);

        const std::size_t max_motors =
            std::min<std::size_t>(6, static_cast<std::size_t>(runtime_cfg_.mit_max_motors));
        const std::size_t command_count = std::min(motor_ids.size(), max_motors);
        for (size_t i = 0; i < command_count; ++i) {
            const double vel_rad = (i < joint_velocities.size()) ? joint_velocities[i] : 0.0;
            const double vel_deg = vel_rad * 180.0 / M_PI;
            // ReactiveTask is a velocity-space controller. Align the MIT packet
            // semantics with the dedicated joint/cartesian velocity controllers:
            // zero position target, configured kp/kd, velocity command plus gravity
            // compensation. The critical part is not anchoring the packet to the
            // live joint position while we intend to run in pure velocity mode.
            batch_velocities[i] = vel_deg;
            batch_positions[i] = 0.0;
            batch_efforts[i] = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;
        }

        const double qdot_norm = [&]() {
            double sum = 0.0;
            for (const double v : joint_velocities) {
                sum += v * v;
            }
            return std::sqrt(sum);
        }();
        if (qdot_norm > 0.05) {
            std::ostringstream oss;
            oss << "neo MIT cmd count=" << command_count
                << " kp=" << runtime_cfg_.mit_kp
                << " kd=" << runtime_cfg_.mit_kd
                << " qdot_norm=" << qdot_norm
                << " vel_deg=[";
            for (std::size_t i = 0; i < command_count; ++i) {
                if (i > 0) {
                    oss << ", ";
                }
                oss << batch_velocities[i];
            }
            oss << "] tau=[";
            for (std::size_t i = 0; i < command_count; ++i) {
                if (i > 0) {
                    oss << ", ";
                }
                oss << batch_efforts[i];
            }
            oss << "]";
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                500,
                "[%s] %s",
                mapping.c_str(),
                oss.str().c_str());
        }

        return hardware_driver->send_realtime_mit_command(
            interface,
            batch_positions,
            batch_velocities,
            batch_efforts,
            batch_kps,
            batch_kds);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] neo send velocity exception: %s",
            mapping.c_str(),
            e.what());
        return false;
    }
}

bool ReactiveTaskController::execute(
    const std::string& mapping,
    const std::vector<double>& parameters) {
    if (parameters.size() != 7) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] reactive_task expected 7 params [x y z qx qy qz qw], got %zu",
            mapping.c_str(),
            parameters.size());
        return false;
    }

    auto pose = std::make_shared<geometry_msgs::msg::Pose>();
    pose->position.x = parameters[0];
    pose->position.y = parameters[1];
    pose->position.z = parameters[2];
    pose->orientation.x = parameters[3];
    pose->orientation.y = parameters[4];
    pose->orientation.z = parameters[5];
    pose->orientation.w = parameters[6];

    last_execution_success_[mapping] = false;
    plan_and_execute(mapping, pose);
    return last_execution_success_[mapping];
}

void ReactiveTaskController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;

    while (consumer_running_) {
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(
                cmd, "ReactiveTask", 10)) {
            continue;
        }

        std::string mapping = cmd.get_mapping();
        auto params = cmd.get_parameters();
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        {
            std::lock_guard<std::mutex> execution_lock(
                arm_controller::CommandQueueIPC::getMappingExecutionMutex(mapping));

            try {
                if (state_mgr) {
                    state_mgr->transitionToMode("ReactiveTask");
                    if (state_mgr->isInHookState()) {
                        std::string target_mode = state_mgr->getTargetMode();
                        if (target_mode.empty()) {
                            target_mode = "ReactiveTask";
                        }
                        if (hook_request_callback_) {
                            hook_request_callback_(mapping, target_mode);
                        }
                        arm_controller::CommandQueueIPC::getInstance().push(cmd);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
                        continue;
                    }
                    state_mgr->setExecutionState(
                        arm_controller::ipc::ExecutionState::EXECUTING);
                }

                start(mapping);
                const bool ok = execute(mapping, params);
                if (state_mgr) {
                    state_mgr->setExecutionState(
                        ok ? arm_controller::ipc::ExecutionState::SUCCESS
                           : arm_controller::ipc::ExecutionState::FAILED);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);

                    arm_controller::ipc::ExecutorControllerState executor_state;
                    std::strncpy(
                        executor_state.current_mode,
                        "ReactiveTask",
                        sizeof(executor_state.current_mode) - 1);
                    executor_state.current_mode[sizeof(executor_state.current_mode) - 1] = '\0';
                    executor_state.execution_state =
                        static_cast<int>(arm_controller::ipc::ExecutionState::IDLE);
                    state_mgr->updateFromExecutor(executor_state);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "[%s] reactive_task command exception: %s",
                    mapping.c_str(),
                    e.what());
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);
                }
            }
        }

        arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
    }
}
