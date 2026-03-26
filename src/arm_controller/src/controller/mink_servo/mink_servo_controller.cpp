#include "mink_servo_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace {
constexpr double kDt = 0.01;                 // 100Hz
constexpr double kPosGain = 1.5;             // pose -> linear velocity
constexpr double kRotGain = 1.0;             // orientation error -> angular velocity
constexpr double kMaxLinear = 0.20;          // m/s
constexpr double kMaxAngular = 0.80;         // rad/s
constexpr double kDampingBase = 1e-4;
constexpr double kDampingAdaptiveGain = 1e-3;
constexpr double kDampingAdaptiveEps = 1e-6;
constexpr double kDampingMax = 5e-2;
constexpr double kNullspaceGain = 0.25;      // joint centering weight
constexpr double kManipThreshold = 0.03;      // below this, singular lock candidate
constexpr double kSecondaryDisableThreshold = 0.06;  // disable null-space term near singularity
constexpr double kManipEps = 1e-9;           // numeric epsilon for determinant stability
constexpr double kSettlePosTolEnter = 0.003; // 3mm
constexpr double kSettleRotTolEnter = 0.03;  // ~1.7deg (rad)
constexpr double kSettlePosTolExit = 0.006;  // 6mm hysteresis exit
constexpr double kSettleRotTolExit = 0.06;   // ~3.4deg hysteresis exit
constexpr int kSettleCycles = 10;            // consecutive cycles to latch
constexpr double kSpeedBoostMax = 2.0;       // max alpha boost for "as-fast-as-possible" tracking
constexpr double kQdAccelLimit = 8.0;        // rad/s^2 slew-rate limit
constexpr double kKp = 0.05;                  // MIT position gain
constexpr double kKd = 0.005;                 // MIT velocity gain
constexpr bool kDryRunPrintOnly = false;       // true: 仅打印即将下发的MIT值令，不实际发送

double clamp_abs(double v, double max_abs) {
    if (v > max_abs) return max_abs;
    if (v < -max_abs) return -max_abs;
    return v;
}
}  // namespace

MinkServoController::MinkServoController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<geometry_msgs::msg::PoseStamped>("MinkServo", node) {
    hardware_manager_ = HardwareManager::getInstance();
    consumer_running_ = true;
    queue_consumer_ = std::make_unique<std::thread>(
        &MinkServoController::command_queue_consumer_thread, this);
}

MinkServoController::~MinkServoController() {
    std::vector<std::string> mappings_to_stop;
    {
        for (const auto& [mapping, _] : rt_running_per_mapping_) {
            mappings_to_stop.push_back(mapping);
        }
    }
    for (const auto& mapping : mappings_to_stop) {
        stop(mapping);
    }

    consumer_running_ = false;
    arm_controller::CommandQueueIPC::getInstance().shutdown();
    if (queue_consumer_ && queue_consumer_->joinable()) {
        queue_consumer_->join();
    }
}

void MinkServoController::start(const std::string& mapping) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

    if (rt_threads_.count(mapping) > 0 ||
        rt_running_per_mapping_.count(mapping) > 0 ||
        computation_threads_.count(mapping) > 0 ||
        computation_running_per_mapping_.count(mapping) > 0) {
        return;
    }

    auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
    if (state_mgr) {
        state_mgr->initializeCurrentMode("MinkServo");
    }

    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] MinkServo: not found in hardware configuration."
        );
    }

    VelocityControllerImpl::start(mapping);
    initialize_moveit_adapter(mapping);
    initialize_pinocchio_context(mapping);

    std::string base_frame = hardware_manager_->get_frame_id(mapping);
    if (base_frame.empty()) {
        base_frame = "base_link";
    }
    mapping_base_frames_[mapping] = base_frame;

    RtState state;

    state.last_update = steady_clock_.now();
    state.target = geometry_msgs::msg::PoseStamped();
    state.target.header.frame_id = base_frame;
    
    rt_states_[mapping] = state;

    auto rt_state_mutex = std::make_unique<std::mutex>();
    rt_states_mutexes_[mapping] = std::move(rt_state_mutex);

    {
        std::lock_guard<std::mutex> lock(rt_buffers_mutex_);
        rt_buffers_[mapping] = std::make_unique<SPSCQueue<PoseCommand, 128>>();
    }

    bool need_subscription = false;
    {
        std::lock_guard<std::mutex> sub_lock(subscriptions_mutex_);
        need_subscription = (subscriptions_.find(mapping) == subscriptions_.end());
    }
    if (need_subscription) {
        init_subscriptions(mapping);
    }

    auto rt_running = std::make_shared<std::atomic<bool>>(true);
    rt_running_per_mapping_[mapping] = rt_running;
    rt_threads_[mapping] = std::thread([this, mapping, rt_running]() {
        auto next = std::chrono::steady_clock::now();
        while (rt_running->load(std::memory_order_acquire)) {
            next += std::chrono::milliseconds(5);
            control_loop_rt(mapping);
            auto now = std::chrono::steady_clock::now();
            if (now < next) {
                std::this_thread::sleep_for(next - now);
            }
        }
    });

    auto computation_running = std::make_shared<std::atomic<bool>>(true);
    computation_running_per_mapping_[mapping] = computation_running;
    {
        auto result_with_mtx = std::make_unique<ComputationResultWithMutex>();
        result_with_mtx->result.valid = false;
        const int dof = static_cast<int>(hardware_manager_->get_joint_names(mapping).size());
        result_with_mtx->result.qd = Eigen::VectorXd::Zero(dof);
        result_with_mtx->result.qd_last = Eigen::VectorXd::Zero(dof);
        result_with_mtx->result.timestamp = steady_clock_.now();
        computation_results_[mapping] = std::move(result_with_mtx);
    }

    computation_threads_[mapping] = std::thread(
        &MinkServoController::mink_computation_thread,
        this, 
        mapping);
}

bool MinkServoController::stop(const std::string& mapping) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    VelocityControllerImpl::stop(mapping);

    auto it_running = rt_running_per_mapping_.find(mapping);
    if (it_running != rt_running_per_mapping_.end()) {
        it_running->second->store(false, std::memory_order_release);
        rt_running_per_mapping_.erase(it_running);
    }

    auto it = rt_threads_.find(mapping);
    if (it != rt_threads_.end()) {
        if (it->second.joinable()) {
            it->second.join();
        }
        rt_threads_.erase(it);
    }

    auto it_comp_running = computation_running_per_mapping_.find(mapping);
    if (it_comp_running != computation_running_per_mapping_.end()) {
        it_comp_running->second->store(false, std::memory_order_release);
        computation_running_per_mapping_.erase(it_comp_running);
    }

    auto it_comp = computation_threads_.find(mapping);
    if (it_comp != computation_threads_.end()) {
        if (it_comp->second.joinable()) {
            it_comp->second.join();
        }
        computation_threads_.erase(it_comp);
    }
    computation_results_.erase(mapping);

    cleanup_subscriptions(mapping);
    moveit_adapters_.erase(mapping);
    pinocchio_contexts_.erase(mapping);
    mapping_base_frames_.erase(mapping);
    {
        std::lock_guard<std::mutex> lock(rt_buffers_mutex_);
        rt_buffers_.erase(mapping);
    }
    {
        auto it_mtx = rt_states_mutexes_.find(mapping);
        if (it_mtx != rt_states_mutexes_.end()) {
            std::lock_guard<std::mutex> lock(*it_mtx->second);
            rt_states_.erase(mapping);
            rt_states_mutexes_.erase(it_mtx);
        } else {
            rt_states_.erase(mapping);
        }
    }
    return true;
}

void MinkServoController::initialize_moveit_adapter(const std::string& mapping) {
    try {
        if (hardware_manager_->get_motors_id(mapping).empty()) {
            return;
        }
        if (moveit_adapters_.find(mapping) != moveit_adapters_.end()) {
            return;
        }
        std::string planning_group = hardware_manager_->get_planning_group(mapping);
        if (planning_group.empty()) {
            return;
        }
        auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
            node_, planning_group);

        if (moveit_adapter) {
            moveit_adapters_[mapping] = moveit_adapter;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] MinkServo init adapter failed: %s",
                     mapping.c_str(), e.what());
    }
}

void MinkServoController::initialize_pinocchio_context(const std::string& mapping) {
    try {
        PinocchioContext ctx;
        const std::string robot_type = hardware_manager_->get_robot_type(mapping);
        if (robot_type.empty()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] MinkServo: empty robot_type, pinocchio disabled",
                        mapping.c_str());
            pinocchio_contexts_[mapping] = std::move(ctx);
            return;
        }

        const std::string robot_desc_path = ament_index_cpp::get_package_share_directory("robot_description");
        const std::string urdf_path = robot_desc_path + "/urdf/" + robot_type + ".urdf";
        pinocchio::urdf::buildModel(urdf_path, ctx.model);
        ctx.data = std::make_unique<pinocchio::Data>(ctx.model);

        const auto& joint_names = hardware_manager_->get_joint_names(mapping);
        if (joint_names.empty()) {
            pinocchio_contexts_[mapping] = std::move(ctx);
            return;
        }

        for (const auto& jn : joint_names) {
            if (!ctx.model.existJointName(jn)) {
                RCLCPP_WARN(node_->get_logger(), "[%s] MinkServo: joint '%s' not in pinocchio model",
                            mapping.c_str(), jn.c_str());
                continue;
            }
            const auto jid = ctx.model.getJointId(jn);
            ctx.q_indices.push_back(static_cast<int>(ctx.model.joints[jid].idx_q()));
            ctx.v_indices.push_back(static_cast<int>(ctx.model.joints[jid].idx_v()));
        }

        if (ctx.q_indices.size() != joint_names.size()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] MinkServo: pinocchio joint mapping incomplete (%zu/%zu)",
                        mapping.c_str(), ctx.q_indices.size(), joint_names.size());
        }

        pinocchio::JointIndex last_joint_id = 0;
        if (ctx.model.existJointName(joint_names.back())) {
            last_joint_id = ctx.model.getJointId(joint_names.back());
        }

        pinocchio::FrameIndex ee = 0;
        for (pinocchio::FrameIndex fid = 0; fid < ctx.model.frames.size(); ++fid) {
            if (ctx.model.frames[fid].parentJoint == last_joint_id) {
                ee = fid;
            }
        }
        ctx.ee_frame = ee;
        ctx.ready = (!ctx.q_indices.empty());
        pinocchio_contexts_[mapping] = std::move(ctx);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] MinkServo pinocchio init failed: %s",
                     mapping.c_str(), e.what());
        pinocchio_contexts_[mapping] = PinocchioContext{};
    }
}

void MinkServoController::velocity_callback(
    const std::string& mapping,
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!msg) return;

    std::lock_guard<std::mutex> lock(rt_buffers_mutex_);
    auto it = rt_buffers_.find(mapping);
    if (it == rt_buffers_.end()) return;

    PoseCommand c;
    c.pose = *msg;
    if (c.pose.header.frame_id.empty()) {
        c.pose.header.frame_id = "world";
    }
    c.stamp = steady_clock_.now();
    it->second->push(c);
}

void MinkServoController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;
    auto last_command_time = steady_clock_.now();
    constexpr std::chrono::milliseconds BATCH_TIMEOUT{100};

    while (consumer_running_) {
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "MinkServo", 10)) {
            auto now = steady_clock_.now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time) >= BATCH_TIMEOUT) {
                arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
                last_command_time = now;
            }
            continue;
        }

        std::string mapping = cmd.get_mapping();
        auto params = cmd.get_parameters();
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        if (state_mgr) {
            std::string current_mode = state_mgr->getCurrentMode();
            if (!current_mode.empty() && current_mode != "MinkServo") {
                if (hook_request_callback_) {
                    hook_request_callback_(mapping, "MinkServo");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            bool transition_ok = state_mgr->transitionToMode("MinkServo");
            if (!transition_ok) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  Mode transition failed", mapping.c_str());
                continue;
            }
            if (state_mgr->isInHookState()) {
                std::string target_mode = state_mgr->getTargetMode();
                if (hook_request_callback_) {
                    hook_request_callback_(mapping, target_mode.empty() ? "MinkServo" : target_mode);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }

        start(mapping);

        {
            std::lock_guard<std::mutex> lock(rt_buffers_mutex_);

            auto it = rt_buffers_.find(mapping);
            if (it != rt_buffers_.end()) {
                PoseCommand c;

                // 构造 PoseStamped 命令
                c.pose.header.frame_id = "world";
                c.pose.header.stamp = node_->now();
                if (params.size() >= 7) {
                    c.pose.pose.position.x = params[0];
                    c.pose.pose.position.y = params[1];
                    c.pose.pose.position.z = params[2];
                    c.pose.pose.orientation.x = params[3];
                    c.pose.pose.orientation.y = params[4];
                    c.pose.pose.orientation.z = params[5];
                    c.pose.pose.orientation.w = params[6];
                }
                c.stamp = steady_clock_.now();

                it->second->push(c);
            }
        }
        
        last_command_time = steady_clock_.now();
    }
}

void MinkServoController::control_loop_rt(const std::string& mapping) {
    if (!is_active(mapping)) {
        return;
    }

    auto it_running = rt_running_per_mapping_.find(mapping);
    if (it_running == rt_running_per_mapping_.end() ||
        !it_running->second->load(std::memory_order_acquire)) {
        return;
    }

    auto it_state = rt_states_.find(mapping);
    if (it_state == rt_states_.end()) {
        return;
    }

    auto& state = it_state->second;

    std::vector<PoseCommand> cmds;
    {
        std::lock_guard<std::mutex> lock(rt_buffers_mutex_);
        auto it_buf = rt_buffers_.find(mapping);
        if (it_buf == rt_buffers_.end()) {
            return;
        }
        PoseCommand cmd;
        while (it_buf->second->pop(cmd)) {
            cmds.push_back(cmd);
        }
    }

    if (!cmds.empty()) {
        auto it_mtx = rt_states_mutexes_.find(mapping);
        if (it_mtx != rt_states_mutexes_.end() && it_mtx->second) {
            std::lock_guard<std::mutex> lock2(*it_mtx->second);
            const auto& cmd = cmds.back();
            state.target = cmd.pose;
            state.last_update = cmd.stamp;
            state.first_command_received = true;
        }
    }

    const auto q_current = hardware_manager_->get_current_joint_positions_lockfree(mapping);
    if (q_current.empty()) {
        return;
    }

    auto it_result = computation_results_.find(mapping);
    if (it_result == computation_results_.end() || !it_result->second) {
        return;
    }

    std::vector<double> qd_cmd(q_current.size(), 0.0);
    {
        std::lock_guard<std::mutex> lock(it_result->second->mtx);
        const auto& result = it_result->second->result;
        const Eigen::VectorXd& qd_to_send = result.valid ? result.qd : result.qd_last;
        if (qd_to_send.size() == static_cast<int>(q_current.size())) {
            std::copy(qd_to_send.data(), qd_to_send.data() + qd_to_send.size(), qd_cmd.begin());
        }
    }

    send_joint_status_command(mapping, q_current, qd_cmd);
}

void MinkServoController::mink_computation_thread(const std::string& mapping) {
    auto it_running = computation_running_per_mapping_.find(mapping);
    if (it_running == computation_running_per_mapping_.end()) {
        return;
    }
    auto computation_running = it_running->second;

    auto mark_invalid = [this, &mapping]() {
        auto it_result = computation_results_.find(mapping);
        if (it_result != computation_results_.end() && it_result->second) {
            std::lock_guard<std::mutex> lock(it_result->second->mtx);
            it_result->second->result.valid = false;
        }
    };

    auto sleep_to_next_cycle = [](const auto& cycle_start) {
        auto elapsed = std::chrono::steady_clock::now() - cycle_start;
        auto remaining = std::chrono::milliseconds(10) - elapsed;
        if (remaining.count() > 0) {
            std::this_thread::sleep_for(remaining);
        }
    };

    std::chrono::steady_clock::time_point last_target_stamp{};
    int settle_counter = 0;
    bool settle_latched = false;
    bool singular_latched = false;
    Eigen::VectorXd qd_prev;
    bool qd_prev_initialized = false;

    while (computation_running->load(std::memory_order_acquire)) {
        auto cycle_start = std::chrono::steady_clock::now();
        if (!is_active(mapping)) {
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        auto it_ctx = pinocchio_contexts_.find(mapping);
        if (it_ctx == pinocchio_contexts_.end() || !it_ctx->second.ready || !it_ctx->second.data) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }
        auto& ctx = it_ctx->second;

        auto it_state = rt_states_.find(mapping);
        if (it_state == rt_states_.end()) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        RtState state_copy;
        {
            auto it_mtx = rt_states_mutexes_.find(mapping);
            if (it_mtx != rt_states_mutexes_.end() && it_mtx->second) {
                std::lock_guard<std::mutex> lock(*it_mtx->second);
                state_copy = it_state->second;
            } else {
                state_copy = it_state->second;
            }
        }

        if (!state_copy.first_command_received) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        // New target command received -> clear settle latch.
        if (state_copy.last_update != last_target_stamp) {
            last_target_stamp = state_copy.last_update;
            settle_counter = 0;
            settle_latched = false;
            singular_latched = false;
        }

        const auto q_current_local = hardware_manager_->get_current_joint_positions_lockfree(mapping);
        if (ctx.q_indices.size() != q_current_local.size() || ctx.v_indices.size() != q_current_local.size()) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }
        const auto joint_names = hardware_manager_->get_joint_names(mapping);
        if (q_current_local.empty() || joint_names.empty()) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(ctx.model.nq);
        for (size_t i = 0; i < q_current_local.size(); ++i) {
            q_full(ctx.q_indices[i]) = q_current_local[i];
        }

        pinocchio::forwardKinematics(ctx.model, *ctx.data, q_full);
        pinocchio::updateFramePlacements(ctx.model, *ctx.data);

        const pinocchio::SE3& ee_pose = ctx.data->oMf[ctx.ee_frame];
        Eigen::Vector3d p_cur = ee_pose.translation();
        Eigen::Quaterniond q_cur(ee_pose.rotation());

        const auto& target_pose = state_copy.target.pose;
        // Step 1) Pose error -> task-space velocity command.
        // Linear error is directly position delta; angular error uses quaternion
        // difference converted to angle-axis (small-angle compatible representation).
        Eigen::Vector3d e_pos(
            target_pose.position.x - p_cur.x(),
            target_pose.position.y - p_cur.y(),
            target_pose.position.z - p_cur.z());

        Eigen::Quaterniond q_tgt(target_pose.orientation.w, target_pose.orientation.x,
                                 target_pose.orientation.y, target_pose.orientation.z);
        if (q_cur.norm() < 1e-8 || q_tgt.norm() < 1e-8) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }
        q_cur.normalize();
        q_tgt.normalize();

        Eigen::Quaterniond q_err = q_tgt * q_cur.conjugate();
        if (q_err.w() < 0.0) {
            q_err.coeffs() *= -1.0;
        }
        Eigen::AngleAxisd aa(q_err);
        Eigen::Vector3d e_rot = aa.axis() * aa.angle();

        // Settled latch with hysteresis to avoid boundary oscillation.
        const bool in_settle_band_enter =
            (e_pos.norm() < kSettlePosTolEnter) && (e_rot.norm() < kSettleRotTolEnter);
        const bool in_settle_band_exit =
            (e_pos.norm() < kSettlePosTolExit) && (e_rot.norm() < kSettleRotTolExit);
        if (!settle_latched && in_settle_band_enter) {
            if (settle_counter < kSettleCycles) {
                settle_counter++;
            }
            if (settle_counter >= kSettleCycles) {
                settle_latched = true;
            }
        } else if (!settle_latched) {
            settle_counter = 0;
        } else if (!in_settle_band_exit) {
            settle_counter = 0;
            settle_latched = false;
        }

        if (settle_latched) {
            auto it_result = computation_results_.find(mapping);
            if (it_result != computation_results_.end() && it_result->second) {
                std::lock_guard<std::mutex> lock(it_result->second->mtx);
                it_result->second->result.qd = Eigen::VectorXd::Zero(q_current_local.size());
                it_result->second->result.qd_last = it_result->second->result.qd;
                it_result->second->result.valid = true;
                it_result->second->result.timestamp = steady_clock_.now();
            }
            qd_prev = Eigen::VectorXd::Zero(q_current_local.size());
            qd_prev_initialized = true;
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        Eigen::Vector3d v_linear = e_pos * kPosGain;
        Eigen::Vector3d v_angular = e_rot * kRotGain;
        // Clamp Cartesian command for safety and to keep IK well-conditioned.
        for (int i = 0; i < 3; ++i) {
            v_linear(i) = clamp_abs(v_linear(i), kMaxLinear);
            v_angular(i) = clamp_abs(v_angular(i), kMaxAngular);
        }

        Eigen::VectorXd v_task(6);
        v_task << v_linear(0), v_linear(1), v_linear(2), v_angular(0), v_angular(1), v_angular(2);
        if (v_task.norm() < 1e-6) {
            auto it_result = computation_results_.find(mapping);
            if (it_result != computation_results_.end() && it_result->second) {
                std::lock_guard<std::mutex> lock(it_result->second->mtx);
                it_result->second->result.qd = Eigen::VectorXd::Zero(q_current_local.size());
                it_result->second->result.qd_last = it_result->second->result.qd;
                it_result->second->result.valid = true;
                it_result->second->result.timestamp = steady_clock_.now();
            }
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        const int dof = static_cast<int>(q_current_local.size());
        Eigen::VectorXd q_current_eig = Eigen::Map<const Eigen::VectorXd>(q_current_local.data(), dof);

        Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, ctx.model.nv);
        pinocchio::computeFrameJacobian(
            ctx.model, *ctx.data, q_full, ctx.ee_frame,
            pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J6);

        // Step 2) Build task Jacobian for controlled joints only.
        // Pinocchio Jacobian is for full model nv; we slice columns by mapping.
        Eigen::MatrixXd J_task(6, dof);
        for (int i = 0; i < dof; ++i) {
            J_task.col(i) = J6.col(ctx.v_indices[i]);
        }
        if (J_task.hasNaN()) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        Eigen::VectorXd qd_max(dof), q_min_pos(dof), q_max_pos(dof);
        for (int i = 0; i < dof; ++i) {
            JointLimits limits;
            hardware_manager_->get_joint_limits(joint_names[i], limits);
            qd_max(i) = limits.has_velocity_limits ? limits.max_velocity : 1.0;
            q_min_pos(i) = limits.min_position;
            q_max_pos(i) = limits.max_position;
        }

        // Step 3) Primary IK objective (damped least-squares) with adaptive damping.
        // As sigma_min decreases near singularity, lambda increases.
        Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd JJt = J_task * J_task.transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_task, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const auto& singular_values = svd.singularValues();
        const double sigma_min = singular_values.size() > 0 ? singular_values(singular_values.size() - 1) : 0.0;
        const double lambda = std::min(
            kDampingBase + kDampingAdaptiveGain / (sigma_min * sigma_min + kDampingAdaptiveEps),
            kDampingMax);
        Eigen::VectorXd qd_primary =
            J_task.transpose() * (JJt + lambda * I6).ldlt().solve(v_task);

        // Step 5) Singular lock:
        // w(q)=sqrt(det(JJ^T+epsI)) below threshold means near singular region.
        // Instead of pushing out, latch zero velocity until a new target arrives.
        auto manipulability = [&](const Eigen::VectorXd& q_full_eval) -> double {
            pinocchio::computeFrameJacobian(
                ctx.model, *ctx.data, q_full_eval, ctx.ee_frame,
                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J6);
            Eigen::MatrixXd J_eval(6, dof);
            for (int c = 0; c < dof; ++c) {
                J_eval.col(c) = J6.col(ctx.v_indices[c]);
            }
            Eigen::MatrixXd M = J_eval * J_eval.transpose()
                              + kManipEps * Eigen::MatrixXd::Identity(6, 6);
            const double det_val = std::max(M.determinant(), 0.0);
            return std::sqrt(det_val);
        };

        const double w_now = manipulability(q_full);
        // Only lock when near target and near singular; do not block large moves.
        if (w_now < kManipThreshold && in_settle_band_exit) {
            singular_latched = true;
        }
        if (singular_latched) {
            auto it_result = computation_results_.find(mapping);
            if (it_result != computation_results_.end() && it_result->second) {
                std::lock_guard<std::mutex> lock(it_result->second->mtx);
                it_result->second->result.qd = Eigen::VectorXd::Zero(q_current_local.size());
                it_result->second->result.qd_last = it_result->second->result.qd;
                it_result->second->result.valid = true;
                it_result->second->result.timestamp = steady_clock_.now();
            }
            qd_prev = Eigen::VectorXd::Zero(q_current_local.size());
            qd_prev_initialized = true;
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        // Step 6) Compose final joint velocity.
        // For non-redundant arms (dof <= 6), null-space dimension is typically zero,
        // so we keep only primary task for cleaner behavior.
        Eigen::VectorXd qd = qd_primary;
        if (dof > 6) {
            // Secondary objective A: joint-centering gradient.
            Eigen::VectorXd q_mid = 0.5 * (q_min_pos + q_max_pos);
            Eigen::VectorXd q_center_grad = (q_mid - q_current_eig);

            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof, dof);
            Eigen::MatrixXd J_pinv = J_task.transpose() * (JJt + lambda * I6).inverse();
            Eigen::MatrixXd N = I - J_pinv * J_task;

            // Null-space term is disabled in poor manipulability region.
            Eigen::VectorXd secondary = Eigen::VectorXd::Zero(dof);
            if (w_now >= kSecondaryDisableThreshold) {
                secondary = kNullspaceGain * q_center_grad;
            }
            qd = qd_primary + N * secondary;
        }

        // Step 7) "As-fast-as-possible" scaling under joint velocity limits.
        // If the solution has slack, scale qd up uniformly by alpha.
        double alpha_max = kSpeedBoostMax;
        for (int i = 0; i < dof; ++i) {
            const double abs_qd = std::abs(qd(i));
            if (abs_qd > 1e-9) {
                alpha_max = std::min(alpha_max, qd_max(i) / abs_qd);
            }
        }
        if (alpha_max > 1.0) {
            qd *= alpha_max;
        }

        // Step 8) Slew-rate limit (acceleration constraint) to suppress oscillation.
        if (!qd_prev_initialized || qd_prev.size() != dof) {
            qd_prev = Eigen::VectorXd::Zero(dof);
            qd_prev_initialized = true;
        }
        const double dq_max = kQdAccelLimit * kDt;
        for (int i = 0; i < dof; ++i) {
            const double low = qd_prev(i) - dq_max;
            const double high = qd_prev(i) + dq_max;
            qd(i) = std::min(std::max(qd(i), low), high);
        }

        for (int i = 0; i < dof; ++i) {
            qd(i) = clamp_abs(qd(i), qd_max(i));
        }
        qd_prev = qd;

        {
            auto it_result = computation_results_.find(mapping);
            if (it_result != computation_results_.end() && it_result->second) {
                std::lock_guard<std::mutex> lock(it_result->second->mtx);
                it_result->second->result.qd = qd;
                it_result->second->result.qd_last = qd;
                it_result->second->result.valid = true;
                it_result->second->result.timestamp = steady_clock_.now();
            }
        }

        sleep_to_next_cycle(cycle_start);
    }
}

bool MinkServoController::send_joint_status_command(const std::string& mapping,
                                                 const std::vector<double>& q_current,
                                                 const std::vector<double>& qd_cmd) {
    if (!hardware_manager_) return false;
    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) return false;

    try {
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const auto& motor_ids = hardware_manager_->get_motors_id(mapping);

        std::array<double, 6> batch_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, 6> batch_velocities = {};
        std::array<double, 6> batch_efforts = {};
        std::array<double, 6> batch_kps = {kKp, kKp, kKp, kKp, kKp, kKp};
        std::array<double, 6> batch_kds = {kKd, kKd, kKd, kKd, kKd, kKd};

        std::vector<double> q_for_gravity = q_current;
        if (q_for_gravity.empty()) {
            q_for_gravity = hardware_manager_->get_current_joint_positions_lockfree(mapping);
        }
        auto gravity_torques = hardware_manager_->compute_gravity_torques(mapping, q_for_gravity);

        for (size_t i = 0; i < motor_ids.size() && i < 6; ++i) {
            const double q = (i < q_for_gravity.size()) ? q_for_gravity[i] : 0.0;
            const double qd = (i < qd_cmd.size()) ? qd_cmd[i] : 0.0;
            // MIT mixed command in each cycle:
            //   position_ref = q + qd*dt  (one-step forward Euler)
            //   velocity_ref = qd         (feed-forward)
            //   effort_ref   = gravity torque compensation
            const double q_ref = q + qd * kDt;
            batch_positions[i] = q_ref * 180.0 / M_PI;
            batch_velocities[i] = qd * 180.0 / M_PI;
            batch_efforts[i] = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;
        }

        if (kDryRunPrintOnly) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4);
            oss << "MIT dry-run " << mapping << " pos[deg]=[";
            for (size_t i = 0; i < motor_ids.size() && i < 6; ++i) {
                if (i > 0) oss << ", ";
                oss << batch_positions[i];
            }
            oss << "] vel[deg/s]=[";
            for (size_t i = 0; i < motor_ids.size() && i < 6; ++i) {
                if (i > 0) oss << ", ";
                oss << batch_velocities[i];
            }
            oss << "] tau[Nm]=[";
            for (size_t i = 0; i < motor_ids.size() && i < 6; ++i) {
                if (i > 0) oss << ", ";
                oss << batch_efforts[i];
            }
            oss << "]";

            auto clock = node_->get_clock();
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock, 200, "[%s] %s",
                                 mapping.c_str(), oss.str().c_str());
            return true;
        }

        return hardware_driver->send_realtime_mit_command(
            interface, batch_positions, batch_velocities, batch_efforts, batch_kps, batch_kds);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] MinkServo MIT send failed: %s",
                     mapping.c_str(), e.what());
        return false;
    }
}

bool MinkServoController::send_velocity(const std::string& mapping, const std::vector<double>& target_pose) {
    if (target_pose.size() < 7) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] MinkServo target_pose must have 7 elements", mapping.c_str());
        return false;
    }

    PoseCommand c;
    c.pose.header.frame_id = "world";
    c.pose.header.stamp = node_->now();
    c.pose.pose.position.x = target_pose[0];
    c.pose.pose.position.y = target_pose[1];
    c.pose.pose.position.z = target_pose[2];
    c.pose.pose.orientation.x = target_pose[3];
    c.pose.pose.orientation.y = target_pose[4];
    c.pose.pose.orientation.z = target_pose[5];
    c.pose.pose.orientation.w = target_pose[6];
    c.stamp = steady_clock_.now();

    std::lock_guard<std::mutex> lock(rt_buffers_mutex_);
    auto it = rt_buffers_.find(mapping);
    if (it != rt_buffers_.end()) {
        it->second->push(c);
    }
    return true;
}
