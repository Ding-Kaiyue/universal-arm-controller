/**
 * @brief A local planner for reactive task execution using TrajOpt.
 * 
 * Algorithm overview:
 * 1. 没有启用 Cartesian cost/constraint, 当前只有关节空间局部优化
 * 2. 中间点用 JointWaypoint + tolerance 做软走廊
 * 3. 首尾点用 StateWaypoint, 并且后处理严格检查首尾不能移动
 * 4. 障碍物来自点云 voxel, 被近似成 sphere obstacle
 * 5. TrajOpt 输出不直接发硬件，而是转成 pose/twist 喂给 NeoPipeline, 由 NeoPipeline 做最终的轨迹生成和跟踪
 * 
 * @author Ding Kaiyue
 * @date 2026-05-20
 */
#include "controller/reactive_task/reactive_task_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <sstream>
#include <utility>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/utils.h>
#include <trajopt_common/collision_types.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/plugin_info.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/commands/add_contact_managers_plugin_info_command.h>
#include <tesseract_environment/commands/add_kinematics_information_command.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/change_joint_origin_command.h>
#include <tesseract_environment/commands/change_link_collision_enabled_command.h>
#include <tesseract_environment/environment.h>
#include <tesseract_geometry/impl/sphere.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_srdf/kinematics_information.h>

namespace arm_controller::controller::reactive_task {

namespace {
constexpr const char *kTrajOptNamespace = "TrajOptMotionPlannerTask";
constexpr const char *kTrajOptProfile = "REACTIVE_TASK";
constexpr const char *kDiscreteContactManager = "BulletDiscreteBVHManager";
constexpr const char *kContinuousContactManager = "BulletCastBVHManager";
constexpr double kCartesianCorridorPositionTolerance = 0.050;
constexpr double kCartesianCorridorOrientationTolerance = 0.35;
constexpr double kJointCorridorSoftToleranceRad = 0.35;
constexpr double kJointCorridorConstraintCoeff = 35.0;
constexpr double kJointCorridorCostCoeff = 4.0;
constexpr double kStartStateToleranceRad = 1.0e-4;
constexpr double kEndStateToleranceRad = 1.0e-4;
constexpr int kMaxTrajOptStates = 16;
constexpr double kInactiveObstacleOffsetM = 50.0;
constexpr double kDefaultObstacleRadiusM = 0.05;
constexpr const char *kObstacleSlotPrefix = "reactive_task_obstacle_slot_";
constexpr const char *kObstacleSlotJointPrefix = "joint_reactive_task_obstacle_slot_";

Eigen::Vector3d orientationErrorWorld(const Eigen::Matrix3d &current_rotation,
                                      const Eigen::Matrix3d &target_rotation) {
    const Eigen::Matrix3d R_err = current_rotation.transpose() * target_rotation;
    Eigen::AngleAxisd aa(R_err);
    if (std::abs(aa.angle()) < 1e-12) {
        return Eigen::Vector3d::Zero();
    }
    return current_rotation * (aa.axis() * aa.angle());
}

double jointCorridorDistance(
    const std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> &joint_waypoints,
    const Eigen::VectorXd &q,
    double *max_abs_distance) {
    if (max_abs_distance != nullptr) {
        *max_abs_distance = std::numeric_limits<double>::infinity();
    }
    if (joint_waypoints.empty() || q.size() <= 0 || !q.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }

    double best_norm = std::numeric_limits<double>::infinity();
    double best_max_abs = std::numeric_limits<double>::infinity();
    auto consider_reference = [&](const Eigen::VectorXd &q_ref) {
        if (q_ref.size() != q.size() || !q_ref.allFinite()) {
            return;
        }
        const Eigen::VectorXd delta = q - q_ref;
        const double norm = delta.norm();
        if (norm < best_norm) {
            best_norm = norm;
            best_max_abs = delta.cwiseAbs().maxCoeff();
        }
    };

    for (std::size_t i = 0; i < joint_waypoints.size(); ++i) {
        consider_reference(joint_waypoints[i]);
        if (i == 0u) {
            continue;
        }
        const Eigen::VectorXd &q0 = joint_waypoints[i - 1u];
        const Eigen::VectorXd &q1 = joint_waypoints[i];
        if (q0.size() != q.size() || q1.size() != q.size() ||
            !q0.allFinite() || !q1.allFinite()) {
            continue;
        }
        const Eigen::VectorXd segment = q1 - q0;
        const double segment_norm_sq = segment.squaredNorm();
        if (segment_norm_sq <= 1e-12) {
            continue;
        }
        const double alpha = std::clamp((q - q0).dot(segment) / segment_norm_sq, 0.0, 1.0);
        consider_reference(q0 + alpha * segment);
    }

    if (max_abs_distance != nullptr) {
        *max_abs_distance = best_max_abs;
    }
    return best_norm;
}

std::string obstacleSlotName(const int index) {
    return std::string(kObstacleSlotPrefix) + std::to_string(index);
}

std::string obstacleSlotJointName(const int index) {
    return std::string(kObstacleSlotJointPrefix) + std::to_string(index);
}

Eigen::Isometry3d obstacleSlotPose(const Eigen::Vector3d &center) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    return pose;
}

Eigen::Vector3d inactiveObstacleCenter(const int index) {
    return Eigen::Vector3d(kInactiveObstacleOffsetM + static_cast<double>(index),
                        kInactiveObstacleOffsetM,
                        kInactiveObstacleOffsetM);
}

double obstacleSlotRadius(const ReactiveTaskLocalPlanner::Input &input) {
    double radius = 0.0;
    for (const auto &obstacle : input.sphere_obstacles) {
        if (std::isfinite(obstacle.radius) && obstacle.radius > radius) {
            radius = obstacle.radius;
        }
    }
    return std::max(radius, kDefaultObstacleRadiusM);
}

int quantizedObstacleRadiusMm(const double radius_m) {
    return static_cast<int>(std::lround(std::max(0.0, radius_m) * 1000.0));
}

std::shared_ptr<tesseract_environment::Environment>makeWorkingEnvironment(
    const std::shared_ptr<const tesseract_environment::Environment> &base_env,
    const ReactiveTaskLocalPlanner::Input &input,
    const int slot_count,
    std::string *error) {
    if (!base_env) {
        if (error != nullptr) {
            *error = "tesseract_trajopt missing base environment";
        }
        return nullptr;
    }

    auto working_env = base_env->clone();
    if (!working_env || !working_env->isInitialized()) {
        if (error != nullptr) {
            *error = "tesseract_trajopt failed to clone environment";
        }
        return nullptr;
    }
    
    std::vector<std::shared_ptr<const tesseract_environment::Command>> commands;
    commands.reserve(static_cast<std::size_t>(std::max(0, slot_count)) * 2u);
    for (int i = 0; i < slot_count; ++i) {
        const bool active =
            i < static_cast<int>(input.sphere_obstacles.size()) &&
            input.sphere_obstacles[static_cast<std::size_t>(i)].center.allFinite();
        const Eigen::Vector3d center = active ? 
            input.sphere_obstacles[static_cast<std::size_t>(i)].center : inactiveObstacleCenter(i);
        // 每次局部规划时，把 obstacle slot 移动到当前点云障碍位置
        commands.push_back(std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
                obstacleSlotJointName(i), obstacleSlotPose(center)));
        // 启用/关闭对应障碍物的碰撞，没用到的挪到50m以外并禁用碰撞来尽量减少对规划性能的影响
        commands.push_back(std::make_shared<tesseract_environment::ChangeLinkCollisionEnabledCommand>(
            obstacleSlotName(i), active));
    }

    if (!commands.empty() && !working_env->applyCommands(commands)) {
        if (error != nullptr) {
            *error = "tesseract_trajopt failed to update obstacle slots";
        }
        return nullptr;
    }
    return working_env;
}

tesseract_common::ContactManagersPluginInfo makeBulletContactManagerInfo() {
    tesseract_common::ContactManagersPluginInfo info;
    info.search_libraries = {"tesseract_collision_bullet_factories"};

    tesseract_common::PluginInfo discrete_info;
    discrete_info.class_name = "BulletDiscreteBVHManagerFactory";
    info.discrete_plugin_infos.default_plugin = kDiscreteContactManager;
    info.discrete_plugin_infos.plugins[kDiscreteContactManager] = discrete_info;

    tesseract_common::PluginInfo continuous_info;
    continuous_info.class_name = "BulletCastBVHManagerFactory";
    info.continuous_plugin_infos.default_plugin = kContinuousContactManager;
    info.continuous_plugin_infos.plugins[kContinuousContactManager] = continuous_info;
    return info;
}

} // namespace

ReactiveTaskLocalPlanner::ReactiveTaskLocalPlanner()
    : ReactiveTaskLocalPlanner(Config{}) {}

ReactiveTaskLocalPlanner::ReactiveTaskLocalPlanner(Config cfg)
    : cfg_(std::move(cfg)) {}

void ReactiveTaskLocalPlanner::configure(Config cfg) {
    cfg_ = std::move(cfg);
}

bool ReactiveTaskLocalPlanner::compute(const Input &input,
                                           Output *output) const {
    if (output == nullptr) {
        return false;
    }

    *output = Output{};

    if (!input.q_goal_valid) {
        output->error = "tesseract_trajopt missing joint-space lookahead target";
        return false;
    }
    if (input.joint_names.empty() || input.q_current.size() != static_cast<Eigen::Index>(input.joint_names.size()) ||
        input.q_goal.size() != input.q_current.size()) {
        output->error = "tesseract_trajopt invalid joint seed/goal";
        return false;
    }
    if (input.planning_group.empty()) {
        output->error = "tesseract_trajopt missing planning_group";
        return false;
    }
    if (!input.joint_to_pose) {
        output->error = "tesseract_trajopt missing joint_to_pose callback";
        return false;
    }

    struct LocalWaypoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::VectorXd q;
        Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
        bool has_cartesian_pose{false};
    };

    std::vector<LocalWaypoint, Eigen::aligned_allocator<LocalWaypoint>> local_seed_waypoints;
    local_seed_waypoints.reserve(input.reference_samples.size() + 2u);
    LocalWaypoint current_waypoint;
    current_waypoint.q = input.q_current;
    current_waypoint.pose = input.current_pose;
    current_waypoint.has_cartesian_pose = false;
    local_seed_waypoints.push_back(std::move(current_waypoint));
    for (const auto &sample : input.reference_samples) {
        if (!sample.has_ik_joint_target ||
            sample.ik_joint_target.size() != input.q_current.size() ||
            !sample.ik_joint_target.allFinite()) {
            continue;
        }
        if ((sample.ik_joint_target - local_seed_waypoints.back().q).norm() < 1e-6) {
            continue;
        }
        LocalWaypoint waypoint;
        waypoint.q = sample.ik_joint_target;
        waypoint.pose = sample.T_target;
        waypoint.has_cartesian_pose = sample.T_target.matrix().allFinite();
        local_seed_waypoints.push_back(std::move(waypoint));
    }
    if ((input.q_goal - local_seed_waypoints.back().q).norm() >= 1e-6) {
        LocalWaypoint goal_waypoint;
        goal_waypoint.q = input.q_goal;
        goal_waypoint.has_cartesian_pose = false;
        if (!input.reference_samples.empty()) {
            const auto &last_sample = input.reference_samples.back();
            goal_waypoint.pose = last_sample.T_target;
            goal_waypoint.has_cartesian_pose = last_sample.T_target.matrix().allFinite();
        }
        local_seed_waypoints.push_back(std::move(goal_waypoint));
    }
    if (local_seed_waypoints.size() < 2u) {
        output->error = "tesseract_trajopt skipped: insufficient distinct joint waypoints";
        return false;
    }
    if (local_seed_waypoints.size() > static_cast<std::size_t>(kMaxTrajOptStates)) {
        std::vector<LocalWaypoint, Eigen::aligned_allocator<LocalWaypoint>> compact_waypoints;
        compact_waypoints.reserve(kMaxTrajOptStates);
        for (int i = 0; i < kMaxTrajOptStates; ++i) {
            const double alpha = static_cast<double>(i) / static_cast<double>(std::max(1, kMaxTrajOptStates - 1));
            const std::size_t index = std::min<std::size_t>(local_seed_waypoints.size() - 1u,
                static_cast<std::size_t>(std::llround(alpha * static_cast<double>(local_seed_waypoints.size() - 1u))));
            if (!compact_waypoints.empty() && (local_seed_waypoints[index].q - compact_waypoints.back().q).norm() < 1e-6) {
                continue;
            }
            compact_waypoints.push_back(local_seed_waypoints[index]);
        }
        if (compact_waypoints.size() >= 2u) {
        local_seed_waypoints = std::move(compact_waypoints);
        }
    }
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> joint_corridor_waypoints;
    joint_corridor_waypoints.reserve(local_seed_waypoints.size());
    for (const auto &waypoint : local_seed_waypoints) {
        joint_corridor_waypoints.push_back(waypoint.q);
    }

    std::string env_error;
    auto env_opaque = getOrCreateTesseractEnvironment(input, &env_error);
    if (!env_opaque) {
        output->error = env_error.empty()
                            ? "tesseract_trajopt environment unavailable"
                            : env_error;
        return false;
    }
    auto base_env = std::static_pointer_cast<const tesseract_environment::Environment>(env_opaque);
    if (!base_env || !base_env->isInitialized()) {
        output->error = "tesseract_trajopt environment is not initialized";
        return false;
    }
    const int obstacle_slot_count = std::max(0, cfg_.max_obstacle_spheres);
    std::string working_env_error;
    auto env = makeWorkingEnvironment(base_env, input, obstacle_slot_count, &working_env_error);
    if (!env) {
        output->error = working_env_error.empty()
                    ? "tesseract_trajopt working environment unavailable"
                    : working_env_error;
        return false;
    }
    const bool has_active_obstacles = obstacle_slot_count > 0 && !input.sphere_obstacles.empty();

    // TrajOpt 需要知道用哪个planning group, 哪个 base frame, 哪个 TCP/link frame 来规划, 
    // 这些信息在 TesseractEnvironment 的 ManipulatorInfo 里配置.
    tesseract_common::ManipulatorInfo manip;
    manip.manipulator = input.planning_group;
    manip.working_frame = input.base_link;
    manip.tcp_frame = input.tip_link;

    // 表示一整段规划任务，也就是 TrajOpt 要优化的轨迹程序，里面按顺序塞多个 MoveInstruction
    // 每个 MoveInstruction 代表一个轨迹段，包含一个 Waypoint（这里是 JointWaypoint）和一些参数配置（比如 cost/constraint 权重）。
    // TrajOpt 会根据这个程序生成对应的优化问题并求解。
    tesseract_planning::CompositeInstruction program(kTrajOptProfile);
    program.setManipulatorInfo(manip);
    const Eigen::VectorXd joint_corridor_lower_tolerance = Eigen::VectorXd::Constant(input.q_current.size(), -kJointCorridorSoftToleranceRad);
    const Eigen::VectorXd joint_corridor_upper_tolerance = Eigen::VectorXd::Constant(input.q_current.size(), kJointCorridorSoftToleranceRad);
    for (std::size_t i = 0; i < local_seed_waypoints.size(); ++i) {
        const auto &local_waypoint = local_seed_waypoints[i];
        if (i == 0u || i + 1u == local_seed_waypoints.size()) {
            // 硬指定一个关节状态，不允许偏离（因为 TrajOpt 默认的起点/终点约束是软约束，可能会导致优化结果的起点/终点不准确）
            tesseract_planning::StateWaypoint waypoint(input.joint_names, local_waypoint.q);
            program.push_back(tesseract_planning::MoveInstruction(
                waypoint, tesseract_planning::MoveInstructionType::FREESPACE,
                kTrajOptProfile));
            continue;
        }
        // 给中间关节参考点，并带 tolerance, 允许优化结果在这个走廊范围内偏离参考点, 以增加优化的可行性和鲁棒性.
        tesseract_planning::JointWaypoint waypoint(
            input.joint_names, local_waypoint.q, joint_corridor_lower_tolerance,
            joint_corridor_upper_tolerance);
        // 把 waypoint 包装成 MoveInstruction 加入到 TrajOpt 的优化程序里. 
        // 这里的 MoveInstructionType::FREESPACE 表示这是一个普通的轨迹段，没有特殊的约束要求.
        program.push_back(tesseract_planning::MoveInstruction(
            waypoint, tesseract_planning::MoveInstructionType::FREESPACE,
            kTrajOptProfile));
    }

    // Keep the rolling problem sparse. generateInterpolatedProgram's min_steps
    // is per segment, so a 10-sample local window can expand to 100+ TrajOpt
    // states and return stale references.
    tesseract_planning::CompositeInstruction seed_program = program;

    // 配置单个 move instruction 的代价/约束权重等参数, 这些参数会被 TrajOpt 用来构建优化问题的目标函数和约束条件.
    auto move_profile = std::make_shared<tesseract_planning::TrajOptDefaultMoveProfile>();
    auto composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();

    move_profile->cartesian_cost_config.enabled = false;
    move_profile->cartesian_cost_config.coeff = Eigen::VectorXd::Constant(6, 20.0);
    move_profile->cartesian_constraint_config.enabled = false;
    move_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, kJointCorridorConstraintCoeff);
    move_profile->joint_cost_config.enabled = true;     // 启用关节空间代价，允许优化结果在走廊范围内偏离参考点
    move_profile->joint_cost_config.use_tolerance_override = true;
    move_profile->joint_cost_config.lower_tolerance = joint_corridor_lower_tolerance;
    move_profile->joint_cost_config.upper_tolerance = joint_corridor_upper_tolerance;
    move_profile->joint_cost_config.coeff = Eigen::VectorXd::Constant(input.q_current.size(), kJointCorridorCostCoeff);
    move_profile->joint_constraint_config.enabled = false;      // 中间 waypoint 是软参考，不是硬约束
    move_profile->joint_constraint_config.use_tolerance_override = true;
    move_profile->joint_constraint_config.lower_tolerance = joint_corridor_lower_tolerance;
    move_profile->joint_constraint_config.upper_tolerance = joint_corridor_upper_tolerance;
    move_profile->joint_constraint_config.coeff = Eigen::VectorXd::Constant(input.q_current.size(), kJointCorridorConstraintCoeff);
    // 配置整条轨迹级别的代价/约束，比如平滑性代价和碰撞代价，这些会作用于整个轨迹而不是单个状态.
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = true;
    composite_profile->smooth_jerks = false;
    // 配置碰撞代价和碰撞约束：（距离阈值，权重系数）
    composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.005, 15.0);     
    composite_profile->collision_cost_config.enabled = cfg_.enable_collision_cost && has_active_obstacles;
    // 约束用 0 距离阈值严格禁止碰撞
    composite_profile->collision_constraint_config = trajopt_common::TrajOptCollisionConfig(0.0, 20.0);
    composite_profile->collision_constraint_config.enabled = cfg_.enable_collision_constraint && has_active_obstacles;

    // 把配置好的 profile 注册给 planner
    auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
    profiles->addProfile(kTrajOptNamespace, kTrajOptProfile, move_profile);
    profiles->addProfile(kTrajOptNamespace, kTrajOptProfile, composite_profile);

    // TrajOpt 求解请求，核心三件套：任务指令、环境、profile 配置
    tesseract_planning::PlannerRequest request;
    request.instructions = seed_program;
    request.env = env;
    request.profiles = profiles;
    request.verbose = false;

    // 调用 TrajOpt 求解器求解优化问题，得到优化后的轨迹结果.
    tesseract_planning::TrajOptMotionPlanner planner(kTrajOptNamespace);
    const tesseract_planning::PlannerResponse response = planner.solve(request);
    if (!response) {
        output->error = response.message.empty() ? "tesseract_trajopt solve failed"
                                                : response.message;
        return false;
    }
    // 把 planner 返回的 instruction 结果转换成 JointTrajectory, 后面再 FK 成 pose/twist 给控制器用.
    const auto trajectory = tesseract_planning::toJointTrajectory(response.results);
    if (trajectory.size() < 2u) {
        output->error = "tesseract_trajopt returned too few trajectory states";
        return false;
    }

    const double nominal_window_sec =
        std::max(1e-3, cfg_.dt_sec * static_cast<double>(std::max<std::size_t>(1u, input.reference_samples.size())));
    const double dt =
        std::max(1e-3, nominal_window_sec / static_cast<double>(std::max<std::size_t>(1u, trajectory.size() - 1u)));
    output->optimized_joint_trajectory.reserve(trajectory.size());
    output->target_poses.reserve(trajectory.size());
    output->target_twists.reserve(trajectory.size());
    Eigen::VectorXd previous_q = input.q_current;
    Eigen::Isometry3d previous_pose = input.current_pose;
    if (!input.joint_to_pose(previous_q, &previous_pose) || !previous_pose.matrix().allFinite()) {
        output->error = "tesseract_trajopt FK failed for current joint state";
        return false;
    }
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        const Eigen::VectorXd q_i = trajectory[i].position;
        if (q_i.size() != static_cast<Eigen::Index>(input.joint_names.size())) {
            output->error = "tesseract_trajopt returned joint dimension mismatch";
            return false;
        }
        if (i == 0u && (q_i - input.q_current).cwiseAbs().maxCoeff() >
                        kStartStateToleranceRad) {
            std::ostringstream error;
            error << "tesseract_trajopt start state moved: max_abs="
                    << (q_i - input.q_current).cwiseAbs().maxCoeff();
            output->error = error.str();
            return false;
        }
        if (i + 1u == trajectory.size() && (q_i - input.q_goal).cwiseAbs().maxCoeff() > kEndStateToleranceRad) {
            const Eigen::VectorXd delta = q_i - input.q_goal;
            std::ostringstream error;
            error << "tesseract_trajopt end state moved: max_abs="
                    << delta.cwiseAbs().maxCoeff() << " norm=" << delta.norm();
            output->error = error.str();
            return false;
        }
        const double corridor_distance = jointCorridorDistance(joint_corridor_waypoints, q_i, nullptr);
        if (!std::isfinite(corridor_distance)) {
            output->error = "tesseract_trajopt returned non-finite joint corridor distance";
            return false;
        }
        Eigen::Isometry3d pose_i = Eigen::Isometry3d::Identity();
        if (!input.joint_to_pose(q_i, &pose_i) || !pose_i.matrix().allFinite()) {
            output->error =
                "tesseract_trajopt FK failed for optimized joint trajectory";
            return false;
        }
        Eigen::Matrix<double, 6, 1> twist_i = Eigen::Matrix<double, 6, 1>::Zero();
        twist_i.head<3>() = (pose_i.translation() - previous_pose.translation()) / dt;
        twist_i.tail<3>() = orientationErrorWorld(previous_pose.linear(), pose_i.linear()) / dt;
        output->optimized_joint_trajectory.push_back(q_i);
        output->target_poses.push_back(pose_i);
        output->target_twists.push_back(twist_i);
        previous_q = q_i;
        previous_pose = pose_i;
    }

    const std::size_t next_index = std::min<std::size_t>(1u, output->target_poses.size() - 1u);
    const Eigen::VectorXd q_next = output->optimized_joint_trajectory[next_index];
    output->target_pose = output->target_poses[next_index];
    output->target_twist = output->target_twists[next_index];
    output->nominal_twist = output->target_twist;
    output->ok = true;
    output->used = true;
    output->sampled_steps = static_cast<int>(trajectory.size());
    output->has_optimized_joint_target = true;
    output->optimized_joint_target = q_next;
    output->optimized_joint_velocity = (q_next - input.q_current) / dt;
    output->optimized_joint_target_time_sec = dt;
    output->trajectory_dt_sec = dt;
    return true;
}

std::shared_ptr<const void> ReactiveTaskLocalPlanner::getOrCreateTesseractEnvironment(const Input &input, std::string *error) const {
    const std::string cache_key = makeEnvironmentCacheKey(input);
    {
        std::lock_guard<std::mutex> lock(tesseract_env_mutex_);
        const auto it = tesseract_env_cache_.find(cache_key);
        if (it != tesseract_env_cache_.end()) {
            return it->second;
        }
    }

    if (input.urdf_path.empty()) {
        if (error != nullptr) {
            *error = "tesseract_trajopt missing urdf_path";
        }
        return nullptr;
    }
    if (!std::filesystem::exists(input.urdf_path)) {
        if (error != nullptr) {
            *error = "tesseract_trajopt urdf_path does not exist: " + input.urdf_path;
        }
        return nullptr;
    }

    // 从 URDF/SRDF 文件创建 Tesseract Environment，URDF 定义了机器人模型，SRDF 定义了机器人语义信息（比如规划组和关节组）
    // 这些都是 TrajOpt 进行运动规划所必须的环境信息.
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract_environment::Environment>();
    bool initialized = false;
    if (!input.srdf_path.empty() && std::filesystem::exists(input.srdf_path)) {
        initialized = env->init(std::filesystem::path(input.urdf_path),
                                std::filesystem::path(input.srdf_path), locator);
    } else {
        initialized = env->init(std::filesystem::path(input.urdf_path), locator);
    }

    if (!initialized) {
        if (error != nullptr) {
            *error = "tesseract_trajopt failed to initialize Environment";
        }
        return nullptr;
    }

    // 注册 Bullet 碰撞检测插件
    if (!env->applyCommand(
        std::make_shared<tesseract_environment::AddContactManagersPluginInfoCommand>(makeBulletContactManagerInfo()))) {
        if (error != nullptr) {
            *error = "tesseract_trajopt failed to configure contact managers";
        }
        return nullptr;
    }

    // TrajOpt 需要知道用哪个 planning group 来规划，如果 SRDF 里没有定义，就动态添加一个包含所有指定关节的关节组.
    if (env->getJointGroup(input.planning_group) == nullptr) {
        tesseract_srdf::KinematicsInformation kin_info;
        kin_info.addJointGroup(input.planning_group, input.joint_names);
        if (!env->applyCommand(std::make_shared<tesseract_environment::AddKinematicsInformationCommand>(kin_info))) {
            if (error != nullptr) {
                *error = "tesseract_trajopt failed to add joint group: " + input.planning_group;
            }
            return nullptr;
        }
    }

    if (env->getJointGroup(input.planning_group) == nullptr) {
        if (error != nullptr) {
            *error = "tesseract_trajopt joint group unavailable after init: " + input.planning_group;
        }
        return nullptr;
    }

    // 预先往环境里加一组球形障碍物 slot, TrajOpt 通过启用/禁用这些障碍物来动态表示当前环境中的障碍物状态
    // 每个障碍物是一个固定关节挂到 base link 下的球形碰撞体, 位置根据输入的障碍物信息动态更新, 半径根据输入的障碍物尺寸动态更新.
    const int obstacle_slot_count = std::max(0, cfg_.max_obstacle_spheres);
    const double obstacle_radius = obstacleSlotRadius(input);
    const std::string parent_link = input.base_link.empty() ? env->getRootLinkName() : input.base_link;
    for (int i = 0; i < obstacle_slot_count; ++i) {
        tesseract_scene_graph::Link link(obstacleSlotName(i));
        auto collision = std::make_shared<tesseract_scene_graph::Collision>();
        collision->name = obstacleSlotName(i) + "_collision";
        collision->origin = Eigen::Isometry3d::Identity();
        collision->geometry = std::make_shared<tesseract_geometry::Sphere>(obstacle_radius);
        link.collision.push_back(std::move(collision));

        tesseract_scene_graph::Joint joint(obstacleSlotJointName(i));
        joint.type = tesseract_scene_graph::JointType::FIXED;
        joint.parent_link_name = parent_link;
        joint.child_link_name = obstacleSlotName(i);
        joint.parent_to_joint_origin_transform = obstacleSlotPose(inactiveObstacleCenter(i));

        if (!env->applyCommand(std::make_shared<tesseract_environment::AddLinkCommand>(link, joint, false))) {
            if (error != nullptr) {
                *error = "tesseract_trajopt failed to add obstacle slot: " +
                        obstacleSlotName(i);
            }
            return nullptr;
        }
        if (!env->applyCommand(
                std::make_shared<
                    tesseract_environment::ChangeLinkCollisionEnabledCommand>(
                    obstacleSlotName(i), false))) {
        if (error != nullptr) {
            *error = "tesseract_trajopt failed to disable obstacle slot: " +
                    obstacleSlotName(i);
        }
        return nullptr;
        }
    }

    {
        std::lock_guard<std::mutex> lock(tesseract_env_mutex_);
        tesseract_env_cache_[cache_key] = env;
    }
    return env;
}

std::string ReactiveTaskLocalPlanner::makeEnvironmentCacheKey(const Input &input) const {
    std::ostringstream oss;
    oss << input.robot_type << '|' << input.planning_group << '|'
        << input.base_link << '|' << input.tip_link << '|' << input.urdf_path
        << '|' << input.srdf_path << "|obs_slots:"
        << std::max(0, cfg_.max_obstacle_spheres) << "|obs_radius_mm:"
        << quantizedObstacleRadiusMm(obstacleSlotRadius(input));
    for (const auto &joint_name : input.joint_names) {
        oss << '|' << joint_name;
    }
    return oss.str();
}

} // namespace arm_controller::controller::reactive_task
