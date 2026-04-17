#include "reactive_task_controller.hpp"

#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include "controller_interface.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <cmath>
#include <fstream>
#include <future>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <yaml-cpp/yaml.h>
#include "algorithm/cartesian_path_planner/map/dummy_distance_field.hpp"
#include "algorithm/cartesian_path_planner/collision/whole_body_ellipsoid_pose_validator.hpp"
#include "algorithm/sphere_model/link_sphere_model.hpp"

namespace rq = arm_controller::algorithm::reactive_qp;
namespace cp = arm_controller::algorithm::cartesian_path_planner;

namespace {
double orientationErrorRad(const Eigen::Matrix3d& r_current, const Eigen::Matrix3d& r_target) {
    const Eigen::Matrix3d r_err = r_current.transpose() * r_target;
    Eigen::AngleAxisd aa(r_err);
    return std::abs(aa.angle());
}

Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose& pose) {
    Eigen::Quaterniond q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    if (q.norm() < 1e-8) {
        q = Eigen::Quaterniond::Identity();
    } else {
        q.normalize();
    }

    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    t.linear() = q.toRotationMatrix();
    return t;
}

bool parseVec3(const YAML::Node& node, Eigen::Vector3d& out) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        return false;
    }
    out << node[0].as<double>(), node[1].as<double>(), node[2].as<double>();
    return true;
}

std::string normalizeArmTypeForTracIk(const std::string& robot_type) {
    if (robot_type == "dual_arm620" || robot_type == "dual_arm380") {
        return robot_type;
    }
    if (robot_type.find("620") != std::string::npos) {
        return "arm620";
    }
    if (robot_type.find("380") != std::string::npos) {
        return "arm380";
    }
    return robot_type.empty() ? "arm620" : robot_type;
}

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Vector3d& p, const Eigen::Matrix3d& R) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z();
    const Eigen::Quaterniond q(R);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

bool loadRuntimeConfigFromYaml(
    const YAML::Node& root,
    ReactiveTaskController::ControllerRuntimeConfig& cfg,
    std::string* error) {
    auto setError = [&](const std::string& msg) {
        if (error != nullptr) {
            *error = msg;
        }
    };
    const YAML::Node rtc = root["reactive_task_controller"];
    if (!rtc || !rtc.IsMap()) {
        setError("Missing required map: reactive_task_controller");
        return false;
    }

    if (!rtc["max_control_ticks"]) {
        setError("Missing required key: reactive_task_controller.max_control_ticks");
        return false;
    }
    cfg.max_control_ticks = std::max(1, rtc["max_control_ticks"].as<int>());
    if (!rtc["neo_control_cycle_sec"]) {
        setError("Missing required key: reactive_task_controller.neo_control_cycle_sec");
        return false;
    }
    cfg.neo_control_cycle_sec = std::max(1e-4, rtc["neo_control_cycle_sec"].as<double>());
    if (!rtc["goal_position_tolerance"]) {
        setError("Missing required key: reactive_task_controller.goal_position_tolerance");
        return false;
    }
    cfg.goal_position_tolerance = std::max(1e-6, rtc["goal_position_tolerance"].as<double>());
    if (!rtc["goal_orientation_tolerance_rad"]) {
        setError("Missing required key: reactive_task_controller.goal_orientation_tolerance_rad");
        return false;
    }
    cfg.goal_orientation_tolerance_rad =
        std::max(1e-6, rtc["goal_orientation_tolerance_rad"].as<double>());

    if (const YAML::Node req = rtc["request"]; req && req.IsMap()) {
        if (!req["safe_distance"]) {
            setError("Missing required key: reactive_task_controller.request.safe_distance");
            return false;
        }
        cfg.request_safe_distance = std::max(0.0, req["safe_distance"].as<double>());
        if (!req["goal_tolerance"]) {
            setError("Missing required key: reactive_task_controller.request.goal_tolerance");
            return false;
        }
        cfg.request_goal_tolerance = std::max(1e-6, req["goal_tolerance"].as<double>());
        if (!req["map_margin_xyz"]) {
            setError("Missing required key: reactive_task_controller.request.map_margin_xyz");
            return false;
        }
        Eigen::Vector3d margin;
        if (!parseVec3(req["map_margin_xyz"], margin)) {
            setError("Invalid reactive_task_controller.request.map_margin_xyz, expected [x,y,z]");
            return false;
        }
        cfg.map_margin_xyz = margin.cwiseMax(Eigen::Vector3d::Zero());
        if (req["whole_body_postcheck_non_blocking"]) {
            cfg.whole_body_postcheck_non_blocking =
                req["whole_body_postcheck_non_blocking"].as<bool>();
        }
        if (req["whole_body_postcheck_max_attempts"]) {
            cfg.whole_body_postcheck_max_attempts =
                std::max(1, req["whole_body_postcheck_max_attempts"].as<int>());
        }
        if (req["whole_body_retry_forbidden_radius"]) {
            cfg.whole_body_retry_forbidden_radius =
                std::max(0.0, req["whole_body_retry_forbidden_radius"].as<double>());
        }
    } else {
        setError("Missing required map: reactive_task_controller.request");
        return false;
    }

    if (const YAML::Node obs = rtc["dummy_obstacle"]; obs && obs.IsMap()) {
        if (!obs["enable"] || !obs["radius"] || !obs["center_left_arm"] || !obs["center_right_arm"]) {
            setError("Missing required keys under reactive_task_controller.dummy_obstacle");
            return false;
        }
        cfg.enable_dummy_obstacle = obs["enable"].as<bool>();
        cfg.dummy_obstacle_radius = std::max(0.0, obs["radius"].as<double>());
        if (!parseVec3(obs["center_left_arm"], cfg.dummy_obstacle_center_left_arm)) {
            setError("Invalid reactive_task_controller.dummy_obstacle.center_left_arm, expected [x,y,z]");
            return false;
        }
        if (!parseVec3(obs["center_right_arm"], cfg.dummy_obstacle_center_right_arm)) {
            setError("Invalid reactive_task_controller.dummy_obstacle.center_right_arm, expected [x,y,z]");
            return false;
        }
    } else {
        setError("Missing required map: reactive_task_controller.dummy_obstacle");
        return false;
    }

    if (const YAML::Node rep = rtc["replanner"]; rep && rep.IsMap()) {
        if (!rep["segment_sample_step_m"] || !rep["replan_every_control_ticks"] ||
            !rep["control_cycle_sec"] || !rep["prediction_horizon_ticks"] ||
            !rep["planning_latency_sec"] || !rep["handoff_blend_points"]) {
            setError("Missing required keys under reactive_task_controller.replanner");
            return false;
        }
        cfg.replanner.segment_sample_step_m = std::max(1e-4, rep["segment_sample_step_m"].as<double>());
        cfg.replanner.replan_every_control_ticks = std::max(1, rep["replan_every_control_ticks"].as<int>());
        cfg.replanner.control_cycle_sec = std::max(1e-4, rep["control_cycle_sec"].as<double>());
        cfg.replanner.prediction_horizon_ticks = std::max(1, rep["prediction_horizon_ticks"].as<int>());
        cfg.replanner.planning_latency_sec = std::max(0.0, rep["planning_latency_sec"].as<double>());
        cfg.replanner.handoff_blend_points = std::max(1, rep["handoff_blend_points"].as<int>());
    } else {
        setError("Missing required map: reactive_task_controller.replanner");
        return false;
    }

    if (const YAML::Node planner = rtc["planner"]; planner && planner.IsMap()) {
        if (const YAML::Node common = planner["common"]; common && common.IsMap()) {
            if (!common["default_segment_speed"] || !common["enable_interpolator_smoothing"] ||
                !common["interpolator_continuity_order"] || !common["interpolator_target_dt"]) {
                setError("Missing required keys under reactive_task_controller.planner.common");
                return false;
            }
            cfg.planner_common.default_segment_speed = common["default_segment_speed"].as<double>();
            cfg.planner_common.enable_interpolator_smoothing =
                common["enable_interpolator_smoothing"].as<bool>();
            cfg.planner_common.interpolator_continuity_order =
                common["interpolator_continuity_order"].as<int>();
            cfg.planner_common.interpolator_target_dt = common["interpolator_target_dt"].as<double>();
        } else {
            setError("Missing required map: reactive_task_controller.planner.common");
            return false;
        }
        if (const YAML::Node astar = planner["astar"]; astar && astar.IsMap()) {
            if (!astar["voxel_resolution"] || !astar["neighbor_mode"] || !astar["use_se3_search"] ||
                !astar["orientation_bin_size_rad"] || !astar["orientation_goal_tolerance_rad"] ||
                !astar["enable_inplace_rotation_neighbors"] ||
                !astar["force_axis_translation_neighbors_in_se3"] ||
                !astar["obstacle_penalty_weight"] || !astar["corridor_deviation_weight"] ||
                !astar["goal_shortcut_clearance_margin"] || !astar["orientation_cost_weight"] ||
                !astar["orientation_heuristic_weight"] || !astar["max_iterations"] ||
                !astar["max_planning_time_sec"] || !astar["edge_check_step"]) {
                setError("Missing required keys under reactive_task_controller.planner.astar");
                return false;
            }
            cfg.planner_astar.voxel_resolution = astar["voxel_resolution"].as<double>();
            cfg.planner_astar.neighbor_mode = astar["neighbor_mode"].as<int>();
            cfg.planner_astar.use_se3_search = astar["use_se3_search"].as<bool>();
            cfg.planner_astar.orientation_bin_size_rad = astar["orientation_bin_size_rad"].as<double>();
            cfg.planner_astar.orientation_goal_tolerance_rad =
                astar["orientation_goal_tolerance_rad"].as<double>();
            cfg.planner_astar.enable_inplace_rotation_neighbors =
                astar["enable_inplace_rotation_neighbors"].as<bool>();
            cfg.planner_astar.force_axis_translation_neighbors_in_se3 =
                astar["force_axis_translation_neighbors_in_se3"].as<bool>();
            cfg.planner_astar.obstacle_penalty_weight = astar["obstacle_penalty_weight"].as<double>();
            cfg.planner_astar.corridor_deviation_weight = astar["corridor_deviation_weight"].as<double>();
            cfg.planner_astar.goal_shortcut_clearance_margin =
                astar["goal_shortcut_clearance_margin"].as<double>();
            cfg.planner_astar.orientation_cost_weight = astar["orientation_cost_weight"].as<double>();
            cfg.planner_astar.orientation_heuristic_weight =
                astar["orientation_heuristic_weight"].as<double>();
            cfg.planner_astar.max_iterations = astar["max_iterations"].as<int>();
            cfg.planner_astar.max_planning_time_sec = astar["max_planning_time_sec"].as<double>();
            cfg.planner_astar.edge_check_step = astar["edge_check_step"].as<double>();
        } else {
            setError("Missing required map: reactive_task_controller.planner.astar");
            return false;
        }
        if (const YAML::Node smoothing = planner["smoothing"]; smoothing && smoothing.IsMap()) {
            if (!smoothing["max_shortcut_trials"] || !smoothing["collision_check_step"] ||
                !smoothing["local_adjust_iterations"] || !smoothing["local_adjust_alpha"]) {
                setError("Missing required keys under reactive_task_controller.planner.smoothing");
                return false;
            }
            cfg.planner_smoothing.max_shortcut_trials = smoothing["max_shortcut_trials"].as<int>();
            cfg.planner_smoothing.collision_check_step = smoothing["collision_check_step"].as<double>();
            cfg.planner_smoothing.local_adjust_iterations = smoothing["local_adjust_iterations"].as<int>();
            cfg.planner_smoothing.local_adjust_alpha = smoothing["local_adjust_alpha"].as<double>();
        } else {
            setError("Missing required map: reactive_task_controller.planner.smoothing");
            return false;
        }
    } else {
        setError("Missing required map: reactive_task_controller.planner");
        return false;
    }

    if (const YAML::Node mit = rtc["mit"]; mit && mit.IsMap()) {
        if (!mit["kp"] || !mit["kd"] || !mit["max_motors"]) {
            setError("Missing required keys under reactive_task_controller.mit");
            return false;
        }
        cfg.mit_kp = mit["kp"].as<double>();
        cfg.mit_kd = mit["kd"].as<double>();
        cfg.mit_max_motors = std::max(1, mit["max_motors"].as<int>());
    } else {
        setError("Missing required map: reactive_task_controller.mit");
        return false;
    }

    return true;
}

}  // namespace

ReactiveTaskController::ReactiveTaskController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<geometry_msgs::msg::Pose>("ReactiveTask", node) {
    hardware_manager_ = HardwareManager::getInstance();
    reactive_cfg_loaded_ = loadReactiveConfig();

    planning_worker_running_ = true;
    planning_worker_ = std::make_unique<std::thread>(&ReactiveTaskController::planning_worker_thread, this);

    consumer_running_ = true;
    queue_consumer_ = std::make_unique<std::thread>(&ReactiveTaskController::command_queue_consumer_thread, this);
}

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

bool ReactiveTaskController::loadReactiveConfig() {
    try {
        const std::string cfg_path =
            ament_index_cpp::get_package_share_directory("arm_controller") +
            "/config/reactive_task_config.yaml";
        std::string error;

        if (!rq::ReactiveQpExampleConfigLoader::loadFromYaml(cfg_path, reactive_cfg_, &error)) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "ReactiveTask: failed to load reactive config '%s': %s.",
                cfg_path.c_str(),
                error.c_str());
            return false;
        }

        const YAML::Node root = YAML::LoadFile(cfg_path);
        if (!loadRuntimeConfigFromYaml(root, runtime_cfg_, &error)) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "ReactiveTask: failed to load runtime config '%s': %s.",
                cfg_path.c_str(),
                error.c_str());
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "ReactiveTask: load config exception: %s", e.what());
        return false;
    }
}

void ReactiveTaskController::start(const std::string& mapping) {
    if (!reactive_cfg_loaded_) {
        throw std::runtime_error(
            "ReactiveTask config not loaded. Check config/reactive_task_config.yaml");
    }
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] ReactiveTask: not found in hardware configuration.");
    }

    TrajectoryControllerImpl::start(mapping);

    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    std::string error;
    if (!initializeMappingContext(mapping, &error)) {
        throw std::runtime_error("ReactiveTask context init failed for '" + mapping + "': " + error);
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] ReactiveTaskController activated", mapping.c_str());
}

bool ReactiveTaskController::stop(const std::string& mapping) {
    TrajectoryControllerImpl::stop(mapping);
    cleanup_subscriptions(mapping);
    RCLCPP_INFO(node_->get_logger(), "[%s] ReactiveTaskController deactivated", mapping.c_str());
    return true;
}

bool ReactiveTaskController::initializeMappingContext(const std::string& mapping, std::string* error) {
    std::lock_guard<std::mutex> lock(mapping_contexts_mutex_);
    auto& ctx = mapping_contexts_[mapping];
    if (ctx.initialized) {
        return true;
    }

    ctx.joint_names = hardware_manager_->get_joint_names(mapping);
    if (ctx.joint_names.empty()) {
        if (error != nullptr) {
            *error = "joint_names is empty";
        }
        return false;
    }

    ctx.robot_type = hardware_manager_->get_robot_type(mapping);
    if (ctx.robot_type.empty()) {
        if (error != nullptr) {
            *error = "robot_type is empty";
        }
        return false;
    }

    pinocchio::Model model;
    try {
        const std::string urdf_path =
            ament_index_cpp::get_package_share_directory("robot_description") +
            "/urdf/" + ctx.robot_type + ".urdf";
        pinocchio::urdf::buildModel(urdf_path, model);
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = std::string("build pinocchio model failed: ") + e.what();
        }
        return false;
    }

    std::vector<int> q_indices;
    std::vector<int> v_indices;
    q_indices.reserve(ctx.joint_names.size());
    v_indices.reserve(ctx.joint_names.size());
    for (const auto& joint_name : ctx.joint_names) {
        if (!model.existJointName(joint_name)) {
            if (error != nullptr) {
                *error = "joint not found in urdf: " + joint_name;
            }
            return false;
        }
        const auto jid = model.getJointId(joint_name);
        q_indices.push_back(static_cast<int>(model.joints[jid].idx_q()));
        v_indices.push_back(static_cast<int>(model.joints[jid].idx_v()));
    }

    pinocchio::FrameIndex ee_frame = 0;
    const auto last_joint = model.getJointId(ctx.joint_names.back());
    for (pinocchio::FrameIndex fid = 0; fid < model.frames.size(); ++fid) {
        if (model.frames[fid].parentJoint == last_joint) {
            ee_frame = fid;
        }
    }

    ctx.fk_provider = std::make_shared<arm_controller::kinematics::PinocchioForwardKinematics>(
        node_, model, q_indices, ee_frame);
    if (!ctx.fk_provider->initialize()) {
        if (error != nullptr) {
            *error = "fk provider initialize failed";
        }
        return false;
    }

    ctx.jacobian_provider = std::make_shared<arm_controller::kinematics::PinocchioJacobianProvider>(
        node_, model, q_indices, v_indices, ee_frame);
    if (!ctx.jacobian_provider->initialize()) {
        if (error != nullptr) {
            *error = "jacobian provider initialize failed";
        }
        return false;
    }

    ctx.manipulability_gradient =
        std::make_unique<rq::ManipulabilityGradient>(ctx.jacobian_provider);

    try {
        const std::string planning_group = hardware_manager_->get_planning_group(mapping);
        if (!planning_group.empty()) {
            ctx.moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                node_, planning_group, "reactive_task");
            ctx.tracik_adapter = std::make_shared<trajectory_planning::infrastructure::integration::TracIKAdapter>(
                node_, planning_group);

            if (ctx.moveit_adapter && ctx.tracik_adapter) {
                ctx.tracik_adapter->setMoveItAdapter(ctx.moveit_adapter.get());

                std::string urdf_xml = ctx.moveit_adapter->getURDFString(ctx.robot_type);
                if (urdf_xml.empty()) {
                    const std::string urdf_path =
                        ament_index_cpp::get_package_share_directory("robot_description") +
                        "/urdf/" + ctx.robot_type + ".urdf";
                    std::ifstream ifs(urdf_path);
                    urdf_xml.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
                }

                const std::string base_link = ctx.moveit_adapter->getBaseLink();
                const std::string tip_link = ctx.moveit_adapter->getEndEffectorLink();
                const bool kdl_ok =
                    !base_link.empty() && !tip_link.empty() &&
                    ctx.tracik_adapter->initializeKDLChain(urdf_xml, base_link, tip_link);
                const bool solver_ok =
                    kdl_ok && ctx.tracik_adapter->initializeSolver(normalizeArmTypeForTracIk(ctx.robot_type));
                ctx.tracik_ready = (kdl_ok && solver_ok);
            }
        }
    } catch (const std::exception&) {
        ctx.tracik_ready = false;
    }

    const std::string reactive_cfg_path =
        ament_index_cpp::get_package_share_directory("arm_controller") +
        "/config/reactive_task_config.yaml";
    std::string pref_error;
    if (!rq::JointPreferenceLoader::loadFromYaml(
            reactive_cfg_path, ctx.joint_names, ctx.joint_preference_cfg, &pref_error)) {
        if (error != nullptr) {
            *error = "load joint preference failed: " + pref_error;
        }
        return false;
    }

    const std::string hardware_cfg_path =
        ament_index_cpp::get_package_share_directory("arm_controller") + "/config/hardware_config.yaml";
    std::string collision_error;
    if (!arm_controller::algorithm::sphere_model::LinkSphereModel::buildEllipsoidsForMapping(
            hardware_cfg_path, mapping, model, ctx.collision_ellipsoids, &collision_error)) {
        if (error != nullptr) {
            *error = "build collision ellipsoids failed: " + collision_error;
        }
        return false;
    }
    if (ctx.collision_ellipsoids.empty()) {
        if (error != nullptr) {
            *error = "collision ellipsoid model is empty";
        }
        return false;
    }

    const int dof = static_cast<int>(ctx.joint_names.size());
    ctx.qd_min = Eigen::VectorXd::Zero(dof);
    ctx.qd_max = Eigen::VectorXd::Zero(dof);
    ctx.joint_limits.q_min = Eigen::VectorXd::Zero(dof);
    ctx.joint_limits.q_max = Eigen::VectorXd::Zero(dof);

    for (int i = 0; i < dof; ++i) {
        JointLimits limits;
        hardware_manager_->get_joint_limits(ctx.joint_names[static_cast<std::size_t>(i)], limits);
        ctx.joint_limits.q_min(i) = limits.min_position;
        ctx.joint_limits.q_max(i) = limits.max_position;
        const double vmax = (limits.has_velocity_limits && limits.max_velocity > 1e-6)
                                ? limits.max_velocity
                                : 1.0;
        ctx.qd_min(i) = -vmax;
        ctx.qd_max(i) = vmax;
    }

    ctx.initialized = true;
    return true;
}

std::shared_ptr<cp::CartesianPathPlanner> ReactiveTaskController::buildPlanner(
    const std::shared_ptr<const cp::DistanceFieldInterface>& map,
    const Eigen::Vector3d& map_min) const {
    return std::make_shared<cp::CartesianPathPlanner>(
        runtime_cfg_.planner_common,
        runtime_cfg_.planner_astar,
        runtime_cfg_.planner_smoothing,
        map,
        map_min);
}

void ReactiveTaskController::trajectory_callback(
    const std::string& mapping,
    const geometry_msgs::msg::Pose::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(planning_queue_mutex_);
        planning_queue_.push(PlanningTask{mapping, msg});
    }
    planning_queue_cv_.notify_one();
}

void ReactiveTaskController::planning_worker_thread() {
    while (planning_worker_running_) {
        PlanningTask task;
        {
            std::unique_lock<std::mutex> lock(planning_queue_mutex_);
            planning_queue_cv_.wait(lock, [this]() {
                return !planning_queue_.empty() || !planning_worker_running_;
            });
            if (!planning_worker_running_) {
                return;
            }
            task = planning_queue_.front();
            planning_queue_.pop();
        }
        plan_and_execute(task.mapping, task.msg);
    }
}

void ReactiveTaskController::plan_and_execute(
    const std::string& mapping,
    const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!msg) {
        return;
    }

    std::string init_error;
    if (!initializeMappingContext(mapping, &init_error)) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ReactiveTask init failed: %s", mapping.c_str(), init_error.c_str());
        last_execution_success_[mapping] = false;
        return;
    }

    MappingContext* ctx = nullptr;
    {
        std::lock_guard<std::mutex> lock(mapping_contexts_mutex_);
        ctx = &mapping_contexts_[mapping];
    }
    if (ctx == nullptr || !ctx->initialized) {
        last_execution_success_[mapping] = false;
        return;
    }

    const std::vector<double> q_current_vec = hardware_manager_->get_current_joint_positions_lockfree(mapping);
    if (q_current_vec.size() != ctx->joint_names.size()) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ReactiveTask current joint size mismatch", mapping.c_str());
        last_execution_success_[mapping] = false;
        return;
    }

    const Eigen::VectorXd q_start = Eigen::Map<const Eigen::VectorXd>(
        q_current_vec.data(), static_cast<Eigen::Index>(q_current_vec.size()));

    arm_controller::kinematics::ForwardKinematicsOutput fk_start;
    if (!ctx->fk_provider->compute(q_start, fk_start)) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ReactiveTask FK failed at start", mapping.c_str());
        last_execution_success_[mapping] = false;
        return;
    }

    const Eigen::Isometry3d T_goal = poseMsgToIso(*msg);

    cp::PathPlanningInput request;
    request.p_start = fk_start.ee_position;
    request.R_start = fk_start.ee_rotation;
    request.p_goal = T_goal.translation();
    request.R_goal = T_goal.linear();
    request.q_start_seed = q_start;
    request.safe_distance = runtime_cfg_.request_safe_distance;
    request.goal_tolerance = runtime_cfg_.request_goal_tolerance;
    request.whole_body_postcheck_non_blocking =
        runtime_cfg_.whole_body_postcheck_non_blocking;
    request.whole_body_postcheck_max_attempts =
        runtime_cfg_.whole_body_postcheck_max_attempts;
    request.whole_body_retry_forbidden_radius =
        runtime_cfg_.whole_body_retry_forbidden_radius;

    const Eigen::Vector3d min_corner = request.p_start.cwiseMin(request.p_goal);
    const Eigen::Vector3d max_corner = request.p_start.cwiseMax(request.p_goal);
    const Eigen::Vector3d map_margin = runtime_cfg_.map_margin_xyz;
    const Eigen::Vector3d map_min = min_corner - map_margin;
    const Eigen::Vector3d map_max = max_corner + map_margin;

    auto map = std::make_shared<cp::DummyDistanceField>(map_min, map_max);
    if (runtime_cfg_.enable_dummy_obstacle && runtime_cfg_.dummy_obstacle_radius > 0.0) {
        cp::SphereObstacle obstacle;
        if (mapping == "right_arm") {
            obstacle.center = runtime_cfg_.dummy_obstacle_center_right_arm;
        } else {
            obstacle.center = runtime_cfg_.dummy_obstacle_center_left_arm;
        }
        obstacle.radius = runtime_cfg_.dummy_obstacle_radius;
        map->addSphere(obstacle);
    }
    auto planner = buildPlanner(map, map_min);
    const auto obstacle_distance_query =
        rq::BodyObstacleConstraintBuilder::makeDistanceQueryFromField(map);

    std::optional<cp::WholeBodyEllipsoidPoseValidator> whole_body_validator;
    if (ctx->tracik_ready && ctx->tracik_adapter && ctx->moveit_adapter && !ctx->collision_ellipsoids.empty()) {
        cp::WholeBodyEllipsoidPoseValidator::Config wb_cfg;
        wb_cfg.ik_max_iterations = 25;
        wb_cfg.ik_pos_tolerance_m = 0.01;
        wb_cfg.ik_rot_tolerance_rad = 0.15;
        wb_cfg.ik_damping = 0.05;
        wb_cfg.ik_step_scale = 0.6;
        wb_cfg.segment_substeps_min = 1;
        wb_cfg.default_q_seed = q_start;
        wb_cfg.ik_solver_fn =
            [moveit = ctx->moveit_adapter, tracik = ctx->tracik_adapter, seed_default = q_current_vec](
                const Eigen::Vector3d& p_target,
                const Eigen::Matrix3d& R_target,
                const std::optional<Eigen::VectorXd>& q_seed,
                Eigen::VectorXd& q_solution) -> bool {
                if (!moveit || !tracik) {
                    return false;
                }

                std::vector<double> seed = seed_default;
                if (q_seed.has_value() && q_seed->size() > 0) {
                    seed.assign(q_seed->data(), q_seed->data() + q_seed->size());
                }
                if (seed.empty()) {
                    return false;
                }

                const geometry_msgs::msg::Pose pose_world = toPoseMsg(p_target, R_target);
                const geometry_msgs::msg::Pose pose_base =
                    moveit->worldPoseToBaseLinkPose(pose_world);

                std::vector<double> q_solution_vec;
                if (!tracik->computeIKClosest(pose_base, seed, q_solution_vec, 5, false)) {
                    return false;
                }
                if (q_solution_vec.empty()) {
                    return false;
                }
                q_solution = Eigen::Map<const Eigen::VectorXd>(
                    q_solution_vec.data(),
                    static_cast<Eigen::Index>(q_solution_vec.size()));
                return true;
            };

        whole_body_validator.emplace(
            wb_cfg,
            map,
            ctx->fk_provider,
            ctx->jacobian_provider,
            ctx->collision_ellipsoids);
        request.whole_body_pose_validator = whole_body_validator->makePoseValidatorFn();
        request.whole_body_segment_validator = whole_body_validator->makeSegmentValidatorFn();
        request.whole_body_pose_diagnostic = whole_body_validator->makePoseDiagnosticFn();
    } else {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] WholeBody validator disabled: tracik_ready=%s moveit=%s tracik=%s ellipsoids=%zu",
            mapping.c_str(),
            ctx->tracik_ready ? "true" : "false",
            ctx->moveit_adapter ? "true" : "false",
            ctx->tracik_adapter ? "true" : "false",
            ctx->collision_ellipsoids.size());
    }

    cp::ReplannerManager replanner(planner);
    const cp::ReplannerConfig replanner_cfg = runtime_cfg_.replanner;
    replanner.setConfig(replanner_cfg);

    std::string error;
    if (!replanner.start(request, &error)) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] ReactiveTask planner start failed: %s",
            mapping.c_str(),
            error.c_str());
        last_execution_success_[mapping] = false;
        return;
    }

    rq::ReactiveQpSolver solver;
    rq::TaskVelocityGenerator task_velocity_generator;

    bool reached_goal = false;
    bool sample_ok = true;
    const double planner_tick_sec = std::max(1e-4, replanner_cfg.control_cycle_sec);
    const double neo_tick_sec = std::max(1e-4, runtime_cfg_.neo_control_cycle_sec);
    double planner_time_sec = 0.0;
    double planner_tick_accumulator = 0.0;
    int planner_tick = 0;
    int neo_iter = 0;
    const int safety_log_stride = std::max(1, static_cast<int>(std::llround(0.1 / neo_tick_sec)));

    auto vecToStr = [](const Eigen::VectorXd& v) {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(4);
        oss << "[";
        for (Eigen::Index i = 0; i < v.size(); ++i) {
            oss << v(i);
            if (i + 1 != v.size()) {
                oss << ", ";
            }
        }
        oss << "]";
        return oss.str();
    };

    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] ReactiveTask motor command is enabled.",
        mapping.c_str());
    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] safety_config: neo_dt=%.4f s, planner_dt=%.4f s, replan_every=%d ticks, max_ticks=%d, safe_distance=%.3f, pos_tol=%.4f, ori_tol=%.4f",
        mapping.c_str(),
        neo_tick_sec,
        planner_tick_sec,
        replanner_cfg.replan_every_control_ticks,
        runtime_cfg_.max_control_ticks,
        request.safe_distance,
        runtime_cfg_.goal_position_tolerance,
        runtime_cfg_.goal_orientation_tolerance_rad);
    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] start_goal: p_start=(%.4f, %.4f, %.4f), p_goal=(%.4f, %.4f, %.4f), q_start=%s",
        mapping.c_str(),
        request.p_start.x(),
        request.p_start.y(),
        request.p_start.z(),
        request.p_goal.x(),
        request.p_goal.y(),
        request.p_goal.z(),
        vecToStr(q_start).c_str());
    if (runtime_cfg_.enable_dummy_obstacle) {
        const Eigen::Vector3d center = (mapping == "right_arm")
                                           ? runtime_cfg_.dummy_obstacle_center_right_arm
                                           : runtime_cfg_.dummy_obstacle_center_left_arm;
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] dummy_obstacle: center=(%.4f, %.4f, %.4f), radius=%.4f",
            mapping.c_str(),
            center.x(),
            center.y(),
            center.z(),
            runtime_cfg_.dummy_obstacle_radius);
    }

    cp::TimedCartesianSample sample;
    if (!replanner.sampleByElapsedTime(planner_time_sec, sample)) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ReactiveTask initial sample failed", mapping.c_str());
        sample_ok = false;
    }
    Eigen::VectorXd ik_seed_q = q_start;
    bool ik_seed_valid = (ik_seed_q.size() == q_start.size()) && ik_seed_q.allFinite();
    const int chunk_ticks = std::max(1, replanner_cfg.replan_every_control_ticks);
    int ticks_in_chunk = 0;
    int chunk_index = 0;
    bool waiting_for_chunk_commit = false;
    bool pending_plan_inflight = false;
    std::future<std::pair<bool, std::string>> pending_plan_future;
    Eigen::VectorXd q_prev_feedback = q_start;
    bool q_prev_feedback_valid = (q_prev_feedback.size() == q_start.size()) && q_prev_feedback.allFinite();
    double best_pos_err = (request.p_start - request.p_goal).norm();
    double best_ori_err = orientationErrorRad(request.R_start, request.R_goal);
    int no_motion_cycles = 0;
    int no_progress_cycles = 0;
    const int no_motion_cycle_limit =
        std::max(20, static_cast<int>(std::llround(0.6 / neo_tick_sec)));
    const int no_progress_cycle_limit =
        std::max(50, static_cast<int>(std::llround(2.0 / neo_tick_sec)));
    constexpr double kWatchdogCmdNormMin = 0.6;
    constexpr double kWatchdogJointDeltaMax = 5e-4;
    constexpr double kPosProgressEps = 1e-4;
    constexpr double kOriProgressEps = 1e-3;
    const double feedback_stale_threshold_sec = std::max(0.2, 5.0 * neo_tick_sec);

    auto launch_pending_chunk_plan = [&]() {
        if (pending_plan_inflight) {
            return;
        }
        const int exec_idx = replanner.pointIndexAtTime(planner_time_sec);
        const int ticks_until_commit = std::max(0, chunk_ticks - ticks_in_chunk);
        const double handoff_time_sec =
            planner_time_sec + static_cast<double>(ticks_until_commit) * planner_tick_sec;
        const int handoff_idx = replanner.pointIndexAtTime(handoff_time_sec);
        try {
            pending_plan_inflight = true;
            pending_plan_future = std::async(
                std::launch::async,
                [&replanner, request, handoff_idx]() -> std::pair<bool, std::string> {
                    std::string local_error;
                    const bool ok =
                        replanner.preparePendingFromActiveTrajectoryPoint(
                            request, handoff_idx, &local_error);
                    return std::make_pair(ok, local_error);
                });
            RCLCPP_INFO(
                node_->get_logger(),
                "[%s] chunk_pipeline: launch pending plan for chunk=%d from exec_idx=%d handoff_idx=%d (planner_t=%.3f handoff_t=%.3f ticks_left=%d)",
                mapping.c_str(),
                chunk_index + 1,
                exec_idx,
                handoff_idx,
                planner_time_sec,
                handoff_time_sec,
                ticks_until_commit);
        } catch (const std::exception& e) {
            pending_plan_inflight = false;
            RCLCPP_WARN(
                node_->get_logger(),
                "[%s] chunk_pipeline: async launch failed: %s",
                mapping.c_str(),
                e.what());
        }
    };

    launch_pending_chunk_plan();

    while (rclcpp::ok() && sample_ok && planner_tick < runtime_cfg_.max_control_ticks) {
        if (!is_active(mapping)) {
            break;
        }

        const std::vector<double> q_now_vec = hardware_manager_->get_current_joint_positions_lockfree(mapping);
        if (q_now_vec.size() != ctx->joint_names.size()) {
            break;
        }
        const double feedback_age_sec = hardware_manager_->get_joint_feedback_age_sec(mapping);
        if (!std::isfinite(feedback_age_sec) || feedback_age_sec > feedback_stale_threshold_sec) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] ReactiveTask abort: stale joint feedback age=%.3f s (threshold=%.3f s).",
                mapping.c_str(),
                feedback_age_sec,
                feedback_stale_threshold_sec);
            break;
        }
        const Eigen::VectorXd q_now = Eigen::Map<const Eigen::VectorXd>(
            q_now_vec.data(), static_cast<Eigen::Index>(q_now_vec.size()));

        arm_controller::kinematics::ForwardKinematicsOutput fk_now;
        if (!ctx->fk_provider->compute(q_now, fk_now)) {
            break;
        }

        rq::TaskVelocityInput task_in;
        task_in.T_current = fk_now.ee_pose;
        task_in.has_target_pose = true;
        task_in.T_target = sample.T_target;
        // Disable feedforward twist for ReactiveTask temporarily and use
        // pure pose-error feedback to rule out end-segment feedforward bias.
        task_in.has_target_twist = false;
        task_in.target_twist.setZero();
        const rq::TaskVelocityOutput task_out =
            task_velocity_generator.compute(task_in, reactive_cfg_.task_velocity);

        const Eigen::MatrixXd J =
            ctx->jacobian_provider->computeJacobian(q_now, "", Eigen::Vector3d::Zero());
        if (J.rows() != 6 || J.cols() != q_now.size()) {
            break;
        }

        Eigen::VectorXd manip_grad = Eigen::VectorXd::Zero(q_now.size());
        double log_m = 0.0;
        if (ctx->manipulability_gradient) {
            const bool ok_manip = ctx->manipulability_gradient->compute(
                q_now,
                reactive_cfg_.manipulability,
                manip_grad,
                &log_m);
            if (!ok_manip) {
                manip_grad.setZero();
            }
        }

        Eigen::VectorXd q_min_task = ctx->joint_limits.q_min;
        Eigen::VectorXd q_max_task = ctx->joint_limits.q_max;
        Eigen::VectorXd posture_qdot_ref = Eigen::VectorXd::Zero(q_now.size());
        Eigen::VectorXd posture_joint_weights = Eigen::VectorXd::Ones(q_now.size());
        Eigen::VectorXd ik_target_q;
        bool using_dynamic_tracik_posture = false;
        if (sample.has_ik_joint_target &&
            sample.ik_joint_target.size() == q_now.size() &&
            sample.ik_joint_target.allFinite()) {
            ik_target_q = sample.ik_joint_target;
            posture_qdot_ref = ctx->joint_preference_cfg.posture_k * (ik_target_q - q_now);
            ik_seed_q = ik_target_q;
            ik_seed_valid = true;
            using_dynamic_tracik_posture = true;
        } else if (ctx->tracik_ready && ctx->tracik_adapter && ctx->moveit_adapter) {
            const geometry_msgs::msg::Pose pose_world =
                toPoseMsg(sample.T_target.translation(), sample.T_target.linear());
            const geometry_msgs::msg::Pose pose_base =
                ctx->moveit_adapter->worldPoseToBaseLinkPose(pose_world);
            const Eigen::VectorXd& seed_vec = (ik_seed_valid && ik_seed_q.size() == q_now.size())
                                                  ? ik_seed_q
                                                  : q_now;
            std::vector<double> seed(seed_vec.data(), seed_vec.data() + seed_vec.size());
            std::vector<double> q_ik_vec;
            if (ctx->tracik_adapter->computeIKClosest(pose_base, seed, q_ik_vec, 5, false) &&
                q_ik_vec.size() == static_cast<std::size_t>(q_now.size())) {
                const Eigen::VectorXd q_ik = Eigen::Map<const Eigen::VectorXd>(
                    q_ik_vec.data(), static_cast<Eigen::Index>(q_ik_vec.size()));
                ik_target_q = q_ik;
                ik_seed_q = q_ik;
                ik_seed_valid = true;
                posture_qdot_ref = ctx->joint_preference_cfg.posture_k * (q_ik - q_now);
                using_dynamic_tracik_posture = true;
            }
        }
        if (!using_dynamic_tracik_posture) {
            posture_qdot_ref.setZero();
        }

        rq::ReactiveQpBuildInput qp_input;
        qp_input.q_current = q_now;
        qp_input.jacobian_task = J;
        qp_input.desired_twist = task_out.v_des;
        qp_input.manipulability_gradient = manip_grad;
        qp_input.posture_velocity_reference = posture_qdot_ref;
        qp_input.posture_joint_weights = posture_joint_weights;
        qp_input.qd_min = ctx->qd_min;
        qp_input.qd_max = ctx->qd_max;
        qp_input.joint_limits.q_min = q_min_task;
        qp_input.joint_limits.q_max = q_max_task;
        if (reactive_cfg_.qp_build.enable_obstacle_damper && runtime_cfg_.enable_dummy_obstacle) {
            std::string obstacle_error;
            const int generated = rq::BodyObstacleConstraintBuilder::appendLinkEllipsoidConstraints(
                q_now,
                fk_now.link_poses,
                ctx->collision_ellipsoids,
                *ctx->jacobian_provider,
                obstacle_distance_query,
                qp_input.obstacle_constraints,
                &obstacle_error);
            if (generated <= 0) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "[%s] ReactiveTask obstacle constraints unavailable at planner tick %d: %s",
                    mapping.c_str(),
                    planner_tick,
                    obstacle_error.c_str());
                break;
            }
        }

        rq::ReactiveQpProblem problem;
        if (!rq::ReactiveQpBuilder::build(qp_input, reactive_cfg_.qp_build, problem, &error)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "[%s] ReactiveTask build QP failed at planner tick %d: %s",
                mapping.c_str(),
                planner_tick,
                error.c_str());
            break;
        }

        Eigen::VectorXd solution;
        if (!solver.solve(problem, solution, &error)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "[%s] ReactiveTask solve QP failed at planner tick %d: %s",
                mapping.c_str(),
                planner_tick,
                error.c_str());
            break;
        }

        const int dof = static_cast<int>(q_now.size());
        std::vector<double> qdot_cmd(static_cast<std::size_t>(dof), 0.0);
        Eigen::VectorXd qdot_eigen = Eigen::VectorXd::Zero(dof);
        for (int i = 0; i < dof; ++i) {
            qdot_cmd[static_cast<std::size_t>(i)] = solution(i);
            qdot_eigen(i) = solution(i);
        }

        const Eigen::VectorXd task_pred = J * qdot_eigen;
        const Eigen::VectorXd task_residual = task_pred - task_out.v_des;
        const double task_residual_norm = task_residual.norm();
        const double qdot_max_abs = qdot_eigen.cwiseAbs().maxCoeff();
        const double qdot_norm = qdot_eigen.norm();
        const bool qdot_limit_violation =
            ((qdot_eigen.array() < ctx->qd_min.array() - 1e-9) ||
             (qdot_eigen.array() > ctx->qd_max.array() + 1e-9))
                .any();
        const double joint_limit_margin_min = (q_now - ctx->joint_limits.q_min)
                                                  .cwiseMin(ctx->joint_limits.q_max - q_now)
                                                  .minCoeff();
        const double pos_err = (fk_now.ee_position - request.p_goal).norm();
        const double ori_err = orientationErrorRad(fk_now.ee_rotation, request.R_goal);

        if (q_prev_feedback_valid && q_prev_feedback.size() == q_now.size()) {
            const double joint_delta_max = (q_now - q_prev_feedback).cwiseAbs().maxCoeff();
            if (qdot_norm >= kWatchdogCmdNormMin && joint_delta_max < kWatchdogJointDeltaMax) {
                ++no_motion_cycles;
            } else {
                no_motion_cycles = 0;
            }
        } else {
            no_motion_cycles = 0;
        }
        q_prev_feedback = q_now;
        q_prev_feedback_valid = q_now.allFinite();

        bool has_progress = false;
        if (pos_err + kPosProgressEps < best_pos_err) {
            best_pos_err = pos_err;
            has_progress = true;
        }
        if (ori_err + kOriProgressEps < best_ori_err) {
            best_ori_err = ori_err;
            has_progress = true;
        }
        if (has_progress || qdot_norm < kWatchdogCmdNormMin) {
            no_progress_cycles = 0;
        } else {
            ++no_progress_cycles;
        }

        if (no_motion_cycles >= no_motion_cycle_limit) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] ReactiveTask watchdog stop: no joint motion for %d cycles while qdot_norm=%.4f (pos_err=%.5f ori_err=%.5f)",
                mapping.c_str(),
                no_motion_cycles,
                qdot_norm,
                pos_err,
                ori_err);
            break;
        }
        if (no_progress_cycles >= no_progress_cycle_limit) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] ReactiveTask watchdog stop: no goal progress for %d cycles (best_pos_err=%.5f best_ori_err=%.5f current_pos_err=%.5f current_ori_err=%.5f)",
                mapping.c_str(),
                no_progress_cycles,
                best_pos_err,
                best_ori_err,
                pos_err,
                ori_err);
            break;
        }

        if ((neo_iter % safety_log_stride) == 0) {
            double dummy_clearance = std::numeric_limits<double>::quiet_NaN();
            if (runtime_cfg_.enable_dummy_obstacle && runtime_cfg_.dummy_obstacle_radius > 0.0) {
                const Eigen::Vector3d center = (mapping == "right_arm")
                                                   ? runtime_cfg_.dummy_obstacle_center_right_arm
                                                   : runtime_cfg_.dummy_obstacle_center_left_arm;
                dummy_clearance = (fk_now.ee_position - center).norm() -
                                  runtime_cfg_.dummy_obstacle_radius - request.safe_distance;
            }
            RCLCPP_INFO(
                node_->get_logger(),
                "[%s] safety_tick: planner_tick=%d neo_iter=%d pos_err=%.5f ori_err=%.5f qdot_norm=%.5f qdot_max=%.5f qdot_limit_violation=%s joint_margin_min=%.5f task_residual_norm=%.6f dummy_clearance=%.5f",
                mapping.c_str(),
                planner_tick,
                neo_iter,
                pos_err,
                ori_err,
                qdot_norm,
                qdot_max_abs,
                qdot_limit_violation ? "true" : "false",
                joint_limit_margin_min,
                task_residual_norm,
                dummy_clearance);
            RCLCPP_INFO(
                node_->get_logger(),
                "[%s] safety_vectors: q_now=%s qdot=%s v_des=%s task_pred=%s task_residual=%s",
                mapping.c_str(),
                vecToStr(q_now).c_str(),
                vecToStr(qdot_eigen).c_str(),
                vecToStr(task_out.v_des).c_str(),
                vecToStr(task_pred).c_str(),
                vecToStr(task_residual).c_str());
        }

        if (!send_joint_velocities(mapping, qdot_cmd)) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] ReactiveTask abort: failed to send joint velocity command.",
                mapping.c_str());
            break;
        }

        if (pos_err <= runtime_cfg_.goal_position_tolerance &&
            ori_err <= runtime_cfg_.goal_orientation_tolerance_rad) {
            reached_goal = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(neo_tick_sec));
        ++neo_iter;
        planner_tick_accumulator += neo_tick_sec;

        while (planner_tick_accumulator + 1e-12 >= planner_tick_sec) {
            planner_tick_accumulator -= planner_tick_sec;
            ++planner_tick;

            if (!waiting_for_chunk_commit) {
                planner_time_sec = std::min(
                    planner_time_sec + planner_tick_sec,
                    replanner.activeSegmentTotalDurationSec());
                if (!replanner.sampleByElapsedTime(planner_time_sec, sample)) {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "[%s] ReactiveTask sample failed at planner tick %d",
                        mapping.c_str(),
                        planner_tick);
                    sample_ok = false;
                    break;
                }
                ++ticks_in_chunk;
                if (ticks_in_chunk >= chunk_ticks) {
                    waiting_for_chunk_commit = true;
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "[%s] chunk_pipeline: reached chunk boundary chunk=%d (ticks=%d), waiting pending commit",
                        mapping.c_str(),
                        chunk_index,
                        ticks_in_chunk);
                }
            }

            if (waiting_for_chunk_commit) {
                if (!pending_plan_inflight) {
                    launch_pending_chunk_plan();
                }
                if (pending_plan_inflight &&
                    pending_plan_future.valid() &&
                    pending_plan_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    const auto [plan_ok, plan_error] = pending_plan_future.get();
                    pending_plan_inflight = false;
                    if (!plan_ok) {
                        RCLCPP_WARN(
                            node_->get_logger(),
                            "[%s] chunk_pipeline: pending plan failed at planner tick %d: %s",
                            mapping.c_str(),
                            planner_tick,
                            plan_error.c_str());
                        launch_pending_chunk_plan();
                        continue;
                    }

                    std::string commit_error;
                    if (!replanner.commitPendingSegment(&commit_error)) {
                        RCLCPP_WARN(
                            node_->get_logger(),
                            "[%s] chunk_pipeline: pending commit failed at planner tick %d: %s",
                            mapping.c_str(),
                            planner_tick,
                            commit_error.c_str());
                        launch_pending_chunk_plan();
                        continue;
                    }

                    ++chunk_index;
                    ticks_in_chunk = 0;
                    waiting_for_chunk_commit = false;
                    planner_time_sec = 0.0;
                    planner_tick_accumulator = 0.0;
                    if (!replanner.sampleByElapsedTime(planner_time_sec, sample)) {
                        RCLCPP_WARN(
                            node_->get_logger(),
                            "[%s] ReactiveTask sample failed after chunk commit at planner tick %d",
                            mapping.c_str(),
                            planner_tick);
                        sample_ok = false;
                        break;
                    }
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "[%s] chunk_pipeline: committed pending chunk=%d",
                        mapping.c_str(),
                        chunk_index);
                    launch_pending_chunk_plan();
                }
            } else if (!pending_plan_inflight) {
                launch_pending_chunk_plan();
            }
        }
    }

    if (pending_plan_inflight && pending_plan_future.valid()) {
        try {
            (void)pending_plan_future.get();
        } catch (const std::exception&) {
            // no-op
        }
    }

    // soft stop
    send_joint_velocities(mapping, std::vector<double>(ctx->joint_names.size(), 0.0));
    last_execution_success_[mapping] = reached_goal;
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
            batch_velocities[i] = vel_deg;
            batch_positions[i] = (i < q_current.size()) ? (q_current[i] * 180.0 / M_PI) : 0.0;
            batch_efforts[i] = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;
        }

        return hardware_driver->send_realtime_mit_command(
            interface,
            batch_positions,
            batch_velocities,
            batch_efforts,
            batch_kps,
            batch_kds);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ReactiveTask send velocity exception: %s", mapping.c_str(), e.what());
        return false;
    }
}

bool ReactiveTaskController::execute(
    const std::string& mapping,
    const std::vector<double>& parameters) {
    if (parameters.size() != 7) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] ReactiveTask: expected 7 params [x y z qx qy qz qw], got %zu",
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
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "ReactiveTask", 10)) {
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
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::EXECUTING);
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
                RCLCPP_ERROR(node_->get_logger(), "[%s] ReactiveTask command exception: %s", mapping.c_str(), e.what());
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
