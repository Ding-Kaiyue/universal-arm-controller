#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include "algorithm/cartesian_path_planner/collision/whole_body_ellipsoid_pose_validator.hpp"
#include "algorithm/cartesian_path_planner/config/astar_config.hpp"
#include "algorithm/cartesian_path_planner/config/planner_common_config.hpp"
#include "algorithm/cartesian_path_planner/config/smoothing_config.hpp"
#include "algorithm/cartesian_path_planner/core/cartesian_path_planner.hpp"
#include "algorithm/cartesian_path_planner/map/dummy_distance_field.hpp"
#include "algorithm/cartesian_path_planner/map/obstacle_primitives.hpp"
#include "algorithm/cartesian_path_planner/replanning/replanner_manager.hpp"
#include "algorithm/sphere_model/link_sphere_model.hpp"
#include "arm_controller/kinematics/forward_kinematics.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/integration/tracik_adapter.hpp"

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace fs = std::filesystem;
namespace rq = arm_controller::algorithm::reactive_qp;
namespace sm = arm_controller::algorithm::sphere_model;
namespace tpi = trajectory_planning::infrastructure::integration;

namespace {

struct MappingContext {
    std::string mapping;
    std::string robot_type;
    std::vector<std::string> joint_names;
    std::vector<double> start_position;
    std::vector<double> q_current;
    Eigen::Vector3d p_current{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d R_current{Eigen::Matrix3d::Identity()};
};

struct AdapterBundle {
    std::shared_ptr<tpi::MoveItAdapter> moveit;
    std::shared_ptr<tpi::TracIKAdapter> tracik;
    std::string base_link;
    std::string tip_link;
};

struct PinocchioBundle {
    pinocchio::Model model;
    std::vector<int> q_indices;
    std::vector<int> v_indices;
    pinocchio::FrameIndex ee_frame{0};
    std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics> fk;
    std::shared_ptr<arm_controller::kinematics::PinocchioJacobianProvider> jacobian;
    std::vector<rq::LinkCollisionEllipsoid> link_ellipsoids;
};

struct DemoScene {
    Eigen::Vector3d map_min{Eigen::Vector3d::Zero()};
    Eigen::Vector3d map_max{Eigen::Vector3d::Zero()};
    std::shared_ptr<cp::DummyDistanceField> map;
    cp::SphereObstacle obstacle;
    cp::PathPlanningInput request;
};

struct RunArtifacts {
    std::vector<std::vector<Eigen::Vector3d>> replanned_points;
    std::vector<std::vector<Eigen::Matrix3d>> replanned_orientations;
    std::vector<std::vector<Eigen::Vector3d>> raw_astar_points;
    std::vector<std::vector<Eigen::Matrix3d>> raw_astar_orientations;
    struct IkEllipsoidSnapshot {
        std::string link_name;
        std::string debug_name;
        Eigen::Vector3d center_world{Eigen::Vector3d::Zero()};
        Eigen::Vector3d radii{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d rotation_world{Eigen::Matrix3d::Identity()};
        double margin{0.0};
    };
    struct IkRecord {
        std::string label;
        int tick{0};
        int waypoint_index{-1};
        bool ik_ok{false};
        bool external_ik_ok{false};
        bool fallback_ik_used{false};
        bool collision_free{false};
        double min_margin{0.0};
        std::string reason;
        std::string worst_link_name;
        Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d target_orientation{Eigen::Matrix3d::Identity()};
        std::vector<double> q_solution;
        std::vector<IkEllipsoidSnapshot> ellipsoids;
    };
    std::vector<IkRecord> ik_records;
};

struct ExampleRuntimeConfig {
    cp::PlannerCommonConfig planner_common;
    cp::AStarConfig planner_astar;
    cp::SmoothingConfig planner_smoothing;
    cp::ReplannerConfig replanner;
    double request_safe_distance{0.03};
    double request_goal_tolerance{0.03};
    bool whole_body_postcheck_non_blocking{false};
    int whole_body_postcheck_max_attempts{5};
    double whole_body_retry_forbidden_radius{0.045};
    Eigen::Vector3d map_margin_xyz{0.60, 0.60, 0.60};
    int max_control_ticks{60};
};

bool parseVec3(const YAML::Node& node, Eigen::Vector3d& out) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        return false;
    }
    out << node[0].as<double>(), node[1].as<double>(), node[2].as<double>();
    return true;
}

bool loadRuntimeConfig(const std::string& yaml_path, ExampleRuntimeConfig& cfg, std::string* error) {
    try {
        // Defaults aligned with ReactiveTaskController fallback.
        cfg.replanner.segment_sample_step_m = 0.04;
        cfg.replanner.replan_every_control_ticks = 10;
        cfg.replanner.control_cycle_sec = 0.10;
        cfg.replanner.prediction_horizon_ticks = 10;
        cfg.replanner.planning_latency_sec = 0.06;
        cfg.replanner.handoff_blend_points = 8;

        cfg.planner_common.default_segment_speed = 0.20;
        cfg.planner_common.enable_interpolator_smoothing = true;
        cfg.planner_common.interpolator_continuity_order = 2;
        cfg.planner_common.interpolator_target_dt = 0.01;

        cfg.planner_astar.voxel_resolution = 0.015;
        cfg.planner_astar.neighbor_mode = 18;
        cfg.planner_astar.use_se3_search = true;
        cfg.planner_astar.orientation_bin_size_rad = 0.7853981634;
        cfg.planner_astar.orientation_goal_tolerance_rad = 1.20;
        cfg.planner_astar.enable_inplace_rotation_neighbors = false;
        cfg.planner_astar.force_axis_translation_neighbors_in_se3 = true;
        cfg.planner_astar.obstacle_penalty_weight = 1.2;
        cfg.planner_astar.corridor_deviation_weight = 3.0;
        cfg.planner_astar.goal_shortcut_clearance_margin = 0.02;
        cfg.planner_astar.orientation_cost_weight = 0.08;
        cfg.planner_astar.orientation_heuristic_weight = 0.20;
        cfg.planner_astar.max_iterations = 3000000;
        cfg.planner_astar.max_planning_time_sec = 12.0;
        cfg.planner_astar.edge_check_step = 0.005;

        cfg.planner_smoothing.max_shortcut_trials = 50;
        cfg.planner_smoothing.collision_check_step = 0.005;
        cfg.planner_smoothing.local_adjust_iterations = 15;
        cfg.planner_smoothing.local_adjust_alpha = 0.30;

        cfg.request_safe_distance = 0.03;
        cfg.request_goal_tolerance = 0.03;
        cfg.whole_body_postcheck_non_blocking = false;
        cfg.whole_body_postcheck_max_attempts = 5;
        cfg.whole_body_retry_forbidden_radius = 0.045;
        cfg.map_margin_xyz = Eigen::Vector3d(0.60, 0.60, 0.60);
        cfg.max_control_ticks = 60;

        const YAML::Node root = YAML::LoadFile(yaml_path);
        const YAML::Node rtc = root["reactive_task_controller"];
        if (!rtc || !rtc.IsMap()) {
            return true;
        }

        if (rtc["max_control_ticks"]) {
            cfg.max_control_ticks = std::max(1, rtc["max_control_ticks"].as<int>());
        }

        if (const YAML::Node req = rtc["request"]; req && req.IsMap()) {
            if (req["safe_distance"]) {
                cfg.request_safe_distance = std::max(0.0, req["safe_distance"].as<double>());
            }
            if (req["goal_tolerance"]) {
                cfg.request_goal_tolerance = std::max(1e-6, req["goal_tolerance"].as<double>());
            } else if (rtc["goal_position_tolerance"]) {
                cfg.request_goal_tolerance = std::max(1e-6, rtc["goal_position_tolerance"].as<double>());
            }
            if (req["whole_body_postcheck_non_blocking"]) {
                cfg.whole_body_postcheck_non_blocking = req["whole_body_postcheck_non_blocking"].as<bool>();
            }
            if (req["whole_body_postcheck_max_attempts"]) {
                cfg.whole_body_postcheck_max_attempts =
                    std::max(1, req["whole_body_postcheck_max_attempts"].as<int>());
            }
            if (req["whole_body_retry_forbidden_radius"]) {
                cfg.whole_body_retry_forbidden_radius =
                    std::max(0.0, req["whole_body_retry_forbidden_radius"].as<double>());
            }
            Eigen::Vector3d map_margin;
            if (parseVec3(req["map_margin_xyz"], map_margin)) {
                cfg.map_margin_xyz = map_margin.cwiseMax(Eigen::Vector3d::Zero());
            }
        } else if (rtc["goal_position_tolerance"]) {
            cfg.request_goal_tolerance = std::max(1e-6, rtc["goal_position_tolerance"].as<double>());
        }

        if (const YAML::Node rep = rtc["replanner"]; rep && rep.IsMap()) {
            if (rep["segment_sample_step_m"]) cfg.replanner.segment_sample_step_m = std::max(1e-4, rep["segment_sample_step_m"].as<double>());
            if (rep["replan_every_control_ticks"]) cfg.replanner.replan_every_control_ticks = std::max(1, rep["replan_every_control_ticks"].as<int>());
            if (rep["control_cycle_sec"]) cfg.replanner.control_cycle_sec = std::max(1e-4, rep["control_cycle_sec"].as<double>());
            if (rep["prediction_horizon_ticks"]) cfg.replanner.prediction_horizon_ticks = std::max(1, rep["prediction_horizon_ticks"].as<int>());
            if (rep["planning_latency_sec"]) cfg.replanner.planning_latency_sec = std::max(0.0, rep["planning_latency_sec"].as<double>());
            if (rep["handoff_blend_points"]) cfg.replanner.handoff_blend_points = std::max(1, rep["handoff_blend_points"].as<int>());
        }

        if (const YAML::Node planner = rtc["planner"]; planner && planner.IsMap()) {
            if (const YAML::Node common = planner["common"]; common && common.IsMap()) {
                if (common["default_segment_speed"]) cfg.planner_common.default_segment_speed = common["default_segment_speed"].as<double>();
                if (common["enable_interpolator_smoothing"]) cfg.planner_common.enable_interpolator_smoothing = common["enable_interpolator_smoothing"].as<bool>();
                if (common["interpolator_continuity_order"]) cfg.planner_common.interpolator_continuity_order = common["interpolator_continuity_order"].as<int>();
                if (common["interpolator_target_dt"]) cfg.planner_common.interpolator_target_dt = common["interpolator_target_dt"].as<double>();
            }
            if (const YAML::Node astar = planner["astar"]; astar && astar.IsMap()) {
                if (astar["voxel_resolution"]) cfg.planner_astar.voxel_resolution = astar["voxel_resolution"].as<double>();
                if (astar["neighbor_mode"]) cfg.planner_astar.neighbor_mode = astar["neighbor_mode"].as<int>();
                if (astar["use_se3_search"]) cfg.planner_astar.use_se3_search = astar["use_se3_search"].as<bool>();
                if (astar["orientation_bin_size_rad"]) cfg.planner_astar.orientation_bin_size_rad = astar["orientation_bin_size_rad"].as<double>();
                if (astar["orientation_goal_tolerance_rad"]) cfg.planner_astar.orientation_goal_tolerance_rad = astar["orientation_goal_tolerance_rad"].as<double>();
                if (astar["enable_inplace_rotation_neighbors"]) cfg.planner_astar.enable_inplace_rotation_neighbors = astar["enable_inplace_rotation_neighbors"].as<bool>();
                if (astar["force_axis_translation_neighbors_in_se3"]) cfg.planner_astar.force_axis_translation_neighbors_in_se3 = astar["force_axis_translation_neighbors_in_se3"].as<bool>();
                if (astar["obstacle_penalty_weight"]) cfg.planner_astar.obstacle_penalty_weight = astar["obstacle_penalty_weight"].as<double>();
                if (astar["corridor_deviation_weight"]) cfg.planner_astar.corridor_deviation_weight = astar["corridor_deviation_weight"].as<double>();
                if (astar["goal_shortcut_clearance_margin"]) cfg.planner_astar.goal_shortcut_clearance_margin = astar["goal_shortcut_clearance_margin"].as<double>();
                if (astar["orientation_cost_weight"]) cfg.planner_astar.orientation_cost_weight = astar["orientation_cost_weight"].as<double>();
                if (astar["orientation_heuristic_weight"]) cfg.planner_astar.orientation_heuristic_weight = astar["orientation_heuristic_weight"].as<double>();
                if (astar["max_iterations"]) cfg.planner_astar.max_iterations = astar["max_iterations"].as<int>();
                if (astar["max_planning_time_sec"]) cfg.planner_astar.max_planning_time_sec = astar["max_planning_time_sec"].as<double>();
                if (astar["edge_check_step"]) cfg.planner_astar.edge_check_step = astar["edge_check_step"].as<double>();
            }
            if (const YAML::Node smoothing = planner["smoothing"]; smoothing && smoothing.IsMap()) {
                if (smoothing["max_shortcut_trials"]) cfg.planner_smoothing.max_shortcut_trials = smoothing["max_shortcut_trials"].as<int>();
                if (smoothing["collision_check_step"]) cfg.planner_smoothing.collision_check_step = smoothing["collision_check_step"].as<double>();
                if (smoothing["local_adjust_iterations"]) cfg.planner_smoothing.local_adjust_iterations = smoothing["local_adjust_iterations"].as<int>();
                if (smoothing["local_adjust_alpha"]) cfg.planner_smoothing.local_adjust_alpha = smoothing["local_adjust_alpha"].as<double>();
            }
        }

        return true;
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = e.what();
        }
        return false;
    }
}

double effectiveEllipsoidRadiusAlongNormal(
    const Eigen::Matrix3d& R_world_link,
    const Eigen::Vector3d& radii_link,
    const Eigen::Vector3d& n_world) {
    const Eigen::Vector3d n_link = R_world_link.transpose() * n_world;
    const double x = radii_link.x() * n_link.x();
    const double y = radii_link.y() * n_link.y();
    const double z = radii_link.z() * n_link.z();
    const double v = x * x + y * y + z * z;
    return (v > 0.0) ? std::sqrt(v) : 0.0;
}

bool loadMappingConfig(
    const std::string& hardware_cfg_path,
    const std::string& mapping,
    MappingContext& ctx,
    std::string* error) {
    try {
        const YAML::Node root = YAML::LoadFile(hardware_cfg_path);
        const YAML::Node hardware = root["hardware"];
        const YAML::Node item = hardware ? hardware[mapping] : YAML::Node();
        if (!item || !item.IsMap()) {
            if (error != nullptr) {
                *error = "mapping '" + mapping + "' not found in hardware_config.yaml";
            }
            return false;
        }

        ctx.mapping = mapping;
        ctx.robot_type = item["robot_type"] ? item["robot_type"].as<std::string>() : "";
        ctx.joint_names.clear();
        ctx.start_position.clear();

        if (item["joint_names"] && item["joint_names"].IsSequence()) {
            for (const auto& value : item["joint_names"]) {
                ctx.joint_names.push_back(value.as<std::string>());
            }
        }
        if (item["start_position"] && item["start_position"].IsSequence()) {
            for (const auto& value : item["start_position"]) {
                ctx.start_position.push_back(value.as<double>());
            }
        }

        if (ctx.robot_type.empty() || ctx.joint_names.empty()) {
            if (error != nullptr) {
                *error = "mapping configuration incomplete for '" + mapping + "'";
            }
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = e.what();
        }
        return false;
    }
}

bool waitForCurrentJointPositions(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<std::string>& joint_names,
    std::vector<double>& out_q,
    double timeout_sec = 5.0) {
    std::mutex msg_mutex;
    sensor_msgs::msg::JointState::SharedPtr latest_msg;

    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [&](sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(msg_mutex);
            latest_msg = std::move(msg);
        });

    const auto t0 = std::chrono::steady_clock::now();
    rclcpp::Rate rate(200.0);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        sensor_msgs::msg::JointState::SharedPtr msg_copy;
        {
            std::lock_guard<std::mutex> lock(msg_mutex);
            msg_copy = latest_msg;
        }

        if (msg_copy && msg_copy->name.size() == msg_copy->position.size()) {
            std::unordered_map<std::string, std::size_t> name_to_idx;
            for (std::size_t i = 0; i < msg_copy->name.size(); ++i) {
                name_to_idx[msg_copy->name[i]] = i;
            }

            std::vector<double> q;
            q.reserve(joint_names.size());
            bool ok = true;
            for (const auto& joint_name : joint_names) {
                const auto it = name_to_idx.find(joint_name);
                if (it == name_to_idx.end()) {
                    ok = false;
                    break;
                }
                q.push_back(msg_copy->position[it->second]);
            }
            if (ok) {
                out_q = std::move(q);
                (void)sub;
                return true;
            }
        }

        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (elapsed > timeout_sec) {
            break;
        }
        rate.sleep();
    }

    (void)sub;
    return false;
}

bool createAdapters(
    const rclcpp::Node::SharedPtr& node,
    const std::string& mapping,
    AdapterBundle& adapters,
    std::string* error) {
    try {
        adapters.moveit = std::make_shared<tpi::MoveItAdapter>(node, mapping, "movel");
        adapters.tracik = std::make_shared<tpi::TracIKAdapter>(node, mapping);
        adapters.base_link = adapters.moveit->getBaseLink();
        adapters.tip_link = adapters.moveit->getEndEffectorLink();
        return true;
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = e.what();
        }
        return false;
    }
}

bool resolveCurrentPose(
    const rclcpp::Node::SharedPtr& node,
    const std::string& urdf_path,
    const std::string& preferred_tip_link,
    MappingContext& ctx,
    std::string* error) {
    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(urdf_path, model);
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = std::string("buildModel failed: ") + e.what();
        }
        return false;
    }

    std::vector<int> q_indices;
    q_indices.reserve(ctx.joint_names.size());
    for (const auto& joint_name : ctx.joint_names) {
        if (!model.existJointName(joint_name)) {
            if (error != nullptr) {
                *error = "joint not in URDF: " + joint_name;
            }
            return false;
        }
        const pinocchio::JointIndex joint_id = model.getJointId(joint_name);
        q_indices.push_back(static_cast<int>(model.joints[joint_id].idx_q()));
    }

    pinocchio::FrameIndex ee_frame = 0;
    if (!preferred_tip_link.empty() && model.existFrame(preferred_tip_link)) {
        ee_frame = model.getFrameId(preferred_tip_link);
    } else {
        const pinocchio::JointIndex last_joint = model.getJointId(ctx.joint_names.back());
        for (pinocchio::FrameIndex fid = 0; fid < model.frames.size(); ++fid) {
            if (model.frames[fid].parentJoint == last_joint) {
                ee_frame = fid;
            }
        }
    }

    arm_controller::kinematics::PinocchioForwardKinematics fk(node, model, q_indices, ee_frame);
    if (!fk.initialize()) {
        if (error != nullptr) {
            *error = "PinocchioForwardKinematics initialize failed";
        }
        return false;
    }

    std::vector<double> q_current;
    const bool got_joint_state =
        waitForCurrentJointPositions(node, ctx.joint_names, q_current, 5.0);
    if (!got_joint_state) {
        q_current = ctx.start_position;
    }
    if (q_current.size() != ctx.joint_names.size()) {
        q_current.assign(ctx.joint_names.size(), 0.0);
    }
    ctx.q_current = q_current;

    const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
        q_current.data(), static_cast<Eigen::Index>(q_current.size()));
    arm_controller::kinematics::ForwardKinematicsOutput fk_out;
    if (!fk.compute(q, fk_out)) {
        if (error != nullptr) {
            *error = "FK compute failed";
        }
        return false;
    }

    ctx.p_current = fk_out.ee_position;
    ctx.R_current = fk_out.ee_rotation;

    std::cout << "[replanner_minimal] mapping=" << ctx.mapping
              << " joint_source=" << (got_joint_state ? "/joint_states" : "start_position_or_zero")
              << " q=[";
    for (std::size_t i = 0; i < q_current.size(); ++i) {
        std::cout << q_current[i];
        if (i + 1 != q_current.size()) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
    std::cout << "[replanner_minimal] start_pose p=("
              << ctx.p_current.x() << ", " << ctx.p_current.y() << ", " << ctx.p_current.z()
              << ")" << std::endl;
    return true;
}

bool buildPinocchioBundle(
    const rclcpp::Node::SharedPtr& node,
    const std::string& hardware_cfg_path,
    const std::string& urdf_path,
    const std::string& tip_link,
    const MappingContext& ctx,
    PinocchioBundle& bundle,
    std::string* error) {
    try {
        pinocchio::urdf::buildModel(urdf_path, bundle.model);
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = std::string("buildModel failed: ") + e.what();
        }
        return false;
    }

    bundle.q_indices.clear();
    bundle.v_indices.clear();
    for (const auto& joint_name : ctx.joint_names) {
        if (!bundle.model.existJointName(joint_name)) {
            if (error != nullptr) {
                *error = "joint not found in model: " + joint_name;
            }
            return false;
        }
        const pinocchio::JointIndex joint_id = bundle.model.getJointId(joint_name);
        bundle.q_indices.push_back(static_cast<int>(bundle.model.joints[joint_id].idx_q()));
        bundle.v_indices.push_back(static_cast<int>(bundle.model.joints[joint_id].idx_v()));
    }

    if (!tip_link.empty() && bundle.model.existFrame(tip_link)) {
        bundle.ee_frame = bundle.model.getFrameId(tip_link);
    } else {
        const pinocchio::JointIndex last_joint = bundle.model.getJointId(ctx.joint_names.back());
        for (pinocchio::FrameIndex fid = 0; fid < bundle.model.frames.size(); ++fid) {
            if (bundle.model.frames[fid].parentJoint == last_joint) {
                bundle.ee_frame = fid;
            }
        }
    }

    bundle.fk = std::make_shared<arm_controller::kinematics::PinocchioForwardKinematics>(
        node, bundle.model, bundle.q_indices, bundle.ee_frame);
    if (!bundle.fk->initialize()) {
        if (error != nullptr) {
            *error = "FK provider init failed";
        }
        return false;
    }

    bundle.jacobian = std::make_shared<arm_controller::kinematics::PinocchioJacobianProvider>(
        node, bundle.model, bundle.q_indices, bundle.v_indices, bundle.ee_frame);
    if (!bundle.jacobian->initialize()) {
        if (error != nullptr) {
            *error = "Jacobian provider init failed";
        }
        return false;
    }

    bundle.link_ellipsoids.clear();
    if (!sm::LinkSphereModel::buildEllipsoidsForMapping(
            hardware_cfg_path, ctx.mapping, bundle.model, bundle.link_ellipsoids, error)) {
        return false;
    }
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

bool initializeTracIk(
    const MappingContext& ctx,
    const std::string& urdf_path,
    const AdapterBundle& adapters,
    std::string* error) {
    adapters.tracik->setMoveItAdapter(adapters.moveit.get());

    std::string urdf_xml = adapters.moveit->getURDFString(ctx.robot_type);
    if (urdf_xml.empty()) {
        std::ifstream ifs(urdf_path);
        urdf_xml.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    }

    if (!adapters.tracik->initializeKDLChain(urdf_xml, adapters.base_link, adapters.tip_link)) {
        if (error != nullptr) {
            *error = "TRAC-IK initializeKDLChain failed";
        }
        return false;
    }
    if (!adapters.tracik->initializeSolver(normalizeArmTypeForTracIk(ctx.robot_type))) {
        if (error != nullptr) {
            *error = "TRAC-IK initializeSolver failed";
        }
        return false;
    }
    return true;
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

cp::WholeBodyEllipsoidPoseValidator buildWholeBodyValidator(
    const MappingContext& ctx,
    const AdapterBundle& adapters,
    const std::shared_ptr<cp::DummyDistanceField>& map,
    const PinocchioBundle& pinocchio) {
    cp::WholeBodyEllipsoidPoseValidator::Config cfg;
    cfg.ik_max_iterations = 25;
    cfg.ik_pos_tolerance_m = 0.01;
    cfg.ik_rot_tolerance_rad = 0.15;
    cfg.ik_damping = 0.05;
    cfg.ik_step_scale = 0.6;
    cfg.segment_substeps_min = 1;
    cfg.default_q_seed = Eigen::Map<const Eigen::VectorXd>(
        ctx.q_current.data(), static_cast<Eigen::Index>(ctx.q_current.size()));
    cfg.ik_solver_fn =
        [moveit = adapters.moveit, tracik = adapters.tracik, seed_default = ctx.q_current](
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
            const geometry_msgs::msg::Pose pose_base = moveit->worldPoseToBaseLinkPose(pose_world);

            std::vector<double> q_solution_vec;
            if (!tracik->computeIKClosest(pose_base, seed, q_solution_vec, 5, false)) {
                return false;
            }

            q_solution = Eigen::Map<const Eigen::VectorXd>(
                q_solution_vec.data(), static_cast<Eigen::Index>(q_solution_vec.size()));
            return true;
        };

    return cp::WholeBodyEllipsoidPoseValidator(
        cfg, map, pinocchio.fk, pinocchio.jacobian, pinocchio.link_ellipsoids);
}

Eigen::Vector3d fixedGoalPositionForMapping(const std::string& mapping) {
    if (mapping == "right_arm") {
        return Eigen::Vector3d(0.19, 0.5, 0.63);
    }
    return Eigen::Vector3d(0.13, -0.5, 0.63);
}

Eigen::Matrix3d fixedGoalOrientation() {
    const double roll = -1.3963928052407684;
    const double pitch = 0.0;
    const double yaw = -1.5707963267948963;
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

DemoScene buildDemoScene(const MappingContext& ctx, const ExampleRuntimeConfig& cfg) {
    DemoScene scene;
    scene.request.p_start = ctx.p_current;
    scene.request.R_start = ctx.R_current;
    scene.request.p_goal = fixedGoalPositionForMapping(ctx.mapping);
    scene.request.R_goal = fixedGoalOrientation();
    scene.request.q_start_seed = Eigen::Map<const Eigen::VectorXd>(
        ctx.q_current.data(), static_cast<Eigen::Index>(ctx.q_current.size()));
    scene.request.safe_distance = cfg.request_safe_distance;
    scene.request.goal_tolerance = cfg.request_goal_tolerance;
    scene.request.whole_body_postcheck_non_blocking = cfg.whole_body_postcheck_non_blocking;
    scene.request.whole_body_postcheck_max_attempts = cfg.whole_body_postcheck_max_attempts;
    scene.request.whole_body_retry_forbidden_radius = cfg.whole_body_retry_forbidden_radius;

    const Eigen::Vector3d min_corner = scene.request.p_start.cwiseMin(scene.request.p_goal);
    const Eigen::Vector3d max_corner = scene.request.p_start.cwiseMax(scene.request.p_goal);
    const Eigen::Vector3d map_margin = cfg.map_margin_xyz;
    scene.map_min = min_corner - map_margin;
    scene.map_max = max_corner + map_margin;
    scene.map = std::make_shared<cp::DummyDistanceField>(scene.map_min, scene.map_max);

    if (ctx.mapping == "right_arm") {
        scene.obstacle.center = Eigen::Vector3d(0.07, 0.52, 0.72);
    } else {
        scene.obstacle.center = Eigen::Vector3d(0.25, -0.52, 0.6);
    }
    scene.obstacle.radius = 0.035;
    scene.map->addSphere(scene.obstacle);
    return scene;
}

std::shared_ptr<cp::CartesianPathPlanner> buildPlanner(
    const std::shared_ptr<cp::DummyDistanceField>& map,
    const Eigen::Vector3d& map_min,
    const ExampleRuntimeConfig& cfg) {
    return std::make_shared<cp::CartesianPathPlanner>(
        cfg.planner_common, cfg.planner_astar, cfg.planner_smoothing, map, map_min);
}

std::vector<Eigen::Vector3d> sampleActiveSegmentPoints(const cp::ReplannerManager& replanner) {
    std::vector<Eigen::Vector3d> points;
    const int count = replanner.activeSegmentPointCount();
    points.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        cp::TimedCartesianSample sample;
        if (!replanner.sample(i, sample)) {
            break;
        }
        points.push_back(sample.T_target.translation());
    }
    return points;
}

std::vector<Eigen::Matrix3d> sampleActiveSegmentOrientations(const cp::ReplannerManager& replanner) {
    std::vector<Eigen::Matrix3d> orientations;
    const int count = replanner.activeSegmentPointCount();
    orientations.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        cp::TimedCartesianSample sample;
        if (!replanner.sample(i, sample)) {
            break;
        }
        orientations.push_back(sample.T_target.linear());
    }
    return orientations;
}

std::vector<Eigen::Vector3d> pathToPoints(const cp::CartesianPath& path) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(path.waypoints.size());
    for (const auto& waypoint : path.waypoints) {
        points.push_back(waypoint.position);
    }
    return points;
}

std::vector<Eigen::Matrix3d> pathToOrientations(const cp::CartesianPath& path) {
    std::vector<Eigen::Matrix3d> orientations;
    orientations.reserve(path.waypoints.size());
    for (const auto& waypoint : path.waypoints) {
        orientations.push_back(waypoint.orientation);
    }
    return orientations;
}

Eigen::Vector3d rotationToRpyDeg(const Eigen::Matrix3d& R) {
    return R.eulerAngles(0, 1, 2) * (180.0 / M_PI);
}

double relativeRotationDeg(const Eigen::Matrix3d& R_prev, const Eigen::Matrix3d& R_cur) {
    Eigen::AngleAxisd aa(R_prev.transpose() * R_cur);
    return std::abs(aa.angle()) * (180.0 / M_PI);
}

std::string getVizDir() {
    const char* env = std::getenv("REPLANNER_TEST_VIZ_DIR");
    if (env == nullptr || std::string(env).empty()) {
        return "";
    }
    return std::string(env);
}

void appendIkRecord(
    const PinocchioBundle& pinocchio,
    const cp::SphereObstacle& obstacle,
    const double safe_distance,
    const std::string& label,
    const int tick,
    const int waypoint_index,
    const Eigen::Vector3d& p_target,
    const Eigen::Matrix3d& R_target,
    const cp::PathPlanningInput::WholeBodyPoseDiagnostic& diag,
    RunArtifacts& artifacts) {
    RunArtifacts::IkRecord record;
    record.label = label;
    record.tick = tick;
    record.waypoint_index = waypoint_index;
    record.ik_ok = diag.ik_ok;
    record.external_ik_ok = diag.external_ik_ok;
    record.fallback_ik_used = diag.fallback_ik_used;
    record.collision_free = diag.collision_free;
    record.min_margin = diag.min_margin;
    record.reason = diag.reason;
    record.worst_link_name = diag.worst_link_name;
    record.target_position = p_target;
    record.target_orientation = R_target;
    record.q_solution.assign(diag.q_solution.data(), diag.q_solution.data() + diag.q_solution.size());

    if (!diag.ik_ok || diag.q_solution.size() <= 0 || !pinocchio.fk) {
        artifacts.ik_records.push_back(std::move(record));
        return;
    }

    arm_controller::kinematics::ForwardKinematicsOutput fk_out;
    if (!pinocchio.fk->compute(diag.q_solution, fk_out)) {
        artifacts.ik_records.push_back(std::move(record));
        return;
    }

    for (const auto& ellipsoid : pinocchio.link_ellipsoids) {
        const auto it = fk_out.link_poses.find(ellipsoid.link_name);
        if (it == fk_out.link_poses.end()) {
            continue;
        }

        RunArtifacts::IkEllipsoidSnapshot snapshot;
        snapshot.link_name = ellipsoid.link_name;
        snapshot.debug_name = ellipsoid.debug_name;
        snapshot.radii = ellipsoid.radii;
        snapshot.center_world = it->second * ellipsoid.center_in_link;
        snapshot.rotation_world = it->second.linear();

        Eigen::Vector3d n_world = snapshot.center_world - obstacle.center;
        const double n_norm = n_world.norm();
        if (n_norm > 1e-9) {
            n_world /= n_norm;
        } else {
            n_world = Eigen::Vector3d::UnitX();
        }
        const double r_eff = effectiveEllipsoidRadiusAlongNormal(
            snapshot.rotation_world, snapshot.radii, n_world);
        snapshot.margin =
            (snapshot.center_world - obstacle.center).norm() - obstacle.radius - safe_distance - r_eff;
        record.ellipsoids.push_back(std::move(snapshot));
    }

    artifacts.ik_records.push_back(std::move(record));
}

void appendPostcheckFailureRecord(
    const PinocchioBundle& pinocchio,
    const DemoScene& scene,
    const cp::PathPlanningInput::WholeBodyPostcheckFailureEvent& event,
    RunArtifacts& artifacts) {
    std::ostringstream label;
    label << "postcheck_fail_attempt_" << event.attempt_index;
    appendIkRecord(
        pinocchio,
        scene.obstacle,
        scene.request.safe_distance,
        label.str(),
        0,
        event.waypoint_index,
        event.position,
        event.orientation,
        event.diagnostic,
        artifacts);
}

void appendSeedRecord(
    const PinocchioBundle& pinocchio,
    const cp::SphereObstacle& obstacle,
    const double safe_distance,
    const double blocking_margin,
    const std::string& label,
    const int tick,
    const int waypoint_index,
    const Eigen::Vector3d& p_target,
    const Eigen::Matrix3d& R_target,
    const Eigen::VectorXd& q_seed,
    RunArtifacts& artifacts) {
    RunArtifacts::IkRecord record;
    record.label = label;
    record.tick = tick;
    record.waypoint_index = waypoint_index;
    record.ik_ok = true;
    record.external_ik_ok = true;
    record.fallback_ik_used = false;
    record.collision_free = true;
    record.min_margin = std::numeric_limits<double>::infinity();
    record.reason = "seed_fk";
    record.target_position = p_target;
    record.target_orientation = R_target;
    record.q_solution.assign(q_seed.data(), q_seed.data() + q_seed.size());

    if (!pinocchio.fk) {
        record.ik_ok = false;
        record.collision_free = false;
        record.min_margin = -1.0;
        record.reason = "fk_unavailable";
        record.worst_link_name = "fk_unavailable";
        artifacts.ik_records.push_back(std::move(record));
        return;
    }

    arm_controller::kinematics::ForwardKinematicsOutput fk_out;
    if (!pinocchio.fk->compute(q_seed, fk_out)) {
        record.ik_ok = false;
        record.collision_free = false;
        record.min_margin = -1.0;
        record.reason = "fk_fail";
        record.worst_link_name = "fk_fail";
        artifacts.ik_records.push_back(std::move(record));
        return;
    }

    for (const auto& ellipsoid : pinocchio.link_ellipsoids) {
        const auto it = fk_out.link_poses.find(ellipsoid.link_name);
        if (it == fk_out.link_poses.end()) {
            continue;
        }

        RunArtifacts::IkEllipsoidSnapshot snapshot;
        snapshot.link_name = ellipsoid.link_name;
        snapshot.debug_name = ellipsoid.debug_name;
        snapshot.radii = ellipsoid.radii;
        snapshot.center_world = it->second * ellipsoid.center_in_link;
        snapshot.rotation_world = it->second.linear();

        Eigen::Vector3d n_world = snapshot.center_world - obstacle.center;
        const double n_norm = n_world.norm();
        if (n_norm > 1e-9) {
            n_world /= n_norm;
        } else {
            n_world = Eigen::Vector3d::UnitX();
        }
        const double r_eff = effectiveEllipsoidRadiusAlongNormal(
            snapshot.rotation_world, snapshot.radii, n_world);
        snapshot.margin =
            (snapshot.center_world - obstacle.center).norm() - obstacle.radius - safe_distance - r_eff;
        if (snapshot.margin < record.min_margin) {
            record.min_margin = snapshot.margin;
            record.worst_link_name = snapshot.debug_name;
        }
        if (snapshot.margin < blocking_margin) {
            record.collision_free = false;
        }
        record.ellipsoids.push_back(std::move(snapshot));
    }

    if (!std::isfinite(record.min_margin)) {
        record.min_margin = -1.0;
    }
    if (record.reason == "seed_fk") {
        record.reason = record.collision_free ? "seed_fk" : "collision_fail";
    }
    artifacts.ik_records.push_back(std::move(record));
}

void captureIkForWaypoints(
    const cp::PathPlanningInput& request,
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::Matrix3d>& orientations,
    const PinocchioBundle& pinocchio,
    const DemoScene& scene,
    const std::string& label_prefix,
    const int tick,
    const bool include_start_pose,
    RunArtifacts& artifacts) {
    if (!request.whole_body_pose_diagnostic) {
        return;
    }
    if (points.size() != orientations.size() || points.empty()) {
        return;
    }

    std::optional<Eigen::VectorXd> q_prev = request.q_start_seed;

    if (include_start_pose && request.q_start_seed.has_value() && request.q_start_seed->size() > 0) {
        appendSeedRecord(
            pinocchio,
            scene.obstacle,
            request.safe_distance,
            -request.safe_distance,
            label_prefix + "_handoff",
            tick,
            0,
            request.p_start,
            request.R_start,
            *request.q_start_seed,
            artifacts);
    }

    for (std::size_t i = 1; i < points.size(); ++i) {
        const auto diag = request.whole_body_pose_diagnostic(
            points[i], orientations[i], request.safe_distance, q_prev);
        appendIkRecord(
            pinocchio,
            scene.obstacle,
            request.safe_distance,
            label_prefix + "_waypoint",
            tick,
            static_cast<int>(i),
            points[i],
            orientations[i],
            diag,
            artifacts);
        if (diag.ik_ok && diag.q_solution.size() > 0) {
            q_prev = diag.q_solution;
        }
    }
}

void exportSegments(
    std::ofstream& ofs,
    const char* key,
    const std::vector<std::vector<Eigen::Vector3d>>& points,
    const std::vector<std::vector<Eigen::Matrix3d>>& orientations) {
    ofs << "  \"" << key << "\": [\n";
    for (std::size_t seg = 0; seg < points.size(); ++seg) {
        ofs << "    {\"id\": " << seg << ", \"points\": [\n";
        for (std::size_t i = 0; i < points[seg].size(); ++i) {
            const auto& p = points[seg][i];
            ofs << "      [" << p.x() << ", " << p.y() << ", " << p.z() << "]";
            ofs << (i + 1 == points[seg].size() ? "\n" : ",\n");
        }
        ofs << "    ], \"orientations\": [\n";
        for (std::size_t i = 0; i < orientations[seg].size(); ++i) {
            const auto& R = orientations[seg][i];
            ofs << "      ["
                << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2) << ", "
                << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2) << ", "
                << R(2, 0) << ", " << R(2, 1) << ", " << R(2, 2) << "]";
            ofs << (i + 1 == orientations[seg].size() ? "\n" : ",\n");
        }
        ofs << "    ]}";
        ofs << (seg + 1 == points.size() ? "\n" : ",\n");
    }
    ofs << "  ]";
}

void exportRunJson(
    const std::string& mapping,
    const DemoScene& scene,
    const RunArtifacts& artifacts) {
    const std::string dir = getVizDir();
    if (dir.empty()) {
        return;
    }

    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        return;
    }

    const fs::path path = fs::path(dir) / ("replanner_segments_" + mapping + ".json");
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        return;
    }

    ofs << std::fixed << std::setprecision(6);
    ofs << "{\n";
    ofs << "  \"map_min\": [" << scene.map_min.x() << ", " << scene.map_min.y() << ", "
        << scene.map_min.z() << "],\n";
    ofs << "  \"map_max\": [" << scene.map_max.x() << ", " << scene.map_max.y() << ", "
        << scene.map_max.z() << "],\n";
    ofs << "  \"request_start\": [" << scene.request.p_start.x() << ", "
        << scene.request.p_start.y() << ", " << scene.request.p_start.z() << "],\n";
    ofs << "  \"request_goal\": [" << scene.request.p_goal.x() << ", "
        << scene.request.p_goal.y() << ", " << scene.request.p_goal.z() << "],\n";
    ofs << "  \"sphere\": {\"center\": [" << scene.obstacle.center.x() << ", "
        << scene.obstacle.center.y() << ", " << scene.obstacle.center.z()
        << "], \"radius\": " << scene.obstacle.radius << "},\n";
    exportSegments(ofs, "segments", artifacts.replanned_points, artifacts.replanned_orientations);
    ofs << ",\n";
    exportSegments(
        ofs,
        "raw_grid_segments",
        artifacts.raw_astar_points,
        artifacts.raw_astar_orientations);
    ofs << ",\n";
    ofs << "  \"ik_records\": [\n";
    for (std::size_t i = 0; i < artifacts.ik_records.size(); ++i) {
        const auto& record = artifacts.ik_records[i];
        ofs << "    {"
            << "\"label\": \"" << record.label << "\", "
            << "\"tick\": " << record.tick << ", "
            << "\"waypoint_index\": " << record.waypoint_index << ", "
            << "\"ik_ok\": " << (record.ik_ok ? "true" : "false") << ", "
            << "\"external_ik_ok\": " << (record.external_ik_ok ? "true" : "false") << ", "
            << "\"fallback_ik_used\": " << (record.fallback_ik_used ? "true" : "false") << ", "
            << "\"collision_free\": " << (record.collision_free ? "true" : "false") << ", "
            << "\"min_margin\": " << record.min_margin << ", "
            << "\"reason\": \"" << record.reason << "\", "
            << "\"worst_link_name\": \"" << record.worst_link_name << "\", "
            << "\"target_position\": ["
            << record.target_position.x() << ", "
            << record.target_position.y() << ", "
            << record.target_position.z() << "], "
            << "\"target_orientation\": ["
            << record.target_orientation(0, 0) << ", "
            << record.target_orientation(0, 1) << ", "
            << record.target_orientation(0, 2) << ", "
            << record.target_orientation(1, 0) << ", "
            << record.target_orientation(1, 1) << ", "
            << record.target_orientation(1, 2) << ", "
            << record.target_orientation(2, 0) << ", "
            << record.target_orientation(2, 1) << ", "
            << record.target_orientation(2, 2) << "], "
            << "\"q_solution\": [";
        for (std::size_t q = 0; q < record.q_solution.size(); ++q) {
            ofs << record.q_solution[q];
            ofs << (q + 1 == record.q_solution.size() ? "" : ", ");
        }
        ofs << "], \"ellipsoids\": [\n";
        for (std::size_t e = 0; e < record.ellipsoids.size(); ++e) {
            const auto& ellipsoid = record.ellipsoids[e];
            ofs << "      {"
                << "\"link_name\": \"" << ellipsoid.link_name << "\", "
                << "\"debug_name\": \"" << ellipsoid.debug_name << "\", "
                << "\"center_world\": ["
                << ellipsoid.center_world.x() << ", "
                << ellipsoid.center_world.y() << ", "
                << ellipsoid.center_world.z() << "], "
                << "\"radii\": ["
                << ellipsoid.radii.x() << ", "
                << ellipsoid.radii.y() << ", "
                << ellipsoid.radii.z() << "], "
                << "\"rotation_world\": ["
                << ellipsoid.rotation_world(0, 0) << ", "
                << ellipsoid.rotation_world(0, 1) << ", "
                << ellipsoid.rotation_world(0, 2) << ", "
                << ellipsoid.rotation_world(1, 0) << ", "
                << ellipsoid.rotation_world(1, 1) << ", "
                << ellipsoid.rotation_world(1, 2) << ", "
                << ellipsoid.rotation_world(2, 0) << ", "
                << ellipsoid.rotation_world(2, 1) << ", "
                << ellipsoid.rotation_world(2, 2) << "], "
                << "\"margin\": " << ellipsoid.margin
                << "}";
            ofs << (e + 1 == record.ellipsoids.size() ? "\n" : ",\n");
        }
        ofs << "    ]}";
        ofs << (i + 1 == artifacts.ik_records.size() ? "\n" : ",\n");
    }
    ofs << "  ]\n";
    ofs << "}\n";
}

void printSceneSummary(const MappingContext& ctx, const DemoScene& scene) {
    std::cout << "\n=== replanner run: " << ctx.mapping << " ===" << std::endl;
    std::cout << "start=(" << scene.request.p_start.x() << ", " << scene.request.p_start.y() << ", "
              << scene.request.p_start.z() << ")"
              << " goal=(" << scene.request.p_goal.x() << ", " << scene.request.p_goal.y() << ", "
              << scene.request.p_goal.z() << ")"
              << " obstacle_center=(" << scene.obstacle.center.x() << ", "
              << scene.obstacle.center.y() << ", " << scene.obstacle.center.z() << ")"
              << " obstacle_r=" << scene.obstacle.radius << std::endl;
}

void printSegmentSummaries(const RunArtifacts& artifacts) {
    for (std::size_t seg = 0; seg < artifacts.replanned_points.size(); ++seg) {
        const auto& points = artifacts.replanned_points[seg];
        const auto& orientations = artifacts.replanned_orientations[seg];
        if (points.empty()) {
            std::cout << "segment " << seg << ": empty" << std::endl;
            continue;
        }

        const auto& p_start = points.front();
        const auto& p_end = points.back();
        const Eigen::Vector3d rpy_start = rotationToRpyDeg(orientations.front());
        const Eigen::Vector3d rpy_end = rotationToRpyDeg(orientations.back());
        std::cout << "segment " << seg
                  << " start=(" << p_start.x() << ", " << p_start.y() << ", " << p_start.z() << ")"
                  << " end=(" << p_end.x() << ", " << p_end.y() << ", " << p_end.z() << ")"
                  << " points=" << points.size()
                  << " start_rpy_deg=(" << rpy_start.x() << ", " << rpy_start.y() << ", "
                  << rpy_start.z() << ")"
                  << " end_rpy_deg=(" << rpy_end.x() << ", " << rpy_end.y() << ", "
                  << rpy_end.z() << ")" << std::endl;

        for (std::size_t i = 0; i < points.size(); ++i) {
            const Eigen::Vector3d rpy = rotationToRpyDeg(orientations[i]);
            const double dtheta_deg =
                (i == 0) ? 0.0 : relativeRotationDeg(orientations[i - 1], orientations[i]);
            std::cout << "  seg[" << seg << "] pt[" << i << "] = ("
                      << points[i].x() << ", " << points[i].y() << ", " << points[i].z() << ")"
                      << " rpy_deg=(" << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << ")"
                      << " dtheta_from_prev_deg=" << dtheta_deg << std::endl;
        }
    }
}

void captureRawAstarPath(
    const std::shared_ptr<cp::CartesianPathPlanner>& planner,
    const cp::PathPlanningInput& request,
    RunArtifacts& artifacts,
    const std::string& tag) {
    cp::PathPlanningInput raw_request = request;
    raw_request.whole_body_pose_validator = nullptr;
    raw_request.whole_body_segment_validator = nullptr;
    raw_request.whole_body_pose_diagnostic = nullptr;

    const auto raw = planner->planPath(raw_request);
    artifacts.raw_astar_points.push_back(pathToPoints(raw.path));
    artifacts.raw_astar_orientations.push_back(pathToOrientations(raw.path));
    std::cout << "[replanner_minimal] " << tag
              << " points=" << artifacts.raw_astar_points.back().size() << std::endl;
}

int runOneMapping(const rclcpp::Node::SharedPtr& node, const std::string& mapping) {
    const std::string arm_share = ament_index_cpp::get_package_share_directory("arm_controller");
    const std::string robot_desc_share =
        ament_index_cpp::get_package_share_directory("robot_description");
    const std::string hardware_cfg_path = arm_share + "/config/hardware_config.yaml";
    const std::string reactive_cfg_path = arm_share + "/config/reactive_task_config.yaml";

    ExampleRuntimeConfig runtime_cfg;
    std::string error;
    if (!loadRuntimeConfig(reactive_cfg_path, runtime_cfg, &error)) {
        std::cerr << "[replanner_minimal] load runtime config failed: " << error << std::endl;
        return 1;
    }

    MappingContext ctx;
    if (!loadMappingConfig(hardware_cfg_path, mapping, ctx, &error)) {
        std::cerr << "[replanner_minimal] loadMappingConfig failed: " << error << std::endl;
        return 1;
    }

    const std::string urdf_path = robot_desc_share + "/urdf/" + ctx.robot_type + ".urdf";

    AdapterBundle adapters;
    if (!createAdapters(node, mapping, adapters, &error)) {
        std::cerr << "[replanner_minimal] createAdapters failed: " << error << std::endl;
        return 1;
    }
    if (!resolveCurrentPose(node, urdf_path, adapters.tip_link, ctx, &error)) {
        std::cerr << "[replanner_minimal] resolveCurrentPose failed: " << error << std::endl;
        return 1;
    }

    PinocchioBundle pinocchio;
    if (!buildPinocchioBundle(
            node, hardware_cfg_path, urdf_path, adapters.tip_link, ctx, pinocchio, &error)) {
        std::cerr << "[replanner_minimal] buildPinocchioBundle failed: " << error << std::endl;
        return 1;
    }
    if (!initializeTracIk(ctx, urdf_path, adapters, &error)) {
        std::cerr << "[replanner_minimal] initializeTracIk failed: " << error << std::endl;
        return 1;
    }

    DemoScene scene = buildDemoScene(ctx, runtime_cfg);
    auto whole_body_validator = buildWholeBodyValidator(ctx, adapters, scene.map, pinocchio);
    scene.request.whole_body_pose_validator = whole_body_validator.makePoseValidatorFn();
    scene.request.whole_body_segment_validator = whole_body_validator.makeSegmentValidatorFn();
    scene.request.whole_body_pose_diagnostic = whole_body_validator.makePoseDiagnosticFn();

    auto planner = buildPlanner(scene.map, scene.map_min, runtime_cfg);
    cp::ReplannerManager replanner(planner);
    const cp::ReplannerConfig replanner_cfg = runtime_cfg.replanner;
    replanner.setConfig(replanner_cfg);

    printSceneSummary(ctx, scene);

    RunArtifacts artifacts;
    scene.request.whole_body_postcheck_failure_callback =
        [&](const cp::PathPlanningInput::WholeBodyPostcheckFailureEvent& event) {
            appendPostcheckFailureRecord(pinocchio, scene, event, artifacts);
        };
    captureRawAstarPath(planner, scene.request, artifacts, "raw_astar0");

    if (!replanner.start(scene.request, &error)) {
        std::cerr << "[replanner_minimal] start failed (" << mapping << "): " << error
                  << std::endl;
        exportRunJson(mapping, scene, artifacts);
        return 1;
    }

    artifacts.replanned_points.push_back(sampleActiveSegmentPoints(replanner));
    artifacts.replanned_orientations.push_back(sampleActiveSegmentOrientations(replanner));
    captureIkForWaypoints(
        scene.request,
        artifacts.replanned_points.back(),
        artifacts.replanned_orientations.back(),
        pinocchio,
        scene,
        "initial_plan",
        0,
        false,
        artifacts);

    bool reached_goal = false;
    double exec_time_sec = 0.0;
    const int total_control_ticks = runtime_cfg.max_control_ticks;
    for (int tick = 1; tick <= total_control_ticks; ++tick) {
        const int point_count = replanner.activeSegmentPointCount();
        if (point_count <= 0) {
            std::cerr << "[replanner_minimal] active segment empty at tick " << tick
                      << " (" << mapping << ")" << std::endl;
            exportRunJson(mapping, scene, artifacts);
            return 1;
        }

        exec_time_sec = std::min(exec_time_sec, replanner.activeSegmentTotalDurationSec());
        const int exec_idx = replanner.pointIndexAtTime(exec_time_sec);

        cp::TimedCartesianSample current;
        if (!replanner.sampleByElapsedTime(exec_time_sec, current)) {
            std::cerr << "[replanner_minimal] sample failed at tick " << tick
                      << " (" << mapping << ")" << std::endl;
            exportRunJson(mapping, scene, artifacts);
            return 1;
        }

        const double dist_to_goal = (current.T_target.translation() - scene.request.p_goal).norm();
        if (dist_to_goal <= scene.request.goal_tolerance) {
            reached_goal = true;
            std::cout << "[replanner_minimal] reached goal at tick " << tick
                      << " (" << mapping << "), dist=" << dist_to_goal << std::endl;
            break;
        }

        if (!replanner.shouldReplanAtControlTick(tick)) {
            exec_time_sec += replanner_cfg.control_cycle_sec;
            continue;
        }

        cp::TimedCartesianSample predicted;
        cp::PathPlanningInput raw_replan_request = scene.request;
        if (replanner.samplePredictedReplanStart(exec_idx, predicted)) {
            raw_replan_request.p_start = predicted.T_target.translation();
            raw_replan_request.R_start = predicted.T_target.linear();
        }
        captureRawAstarPath(planner, raw_replan_request, artifacts, "raw_astar_replan");

        if (!replanner.planFromPredictedActiveTrajectory(scene.request, exec_idx, &error)) {
            std::cerr << "[replanner_minimal] replan failed at tick " << tick
                      << " (" << mapping << "): " << error
                      << " (keep current segment)" << std::endl;
            exec_time_sec += replanner_cfg.control_cycle_sec;
            continue;
        }

        artifacts.replanned_points.push_back(sampleActiveSegmentPoints(replanner));
        artifacts.replanned_orientations.push_back(sampleActiveSegmentOrientations(replanner));
        captureIkForWaypoints(
            raw_replan_request,
            artifacts.replanned_points.back(),
            artifacts.replanned_orientations.back(),
            pinocchio,
            scene,
            "replan",
            tick,
            true,
            artifacts);
        exec_time_sec = 0.0;
    }

    if (!reached_goal) {
        std::cout << "[replanner_minimal] finished ticks without explicit goal-reached flag for "
                  << mapping << std::endl;
    }

    printSegmentSummaries(artifacts);
    exportRunJson(mapping, scene, artifacts);
    std::cout << "[replanner_minimal] done for " << mapping << std::endl;
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("example_cartesian_replanner_minimal");

    std::vector<std::string> mappings;
    if (argc > 1) {
        mappings.push_back(argv[1]);
    } else {
        mappings = {"left_arm", "right_arm"};
    }

    int rc = 0;
    for (const auto& mapping : mappings) {
        rc = std::max(rc, runOneMapping(node, mapping));
    }

    rclcpp::shutdown();
    if (rc == 0) {
        std::cout << "replanner minimal demo done." << std::endl;
    }
    return rc;
}
