#include "reactive_task_controller.hpp"

#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include "controller_interface.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <cctype>
#include <cmath>
#include <fstream>
#include <future>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include "algorithm/cartesian_path_planner/map/dummy_distance_field.hpp"
#include "algorithm/cartesian_path_planner/map/camera_driver_esdf_map_client.hpp"
#include "algorithm/cartesian_path_planner/map/camera_driver_pointcloud_map_adapter.hpp"
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

geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    return point;
}

int markerBaseIdForMapping(const std::string& mapping) {
    return (mapping == "right_arm") ? 100 : 0;
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
    if (rtc["terminal_goal_capture_enable"]) {
        cfg.terminal_goal_capture_enable = rtc["terminal_goal_capture_enable"].as<bool>();
    }
    if (rtc["terminal_goal_capture_pos_err_threshold_m"]) {
        cfg.terminal_goal_capture_pos_err_threshold_m =
            std::max(1e-6, rtc["terminal_goal_capture_pos_err_threshold_m"].as<double>());
    }
    if (rtc["terminal_goal_capture_ori_err_threshold_rad"]) {
        cfg.terminal_goal_capture_ori_err_threshold_rad =
            std::max(1e-6, rtc["terminal_goal_capture_ori_err_threshold_rad"].as<double>());
    }
    if (rtc["terminal_goal_capture_no_progress_cycles"]) {
        cfg.terminal_goal_capture_no_progress_cycles =
            std::max(1, rtc["terminal_goal_capture_no_progress_cycles"].as<int>());
    }
    if (rtc["terminal_goal_capture_suppress_replanning"]) {
        cfg.terminal_goal_capture_suppress_replanning =
            rtc["terminal_goal_capture_suppress_replanning"].as<bool>();
    }
    if (rtc["terminal_goal_capture_relax_obstacle_damper_when_far"]) {
        cfg.terminal_goal_capture_relax_obstacle_damper_when_far =
            rtc["terminal_goal_capture_relax_obstacle_damper_when_far"].as<bool>();
    }
    if (rtc["terminal_goal_capture_obstacle_near_distance_m"]) {
        cfg.terminal_goal_capture_obstacle_near_distance_m =
            std::max(0.0, rtc["terminal_goal_capture_obstacle_near_distance_m"].as<double>());
    }

    if (const YAML::Node req = rtc["request"]; req && req.IsMap()) {
        if (!req["safe_distance"]) {
            setError("Missing required key: reactive_task_controller.request.safe_distance");
            return false;
        }
        cfg.request_safe_distance = std::max(0.0, req["safe_distance"].as<double>());
        if (req["hard_clearance"]) {
            cfg.request_hard_clearance = std::max(0.0, req["hard_clearance"].as<double>());
        }
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
        if (req["whole_body_segment_check_step_m"]) {
            cfg.whole_body_segment_check_step_m =
                std::max(1e-3, req["whole_body_segment_check_step_m"].as<double>());
        }
    } else {
        setError("Missing required map: reactive_task_controller.request");
        return false;
    }

    if (rtc["map_source"]) {
        cfg.map_source = rtc["map_source"].as<std::string>();
        std::transform(cfg.map_source.begin(), cfg.map_source.end(), cfg.map_source.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
    }
    if (cfg.map_source != "dummy" &&
        cfg.map_source != "camera_driver_pointcloud" &&
        cfg.map_source != "camera_driver_esdf") {
        setError(
            "Invalid reactive_task_controller.map_source, expected "
            "'dummy', 'camera_driver_pointcloud', or 'camera_driver_esdf'");
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

    if (const YAML::Node live = rtc["camera_driver_pointcloud"]; live && live.IsMap()) {
        if (live["pointcloud_topic"]) {
            cfg.camera_driver_pointcloud.pointcloud_topic =
                live["pointcloud_topic"].as<std::string>();
        }
        if (live["pointcloud_queue_depth"]) {
            cfg.camera_driver_pointcloud.pointcloud_queue_depth =
                std::max(1, live["pointcloud_queue_depth"].as<int>());
        }
        if (live["voxel_size_m"]) {
            cfg.camera_driver_pointcloud.voxel_size_m =
                std::max(1e-3, live["voxel_size_m"].as<double>());
        }
        if (live["max_distance_m"]) {
            cfg.camera_driver_pointcloud.max_distance_m =
                std::max(
                    cfg.camera_driver_pointcloud.voxel_size_m,
                    live["max_distance_m"].as<double>());
        }
        if (live["observation_margin_m"]) {
            cfg.camera_driver_pointcloud.observation_margin_m =
                std::max(0.0, live["observation_margin_m"].as<double>());
        }
        if (live["isolated_min_neighbor_count"]) {
            cfg.camera_driver_pointcloud.isolated_min_neighbor_count =
                std::max(0, live["isolated_min_neighbor_count"].as<int>());
        }
        if (live["isolated_neighbor_radius_cells"]) {
            cfg.camera_driver_pointcloud.isolated_neighbor_radius_cells =
                std::max(1, live["isolated_neighbor_radius_cells"].as<int>());
        }
    } else if (cfg.map_source == "camera_driver_pointcloud") {
        setError("Missing required map: reactive_task_controller.camera_driver_pointcloud");
        return false;
    }

    if (const YAML::Node live = rtc["camera_driver_esdf"]; live && live.IsMap()) {
        if (live["service_name"]) {
            cfg.camera_driver_esdf.service_name = live["service_name"].as<std::string>();
        }
        if (live["request_timeout_ms"]) {
            cfg.camera_driver_esdf.request_timeout_ms =
                std::max(1, live["request_timeout_ms"].as<int>());
        }
        if (live["startup_wait_timeout_ms"]) {
            cfg.camera_driver_esdf.startup_wait_timeout_ms =
                std::max(1, live["startup_wait_timeout_ms"].as<int>());
        }
        if (live["cache_max_entries"]) {
            cfg.camera_driver_esdf.cache_max_entries =
                std::max<std::size_t>(1u, live["cache_max_entries"].as<std::size_t>());
        }
    } else if (cfg.map_source == "camera_driver_esdf") {
        setError("Missing required map: reactive_task_controller.camera_driver_esdf");
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
            if (common["use_joint_space_sampling"]) {
                cfg.planner_common.use_joint_space_sampling =
                    common["use_joint_space_sampling"].as<bool>();
            }
            if (common["joint_space_sampling_max_iterations"]) {
                cfg.planner_common.joint_space_sampling_max_iterations =
                    std::max(1, common["joint_space_sampling_max_iterations"].as<int>());
            }
            if (common["joint_space_sampling_step_rad"]) {
                cfg.planner_common.joint_space_sampling_step_rad =
                    std::max(1e-3, common["joint_space_sampling_step_rad"].as<double>());
            }
            if (common["joint_space_sampling_goal_bias"]) {
                cfg.planner_common.joint_space_sampling_goal_bias =
                    std::clamp(common["joint_space_sampling_goal_bias"].as<double>(), 0.0, 1.0);
            }
            if (common["joint_space_sampling_connect_threshold_rad"]) {
                cfg.planner_common.joint_space_sampling_connect_threshold_rad =
                    std::max(1e-3, common["joint_space_sampling_connect_threshold_rad"].as<double>());
            }
            if (common["joint_space_sampling_local_window_rad"]) {
                cfg.planner_common.joint_space_sampling_local_window_rad =
                    std::max(1e-3, common["joint_space_sampling_local_window_rad"].as<double>());
            }
            if (common["joint_space_sampling_search_stages"]) {
                cfg.planner_common.joint_space_sampling_search_stages =
                    std::max(1, common["joint_space_sampling_search_stages"].as<int>());
            }
            if (common["joint_space_sampling_window_scale"]) {
                cfg.planner_common.joint_space_sampling_window_scale =
                    std::max(1.0, common["joint_space_sampling_window_scale"].as<double>());
            }
            if (common["joint_space_sampling_allow_full_joint_limit_fallback"]) {
                cfg.planner_common.joint_space_sampling_allow_full_joint_limit_fallback =
                    common["joint_space_sampling_allow_full_joint_limit_fallback"].as<bool>();
            }
            if (common["joint_space_shortcut_trials"]) {
                cfg.planner_common.joint_space_shortcut_trials =
                    std::max(0, common["joint_space_shortcut_trials"].as<int>());
            }
            if (common["joint_space_sampling_solution_pool_size"]) {
                cfg.planner_common.joint_space_sampling_solution_pool_size =
                    std::max(1, common["joint_space_sampling_solution_pool_size"].as<int>());
            }
            if (common["prefer_clearance_shell"]) {
                cfg.planner_common.prefer_clearance_shell =
                    common["prefer_clearance_shell"].as<bool>();
            }
            if (common["preferred_clearance_shell_margin_m"]) {
                cfg.planner_common.preferred_clearance_shell_margin_m =
                    common["preferred_clearance_shell_margin_m"].as<double>();
            }
            if (common["preferred_clearance_shell_weight"]) {
                cfg.planner_common.preferred_clearance_shell_weight =
                    std::max(0.0, common["preferred_clearance_shell_weight"].as<double>());
            }
            if (common["joint_space_path_length_weight"]) {
                cfg.planner_common.joint_space_path_length_weight =
                    std::max(0.0, common["joint_space_path_length_weight"].as<double>());
            }
            if (common["joint_space_joint_motion_weight"]) {
                cfg.planner_common.joint_space_joint_motion_weight =
                    std::max(0.0, common["joint_space_joint_motion_weight"].as<double>());
            }
            if (common["enable_joint_trajectory_post_optimization"]) {
                cfg.planner_common.enable_joint_trajectory_post_optimization =
                    common["enable_joint_trajectory_post_optimization"].as<bool>();
            }
            if (common["joint_trajectory_postopt_iterations"]) {
                cfg.planner_common.joint_trajectory_postopt_iterations =
                    std::max(0, common["joint_trajectory_postopt_iterations"].as<int>());
            }
            if (common["joint_trajectory_postopt_samples_per_waypoint"]) {
                cfg.planner_common.joint_trajectory_postopt_samples_per_waypoint =
                    std::max(1, common["joint_trajectory_postopt_samples_per_waypoint"].as<int>());
            }
            if (common["joint_trajectory_postopt_perturbation_rad"]) {
                cfg.planner_common.joint_trajectory_postopt_perturbation_rad =
                    std::max(1e-4, common["joint_trajectory_postopt_perturbation_rad"].as<double>());
            }
            if (common["joint_trajectory_shell_weight"]) {
                cfg.planner_common.joint_trajectory_shell_weight =
                    std::max(0.0, common["joint_trajectory_shell_weight"].as<double>());
            }
            if (common["joint_trajectory_orientation_weight"]) {
                cfg.planner_common.joint_trajectory_orientation_weight =
                    std::max(0.0, common["joint_trajectory_orientation_weight"].as<double>());
            }
            if (common["joint_trajectory_smoothness_weight"]) {
                cfg.planner_common.joint_trajectory_smoothness_weight =
                    std::max(0.0, common["joint_trajectory_smoothness_weight"].as<double>());
            }
            if (common["joint_trajectory_position_weight"]) {
                cfg.planner_common.joint_trajectory_position_weight =
                    std::max(0.0, common["joint_trajectory_position_weight"].as<double>());
            }
            cfg.planner_common.enable_interpolator_smoothing =
                common["enable_interpolator_smoothing"].as<bool>();
            cfg.planner_common.interpolator_continuity_order =
                common["interpolator_continuity_order"].as<int>();
            cfg.planner_common.interpolator_target_dt = common["interpolator_target_dt"].as<double>();
            if (common["enable_minimum_snap_optimization"]) {
                cfg.planner_common.enable_minimum_snap_optimization =
                    common["enable_minimum_snap_optimization"].as<bool>();
            }
            if (common["minimum_snap_iterations"]) {
                cfg.planner_common.minimum_snap_iterations =
                    std::max(0, common["minimum_snap_iterations"].as<int>());
            }
            if (common["minimum_snap_data_weight"]) {
                cfg.planner_common.minimum_snap_data_weight =
                    std::max(0.0, common["minimum_snap_data_weight"].as<double>());
            }
            if (common["minimum_snap_weight"]) {
                cfg.planner_common.minimum_snap_weight =
                    std::max(0.0, common["minimum_snap_weight"].as<double>());
            }
            if (common["minimum_snap_relaxation"]) {
                cfg.planner_common.minimum_snap_relaxation =
                    std::clamp(common["minimum_snap_relaxation"].as<double>(), 1e-3, 1.0);
            }
        } else {
            setError("Missing required map: reactive_task_controller.planner.common");
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
    ensureCameraDriverDistanceFieldInitialized();
    const auto marker_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    collision_ellipsoid_marker_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/reactive_task/collision_ellipsoid_markers",
            marker_qos);
    trajectory_marker_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/reactive_task/trajectory_markers",
            marker_qos);
    collision_ellipsoid_marker_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            if (!hardware_manager_) {
                return;
            }

            const std::vector<std::string> mappings = hardware_manager_->get_all_mappings();
            for (const std::string& mapping : mappings) {
                std::string init_error;
                if (!initializeMappingContext(mapping, &init_error)) {
                    continue;
                }

                MappingContext* ctx = nullptr;
                {
                    std::lock_guard<std::mutex> lock(mapping_contexts_mutex_);
                    auto it = mapping_contexts_.find(mapping);
                    if (it == mapping_contexts_.end() || !it->second.initialized) {
                        continue;
                    }
                    ctx = &it->second;
                }
                if (ctx == nullptr) {
                    continue;
                }

                const std::vector<double> q_current_vec =
                    hardware_manager_->get_current_joint_positions_lockfree(mapping);
                if (q_current_vec.size() != ctx->joint_names.size()) {
                    continue;
                }
                const Eigen::VectorXd q_current = Eigen::Map<const Eigen::VectorXd>(
                    q_current_vec.data(),
                    static_cast<Eigen::Index>(q_current_vec.size()));
                publishCollisionEllipsoidMarkers(mapping, q_current, *ctx);
            }
        });

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

void ReactiveTaskController::ensureCameraDriverDistanceFieldInitialized() {
    if (!reactive_cfg_loaded_) {
        return;
    }

    std::lock_guard<std::mutex> lock(live_distance_field_mutex_);
    if (runtime_cfg_.map_source == "camera_driver_pointcloud") {
        if (camera_driver_pointcloud_map_) {
            return;
        }
        camera_driver_pointcloud_map_ =
            std::make_shared<cp::CameraDriverPointcloudMapAdapter>(
                runtime_cfg_.camera_driver_pointcloud,
                node_);
        RCLCPP_INFO(
            node_->get_logger(),
            "[ReactiveTask] Pre-initialized camera_driver pointcloud adapter before controller activation.");
        return;
    }
    if (runtime_cfg_.map_source == "camera_driver_esdf") {
        if (camera_driver_esdf_map_) {
            return;
        }
        camera_driver_esdf_map_ =
            std::make_shared<cp::CameraDriverEsdfMapClient>(
                runtime_cfg_.camera_driver_esdf,
                node_);
        RCLCPP_INFO(
            node_->get_logger(),
            "[ReactiveTask] Pre-initialized camera_driver ESDF client before controller activation.");
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

    ensureCameraDriverDistanceFieldInitialized();

    std::string error;
    if (!initializeMappingContext(mapping, &error)) {
        throw std::runtime_error("ReactiveTask context init failed for '" + mapping + "': " + error);
    }

    {
        std::lock_guard<std::mutex> lock(mapping_contexts_mutex_);
        auto it = mapping_contexts_.find(mapping);
        if (it != mapping_contexts_.end() && it->second.initialized) {
            const std::vector<double> q_current_vec =
                hardware_manager_->get_current_joint_positions_lockfree(mapping);
            if (q_current_vec.size() == it->second.joint_names.size()) {
                const Eigen::VectorXd q_current = Eigen::Map<const Eigen::VectorXd>(
                    q_current_vec.data(),
                    static_cast<Eigen::Index>(q_current_vec.size()));
                publishCollisionEllipsoidMarkers(mapping, q_current, it->second);
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] ReactiveTaskController activated", mapping.c_str());
}

bool ReactiveTaskController::stop(const std::string& mapping) {
    TrajectoryControllerImpl::stop(mapping);
    cleanup_subscriptions(mapping);
    clearCollisionEllipsoidMarkers(mapping);
    clearTrajectoryMarkers(mapping);
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
    const Eigen::Vector3d& /*map_min*/) const {
    return std::make_shared<cp::CartesianPathPlanner>(
        runtime_cfg_.planner_common,
        runtime_cfg_.planner_smoothing,
        map);
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
    request.q_min = ctx->joint_limits.q_min;
    request.q_max = ctx->joint_limits.q_max;
    request.safe_distance = runtime_cfg_.request_safe_distance;
    request.hard_clearance = runtime_cfg_.request_hard_clearance;
    request.goal_tolerance = runtime_cfg_.request_goal_tolerance;
    const Eigen::Vector3d min_corner = request.p_start.cwiseMin(request.p_goal);
    const Eigen::Vector3d max_corner = request.p_start.cwiseMax(request.p_goal);
    const Eigen::Vector3d map_margin = runtime_cfg_.map_margin_xyz;
    const Eigen::Vector3d map_min = min_corner - map_margin;
    const Eigen::Vector3d map_max = max_corner + map_margin;

    std::shared_ptr<cp::DistanceFieldInterface> map_mutable;
    if (runtime_cfg_.map_source == "camera_driver_pointcloud" ||
        runtime_cfg_.map_source == "camera_driver_esdf") {
        ensureCameraDriverDistanceFieldInitialized();
        std::lock_guard<std::mutex> lock(live_distance_field_mutex_);
        if (runtime_cfg_.map_source == "camera_driver_pointcloud") {
            map_mutable = camera_driver_pointcloud_map_;
        } else {
            map_mutable = camera_driver_esdf_map_;
        }
    } else {
        auto dummy_map = std::make_shared<cp::DummyDistanceField>(map_min, map_max);
        if (runtime_cfg_.enable_dummy_obstacle && runtime_cfg_.dummy_obstacle_radius > 0.0) {
            cp::SphereObstacle obstacle;
            if (mapping == "right_arm") {
                obstacle.center = runtime_cfg_.dummy_obstacle_center_right_arm;
            } else {
                obstacle.center = runtime_cfg_.dummy_obstacle_center_left_arm;
            }
            obstacle.radius = runtime_cfg_.dummy_obstacle_radius;
            dummy_map->addSphere(obstacle);
        }
        map_mutable = dummy_map;
    }

    const std::shared_ptr<const cp::DistanceFieldInterface> map = map_mutable;

    auto logMapProbe = [&](const char* tag, const Eigen::Vector3d& p) {
        const auto query = map->queryDistanceAndGradient(p);
        const Eigen::Vector3d gradient =
            query.gradient_valid ? query.gradient : Eigen::Vector3d::Zero();
        const double distance = query.distance_valid ? query.distance : -1.0;
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] map_probe[%s]: p=(%.4f, %.4f, %.4f) inside=%s distance=%.5f gradient=(%.4f, %.4f, %.4f)",
            mapping.c_str(),
            tag,
            p.x(),
            p.y(),
            p.z(),
            query.observed ? "true" : "false",
            distance,
            gradient.x(),
            gradient.y(),
            gradient.z());
    };

    if (runtime_cfg_.map_source != "dummy") {
        logMapProbe("start", request.p_start);
        logMapProbe("goal", request.p_goal);
    }
    publishCollisionEllipsoidMarkers(mapping, q_start, *ctx);

    auto planner = buildPlanner(map, map_min);
    const bool enable_obstacle_constraints =
        reactive_cfg_.qp_build.enable_obstacle_damper &&
        ((runtime_cfg_.map_source == "camera_driver_pointcloud") ||
         (runtime_cfg_.map_source == "camera_driver_esdf") ||
         (runtime_cfg_.map_source == "dummy" && runtime_cfg_.enable_dummy_obstacle));

    std::optional<cp::WholeBodyEllipsoidPoseValidator> whole_body_validator;
    if (ctx->tracik_ready && ctx->tracik_adapter && ctx->moveit_adapter && !ctx->collision_ellipsoids.empty()) {
        cp::WholeBodyEllipsoidPoseValidator::Config wb_cfg;
        wb_cfg.ik_max_iterations = 25;
        wb_cfg.ik_pos_tolerance_m = 0.01;
        wb_cfg.ik_rot_tolerance_rad = 0.15;
        wb_cfg.ik_damping = 0.05;
        wb_cfg.ik_step_scale = 0.6;
        wb_cfg.segment_substeps_min = 1;
        wb_cfg.segment_check_step_m = runtime_cfg_.whole_body_segment_check_step_m;
        wb_cfg.collision_blocking_margin_m =
            request.hard_clearance - request.safe_distance;
        wb_cfg.default_q_seed = q_start;
        wb_cfg.ik_solver_fn =
            [moveit = ctx->moveit_adapter,
             tracik = ctx->tracik_adapter,
             fk_provider = ctx->fk_provider,
             seed_default = q_current_vec,
             goal_position = request.p_goal,
             goal_orientation = request.R_goal,
             goal_tolerance = request.goal_tolerance](
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

                const geometry_msgs::msg::Pose pose_world =
                    toPoseMsg(p_target, R_target);
                const geometry_msgs::msg::Pose pose_base =
                    moveit->worldPoseToBaseLinkPose(pose_world);

                std::vector<double> q_solution_vec;
                if (!tracik->computeIKClosest(pose_base, seed, q_solution_vec, 8, false)) {
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
        request.joint_state_validator = whole_body_validator->makeJointStateValidatorFn();
        request.joint_segment_validator = whole_body_validator->makeJointSegmentValidatorFn();
        request.joint_to_pose_fn =
            [fk = ctx->fk_provider](
                const Eigen::VectorXd& q,
                cp::CartesianWaypoint& wp) -> bool {
                if (!fk) {
                    return false;
                }
                arm_controller::kinematics::ForwardKinematicsOutput fk_out;
                if (!fk->compute(q, fk_out)) {
                    return false;
                }
                wp.position = fk_out.ee_position;
                wp.orientation = fk_out.ee_rotation;
                return true;
            };

        const geometry_msgs::msg::Pose pose_goal_world =
            toPoseMsg(request.p_goal, request.R_goal);
        const geometry_msgs::msg::Pose pose_goal_base =
            ctx->moveit_adapter->worldPoseToBaseLinkPose(pose_goal_world);
        std::vector<double> seed_goal = q_current_vec;
        std::vector<double> q_goal_vec;
        for (int attempt = 0; attempt < 8; ++attempt) {
            if (!ctx->tracik_adapter->computeIKClosest(
                    pose_goal_base, seed_goal, q_goal_vec, 1, false) ||
                q_goal_vec.size() != q_current_vec.size()) {
                continue;
            }
            const Eigen::VectorXd q_goal = Eigen::Map<const Eigen::VectorXd>(
                q_goal_vec.data(),
                static_cast<Eigen::Index>(q_goal_vec.size()));
            bool duplicate = false;
            for (const auto& existing : request.q_goal_candidates) {
                if (existing.size() == q_goal.size() &&
                    (existing - q_goal).cwiseAbs().maxCoeff() < 1e-3) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                request.q_goal_candidates.push_back(q_goal);
            }
            seed_goal = q_goal_vec;
            if (!seed_goal.empty()) {
                const std::size_t idx = static_cast<std::size_t>(attempt % seed_goal.size());
                seed_goal[idx] += ((attempt % 2) == 0 ? 0.15 : -0.15);
            }
        }
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

    if (request.q_goal_candidates.empty()) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] ReactiveTask planner start failed: no valid goal IK candidates.",
            mapping.c_str());
        last_execution_success_[mapping] = false;
        return;
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
        "[%s] safety_config: neo_dt=%.4f s, planner_dt=%.4f s, replan_every=%d ticks, max_ticks=%d, desired_clearance=%.3f, hard_clearance=%.3f, joint_sampling=%s, pos_tol=%.4f, ori_tol=%.4f",
        mapping.c_str(),
        neo_tick_sec,
        planner_tick_sec,
        replanner_cfg.replan_every_control_ticks,
        runtime_cfg_.max_control_ticks,
        request.safe_distance,
        request.hard_clearance,
        runtime_cfg_.planner_common.use_joint_space_sampling ? "true" : "false",
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
    if (runtime_cfg_.map_source == "camera_driver_pointcloud") {
        int processed_frames = 0;
        std::size_t active_cells = 0u;
        {
            std::lock_guard<std::mutex> lock(live_distance_field_mutex_);
            if (camera_driver_pointcloud_map_) {
                processed_frames = camera_driver_pointcloud_map_->processedFrames();
                active_cells = camera_driver_pointcloud_map_->activeCellCount();
            }
        }
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] map_source=camera_driver_pointcloud: cloud_topic=%s voxel=%.3f max_dist=%.3f margin=%.3f isolated_neighbors>=%d radius=%d processed_frames=%d active_cells=%zu",
            mapping.c_str(),
            runtime_cfg_.camera_driver_pointcloud.pointcloud_topic.c_str(),
            runtime_cfg_.camera_driver_pointcloud.voxel_size_m,
            runtime_cfg_.camera_driver_pointcloud.max_distance_m,
            runtime_cfg_.camera_driver_pointcloud.observation_margin_m,
            runtime_cfg_.camera_driver_pointcloud.isolated_min_neighbor_count,
            runtime_cfg_.camera_driver_pointcloud.isolated_neighbor_radius_cells,
            processed_frames,
            active_cells);
    } else if (runtime_cfg_.map_source == "camera_driver_esdf") {
        std::size_t successful_queries = 0u;
        std::size_t failed_queries = 0u;
        bool service_ready = false;
        {
            std::lock_guard<std::mutex> lock(live_distance_field_mutex_);
            if (camera_driver_esdf_map_) {
                successful_queries = camera_driver_esdf_map_->successfulQueries();
                failed_queries = camera_driver_esdf_map_->failedQueries();
                service_ready = camera_driver_esdf_map_->isServiceReady();
            }
        }
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] map_source=camera_driver_esdf: service=%s request_timeout_ms=%d startup_wait_timeout_ms=%d service_ready=%s successful_queries=%zu failed_queries=%zu",
            mapping.c_str(),
            runtime_cfg_.camera_driver_esdf.service_name.c_str(),
            runtime_cfg_.camera_driver_esdf.request_timeout_ms,
            runtime_cfg_.camera_driver_esdf.startup_wait_timeout_ms,
            service_ready ? "true" : "false",
            successful_queries,
            failed_queries);
    } else if (runtime_cfg_.enable_dummy_obstacle) {
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
    const double active_duration_sec = replanner.activeSegmentTotalDurationSec();
    const int configured_max_planner_ticks = std::max(1, runtime_cfg_.max_control_ticks);
    const int duration_limited_min_ticks = std::max(
        1,
        static_cast<int>(std::ceil((active_duration_sec + 1.0) / planner_tick_sec)));
    const int effective_max_planner_ticks =
        std::max(configured_max_planner_ticks, duration_limited_min_ticks);

    if (sample_ok) {
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] active_trajectory: duration=%.3f s sampled_points=%d max_ticks(configured=%d effective=%d)",
            mapping.c_str(),
            active_duration_sec,
            replanner.activeSegmentPointCount(),
            configured_max_planner_ticks,
            effective_max_planner_ticks);
        publishTrajectoryMarkers(mapping, replanner, sample, fk_start.ee_position);
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
    bool terminal_goal_capture_active = false;
    bool terminal_goal_capture_logged = false;

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

    while (rclcpp::ok() && sample_ok && planner_tick < effective_max_planner_ticks) {
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
        const double continuous_sample_time_sec =
            std::min(
                planner_time_sec + planner_tick_accumulator,
                replanner.activeSegmentTotalDurationSec());
        if (!replanner.sampleByElapsedTime(continuous_sample_time_sec, sample)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "[%s] ReactiveTask continuous sample failed at planner tick %d",
                mapping.c_str(),
                planner_tick);
            sample_ok = false;
            break;
        }
        const bool reference_finished =
            continuous_sample_time_sec >=
            std::max(0.0, replanner.activeSegmentTotalDurationSec() - 1e-6);
        if ((neo_iter % safety_log_stride) == 0) {
            publishCollisionEllipsoidMarkers(mapping, q_now, *ctx);
        }

        const double pos_err_now =
            (sample.T_target.translation() - fk_now.ee_pose.translation()).norm();
        const Eigen::Matrix3d R_err_now =
            fk_now.ee_pose.rotation().transpose() * sample.T_target.rotation();
        Eigen::AngleAxisd aa_err_now(R_err_now);
        const double ori_err_now =
            std::isfinite(aa_err_now.angle()) ? std::abs(aa_err_now.angle()) : 0.0;
        const double pos_err_goal = (fk_now.ee_position - request.p_goal).norm();
        const double ori_err_goal = orientationErrorRad(fk_now.ee_rotation, request.R_goal);

        const bool should_enter_terminal_goal_capture =
            runtime_cfg_.terminal_goal_capture_enable &&
            !terminal_goal_capture_active &&
            ((pos_err_goal <= runtime_cfg_.terminal_goal_capture_pos_err_threshold_m &&
              ori_err_goal <= runtime_cfg_.terminal_goal_capture_ori_err_threshold_rad) ||
             no_progress_cycles >= runtime_cfg_.terminal_goal_capture_no_progress_cycles ||
             reference_finished);
        if (should_enter_terminal_goal_capture) {
            terminal_goal_capture_active = true;
            if (!terminal_goal_capture_logged) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "[%s] terminal_goal_capture: enabled at planner_tick=%d pos_err_goal=%.5f ori_err_goal=%.5f no_progress_cycles=%d reference_finished=%s",
                    mapping.c_str(),
                    planner_tick,
                    pos_err_goal,
                    ori_err_goal,
                    no_progress_cycles,
                    reference_finished ? "true" : "false");
                terminal_goal_capture_logged = true;
            }
        }

        const double terminal_position_release_tolerance =
            3.0 * runtime_cfg_.goal_position_tolerance;
        const double terminal_orientation_blend =
            terminal_goal_capture_active
                ? std::clamp(
                      (terminal_position_release_tolerance - pos_err_goal) /
                          std::max(1e-6,
                                   terminal_position_release_tolerance -
                                       runtime_cfg_.goal_position_tolerance),
                      0.0,
                      1.0)
                : 1.0;
        const bool terminal_position_priority =
            terminal_goal_capture_active && terminal_orientation_blend < 0.999;

        Eigen::Isometry3d commanded_target_pose = sample.T_target;
        if (terminal_goal_capture_active) {
            commanded_target_pose.translation() = request.p_goal;

            Eigen::Quaterniond q_current_terminal(fk_now.ee_pose.linear());
            Eigen::Quaterniond q_goal_terminal(request.R_goal);
            q_current_terminal.normalize();
            q_goal_terminal.normalize();
            if (q_current_terminal.dot(q_goal_terminal) < 0.0) {
                q_goal_terminal.coeffs() *= -1.0;
            }
            commanded_target_pose.linear() =
                q_current_terminal
                    .slerp(terminal_orientation_blend, q_goal_terminal)
                    .normalized()
                    .toRotationMatrix();
        }

        rq::TaskVelocityInput task_in;
        task_in.T_current = fk_now.ee_pose;
        task_in.has_target_pose = true;
        task_in.T_target = commanded_target_pose;
        task_in.has_target_twist = !reference_finished && !terminal_goal_capture_active;
        task_in.target_twist =
            (reference_finished || terminal_goal_capture_active)
                ? Eigen::Matrix<double, 6, 1>::Zero()
                : sample.target_twist;
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
        if (terminal_goal_capture_active) {
            manip_grad *= terminal_orientation_blend;
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
                toPoseMsg(commanded_target_pose.translation(), commanded_target_pose.linear());
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
        const bool allow_posture_tracking =
            ((terminal_goal_capture_active && terminal_orientation_blend >= 0.95) ||
             (!terminal_goal_capture_active &&
              pos_err_now <= 0.05 && ori_err_now <= 0.10));
        if (!allow_posture_tracking) {
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
        rq::ReactiveQpBuildConfig qp_build_cfg = reactive_cfg_.qp_build;
        bool terminal_obstacle_relaxed = false;
        double terminal_obstacle_min_distance = std::numeric_limits<double>::quiet_NaN();
        double terminal_obstacle_release_distance =
            runtime_cfg_.terminal_goal_capture_obstacle_near_distance_m;
        if (enable_obstacle_constraints) {
            std::string obstacle_error;
            const int generated = rq::BodyObstacleConstraintBuilder::appendLinkEllipsoidConstraints(
                q_now,
                fk_now.link_poses,
                ctx->collision_ellipsoids,
                *ctx->jacobian_provider,
                map,
                qp_input.obstacle_constraints,
                &obstacle_error);
            if (generated <= 0) {
                qp_input.obstacle_constraints.clear();
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    2000,
                    "[%s] ReactiveTask obstacle constraints skipped at planner tick %d; proceeding without obstacle damper for this tick: %s",
                    mapping.c_str(),
                    planner_tick,
                    obstacle_error.c_str());
            }
            if (!qp_input.obstacle_constraints.empty()) {
                terminal_obstacle_min_distance = std::numeric_limits<double>::infinity();
                for (const auto& c : qp_input.obstacle_constraints) {
                    if (std::isfinite(c.distance)) {
                        terminal_obstacle_min_distance =
                            std::min(terminal_obstacle_min_distance, c.distance);
                    }
                }
                if (!std::isfinite(terminal_obstacle_min_distance)) {
                    terminal_obstacle_min_distance = std::numeric_limits<double>::quiet_NaN();
                }
            }
            if (terminal_goal_capture_active &&
                runtime_cfg_.terminal_goal_capture_relax_obstacle_damper_when_far &&
                std::isfinite(terminal_obstacle_min_distance)) {
                terminal_obstacle_release_distance = std::max(
                    0.0,
                    request.hard_clearance + 1e-4);
                if (terminal_obstacle_min_distance > terminal_obstacle_release_distance) {
                    qp_build_cfg.obstacle_damper.safety_distance =
                        std::max(0.0, request.hard_clearance);
                    terminal_obstacle_relaxed = true;
                }
            }
        }

        rq::ReactiveQpProblem problem;
        if (!rq::ReactiveQpBuilder::build(qp_input, qp_build_cfg, problem, &error)) {
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
        if (!qdot_eigen.allFinite()) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] ReactiveTask abort: QP produced non-finite qdot.",
                mapping.c_str());
            break;
        }
        for (int i = 0; i < dof; ++i) {
            qdot_eigen(i) = std::clamp(qdot_eigen(i), ctx->qd_min(i), ctx->qd_max(i));
            qdot_cmd[static_cast<std::size_t>(i)] = qdot_eigen(i);
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
        const double pos_err = pos_err_goal;
        const double ori_err = ori_err_goal;

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
            double ee_clearance = std::numeric_limits<double>::quiet_NaN();
            if (runtime_cfg_.map_source != "dummy") {
                const auto query = map->queryDistanceAndGradient(fk_now.ee_position);
                if (query.observed && query.distance_valid) {
                    ee_clearance = query.distance - request.safe_distance;
                }
            } else if (runtime_cfg_.enable_dummy_obstacle && runtime_cfg_.dummy_obstacle_radius > 0.0) {
                const Eigen::Vector3d center = (mapping == "right_arm")
                                                   ? runtime_cfg_.dummy_obstacle_center_right_arm
                                                   : runtime_cfg_.dummy_obstacle_center_left_arm;
                ee_clearance = (fk_now.ee_position - center).norm() -
                               runtime_cfg_.dummy_obstacle_radius - request.safe_distance;
            }
            double whole_body_min_margin = std::numeric_limits<double>::quiet_NaN();
            bool whole_body_collision_free = false;
            std::string whole_body_state = "validator_unavailable";
            if (whole_body_validator.has_value()) {
                const auto diag = whole_body_validator->diagnoseJointState(q_now, request.safe_distance);
                whole_body_min_margin = diag.min_margin;
                whole_body_collision_free = diag.collision_free;
                whole_body_state = diag.reason;
            }
            publishTrajectoryMarkers(mapping, replanner, sample, fk_now.ee_position);
            RCLCPP_INFO(
                node_->get_logger(),
                "[%s] safety_tick: planner_tick=%d neo_iter=%d pos_err=%.5f ori_err=%.5f qdot_norm=%.5f qdot_max=%.5f qdot_limit_violation=%s joint_margin_min=%.5f task_residual_norm=%.6f ee_clearance=%.5f whole_body_margin=%.5f whole_body_collision_free=%s whole_body_state=%s terminal_goal_capture=%s terminal_position_priority=%s terminal_position_release_tol=%.5f terminal_orientation_blend=%.3f",
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
                ee_clearance,
                whole_body_min_margin,
                whole_body_collision_free ? "true" : "false",
                whole_body_state.c_str(),
                terminal_goal_capture_active ? "true" : "false",
                terminal_position_priority ? "true" : "false",
                terminal_position_release_tolerance,
                terminal_orientation_blend);
            if (terminal_goal_capture_active) {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "[%s] terminal_goal_capture_obstacle: min_distance=%.5f release_threshold=%.5f relaxed=%s active_safety_distance=%.5f",
                    mapping.c_str(),
                    terminal_obstacle_min_distance,
                    terminal_obstacle_release_distance,
                    terminal_obstacle_relaxed ? "true" : "false",
                    qp_build_cfg.obstacle_damper.safety_distance);
            }
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
                if (ticks_in_chunk >= chunk_ticks &&
                    !(terminal_goal_capture_active &&
                      runtime_cfg_.terminal_goal_capture_suppress_replanning)) {
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
            } else if (!pending_plan_inflight &&
                       !(terminal_goal_capture_active &&
                         runtime_cfg_.terminal_goal_capture_suppress_replanning)) {
                launch_pending_chunk_plan();
            }
        }
    }

    if (!reached_goal && sample_ok && planner_tick >= effective_max_planner_ticks) {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] ReactiveTask stopped by planner tick budget: planner_tick=%d effective_max_ticks=%d active_duration=%.3f s planner_dt=%.3f s",
            mapping.c_str(),
            planner_tick,
            effective_max_planner_ticks,
            active_duration_sec,
            planner_tick_sec);
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

void ReactiveTaskController::publishCollisionEllipsoidMarkers(
    const std::string& mapping,
    const Eigen::VectorXd& q_current,
    const MappingContext& ctx) {
    if (!collision_ellipsoid_marker_pub_ || !ctx.fk_provider ||
        ctx.collision_ellipsoids.empty() || q_current.size() <= 0) {
        return;
    }

    arm_controller::kinematics::ForwardKinematicsOutput fk_out;
    if (!ctx.fk_provider->compute(q_current, fk_out)) {
        return;
    }

    visualization_msgs::msg::MarkerArray array_msg;
    const int base_id = markerBaseIdForMapping(mapping) + 1000;
    std::size_t marker_count = 0u;

    for (const auto& ellipsoid : ctx.collision_ellipsoids) {
        const auto it = fk_out.link_poses.find(ellipsoid.link_name);
        if (it == fk_out.link_poses.end()) {
            continue;
        }

        const Eigen::Isometry3d& T_world_link = it->second;
        const Eigen::Vector3d center_world = T_world_link * ellipsoid.center_in_link;
        const Eigen::Quaterniond q_world(T_world_link.linear());

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node_->now();
        marker.ns = "reactive_task_collision_ellipsoids";
        marker.id = base_id + static_cast<int>(marker_count);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = center_world.x();
        marker.pose.position.y = center_world.y();
        marker.pose.position.z = center_world.z();
        marker.pose.orientation.x = q_world.x();
        marker.pose.orientation.y = q_world.y();
        marker.pose.orientation.z = q_world.z();
        marker.pose.orientation.w = q_world.w();
        marker.scale.x = 2.0 * ellipsoid.radii.x();
        marker.scale.y = 2.0 * ellipsoid.radii.y();
        marker.scale.z = 2.0 * ellipsoid.radii.z();
        marker.color.r = 1.0f;
        marker.color.g = 0.1f;
        marker.color.b = 0.1f;
        marker.color.a = 0.65f;
        array_msg.markers.push_back(marker);
        ++marker_count;
    }

    {
        std::lock_guard<std::mutex> lock(collision_ellipsoid_marker_mutex_);
        const std::size_t previous_count = collision_ellipsoid_marker_counts_[mapping];
        for (std::size_t i = marker_count; i < previous_count; ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = node_->now();
            marker.ns = "reactive_task_collision_ellipsoids";
            marker.id = base_id + static_cast<int>(i);
            marker.action = visualization_msgs::msg::Marker::DELETE;
            array_msg.markers.push_back(marker);
        }
        collision_ellipsoid_marker_counts_[mapping] = marker_count;
    }

    collision_ellipsoid_marker_pub_->publish(array_msg);
}

void ReactiveTaskController::clearCollisionEllipsoidMarkers(
    const std::string& mapping) {
    if (!collision_ellipsoid_marker_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray array_msg;
    const int base_id = markerBaseIdForMapping(mapping) + 1000;
    std::size_t previous_count = 0u;
    {
        std::lock_guard<std::mutex> lock(collision_ellipsoid_marker_mutex_);
        const auto it = collision_ellipsoid_marker_counts_.find(mapping);
        if (it != collision_ellipsoid_marker_counts_.end()) {
            previous_count = it->second;
            collision_ellipsoid_marker_counts_.erase(it);
        }
    }
    for (std::size_t i = 0; i < previous_count; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node_->now();
        marker.ns = "reactive_task_collision_ellipsoids";
        marker.id = base_id + static_cast<int>(i);
        marker.action = visualization_msgs::msg::Marker::DELETE;
        array_msg.markers.push_back(marker);
    }
    if (!array_msg.markers.empty()) {
        collision_ellipsoid_marker_pub_->publish(array_msg);
    }
}

void ReactiveTaskController::publishTrajectoryMarkers(
    const std::string& mapping,
    const cp::ReplannerManager& replanner,
    const cp::TimedCartesianSample& sample,
    const Eigen::Vector3d& ee_position) {
    if (!trajectory_marker_pub_) {
        return;
    }
    if (!replanner.hasActiveTrajectory()) {
        clearTrajectoryMarkers(mapping);
        return;
    }

    visualization_msgs::msg::MarkerArray array_msg;
    const int base_id = markerBaseIdForMapping(mapping) + 2000;
    const auto stamp = node_->now();

    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = stamp;
    path_marker.ns = "reactive_task_active_path";
    path_marker.id = base_id;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.01;
    path_marker.color.r = 0.05f;
    path_marker.color.g = 0.85f;
    path_marker.color.b = 0.95f;
    path_marker.color.a = 0.95f;
    const double total_duration_sec = replanner.activeSegmentTotalDurationSec();
    const double marker_sample_dt_sec = 0.05;
    const int dense_point_count = std::max(
        2,
        std::min(
            800,
            static_cast<int>(std::ceil(total_duration_sec / marker_sample_dt_sec)) + 1));
    path_marker.points.reserve(static_cast<std::size_t>(dense_point_count));
    for (int i = 0; i < dense_point_count; ++i) {
        const double alpha = (dense_point_count <= 1)
                                 ? 0.0
                                 : static_cast<double>(i) /
                                       static_cast<double>(dense_point_count - 1);
        const double t = alpha * total_duration_sec;
        cp::TimedCartesianSample path_sample;
        if (!replanner.sampleByElapsedTime(t, path_sample)) {
            continue;
        }
        path_marker.points.push_back(toPointMsg(path_sample.T_target.translation()));
    }
    if (path_marker.points.size() >= 2u) {
        array_msg.markers.push_back(path_marker);
    } else {
        path_marker.action = visualization_msgs::msg::Marker::DELETE;
        array_msg.markers.push_back(path_marker);
    }

    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = "world";
    target_marker.header.stamp = stamp;
    target_marker.ns = "reactive_task_target_point";
    target_marker.id = base_id + 1;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.pose.position = toPointMsg(sample.T_target.translation());
    target_marker.pose.orientation.w = 1.0;
    target_marker.scale.x = 0.035;
    target_marker.scale.y = 0.035;
    target_marker.scale.z = 0.035;
    target_marker.color.r = 1.0f;
    target_marker.color.g = 0.82f;
    target_marker.color.b = 0.10f;
    target_marker.color.a = 0.95f;
    array_msg.markers.push_back(target_marker);

    visualization_msgs::msg::Marker actual_marker;
    actual_marker.header.frame_id = "world";
    actual_marker.header.stamp = stamp;
    actual_marker.ns = "reactive_task_actual_point";
    actual_marker.id = base_id + 2;
    actual_marker.type = visualization_msgs::msg::Marker::SPHERE;
    actual_marker.action = visualization_msgs::msg::Marker::ADD;
    actual_marker.pose.position = toPointMsg(ee_position);
    actual_marker.pose.orientation.w = 1.0;
    actual_marker.scale.x = 0.03;
    actual_marker.scale.y = 0.03;
    actual_marker.scale.z = 0.03;
    actual_marker.color.r = 0.10f;
    actual_marker.color.g = 1.0f;
    actual_marker.color.b = 0.25f;
    actual_marker.color.a = 0.95f;
    array_msg.markers.push_back(actual_marker);

    trajectory_marker_pub_->publish(array_msg);
}

void ReactiveTaskController::clearTrajectoryMarkers(const std::string& mapping) {
    if (!trajectory_marker_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray array_msg;
    const int base_id = markerBaseIdForMapping(mapping) + 2000;
    const char* namespaces[] = {
        "reactive_task_active_path",
        "reactive_task_target_point",
        "reactive_task_actual_point",
    };
    for (int i = 0; i < 3; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node_->now();
        marker.ns = namespaces[i];
        marker.id = base_id + i;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        array_msg.markers.push_back(marker);
    }
    trajectory_marker_pub_->publish(array_msg);
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
