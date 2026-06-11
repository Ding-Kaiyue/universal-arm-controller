#include "controller/reactive_task/reactive_task_controller.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

#include <yaml-cpp/yaml.h>

namespace rq = arm_controller::algorithm::reactive_qp;

namespace {

bool parseVec3(const YAML::Node &node, Eigen::Vector3d &out) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    return false;
  }
  out << node[0].as<double>(), node[1].as<double>(), node[2].as<double>();
  return true;
}

void normalizeSourceName(std::string &source) {
  std::transform(
      source.begin(), source.end(), source.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
}

bool isValidMapSource(const std::string &source) {
  return source == "dummy" || source == "camera_driver_pointcloud" ||
         source == "camera_driver_esdf";
}

bool loadRuntimeConfigFromYaml(
    const YAML::Node &root,
    ReactiveTaskController::ControllerRuntimeConfig &cfg, std::string *error) {
  auto setError = [&](const std::string &msg) {
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
    setError(
        "Missing required key: reactive_task_controller.max_control_ticks");
    return false;
  }
  cfg.max_control_ticks = std::max(1, rtc["max_control_ticks"].as<int>());
  if (rtc["neo_control_cycle_sec"]) {
    cfg.neo_control_cycle_sec =
        std::max(1e-4, rtc["neo_control_cycle_sec"].as<double>());
    cfg.global_trajectory.control_cycle_sec = cfg.neo_control_cycle_sec;
  }
  if (!rtc["goal_position_tolerance"]) {
    setError("Missing required key: "
             "reactive_task_controller.goal_position_tolerance");
    return false;
  }
  cfg.goal_position_tolerance =
      std::max(1e-6, rtc["goal_position_tolerance"].as<double>());
  if (!rtc["goal_orientation_tolerance_rad"]) {
    setError("Missing required key: "
             "reactive_task_controller.goal_orientation_tolerance_rad");
    return false;
  }
  cfg.goal_orientation_tolerance_rad =
      std::max(1e-6, rtc["goal_orientation_tolerance_rad"].as<double>());
  if (const YAML::Node controller = rtc["controller"];
      controller && controller.IsMap()) {
    if (controller["frequency_hz"]) {
      const double controller_hz =
          std::max(1e-3, controller["frequency_hz"].as<double>());
      cfg.neo_control_cycle_sec = 1.0 / controller_hz;
      cfg.global_trajectory.control_cycle_sec = cfg.neo_control_cycle_sec;
    }
  }
  if (const YAML::Node feedback = rtc["feedback"];
      feedback && feedback.IsMap()) {
    if (feedback["arm_state_source"]) {
      cfg.arm_state_source = feedback["arm_state_source"].as<std::string>();
      normalizeSourceName(cfg.arm_state_source);
    }
    if (feedback["command_output"]) {
      cfg.command_output = feedback["command_output"].as<std::string>();
      normalizeSourceName(cfg.command_output);
    }
    if (feedback["joint_state_topic"]) {
      cfg.joint_state_topic = feedback["joint_state_topic"].as<std::string>();
    }
    if (feedback["left_arm_velocity_command_topic"]) {
      cfg.left_arm_velocity_command_topic =
          feedback["left_arm_velocity_command_topic"].as<std::string>();
    }
    if (feedback["right_arm_velocity_command_topic"]) {
      cfg.right_arm_velocity_command_topic =
          feedback["right_arm_velocity_command_topic"].as<std::string>();
    }
    if (feedback["arm_velocity_command_topic"]) {
      cfg.arm_velocity_command_topic =
          feedback["arm_velocity_command_topic"].as<std::string>();
    }
    if (feedback["joint_state_stale_timeout_sec"]) {
      cfg.joint_state_stale_timeout_sec =
          std::max(0.01, feedback["joint_state_stale_timeout_sec"].as<double>());
    }
  }
  if (cfg.arm_state_source != "hardware" &&
      cfg.arm_state_source != "joint_states") {
    setError("Invalid reactive_task_controller.feedback.arm_state_source, "
             "expected 'hardware' or 'joint_states'");
    return false;
  }
  if (cfg.command_output != "real" && cfg.command_output != "gazebo") {
    setError("Invalid reactive_task_controller.feedback.command_output, "
             "expected 'real' or 'gazebo'");
    return false;
  }
  if (const YAML::Node whole_body = rtc["whole_body"];
      whole_body && whole_body.IsMap()) {
    if (whole_body["enable_mobile_base_in_planning"]) {
      cfg.enable_mobile_base_in_planning =
          whole_body["enable_mobile_base_in_planning"].as<bool>();
    }
    if (whole_body["enable_mobile_base_in_neo"]) {
      cfg.enable_mobile_base_in_neo =
          whole_body["enable_mobile_base_in_neo"].as<bool>();
    }
    if (whole_body["mobile_base_type"]) {
      cfg.mobile_base_type = whole_body["mobile_base_type"].as<std::string>();
      normalizeSourceName(cfg.mobile_base_type);
    }
    if (whole_body["mobile_base_state_source"]) {
      cfg.mobile_base_state_source =
          whole_body["mobile_base_state_source"].as<std::string>();
      normalizeSourceName(cfg.mobile_base_state_source);
    }
    if (whole_body["mobile_base_odom_frame"]) {
      cfg.mobile_base_odom_frame =
          whole_body["mobile_base_odom_frame"].as<std::string>();
    }
    if (whole_body["mobile_base_frame"]) {
      cfg.mobile_base_frame = whole_body["mobile_base_frame"].as<std::string>();
    }
    if (whole_body["cmd_vel_topic"]) {
      cfg.cmd_vel_topic = whole_body["cmd_vel_topic"].as<std::string>();
    }
    if (whole_body["base_max_vx"]) {
      cfg.base_max_vx = std::max(0.0, whole_body["base_max_vx"].as<double>());
    }
    if (whole_body["base_max_vy"]) {
      cfg.base_max_vy = std::max(0.0, whole_body["base_max_vy"].as<double>());
    }
    if (whole_body["base_max_wz"]) {
      cfg.base_max_wz = std::max(0.0, whole_body["base_max_wz"].as<double>());
    }
    if (whole_body["base_velocity_weight_scale"]) {
      cfg.base_velocity_weight_scale = std::max(
          1e-6, whole_body["base_velocity_weight_scale"].as<double>());
    }
  }
  if (const YAML::Node global = rtc["global_planner"];
      global && global.IsMap()) {
    if (global["planning_latency_sec"]) {
      cfg.global_trajectory.planning_latency_sec =
          std::max(0.0, global["planning_latency_sec"].as<double>());
    }
  }
  if (const YAML::Node local = rtc["local_planner"]; local && local.IsMap()) {
    if (local["frequency_hz"]) {
      const double local_hz =
          std::max(1e-3, local["frequency_hz"].as<double>());
      cfg.local_planner.update_period_sec = 1.0 / local_hz;
      cfg.local_planner.dt_sec = cfg.local_planner.update_period_sec;
    }
    if (local["horizon_steps"]) {
      cfg.local_planner.horizon_steps =
          std::max(2, local["horizon_steps"].as<int>());
    }
    if (local["enable_collision_cost"]) {
      cfg.local_planner.enable_collision_cost =
          local["enable_collision_cost"].as<bool>();
    }
    if (local["enable_collision_constraint"]) {
      cfg.local_planner.enable_collision_constraint =
          local["enable_collision_constraint"].as<bool>();
    }
    if (local["max_obstacle_spheres"]) {
      cfg.local_planner.max_obstacle_spheres =
          std::max(0, local["max_obstacle_spheres"].as<int>());
    }
    if (local["obstacle_selection_radius_m"]) {
      cfg.local_planner.obstacle_selection_radius_m =
          std::max(0.01, local["obstacle_selection_radius_m"].as<double>());
    }
    if (local["obstacle_padding_m"]) {
      cfg.local_planner.obstacle_padding_m =
          std::max(0.0, local["obstacle_padding_m"].as<double>());
    }
  }
  if (const YAML::Node freq = rtc["frequencies"]; freq && freq.IsMap()) {
    if (freq["neo_hz"]) {
      const double neo_hz = std::max(1e-3, freq["neo_hz"].as<double>());
      cfg.neo_control_cycle_sec = 1.0 / neo_hz;
      cfg.global_trajectory.control_cycle_sec = cfg.neo_control_cycle_sec;
    }
    if (freq["trajopt_hz"]) {
      const double trajopt_hz = std::max(1e-3, freq["trajopt_hz"].as<double>());
      cfg.local_planner.update_period_sec = 1.0 / trajopt_hz;
      cfg.local_planner.dt_sec = cfg.local_planner.update_period_sec;
    }
    if (freq["rrt_connect_hz"]) {
      // Deprecated: RRT-Connect is the global planner and is not run
      // periodically. Keep accepting the old key without changing behavior.
    }
  }
  if (const YAML::Node local_planner = rtc["local_planner"];
      local_planner && local_planner.IsMap()) {
    if (local_planner["horizon_steps"]) {
      cfg.local_planner.horizon_steps =
          std::max(2, local_planner["horizon_steps"].as<int>());
    }
    if (local_planner["update_period_sec"]) {
      cfg.local_planner.update_period_sec =
          std::max(1e-3, local_planner["update_period_sec"].as<double>());
    }
    if (local_planner["dt_sec"]) {
      cfg.local_planner.dt_sec =
          std::max(1e-3, local_planner["dt_sec"].as<double>());
    }
    if (local_planner["enable_collision_cost"]) {
      cfg.local_planner.enable_collision_cost =
          local_planner["enable_collision_cost"].as<bool>();
    }
    if (local_planner["enable_collision_constraint"]) {
      cfg.local_planner.enable_collision_constraint =
          local_planner["enable_collision_constraint"].as<bool>();
    }
    if (local_planner["max_obstacle_spheres"]) {
      cfg.local_planner.max_obstacle_spheres =
          std::max(0, local_planner["max_obstacle_spheres"].as<int>());
    }
    if (local_planner["obstacle_selection_radius_m"]) {
      cfg.local_planner.obstacle_selection_radius_m =
          std::max(0.01,
                   local_planner["obstacle_selection_radius_m"].as<double>());
    }
    if (local_planner["obstacle_padding_m"]) {
      cfg.local_planner.obstacle_padding_m =
          std::max(0.0, local_planner["obstacle_padding_m"].as<double>());
    }
  }
  if (const YAML::Node req = rtc["request"]; req && req.IsMap()) {
    if (!req["safe_distance"]) {
      setError("Missing required key: "
               "reactive_task_controller.request.safe_distance");
      return false;
    }
    cfg.request_safe_distance =
        std::max(0.0, req["safe_distance"].as<double>());
    cfg.request_planning_safe_distance = cfg.request_safe_distance;
    if (req["planning_safe_distance"]) {
      cfg.request_planning_safe_distance =
          std::max(cfg.request_safe_distance,
                   req["planning_safe_distance"].as<double>());
    }
    if (req["hard_clearance"]) {
      cfg.request_hard_clearance =
          std::max(0.0, req["hard_clearance"].as<double>());
    }
    if (!req["goal_tolerance"]) {
      setError("Missing required key: "
               "reactive_task_controller.request.goal_tolerance");
      return false;
    }
    cfg.request_goal_tolerance =
        std::max(1e-6, req["goal_tolerance"].as<double>());
    if (!req["map_margin_xyz"]) {
      setError("Missing required key: "
               "reactive_task_controller.request.map_margin_xyz");
      return false;
    }
    Eigen::Vector3d margin;
    if (!parseVec3(req["map_margin_xyz"], margin)) {
      setError("Invalid reactive_task_controller.request.map_margin_xyz, "
               "expected [x,y,z]");
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
    setError("reactive_task_controller.map_source is deprecated; use "
             "distance_field_source and collision_map_source");
    return false;
  }
  if (rtc["distance_field_source"]) {
    cfg.distance_field_source = rtc["distance_field_source"].as<std::string>();
    normalizeSourceName(cfg.distance_field_source);
  }
  if (!isValidMapSource(cfg.distance_field_source)) {
    setError("Invalid reactive_task_controller.distance_field_source, expected "
             "'dummy', 'camera_driver_pointcloud', or 'camera_driver_esdf'");
    return false;
  }
  if (rtc["collision_map_source"]) {
    cfg.collision_map_source = rtc["collision_map_source"].as<std::string>();
    normalizeSourceName(cfg.collision_map_source);
  }
  if (!isValidMapSource(cfg.collision_map_source)) {
    setError("Invalid reactive_task_controller.collision_map_source, expected "
             "'dummy', 'camera_driver_pointcloud', or 'camera_driver_esdf'");
    return false;
  }

  if (const YAML::Node obs = rtc["dummy_obstacle"]; obs && obs.IsMap()) {
    if (!obs["enable"] || !obs["radius"] || !obs["center_left_arm"] ||
        !obs["center_right_arm"]) {
      setError("Missing required keys under "
               "reactive_task_controller.dummy_obstacle");
      return false;
    }
    cfg.enable_dummy_obstacle = obs["enable"].as<bool>();
    cfg.dummy_obstacle_radius = std::max(0.0, obs["radius"].as<double>());
    if (!parseVec3(obs["center_left_arm"],
                   cfg.dummy_obstacle_center_left_arm)) {
      setError(
          "Invalid reactive_task_controller.dummy_obstacle.center_left_arm, "
          "expected [x,y,z]");
      return false;
    }
    if (!parseVec3(obs["center_right_arm"],
                   cfg.dummy_obstacle_center_right_arm)) {
      setError(
          "Invalid reactive_task_controller.dummy_obstacle.center_right_arm, "
          "expected [x,y,z]");
      return false;
    }
  } else {
    setError("Missing required map: reactive_task_controller.dummy_obstacle");
    return false;
  }

  if (const YAML::Node live = rtc["camera_driver_pointcloud"];
      live && live.IsMap()) {
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
          std::max(cfg.camera_driver_pointcloud.voxel_size_m,
                   live["max_distance_m"].as<double>());
    }
    if (live["observation_margin_m"]) {
      cfg.camera_driver_pointcloud.observation_margin_m =
          std::max(0.0, live["observation_margin_m"].as<double>());
    }
    if (live["occupancy_retention_sec"]) {
      cfg.camera_driver_pointcloud.occupancy_retention_sec =
          std::max(0.0, live["occupancy_retention_sec"].as<double>());
    }
    if (live["max_cached_cells"]) {
      cfg.camera_driver_pointcloud.max_cached_cells =
          std::max<std::size_t>(
              1u, live["max_cached_cells"].as<std::size_t>());
    }
    if (live["accumulate_observed_bounds"]) {
      cfg.camera_driver_pointcloud.accumulate_observed_bounds =
          live["accumulate_observed_bounds"].as<bool>();
    }
    if (live["isolated_min_neighbor_count"]) {
      cfg.camera_driver_pointcloud.isolated_min_neighbor_count =
          std::max(0, live["isolated_min_neighbor_count"].as<int>());
    }
    if (live["isolated_neighbor_radius_cells"]) {
      cfg.camera_driver_pointcloud.isolated_neighbor_radius_cells =
          std::max(1, live["isolated_neighbor_radius_cells"].as<int>());
    }
    if (live["min_cluster_cell_count"]) {
      cfg.camera_driver_pointcloud.min_cluster_cell_count =
          std::max(1, live["min_cluster_cell_count"].as<int>());
    }
  } else if (cfg.collision_map_source == "camera_driver_pointcloud" ||
             cfg.distance_field_source == "camera_driver_pointcloud") {
    setError("Missing required map: "
             "reactive_task_controller.camera_driver_pointcloud");
    return false;
  }

  if (const YAML::Node live = rtc["camera_driver_esdf"]; live && live.IsMap()) {
    if (live["shm_name"]) {
      cfg.camera_driver_esdf.shm_name = live["shm_name"].as<std::string>();
    }
    if (live["cache_max_entries"]) {
      cfg.camera_driver_esdf.cache_max_entries = std::max<std::size_t>(
          1u, live["cache_max_entries"].as<std::size_t>());
    }
  } else if (cfg.distance_field_source == "camera_driver_esdf" ||
             cfg.collision_map_source == "camera_driver_esdf") {
    setError(
        "Missing required map: reactive_task_controller.camera_driver_esdf");
    return false;
  }

  const YAML::Node global_trajectory_node =
      rtc["global_trajectory"] ? rtc["global_trajectory"] : rtc["replanner"];
  if (const YAML::Node rep = global_trajectory_node; rep && rep.IsMap()) {
    if (!rep["planning_latency_sec"]) {
      setError(
          "Missing required keys under reactive_task_controller.global_trajectory");
      return false;
    }
    if (rep["control_cycle_sec"]) {
      cfg.global_trajectory.control_cycle_sec =
          std::max(1e-4, rep["control_cycle_sec"].as<double>());
    }
    cfg.global_trajectory.planning_latency_sec =
        std::max(0.0, rep["planning_latency_sec"].as<double>());
  }

  if (const YAML::Node planner = rtc["planner"]; planner && planner.IsMap()) {
    if (const YAML::Node common = planner["common"]; common && common.IsMap()) {
      if (!common["default_segment_speed"] ||
          !common["enable_interpolator_smoothing"] ||
          !common["interpolator_continuity_order"] ||
          !common["interpolator_target_dt"]) {
        setError("Missing required keys under "
                 "reactive_task_controller.planner.common");
        return false;
      }
      cfg.planner_common.default_segment_speed =
          common["default_segment_speed"].as<double>();
      if (common["use_joint_space_sampling"]) {
        cfg.planner_common.use_joint_space_sampling =
            common["use_joint_space_sampling"].as<bool>();
      }
      if (common["joint_space_sampling_max_iterations"]) {
        cfg.planner_common.joint_space_sampling_max_iterations = std::max(
            1, common["joint_space_sampling_max_iterations"].as<int>());
      }
      if (common["joint_space_sampling_step_rad"]) {
        cfg.planner_common.joint_space_sampling_step_rad = std::max(
            1e-3, common["joint_space_sampling_step_rad"].as<double>());
      }
      if (common["joint_space_sampling_goal_bias"]) {
        cfg.planner_common.joint_space_sampling_goal_bias = std::clamp(
            common["joint_space_sampling_goal_bias"].as<double>(), 0.0, 1.0);
      }
      if (common["joint_space_sampling_connect_threshold_rad"]) {
        cfg.planner_common.joint_space_sampling_connect_threshold_rad =
            std::max(1e-3, common["joint_space_sampling_connect_threshold_rad"]
                               .as<double>());
      }
      if (common["joint_space_sampling_local_window_rad"]) {
        cfg.planner_common.joint_space_sampling_local_window_rad = std::max(
            1e-3, common["joint_space_sampling_local_window_rad"].as<double>());
      }
      if (common["joint_space_sampling_search_stages"]) {
        cfg.planner_common.joint_space_sampling_search_stages =
            std::max(1, common["joint_space_sampling_search_stages"].as<int>());
      }
      if (common["enable_two_way_bypass"]) {
        cfg.planner_common.enable_two_way_bypass =
            common["enable_two_way_bypass"].as<bool>();
      }
      if (common["two_way_bypass_offset_m"]) {
        cfg.planner_common.two_way_bypass_offset_m =
            std::max(0.0, common["two_way_bypass_offset_m"].as<double>());
      }
      if (common["joint_space_sampling_window_scale"]) {
        cfg.planner_common.joint_space_sampling_window_scale = std::max(
            1.0, common["joint_space_sampling_window_scale"].as<double>());
      }
      if (common["joint_space_sampling_allow_full_joint_limit_fallback"]) {
        cfg.planner_common
            .joint_space_sampling_allow_full_joint_limit_fallback =
            common["joint_space_sampling_allow_full_joint_limit_fallback"]
                .as<bool>();
      }
      if (common["joint_space_shortcut_trials"]) {
        cfg.planner_common.joint_space_shortcut_trials =
            std::max(0, common["joint_space_shortcut_trials"].as<int>());
      }
      if (common["joint_space_sampling_solution_pool_size"]) {
        cfg.planner_common.joint_space_sampling_solution_pool_size = std::max(
            1, common["joint_space_sampling_solution_pool_size"].as<int>());
      }
      if (common["joint_space_path_length_weight"]) {
        cfg.planner_common.joint_space_path_length_weight = std::max(
            0.0, common["joint_space_path_length_weight"].as<double>());
      }
      if (common["joint_space_joint_motion_weight"]) {
        cfg.planner_common.joint_space_joint_motion_weight = std::max(
            0.0, common["joint_space_joint_motion_weight"].as<double>());
      }
      if (common["joint_space_clearance_deficit_weight"]) {
        cfg.planner_common.joint_space_clearance_deficit_weight = std::max(
            0.0, common["joint_space_clearance_deficit_weight"].as<double>());
      }
      if (common["joint_space_early_clearance_deficit_weight"]) {
        cfg.planner_common.joint_space_early_clearance_deficit_weight =
            std::max(0.0, common["joint_space_early_clearance_deficit_weight"]
                              .as<double>());
      }
      if (common["joint_space_min_clearance_deficit_weight"]) {
        cfg.planner_common.joint_space_min_clearance_deficit_weight = std::max(
            0.0,
            common["joint_space_min_clearance_deficit_weight"].as<double>());
      }
      if (common["joint_space_early_min_clearance_deficit_weight"]) {
        cfg.planner_common.joint_space_early_min_clearance_deficit_weight =
            std::max(0.0,
                     common["joint_space_early_min_clearance_deficit_weight"]
                         .as<double>());
      }
      if (common["joint_space_clearance_reward_weight"]) {
        cfg.planner_common.joint_space_clearance_reward_weight = std::max(
            0.0, common["joint_space_clearance_reward_weight"].as<double>());
      }
      if (common["joint_space_clearance_reward_cap_m"]) {
        cfg.planner_common.joint_space_clearance_reward_cap_m = std::max(
            1e-6, common["joint_space_clearance_reward_cap_m"].as<double>());
      }
      if (common["joint_space_preferred_min_margin_m"]) {
        cfg.planner_common.joint_space_preferred_min_margin_m = std::max(
            0.0, common["joint_space_preferred_min_margin_m"].as<double>());
      }
      if (common["joint_space_min_margin_preference_weight"]) {
        cfg.planner_common.joint_space_min_margin_preference_weight = std::max(
            0.0,
            common["joint_space_min_margin_preference_weight"].as<double>());
      }
      if (common["joint_space_start_recovery_enable"]) {
        cfg.planner_common.joint_space_start_recovery_enable =
            common["joint_space_start_recovery_enable"].as<bool>();
      }
      if (common["joint_space_start_recovery_samples"]) {
        cfg.planner_common.joint_space_start_recovery_samples =
            std::max(1, common["joint_space_start_recovery_samples"].as<int>());
      }
      if (common["joint_space_start_recovery_target_margin_m"]) {
        cfg.planner_common.joint_space_start_recovery_target_margin_m =
            std::max(0.0, common["joint_space_start_recovery_target_margin_m"]
                              .as<double>());
      }
      if (common["joint_space_sampling_time_budget_sec"]) {
        cfg.planner_common.joint_space_sampling_time_budget_sec = std::max(
            0.0, common["joint_space_sampling_time_budget_sec"].as<double>());
      }
      if (common["joint_trajectory_orientation_weight"]) {
        cfg.planner_common.joint_trajectory_orientation_weight = std::max(
            0.0, common["joint_trajectory_orientation_weight"].as<double>());
      }
      if (common["joint_trajectory_smoothness_weight"]) {
        cfg.planner_common.joint_trajectory_smoothness_weight = std::max(
            0.0, common["joint_trajectory_smoothness_weight"].as<double>());
      }
      if (common["joint_trajectory_position_weight"]) {
        cfg.planner_common.joint_trajectory_position_weight = std::max(
            0.0, common["joint_trajectory_position_weight"].as<double>());
      }
      cfg.planner_common.enable_interpolator_smoothing =
          common["enable_interpolator_smoothing"].as<bool>();
      cfg.planner_common.interpolator_continuity_order =
          common["interpolator_continuity_order"].as<int>();
      cfg.planner_common.interpolator_target_dt =
          common["interpolator_target_dt"].as<double>();
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
        cfg.planner_common.minimum_snap_relaxation = std::clamp(
            common["minimum_snap_relaxation"].as<double>(), 1e-3, 1.0);
      }
    } else {
      setError("Missing required map: reactive_task_controller.planner.common");
      return false;
    }
    if (const YAML::Node smoothing = planner["smoothing"];
        smoothing && smoothing.IsMap()) {
      if (!smoothing["max_shortcut_trials"] ||
          !smoothing["collision_check_step"] ||
          !smoothing["local_adjust_iterations"] ||
          !smoothing["local_adjust_alpha"]) {
        setError("Missing required keys under "
                 "reactive_task_controller.planner.smoothing");
        return false;
      }
      cfg.planner_smoothing.max_shortcut_trials =
          smoothing["max_shortcut_trials"].as<int>();
      cfg.planner_smoothing.collision_check_step =
          smoothing["collision_check_step"].as<double>();
      cfg.planner_smoothing.local_adjust_iterations =
          smoothing["local_adjust_iterations"].as<int>();
      cfg.planner_smoothing.local_adjust_alpha =
          smoothing["local_adjust_alpha"].as<double>();
    } else {
      setError(
          "Missing required map: reactive_task_controller.planner.smoothing");
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

template <typename T>
T declareOrGetParam(
    const rclcpp::Node::SharedPtr &node,
    const std::string &name,
    const T &fallback) {
  if (!node) {
    return fallback;
  }
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, fallback);
  }
  return node->get_parameter(name).get_value<T>();
}

template <typename T>
void applyParamOverride(
    const rclcpp::Node::SharedPtr &node,
    const std::vector<std::string> &names,
    T *value) {
  if (value == nullptr) {
    return;
  }
  for (const std::string &name : names) {
    *value = declareOrGetParam<T>(node, name, *value);
  }
}

void applyRosParameterOverrides(
    const rclcpp::Node::SharedPtr &node,
    ReactiveTaskController::ControllerRuntimeConfig *cfg) {
  if (!node || cfg == nullptr) {
    return;
  }

  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.feedback.arm_state_source",
       "reactive_task.feedback.arm_state_source"},
      &cfg->arm_state_source);
  normalizeSourceName(cfg->arm_state_source);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.feedback.command_output",
       "reactive_task.feedback.command_output"},
      &cfg->command_output);
  normalizeSourceName(cfg->command_output);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.feedback.joint_state_topic",
       "reactive_task.feedback.joint_state_topic"},
      &cfg->joint_state_topic);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.feedback.left_arm_velocity_command_topic",
       "reactive_task.feedback.left_arm_velocity_command_topic"},
      &cfg->left_arm_velocity_command_topic);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.feedback.right_arm_velocity_command_topic",
       "reactive_task.feedback.right_arm_velocity_command_topic"},
      &cfg->right_arm_velocity_command_topic);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.feedback.arm_velocity_command_topic",
       "reactive_task.feedback.arm_velocity_command_topic"},
      &cfg->arm_velocity_command_topic);
  applyParamOverride<double>(
      node,
      {"reactive_task_controller.feedback.joint_state_stale_timeout_sec",
       "reactive_task.feedback.joint_state_stale_timeout_sec"},
      &cfg->joint_state_stale_timeout_sec);
  cfg->joint_state_stale_timeout_sec =
      std::max(0.01, cfg->joint_state_stale_timeout_sec);

  applyParamOverride<bool>(
      node,
      {"reactive_task_controller.whole_body.enable_mobile_base_in_planning",
       "reactive_task.whole_body.enable_mobile_base_in_planning"},
      &cfg->enable_mobile_base_in_planning);
  applyParamOverride<bool>(
      node,
      {"reactive_task_controller.whole_body.enable_mobile_base_in_neo",
       "reactive_task.whole_body.enable_mobile_base_in_neo"},
      &cfg->enable_mobile_base_in_neo);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.whole_body.mobile_base_type",
       "reactive_task.whole_body.mobile_base_type"},
      &cfg->mobile_base_type);
  normalizeSourceName(cfg->mobile_base_type);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.whole_body.mobile_base_state_source",
       "reactive_task.whole_body.mobile_base_state_source"},
      &cfg->mobile_base_state_source);
  normalizeSourceName(cfg->mobile_base_state_source);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.whole_body.mobile_base_odom_frame",
       "reactive_task.whole_body.mobile_base_odom_frame"},
      &cfg->mobile_base_odom_frame);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.whole_body.mobile_base_frame",
       "reactive_task.whole_body.mobile_base_frame"},
      &cfg->mobile_base_frame);
  applyParamOverride<std::string>(
      node,
      {"reactive_task_controller.whole_body.cmd_vel_topic",
       "reactive_task.whole_body.cmd_vel_topic"},
      &cfg->cmd_vel_topic);
  applyParamOverride<double>(
      node,
      {"reactive_task_controller.whole_body.base_max_vx",
       "reactive_task.whole_body.base_max_vx"},
      &cfg->base_max_vx);
  applyParamOverride<double>(
      node,
      {"reactive_task_controller.whole_body.base_max_vy",
       "reactive_task.whole_body.base_max_vy"},
      &cfg->base_max_vy);
  applyParamOverride<double>(
      node,
      {"reactive_task_controller.whole_body.base_max_wz",
       "reactive_task.whole_body.base_max_wz"},
      &cfg->base_max_wz);
  cfg->base_max_vx = std::max(0.0, cfg->base_max_vx);
  cfg->base_max_vy = std::max(0.0, cfg->base_max_vy);
  cfg->base_max_wz = std::max(0.0, cfg->base_max_wz);
}

} // namespace

bool ReactiveTaskController::loadReactiveConfig() {
  try {
    const std::string cfg_path =
        ament_index_cpp::get_package_share_directory("arm_controller") +
        "/config/reactive_task_config.yaml";
    std::string error;

    if (!rq::ReactiveQpExampleConfigLoader::loadFromYaml(
            cfg_path, reactive_cfg_, &error)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "reactive_task: failed to load reactive config '%s': %s.",
                   cfg_path.c_str(), error.c_str());
      return false;
    }

    const YAML::Node root = YAML::LoadFile(cfg_path);
    if (!loadRuntimeConfigFromYaml(root, runtime_cfg_, &error)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "reactive_task: failed to load runtime config '%s': %s.",
                   cfg_path.c_str(), error.c_str());
      return false;
    }
    applyRosParameterOverrides(node_, &runtime_cfg_);
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "reactive_task: load config exception: %s",
                 e.what());
    return false;
  }
}
