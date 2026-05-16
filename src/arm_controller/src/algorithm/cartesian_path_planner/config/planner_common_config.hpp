#pragma once

namespace arm_controller::algorithm::cartesian_path_planner {

struct PlannerCommonConfig {
    double path_resolution{0.02};
    double max_path_length{20.0};
    double default_segment_speed{0.1}; // m/s
    bool use_joint_space_sampling{false};
    int joint_space_sampling_max_iterations{4000};
    double joint_space_sampling_step_rad{0.20};
    double joint_space_sampling_goal_bias{0.20};
    double joint_space_sampling_connect_threshold_rad{0.25};
    double joint_space_sampling_local_window_rad{0.35};
    int joint_space_sampling_search_stages{3};
    double joint_space_sampling_window_scale{2.0};
    bool joint_space_sampling_allow_full_joint_limit_fallback{true};
    int joint_space_shortcut_trials{100};
    int joint_space_sampling_solution_pool_size{6};
    double joint_space_path_length_weight{1.0};
    double joint_space_joint_motion_weight{0.15};
    double joint_space_clearance_deficit_weight{2.5};
    double joint_space_early_clearance_deficit_weight{4.0};
    double joint_space_min_clearance_deficit_weight{8.0};
    double joint_space_early_min_clearance_deficit_weight{12.0};
    double joint_space_clearance_reward_weight{0.0};
    double joint_space_clearance_reward_cap_m{0.08};
    double joint_space_preferred_min_margin_m{0.020};
    double joint_space_min_margin_preference_weight{50.0};
    bool joint_space_start_recovery_enable{true};
    int joint_space_start_recovery_samples{96};
    double joint_space_start_recovery_target_margin_m{0.020};
    double joint_space_sampling_time_budget_sec{1.50};
    bool enable_joint_trajectory_post_optimization{true};
    int joint_trajectory_postopt_iterations{2};
    int joint_trajectory_postopt_samples_per_waypoint{12};
    double joint_trajectory_postopt_perturbation_rad{0.05};
    double joint_trajectory_orientation_weight{3.0};
    double joint_trajectory_smoothness_weight{0.25};
    double joint_trajectory_position_weight{1.0};

    // Use trajectory_interpolator to smooth Cartesian waypoints in time domain.
    bool enable_interpolator_smoothing{true};
    // 2 -> C2 cubic spline, 1 -> C1 cubic Hermite.
    int interpolator_continuity_order{2};
    // Re-sampling interval for interpolated Cartesian trajectory.
    double interpolator_target_dt{0.01};

    // Optional discrete minimum-snap smoother applied after timing /
    // interpolation. The objective penalizes the fourth-order finite
    // difference of the Cartesian samples while keeping them close to the
    // input trajectory.
    bool enable_minimum_snap_optimization{false};
    int minimum_snap_iterations{80};
    double minimum_snap_data_weight{0.20};
    double minimum_snap_weight{0.05};
    double minimum_snap_relaxation{0.50};

};

}  // namespace arm_controller::algorithm::cartesian_path_planner
