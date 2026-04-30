#pragma once

namespace arm_controller::algorithm::cartesian_path_planner {

struct PlannerCommonConfig {
    double path_resolution{0.02};
    double max_path_length{20.0};
    double default_segment_speed{0.1}; // m/s
    // B-scheme: keep the front-end as geometric search only, and move
    // whole-body / IK feasibility to the trajectory back-end.
    bool frontend_geometry_only{false};
    // Begin rotating from start pose toward goal pose after this fraction
    // of the geometric path has been traversed.
    double goal_orientation_transition_start_ratio{0.60};

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
