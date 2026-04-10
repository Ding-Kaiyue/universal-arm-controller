#pragma once

namespace arm_controller::algorithm::cartesian_path_planner {

struct PlannerCommonConfig {
    double path_resolution{0.02};
    double max_path_length{20.0};
    double default_segment_speed{0.1}; // m/s

    // Use trajectory_interpolator to smooth Cartesian waypoints in time domain.
    bool enable_interpolator_smoothing{true};
    // 2 -> C2 cubic spline, 1 -> C1 cubic Hermite.
    int interpolator_continuity_order{2};
    // Re-sampling interval for interpolated Cartesian trajectory.
    double interpolator_target_dt{0.01};

};

}  // namespace arm_controller::algorithm::cartesian_path_planner
