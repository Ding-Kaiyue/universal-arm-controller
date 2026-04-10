#pragma once

namespace arm_controller::algorithm::cartesian_path_planner {

struct SmoothingConfig {
    int max_shortcut_trials{200};
    double collision_check_step{0.03};
    // Iterative local point-relaxation smoothing (0 disables).
    int local_adjust_iterations{0};
    // Relaxation step in (0, 1], higher values bend faster.
    double local_adjust_alpha{0.35};
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
