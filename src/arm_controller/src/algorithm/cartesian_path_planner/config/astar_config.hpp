#pragma once

namespace arm_controller::algorithm::cartesian_path_planner {

struct AStarConfig {
    // false: legacy 3D (x,y,z), true: 6D SE(3) lattice (x,y,z + r,p,y bins)
    bool use_se3_search{true};

    double voxel_resolution{0.015};  // m
    // 6 / 18 / 26 邻接，26 更平滑但计算量更大
    int neighbor_mode{26};

    // Orientation lattice resolution for roll/pitch/yaw bins (rad).
    // Typical robust range: [10deg, 30deg].
    double orientation_bin_size_rad{0.3490658504};  // 20 deg
    // Orientation goal tolerance for SE(3) planning (rad).
    double orientation_goal_tolerance_rad{0.35};
    // Add in-place orientation neighbors (+/-1 bin per axis) in 6D mode.
    bool enable_inplace_rotation_neighbors{true};
    bool goal_orientation_only{false};  // only enforce target orientation at final waypoint
    double goal_orientation_blend_distance{0.15};  // start blending toward goal orientation within this distance
    // Limit translational branching in 6D mode by forcing axis neighbors.
    // 6D search can still explode; this is a strong guard.
    bool force_axis_translation_neighbors_in_se3{false};

    double obstacle_penalty_weight{0.2};  // 靠近障碍物的路径惩罚权重，越大越远离障碍物
    double corridor_deviation_weight{0.0};  // 偏离 start->goal 走廊的代价权重
    double whole_body_penalty_weight{3.0};  // whole-body 间隙不足时的软代价
    double whole_body_penalty_margin{0.06};  // 小于该间隙开始增加 whole-body 代价
    double whole_body_hard_reject_margin{-0.08};  // 小于该间隙直接拒绝该 EE pose
    bool whole_body_reject_on_ik_fail{false};  // IK 失败的 whole-body 节点是否直接拒绝
    // Extra geometric clearance required for the direct-to-goal shortcut.
    // Shortcut is allowed only when sampled distance >= safe_distance + this margin.
    double goal_shortcut_clearance_margin{0.03};
    double orientation_cost_weight{0.10};  // rotation step cost weight
    double orientation_heuristic_weight{0.25};  // rotation term in heuristic
    double heuristic_weight{1.0};  // A*启发式权重，1.0表示标准A*，0.0表示Dijkstra
    int max_iterations{200000};  // A*最大迭代次数，避免死循环
    double max_planning_time_sec{0.2};  // A*最大规划时间（秒）
    double edge_check_step{0.01};  // 边碰撞离散检查步长
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
