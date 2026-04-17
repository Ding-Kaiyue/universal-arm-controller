#include "algorithm/cartesian_path_planner/astar/astar_planner.hpp"

#include "algorithm/cartesian_path_planner/astar/astar_node.hpp"
#include "algorithm/cartesian_path_planner/astar/grid_index.hpp"
#include "algorithm/cartesian_path_planner/astar/heuristic.hpp"
#include "algorithm/cartesian_path_planner/astar/neighbor_expander.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace arm_controller::algorithm::cartesian_path_planner {

namespace {
struct OpenItem {
    PoseGridIndex index;
    double f{0.0};

    bool operator<(const OpenItem& other) const {
        return f > other.f;
    }
};

struct SearchDiagnostics {
    int expanded_nodes{0};
    int neighbor_candidates{0};
    int rejected_state{0};
    int rejected_segment{0};
    int rejected_whole_body{0};
    int accepted_neighbors{0};
};

Eigen::Matrix3d slerpRotation(
    const Eigen::Matrix3d& R0,
    const Eigen::Matrix3d& R1,
    double t) {
    const Eigen::Quaterniond q0(R0);
    const Eigen::Quaterniond q1(R1);
    return q0.slerp(t, q1).normalized().toRotationMatrix();
}

CartesianWaypoint makeWaypoint(
    const PoseGridIndex& index,
    const Eigen::Vector3d& map_min,
    const AStarConfig& cfg,
    const Eigen::Matrix3d& fallback_orientation) {
    CartesianWaypoint wp;
    const GridIndex xyz{index.x, index.y, index.z};
    wp.position = gridToWorld(xyz, map_min, cfg.voxel_resolution);
    wp.orientation = cfg.use_se3_search
                         ? binsToRotation(
                               index.wx, index.wy, index.wz, cfg.orientation_bin_size_rad)
                         : fallback_orientation;
    return wp;
}

CartesianWaypoint resolveWaypoint(
    const PoseGridIndex& index,
    const PoseGridIndex& start,
    const PoseGridIndex& goal,
    const CartesianWaypoint& start_wp,
    const CartesianWaypoint& goal_wp,
    const Eigen::Vector3d& map_min,
    const AStarConfig& cfg,
    const Eigen::Matrix3d& fallback_orientation) {
    if (index == start) {
        return start_wp;
    }
    if (index == goal) {
        return goal_wp;
    }
    return makeWaypoint(index, map_min, cfg, fallback_orientation);
}

bool sampledWholeBodySegmentCheck(
    const CartesianWaypoint& from,
    const CartesianWaypoint& to,
    double safe_distance,
    double edge_step,
    const PathPlanningInput::WholeBodyPoseValidatorFn& pose_validator,
    const std::optional<Eigen::VectorXd>& q_seed,
    Eigen::VectorXd& q_end) {
    if (!pose_validator) {
        return true;
    }

    const double dist = (to.position - from.position).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / std::max(edge_step, 1e-3))));

    std::optional<Eigen::VectorXd> q_prev = q_seed;
    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Eigen::Vector3d p = (1.0 - t) * from.position + t * to.position;
        const Eigen::Matrix3d R = slerpRotation(from.orientation, to.orientation, t);
        Eigen::VectorXd q_i;
        if (!pose_validator(p, R, safe_distance, q_prev, q_i)) {
            return false;
        }
        q_prev = q_i;
    }

    if (!q_prev.has_value()) {
        return false;
    }
    q_end = *q_prev;
    return true;
}

bool validateTransition(
    const CartesianWaypoint& from,
    const CartesianWaypoint& to,
    const PathPlanningInput& input,
    const CartesianCollisionChecker& collision_checker,
    const double edge_step) {
    if (!collision_checker.isSegmentValid(
            from.position, to.position, input.safe_distance, edge_step, input.forbidden_spheres)) {
        return false;
    }
    return true;
}

bool hasShortcutClearance(
    const CartesianWaypoint& from,
    const CartesianWaypoint& to,
    const PathPlanningInput& input,
    const std::shared_ptr<const DistanceFieldInterface>& distance_field,
    const double edge_step,
    const double extra_margin) {
    if (!distance_field) {
        return true;
    }

    const double len = (to.position - from.position).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(edge_step, 1e-3))));
    const double limit = input.safe_distance + std::max(0.0, extra_margin);

    for (int i = 0; i <= n; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(n);
        const Eigen::Vector3d p = (1.0 - s) * from.position + s * to.position;
        if (!distance_field->isInsideMap(p)) {
            return false;
        }
        if (distance_field->getDistance(p) < limit) {
            return false;
        }
        for (const auto& forbidden : input.forbidden_spheres) {
            if (forbidden.radius > 0.0 && (p - forbidden.center).norm() < forbidden.radius) {
                return false;
            }
        }
    }
    return true;
}

double forbiddenSpherePenalty(
    const Eigen::Vector3d& p,
    const PathPlanningInput& input) {
    if (input.forbidden_spheres.empty() || input.whole_body_retry_penalty_weight <= 0.0) {
        return 0.0;
    }

    const double margin = std::max(1e-3, input.whole_body_retry_penalty_margin);
    double penalty = 0.0;
    for (const auto& forbidden : input.forbidden_spheres) {
        if (forbidden.radius <= 0.0) {
            continue;
        }
        const double dist = (p - forbidden.center).norm();
        const double influence = forbidden.radius + margin;
        if (dist >= influence) {
            continue;
        }
        const double ratio = (influence - dist) / margin;
        penalty += input.whole_body_retry_penalty_weight * ratio * ratio;
    }
    return penalty;
}

double pointToSegmentDistance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b) {
    const Eigen::Vector3d ab = b - a;
    const double ab2 = ab.squaredNorm();
    if (ab2 <= 1e-12) {
        return (p - a).norm();
    }
    const double t = std::clamp((p - a).dot(ab) / ab2, 0.0, 1.0);
    const Eigen::Vector3d proj = a + t * ab;
    return (p - proj).norm();
}

double corridorDeviationPenalty(
    const Eigen::Vector3d& p,
    const PathPlanningInput& input,
    const AStarConfig& cfg) {
    if (cfg.corridor_deviation_weight <= 0.0) {
        return 0.0;
    }
    const double d = pointToSegmentDistance(p, input.p_start, input.p_goal);
    return cfg.corridor_deviation_weight * d * d;
}

}  // namespace

AStarPlanner::AStarPlanner(
    const AStarConfig& cfg,
    std::shared_ptr<const DistanceFieldInterface> distance_field,
    const Eigen::Vector3d& map_min)
    : cfg_(cfg),
      map_min_(map_min),
      distance_field_(std::move(distance_field)),
      collision_checker_(distance_field_),
      clearance_evaluator_(distance_field_) {}

PathPlanningOutput AStarPlanner::plan(const PathPlanningInput& input) {
    PathPlanningOutput out;
    SearchDiagnostics diag;
    const auto t0 = std::chrono::steady_clock::now();
    if (!distance_field_) {
        std::cout << "[astar] failed: distance_field is null" << std::endl;
        return out;
    }
    if (!collision_checker_.isStateValid(input.p_start, input.safe_distance, input.forbidden_spheres) ||
        !collision_checker_.isStateValid(input.p_goal, input.safe_distance, input.forbidden_spheres)) {
        std::cout << "[astar] failed: start/goal state invalid for safe_distance="
                  << input.safe_distance
                  << " start_valid="
                  << (collision_checker_.isStateValid(
                          input.p_start, input.safe_distance, input.forbidden_spheres) ? "true" : "false")
                  << " goal_valid="
                  << (collision_checker_.isStateValid(
                          input.p_goal, input.safe_distance, input.forbidden_spheres) ? "true" : "false")
                  << std::endl;
        return out;
    }

    const CartesianWaypoint start_wp{input.p_start, input.R_start};
    const CartesianWaypoint goal_wp{input.p_goal, input.R_goal};

    // Fast path: if exact start->goal connection is already valid, skip lattice search.
    {
        Eigen::VectorXd q_goal_direct;
        if (validateTransition(
                start_wp,
                goal_wp,
                input,
                collision_checker_,
                cfg_.edge_check_step) &&
            hasShortcutClearance(
                start_wp,
                goal_wp,
                input,
                distance_field_,
                cfg_.edge_check_step,
                input.whole_body_retry_penalty_margin)) {
            out.success = true;
            out.path.waypoints = {start_wp, goal_wp};
            out.path.length = (input.p_goal - input.p_start).norm();
            return out;
        }
    }

    const PoseGridIndex start = worldPoseToGrid(
        input.p_start, input.R_start, map_min_, cfg_.voxel_resolution, cfg_.orientation_bin_size_rad);
    const PoseGridIndex goal = worldPoseToGrid(
        input.p_goal, input.R_goal, map_min_, cfg_.voxel_resolution, cfg_.orientation_bin_size_rad);

    std::priority_queue<OpenItem> open;
    std::unordered_map<PoseGridIndex, AStarNode, PoseGridIndexHash> nodes;
    AStarNode start_node;
    start_node.index = start;
    start_node.g = 0.0;
    start_node.h = cfg_.heuristic_weight *
                   se3GridHeuristic(
                       start,
                       goal,
                       cfg_.voxel_resolution,
                       cfg_.orientation_bin_size_rad,
                       cfg_.orientation_heuristic_weight);
    nodes[start] = start_node;
    open.push(OpenItem{start, start_node.f()});

    // 6D anti-explosion strategy:
    // - translational neighbors can be forced to axis-only in SE3 mode
    // - optional in-place orientation neighbors (+/-1 bin)
    const auto neighbor_offsets = getPoseNeighborOffsets(
        cfg_.neighbor_mode,
        cfg_.use_se3_search && cfg_.force_axis_translation_neighbors_in_se3,
        cfg_.use_se3_search && cfg_.enable_inplace_rotation_neighbors);

    int iterations = 0;
    bool found = false;
    PoseGridIndex reached_goal = start;
    const int ori_bins = orientationBinCount(cfg_.orientation_bin_size_rad);

    while (!open.empty() && iterations++ < cfg_.max_iterations) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (elapsed > cfg_.max_planning_time_sec) {
            break;
        }

        const OpenItem current_item = open.top();
        open.pop();
        const auto current_node_it = nodes.find(current_item.index);
        if (current_node_it == nodes.end()) {
            continue;
        }
        const AStarNode current = current_node_it->second;
        if (current_item.f > current.f() + 1e-12) {
            continue;  // stale queue item
        }
        ++diag.expanded_nodes;
        const CartesianWaypoint current_wp = resolveWaypoint(
            current.index, start, goal, start_wp, goal_wp, map_min_, cfg_, input.R_start);
        const Eigen::Vector3d& p_cur = current_wp.position;
        const bool pos_reached = (p_cur - input.p_goal).norm() <= input.goal_tolerance;
        const bool ori_reached =
            (!cfg_.use_se3_search) ||
            (orientationBinDistanceRad(current.index, goal, cfg_.orientation_bin_size_rad) <=
             cfg_.orientation_goal_tolerance_rad);
        if (pos_reached && ori_reached) {
            found = true;
            reached_goal = current.index;
            break;
        }

        // Pure geometric A* benefits a lot from line-of-sight shortcut to goal.
        // Whole-body checking stays in the post-validation stage.
        const double dist_to_goal = (p_cur - input.p_goal).norm();
        if (validateTransition(
                current_wp,
                goal_wp,
                input,
                collision_checker_,
                cfg_.edge_check_step) &&
            hasShortcutClearance(
                current_wp,
                goal_wp,
                input,
                distance_field_,
                cfg_.edge_check_step,
                cfg_.goal_shortcut_clearance_margin)) {
            AStarNode goal_node;
            goal_node.index = goal;
            goal_node.g = current.g + dist_to_goal;
            goal_node.h = 0.0;
            goal_node.parent = current.index;
            goal_node.has_parent = true;
            nodes[goal] = goal_node;
            found = true;
            reached_goal = goal;
            break;
        }

        for (const auto& off : neighbor_offsets) {
            ++diag.neighbor_candidates;
            const bool is_inplace_rotation =
                (off.x == 0 && off.y == 0 && off.z == 0) &&
                (off.wx != 0 || off.wy != 0 || off.wz != 0);
            if (is_inplace_rotation &&
                !cfg_.use_se3_search) {
                continue;
            }

            PoseGridIndex nb{
                current.index.x + off.x,
                current.index.y + off.y,
                current.index.z + off.z,
                wrapOrientationBin(current.index.wx + off.wx, ori_bins),
                wrapOrientationBin(current.index.wy + off.wy, ori_bins),
                wrapOrientationBin(current.index.wz + off.wz, ori_bins)};

            const CartesianWaypoint next_wp = resolveWaypoint(
                nb, start, goal, start_wp, goal_wp, map_min_, cfg_, input.R_start);
            const Eigen::Vector3d& p_nb = next_wp.position;
            if (!collision_checker_.isStateValid(
                    p_nb, input.safe_distance, input.forbidden_spheres)) {
                ++diag.rejected_state;
                continue;
            }

            if (!collision_checker_.isSegmentValid(
                    current_wp.position,
                    next_wp.position,
                    input.safe_distance,
                    cfg_.edge_check_step,
                    input.forbidden_spheres)) {
                ++diag.rejected_segment;
                continue;
            }

            if (!validateTransition(
                    current_wp,
                    next_wp,
                    input,
                    collision_checker_,
                    cfg_.edge_check_step)) {
                continue;
            }
            ++diag.accepted_neighbors;

            const double trans_cost = (p_nb - p_cur).norm();
            const double rot_cost = cfg_.use_se3_search
                                        ? cfg_.orientation_cost_weight *
                                              orientationBinDistanceRad(
                                                  current.index, nb, cfg_.orientation_bin_size_rad)
                                        : 0.0;
            const double penalty = cfg_.obstacle_penalty_weight *
                                   clearance_evaluator_.obstaclePenalty(p_nb, input.safe_distance);
            const double retry_penalty = forbiddenSpherePenalty(p_nb, input);
            const double corridor_penalty = corridorDeviationPenalty(p_nb, input, cfg_);
            const double new_g =
                current.g + trans_cost + rot_cost + penalty + retry_penalty + corridor_penalty;

            auto it = nodes.find(nb);
            if (it == nodes.end() || new_g < it->second.g) {
                AStarNode node;
                node.index = nb;
                node.g = new_g;
                node.h = cfg_.heuristic_weight *
                         se3GridHeuristic(
                             nb,
                             goal,
                             cfg_.voxel_resolution,
                             cfg_.orientation_bin_size_rad,
                             cfg_.orientation_heuristic_weight);
                node.parent = current.index;
                node.has_parent = true;
                nodes[nb] = node;
                open.push(OpenItem{nb, node.f()});
            }
        }
    }

    if (!found || reached_goal == start) {
        std::cout << "[astar] failed: expanded=" << diag.expanded_nodes
                  << " candidates=" << diag.neighbor_candidates
                  << " reject_state=" << diag.rejected_state
                  << " reject_segment=" << diag.rejected_segment
                  << " reject_whole_body=" << diag.rejected_whole_body
                  << " accepted=" << diag.accepted_neighbors
                  << " iterations=" << iterations
                  << std::endl;
        return out;
    }

    std::vector<CartesianWaypoint> rev_path;
    PoseGridIndex cur = reached_goal;
    while (true) {
        rev_path.push_back(resolveWaypoint(
            cur, start, goal, start_wp, goal_wp, map_min_, cfg_, input.R_start));

        const auto it = nodes.find(cur);
        if (it == nodes.end() || !it->second.has_parent) {
            break;
        }
        cur = it->second.parent;
    }

    std::reverse(rev_path.begin(), rev_path.end());
    if (!rev_path.empty()) {
        rev_path.front().position = input.p_start;
        rev_path.front().orientation = input.R_start;
        rev_path.back().position = input.p_goal;
        rev_path.back().orientation = input.R_goal;
    }

    double length = 0.0;
    for (size_t i = 1; i < rev_path.size(); ++i) {
        length += (rev_path[i].position - rev_path[i - 1].position).norm();
    }
    out.success = true;
    out.path.waypoints = std::move(rev_path);
    out.path.length = length;
    return out;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
