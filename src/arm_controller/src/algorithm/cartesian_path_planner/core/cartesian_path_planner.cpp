#include "algorithm/cartesian_path_planner/core/cartesian_path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace arm_controller::algorithm::cartesian_path_planner {

namespace {

bool validateJointState(
    const Eigen::VectorXd& q,
    const PathPlanningInput& input,
    PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
    if (!input.joint_state_validator) {
        return false;
    }
    return input.joint_state_validator(q, input.safe_distance, diag);
}

bool validateJointSegment(
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    const PathPlanningInput& input,
    PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
    if (!input.joint_segment_validator) {
        return false;
    }
    return input.joint_segment_validator(q_from, q_to, input.safe_distance, diag);
}

bool isFiniteNonEmpty(const Eigen::VectorXd& q) {
    return q.size() > 0 && q.allFinite();
}

}  // namespace

CartesianPathPlanner::CartesianPathPlanner(
    const PlannerCommonConfig& common_cfg,
    const SmoothingConfig& /*smoothing_cfg*/,
    std::shared_ptr<const DistanceFieldInterface> /*distance_field*/)
    : common_cfg_(common_cfg) {}

PathPlanningOutput CartesianPathPlanner::planPath(const PathPlanningInput& input) {
    return planJointSpacePath(input);
}

TimedCartesianTrajectory CartesianPathPlanner::planTrajectory(
    const PathPlanningInput& input) {
    const PathPlanningOutput output = planJointSpacePath(input);
    if (!output.success || output.joint_waypoints.size() < 2) {
        return {};
    }
    return buildJointSpaceTrajectory(optimizeJointTrajectory(output.joint_waypoints, input), input);
}

PathPlanningOutput CartesianPathPlanner::planJointSpacePath(const PathPlanningInput& input) {
    PathPlanningOutput out;
    if (!input.q_start_seed.has_value() || !isFiniteNonEmpty(*input.q_start_seed) ||
        input.q_goal_candidates.empty() || !input.joint_state_validator ||
        !input.joint_segment_validator || !input.joint_to_pose_fn) {
        std::cout << "[planner] joint-space sampling missing required inputs" << std::endl;
        return out;
    }

    const Eigen::VectorXd q_start = *input.q_start_seed;
    PathPlanningInput::WholeBodyPoseDiagnostic start_diag;
    if (!validateJointState(q_start, input, &start_diag)) {
        std::cout << "[planner] joint-space sampling invalid start state reason="
                  << start_diag.reason << std::endl;
        return out;
    }

    std::vector<Eigen::VectorXd> valid_goals;
    valid_goals.reserve(input.q_goal_candidates.size());
    for (const auto& q_goal : input.q_goal_candidates) {
        if (q_goal.size() != q_start.size() || !q_goal.allFinite()) {
            continue;
        }
        PathPlanningInput::WholeBodyPoseDiagnostic goal_diag;
        if (!validateJointState(q_goal, input, &goal_diag)) {
            continue;
        }
        valid_goals.push_back(q_goal);
    }
    if (valid_goals.empty()) {
        std::cout << "[planner] joint-space sampling no valid goal states" << std::endl;
        return out;
    }

    Eigen::VectorXd hard_q_min = input.q_min;
    Eigen::VectorXd hard_q_max = input.q_max;
    const bool has_hard_bounds =
        hard_q_min.size() == q_start.size() && hard_q_max.size() == q_start.size() &&
        hard_q_min.allFinite() && hard_q_max.allFinite();

    Eigen::VectorXd seed_q_min = q_start;
    Eigen::VectorXd seed_q_max = q_start;
    for (const auto& q_goal : valid_goals) {
        seed_q_min = seed_q_min.cwiseMin(q_goal);
        seed_q_max = seed_q_max.cwiseMax(q_goal);
    }

    const auto clampBounds = [&](Eigen::VectorXd* q_min, Eigen::VectorXd* q_max) {
        for (Eigen::Index i = 0; i < q_min->size(); ++i) {
            if ((*q_min)[i] > (*q_max)[i]) {
                std::swap((*q_min)[i], (*q_max)[i]);
            }
            if (has_hard_bounds) {
                (*q_min)[i] = std::max((*q_min)[i], hard_q_min[i]);
                (*q_max)[i] = std::min((*q_max)[i], hard_q_max[i]);
                if ((*q_min)[i] > (*q_max)[i]) {
                    const double mid = 0.5 * (hard_q_min[i] + hard_q_max[i]);
                    (*q_min)[i] = mid;
                    (*q_max)[i] = mid;
                }
            }
        }
    };

    std::random_device rd;
    std::mt19937 rng(rd());

    const int max_iterations = std::max(1, common_cfg_.joint_space_sampling_max_iterations);
    const int search_stages = std::max(1, common_cfg_.joint_space_sampling_search_stages);
    const int iterations_per_stage = std::max(1, max_iterations / search_stages);
    const double step_rad = std::max(1e-3, common_cfg_.joint_space_sampling_step_rad);
    const double connect_threshold =
        std::max(step_rad, common_cfg_.joint_space_sampling_connect_threshold_rad);
    const double base_window = std::max(1e-3, common_cfg_.joint_space_sampling_local_window_rad);
    const double window_scale = std::max(1.0, common_cfg_.joint_space_sampling_window_scale);

    std::vector<JointPathCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(
        std::max(1, common_cfg_.joint_space_sampling_solution_pool_size)));

    auto runSearchStage = [&](const Eigen::VectorXd& q_min,
                              const Eigen::VectorXd& q_max,
                              const int iterations_budget) {
        std::vector<JointTreeNode> start_tree;
        std::vector<JointTreeNode> goal_tree;
        start_tree.push_back(JointTreeNode{q_start, -1});
        goal_tree.reserve(valid_goals.size());
        for (const auto& q_goal : valid_goals) {
            goal_tree.push_back(JointTreeNode{q_goal, -1});
        }

        for (int iter = 0; iter < iterations_budget; ++iter) {
            std::vector<JointTreeNode>* tree_from = (iter % 2 == 0) ? &start_tree : &goal_tree;
            std::vector<JointTreeNode>* tree_to = (iter % 2 == 0) ? &goal_tree : &start_tree;

            const Eigen::VectorXd q_rand = sampleJointState(
                q_min,
                q_max,
                valid_goals,
                common_cfg_.joint_space_sampling_goal_bias,
                rng);
            const int nearest_from = nearestJointNode(*tree_from, q_rand);
            if (nearest_from < 0) {
                continue;
            }

            const ExtendResult grow_from = extendTree(
                *tree_from,
                nearest_from,
                q_rand,
                step_rad,
                connect_threshold,
                input);
            if (!grow_from.advanced) {
                continue;
            }
            const int last_from = grow_from.new_index;
            const Eigen::VectorXd q_new = (*tree_from)[static_cast<std::size_t>(last_from)].q;

            const int nearest_to = nearestJointNode(*tree_to, q_new);
            if (nearest_to < 0) {
                continue;
            }

            const ExtendResult grow_to = extendTree(
                *tree_to,
                nearest_to,
                q_new,
                step_rad,
                connect_threshold,
                input);
            if (!grow_to.advanced && !grow_to.reached_target) {
                continue;
            }
            const int last_to = grow_to.new_index;
            const Eigen::VectorXd q_connect = (*tree_to)[static_cast<std::size_t>(last_to)].q;
            if (jointDistance(q_new, q_connect) > connect_threshold) {
                continue;
            }

            int connect_start_idx = -1;
            int connect_goal_idx = -1;
            if (iter % 2 == 0) {
                connect_start_idx = last_from;
                connect_goal_idx = last_to;
            } else {
                connect_start_idx = last_to;
                connect_goal_idx = last_from;
            }

            std::vector<Eigen::VectorXd> joint_path = backtrackJointPath(start_tree, connect_start_idx);
            std::vector<Eigen::VectorXd> joint_tail =
                backtrackJointPathReversed(goal_tree, connect_goal_idx);
            if (!joint_path.empty() && !joint_tail.empty() &&
                jointDistance(joint_path.back(), joint_tail.front()) < 1e-8) {
                joint_tail.erase(joint_tail.begin());
            }
            joint_path.insert(joint_path.end(), joint_tail.begin(), joint_tail.end());
            shortcutJointPath(joint_path, input, common_cfg_.joint_space_shortcut_trials);

            JointPathCandidate candidate = evaluateJointPathCandidate(joint_path, input);
            if (std::isfinite(candidate.score)) {
                candidates.push_back(std::move(candidate));
                if (static_cast<int>(candidates.size()) >=
                    std::max(1, common_cfg_.joint_space_sampling_solution_pool_size)) {
                    return;
                }
            }
        }
    };

    for (int stage = 0; stage < search_stages; ++stage) {
        const double expansion =
            base_window * std::pow(window_scale, static_cast<double>(stage));
        Eigen::VectorXd q_min = seed_q_min;
        Eigen::VectorXd q_max = seed_q_max;
        q_min.array() -= expansion;
        q_max.array() += expansion;
        clampBounds(&q_min, &q_max);

        std::cout << "[planner] joint-space sampling stage=" << (stage + 1)
                  << "/" << search_stages
                  << " expansion_rad=" << expansion
                  << " iterations=" << iterations_per_stage
                  << std::endl;
        runSearchStage(q_min, q_max, iterations_per_stage);
        if (!candidates.empty()) {
            break;
        }
    }

    if (candidates.empty() &&
        has_hard_bounds &&
        common_cfg_.joint_space_sampling_allow_full_joint_limit_fallback) {
        Eigen::VectorXd q_min = hard_q_min;
        Eigen::VectorXd q_max = hard_q_max;
        clampBounds(&q_min, &q_max);
        std::cout << "[planner] joint-space sampling fallback=full_joint_limits iterations="
                  << max_iterations << std::endl;
        runSearchStage(q_min, q_max, max_iterations);
    }

    if (candidates.empty()) {
        std::cout << "[planner] joint-space sampling failed: iterations="
                  << max_iterations << " stages=" << search_stages << std::endl;
        return out;
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const JointPathCandidate& a, const JointPathCandidate& b) {
            return a.score < b.score;
        });
    const JointPathCandidate& best_candidate = candidates.front();
    std::cout << "[planner] joint-space candidate selected: candidates="
              << candidates.size()
              << " score=" << best_candidate.score
              << " cartesian_length=" << best_candidate.cartesian_length
              << " joint_motion=" << best_candidate.joint_motion
              << " shell_cost=" << best_candidate.shell_deviation_cost
              << std::endl;

    out.joint_waypoints = best_candidate.joint_path;
    out.path.waypoints.reserve(best_candidate.joint_path.size());
    for (const auto& q : best_candidate.joint_path) {
        CartesianWaypoint wp;
        if (!input.joint_to_pose_fn(q, wp)) {
            std::cout << "[planner] joint-space sampling FK conversion failed" << std::endl;
            return {};
        }
        out.path.waypoints.push_back(wp);
    }

    double length = 0.0;
    for (std::size_t i = 1; i < out.path.waypoints.size(); ++i) {
        length += (out.path.waypoints[i].position - out.path.waypoints[i - 1].position).norm();
    }
    out.path.length = length;
    out.success = out.path.waypoints.size() >= 2;
    return out;
}

TimedCartesianTrajectory CartesianPathPlanner::buildJointSpaceTrajectory(
    const std::vector<Eigen::VectorXd>& joint_path,
    const PathPlanningInput& input) const {
    TimedCartesianTrajectory traj;
    if (joint_path.size() < 2 || !input.joint_to_pose_fn) {
        return traj;
    }

    traj.waypoints.reserve(joint_path.size());
    traj.waypoint_joint_targets = joint_path;
    traj.segment_durations.reserve(joint_path.size() - 1);
    traj.cumulative_times.reserve(joint_path.size());
    traj.cumulative_times.push_back(0.0);

    for (const auto& q : joint_path) {
        CartesianWaypoint wp;
        if (!input.joint_to_pose_fn(q, wp)) {
            return {};
        }
        traj.waypoints.push_back(wp);
    }

    traj.waypoints.front().orientation = input.R_start;
    traj.waypoints.back().orientation = input.R_goal;

    double total_path_length = 0.0;
    for (std::size_t i = 1; i < traj.waypoints.size(); ++i) {
        total_path_length +=
            (traj.waypoints[i].position - traj.waypoints[i - 1].position).norm();
    }

    const double safe_total_length = std::max(1e-9, total_path_length);
    double accumulated_length = 0.0;
    for (std::size_t i = 1; i + 1 < traj.waypoints.size(); ++i) {
        accumulated_length +=
            (traj.waypoints[i].position - traj.waypoints[i - 1].position).norm();
        const double alpha = std::clamp(accumulated_length / safe_total_length, 0.0, 1.0);
        Eigen::Quaterniond q0(input.R_start);
        Eigen::Quaterniond q1(input.R_goal);
        q0.normalize();
        q1.normalize();
        if (q0.dot(q1) < 0.0) {
            q1.coeffs() *= -1.0;
        }
        traj.waypoints[i].orientation = q0.slerp(alpha, q1).toRotationMatrix();
    }

    double total_time = 0.0;
    for (std::size_t i = 1; i < traj.waypoints.size(); ++i) {
        const double segment_length =
            (traj.waypoints[i].position - traj.waypoints[i - 1].position).norm();
        const double dt =
            std::max(1e-3, segment_length / std::max(1e-3, common_cfg_.default_segment_speed));
        traj.segment_durations.push_back(dt);
        total_time += dt;
        traj.cumulative_times.push_back(total_time);
    }
    traj.total_duration = total_time;
    return traj;
}

std::vector<Eigen::VectorXd> CartesianPathPlanner::optimizeJointTrajectory(
    const std::vector<Eigen::VectorXd>& joint_path,
    const PathPlanningInput& input) const {
    if (!common_cfg_.enable_joint_trajectory_post_optimization ||
        common_cfg_.joint_trajectory_postopt_iterations <= 0 ||
        joint_path.size() < 3 || !input.joint_to_pose_fn ||
        !input.joint_state_validator || !input.joint_segment_validator) {
        return joint_path;
    }

    std::vector<Eigen::VectorXd> optimized = joint_path;
    std::random_device rd;
    std::mt19937 rng(rd());
    std::normal_distribution<double> unit_noise(0.0, 1.0);

    const auto orientationError = [](const Eigen::Matrix3d& R_des,
                                     const Eigen::Matrix3d& R_now) {
        const Eigen::Matrix3d R_err = R_des.transpose() * R_now;
        Eigen::AngleAxisd aa(R_err);
        return std::abs(aa.angle());
    };

    const auto evaluateWaypointCandidate =
        [&](const Eigen::VectorXd& q_prev,
            const Eigen::VectorXd& q_candidate,
            const Eigen::VectorXd& q_next,
            const double alpha) -> double {
        PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateJointState(q_candidate, input, &diag) ||
            !validateJointSegment(q_prev, q_candidate, input) ||
            !validateJointSegment(q_candidate, q_next, input)) {
            return std::numeric_limits<double>::infinity();
        }

        CartesianWaypoint wp_prev;
        CartesianWaypoint wp_candidate;
        CartesianWaypoint wp_next;
        if (!input.joint_to_pose_fn(q_prev, wp_prev) ||
            !input.joint_to_pose_fn(q_candidate, wp_candidate) ||
            !input.joint_to_pose_fn(q_next, wp_next)) {
            return std::numeric_limits<double>::infinity();
        }

        Eigen::Quaterniond q0(input.R_start);
        Eigen::Quaterniond q1(input.R_goal);
        q0.normalize();
        q1.normalize();
        if (q0.dot(q1) < 0.0) {
            q1.coeffs() *= -1.0;
        }
        const Eigen::Matrix3d R_des =
            q0.slerp(std::clamp(alpha, 0.0, 1.0), q1).toRotationMatrix();
        const double orientation_cost = orientationError(R_des, wp_candidate.orientation);
        const double smoothness_cost =
            (q_prev - 2.0 * q_candidate + q_next).squaredNorm();
        const double position_cost =
            (wp_candidate.position - 0.5 * (wp_prev.position + wp_next.position)).squaredNorm();
        const double shell_cost =
            std::isfinite(diag.min_margin)
                ? std::abs(diag.min_margin - common_cfg_.preferred_clearance_shell_margin_m)
                : 1e3;

        return common_cfg_.joint_trajectory_shell_weight * shell_cost +
               common_cfg_.joint_trajectory_orientation_weight * orientation_cost +
               common_cfg_.joint_trajectory_smoothness_weight * smoothness_cost +
               common_cfg_.joint_trajectory_position_weight * position_cost;
    };

    const int samples_per_waypoint =
        std::max(1, common_cfg_.joint_trajectory_postopt_samples_per_waypoint);
    const double perturbation_rad =
        std::max(1e-4, common_cfg_.joint_trajectory_postopt_perturbation_rad);

    for (int iter = 0; iter < common_cfg_.joint_trajectory_postopt_iterations; ++iter) {
        bool any_improved = false;
        for (std::size_t i = 1; i + 1 < optimized.size(); ++i) {
            const Eigen::VectorXd& q_prev = optimized[i - 1];
            const Eigen::VectorXd& q_next = optimized[i + 1];
            const double alpha = static_cast<double>(i) /
                                 static_cast<double>(optimized.size() - 1);

            Eigen::VectorXd best_q = optimized[i];
            double best_score = evaluateWaypointCandidate(q_prev, best_q, q_next, alpha);

            std::vector<Eigen::VectorXd> candidates;
            candidates.reserve(static_cast<std::size_t>(samples_per_waypoint + 2));
            candidates.push_back(best_q);
            candidates.push_back(0.5 * (q_prev + q_next));

            for (int sample_idx = 0; sample_idx < samples_per_waypoint; ++sample_idx) {
                Eigen::VectorXd q_try = best_q;
                const Eigen::VectorXd center = 0.5 * (q_prev + q_next);
                const double blend = (sample_idx % 2 == 0) ? 0.35 : 0.65;
                q_try = (1.0 - blend) * q_try + blend * center;
                for (Eigen::Index j = 0; j < q_try.size(); ++j) {
                    q_try[j] += perturbation_rad * unit_noise(rng);
                }
                candidates.push_back(std::move(q_try));
            }

            for (const auto& q_candidate : candidates) {
                const double score =
                    evaluateWaypointCandidate(q_prev, q_candidate, q_next, alpha);
                if (score + 1e-9 < best_score) {
                    best_score = score;
                    best_q = q_candidate;
                }
            }

            if ((best_q - optimized[i]).norm() > 1e-6) {
                optimized[i] = best_q;
                any_improved = true;
            }
        }
        if (!any_improved) {
            break;
        }
    }

    double shell_accum = 0.0;
    double orientation_accum = 0.0;
    int counted = 0;
    for (std::size_t i = 1; i + 1 < optimized.size(); ++i) {
        PathPlanningInput::WholeBodyPoseDiagnostic diag;
        CartesianWaypoint wp;
        if (!validateJointState(optimized[i], input, &diag) ||
            !input.joint_to_pose_fn(optimized[i], wp)) {
            continue;
        }
        Eigen::Quaterniond q0(input.R_start);
        Eigen::Quaterniond q1(input.R_goal);
        q0.normalize();
        q1.normalize();
        if (q0.dot(q1) < 0.0) {
            q1.coeffs() *= -1.0;
        }
        const double alpha = static_cast<double>(i) /
                             static_cast<double>(optimized.size() - 1);
        const Eigen::Matrix3d R_des =
            q0.slerp(std::clamp(alpha, 0.0, 1.0), q1).toRotationMatrix();
        shell_accum += std::abs(diag.min_margin - common_cfg_.preferred_clearance_shell_margin_m);
        orientation_accum += orientationError(R_des, wp.orientation);
        ++counted;
    }
    if (counted > 0) {
        std::cout << "[planner] joint-trajectory postopt: waypoints=" << optimized.size()
                  << " avg_shell_cost=" << (shell_accum / counted)
                  << " avg_orientation_err=" << (orientation_accum / counted)
                  << std::endl;
    }

    return optimized;
}

Eigen::VectorXd CartesianPathPlanner::sampleJointState(
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max,
    const std::vector<Eigen::VectorXd>& q_goal_candidates,
    const double goal_bias,
    std::mt19937& rng) const {
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    if (!q_goal_candidates.empty() && unit_dist(rng) < std::clamp(goal_bias, 0.0, 1.0)) {
        std::uniform_int_distribution<std::size_t> goal_dist(0, q_goal_candidates.size() - 1);
        return q_goal_candidates[goal_dist(rng)];
    }

    Eigen::VectorXd q(q_min.size());
    for (Eigen::Index i = 0; i < q.size(); ++i) {
        const double lo = std::min(q_min[i], q_max[i]);
        const double hi = std::max(q_min[i], q_max[i]);
        std::uniform_real_distribution<double> dist(lo, hi);
        q[i] = dist(rng);
    }
    return q;
}

int CartesianPathPlanner::nearestJointNode(
    const std::vector<JointTreeNode>& tree,
    const Eigen::VectorXd& q) {
    if (tree.empty()) {
        return -1;
    }
    double best_dist = std::numeric_limits<double>::infinity();
    int best_index = -1;
    for (std::size_t i = 0; i < tree.size(); ++i) {
        const double dist = jointDistance(tree[i].q, q);
        if (dist < best_dist) {
            best_dist = dist;
            best_index = static_cast<int>(i);
        }
    }
    return best_index;
}

Eigen::VectorXd CartesianPathPlanner::steerJointState(
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    const double step_rad) {
    const Eigen::VectorXd delta = q_to - q_from;
    const double norm = delta.norm();
    if (norm <= step_rad) {
        return q_to;
    }
    return q_from + (step_rad / std::max(norm, 1e-9)) * delta;
}

double CartesianPathPlanner::jointDistance(
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& b) {
    if (a.size() == 0 || a.size() != b.size() || !a.allFinite() || !b.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    return (a - b).norm();
}

std::vector<Eigen::VectorXd> CartesianPathPlanner::backtrackJointPath(
    const std::vector<JointTreeNode>& tree,
    int node_index) {
    std::vector<Eigen::VectorXd> path;
    while (node_index >= 0 && node_index < static_cast<int>(tree.size())) {
        path.push_back(tree[static_cast<std::size_t>(node_index)].q);
        node_index = tree[static_cast<std::size_t>(node_index)].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Eigen::VectorXd> CartesianPathPlanner::backtrackJointPathReversed(
    const std::vector<JointTreeNode>& tree,
    int node_index) {
    std::vector<Eigen::VectorXd> path;
    while (node_index >= 0 && node_index < static_cast<int>(tree.size())) {
        path.push_back(tree[static_cast<std::size_t>(node_index)].q);
        node_index = tree[static_cast<std::size_t>(node_index)].parent;
    }
    return path;
}

void CartesianPathPlanner::shortcutJointPath(
    std::vector<Eigen::VectorXd>& joint_path,
    const PathPlanningInput& input,
    const int shortcut_trials) {
    if (joint_path.size() < 3 || !input.joint_segment_validator) {
        return;
    }

    std::random_device rd;
    std::mt19937 rng(rd());
    for (int trial = 0; trial < std::max(0, shortcut_trials); ++trial) {
        if (joint_path.size() < 3) {
            break;
        }
        std::uniform_int_distribution<std::size_t> dist(0, joint_path.size() - 1);
        std::size_t i = dist(rng);
        std::size_t j = dist(rng);
        if (i == j) {
            continue;
        }
        if (i > j) {
            std::swap(i, j);
        }
        if (j <= i + 1) {
            continue;
        }
        if (!validateJointSegment(joint_path[i], joint_path[j], input)) {
            continue;
        }
        joint_path.erase(
            joint_path.begin() + static_cast<std::ptrdiff_t>(i + 1),
            joint_path.begin() + static_cast<std::ptrdiff_t>(j));
    }
}

CartesianPathPlanner::JointPathCandidate CartesianPathPlanner::evaluateJointPathCandidate(
    const std::vector<Eigen::VectorXd>& joint_path,
    const PathPlanningInput& input) const {
    JointPathCandidate candidate;
    candidate.joint_path = joint_path;
    if (joint_path.size() < 2 || !input.joint_to_pose_fn) {
        return candidate;
    }

    std::vector<CartesianWaypoint> waypoints;
    waypoints.reserve(joint_path.size());
    for (const auto& q : joint_path) {
        CartesianWaypoint wp;
        if (!input.joint_to_pose_fn(q, wp)) {
            return candidate;
        }
        waypoints.push_back(wp);
    }

    double shell_cost = 0.0;
    double path_length = 0.0;
    double joint_motion = 0.0;
    const bool prefer_shell = common_cfg_.prefer_clearance_shell;
    const double preferred_margin = common_cfg_.preferred_clearance_shell_margin_m;

    for (std::size_t i = 0; i < joint_path.size(); ++i) {
        if (i > 0) {
            path_length += (waypoints[i].position - waypoints[i - 1].position).norm();
            joint_motion += jointDistance(joint_path[i - 1], joint_path[i]);
        }
        if (prefer_shell && input.joint_state_validator) {
            PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (!input.joint_state_validator(joint_path[i], input.safe_distance, &diag) ||
                !std::isfinite(diag.min_margin)) {
                candidate.score = std::numeric_limits<double>::infinity();
                return candidate;
            }
            shell_cost += std::abs(diag.min_margin - preferred_margin);
        }
    }

    candidate.cartesian_length = path_length;
    candidate.joint_motion = joint_motion;
    candidate.shell_deviation_cost = shell_cost;
    candidate.score =
        common_cfg_.joint_space_path_length_weight * candidate.cartesian_length +
        common_cfg_.joint_space_joint_motion_weight * candidate.joint_motion +
        common_cfg_.preferred_clearance_shell_weight * candidate.shell_deviation_cost;
    return candidate;
}

CartesianPathPlanner::ExtendResult CartesianPathPlanner::extendTree(
    std::vector<JointTreeNode>& tree,
    const int from_index,
    const Eigen::VectorXd& q_target,
    const double step_rad,
    const double connect_threshold,
    const PathPlanningInput& input) {
    ExtendResult result;
    if (from_index < 0 || from_index >= static_cast<int>(tree.size())) {
        return result;
    }

    int parent = from_index;
    Eigen::VectorXd q_current = tree[static_cast<std::size_t>(from_index)].q;
    while (jointDistance(q_current, q_target) > connect_threshold) {
        const Eigen::VectorXd q_next = steerJointState(q_current, q_target, step_rad);
        if ((q_next - q_current).norm() < 1e-9) {
            break;
        }
        if (!validateJointSegment(q_current, q_next, input)) {
            return result;
        }
        tree.push_back(JointTreeNode{q_next, parent});
        parent = static_cast<int>(tree.size()) - 1;
        q_current = q_next;
        result.advanced = true;
        result.new_index = parent;
    }

    if (jointDistance(q_current, q_target) <= connect_threshold &&
        validateJointSegment(q_current, q_target, input)) {
        if (jointDistance(q_current, q_target) > 1e-9) {
            tree.push_back(JointTreeNode{q_target, parent});
            parent = static_cast<int>(tree.size()) - 1;
            result.advanced = true;
        }
        result.reached_target = true;
        result.new_index = parent;
    }
    return result;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
