#include "algorithm/cartesian_path_planner/core/cartesian_path_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace arm_controller::algorithm::cartesian_path_planner {

namespace {

namespace ob = ompl::base;
namespace og = ompl::geometric;

double feasibilitySafeDistance(const PathPlanningInput& input) {
    if (std::isfinite(input.feasibility_safe_distance) &&
        input.feasibility_safe_distance > 0.0) {
        return std::min(input.safe_distance, input.feasibility_safe_distance);
    }
    return input.safe_distance;
}

bool validateJointState(
    const Eigen::VectorXd& q,
    const PathPlanningInput& input,
    const double safe_distance,
    PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
    if (!input.joint_state_validator) {
        return false;
    }
    return input.joint_state_validator(q, safe_distance, diag);
}

bool validateJointSegment(
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    const PathPlanningInput& input,
    PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
    if (!input.joint_segment_validator) {
        return false;
    }
    return input.joint_segment_validator(q_from, q_to, feasibilitySafeDistance(input), diag);
}

bool isFiniteNonEmpty(const Eigen::VectorXd& q) {
    return q.size() > 0 && q.allFinite();
}

}  // namespace

CartesianPathPlanner::CartesianPathPlanner(
    const PlannerCommonConfig& common_cfg,
    const SmoothingConfig& /*smoothing_cfg*/)
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
    TimedCartesianTrajectory traj =
        buildJointSpaceTrajectory(optimizeJointTrajectory(output.joint_waypoints, input), input);
    traj.segment_kind = output.segment_kind;
    return traj;
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
    if (!validateJointState(q_start, input, input.safe_distance, &start_diag)) {
        std::cout << "[planner] planning rejected: start outside planning safety shell "
                  << "reason=" << start_diag.reason
                  << " strict_margin=" << start_diag.min_margin
                  << " worst_link=" << start_diag.worst_link_name
                  << " worst_distance=" << start_diag.worst_distance
                  << " safe_distance=" << input.safe_distance << std::endl;
        return out;
    }

    std::vector<Eigen::VectorXd> valid_goals;
    valid_goals.reserve(input.q_goal_candidates.size());
    for (const auto& q_goal : input.q_goal_candidates) {
        if (q_goal.size() != q_start.size() || !q_goal.allFinite()) {
            continue;
        }
        PathPlanningInput::WholeBodyPoseDiagnostic goal_diag;
        if (!validateJointState(q_goal, input, input.safe_distance, &goal_diag)) {
            std::cout << "[planner] goal candidate rejected: outside planning safety shell "
                      << "reason=" << goal_diag.reason
                      << " strict_margin=" << goal_diag.min_margin
                      << " worst_link=" << goal_diag.worst_link_name
                      << " worst_distance=" << goal_diag.worst_distance
                      << " safe_distance=" << input.safe_distance << std::endl;
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

    PathPlanningOutput ompl_out = planOmplRrtConnectPath(
        input,
        q_start,
        valid_goals);
    if (ompl_out.success) {
        return ompl_out;
    }
    std::cout << "[planner] ompl_rrt_connect failed; falling back to legacy joint-space sampler"
              << std::endl;

    const int max_iterations = std::max(1, common_cfg_.joint_space_sampling_max_iterations);
    const int search_stages = std::max(1, common_cfg_.joint_space_sampling_search_stages);
    const int iterations_per_stage = std::max(1, max_iterations / search_stages);
    const double step_rad = std::max(1e-3, common_cfg_.joint_space_sampling_step_rad);
    const double connect_threshold =
        std::max(step_rad, common_cfg_.joint_space_sampling_connect_threshold_rad);
    const double base_window = std::max(1e-3, common_cfg_.joint_space_sampling_local_window_rad);
    const double window_scale = std::max(1.0, common_cfg_.joint_space_sampling_window_scale);
    const int solution_pool_size =
        std::max(1, common_cfg_.joint_space_sampling_solution_pool_size);
    const int per_stage_solution_target = std::max(
        1,
        static_cast<int>(std::ceil(
            static_cast<double>(solution_pool_size) / static_cast<double>(search_stages))));
    const int min_stage_iterations_before_early_stop =
        std::min(iterations_per_stage, std::max(32, 8 * per_stage_solution_target));
    const auto sampling_start_time = std::chrono::steady_clock::now();
    const double time_budget_sec = common_cfg_.joint_space_sampling_time_budget_sec;
    auto timeBudgetExceeded = [&]() {
        if (time_budget_sec <= 0.0) {
            return false;
        }
        const auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - sampling_start_time).count();
        return elapsed >= time_budget_sec;
    };

    std::vector<JointPathCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(solution_pool_size));

    const auto insertCandidate = [&](JointPathCandidate candidate) {
        if (!std::isfinite(candidate.score)) {
            return;
        }
        candidates.push_back(std::move(candidate));
        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const JointPathCandidate& a, const JointPathCandidate& b) {
                return a.score < b.score;
            });
        if (static_cast<int>(candidates.size()) > solution_pool_size) {
            candidates.resize(static_cast<std::size_t>(solution_pool_size));
        }
    };

    auto runSearchStage = [&](const Eigen::VectorXd& q_min,
                              const Eigen::VectorXd& q_max,
                              const int iterations_budget,
                              const int stage_index) {
        std::vector<JointTreeNode> start_tree;
        std::vector<JointTreeNode> goal_tree;
        start_tree.push_back(JointTreeNode{q_start, -1});
        goal_tree.reserve(valid_goals.size());
        for (const auto& q_goal : valid_goals) {
            goal_tree.push_back(JointTreeNode{q_goal, -1});
        }

        int stage_candidate_count = 0;
        JointPathCandidate stage_best_candidate;

        for (int iter = 0; iter < iterations_budget; ++iter) {
            if (timeBudgetExceeded()) {
                std::cout << "[planner] joint-space stage_time_budget stage=" << stage_index
                          << " iterations=" << iter
                          << " candidates=" << stage_candidate_count
                          << " budget_sec=" << time_budget_sec
                          << std::endl;
                break;
            }
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
            shortcutJointPath(
                joint_path,
                input,
                common_cfg_.joint_space_shortcut_trials);

            JointPathCandidate candidate = evaluateJointPathCandidate(
                joint_path,
                input);
            candidate.source_stage = stage_index;
            if (std::isfinite(candidate.score)) {
                ++stage_candidate_count;
                if (!std::isfinite(stage_best_candidate.score) ||
                    candidate.score + 1e-9 < stage_best_candidate.score) {
                    stage_best_candidate = candidate;
                }
            }
            insertCandidate(std::move(candidate));

            if (stage_candidate_count >= per_stage_solution_target &&
                (iter + 1) >= min_stage_iterations_before_early_stop) {
                std::cout << "[planner] joint-space stage_early_stop stage=" << stage_index
                          << " iterations=" << (iter + 1)
                          << " candidates=" << stage_candidate_count
                          << " target=" << per_stage_solution_target
                          << std::endl;
                break;
            }
        }

        if (stage_candidate_count > 0) {
            std::cout << "[planner] joint-space stage_result stage=" << stage_index
                      << " candidates=" << stage_candidate_count
                      << " best_score=" << stage_best_candidate.score
                      << " best_cartesian_length=" << stage_best_candidate.cartesian_length
                      << " best_joint_motion=" << stage_best_candidate.joint_motion
                      << " best_clearance_deficit=" << stage_best_candidate.mean_clearance_deficit
                      << " best_early_clearance_deficit="
                      << stage_best_candidate.mean_early_clearance_deficit
                      << " best_min_margin=" << stage_best_candidate.min_margin
                      << " best_early_min_margin=" << stage_best_candidate.early_min_margin
                      << " best_clearance_reward="
                      << stage_best_candidate.mean_clearance_reward
                      << " best_worst_link=" << stage_best_candidate.worst_link_name
                      << " best_early_worst_link=" << stage_best_candidate.early_worst_link_name
                      << std::endl;
        } else {
            std::cout << "[planner] joint-space stage_result stage=" << stage_index
                      << " candidates=0" << std::endl;
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
        runSearchStage(q_min, q_max, iterations_per_stage, stage + 1);
    }

    if (candidates.empty() &&
        has_hard_bounds &&
        common_cfg_.joint_space_sampling_allow_full_joint_limit_fallback) {
        Eigen::VectorXd q_min = hard_q_min;
        Eigen::VectorXd q_max = hard_q_max;
        clampBounds(&q_min, &q_max);
        std::cout << "[planner] joint-space sampling fallback=full_joint_limits iterations="
                  << max_iterations << std::endl;
        runSearchStage(q_min, q_max, max_iterations, search_stages + 1);
    }

    if (candidates.empty()) {
        std::cout << "[planner] joint-space sampling failed: iterations="
                  << max_iterations
                  << " stages=" << search_stages
                  << " safe_distance=" << input.safe_distance
                  << " feasibility_safe_distance=" << feasibilitySafeDistance(input)
                  << " hard_clearance=" << input.hard_clearance
                  << " valid_goals=" << valid_goals.size()
                  << std::endl;
        return out;
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const JointPathCandidate& a, const JointPathCandidate& b) {
            return a.score < b.score;
        });
    const double preferred_margin =
        std::max(0.0, common_cfg_.joint_space_preferred_min_margin_m);
    const double required_normal_margin =
        std::min(preferred_margin, std::max(0.0, input.safe_distance - feasibilitySafeDistance(input)));
    auto preferred_candidate_it = std::find_if(
        candidates.begin(),
        candidates.end(),
        [required_normal_margin](const JointPathCandidate& candidate) {
            return std::isfinite(candidate.min_margin) &&
                   candidate.min_margin >= required_normal_margin &&
                   (!std::isfinite(candidate.early_min_margin) ||
                    candidate.early_min_margin >= required_normal_margin);
        });
    const bool selected_preferred_margin = preferred_candidate_it != candidates.end();
    if (!selected_preferred_margin) {
        const JointPathCandidate& best_candidate = candidates.front();
        std::cout << "[planner] joint-space sampling rejected normal goal segment: candidates="
                  << candidates.size()
                  << " required_preferred_margin=" << preferred_margin
                  << " required_normal_margin=" << required_normal_margin
                  << " feasibility_safe_distance=" << feasibilitySafeDistance(input)
                  << " best_score=" << best_candidate.score
                  << " best_min_margin=" << best_candidate.min_margin
                  << " best_early_min_margin=" << best_candidate.early_min_margin
                  << " best_worst_link=" << best_candidate.worst_link_name
                  << " best_early_worst_link=" << best_candidate.early_worst_link_name
                  << std::endl;
        return out;
    }
    const JointPathCandidate& best_candidate = *preferred_candidate_it;
    std::cout << "[planner] joint-space candidate selected: candidates="
              << candidates.size()
              << " source_stage=" << best_candidate.source_stage
              << " preferred_margin_ok=true"
              << " score=" << best_candidate.score
              << " cartesian_length=" << best_candidate.cartesian_length
              << " joint_motion=" << best_candidate.joint_motion
              << " clearance_deficit=" << best_candidate.mean_clearance_deficit
              << " early_clearance_deficit=" << best_candidate.mean_early_clearance_deficit
              << " min_margin=" << best_candidate.min_margin
              << " early_min_margin=" << best_candidate.early_min_margin
              << " clearance_reward=" << best_candidate.mean_clearance_reward
              << " worst_link=" << best_candidate.worst_link_name
              << " early_worst_link=" << best_candidate.early_worst_link_name
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
    out.segment_kind = PlannedSegmentKind::Goal;
    out.success = out.path.waypoints.size() >= 2;
    return out;
}

PathPlanningOutput CartesianPathPlanner::planOmplRrtConnectPath(
    const PathPlanningInput& input,
    const Eigen::VectorXd& q_start,
    const std::vector<Eigen::VectorXd>& valid_goals) const {
    PathPlanningOutput out;
    if (q_start.size() <= 0 || valid_goals.empty() || !input.joint_state_validator ||
        !input.joint_segment_validator || !input.joint_to_pose_fn) {
        return out;
    }
    if (input.q_min.size() != q_start.size() || input.q_max.size() != q_start.size() ||
        !input.q_min.allFinite() || !input.q_max.allFinite()) {
        std::cout << "[planner] ompl_rrt_connect skipped: invalid joint bounds" << std::endl;
        return out;
    }

    auto space = std::make_shared<ob::RealVectorStateSpace>(
        static_cast<unsigned int>(q_start.size()));
    ob::RealVectorBounds bounds(static_cast<unsigned int>(q_start.size()));
    for (Eigen::Index i = 0; i < q_start.size(); ++i) {
        const double lo = std::min(input.q_min[i], input.q_max[i]);
        const double hi = std::max(input.q_min[i], input.q_max[i]);
        if (!(std::isfinite(lo) && std::isfinite(hi)) || lo > hi) {
            return out;
        }
        bounds.setLow(static_cast<unsigned int>(i), lo);
        bounds.setHigh(static_cast<unsigned int>(i), hi);
    }
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    const auto stateToEigen = [dof = q_start.size()](const ob::State* state) {
        const auto* rv_state = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd q(dof);
        for (Eigen::Index i = 0; i < dof; ++i) {
            q[i] = rv_state->values[i];
        }
        return q;
    };
    const auto eigenToState = [](const Eigen::VectorXd& q, ob::ScopedState<>& state) {
        auto* rv_state = state->as<ob::RealVectorStateSpace::StateType>();
        for (Eigen::Index i = 0; i < q.size(); ++i) {
            rv_state->values[i] = q[i];
        }
    };
    const double bounds_extent = std::max(1e-6, jointDistance(input.q_min, input.q_max));

    si->setStateValidityChecker([&](const ob::State* state) {
        const Eigen::VectorXd q = stateToEigen(state);
        return validateJointState(q, input, input.safe_distance, nullptr);
    });
    class WholeBodyMotionValidator final : public ob::MotionValidator {
    public:
        WholeBodyMotionValidator(
            const ob::SpaceInformationPtr& si,
            const PathPlanningInput& input,
            std::function<Eigen::VectorXd(const ob::State*)> state_to_eigen)
            : ob::MotionValidator(si),
              input_(input),
              state_to_eigen_(std::move(state_to_eigen)) {}

        bool checkMotion(const ob::State* s1, const ob::State* s2) const override {
            return validateJointSegment(
                state_to_eigen_(s1),
                state_to_eigen_(s2),
                input_);
        }

        bool checkMotion(
            const ob::State* s1,
            const ob::State* s2,
            std::pair<ob::State*, double>& last_valid) const override {
            const bool ok = checkMotion(s1, s2);
            if (ok) {
                if (last_valid.first != nullptr) {
                    si_->copyState(last_valid.first, s2);
                }
                last_valid.second = 1.0;
            } else {
                if (last_valid.first != nullptr) {
                    si_->copyState(last_valid.first, s1);
                }
                last_valid.second = 0.0;
            }
            return ok;
        }

    private:
        const PathPlanningInput& input_;
        std::function<Eigen::VectorXd(const ob::State*)> state_to_eigen_;
    };
    si->setMotionValidator(std::make_shared<WholeBodyMotionValidator>(
        si,
        input,
        stateToEigen));
    si->setStateValidityCheckingResolution(
        std::clamp(common_cfg_.joint_space_sampling_step_rad / bounds_extent, 0.002, 0.05));
    si->setup();

    ob::ScopedState<> start(space);
    eigenToState(q_start, start);
    og::RRTConnect planner(si);
    planner.setRange(std::max(1e-3, common_cfg_.joint_space_sampling_step_rad));

    const double solve_time_sec =
        std::max(0.05, common_cfg_.joint_space_sampling_time_budget_sec);
    std::vector<JointPathCandidate> candidates;
    candidates.reserve(valid_goals.size());
    for (std::size_t goal_index = 0; goal_index < valid_goals.size(); ++goal_index) {
        const Eigen::VectorXd& q_goal = valid_goals[goal_index];
        if (q_goal.size() != q_start.size() || !q_goal.allFinite()) {
            continue;
        }

        ob::ScopedState<> goal(space);
        eigenToState(q_goal, goal);
        planner.clear();
        ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal);
        planner.setProblemDefinition(pdef);
        planner.setup();
        const ob::PlannerStatus solved =
            planner.solve(ob::timedPlannerTerminationCondition(solve_time_sec));
        if (!solved) {
            continue;
        }

        auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        if (!path || path->getStateCount() < 2) {
            continue;
        }
        path->interpolate();
        std::vector<Eigen::VectorXd> joint_path;
        joint_path.reserve(path->getStateCount());
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            joint_path.push_back(stateToEigen(path->getState(i)));
        }
        shortcutJointPath(
            joint_path,
            input,
            common_cfg_.joint_space_shortcut_trials);

        JointPathCandidate candidate = evaluateJointPathCandidate(
            joint_path,
            input);
        candidate.source_stage = static_cast<int>(goal_index) + 1;
        if (std::isfinite(candidate.score)) {
            candidates.push_back(std::move(candidate));
        }
    }

    if (candidates.empty()) {
        std::cout << "[planner] ompl_rrt_connect failed: goals=" << valid_goals.size()
                  << " solve_time_sec=" << solve_time_sec
                  << " range=" << std::max(1e-3, common_cfg_.joint_space_sampling_step_rad)
                  << std::endl;
        return out;
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const JointPathCandidate& a, const JointPathCandidate& b) {
            return a.score < b.score;
        });
    const double preferred_margin =
        std::max(0.0, common_cfg_.joint_space_preferred_min_margin_m);
    const double required_normal_margin =
        std::min(preferred_margin, std::max(0.0, input.safe_distance - feasibilitySafeDistance(input)));
    auto best_it = std::find_if(
        candidates.begin(),
        candidates.end(),
        [required_normal_margin](const JointPathCandidate& candidate) {
            return std::isfinite(candidate.min_margin) &&
                   candidate.min_margin >= required_normal_margin &&
                   (!std::isfinite(candidate.early_min_margin) ||
                    candidate.early_min_margin >= required_normal_margin);
        });
    if (best_it == candidates.end()) {
        const JointPathCandidate& best_candidate = candidates.front();
        std::cout << "[planner] ompl_rrt_connect rejected normal goal segment: candidates="
                  << candidates.size()
                  << " required_normal_margin=" << required_normal_margin
                  << " best_min_margin=" << best_candidate.min_margin
                  << " best_early_min_margin=" << best_candidate.early_min_margin
                  << " best_worst_link=" << best_candidate.worst_link_name
                  << std::endl;
        return out;
    }

    const JointPathCandidate& best_candidate = *best_it;
    out.joint_waypoints = best_candidate.joint_path;
    out.path.waypoints.reserve(out.joint_waypoints.size());
    for (const auto& q : out.joint_waypoints) {
        CartesianWaypoint wp;
        if (!input.joint_to_pose_fn(q, wp)) {
            return {};
        }
        out.path.waypoints.push_back(wp);
    }
    double length = 0.0;
    for (std::size_t i = 1; i < out.path.waypoints.size(); ++i) {
        length += (out.path.waypoints[i].position - out.path.waypoints[i - 1].position).norm();
    }
    out.path.length = length;
    out.segment_kind = PlannedSegmentKind::Goal;
    out.success = out.path.waypoints.size() >= 2;
    std::cout << "[planner] ompl_rrt_connect selected: candidates=" << candidates.size()
              << " source_goal=" << best_candidate.source_stage
              << " score=" << best_candidate.score
              << " cartesian_length=" << best_candidate.cartesian_length
              << " joint_motion=" << best_candidate.joint_motion
              << " min_margin=" << best_candidate.min_margin
              << " early_min_margin=" << best_candidate.early_min_margin
              << " worst_link=" << best_candidate.worst_link_name
              << std::endl;
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
    traj.segment_kind = PlannedSegmentKind::Goal;
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
    // Keep the Cartesian reference pose fully consistent with the joint path FK.
    // Overwriting waypoint orientations with an independent start-goal slerp can
    // create a pose/twist field that no sampled joint state actually realizes.

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
    CartesianWaypointList reference_waypoints;
    reference_waypoints.reserve(joint_path.size());
    for (const auto& q : joint_path) {
        CartesianWaypoint wp;
        if (!input.joint_to_pose_fn(q, wp)) {
            return joint_path;
        }
        reference_waypoints.push_back(wp);
    }

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
            const std::size_t waypoint_index) -> double {
        PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateJointState(q_candidate, input, feasibilitySafeDistance(input), &diag) ||
            !validateJointSegment(q_prev, q_candidate, input) ||
            !validateJointSegment(q_candidate, q_next, input)) {
            return std::numeric_limits<double>::infinity();
        }
        PathPlanningInput::WholeBodyPoseDiagnostic preference_diag;
        (void)validateJointState(q_candidate, input, input.safe_distance, &preference_diag);
        if (std::isfinite(preference_diag.min_margin)) {
            diag = preference_diag;
        }

        CartesianWaypoint wp_prev;
        CartesianWaypoint wp_candidate;
        CartesianWaypoint wp_next;
        if (!input.joint_to_pose_fn(q_prev, wp_prev) ||
            !input.joint_to_pose_fn(q_candidate, wp_candidate) ||
            !input.joint_to_pose_fn(q_next, wp_next)) {
            return std::numeric_limits<double>::infinity();
        }

        const CartesianWaypoint& wp_reference = reference_waypoints[waypoint_index];
        const double orientation_cost =
            orientationError(wp_reference.orientation, wp_candidate.orientation);
        const double smoothness_cost =
            (q_prev - 2.0 * q_candidate + q_next).squaredNorm();
        const double position_cost =
            (wp_candidate.position - wp_reference.position).squaredNorm();
        const double joint_reference_cost =
            (q_candidate - joint_path[waypoint_index]).squaredNorm();
        const double local_geometry_cost =
            (wp_candidate.position - 0.5 * (wp_prev.position + wp_next.position)).squaredNorm();
        const double clearance_reward = std::min(
            std::max(0.0, diag.min_margin),
            std::max(1e-6, common_cfg_.joint_space_clearance_reward_cap_m));
        return common_cfg_.joint_trajectory_orientation_weight * orientation_cost +
               common_cfg_.joint_trajectory_smoothness_weight * smoothness_cost +
               common_cfg_.joint_trajectory_position_weight * position_cost +
               0.5 * common_cfg_.joint_trajectory_smoothness_weight * joint_reference_cost +
               0.25 * common_cfg_.joint_trajectory_position_weight * local_geometry_cost -
               common_cfg_.joint_space_clearance_reward_weight * clearance_reward;
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

            Eigen::VectorXd best_q = optimized[i];
            double best_score = evaluateWaypointCandidate(q_prev, best_q, q_next, i);

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
                    evaluateWaypointCandidate(q_prev, q_candidate, q_next, i);
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

    double orientation_accum = 0.0;
    double position_accum = 0.0;
    int counted = 0;
    for (std::size_t i = 1; i + 1 < optimized.size(); ++i) {
        CartesianWaypoint wp;
        if (!input.joint_to_pose_fn(optimized[i], wp)) {
            continue;
        }
        orientation_accum += orientationError(reference_waypoints[i].orientation, wp.orientation);
        position_accum +=
            (wp.position - reference_waypoints[i].position).norm();
        ++counted;
    }
    if (counted > 0) {
        std::cout << "[planner] joint-trajectory postopt: waypoints=" << optimized.size()
                  << " avg_reference_orientation_err=" << (orientation_accum / counted)
                  << " avg_reference_position_err=" << (position_accum / counted)
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

    CartesianWaypointList waypoints;
    waypoints.reserve(joint_path.size());
    for (const auto& q : joint_path) {
        CartesianWaypoint wp;
        if (!input.joint_to_pose_fn(q, wp)) {
            return candidate;
        }
        waypoints.push_back(wp);
    }

    double path_length = 0.0;
    double joint_motion = 0.0;
    double clearance_deficit_accum = 0.0;
    double early_clearance_deficit_accum = 0.0;
    double clearance_reward_accum = 0.0;
    int clearance_samples = 0;
    int early_clearance_samples = 0;
    double min_margin = std::numeric_limits<double>::infinity();
    double early_min_margin = std::numeric_limits<double>::infinity();
    std::string worst_link_name;
    std::string early_worst_link_name;
    const int early_window =
        std::max(1, static_cast<int>(std::ceil(0.5 * static_cast<double>(joint_path.size()))));

    for (std::size_t i = 0; i < joint_path.size(); ++i) {
        PathPlanningInput::WholeBodyPoseDiagnostic diag;
        PathPlanningInput::WholeBodyPoseDiagnostic preference_diag;
        if (!validateJointState(joint_path[i], input, feasibilitySafeDistance(input), &diag)) {
            return candidate;
        }
        (void)validateJointState(joint_path[i], input, input.safe_distance, &preference_diag);
        if (std::isfinite(preference_diag.min_margin)) {
            diag = preference_diag;
        }
        if (std::isfinite(diag.min_margin)) {
            const double deficit = std::max(0.0, -diag.min_margin);
            clearance_deficit_accum += deficit;
            clearance_reward_accum += std::min(
                std::max(0.0, diag.min_margin),
                std::max(1e-6, common_cfg_.joint_space_clearance_reward_cap_m));
            ++clearance_samples;
            if (diag.min_margin < min_margin) {
                min_margin = diag.min_margin;
                worst_link_name = diag.worst_link_name;
            }
            if (static_cast<int>(i) < early_window) {
                early_clearance_deficit_accum += deficit;
                ++early_clearance_samples;
                if (diag.min_margin < early_min_margin) {
                    early_min_margin = diag.min_margin;
                    early_worst_link_name = diag.worst_link_name;
                }
            }
        }
        if (i > 0) {
            path_length += (waypoints[i].position - waypoints[i - 1].position).norm();
            joint_motion += jointDistance(joint_path[i - 1], joint_path[i]);
        }
    }

    candidate.cartesian_length = path_length;
    candidate.joint_motion = joint_motion;
    const double mean_clearance_deficit =
        clearance_samples > 0 ? (clearance_deficit_accum / static_cast<double>(clearance_samples))
                              : 0.0;
    const double mean_early_clearance_deficit =
        early_clearance_samples > 0
            ? (early_clearance_deficit_accum / static_cast<double>(early_clearance_samples))
            : 0.0;
    candidate.mean_clearance_deficit = mean_clearance_deficit;
    candidate.mean_early_clearance_deficit = mean_early_clearance_deficit;
    candidate.mean_clearance_reward =
        clearance_samples > 0 ? (clearance_reward_accum / static_cast<double>(clearance_samples))
                              : 0.0;
    candidate.min_margin = min_margin;
    candidate.early_min_margin = early_min_margin;
    candidate.worst_link_name = worst_link_name;
    candidate.early_worst_link_name = early_worst_link_name;
    const double min_clearance_deficit =
        std::isfinite(min_margin) ? std::max(0.0, -min_margin) : 0.0;
    const double early_min_clearance_deficit =
        std::isfinite(early_min_margin) ? std::max(0.0, -early_min_margin) : 0.0;
    const double preferred_margin =
        std::max(0.0, common_cfg_.joint_space_preferred_min_margin_m);
    const double min_margin_preference_deficit =
        std::isfinite(min_margin) ? std::max(0.0, preferred_margin - min_margin) : preferred_margin;
    const double early_min_margin_preference_deficit =
        std::isfinite(early_min_margin)
            ? std::max(0.0, preferred_margin - early_min_margin)
            : preferred_margin;
    candidate.score =
        common_cfg_.joint_space_path_length_weight * candidate.cartesian_length +
        common_cfg_.joint_space_joint_motion_weight * candidate.joint_motion +
        common_cfg_.joint_space_clearance_deficit_weight * mean_clearance_deficit +
        common_cfg_.joint_space_early_clearance_deficit_weight * mean_early_clearance_deficit +
        common_cfg_.joint_space_min_clearance_deficit_weight * min_clearance_deficit +
        common_cfg_.joint_space_early_min_clearance_deficit_weight *
            early_min_clearance_deficit -
        common_cfg_.joint_space_clearance_reward_weight * candidate.mean_clearance_reward +
        common_cfg_.joint_space_min_margin_preference_weight *
            (min_margin_preference_deficit + 0.5 * early_min_margin_preference_deficit);
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
