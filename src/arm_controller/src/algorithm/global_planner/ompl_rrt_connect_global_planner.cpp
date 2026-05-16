#include "algorithm/global_planner/ompl_rrt_connect_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iostream>
#include <random>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace arm_controller::algorithm::global_planner {

namespace {

namespace ob = ompl::base;
namespace og = ompl::geometric;

double feasibilitySafeDistance(const cp::PathPlanningInput& input) {
    if (std::isfinite(input.feasibility_safe_distance) &&
        input.feasibility_safe_distance > 0.0) {
        return std::min(input.safe_distance, input.feasibility_safe_distance);
    }
    return input.safe_distance;
}

bool isFiniteNonEmpty(const Eigen::VectorXd& q) {
    return q.size() > 0 && q.allFinite();
}

bool validateJointState(
    const Eigen::VectorXd& q,
    const cp::PathPlanningInput& input,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
    if (!input.joint_state_validator) {
        return false;
    }
    return input.joint_state_validator(q, safe_distance, diag);
}

bool validateJointSegment(
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    const cp::PathPlanningInput& input,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
    if (!input.joint_segment_validator) {
        return false;
    }
    return input.joint_segment_validator(q_from, q_to, feasibilitySafeDistance(input), diag);
}

}  // namespace

OmplRrtConnectGlobalPlanner::OmplRrtConnectGlobalPlanner(
    const cp::PlannerCommonConfig& common_cfg,
    const cp::SmoothingConfig& /*smoothing_cfg*/)
    : common_cfg_(common_cfg) {}

cp::TimedJointTrajectory OmplRrtConnectGlobalPlanner::planTrajectory(
    const cp::PathPlanningInput& input) {
    std::fprintf(
        stderr,
        "[global_planner] planTrajectory: entered goals=%zu has_start=%s\n",
        input.q_goal_candidates.size(),
        input.q_start_seed.has_value() ? "true" : "false");
    std::fflush(stderr);
    const cp::PathPlanningOutput output = planPath(input);
    if (!output.success || output.joint_waypoints.size() < 2) {
        return {};
    }

    cp::TimedJointTrajectory traj =
        buildJointSpaceTrajectory(optimizeJointTrajectory(output.joint_waypoints, input));
    traj.segment_kind = output.segment_kind;
    return traj;
}

cp::PathPlanningOutput OmplRrtConnectGlobalPlanner::planPath(
    const cp::PathPlanningInput& input) const {
    std::fprintf(
        stderr,
        "[global_planner] planPath: entered goals=%zu has_start=%s validators(state/segment/pose)=%s/%s/%s\n",
        input.q_goal_candidates.size(),
        input.q_start_seed.has_value() ? "true" : "false",
        input.joint_state_validator ? "true" : "false",
        input.joint_segment_validator ? "true" : "false",
        input.joint_to_pose_fn ? "true" : "false");
    std::fflush(stderr);
    cp::PathPlanningOutput out;
    if (!input.q_start_seed.has_value() || !isFiniteNonEmpty(*input.q_start_seed) ||
        input.q_goal_candidates.empty() || !input.joint_state_validator ||
        !input.joint_segment_validator || !input.joint_to_pose_fn) {
        std::cout << "[global_planner] rrt_connect missing required inputs" << std::endl;
        return out;
    }

    const Eigen::VectorXd q_start = *input.q_start_seed;
    const double feasibility_safe_distance = feasibilitySafeDistance(input);
    cp::PathPlanningInput::WholeBodyPoseDiagnostic start_diag;
    std::fprintf(stderr, "[global_planner] planPath: validate_start begin qdim=%ld\n",
                 static_cast<long>(q_start.size()));
    std::fflush(stderr);
    const bool start_valid =
        validateJointState(q_start, input, feasibility_safe_distance, &start_diag);
    std::fprintf(stderr, "[global_planner] planPath: validate_start returned ok=%s\n",
                 start_valid ? "true" : "false");
    std::fflush(stderr);
    if (!start_valid) {
        std::cout << "[global_planner] rrt_connect rejected start: reason="
                  << start_diag.reason
                  << " margin=" << start_diag.min_margin
                  << " worst_link=" << start_diag.worst_link_name
                  << " safe_distance=" << feasibility_safe_distance
                  << " preference_safe_distance=" << input.safe_distance << std::endl;
        return out;
    }
    std::fprintf(stderr, "[global_planner] planPath: validate_start ok margin=%.6f reason=%s\n",
                 start_diag.min_margin,
                 start_diag.reason.c_str());
    std::fflush(stderr);

    std::vector<std::size_t> valid_goal_indices;
    valid_goal_indices.reserve(input.q_goal_candidates.size());
    for (std::size_t goal_candidate_index = 0;
         goal_candidate_index < input.q_goal_candidates.size();
         ++goal_candidate_index) {
        const Eigen::VectorXd& q_goal =
            input.q_goal_candidates[goal_candidate_index];
        if (q_goal.size() != q_start.size() || !q_goal.allFinite()) {
            continue;
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic goal_diag;
        std::fprintf(stderr, "[global_planner] planPath: validate_goal begin qdim=%ld\n",
                     static_cast<long>(q_goal.size()));
        std::fflush(stderr);
        if (!validateJointState(q_goal, input, feasibility_safe_distance, &goal_diag)) {
            std::cout << "[global_planner] rrt_connect rejected goal: reason="
                      << goal_diag.reason
                      << " margin=" << goal_diag.min_margin
                      << " worst_link=" << goal_diag.worst_link_name
                      << " safe_distance=" << feasibility_safe_distance
                      << " preference_safe_distance=" << input.safe_distance << std::endl;
            continue;
        }
        std::fprintf(stderr, "[global_planner] planPath: validate_goal ok margin=%.6f reason=%s\n",
                     goal_diag.min_margin,
                     goal_diag.reason.c_str());
        std::fflush(stderr);
        valid_goal_indices.push_back(goal_candidate_index);
    }
    if (valid_goal_indices.empty()) {
        std::cout << "[global_planner] rrt_connect no valid goal states" << std::endl;
        return out;
    }

    if (input.q_min.size() != q_start.size() || input.q_max.size() != q_start.size() ||
        !input.q_min.allFinite() || !input.q_max.allFinite()) {
        std::cout << "[global_planner] rrt_connect skipped: invalid joint bounds" << std::endl;
        return out;
    }

    std::fprintf(stderr, "[global_planner] planPath: create_state_space begin valid_goals=%zu\n",
                 valid_goal_indices.size());
    std::fflush(stderr);
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
    std::fprintf(stderr, "[global_planner] planPath: create_state_space ok\n");
    std::fflush(stderr);

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
            const cp::PathPlanningInput& input,
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
        const cp::PathPlanningInput& input_;
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
    candidates.reserve(valid_goal_indices.size());
    for (std::size_t goal_index = 0; goal_index < valid_goal_indices.size(); ++goal_index) {
        const Eigen::VectorXd& q_goal =
            input.q_goal_candidates[valid_goal_indices[goal_index]];
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
        const std::size_t raw_state_count = path->getStateCount();
        path->interpolate();
        const std::size_t interpolated_state_count = path->getStateCount();
        std::vector<Eigen::VectorXd> joint_path;
        joint_path.reserve(path->getStateCount());
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            joint_path.push_back(stateToEigen(path->getState(i)));
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic direct_diag;
        const bool direct_segment_valid =
            validateJointSegment(q_start, q_goal, input, &direct_diag);
        shortcutJointPath(
            joint_path,
            input,
            common_cfg_.joint_space_shortcut_trials);
        std::cout << "[global_planner] rrt_connect path_detail: goal="
                  << (goal_index + 1)
                  << " raw_states=" << raw_state_count
                  << " interpolated_states=" << interpolated_state_count
                  << " shortcut_trials=" << common_cfg_.joint_space_shortcut_trials
                  << " output_states=" << joint_path.size()
                  << " direct_valid=" << (direct_segment_valid ? "true" : "false")
                  << " direct_margin=" << direct_diag.min_margin
                  << " direct_reason=" << direct_diag.reason
                  << " direct_worst_link=" << direct_diag.worst_link_name
                  << std::endl;

        JointPathCandidate candidate = evaluateJointPathCandidate(joint_path, input);
        candidate.source_stage = static_cast<int>(goal_index) + 1;
        if (std::isfinite(candidate.score)) {
            candidates.push_back(std::move(candidate));
        }
    }

    if (candidates.empty()) {
        std::cout << "[global_planner] rrt_connect failed: goals=" << valid_goal_indices.size()
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
        std::cout << "[global_planner] rrt_connect rejected segment: candidates="
                  << candidates.size()
                  << " required_margin=" << required_normal_margin
                  << " best_min_margin=" << best_candidate.min_margin
                  << " best_early_min_margin=" << best_candidate.early_min_margin
                  << " best_worst_link=" << best_candidate.worst_link_name
                  << std::endl;
        return out;
    }

    const JointPathCandidate& best_candidate = *best_it;
    out.joint_waypoints = best_candidate.joint_path;
    out.segment_kind = cp::PlannedSegmentKind::Goal;
    out.success = out.joint_waypoints.size() >= 2;
    std::cout << "[global_planner] rrt_connect selected: candidates=" << candidates.size()
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

cp::TimedJointTrajectory OmplRrtConnectGlobalPlanner::buildJointSpaceTrajectory(
    const std::vector<Eigen::VectorXd>& joint_path) const {
    cp::TimedJointTrajectory traj;
    if (joint_path.size() < 2) {
        return traj;
    }

    traj.joint_targets = joint_path;
    traj.segment_kind = cp::PlannedSegmentKind::Goal;
    traj.segment_durations.reserve(joint_path.size() - 1);
    traj.cumulative_times.reserve(joint_path.size());
    traj.cumulative_times.push_back(0.0);

    double total_time = 0.0;
    for (std::size_t i = 1; i < traj.joint_targets.size(); ++i) {
        const double joint_motion =
            (traj.joint_targets[i] - traj.joint_targets[i - 1]).norm();
        const double dt =
            std::max(1e-3, joint_motion / std::max(1e-3, common_cfg_.default_segment_speed));
        traj.segment_durations.push_back(dt);
        total_time += dt;
        traj.cumulative_times.push_back(total_time);
    }
    traj.total_duration = total_time;
    return traj;
}

std::vector<Eigen::VectorXd> OmplRrtConnectGlobalPlanner::optimizeJointTrajectory(
    const std::vector<Eigen::VectorXd>& joint_path,
    const cp::PathPlanningInput& input) const {
    if (!common_cfg_.enable_joint_trajectory_post_optimization ||
        common_cfg_.joint_trajectory_postopt_iterations <= 0 ||
        joint_path.size() < 3 || !input.joint_to_pose_fn ||
        !input.joint_state_validator || !input.joint_segment_validator) {
        return joint_path;
    }

    std::vector<Eigen::VectorXd> optimized = joint_path;
    cp::CartesianWaypointList reference_waypoints;
    reference_waypoints.reserve(joint_path.size());
    for (const auto& q : joint_path) {
        cp::CartesianWaypoint wp;
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
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateJointState(q_candidate, input, feasibilitySafeDistance(input), &diag) ||
            !validateJointSegment(q_prev, q_candidate, input) ||
            !validateJointSegment(q_candidate, q_next, input)) {
            return std::numeric_limits<double>::infinity();
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic preference_diag;
        (void)validateJointState(q_candidate, input, input.safe_distance, &preference_diag);
        if (std::isfinite(preference_diag.min_margin)) {
            diag = preference_diag;
        }

        cp::CartesianWaypoint wp_prev;
        cp::CartesianWaypoint wp_candidate;
        cp::CartesianWaypoint wp_next;
        if (!input.joint_to_pose_fn(q_prev, wp_prev) ||
            !input.joint_to_pose_fn(q_candidate, wp_candidate) ||
            !input.joint_to_pose_fn(q_next, wp_next)) {
            return std::numeric_limits<double>::infinity();
        }

        const cp::CartesianWaypoint& wp_reference = reference_waypoints[waypoint_index];
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

    return optimized;
}

OmplRrtConnectGlobalPlanner::JointPathCandidate
OmplRrtConnectGlobalPlanner::evaluateJointPathCandidate(
    const std::vector<Eigen::VectorXd>& joint_path,
    const cp::PathPlanningInput& input) const {
    JointPathCandidate candidate;
    candidate.joint_path = joint_path;
    if (joint_path.size() < 2 || !input.joint_to_pose_fn) {
        return candidate;
    }

    cp::CartesianWaypointList waypoints;
    waypoints.reserve(joint_path.size());
    for (const auto& q : joint_path) {
        cp::CartesianWaypoint wp;
        if (!input.joint_to_pose_fn(q, wp)) {
            return candidate;
        }
        waypoints.push_back(wp);
    }

    double clearance_deficit_accum = 0.0;
    double early_clearance_deficit_accum = 0.0;
    double clearance_reward_accum = 0.0;
    int clearance_samples = 0;
    int early_clearance_samples = 0;
    const int early_window =
        std::max(1, static_cast<int>(std::ceil(0.5 * static_cast<double>(joint_path.size()))));

    for (std::size_t i = 0; i < joint_path.size(); ++i) {
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        cp::PathPlanningInput::WholeBodyPoseDiagnostic preference_diag;
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
            if (diag.min_margin < candidate.min_margin) {
                candidate.min_margin = diag.min_margin;
                candidate.worst_link_name = diag.worst_link_name;
            }
            if (static_cast<int>(i) < early_window) {
                early_clearance_deficit_accum += deficit;
                ++early_clearance_samples;
                if (diag.min_margin < candidate.early_min_margin) {
                    candidate.early_min_margin = diag.min_margin;
                    candidate.early_worst_link_name = diag.worst_link_name;
                }
            }
        }
        if (i > 0) {
            candidate.cartesian_length +=
                (waypoints[i].position - waypoints[i - 1].position).norm();
            candidate.joint_motion += jointDistance(joint_path[i - 1], joint_path[i]);
        }
    }

    candidate.mean_clearance_deficit =
        clearance_samples > 0
            ? (clearance_deficit_accum / static_cast<double>(clearance_samples))
            : 0.0;
    candidate.mean_early_clearance_deficit =
        early_clearance_samples > 0
            ? (early_clearance_deficit_accum / static_cast<double>(early_clearance_samples))
            : 0.0;
    candidate.mean_clearance_reward =
        clearance_samples > 0
            ? (clearance_reward_accum / static_cast<double>(clearance_samples))
            : 0.0;

    const double min_clearance_deficit =
        std::isfinite(candidate.min_margin) ? std::max(0.0, -candidate.min_margin) : 0.0;
    const double early_min_clearance_deficit =
        std::isfinite(candidate.early_min_margin)
            ? std::max(0.0, -candidate.early_min_margin)
            : 0.0;
    const double preferred_margin =
        std::max(0.0, common_cfg_.joint_space_preferred_min_margin_m);
    const double min_margin_preference_deficit =
        std::isfinite(candidate.min_margin)
            ? std::max(0.0, preferred_margin - candidate.min_margin)
            : preferred_margin;
    const double early_min_margin_preference_deficit =
        std::isfinite(candidate.early_min_margin)
            ? std::max(0.0, preferred_margin - candidate.early_min_margin)
            : preferred_margin;

    candidate.score =
        common_cfg_.joint_space_path_length_weight * candidate.cartesian_length +
        common_cfg_.joint_space_joint_motion_weight * candidate.joint_motion +
        common_cfg_.joint_space_clearance_deficit_weight * candidate.mean_clearance_deficit +
        common_cfg_.joint_space_early_clearance_deficit_weight *
            candidate.mean_early_clearance_deficit +
        common_cfg_.joint_space_min_clearance_deficit_weight * min_clearance_deficit +
        common_cfg_.joint_space_early_min_clearance_deficit_weight *
            early_min_clearance_deficit -
        common_cfg_.joint_space_clearance_reward_weight * candidate.mean_clearance_reward +
        common_cfg_.joint_space_min_margin_preference_weight *
            (min_margin_preference_deficit + 0.5 * early_min_margin_preference_deficit);
    return candidate;
}

void OmplRrtConnectGlobalPlanner::shortcutJointPath(
    std::vector<Eigen::VectorXd>& joint_path,
    const cp::PathPlanningInput& input,
    const int shortcut_trials) const {
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

double OmplRrtConnectGlobalPlanner::jointDistance(
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& b) {
    if (a.size() == 0 || a.size() != b.size() || !a.allFinite() || !b.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    return (a - b).norm();
}

}  // namespace arm_controller::algorithm::global_planner
