#include "algorithm/global_planner/ompl_rrt_connect_global_planner.hpp"

#include <algorithm>
#include <array>
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

bool hasValidJointBounds(
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max,
    const Eigen::Index dof) {
    return q_min.size() == dof && q_max.size() == dof &&
           q_min.allFinite() && q_max.allFinite();
}

bool normalizeJointVectorToBounds(
    Eigen::VectorXd& q,
    const Eigen::VectorXd& reference,
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max) {
    if (!isFiniteNonEmpty(q) || q.size() != reference.size() ||
        !hasValidJointBounds(q_min, q_max, q.size())) {
        return false;
    }

    constexpr double kTwoPi = 2.0 * M_PI;
    constexpr double kEps = 1e-9;
    for (Eigen::Index i = 0; i < q.size(); ++i) {
        const double lo = std::min(q_min[i], q_max[i]);
        const double hi = std::max(q_min[i], q_max[i]);
        if (!(std::isfinite(lo) && std::isfinite(hi)) || lo > hi) {
            return false;
        }

        double qi = q[i];
        if (qi < lo - kEps || qi > hi + kEps) {
            const double k_center = std::round((reference[i] - qi) / kTwoPi);
            double best_q = std::numeric_limits<double>::quiet_NaN();
            double best_distance = std::numeric_limits<double>::infinity();
            for (int delta = -2; delta <= 2; ++delta) {
                const double candidate = qi + (k_center + static_cast<double>(delta)) * kTwoPi;
                if (candidate < lo - kEps || candidate > hi + kEps) {
                    continue;
                }
                const double distance = std::abs(candidate - reference[i]);
                if (distance < best_distance) {
                    best_distance = distance;
                    best_q = candidate;
                }
            }
            if (!std::isfinite(best_q)) {
                return false;
            }
            qi = best_q;
        }

        q[i] = std::clamp(qi, lo, hi);
    }
    return true;
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
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
    if (!input.joint_segment_validator) {
        return false;
    }
    return input.joint_segment_validator(q_from, q_to, safe_distance, diag);
}

double stateMargin(
    const Eigen::VectorXd& q,
    const cp::PathPlanningInput& input,
    const double safe_distance) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
    if (!validateJointState(q, input, safe_distance, &diag) ||
        !std::isfinite(diag.min_margin)) {
        return -std::numeric_limits<double>::infinity();
    }
    return diag.min_margin;
}

bool hasMeaningfulMargin(
    const cp::PathPlanningInput::WholeBodyPoseDiagnostic& diag) {
    if (!std::isfinite(diag.min_margin)) {
        return false;
    }
    return diag.collision_free || !diag.reason.empty() ||
           !diag.worst_link_name.empty() || diag.safe_distance_used > 0.0 ||
           diag.required_clearance > 0.0 || diag.worst_distance > 0.0 ||
           diag.has_failed_pose;
}

double normalizeAngle(const double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct GoalErrorBreakdown {
    double total{std::numeric_limits<double>::infinity()};
    double base_position{std::numeric_limits<double>::quiet_NaN()};
    double base_yaw{std::numeric_limits<double>::quiet_NaN()};
    double arm_joint{std::numeric_limits<double>::quiet_NaN()};
    double left_arm_joint{std::numeric_limits<double>::quiet_NaN()};
    double right_arm_joint{std::numeric_limits<double>::quiet_NaN()};
};

GoalErrorBreakdown computeGoalErrorBreakdown(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_goal) {
    GoalErrorBreakdown out;
    if (q.size() == 0 || q.size() != q_goal.size() ||
        !q.allFinite() || !q_goal.allFinite()) {
        return out;
    }

    out.total = (q - q_goal).norm();
    if (q.size() < 3) {
        return out;
    }

    out.base_position = (q.head<2>() - q_goal.head<2>()).norm();
    out.base_yaw = std::abs(normalizeAngle(q[2] - q_goal[2]));
    if (q.size() > 3) {
        out.arm_joint =
            (q.tail(q.size() - 3) - q_goal.tail(q_goal.size() - 3)).norm();
    }
    if (q.size() >= 9) {
        out.left_arm_joint = (q.segment(3, 6) - q_goal.segment(3, 6)).norm();
    }
    if (q.size() >= 15) {
        out.right_arm_joint = (q.segment(9, 6) - q_goal.segment(9, 6)).norm();
    }
    return out;
}

Eigen::Vector3d normalizedOrZero(const Eigen::Vector3d& v) {
    if (!v.allFinite()) {
        return Eigen::Vector3d::Zero();
    }
    const double n = v.norm();
    if (n < 1e-9) {
        return Eigen::Vector3d::Zero();
    }
    return v / n;
}

void appendDirection(
    std::vector<Eigen::Vector3d>& directions,
    const Eigen::Vector3d& direction) {
    const Eigen::Vector3d unit = normalizedOrZero(direction);
    if (unit.norm() < 1e-9) {
        return;
    }
    for (const auto& existing : directions) {
        if (existing.dot(unit) > 0.96) {
            return;
        }
    }
    directions.push_back(unit);
}

Eigen::Matrix3d slerpRotation(
    const Eigen::Matrix3d& from,
    const Eigen::Matrix3d& to,
    const double t) {
    Eigen::Quaterniond q_from(from);
    Eigen::Quaterniond q_to(to);
    q_from.normalize();
    q_to.normalize();
    return q_from.slerp(std::clamp(t, 0.0, 1.0), q_to).normalized().toRotationMatrix();
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
    cp::PathPlanningOutput direct_output = planDirectPath(input);
    if (!common_cfg_.enable_two_way_bypass) {
        return direct_output;
    }
    if (direct_output.success) {
        const JointPathCandidate direct_candidate =
            evaluateJointPathCandidate(direct_output.joint_waypoints, input);
        const double preferred_margin =
            std::max(0.0, common_cfg_.joint_space_preferred_min_margin_m);
        const bool direct_near_obstacle =
            std::isfinite(direct_candidate.min_margin) &&
            direct_candidate.min_margin < preferred_margin;
        if (!direct_near_obstacle) {
            return direct_output;
        }
        cp::PathPlanningOutput bypass_output = planTwoWayBypassPath(input);
        if (!bypass_output.success) {
            cp::PathPlanningOutput failed_output;
            std::cout << "[global_planner] adaptive_bypass failed: rejecting low-clearance "
                      << "direct path margin=" << direct_candidate.min_margin
                      << " preferred_margin=" << preferred_margin
                      << std::endl;
            return failed_output;
        }
        std::cout << "[global_planner] adaptive_bypass replaced low-clearance direct: "
                  << "direct_margin=" << direct_candidate.min_margin
                  << " preferred_margin=" << preferred_margin
                  << " direct_score=" << direct_candidate.score
                  << std::endl;
        return bypass_output;
    }
    cp::PathPlanningOutput bypass_output = planTwoWayBypassPath(input);
    if (bypass_output.success) {
        return bypass_output;
    }
    return direct_output;
}

cp::PathPlanningOutput OmplRrtConnectGlobalPlanner::planDirectPath(
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

    if (input.q_min.size() != input.q_start_seed->size() ||
        input.q_max.size() != input.q_start_seed->size() ||
        !input.q_min.allFinite() || !input.q_max.allFinite()) {
        std::cout << "[global_planner] rrt_connect skipped: invalid joint bounds" << std::endl;
        return out;
    }

    Eigen::VectorXd q_start = *input.q_start_seed;
    if (!normalizeJointVectorToBounds(q_start, *input.q_start_seed, input.q_min, input.q_max)) {
        std::cout << "[global_planner] rrt_connect rejected start: outside joint bounds"
                  << std::endl;
        return out;
    }
    const double start_normalization_delta = (q_start - *input.q_start_seed).norm();
    if (start_normalization_delta > 1e-6) {
        std::cout << "[global_planner] rrt_connect normalized start into bounds: delta="
                  << start_normalization_delta << std::endl;
    }
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

    std::vector<Eigen::VectorXd> valid_goals;
    valid_goals.reserve(input.q_goal_candidates.size());
    for (std::size_t goal_candidate_index = 0;
         goal_candidate_index < input.q_goal_candidates.size();
         ++goal_candidate_index) {
        Eigen::VectorXd q_goal = input.q_goal_candidates[goal_candidate_index];
        if (q_goal.size() != q_start.size() || !q_goal.allFinite()) {
            continue;
        }
        const Eigen::VectorXd q_goal_raw = q_goal;
        if (!normalizeJointVectorToBounds(q_goal, q_start, input.q_min, input.q_max)) {
            std::cout << "[global_planner] rrt_connect rejected goal: outside joint bounds"
                      << " candidate=" << (goal_candidate_index + 1)
                      << std::endl;
            continue;
        }
        const double goal_normalization_delta = (q_goal - q_goal_raw).norm();
        if (goal_normalization_delta > 1e-6) {
            std::cout << "[global_planner] rrt_connect normalized goal into bounds: candidate="
                      << (goal_candidate_index + 1)
                      << " delta=" << goal_normalization_delta << std::endl;
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
        valid_goals.push_back(std::move(q_goal));
    }
    if (valid_goals.empty()) {
        std::cout << "[global_planner] rrt_connect no valid goal states" << std::endl;
        return out;
    }

    std::fprintf(stderr, "[global_planner] planPath: create_state_space begin valid_goals=%zu\n",
                 valid_goals.size());
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

    const double ompl_safe_distance = feasibility_safe_distance;
    si->setStateValidityChecker([&](const ob::State* state) {
        const Eigen::VectorXd q = stateToEigen(state);
        return validateJointState(q, input, ompl_safe_distance, nullptr);
    });

    class WholeBodyMotionValidator final : public ob::MotionValidator {
    public:
        WholeBodyMotionValidator(
            const ob::SpaceInformationPtr& si,
            const cp::PathPlanningInput& input,
            std::function<Eigen::VectorXd(const ob::State*)> state_to_eigen,
            const double safe_distance)
            : ob::MotionValidator(si),
              input_(input),
              state_to_eigen_(std::move(state_to_eigen)),
              safe_distance_(safe_distance) {}

        bool checkMotion(const ob::State* s1, const ob::State* s2) const override {
            return validateJointSegment(
                state_to_eigen_(s1),
                state_to_eigen_(s2),
                input_,
                safe_distance_);
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
        double safe_distance_{0.0};
    };
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
    int solved_paths = 0;
    int accepted_approximate_paths = 0;
    int invalid_solution_paths = 0;
    si->setMotionValidator(std::make_shared<WholeBodyMotionValidator>(
        si,
        input,
        stateToEigen,
        ompl_safe_distance));
    si->setup();
    for (std::size_t goal_index = 0; goal_index < valid_goals.size(); ++goal_index) {
        const Eigen::VectorXd& q_goal = valid_goals[goal_index];
        if (q_goal.size() != q_start.size() || !q_goal.allFinite()) {
            continue;
        }
        const GoalErrorBreakdown start_goal_error =
            computeGoalErrorBreakdown(q_start, q_goal);
        std::cout << "[global_planner] rrt_connect goal_error: goal="
                  << (goal_index + 1)
                  << " start_distance=" << start_goal_error.total
                  << " start_base_pos_error=" << start_goal_error.base_position
                  << " start_base_yaw_error=" << start_goal_error.base_yaw
                  << " start_arm_joint_error=" << start_goal_error.arm_joint
                  << " start_left_arm_error=" << start_goal_error.left_arm_joint
                  << " start_right_arm_error=" << start_goal_error.right_arm_joint
                  << std::endl;

        ob::ScopedState<> goal(space);
        eigenToState(q_goal, goal);
        planner.clear();
        ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal);
        planner.setProblemDefinition(pdef);
        planner.setup();
        const ob::PlannerStatus solved =
            planner.solve(ob::timedPlannerTerminationCondition(solve_time_sec));
        const bool exact_solution = solved == ob::PlannerStatus::EXACT_SOLUTION;
        const bool approximate_solution =
            solved == ob::PlannerStatus::APPROXIMATE_SOLUTION;
        if (!exact_solution && !approximate_solution) {
            continue;
        }

        auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        if (!path || path->getStateCount() < 2) {
            continue;
        }
        Eigen::VectorXd approximate_tail = stateToEigen(path->getState(path->getStateCount() - 1));
        const GoalErrorBreakdown approximate_goal_error =
            computeGoalErrorBreakdown(approximate_tail, q_goal);
        double approximate_goal_distance = approximate_goal_error.total;
        const double approximate_accept_distance =
            std::max({input.goal_tolerance,
                      common_cfg_.joint_space_sampling_connect_threshold_rad,
                      common_cfg_.joint_space_sampling_step_rad});
        if (!exact_solution) {
            cp::PathPlanningInput::WholeBodyPoseDiagnostic approx_diag;
            const bool close_enough =
                approximate_goal_distance <= approximate_accept_distance;
            const bool tail_to_goal_valid =
                close_enough &&
                validateJointSegment(
                    approximate_tail,
                    q_goal,
                    input,
                    ompl_safe_distance,
                    &approx_diag);
            if (!tail_to_goal_valid) {
                std::cout << "[global_planner] rrt_connect rejected approximate solution: goal="
                          << (goal_index + 1)
                          << " tail_distance=" << approximate_goal_distance
                          << " base_pos_error=" << approximate_goal_error.base_position
                          << " base_yaw_error=" << approximate_goal_error.base_yaw
                          << " arm_joint_error=" << approximate_goal_error.arm_joint
                          << " left_arm_error=" << approximate_goal_error.left_arm_joint
                          << " right_arm_error=" << approximate_goal_error.right_arm_joint
                          << " accept_distance=" << approximate_accept_distance
                          << " tail_valid=" << (close_enough ? "false" : "skipped")
                          << " tail_margin=" << approx_diag.min_margin
                          << " tail_reason=" << approx_diag.reason
                          << std::endl;
                continue;
            }
            ++accepted_approximate_paths;
            std::cout << "[global_planner] rrt_connect accepted approximate solution: goal="
                      << (goal_index + 1)
                      << " tail_distance=" << approximate_goal_distance
                      << " base_pos_error=" << approximate_goal_error.base_position
                      << " base_yaw_error=" << approximate_goal_error.base_yaw
                      << " arm_joint_error=" << approximate_goal_error.arm_joint
                      << " left_arm_error=" << approximate_goal_error.left_arm_joint
                      << " right_arm_error=" << approximate_goal_error.right_arm_joint
                      << " accept_distance=" << approximate_accept_distance
                      << " tail_margin=" << approx_diag.min_margin
                      << " tail_reason=" << approx_diag.reason
                      << std::endl;
        } else {
            ++solved_paths;
        }
        const std::size_t raw_state_count = path->getStateCount();
        path->interpolate();
        const std::size_t interpolated_state_count = path->getStateCount();
        std::vector<Eigen::VectorXd> joint_path;
        joint_path.reserve(path->getStateCount());
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            joint_path.push_back(stateToEigen(path->getState(i)));
        }
        if (!exact_solution && !joint_path.empty() &&
            jointDistance(joint_path.back(), q_goal) > 1e-9) {
            joint_path.push_back(q_goal);
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic direct_diag;
        const bool direct_segment_valid =
            validateJointSegment(q_start, q_goal, input, ompl_safe_distance, &direct_diag);
        if (direct_segment_valid) {
            std::vector<Eigen::VectorXd> direct_joint_path;
            direct_joint_path.reserve(2u);
            direct_joint_path.push_back(q_start);
            direct_joint_path.push_back(q_goal);
            JointPathCandidate direct_candidate =
                evaluateJointPathCandidate(direct_joint_path, input);
            direct_candidate.source_stage = static_cast<int>(goal_index) + 1;
            if (std::isfinite(direct_candidate.score)) {
                candidates.push_back(std::move(direct_candidate));
            }
        }
        shortcutJointPath(
            joint_path,
            input,
            common_cfg_.joint_space_shortcut_trials,
            ompl_safe_distance);
        std::cout << "[global_planner] rrt_connect path_detail: goal="
                  << (goal_index + 1)
                  << " ompl_safe_distance=" << ompl_safe_distance
                  << " preference_safe_distance=" << input.safe_distance
                  << " raw_states=" << raw_state_count
                  << " interpolated_states=" << interpolated_state_count
                  << " shortcut_trials=" << common_cfg_.joint_space_shortcut_trials
                  << " output_states=" << joint_path.size()
                  << " exact=" << (exact_solution ? "true" : "false")
                  << " approximate_tail_distance=" << approximate_goal_distance
                  << " approximate_base_pos_error=" << approximate_goal_error.base_position
                  << " approximate_base_yaw_error=" << approximate_goal_error.base_yaw
                  << " approximate_arm_joint_error=" << approximate_goal_error.arm_joint
                  << " approximate_left_arm_error=" << approximate_goal_error.left_arm_joint
                  << " approximate_right_arm_error=" << approximate_goal_error.right_arm_joint
                  << " direct_valid=" << (direct_segment_valid ? "true" : "false")
                  << " direct_margin=" << direct_diag.min_margin
                  << " direct_reason=" << direct_diag.reason
                  << " direct_worst_link=" << direct_diag.worst_link_name
                  << std::endl;

        JointPathCandidate candidate = evaluateJointPathCandidate(joint_path, input);
        candidate.source_stage = static_cast<int>(goal_index) + 1;
        if (std::isfinite(candidate.score)) {
            candidates.push_back(std::move(candidate));
        } else {
            ++invalid_solution_paths;
        }
    }

    if (candidates.empty()) {
        std::cout << "[global_planner] rrt_connect failed: goals=" << valid_goals.size()
                  << " solved_paths=" << solved_paths
                  << " accepted_approximate_paths=" << accepted_approximate_paths
                  << " invalid_solution_paths=" << invalid_solution_paths
                  << " feasibility_safe_distance=" << feasibility_safe_distance
                  << " preference_safe_distance=" << input.safe_distance
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
    const JointPathCandidate& best_candidate = candidates.front();
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

cp::PathPlanningOutput OmplRrtConnectGlobalPlanner::planTwoWayBypassPath(
    const cp::PathPlanningInput& input) const {
    cp::PathPlanningOutput out;
    if (!input.q_start_seed.has_value() || input.q_goal_candidates.empty() ||
        !input.joint_segment_validator || !input.joint_to_pose_fn ||
        !input.whole_body_pose_validator) {
        return out;
    }

    Eigen::VectorXd q_start = *input.q_start_seed;
    if (!normalizeJointVectorToBounds(q_start, *input.q_start_seed, input.q_min, input.q_max)) {
        return out;
    }
    const double feasible_safe_distance = feasibilitySafeDistance(input);
    const double diagnostic_safe_distance =
        std::max(feasible_safe_distance, input.safe_distance);
    Eigen::VectorXd q_goal_for_diag = input.q_goal_candidates.front();
    if (!normalizeJointVectorToBounds(
            q_goal_for_diag,
            q_start,
            input.q_min,
            input.q_max)) {
        return out;
    }
    const double start_preference_margin =
        stateMargin(q_start, input, input.safe_distance);
    const double goal_preference_margin =
        stateMargin(q_goal_for_diag, input, input.safe_distance);
    const double endpoint_margin_floor =
        std::min(start_preference_margin, goal_preference_margin) - 0.003;
    cp::PathPlanningInput::WholeBodyPoseDiagnostic direct_diag;
    if (validateJointSegment(
            q_start,
            q_goal_for_diag,
            input,
            diagnostic_safe_distance,
            &direct_diag)) {
        return out;
    }

    const double failed_t = std::clamp(direct_diag.failed_segment_t, 0.15, 0.85);
    const Eigen::Vector3d path_midpoint =
        (1.0 - failed_t) * input.p_start + failed_t * input.p_goal;
    const Eigen::Vector3d via_center =
        direct_diag.has_failed_pose && direct_diag.failed_pose_world.allFinite()
            ? direct_diag.failed_pose_world
            : path_midpoint;

    const Eigen::Vector3d path_dir = normalizedOrZero(input.p_goal - input.p_start);
    Eigen::Vector3d hint_axis = normalizedOrZero(input.bypass_axis_hint);
    if (hint_axis.norm() < 1e-9) {
        hint_axis = Eigen::Vector3d::UnitZ();
    }

    Eigen::Vector3d obstacle_away = Eigen::Vector3d::Zero();
    if (direct_diag.worst_gradient_norm > 1e-6 &&
        direct_diag.worst_gradient_world.allFinite()) {
        obstacle_away =
            direct_diag.worst_gradient_world / direct_diag.worst_gradient_norm;
    }
    if (obstacle_away.norm() < 1e-9 && direct_diag.worst_point_world.allFinite()) {
        obstacle_away = normalizedOrZero(direct_diag.worst_point_world - via_center);
    }

    std::vector<Eigen::Vector3d> base_directions;
    appendDirection(base_directions, obstacle_away);
    appendDirection(base_directions, -obstacle_away);
    appendDirection(base_directions, path_dir);
    appendDirection(base_directions, -path_dir);
    appendDirection(base_directions, path_dir.cross(obstacle_away));
    appendDirection(base_directions, -path_dir.cross(obstacle_away));
    appendDirection(base_directions, path_dir.cross(hint_axis));
    appendDirection(base_directions, -path_dir.cross(hint_axis));
    appendDirection(base_directions, hint_axis);
    appendDirection(base_directions, -hint_axis);
    if (base_directions.empty()) {
        appendDirection(base_directions, Eigen::Vector3d::UnitZ());
        appendDirection(base_directions, -Eigen::Vector3d::UnitZ());
    }

    std::vector<Eigen::Matrix3d> entry_via_orientations;
    entry_via_orientations.reserve(4u);
    entry_via_orientations.push_back(slerpRotation(input.R_start, input.R_goal, failed_t));
    entry_via_orientations.push_back(input.R_start);
    entry_via_orientations.push_back(input.R_goal);
    if (direct_diag.has_failed_pose && direct_diag.failed_pose_orientation.allFinite()) {
        entry_via_orientations.push_back(direct_diag.failed_pose_orientation);
    }

    const double exit_t = std::clamp(failed_t + 0.25, 0.55, 0.92);
    const Eigen::Vector3d exit_center =
        (1.0 - exit_t) * input.p_start + exit_t * input.p_goal;
    std::vector<Eigen::Matrix3d> exit_via_orientations;
    exit_via_orientations.reserve(3u);
    exit_via_orientations.push_back(input.R_goal);
    exit_via_orientations.push_back(slerpRotation(input.R_start, input.R_goal, exit_t));
    exit_via_orientations.push_back(slerpRotation(input.R_start, input.R_goal, failed_t));

    const double base_offset = std::max(0.01, common_cfg_.two_way_bypass_offset_m);
    const std::array<double, 1> offset_scales{{1.0}};
    const std::size_t max_pose_attempts = 96u;
    std::size_t generated = 0u;
    std::size_t pose_valid = 0u;
    std::size_t entry_generated = 0u;
    std::size_t entry_pose_valid = 0u;
    std::size_t exit_generated = 0u;
    std::size_t exit_pose_valid = 0u;
    std::size_t first_leg_ok = 0u;
    std::size_t second_leg_ok = 0u;
    std::size_t third_leg_ok = 0u;
    std::size_t two_via_attempts = 0u;
    std::vector<JointPathCandidate> bypass_candidates;
    bypass_candidates.reserve(8u);
    struct ViaCandidate {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d p{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
        Eigen::VectorXd q;
        int source_stage{0};
        std::size_t direction_index{0u};
        std::size_t orientation_index{0u};
        double priority{0.0};
    };

    int candidate_id = 0;
    const auto generateViaCandidates =
        [&](const Eigen::Vector3d& center,
            const std::vector<Eigen::Matrix3d>& orientations,
            const Eigen::VectorXd& ik_seed,
            const Eigen::VectorXd& normalize_reference,
            const int source_base,
            std::size_t& local_generated,
            std::size_t& local_pose_valid) {
            std::vector<ViaCandidate> candidates;
            candidates.reserve(8u);
            for (const double scale : offset_scales) {
                const double offset = base_offset * scale;
                for (std::size_t direction_index = 0;
                     direction_index < base_directions.size();
                     ++direction_index) {
                    const auto& direction = base_directions[direction_index];
                    const Eigen::Vector3d p_via = center + offset * direction;
                    if (!p_via.allFinite()) {
                        continue;
                    }
                    for (std::size_t orientation_index = 0;
                         orientation_index < orientations.size();
                         ++orientation_index) {
                        const auto& R_via = orientations[orientation_index];
                        if (local_generated >= max_pose_attempts) {
                            break;
                        }
                        ++generated;
                        ++local_generated;
                        ++candidate_id;
                        Eigen::VectorXd q_via;
                        if (!input.whole_body_pose_validator(
                                p_via,
                                R_via,
                                feasible_safe_distance,
                                ik_seed,
                                q_via)) {
                            continue;
                        }
                        if (!normalizeJointVectorToBounds(
                                q_via,
                                normalize_reference,
                                input.q_min,
                                input.q_max)) {
                            continue;
                        }
                        cp::PathPlanningInput::WholeBodyPoseDiagnostic normalized_via_diag;
                        if (!validateJointState(
                                q_via,
                                input,
                                feasible_safe_distance,
                                &normalized_via_diag)) {
                            continue;
                        }
                        ++pose_valid;
                        ++local_pose_valid;
                        ViaCandidate via_candidate;
                        via_candidate.p = p_via;
                        via_candidate.R = R_via;
                        via_candidate.q = q_via;
                        via_candidate.source_stage = source_base + candidate_id;
                        via_candidate.direction_index = direction_index;
                        via_candidate.orientation_index = orientation_index;
                        via_candidate.priority =
                            static_cast<double>(orientation_index) +
                            0.10 * static_cast<double>(direction_index);
                        candidates.push_back(std::move(via_candidate));
                    }
                    if (local_generated >= max_pose_attempts) {
                        break;
                    }
                }
                if (local_generated >= max_pose_attempts) {
                    break;
                }
            }
            std::sort(
                candidates.begin(),
                candidates.end(),
                [](const ViaCandidate& a, const ViaCandidate& b) {
                    return a.priority < b.priority;
                });
            return candidates;
        };
    const std::size_t max_rrt_via_attempts = 8u;
    const auto selectViaCandidates =
        [&](const std::vector<ViaCandidate>& candidates) {
            std::vector<ViaCandidate> selected;
            selected.reserve(max_rrt_via_attempts);
            std::vector<bool> direction_selected(base_directions.size(), false);
            for (const auto& candidate : candidates) {
                if (selected.size() >= max_rrt_via_attempts) {
                    break;
                }
                if (candidate.direction_index >= direction_selected.size() ||
                    direction_selected[candidate.direction_index]) {
                    continue;
                }
                direction_selected[candidate.direction_index] = true;
                selected.push_back(candidate);
            }
            for (const auto& candidate : candidates) {
                if (selected.size() >= max_rrt_via_attempts) {
                    break;
                }
                const bool already_selected =
                    std::any_of(
                        selected.begin(),
                        selected.end(),
                        [&candidate](const ViaCandidate& existing) {
                            return existing.source_stage == candidate.source_stage;
                        });
                if (!already_selected) {
                    selected.push_back(candidate);
                }
            }
            return selected;
        };
    const auto entry_via_candidates = generateViaCandidates(
        via_center,
        entry_via_orientations,
        q_start,
        q_start,
        0,
        entry_generated,
        entry_pose_valid);
    const auto exit_via_candidates = generateViaCandidates(
        exit_center,
        exit_via_orientations,
        q_goal_for_diag,
        q_goal_for_diag,
        10000,
        exit_generated,
        exit_pose_valid);
    const std::vector<ViaCandidate> selected_entry_via_candidates =
        selectViaCandidates(entry_via_candidates);
    const std::vector<ViaCandidate> selected_exit_via_candidates =
        selectViaCandidates(exit_via_candidates);

    const auto planLeg = [this, &input](
                             const Eigen::Vector3d& p_start,
                             const Eigen::Matrix3d& R_start,
                             const Eigen::VectorXd& q_start_leg,
                             const Eigen::Vector3d& p_goal,
                             const Eigen::Matrix3d& R_goal,
                             const Eigen::VectorXd& q_goal_leg) {
        cp::PathPlanningInput leg = input;
        leg.p_start = p_start;
        leg.R_start = R_start;
        leg.q_start_seed = q_start_leg;
        leg.p_goal = p_goal;
        leg.R_goal = R_goal;
        leg.q_goal_candidates.clear();
        leg.q_goal_candidates.push_back(q_goal_leg);
        return planDirectPath(leg);
    };

    for (const ViaCandidate& via_candidate : selected_entry_via_candidates) {
        cp::PathPlanningOutput first_output = planLeg(
            input.p_start,
            input.R_start,
            q_start,
            via_candidate.p,
            via_candidate.R,
            via_candidate.q);
        if (!first_output.success || first_output.joint_waypoints.size() < 2u) {
            continue;
        }
        ++first_leg_ok;

        cp::PathPlanningOutput second_output = planLeg(
            via_candidate.p,
            via_candidate.R,
            via_candidate.q,
            input.p_goal,
            input.R_goal,
            q_goal_for_diag);
        if (!second_output.success || second_output.joint_waypoints.size() < 2u) {
            const std::size_t max_two_via_pairs = 48u;
            for (const ViaCandidate& via_candidate2 : selected_exit_via_candidates) {
                if (two_via_attempts >= max_two_via_pairs) {
                    break;
                }
                if ((via_candidate2.p - via_candidate.p).norm() < 0.02) {
                    continue;
                }
                ++two_via_attempts;
                cp::PathPlanningOutput middle_output = planLeg(
                    via_candidate.p,
                    via_candidate.R,
                    via_candidate.q,
                    via_candidate2.p,
                    via_candidate2.R,
                    via_candidate2.q);
                if (!middle_output.success || middle_output.joint_waypoints.size() < 2u) {
                    continue;
                }
                ++second_leg_ok;
                cp::PathPlanningOutput final_output = planLeg(
                    via_candidate2.p,
                    via_candidate2.R,
                    via_candidate2.q,
                    input.p_goal,
                    input.R_goal,
                    q_goal_for_diag);
                if (!final_output.success || final_output.joint_waypoints.size() < 2u) {
                    continue;
                }
                ++third_leg_ok;

                std::vector<Eigen::VectorXd> joint_path = first_output.joint_waypoints;
                joint_path.insert(
                    joint_path.end(),
                    middle_output.joint_waypoints.begin() + 1,
                    middle_output.joint_waypoints.end());
                joint_path.insert(
                    joint_path.end(),
                    final_output.joint_waypoints.begin() + 1,
                    final_output.joint_waypoints.end());
                JointPathCandidate candidate = evaluateJointPathCandidate(joint_path, input);
                candidate.source_stage =
                    1000 * via_candidate.source_stage + via_candidate2.source_stage;
                candidate.via_count = 2;
                candidate.via_point = via_candidate.p;
                candidate.via_point2 = via_candidate2.p;
                if (std::isfinite(candidate.score) &&
                    candidate.min_margin >= endpoint_margin_floor) {
                    bypass_candidates.push_back(std::move(candidate));
                }
            }
            continue;
        }
        ++second_leg_ok;

        std::vector<Eigen::VectorXd> joint_path = first_output.joint_waypoints;
        joint_path.insert(
            joint_path.end(),
            second_output.joint_waypoints.begin() + 1,
            second_output.joint_waypoints.end());
        JointPathCandidate candidate = evaluateJointPathCandidate(joint_path, input);
        candidate.source_stage = via_candidate.source_stage;
        candidate.via_count = 1;
        candidate.via_point = via_candidate.p;
        if (std::isfinite(candidate.score) &&
            candidate.min_margin >= endpoint_margin_floor) {
            bypass_candidates.push_back(std::move(candidate));
        }
    }

    if (bypass_candidates.empty()) {
        std::cout << "[global_planner] adaptive_bypass failed: generated="
                  << generated
                  << " pose_valid=" << pose_valid
                  << " entry_generated=" << entry_generated
                  << " entry_pose_valid=" << entry_pose_valid
                  << " exit_generated=" << exit_generated
                  << " exit_pose_valid=" << exit_pose_valid
                  << " entry_via_attempts=" << selected_entry_via_candidates.size()
                  << " exit_via_attempts=" << selected_exit_via_candidates.size()
                  << " two_via_attempts=" << two_via_attempts
                  << " first_leg_ok=" << first_leg_ok
                  << " second_leg_ok=" << second_leg_ok
                  << " third_leg_ok=" << third_leg_ok
                  << " center=(" << via_center.x() << ", " << via_center.y()
                  << ", " << via_center.z() << ")"
                  << " exit_center=(" << exit_center.x() << ", " << exit_center.y()
                  << ", " << exit_center.z() << ")"
                  << " obstacle_away=(" << obstacle_away.x() << ", "
                  << obstacle_away.y() << ", " << obstacle_away.z() << ")"
                  << " base_offset=" << base_offset
                  << " direct_reason=" << direct_diag.reason
                  << " direct_worst_link=" << direct_diag.worst_link_name
                  << std::endl;
        return out;
    }

    std::sort(
        bypass_candidates.begin(),
        bypass_candidates.end(),
        [](const JointPathCandidate& a, const JointPathCandidate& b) {
            constexpr double kMarginTie = 0.003;
            if (std::abs(a.min_margin - b.min_margin) > kMarginTie) {
                return a.min_margin > b.min_margin;
            }
            return a.score < b.score;
        });
    const JointPathCandidate& best = bypass_candidates.front();
    out.joint_waypoints = best.joint_path;
    out.segment_kind = cp::PlannedSegmentKind::Goal;
    out.success = out.joint_waypoints.size() >= 2u;
    std::cout << "[global_planner] adaptive_bypass selected: candidate="
              << best.source_stage
              << " candidates=" << bypass_candidates.size()
              << " generated=" << generated
              << " pose_valid=" << pose_valid
              << " entry_generated=" << entry_generated
              << " entry_pose_valid=" << entry_pose_valid
              << " exit_generated=" << exit_generated
              << " exit_pose_valid=" << exit_pose_valid
              << " entry_via_attempts=" << selected_entry_via_candidates.size()
              << " exit_via_attempts=" << selected_exit_via_candidates.size()
              << " two_via_attempts=" << two_via_attempts
              << " first_leg_ok=" << first_leg_ok
              << " second_leg_ok=" << second_leg_ok
              << " third_leg_ok=" << third_leg_ok
              << " via_count=" << best.via_count
              << " via=(" << best.via_point.x() << ", " << best.via_point.y()
              << ", " << best.via_point.z() << ")"
              << " via2=(" << best.via_point2.x() << ", " << best.via_point2.y()
              << ", " << best.via_point2.z() << ")"
              << " score=" << best.score
              << " min_margin=" << best.min_margin
              << " worst_link=" << best.worst_link_name
              << std::endl;
    return out;
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
        const double feasible_safe_distance = feasibilitySafeDistance(input);
        if (!validateJointState(q_candidate, input, feasible_safe_distance, &diag) ||
            !validateJointSegment(q_prev, q_candidate, input, feasible_safe_distance) ||
            !validateJointSegment(q_candidate, q_next, input, feasible_safe_distance)) {
            return std::numeric_limits<double>::infinity();
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic preference_diag;
        (void)validateJointState(q_candidate, input, input.safe_distance, &preference_diag);
        if (hasMeaningfulMargin(preference_diag)) {
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
        if (hasMeaningfulMargin(preference_diag)) {
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
            cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
            if (!validateJointSegment(
                    joint_path[i - 1],
                    joint_path[i],
                    input,
                    feasibilitySafeDistance(input),
                    &segment_diag)) {
                return candidate;
            }
            cp::PathPlanningInput::WholeBodyPoseDiagnostic preference_segment_diag;
            (void)validateJointSegment(
                joint_path[i - 1],
                joint_path[i],
                input,
                input.safe_distance,
                &preference_segment_diag);
            if (hasMeaningfulMargin(preference_segment_diag) &&
                preference_segment_diag.min_margin < candidate.min_margin) {
                candidate.min_margin = preference_segment_diag.min_margin;
                candidate.worst_link_name = preference_segment_diag.worst_link_name;
            }
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
    const int shortcut_trials,
    const double safe_distance) const {
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
        if (!validateJointSegment(joint_path[i], joint_path[j], input, safe_distance)) {
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
