#pragma once

#include "algorithm/cartesian_path_planner/config/planner_common_config.hpp"
#include "algorithm/cartesian_path_planner/config/smoothing_config.hpp"
#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "algorithm/cartesian_path_planner/types.hpp"

#include <memory>
#include <random>

namespace arm_controller::algorithm::cartesian_path_planner {

class CartesianPathPlanner {
public:
    CartesianPathPlanner(
        const PlannerCommonConfig& common_cfg,
        const SmoothingConfig& smoothing_cfg,
        std::shared_ptr<const DistanceFieldInterface> distance_field);

    PathPlanningOutput planPath(const PathPlanningInput& input);
    TimedCartesianTrajectory planTrajectory(const PathPlanningInput& input);

private:
    struct ExtendResult {
        bool advanced{false};
        bool reached_target{false};
        int new_index{-1};
    };

    struct JointTreeNode {
        Eigen::VectorXd q;
        int parent{-1};
    };

    struct JointPathCandidate {
        std::vector<Eigen::VectorXd> joint_path;
        double cartesian_length{0.0};
        double joint_motion{0.0};
        double shell_deviation_cost{0.0};
        double score{std::numeric_limits<double>::infinity()};
    };

    PathPlanningOutput planJointSpacePath(const PathPlanningInput& input);
    TimedCartesianTrajectory buildJointSpaceTrajectory(
        const std::vector<Eigen::VectorXd>& joint_path,
        const PathPlanningInput& input) const;
    std::vector<Eigen::VectorXd> optimizeJointTrajectory(
        const std::vector<Eigen::VectorXd>& joint_path,
        const PathPlanningInput& input) const;
    Eigen::VectorXd sampleJointState(
        const Eigen::VectorXd& q_min,
        const Eigen::VectorXd& q_max,
        const std::vector<Eigen::VectorXd>& q_goal_candidates,
        double goal_bias,
        std::mt19937& rng) const;
    static int nearestJointNode(
        const std::vector<JointTreeNode>& tree,
        const Eigen::VectorXd& q);
    static Eigen::VectorXd steerJointState(
        const Eigen::VectorXd& q_from,
        const Eigen::VectorXd& q_to,
        double step_rad);
    static double jointDistance(
        const Eigen::VectorXd& a,
        const Eigen::VectorXd& b);
    static std::vector<Eigen::VectorXd> backtrackJointPath(
        const std::vector<JointTreeNode>& tree,
        int node_index);
    static std::vector<Eigen::VectorXd> backtrackJointPathReversed(
        const std::vector<JointTreeNode>& tree,
        int node_index);
    static void shortcutJointPath(
        std::vector<Eigen::VectorXd>& joint_path,
        const PathPlanningInput& input,
        int shortcut_trials);
    JointPathCandidate evaluateJointPathCandidate(
        const std::vector<Eigen::VectorXd>& joint_path,
        const PathPlanningInput& input) const;
    static ExtendResult extendTree(
        std::vector<JointTreeNode>& tree,
        int from_index,
        const Eigen::VectorXd& q_target,
        double step_rad,
        double connect_threshold,
        const PathPlanningInput& input);

    PlannerCommonConfig common_cfg_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
