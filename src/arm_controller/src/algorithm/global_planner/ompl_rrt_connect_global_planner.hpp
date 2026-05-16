#pragma once

#include "algorithm/cartesian_path_planner/config/planner_common_config.hpp"
#include "algorithm/cartesian_path_planner/config/smoothing_config.hpp"
#include "algorithm/global_planner/global_planner_interface.hpp"

#include <limits>
#include <string>
#include <vector>

namespace arm_controller::algorithm::global_planner {

namespace cp = arm_controller::algorithm::cartesian_path_planner;

class OmplRrtConnectGlobalPlanner final : public GlobalPlannerInterface {
public:
    OmplRrtConnectGlobalPlanner(
        const cp::PlannerCommonConfig& common_cfg,
        const cp::SmoothingConfig& smoothing_cfg);

    cp::TimedJointTrajectory planTrajectory(
        const cp::PathPlanningInput& input) override;

private:
    struct JointPathCandidate {
        std::vector<Eigen::VectorXd> joint_path;
        double cartesian_length{0.0};
        double joint_motion{0.0};
        double mean_clearance_deficit{0.0};
        double mean_early_clearance_deficit{0.0};
        double min_margin{std::numeric_limits<double>::infinity()};
        double early_min_margin{std::numeric_limits<double>::infinity()};
        double mean_clearance_reward{0.0};
        std::string worst_link_name;
        std::string early_worst_link_name;
        int source_stage{0};
        double score{std::numeric_limits<double>::infinity()};
    };

    cp::PathPlanningOutput planPath(const cp::PathPlanningInput& input) const;
    cp::TimedJointTrajectory buildJointSpaceTrajectory(
        const std::vector<Eigen::VectorXd>& joint_path) const;
    std::vector<Eigen::VectorXd> optimizeJointTrajectory(
        const std::vector<Eigen::VectorXd>& joint_path,
        const cp::PathPlanningInput& input) const;
    JointPathCandidate evaluateJointPathCandidate(
        const std::vector<Eigen::VectorXd>& joint_path,
        const cp::PathPlanningInput& input) const;
    void shortcutJointPath(
        std::vector<Eigen::VectorXd>& joint_path,
        const cp::PathPlanningInput& input,
        int shortcut_trials) const;
    static double jointDistance(
        const Eigen::VectorXd& a,
        const Eigen::VectorXd& b);

    cp::PlannerCommonConfig common_cfg_;
};

}  // namespace arm_controller::algorithm::global_planner
