#pragma once

#include "algorithm/cartesian_path_planner/config/planner_common_config.hpp"
#include "algorithm/global_planner/global_planner_interface.hpp"

#include <Eigen/Core>

#include <map>
#include <string>
#include <vector>

namespace arm_controller::algorithm::global_planner {

namespace cp = arm_controller::algorithm::cartesian_path_planner;

class BaseGuidedWholeBodyPlanner final : public GlobalPlannerInterface {
public:
    explicit BaseGuidedWholeBodyPlanner(const cp::PlannerCommonConfig& common_cfg);

    cp::TimedJointTrajectory planTrajectory(
        const cp::PathPlanningInput& input) override;

    struct BaseState {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

private:
    struct LayerRejectStats {
        std::size_t state_attempts{0};
        std::size_t state_rejects{0};
        std::size_t state_accepts{0};
        std::size_t duplicate_rejects{0};
        std::size_t velocity_rejects{0};
        std::size_t segment_attempts{0};
        std::size_t segment_rejects{0};
        std::map<std::string, std::size_t> state_reasons;
        std::map<std::string, std::size_t> segment_reasons;
        cp::PathPlanningInput::WholeBodyPoseDiagnostic representative_state_diag;
        cp::PathPlanningInput::WholeBodyPoseDiagnostic representative_segment_diag;

        static std::string keyFromDiag(
            const cp::PathPlanningInput::WholeBodyPoseDiagnostic& diag) {
            std::string key = diag.reason.empty() ? "unknown" : diag.reason;
            if (!diag.worst_link_name.empty()) {
                key += ":";
                key += diag.worst_link_name;
            }
            return key;
        }

        void addStateReject(
            const cp::PathPlanningInput::WholeBodyPoseDiagnostic& diag) {
            ++state_rejects;
            ++state_reasons[keyFromDiag(diag)];
            if (representative_state_diag.reason.empty() ||
                diag.min_margin < representative_state_diag.min_margin) {
                representative_state_diag = diag;
            }
        }

        void addSegmentReject(
            const cp::PathPlanningInput::WholeBodyPoseDiagnostic& diag) {
            ++segment_rejects;
            ++segment_reasons[keyFromDiag(diag)];
            if (representative_segment_diag.reason.empty() ||
                diag.min_margin < representative_segment_diag.min_margin) {
                representative_segment_diag = diag;
            }
        }
    };
    struct BaseRoute {
        std::vector<BaseState> states;
        std::vector<BaseState> dense_check_states;
        std::vector<std::vector<BaseState>> segment_check_states;
        std::vector<double> segment_times;
        std::vector<int> singularities;

        bool empty() const { return states.empty(); }
        std::size_t size() const { return states.size(); }
        const BaseState& operator[](std::size_t index) const {
            return states[index];
        }
    };
    using JointPath = std::vector<Eigen::VectorXd>;
    using Layer = std::vector<Eigen::VectorXd>;

    std::vector<BaseRoute> makeBaseRoutes(
        const BaseState& start,
        const BaseState& goal,
        const Eigen::VectorXd& q_start,
        const cp::PathPlanningInput& input,
        double safe_distance) const;
    JointPath planLayeredArmGraph(
        const BaseRoute& route,
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal,
        const cp::PathPlanningInput& input,
        double safe_distance) const;
    JointPath planBidirectionalLayerRrt(
        const BaseRoute& route,
        const std::vector<Layer>& layers,
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal,
        const cp::PathPlanningInput& input,
        double safe_distance) const;
    JointPath repairPathWithLayerRrt(
        const BaseRoute& route,
        const JointPath& seed_path,
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal,
        const cp::PathPlanningInput& input,
        double safe_distance) const;
    Layer buildArmCandidatesForLayer(
        const BaseState& base,
        std::size_t layer_index,
        double t,
        bool is_start,
        bool is_goal,
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal,
        const cp::PathPlanningInput& input,
        double safe_distance,
        LayerRejectStats* reject_stats = nullptr) const;
    Layer buildArmCandidatesFromSeedsForLayer(
        const BaseState& base,
        std::size_t layer_index,
        double t,
        bool is_goal,
        const Layer& seed_layer,
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal,
        const cp::PathPlanningInput& input,
        double safe_distance,
        double segment_dt,
        LayerRejectStats* reject_stats = nullptr) const;
    bool validatePath(
        const JointPath& path,
        const cp::PathPlanningInput& input,
        double safe_distance) const;
    bool validateLayerPath(
        const JointPath& path,
        const cp::PathPlanningInput& input,
        double safe_distance) const;
    JointPath smoothArmPathConservatively(
        const BaseRoute& route,
        JointPath path,
        const cp::PathPlanningInput& input,
        double safe_distance) const;
    cp::TimedJointTrajectory buildTrajectory(
        const JointPath& path,
        const std::vector<double>& segment_times = {}) const;

    static BaseState baseFromQ(const Eigen::VectorXd& q);
    static double normalizeAngle(double angle);
    static double angularDistance(double from, double to);
    static Eigen::VectorXd interpolateState(
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal,
        const BaseState& base,
        double arm_alpha);
    static std::vector<double> estimateRouteSegmentTimes(
        const BaseRoute& route,
        double default_segment_speed);
    static std::vector<double> estimateArmLayerSegmentTimes(
        const BaseRoute& route,
        double default_segment_speed);
    static std::vector<double> estimateWholeBodyPathSegmentTimes(
        const JointPath& path,
        double default_segment_speed);

    cp::PlannerCommonConfig common_cfg_;
};

}  // namespace arm_controller::algorithm::global_planner
