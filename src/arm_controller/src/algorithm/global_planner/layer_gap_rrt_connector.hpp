#pragma once

#include "algorithm/cartesian_path_planner/types.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace arm_controller::algorithm::global_planner {

namespace cp = arm_controller::algorithm::cartesian_path_planner;

class LayerGapRrtConnector {
public:
    struct BaseState {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    struct Config {
        int max_iterations{1200};
        double goal_bias{0.35};
        double max_joint_velocity_rad_per_sec{0.75};
        double time_scale{3.0};
        double meet_tolerance_rad{1e-4};
        bool require_complete_endpoint_path{false};
    };

    struct Input {
        struct SeedState {
            Eigen::VectorXd arm;
            int layer{0};
        };

        const cp::PathPlanningInput* planning_input{nullptr};
        std::vector<BaseState> route;
        std::vector<std::vector<BaseState>> segment_check_states;
        std::vector<double> segment_times;
        Eigen::VectorXd arm_start;
        Eigen::VectorXd arm_goal;
        std::vector<SeedState> start_seeds;
        std::vector<SeedState> goal_seeds;
        double safe_distance{0.05};
    };

    struct Result {
        bool success{false};
        // Arm-only states aligned with path_layers. The caller owns the fixed
        // base route and composes full whole-body states only at the boundary.
        std::vector<Eigen::VectorXd> path;
        std::vector<int> path_layers;
        int start_layer{-1};
        int end_layer{-1};
        int iterations{0};
        std::size_t node_count{0};
        std::size_t extend_rejects{0};
        std::string top_extend_reject_reason;
    };

    LayerGapRrtConnector();
    explicit LayerGapRrtConnector(Config config);

    Result connect(const Input& input) const;

private:
    Config config_;
};

}  // namespace arm_controller::algorithm::global_planner
