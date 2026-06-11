#include <gtest/gtest.h>

#include "algorithm/global_planner/layer_gap_rrt_connector.hpp"

namespace gp = arm_controller::algorithm::global_planner;
namespace cp = arm_controller::algorithm::cartesian_path_planner;

namespace {
constexpr int kDof = 15;

cp::PathPlanningInput makeInput() {
    cp::PathPlanningInput input;
    input.safe_distance = 0.03;
    input.q_min = Eigen::VectorXd::Constant(kDof, -1.0);
    input.q_max = Eigen::VectorXd::Constant(kDof, 1.0);
    input.q_min.head<3>() = Eigen::Vector3d(-5.0, -5.0, -3.14);
    input.q_max.head<3>() = Eigen::Vector3d(5.0, 5.0, 3.14);
    input.joint_state_validator =
        [](const Eigen::VectorXd& q,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool ok = q.size() == kDof && q.allFinite();
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.1 : -0.1;
                diag->reason = ok ? "ok" : "invalid";
            }
            return ok;
        };
    input.joint_segment_validator =
        [](const Eigen::VectorXd& from,
           const Eigen::VectorXd& to,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool finite = from.size() == kDof && to.size() == kDof &&
                                from.allFinite() && to.allFinite();
            const double dq = (to.segment(3, 12) - from.segment(3, 12))
                                  .cwiseAbs()
                                  .maxCoeff();
            const bool ok = finite && dq <= 0.26;
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.1 : -0.01;
                diag->reason = ok ? "ok" : "large_joint_jump_blocked";
            }
            return ok;
        };
    return input;
}

}  // namespace

TEST(LayerGapRrtConnectorTest, FillsGapWithIntermediateWholeBodyStates) {
    cp::PathPlanningInput input = makeInput();
    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd q_goal = Eigen::VectorXd::Zero(kDof);
    q_goal[0] = 0.80;
    q_goal[3] = 0.60;

    gp::LayerGapRrtConnector::Input connector_input;
    connector_input.planning_input = &input;
    connector_input.safe_distance = input.safe_distance;
    connector_input.arm_start = q_start.segment(3, 12);
    connector_input.arm_goal = q_goal.segment(3, 12);
    connector_input.route = {
        {0.0, 0.0, 0.0},
        {0.2, 0.0, 0.0},
        {0.4, 0.0, 0.0},
        {0.6, 0.0, 0.0},
        {0.8, 0.0, 0.0},
    };
    connector_input.segment_times = {1.0, 1.0, 1.0, 1.0};

    gp::LayerGapRrtConnector::Config cfg;
    cfg.max_iterations = 800;
    cfg.goal_bias = 0.65;
    cfg.max_joint_velocity_rad_per_sec = 0.25;
    cfg.time_scale = 1.0;
    cfg.meet_tolerance_rad = 1e-4;
    gp::LayerGapRrtConnector connector(cfg);

    const gp::LayerGapRrtConnector::Result result =
        connector.connect(connector_input);

    ASSERT_TRUE(result.success);
    ASSERT_GE(result.path.size(), 4u);
    ASSERT_EQ(result.path.size(), result.path_layers.size());
    EXPECT_TRUE(result.path.front().isApprox(connector_input.arm_start));
    EXPECT_TRUE(result.path.back().isApprox(connector_input.arm_goal));
    for (std::size_t i = 1; i < result.path.size(); ++i) {
        Eigen::VectorXd from = Eigen::VectorXd::Zero(kDof);
        Eigen::VectorXd to = Eigen::VectorXd::Zero(kDof);
        const auto& from_base =
            connector_input.route[static_cast<std::size_t>(result.path_layers[i - 1u])];
        const auto& to_base =
            connector_input.route[static_cast<std::size_t>(result.path_layers[i])];
        from[0] = from_base.x;
        from[1] = from_base.y;
        from[2] = from_base.yaw;
        from.segment(3, 12) = result.path[i - 1u];
        to[0] = to_base.x;
        to[1] = to_base.y;
        to[2] = to_base.yaw;
        to.segment(3, 12) = result.path[i];
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        EXPECT_TRUE(input.joint_segment_validator(
            from, to, input.safe_distance, &diag));
    }
}

TEST(LayerGapRrtConnectorTest, AcceptsMultipleSeedNodesAndShortcutsResult) {
    cp::PathPlanningInput input = makeInput();
    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd q_goal = Eigen::VectorXd::Zero(kDof);
    q_goal[0] = 0.80;
    q_goal[3] = 0.10;

    Eigen::VectorXd forward_seed = Eigen::VectorXd::Zero(kDof);
    forward_seed[0] = 0.20;
    forward_seed[3] = 0.025;
    Eigen::VectorXd backward_seed = q_goal;
    backward_seed[0] = 0.60;
    backward_seed[3] = 0.075;

    gp::LayerGapRrtConnector::Input connector_input;
    connector_input.planning_input = &input;
    connector_input.safe_distance = input.safe_distance;
    connector_input.arm_start = q_start.segment(3, 12);
    connector_input.arm_goal = q_goal.segment(3, 12);
    connector_input.route = {
        {0.0, 0.0, 0.0},
        {0.2, 0.0, 0.0},
        {0.4, 0.0, 0.0},
        {0.6, 0.0, 0.0},
        {0.8, 0.0, 0.0},
    };
    connector_input.segment_times = {1.0, 1.0, 1.0, 1.0};
    connector_input.start_seeds.push_back({q_start.segment(3, 12), 0});
    connector_input.start_seeds.push_back({forward_seed.segment(3, 12), 1});
    connector_input.goal_seeds.push_back({q_goal.segment(3, 12), 4});
    connector_input.goal_seeds.push_back({backward_seed.segment(3, 12), 3});

    gp::LayerGapRrtConnector::Config cfg;
    cfg.max_iterations = 300;
    cfg.goal_bias = 1.0;
    cfg.max_joint_velocity_rad_per_sec = 0.30;
    cfg.time_scale = 1.0;
    gp::LayerGapRrtConnector connector(cfg);

    const gp::LayerGapRrtConnector::Result result =
        connector.connect(connector_input);

    ASSERT_TRUE(result.success);
    ASSERT_GE(result.path.size(), 2u);
    EXPECT_TRUE(result.path.front().isApprox(q_start.segment(3, 12)) ||
                result.path.front().isApprox(forward_seed.segment(3, 12)));
    EXPECT_TRUE(result.path.back().isApprox(q_goal.segment(3, 12)) ||
                result.path.back().isApprox(backward_seed.segment(3, 12)));
    EXPECT_LE(result.path.size(), 5u);
}
