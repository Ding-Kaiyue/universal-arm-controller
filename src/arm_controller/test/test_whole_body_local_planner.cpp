#include "controller/reactive_task/local_planner/reactive_task_whole_body_local_planner.hpp"

#include <gtest/gtest.h>

namespace rt = arm_controller::controller::reactive_task;
namespace cp = arm_controller::algorithm::cartesian_path_planner;

namespace {

constexpr int kFullDof = 15;

Eigen::VectorXd makeState(
    const double x,
    const double y,
    const double yaw,
    const double arm_value) {
  Eigen::VectorXd q = Eigen::VectorXd::Constant(kFullDof, arm_value);
  q[0] = x;
  q[1] = y;
  q[2] = yaw;
  return q;
}

cp::TimedJointTrajectory makeReference() {
  cp::TimedJointTrajectory trajectory;
  trajectory.joint_targets = {
      makeState(0.0, 0.0, 0.0, 0.0),
      makeState(0.2, 0.0, 0.0, 0.1),
      makeState(0.4, 0.1, 0.1, 0.2),
      makeState(0.6, 0.1, 0.1, 0.3),
  };
  trajectory.segment_durations = {0.1, 0.1, 0.1};
  trajectory.cumulative_times = {0.0, 0.1, 0.2, 0.3};
  trajectory.total_duration = 0.3;
  return trajectory;
}

}  // namespace

TEST(WholeBodyLocalPlannerTest, ProducesValidated15DLocalTrajectory) {
  rt::WholeBodyLocalPlanner::Config cfg;
  cfg.horizon_steps = 4;
  cfg.dt_sec = 0.1;
  rt::WholeBodyLocalPlanner planner(cfg);

  rt::WholeBodyLocalPlanner::Input input;
  input.q_current = makeState(0.0, 0.0, 0.0, 0.0);
  input.global_reference = makeReference();
  input.global_time_sec = 0.0;
  input.safe_distance = 0.05;
  input.q_min = Eigen::VectorXd::Constant(kFullDof, -2.0);
  input.q_max = Eigen::VectorXd::Constant(kFullDof, 2.0);

  int state_checks = 0;
  int segment_checks = 0;
  input.joint_state_validator =
      [&state_checks](
          const Eigen::VectorXd& q,
          double,
          cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
        ++state_checks;
        if (diag != nullptr) {
          diag->collision_free = q.size() == kFullDof && q.allFinite();
        }
        return q.size() == kFullDof && q.allFinite();
      };
  input.joint_segment_validator =
      [&segment_checks](
          const Eigen::VectorXd& from,
          const Eigen::VectorXd& to,
          double,
          cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
        ++segment_checks;
        const bool ok =
            from.size() == kFullDof && to.size() == kFullDof &&
            from.allFinite() && to.allFinite() &&
            (to.head<3>() - from.head<3>()).norm() < 0.5;
        if (diag != nullptr) {
          diag->collision_free = ok;
        }
        return ok;
      };

  rt::WholeBodyLocalPlanner::Output output;
  ASSERT_TRUE(planner.compute(input, &output)) << output.error;
  ASSERT_TRUE(output.ok);
  ASSERT_TRUE(output.used);
  EXPECT_TRUE(output.replanned_window);
  ASSERT_EQ(output.joint_targets.size(), 4u);
  ASSERT_EQ(output.next_joint_target.size(), kFullDof);
  ASSERT_EQ(output.next_joint_velocity.size(), kFullDof);
  EXPECT_NEAR(output.joint_targets.front()[0], input.q_current[0], 1e-9);
  EXPECT_GT(output.next_joint_target[0], input.q_current[0]);
  EXPECT_GT(state_checks, 0);
  EXPECT_GT(segment_checks, 0);
}

TEST(WholeBodyLocalPlannerTest, LimitsNextTargetByWholeBodyDynamics) {
  rt::WholeBodyLocalPlanner::Config cfg;
  cfg.horizon_steps = 4;
  cfg.dt_sec = 0.1;
  cfg.max_base_vx = 0.2;
  cfg.max_base_vy = 0.1;
  cfg.max_base_wz = 0.3;
  cfg.max_arm_qdot = 0.4;
  rt::WholeBodyLocalPlanner planner(cfg);

  cp::TimedJointTrajectory trajectory;
  trajectory.joint_targets = {
      makeState(0.0, 0.0, 0.0, 0.0),
      makeState(1.0, 1.0, 1.0, 1.0),
      makeState(2.0, 2.0, 1.0, 1.0),
      makeState(3.0, 3.0, 1.0, 1.0),
  };
  trajectory.segment_durations = {0.1, 0.1, 0.1};
  trajectory.cumulative_times = {0.0, 0.1, 0.2, 0.3};
  trajectory.total_duration = 0.3;

  rt::WholeBodyLocalPlanner::Input input;
  input.q_current = makeState(0.0, 0.0, 0.0, 0.0);
  input.global_reference = trajectory;
  input.global_time_sec = 0.0;
  input.safe_distance = 0.05;
  input.q_min = Eigen::VectorXd::Constant(kFullDof, -4.0);
  input.q_max = Eigen::VectorXd::Constant(kFullDof, 4.0);
  input.joint_state_validator =
      [](const Eigen::VectorXd& q,
         double,
         cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
        if (diag != nullptr) {
          diag->collision_free = q.size() == kFullDof && q.allFinite();
        }
        return q.size() == kFullDof && q.allFinite();
      };
  input.joint_segment_validator =
      [](const Eigen::VectorXd& from,
         const Eigen::VectorXd& to,
         double,
         cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
        const bool ok = from.size() == kFullDof && to.size() == kFullDof;
        if (diag != nullptr) {
          diag->collision_free = ok;
        }
        return ok;
      };

  rt::WholeBodyLocalPlanner::Output output;
  ASSERT_TRUE(planner.compute(input, &output)) << output.error;
  ASSERT_EQ(output.joint_targets.size(), 4u);
  const Eigen::VectorXd delta = output.joint_targets[1] - output.joint_targets[0];
  EXPECT_LE(std::abs(delta[0]), cfg.max_base_vx * cfg.dt_sec + 1e-9);
  EXPECT_LE(std::abs(delta[1]), cfg.max_base_vy * cfg.dt_sec + 1e-9);
  EXPECT_LE(std::abs(delta[2]), cfg.max_base_wz * cfg.dt_sec + 1e-9);
  for (int i = 3; i < kFullDof; ++i) {
    EXPECT_LE(std::abs(delta[i]), cfg.max_arm_qdot * cfg.dt_sec + 1e-9);
  }
}

TEST(WholeBodyLocalPlannerTest, FailsWhenRemaniStyleWindowCannotBeValidated) {
  rt::WholeBodyLocalPlanner::Config cfg;
  cfg.horizon_steps = 6;
  cfg.dt_sec = 0.1;
  cfg.max_base_vx = 10.0;
  cfg.max_base_vy = 10.0;
  cfg.max_base_wz = 10.0;
  cfg.max_arm_qdot = 10.0;
  rt::WholeBodyLocalPlanner planner(cfg);

  cp::TimedJointTrajectory trajectory;
  trajectory.joint_targets = {
      makeState(0.0, 0.0, 0.0, 0.0),
      makeState(0.1, 0.0, 0.0, 0.1),
      makeState(0.2, 0.0, 0.0, 0.2),
      makeState(0.3, 0.0, 0.0, 0.3),
      makeState(0.4, 0.0, 0.0, 0.4),
      makeState(0.5, 0.0, 0.0, 0.5),
  };
  trajectory.segment_durations = {0.1, 0.1, 0.1, 0.1, 0.1};
  trajectory.cumulative_times = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  trajectory.total_duration = 0.5;

  rt::WholeBodyLocalPlanner::Input input;
  input.q_current = makeState(0.0, 0.0, 0.0, 0.0);
  input.global_reference = trajectory;
  input.global_time_sec = 0.0;
  input.safe_distance = 0.05;
  input.q_min = Eigen::VectorXd::Constant(kFullDof, -4.0);
  input.q_max = Eigen::VectorXd::Constant(kFullDof, 4.0);
  input.joint_state_validator =
      [](const Eigen::VectorXd& q,
         double,
         cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
        const bool ok = q.size() == kFullDof && q.allFinite() && q[0] < 0.25;
        if (diag != nullptr) {
          diag->collision_free = ok;
          diag->reason = ok ? "ok" : "collision_fail";
        }
        return ok;
      };
  input.joint_segment_validator =
      [](const Eigen::VectorXd& from,
         const Eigen::VectorXd& to,
         double,
         cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
        const bool ok =
            from.size() == kFullDof && to.size() == kFullDof &&
            from.allFinite() && to.allFinite() && to[0] < 0.25;
        if (diag != nullptr) {
          diag->collision_free = ok;
          diag->reason = ok ? "ok" : "collision_fail";
        }
        return ok;
      };

  rt::WholeBodyLocalPlanner::Output output;
  EXPECT_FALSE(planner.compute(input, &output));
  EXPECT_FALSE(output.ok);
  EXPECT_TRUE(output.joint_targets.empty());
  EXPECT_FALSE(output.error.empty());
  EXPECT_TRUE(
      output.error.find("invalid") != std::string::npos ||
      output.error.find("collision_fail") != std::string::npos ||
      output.error.find("safe local window goal") != std::string::npos ||
      output.error.find("failed") != std::string::npos)
      << output.error;
}
