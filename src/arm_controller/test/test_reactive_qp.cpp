#include <gtest/gtest.h>

#include <cmath>
#include <Eigen/Geometry>

#include "algorithm/neo/hessian_builder.hpp"
#include "algorithm/neo/joint_limit_adapter.hpp"
#include "algorithm/neo/manipulability_gradient.hpp"
#include "algorithm/neo/obstacle_damper.hpp"
#include "algorithm/neo/reactive_qp_builder.hpp"
#include "algorithm/neo/reactive_qp_problem.hpp"
#include "algorithm/neo/reactive_qp_solver.hpp"
#include "algorithm/neo/reactive_qp_validator.hpp"
#include "algorithm/neo/task_velocity_generator.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"

namespace rq = arm_controller::algorithm::reactive_qp;

TEST(ReactiveQpTaskVelocityTest, ZeroInputProducesZeroDesiredTwist) {
    rq::TaskVelocityGenerator generator;
    rq::TaskVelocityInput input;
    rq::TaskVelocityConfig config;

    const rq::TaskVelocityOutput out = generator.compute(input, config);
    EXPECT_NEAR(out.v_des.norm(), 0.0, 1e-12);
    EXPECT_NEAR(out.v_ff.norm(), 0.0, 1e-12);
    EXPECT_NEAR(out.v_fb.norm(), 0.0, 1e-12);
}

TEST(ReactiveQpTaskVelocityTest, PoseErrorProducesFeedbackVelocity) {
    rq::TaskVelocityGenerator generator;
    rq::TaskVelocityInput input;
    rq::TaskVelocityConfig config;

    input.has_target_pose = true;
    input.T_current = Eigen::Isometry3d::Identity();
    input.T_target = Eigen::Isometry3d::Identity();
    input.T_target.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);

    const rq::TaskVelocityOutput out = generator.compute(input, config);
    EXPECT_GT(out.v_des(0), 0.0);
    EXPECT_NEAR(out.v_des(1), 0.0, 1e-9);
    EXPECT_NEAR(out.v_des(2), 0.0, 1e-9);
}

TEST(ReactiveQpTaskVelocityTest, SpeedLimitIsApplied) {
    rq::TaskVelocityGenerator generator;
    rq::TaskVelocityInput input;
    rq::TaskVelocityConfig config;
    config.max_linear_speed = 0.2;
    config.max_angular_speed = 0.3;
    input.has_target_twist = true;
    input.target_twist << 1.0, 0.0, 0.0, 0.0, 0.0, -2.0;

    const rq::TaskVelocityOutput out = generator.compute(input, config);
    EXPECT_NEAR(out.v_des.head<3>().norm(), 0.2, 1e-9);
    EXPECT_NEAR(out.v_des.tail<3>().norm(), 0.3, 1e-9);
}

TEST(ReactiveQpProblemTest, RejectsNonSymmetricHessian) {
    rq::ReactiveQpProblem p;
    p.hessian = Eigen::Matrix2d::Zero();
    p.hessian(0, 1) = 1.0;
    p.gradient = Eigen::Vector2d::Zero();
    p.constraint_matrix = Eigen::Matrix2d::Identity();
    p.lower_bound = Eigen::Vector2d(-1.0, -1.0);
    p.upper_bound = Eigen::Vector2d(1.0, 1.0);

    std::string err;
    EXPECT_FALSE(p.isWellFormed(&err));
    EXPECT_NE(err.find("not symmetric"), std::string::npos);
}

TEST(ReactiveQpValidatorTest, RejectsDimensionMismatch) {
    const Eigen::MatrixXd J = Eigen::MatrixXd::Identity(2, 2);
    const Eigen::VectorXd v_des = Eigen::VectorXd::Zero(1);
    const Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
    const Eigen::VectorXd qd_min = Eigen::VectorXd::Constant(2, -1.0);
    const Eigen::VectorXd qd_max = Eigen::VectorXd::Constant(2, 1.0);
    const Eigen::VectorXd q_min = Eigen::VectorXd::Constant(2, -2.0);
    const Eigen::VectorXd q_max = Eigen::VectorXd::Constant(2, 2.0);
    std::string err;
    EXPECT_FALSE(rq::ReactiveQpValidator::validateTaskInput(
        J, v_des, q, qd_min, qd_max, q_min, q_max, &err));
    EXPECT_NE(err.find("dimension mismatch"), std::string::npos);
}

TEST(HessianBuilderTest, RejectsBadManipGradientDimension) {
    rq::HessianBuildInput in;
    in.jacobian_task = Eigen::MatrixXd::Identity(2, 2);
    in.desired_twist = Eigen::Vector2d::Zero();
    in.manipulability_gradient = Eigen::Vector3d::Zero();
    rq::HessianBuilderConfig cfg;
    Eigen::MatrixXd H;
    Eigen::VectorXd g;
    EXPECT_FALSE(rq::HessianBuilder::build(in, cfg, H, g));
}

TEST(HessianBuilderTest, InjectsLogManipulabilityLinearTerm) {
    rq::HessianBuildInput in;
    in.jacobian_task = Eigen::MatrixXd::Identity(2, 2);
    in.desired_twist = Eigen::Vector2d::Zero();
    in.manipulability_gradient = (Eigen::Vector2d() << 1.0, -2.0).finished();
    rq::HessianBuilderConfig cfg;
    cfg.manipulability_weight = 0.5;
    cfg.task_tracking_weight = 1.0;
    cfg.joint_velocity_weight = 0.0;
    cfg.slack_weight = 0.0;

    Eigen::MatrixXd H;
    Eigen::VectorXd g;
    ASSERT_TRUE(rq::HessianBuilder::build(in, cfg, H, g));
    EXPECT_NEAR(g(0), -0.5, 1e-12);
    EXPECT_NEAR(g(1), 1.0, 1e-12);
}

TEST(JointLimitDamperTest, AppendsExpectedLowerAndUpperCbfRows) {
    const Eigen::Vector2d q(0.05, 0.95);
    rq::JointLimitData limits;
    limits.q_min = Eigen::Vector2d(0.0, 0.0);
    limits.q_max = Eigen::Vector2d(1.0, 1.0);
    rq::JointLimitDamperConfig cfg;
    cfg.safety_distance = 0.1;
    cfg.influence_distance = 0.2;
    cfg.cbf_gain_lower = 2.0;
    cfg.cbf_gain_upper = 2.0;

    const int rows = rq::JointLimitDamper::countActiveRows(q, limits, cfg);
    ASSERT_EQ(rows, 2);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows, 2);
    Eigen::VectorXd lb = Eigen::VectorXd::Zero(rows);
    Eigen::VectorXd ub = Eigen::VectorXd::Zero(rows);
    const int written = rq::JointLimitDamper::appendConstraints(q, limits, cfg, A, lb, ub, 0);
    ASSERT_EQ(written, 2);
    EXPECT_NEAR(A(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(A(1, 1), 1.0, 1e-12);
    EXPECT_GT(lb(0), 0.0);
    EXPECT_LT(ub(1), 0.0);
}

TEST(ObstacleDamperTest, AppendsExpectedCbfRow) {
    rq::ObstacleDamperConfig cfg;
    cfg.influence_distance = 0.4;
    cfg.safety_distance = 0.1;
    cfg.cbf_gain = 5.0;
    rq::ObstacleConstraintInput c;
    c.normal_jacobian = Eigen::RowVector2d(1.0, -0.5);
    c.distance = 0.2;
    rq::ObstacleConstraintInputList all{c};

    const int rows = rq::ObstacleDamper::countActiveRows(all, cfg, 2);
    ASSERT_EQ(rows, 1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows, 2);
    Eigen::VectorXd lb = Eigen::VectorXd::Zero(rows);
    Eigen::VectorXd ub = Eigen::VectorXd::Zero(rows);
    const int written = rq::ObstacleDamper::appendConstraints(all, cfg, A, lb, ub, 0);
    ASSERT_EQ(written, 1);
    EXPECT_NEAR(A(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(A(0, 1), -0.5, 1e-12);
    EXPECT_NEAR(lb(0), -0.5, 1e-12);  // -gamma*(d-ds) = -5*(0.2-0.1)
    EXPECT_GT(ub(0), 1e10);
}

TEST(ObstacleDamperTest, InvalidConfigProducesNoRows) {
    rq::ObstacleDamperConfig cfg;
    cfg.cbf_gain = 0.0;
    rq::ObstacleConstraintInput c;
    c.normal_jacobian = Eigen::RowVector2d(1.0, 0.0);
    c.distance = 0.1;
    rq::ObstacleConstraintInputList all{c};
    EXPECT_EQ(rq::ObstacleDamper::countActiveRows(all, cfg, 2), 0);
}

class MockJacobianProvider : public arm_controller::kinematics::JacobianProvider {
 public:
    bool initialize() override { return true; }
    Eigen::MatrixXd computeJacobian(
        const Eigen::VectorXd& q,
        const std::string&,
        const Eigen::Vector3d&) const override {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 2);
        J << 1.0 + q(0), 0.0,
             0.0, 1.0 + q(1),
             0.2, 0.1,
             0.0, 0.0,
             0.0, 0.0,
             0.0, 0.0;
        return J;
    }
};

TEST(ManipulabilityGradientTest, ComputesFiniteLogManipGradient) {
    auto mock = std::make_shared<MockJacobianProvider>();
    rq::ManipulabilityGradient mg(mock);
    rq::ManipulabilityGradientConfig cfg;
    cfg.finite_difference_step = 1e-4;
    cfg.determinant_damping = 1e-8;
    Eigen::VectorXd grad;
    double log_m = 0.0;
    ASSERT_TRUE(mg.compute(Eigen::Vector2d(0.1, -0.2), cfg, grad, &log_m));
    ASSERT_EQ(grad.size(), 2);
    EXPECT_TRUE(grad.allFinite());
    EXPECT_TRUE(std::isfinite(log_m));
}

TEST(ReactiveQpBuilderSolverTest, BuildAndSolveSimpleCase) {
    rq::ReactiveQpBuildInput input;
    input.q_current = Eigen::Vector2d(0.5, -0.5);
    input.jacobian_task = Eigen::Matrix2d::Identity();
    input.desired_twist = Eigen::Vector2d(0.2, -0.1);
    input.manipulability_gradient = Eigen::Vector2d::Zero();
    input.qd_min = Eigen::Vector2d(-1.0, -1.0);
    input.qd_max = Eigen::Vector2d(1.0, 1.0);
    input.joint_limits.q_min = Eigen::Vector2d(-2.0, -2.0);
    input.joint_limits.q_max = Eigen::Vector2d(2.0, 2.0);

    rq::ReactiveQpBuildConfig config;
    config.enable_joint_limit_damper = false;
    config.enable_obstacle_damper = false;
    config.hessian.task_tracking_weight = 1.0;
    config.hessian.joint_velocity_weight = 1e-4;
    config.hessian.slack_weight = 100.0;
    config.hessian.manipulability_weight = 0.0;

    rq::ReactiveQpProblem problem;
    std::string err;
    ASSERT_TRUE(rq::ReactiveQpBuilder::build(input, config, problem, &err)) << err;
    EXPECT_EQ(problem.numVariables(), 4);    // [qdot(2), slack(2)]
    EXPECT_EQ(problem.numConstraints(), 4);  // qdot bounds + slack bounds

    rq::ReactiveQpSolver solver;
    Eigen::VectorXd solution;
    ASSERT_TRUE(solver.solve(problem, solution, &err)) << err;
    ASSERT_EQ(solution.size(), 4);

    const Eigen::Vector2d qdot = solution.head<2>();
    EXPECT_NEAR(qdot(0), input.desired_twist(0), 3e-2);
  EXPECT_NEAR(qdot(1), input.desired_twist(1), 3e-2);
}

TEST(ReactiveQpBuilderSolverTest, EnforcesJointVelocityBoundsInSolution) {
    rq::ReactiveQpBuildInput input;
    input.q_current = Eigen::Vector2d::Zero();
    input.jacobian_task = Eigen::Matrix2d::Identity();
    input.desired_twist = Eigen::Vector2d(1.0, 0.0);  // ask for faster than allowed
    input.manipulability_gradient = Eigen::Vector2d::Zero();
    input.qd_min = Eigen::Vector2d(-1.0, -1.0);
    input.qd_max = Eigen::Vector2d(0.05, 1.0);        // tight upper bound on joint 0
    input.joint_limits.q_min = Eigen::Vector2d(-2.0, -2.0);
    input.joint_limits.q_max = Eigen::Vector2d(2.0, 2.0);

    rq::ReactiveQpBuildConfig config;
    config.enable_joint_limit_damper = false;
    config.enable_obstacle_damper = false;
    config.hessian.task_tracking_weight = 1.0;
    config.hessian.joint_velocity_weight = 1e-4;
    config.hessian.slack_weight = 100.0;

    rq::ReactiveQpProblem problem;
    std::string err;
    ASSERT_TRUE(rq::ReactiveQpBuilder::build(input, config, problem, &err)) << err;

    rq::ReactiveQpSolver solver;
    Eigen::VectorXd solution;
    ASSERT_TRUE(solver.solve(problem, solution, &err)) << err;
    ASSERT_EQ(solution.size(), 4);

    const Eigen::Vector2d qdot = solution.head<2>();
    EXPECT_LE(qdot(0), input.qd_max(0) + 1e-7);
    EXPECT_GE(qdot(0), input.qd_min(0) - 1e-7);
    EXPECT_NEAR(qdot(0), input.qd_max(0), 1e-3);
}

TEST(ReactiveQpBuilderSolverTest, EnforcesObstacleConstraintInSolution) {
    rq::ReactiveQpBuildInput input;
    input.q_current = Eigen::Vector2d::Zero();
    input.jacobian_task = Eigen::Matrix2d::Identity();
    input.desired_twist = Eigen::Vector2d::Zero();
    input.manipulability_gradient = Eigen::Vector2d::Zero();
    input.qd_min = Eigen::Vector2d(-1.0, -1.0);
    input.qd_max = Eigen::Vector2d(1.0, 1.0);
    input.joint_limits.q_min = Eigen::Vector2d(-2.0, -2.0);
    input.joint_limits.q_max = Eigen::Vector2d(2.0, 2.0);

    rq::ObstacleConstraintInput obstacle;
    obstacle.normal_jacobian = Eigen::RowVector2d(1.0, 0.0);
    obstacle.distance = 0.09;  // inside safety zone -> requires positive qdot(0)
    input.obstacle_constraints.push_back(obstacle);

    rq::ReactiveQpBuildConfig config;
    config.enable_joint_limit_damper = false;
    config.enable_obstacle_damper = true;
    config.obstacle_damper.influence_distance = 0.4;
    config.obstacle_damper.safety_distance = 0.1;
    config.obstacle_damper.cbf_gain = 5.0;
    config.hessian.task_tracking_weight = 1.0;
    config.hessian.joint_velocity_weight = 1e-4;
    config.hessian.slack_weight = 100.0;

    rq::ReactiveQpProblem problem;
    std::string err;
    ASSERT_TRUE(rq::ReactiveQpBuilder::build(input, config, problem, &err)) << err;

    rq::ReactiveQpSolver solver;
    Eigen::VectorXd solution;
    ASSERT_TRUE(solver.solve(problem, solution, &err)) << err;
    ASSERT_EQ(solution.size(), 4);

    const Eigen::Vector2d qdot = solution.head<2>();
    const double lower_bound = -config.obstacle_damper.cbf_gain *
        (obstacle.distance - config.obstacle_damper.safety_distance);
    const double lhs = obstacle.normal_jacobian.dot(qdot.transpose());
    EXPECT_GE(lhs + 1e-6, lower_bound);
}

TEST(ReactiveQpBuilderValidationTest, DetectsJointCbfVelocityConflict) {
    rq::ReactiveQpBuildInput input;
    input.q_current = Eigen::Vector2d(0.05, 0.0);
    input.jacobian_task = Eigen::Matrix2d::Identity();
    input.desired_twist = Eigen::Vector2d::Zero();
    input.manipulability_gradient = Eigen::Vector2d::Zero();
    input.qd_min = Eigen::Vector2d(-1.0, -1.0);
    input.qd_max = Eigen::Vector2d(0.1, 1.0);
    input.joint_limits.q_min = Eigen::Vector2d(0.0, -1.0);
    input.joint_limits.q_max = Eigen::Vector2d(1.0, 1.0);

    rq::ReactiveQpBuildConfig config;
    config.enable_joint_limit_damper = true;
    config.enable_obstacle_damper = false;
    config.joint_limit_damper.safety_distance = 0.1;
    config.joint_limit_damper.influence_distance = 0.2;
    config.joint_limit_damper.cbf_gain_lower = 5.0;

    rq::ReactiveQpProblem problem;
    std::string err;
    EXPECT_FALSE(rq::ReactiveQpBuilder::build(input, config, problem, &err));
    EXPECT_NE(err.find("Infeasible joint"), std::string::npos);
}

TEST(ReactiveQpBuilderValidationTest, DetectsObstacleCbfVelocityConflict) {
    rq::ReactiveQpBuildInput input;
    input.q_current = Eigen::Vector2d::Zero();
    input.jacobian_task = Eigen::Matrix2d::Identity();
    input.desired_twist = Eigen::Vector2d::Zero();
    input.manipulability_gradient = Eigen::Vector2d::Zero();
    input.qd_min = Eigen::Vector2d(-0.1, -0.1);
    input.qd_max = Eigen::Vector2d(0.1, 0.1);
    input.joint_limits.q_min = Eigen::Vector2d(-1.0, -1.0);
    input.joint_limits.q_max = Eigen::Vector2d(1.0, 1.0);

    rq::ObstacleConstraintInput obstacle;
    obstacle.normal_jacobian = Eigen::RowVector2d(1.0, 0.0);
    obstacle.distance = 0.0;  // deep inside safety zone
    obstacle.debug_name = "box_0";
    input.obstacle_constraints.push_back(obstacle);

    rq::ReactiveQpBuildConfig config;
    config.enable_joint_limit_damper = false;
    config.enable_obstacle_damper = true;
    config.obstacle_damper.influence_distance = 0.4;
    config.obstacle_damper.safety_distance = 0.1;
    config.obstacle_damper.cbf_gain = 5.0;

    rq::ReactiveQpProblem problem;
    std::string err;
    EXPECT_FALSE(rq::ReactiveQpBuilder::build(input, config, problem, &err));
    EXPECT_NE(err.find("Infeasible obstacle CBF"), std::string::npos);
}

TEST(ReactiveQpSolverTest, RejectsMalformedProblem) {
    rq::ReactiveQpProblem bad;
    bad.hessian = Eigen::Matrix2d::Zero();
    bad.hessian(0, 1) = 1.0;  // non-symmetric
    bad.gradient = Eigen::Vector2d::Zero();
    bad.constraint_matrix = Eigen::Matrix2d::Identity();
    bad.lower_bound = Eigen::Vector2d(-1.0, -1.0);
    bad.upper_bound = Eigen::Vector2d(1.0, 1.0);

    rq::ReactiveQpSolver solver;
    Eigen::VectorXd sol;
    std::string err;
    EXPECT_FALSE(solver.solve(bad, sol, &err));
    EXPECT_NE(err.find("not symmetric"), std::string::npos);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
