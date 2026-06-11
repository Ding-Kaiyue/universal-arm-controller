#include <gtest/gtest.h>

#include "controller/reactive_task/goal/whole_body_goal_generator.hpp"

namespace rt = arm_controller::controller::reactive_task;
namespace cp = arm_controller::algorithm::cartesian_path_planner;

namespace {

constexpr int kBaseDof = rt::WholeBodyGoalGenerator::kBaseDof;
constexpr int kArmDof = rt::WholeBodyGoalGenerator::kArmDof;
constexpr int kFullDof = rt::WholeBodyGoalGenerator::kFullDof;

Eigen::Matrix3d yawRotation(const double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Isometry3d makePose(
    const Eigen::Vector3d& p,
    const double yaw = 0.0) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = p;
    pose.linear() = yawRotation(yaw);
    return pose;
}

bool mockFullStateFk(
    const Eigen::VectorXd& q,
    Eigen::Isometry3d* left,
    Eigen::Isometry3d* right) {
    if (q.size() != kFullDof || left == nullptr || right == nullptr) {
        return false;
    }

    const Eigen::Vector3d base(q[0], q[1], 0.0);
    const double yaw = q[2];
    const Eigen::Matrix3d R = yawRotation(yaw);
    const Eigen::Vector3d left_local = q.segment(kBaseDof, 3);
    const Eigen::Vector3d right_local = q.segment(kBaseDof + 6, 3);

    *left = makePose(base + R * left_local, yaw + q[kBaseDof + 3]);
    *right = makePose(base + R * right_local, yaw + q[kBaseDof + 6 + 3]);
    return true;
}

rt::WholeBodyGoalGenerator::Input makeBaseInput() {
    rt::WholeBodyGoalGenerator::Input input;
    input.q_start = Eigen::VectorXd::Zero(kFullDof);
    input.q_min = Eigen::VectorXd::Constant(kFullDof, -10.0);
    input.q_max = Eigen::VectorXd::Constant(kFullDof, 10.0);
    input.q_min[2] = -M_PI;
    input.q_max[2] = M_PI;
    input.q_nominal = input.q_start;
    input.left_target = makePose(Eigen::Vector3d(2.0, 0.40, 0.70), 0.0);
    input.right_target = makePose(Eigen::Vector3d(2.0, -0.40, 0.70), 0.0);
    input.full_state_fk = mockFullStateFk;
    input.joint_state_validator =
        [](const Eigen::VectorXd& q,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool ok = q.size() == kFullDof && q.allFinite() && q[0] > 1.0;
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.10 : -0.10;
                diag->reason = ok ? "ok" : "base_too_close";
            }
            return ok;
        };
    return input;
}

}  // namespace

TEST(WholeBodyGoalGeneratorTest, ProducesVerifiedQGoalFromProjectedSeed) {
    rt::WholeBodyGoalGenerator::Config cfg;
    cfg.position_tolerance_m = 1e-3;
    cfg.orientation_tolerance_rad = 1e-3;
    cfg.optimizer_iterations = 0;
    cfg.max_goal_candidates = 4;
    rt::WholeBodyGoalGenerator generator(cfg);

    rt::WholeBodyGoalGenerator::Input input = makeBaseInput();
    rt::WholeBodyGoalGenerator::Seed seed;
    seed.base = {1.50, 0.0, 0.0};
    seed.arm_seed = Eigen::VectorXd::Zero(kArmDof);
    input.seeds.push_back(seed);
    input.seed_projector =
        [&input](const rt::WholeBodyGoalGenerator::BaseState& base,
                 const Eigen::VectorXd&,
                 Eigen::VectorXd* arm_solution) {
            if (arm_solution == nullptr) {
                return false;
            }
            arm_solution->setZero(kArmDof);
            (*arm_solution)[0] = input.left_target.translation().x() - base.x;
            (*arm_solution)[1] = input.left_target.translation().y() - base.y;
            (*arm_solution)[2] = input.left_target.translation().z();
            (*arm_solution)[3] = -base.yaw;
            (*arm_solution)[6] = input.right_target.translation().x() - base.x;
            (*arm_solution)[7] = input.right_target.translation().y() - base.y;
            (*arm_solution)[8] = input.right_target.translation().z();
            (*arm_solution)[9] = -base.yaw;
            return true;
        };

    const rt::WholeBodyGoalGenerator::Output output = generator.generate(input);

    ASSERT_EQ(output.q_goal_candidates.size(), 1u);
    ASSERT_FALSE(output.diagnostics.empty());
    const Eigen::VectorXd& q_goal = output.q_goal_candidates.front();
    EXPECT_EQ(q_goal.size(), kFullDof);
    EXPECT_NEAR(q_goal[0], 1.50, 1e-9);
    EXPECT_NEAR(q_goal[1], 0.0, 1e-9);
    EXPECT_NEAR(q_goal[2], 0.0, 1e-9);

    Eigen::Isometry3d left = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d right = Eigen::Isometry3d::Identity();
    ASSERT_TRUE(mockFullStateFk(q_goal, &left, &right));
    EXPECT_TRUE(left.translation().isApprox(input.left_target.translation(), 1e-9));
    EXPECT_TRUE(right.translation().isApprox(input.right_target.translation(), 1e-9));
    EXPECT_NEAR(
        Eigen::AngleAxisd(left.linear().transpose() * input.left_target.linear()).angle(),
        0.0,
        1e-9);
    EXPECT_NEAR(
        Eigen::AngleAxisd(right.linear().transpose() * input.right_target.linear()).angle(),
        0.0,
        1e-9);

    const auto valid_diag = std::find_if(
        output.diagnostics.begin(),
        output.diagnostics.end(),
        [](const auto& diag) { return diag.valid; });
    ASSERT_NE(valid_diag, output.diagnostics.end());
    EXPECT_TRUE(valid_diag->collision_free);
    EXPECT_EQ(valid_diag->reason, "ok");
    EXPECT_LE(valid_diag->left_position_error_m, cfg.position_tolerance_m);
    EXPECT_LE(valid_diag->right_position_error_m, cfg.position_tolerance_m);
    EXPECT_LE(valid_diag->left_orientation_error_rad, cfg.orientation_tolerance_rad);
    EXPECT_LE(valid_diag->right_orientation_error_rad, cfg.orientation_tolerance_rad);
}

TEST(WholeBodyGoalGeneratorTest, RejectsPoseErrorAndCollisionFailures) {
    rt::WholeBodyGoalGenerator::Config cfg;
    cfg.position_tolerance_m = 0.01;
    cfg.orientation_tolerance_rad = 0.01;
    cfg.optimizer_iterations = 0;
    cfg.max_goal_candidates = 4;
    rt::WholeBodyGoalGenerator generator(cfg);

    rt::WholeBodyGoalGenerator::Input input = makeBaseInput();
    rt::WholeBodyGoalGenerator::Seed colliding_seed;
    colliding_seed.base = {0.20, 0.0, 0.0};
    colliding_seed.arm_seed = Eigen::VectorXd::Zero(kArmDof);
    input.seeds.push_back(colliding_seed);

    rt::WholeBodyGoalGenerator::Seed pose_error_seed;
    pose_error_seed.base = {1.50, 0.0, 0.0};
    pose_error_seed.arm_seed = Eigen::VectorXd::Zero(kArmDof);
    input.seeds.push_back(pose_error_seed);

    input.seed_projector =
        [](const rt::WholeBodyGoalGenerator::BaseState& base,
           const Eigen::VectorXd&,
           Eigen::VectorXd* arm_solution) {
            if (arm_solution == nullptr) {
                return false;
            }
            arm_solution->setZero(kArmDof);
            if (base.x < 1.0) {
                // Correct pose, but validator should reject the base location.
                (*arm_solution)[0] = 2.0 - base.x;
                (*arm_solution)[1] = 0.40 - base.y;
                (*arm_solution)[2] = 0.70;
                (*arm_solution)[6] = 2.0 - base.x;
                (*arm_solution)[7] = -0.40 - base.y;
                (*arm_solution)[8] = 0.70;
            } else {
                // Deliberately wrong pose.
                (*arm_solution)[0] = -2.0;
                (*arm_solution)[6] = -2.0;
            }
            return true;
        };

    const rt::WholeBodyGoalGenerator::Output output = generator.generate(input);

    EXPECT_TRUE(output.q_goal_candidates.empty());
    ASSERT_EQ(output.diagnostics.size(), 2u);
    EXPECT_FALSE(output.diagnostics[0].valid);
    EXPECT_EQ(output.diagnostics[0].reason, "base_too_close");
    EXPECT_FALSE(output.diagnostics[0].collision_free);
    EXPECT_FALSE(output.diagnostics[1].valid);
    EXPECT_EQ(output.diagnostics[1].reason, "pose_error_too_large");
}

