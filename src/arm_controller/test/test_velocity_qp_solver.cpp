#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include "arm_controller/utils/velocity_qp_solver.hpp"

using namespace arm_controller::utils;

// 创建 Logger 的 Mock 版本以用于测试
class VelocityQPSolverTest : public ::testing::Test {
protected:
  void SetUp() override {
    // 初始化 ROS2 日志系统
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // 在每个测试中使用 rclcpp::get_logger() 而不是在成员中存储

  // 创建一个简单的 6-DOF 机械臂 Jacobian 矩阵
  // J 是 6x6 矩阵，连接笛卡尔速度到关节速度
  static Eigen::MatrixXd createSimpleJacobian() {
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
    return J;
  }

  // 创建一个奇异的 Jacobian 矩阵（秩不足）
  static Eigen::MatrixXd createSingularJacobian() {
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
    // 设置部分行为线性相关
    J(0, 0) = 1.0; J(0, 1) = 0.5;
    J(1, 0) = 2.0; J(1, 1) = 1.0;  // 行2 = 2 * 行1
    J(2, 2) = 1.0;
    J(3, 3) = 1.0;
    J(4, 4) = 1.0;
    J(5, 5) = 1.0;
    return J;
  }

  // 创建一个有条件数较大的 Jacobian
  static Eigen::MatrixXd createIllConditionedJacobian() {
    Eigen::MatrixXd J(6, 6);
    J << 1000.0,    0.0,    0.0,    0.0,    0.0,    0.0,
         0.0,    1000.0,    0.0,    0.0,    0.0,    0.0,
         0.0,       0.0,    1.0,    0.0,    0.0,    0.0,
         0.0,       0.0,    0.0,    1.0,    0.0,    0.0,
         0.0,       0.0,    0.0,    0.0,    1.0,    0.0,
         0.0,       0.0,    0.0,    0.0,    0.0,    1.0;
    return J;
  }
};

// ============================================================================
// Test: check_direction_feasibility - 简单恒等 Jacobian
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckFeasibilitySimpleJacobian) {
  Eigen::MatrixXd J = createSimpleJacobian();
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6);

  double feasibility_ratio, residual_norm, alignment, cond;
  VelocityQPSolver::check_direction_feasibility(J, v_ee, feasibility_ratio, residual_norm, alignment, cond);

  // 对于恒等 Jacobian，所有向量都可行
  EXPECT_NEAR(feasibility_ratio, 1.0, 0.01);
  EXPECT_NEAR(residual_norm, 0.0, 1e-9);
  EXPECT_NEAR(alignment, 1.0, 0.01);
  EXPECT_NEAR(cond, 1.0, 0.01);  // 条件数接近 1
}

// ============================================================================
// Test: check_direction_feasibility - 零速度向量
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckFeasibilityZeroVelocity) {
  Eigen::MatrixXd J = createSimpleJacobian();
  Eigen::VectorXd v_ee = Eigen::VectorXd::Zero(6);

  double feasibility_ratio, residual_norm, alignment, cond;
  VelocityQPSolver::check_direction_feasibility(J, v_ee, feasibility_ratio, residual_norm, alignment, cond);

  // 零向量的可行性应该是 0
  EXPECT_EQ(feasibility_ratio, 0.0);
  EXPECT_EQ(residual_norm, 0.0);
  EXPECT_EQ(alignment, 0.0);
}

// ============================================================================
// Test: check_direction_feasibility - 低秩 Jacobian
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckFeasibilitySingularJacobian) {
  // 创建真正的秩不足矩阵（秩=3，大小=6x6）
  Eigen::MatrixXd J(6, 6);
  J.setZero();
  J.topLeftCorner(3, 3).setIdentity();  // 只有前3个对角元素非零

  Eigen::VectorXd v_ee = Eigen::VectorXd::Random(6);

  double feasibility_ratio, residual_norm, alignment, cond;
  VelocityQPSolver::check_direction_feasibility(J, v_ee, feasibility_ratio, residual_norm, alignment, cond);

  // 低秩 Jacobian 应该产生有限的可行性比率和条件数
  EXPECT_GE(feasibility_ratio, 0.0);
  EXPECT_LE(feasibility_ratio, 1.0);
  EXPECT_GT(cond, 1.0);  // 条件数应该大于1（由于秩不足）
}

// ============================================================================
// Test: check_direction_feasibility - 不良条件数
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckFeasibilityIllConditioned) {
  Eigen::MatrixXd J = createIllConditionedJacobian();
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6);

  double feasibility_ratio, residual_norm, alignment, cond;
  VelocityQPSolver::check_direction_feasibility(J, v_ee, feasibility_ratio, residual_norm, alignment, cond);

  // 高度不平衡的对角矩阵应该有较大的条件数
  EXPECT_GT(cond, 100.0);  // 条件数应该很大
}

// ============================================================================
// Test: post_verify_direction - 方向完全匹配
// ============================================================================
TEST_F(VelocityQPSolverTest, PostVerifyDirectionPerfectMatch) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_solution = Eigen::VectorXd::Ones(6);

  bool result = VelocityQPSolver::post_verify_direction(J, qd_solution, v_ee, 5.0, rclcpp::get_logger("test"));
  EXPECT_TRUE(result);
}

// ============================================================================
// Test: post_verify_direction - 零速度解
// ============================================================================
TEST_F(VelocityQPSolverTest, PostVerifyDirectionZeroSolution) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_solution = Eigen::VectorXd::Zero(6);

  // 零速度被认为是安全的（停止）
  bool result = VelocityQPSolver::post_verify_direction(J, qd_solution, v_ee, 5.0, rclcpp::get_logger("test"));
  EXPECT_TRUE(result);
}

// ============================================================================
// Test: post_verify_direction - 零期望速度
// ============================================================================
TEST_F(VelocityQPSolverTest, PostVerifyDirectionZeroDesiredVelocity) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd qd_solution = Eigen::VectorXd::Ones(6);

  bool result = VelocityQPSolver::post_verify_direction(J, qd_solution, v_ee, 5.0, rclcpp::get_logger("test"));
  EXPECT_TRUE(result);  // 认为是安全的
}

// ============================================================================
// Test: post_verify_direction - 方向偏离过大
// ============================================================================
TEST_F(VelocityQPSolverTest, PostVerifyDirectionLargeDeviation) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee(6);
  v_ee << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // 沿 X 方向

  Eigen::VectorXd qd_solution(6);
  qd_solution << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;  // 沿 Y 方向（90度偏离）

  // 允许的角度为 5 度，但实际偏离 90 度，应该返回 false
  bool result = VelocityQPSolver::post_verify_direction(J, qd_solution, v_ee, 5.0, rclcpp::get_logger("test"));
  EXPECT_FALSE(result);
}

// ============================================================================
// Test: post_verify_direction - 小角度偏离
// ============================================================================
TEST_F(VelocityQPSolverTest, PostVerifyDirectionSmallDeviation) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee(6);
  v_ee << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // 略微偏离方向（大约 2 度）
  double angle_rad = 2.0 * M_PI / 180.0;
  Eigen::VectorXd qd_solution(6);
  qd_solution << std::cos(angle_rad), std::sin(angle_rad), 0.0, 0.0, 0.0, 0.0;
  qd_solution.normalize();

  bool result = VelocityQPSolver::post_verify_direction(J, qd_solution, v_ee, 5.0, rclcpp::get_logger("test"));
  EXPECT_TRUE(result);
}

// ============================================================================
// Test: solve_velocity_qp - 简单情况
// ============================================================================
TEST_F(VelocityQPSolverTest, SolveVelocityQPSimple) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_min = -Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_solution;

  bool result = VelocityQPSolver::solve_velocity_qp(J, v_ee, qd_solution, qd_min, qd_max, rclcpp::get_logger("test"));

  EXPECT_TRUE(result);
  EXPECT_EQ(qd_solution.size(), 6);
  // 解应该接近目标速度
  EXPECT_LE((qd_solution - v_ee).norm(), 1.0);
}

// ============================================================================
// Test: solve_velocity_qp - 零速度
// ============================================================================
TEST_F(VelocityQPSolverTest, SolveVelocityQPZeroVelocity) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd qd_min = -Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_solution;

  bool result = VelocityQPSolver::solve_velocity_qp(J, v_ee, qd_solution, qd_min, qd_max, rclcpp::get_logger("test"));

  EXPECT_TRUE(result);
  EXPECT_NEAR(qd_solution.norm(), 0.0, 1e-9);
}

// ============================================================================
// Test: solve_velocity_qp - 关节速度约束
// ============================================================================
TEST_F(VelocityQPSolverTest, SolveVelocityQPWithVelocityConstraint) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6) * 2.0;
  Eigen::VectorXd qd_min = -Eigen::VectorXd::Ones(6) * 0.5;
  Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(6) * 0.5;
  Eigen::VectorXd qd_solution;

  bool result = VelocityQPSolver::solve_velocity_qp(J, v_ee, qd_solution, qd_min, qd_max, rclcpp::get_logger("test"));

  EXPECT_TRUE(result);
  // 解应该受到速度约束（允许数值误差：1e-4）
  for (int i = 0; i < 6; ++i) {
    EXPECT_GE(qd_solution(i), qd_min(i) - 1e-4);
    EXPECT_LE(qd_solution(i), qd_max(i) + 1e-4);
  }
}

// ============================================================================
// Test: solve_velocity_qp - 不可行的方向
// ============================================================================
TEST_F(VelocityQPSolverTest, SolveVelocityQPInfeasibleDirection) {
  Eigen::MatrixXd J = createSingularJacobian();
  // 创建一个不在列空间内的速度向量
  Eigen::VectorXd v_ee = Eigen::VectorXd::Zero(6);
  v_ee(1) = 1.0;  // 一个可能不在列空间内的方向

  Eigen::VectorXd qd_min = -Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_solution;

  bool result = VelocityQPSolver::solve_velocity_qp(J, v_ee, qd_solution, qd_min, qd_max, rclcpp::get_logger("test"));

  // 应该安全地停止
  EXPECT_TRUE(result);
  EXPECT_NEAR(qd_solution.norm(), 0.0, 1e-6);
}

// ============================================================================
// Test: solve_velocity_qp - 非等腰矩阵
// ============================================================================
TEST_F(VelocityQPSolverTest, SolveVelocityQPNonSquareJacobian) {
  // 3-DOF 机械臂，6 维任务空间
  Eigen::MatrixXd J(6, 3);
  J << 1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0,
       0.5, 0.0, 0.0,
       0.0, 0.5, 0.0,
       0.0, 0.0, 0.5;

  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_min = -Eigen::VectorXd::Ones(3) * 2.0;
  Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(3) * 2.0;
  Eigen::VectorXd qd_solution;

  bool result = VelocityQPSolver::solve_velocity_qp(J, v_ee, qd_solution, qd_min, qd_max, rclcpp::get_logger("test"));

  EXPECT_TRUE(result);
  EXPECT_EQ(qd_solution.size(), 3);
}

// ============================================================================
// Test: solve_velocity_qp - 不对称的关节限制
// ============================================================================
TEST_F(VelocityQPSolverTest, SolveVelocityQPAsymmetricLimits) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6) * 0.1;  // 更小的期望速度

  Eigen::VectorXd qd_min(6);
  Eigen::VectorXd qd_max(6);
  qd_min << -0.5, -0.4, -0.3, -0.2, -0.1, -0.05;
  qd_max << 0.3, 0.2, 0.15, 0.1, 0.05, 0.02;

  Eigen::VectorXd qd_solution;

  bool result = VelocityQPSolver::solve_velocity_qp(J, v_ee, qd_solution, qd_min, qd_max, rclcpp::get_logger("test"));

  EXPECT_TRUE(result);
  // 检查解是否满足约束（允许合理的数值误差）
  for (int i = 0; i < 6; ++i) {
    EXPECT_GE(qd_solution(i), qd_min(i) - 1e-3);
    EXPECT_LE(qd_solution(i), qd_max(i) + 1e-3);
  }
}

// ============================================================================
// Test: solve_velocity_qp - 非常小的速度
// ============================================================================
TEST_F(VelocityQPSolverTest, SolveVelocityQPTinyVelocity) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6) * 1e-10;
  Eigen::VectorXd qd_min = -Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_solution;

  bool result = VelocityQPSolver::solve_velocity_qp(J, v_ee, qd_solution, qd_min, qd_max, rclcpp::get_logger("test"));

  EXPECT_TRUE(result);
  // 极小的速度应该导致零解
  EXPECT_NEAR(qd_solution.norm(), 0.0, 1e-6);
}

// ============================================================================
// Test: solve_velocity_qp - 大速度
// ============================================================================
TEST_F(VelocityQPSolverTest, SolveVelocityQPLargeVelocity) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6) * 10.0;
  Eigen::VectorXd qd_min = -Eigen::VectorXd::Ones(6) * 2.0;
  Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(6) * 2.0;
  Eigen::VectorXd qd_solution;

  bool result = VelocityQPSolver::solve_velocity_qp(J, v_ee, qd_solution, qd_min, qd_max, rclcpp::get_logger("test"));

  EXPECT_TRUE(result);
  // 解应该受到约束
  for (int i = 0; i < 6; ++i) {
    EXPECT_GE(qd_solution(i), qd_min(i) - 1e-6);
    EXPECT_LE(qd_solution(i), qd_max(i) + 1e-6);
  }
}

// ============================================================================
// Test: solve_velocity_qp - 任意 Jacobian 矩阵
// ============================================================================
TEST_F(VelocityQPSolverTest, SolveVelocityQPArbitraryJacobian) {
  Eigen::MatrixXd J(6, 6);
  J << 1.0, 0.1, 0.0, 0.0, 0.0, 0.0,
       0.1, 1.0, 0.1, 0.0, 0.0, 0.0,
       0.0, 0.1, 1.0, 0.1, 0.0, 0.0,
       0.0, 0.0, 0.1, 1.0, 0.1, 0.0,
       0.0, 0.0, 0.0, 0.1, 1.0, 0.1,
       0.0, 0.0, 0.0, 0.0, 0.1, 1.0;

  Eigen::VectorXd v_ee = Eigen::VectorXd::Random(6);
  Eigen::VectorXd qd_min = -Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(6);
  Eigen::VectorXd qd_solution;

  bool result = VelocityQPSolver::solve_velocity_qp(J, v_ee, qd_solution, qd_min, qd_max, rclcpp::get_logger("test"));

  EXPECT_TRUE(result);
  EXPECT_EQ(qd_solution.size(), 6);
}

// ============================================================================
// Test: post_verify_direction - Direction within tolerance
// ============================================================================
TEST_F(VelocityQPSolverTest, PostVerifyDirectionWithinTolerance) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd qd_solution = Eigen::VectorXd::Ones(6) * 0.1;
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6) * 0.1;

  bool result = VelocityQPSolver::post_verify_direction(
    J, qd_solution, v_ee, 5.0, rclcpp::get_logger("test"));

  EXPECT_TRUE(result);
}

// ============================================================================
// Test: post_verify_direction - Large direction deviation (perpendicular)
// ============================================================================
TEST_F(VelocityQPSolverTest, PostVerifyDirectionPerpendicularDeviation) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd qd_solution(6);
  qd_solution << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd v_ee(6);
  v_ee << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;  // Perpendicular direction

  bool result = VelocityQPSolver::post_verify_direction(
    J, qd_solution, v_ee, 5.0, rclcpp::get_logger("test"));

  EXPECT_FALSE(result);  // Should fail due to large deviation
}

// ============================================================================
// Test: check_direction_feasibility - Singular matrix
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckDirectionFeasibilitySingularMatrix) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
  J(0, 0) = 1.0;  // Only one non-zero element

  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6);

  double feasibility_ratio, residual_norm, alignment, cond;
  VelocityQPSolver::check_direction_feasibility(
    J, v_ee, feasibility_ratio, residual_norm, alignment, cond);

  // Condition number should be very large for near-singular matrix
  EXPECT_GT(cond, 1e9);
}

// ============================================================================
// Test: check_direction_feasibility - Well-conditioned identity matrix
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckDirectionFeasibilityIdentityMatrix) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd v_ee = Eigen::VectorXd::Ones(6);

  double feasibility_ratio, residual_norm, alignment, cond;
  VelocityQPSolver::check_direction_feasibility(
    J, v_ee, feasibility_ratio, residual_norm, alignment, cond);

  // Identity matrix is perfectly conditioned
  EXPECT_LT(cond, 2.0);
  EXPECT_GT(feasibility_ratio, 0.99);
}

// ============================================================================
// Test: check_workspace_boundary - Safe motion within limits
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckWorkspaceBoundarySafeMotion) {
  auto hardware_manager = HardwareManager::getInstance();

  std::vector<double> joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Eigen::VectorXd qd_command = Eigen::VectorXd::Zero(6);
  qd_command << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

  std::vector<std::string> joint_names = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
  };

  // Before initialization, there are no joint limits, so motion is considered safe
  bool result = VelocityQPSolver::check_workspace_boundary(
    joint_positions, qd_command, joint_names, hardware_manager, 0.01);

  // Should return true when no limits are configured
  EXPECT_TRUE(result);
}

// ============================================================================
// Test: check_workspace_boundary - Zero velocity is always safe
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckWorkspaceBoundaryZeroVelocity) {
  auto hardware_manager = HardwareManager::getInstance();

  std::vector<double> joint_positions = {0.5, 1.0, -0.5, 0.2, 1.5, -1.0};
  Eigen::VectorXd qd_command = Eigen::VectorXd::Zero(6);

  std::vector<std::string> joint_names = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
  };

  bool result = VelocityQPSolver::check_workspace_boundary(
    joint_positions, qd_command, joint_names, hardware_manager, 0.01);

  // Zero velocity should always be safe
  EXPECT_TRUE(result);
}

// ============================================================================
// Test: check_workspace_boundary - With custom time step
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckWorkspaceBoundaryCustomTimeStep) {
  auto hardware_manager = HardwareManager::getInstance();

  std::vector<double> joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Eigen::VectorXd qd_command = Eigen::VectorXd::Ones(6) * 0.05;

  std::vector<std::string> joint_names = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
  };

  // Larger time step
  bool result = VelocityQPSolver::check_workspace_boundary(
    joint_positions, qd_command, joint_names, hardware_manager, 0.1);

  EXPECT_TRUE(result);
}

// ============================================================================
// Test: check_workspace_boundary - Empty joint names
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckWorkspaceBoundaryEmptyJointNames) {
  auto hardware_manager = HardwareManager::getInstance();

  std::vector<double> joint_positions = {0.0};
  Eigen::VectorXd qd_command = Eigen::VectorXd::Zero(1);
  std::vector<std::string> joint_names = {};

  bool result = VelocityQPSolver::check_workspace_boundary(
    joint_positions, qd_command, joint_names, hardware_manager, 0.01);

  // Should handle empty joint names gracefully
  EXPECT_TRUE(result);
}

// ============================================================================
// Test: check_workspace_boundary - Single joint
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckWorkspaceBoundarySingleJoint) {
  auto hardware_manager = HardwareManager::getInstance();

  std::vector<double> joint_positions = {0.5};
  Eigen::VectorXd qd_command = Eigen::VectorXd::Zero(1);
  qd_command << 0.2;

  std::vector<std::string> joint_names = {"joint_1"};

  bool result = VelocityQPSolver::check_workspace_boundary(
    joint_positions, qd_command, joint_names, hardware_manager, 0.01);

  EXPECT_TRUE(result);
}

// ============================================================================
// Test: check_workspace_boundary - Negative velocities
// ============================================================================
TEST_F(VelocityQPSolverTest, CheckWorkspaceBoundaryNegativeVelocities) {
  auto hardware_manager = HardwareManager::getInstance();

  std::vector<double> joint_positions = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  Eigen::VectorXd qd_command = Eigen::VectorXd::Ones(6) * -0.1;

  std::vector<std::string> joint_names = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
  };

  bool result = VelocityQPSolver::check_workspace_boundary(
    joint_positions, qd_command, joint_names, hardware_manager, 0.01);

  // Moving in negative direction should be fine without configured limits
  EXPECT_TRUE(result);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
