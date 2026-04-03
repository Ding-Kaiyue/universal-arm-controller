#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace arm_controller::algorithm::reactive_qp {

struct ObstacleDamperConfig {
    // 仅当距离进入 influence zone 时，才激活该障碍约束
    double influence_distance{0.40};   // m

    // 安全距离，CBF 保证 d(q) >= safety_distance
    double safety_distance{0.05};      // m

    // CBF 增益，越大表示越强地“推离障碍物”
    double cbf_gain{5.0};
};


struct ObstacleConstraintInput {
    // normal_jacobian = n^T * J_point, shape: 1 x dof
    // 使得 d_dot = normal_jacobian * qdot
    Eigen::RowVectorXd normal_jacobian;

    // 当前有符号距离 d(q)
    double distance{1e9};

    std::string debug_name;
};

class ObstacleDamper {
public:
    // 统计当前会激活多少条障碍 CBF 约束
    static int countActiveRows(
        const std::vector<ObstacleConstraintInput>& constraints,
        const ObstacleDamperConfig& config,
        int dof);
    
    // 从 start_row 开始往 A_qdot / lb / ub 中追加约束
    //
    // 约束格式：
    //   lb <= A_qdot * qdot <= ub
    //
    // 对每个激活障碍，添加：
    //   normal_jacobian * qdot >= -cbf_gain * (distance - safety_distance)
    //
    // 返回实际追加的约束行数
    static int appendConstraints(
        const std::vector<ObstacleConstraintInput>& constraints,
        const ObstacleDamperConfig& config,
        Eigen::MatrixXd& A_qdot,
        Eigen::VectorXd& lb,
        Eigen::VectorXd& ub,
        int start_row);
};

}  // namespace arm_controller::algorithm::reactive_qp
