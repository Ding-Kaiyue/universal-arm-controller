#pragma once

#include <Eigen/Core>

namespace arm_controller::algorithm::reactive_qp {

struct JointLimitData {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::VectorXd q_min;
	Eigen::VectorXd q_max;
};

struct JointLimitDamperConfig {
	// 在距离真实关节极限 safety_distance 的位置定义安全边界：
	//   q_i >= q_min(i) + safety_distance
	//   q_i <= q_max(i) - safety_distance
	//
	// CBF 将保证系统不穿过这个“收缩后的安全边界”。
	double safety_distance{0.05};      // rad
	// 仅当关节进入 influence zone 时，才向QP中添加对应约束。
	// 下限触发条件:
	//   q_i <= q_min(i) + influence_distance
	// 上限触发条件:
	//   q_i >= q_max(i) - influence_distance
	double influence_distance{0.15};   // rad

	// CBF 增益。越大表示越强地“推回安全区”
	double cbf_gain_lower = 5.0;
	double cbf_gain_upper = 5.0;
};

class JointLimitDamper {
public:
	// 统计当前会激活多少条关节限位约束
	static int countActiveRows(
		const Eigen::VectorXd& q_current,
		const JointLimitData& limits,
		const JointLimitDamperConfig& config);
	
	// 从 start_row 开始往 A_qdot / lb / ub 中追加约束
	//
	// 约束格式：
	//   lb <= A_qdot * qdot <= ub
	//
	// 返回实际追加的约束行数
	static int appendConstraints(
		const Eigen::VectorXd& q_current,
		const JointLimitData& limits,
		const JointLimitDamperConfig& config,
		Eigen::MatrixXd& A_qdot,
		Eigen::VectorXd& lb,
		Eigen::VectorXd& ub,
		int start_row);
};

}  // namespace arm_controller::algorithm::reactive_qp
