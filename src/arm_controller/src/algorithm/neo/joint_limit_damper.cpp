#include "joint_limit_adapter.hpp"

#include <algorithm>
#include <limits>

namespace arm_controller::algorithm::reactive_qp {

namespace {
constexpr double kInfinity = 1e20;
constexpr double kEps = 1e-9;
}  // namespace

int JointLimitDamper::countActiveRows(
    const Eigen::VectorXd& q_current,
    const JointLimitData& limits,
    const JointLimitDamperConfig& config) {

	const int dof = static_cast<int>(q_current.size());
	if (dof <= 0 || limits.q_min.size() != dof || limits.q_max.size() != dof) {
		return 0;
	}
	if (config.influence_distance < 0.0 || config.safety_distance < 0.0) {
		return 0;
	}

	int active_rows = 0;
	for (int i = 0; i < dof; ++i) {
		const double q = q_current(i);
		const double q_min = limits.q_min(i);
		const double q_max = limits.q_max(i);

		if (!(q_min < q_max)) {
			continue;
		}
		const double q_low_inf = q_min + config.influence_distance;
		const double q_up_inf = q_max - config.influence_distance;

		if (q <= q_low_inf) {
			++active_rows;
		}
		if (q >= q_up_inf) {
			++active_rows;
		}
	}
	return active_rows;
}

int JointLimitDamper::appendConstraints(
    const Eigen::VectorXd& q_current,
    const JointLimitData& limits,
    const JointLimitDamperConfig& config,
    Eigen::MatrixXd& A_qdot,
    Eigen::VectorXd& lb,
    Eigen::VectorXd& ub,
    int start_row) {

	const int dof = static_cast<int>(q_current.size());
	if (dof <= 0 || limits.q_min.size() != dof || limits.q_max.size() != dof) {
		return 0;
	}
	if (start_row < 0) {
		return 0;
	}
	if (config.influence_distance < 0.0 || config.safety_distance < 0.0) {
		return 0;
	}
	if (config.cbf_gain_lower < 0.0 || config.cbf_gain_upper < 0.0) {
		return 0;
	}
	if (A_qdot.cols() != dof || lb.size() != A_qdot.rows() || ub.size() != A_qdot.rows()) {
		return 0;
	}
	int row = start_row;
	for (int i = 0; i < dof; ++i) {
		const double q = q_current(i);
		const double q_min = limits.q_min(i);
		const double q_max = limits.q_max(i);

		if (!(q_min < q_max)) {
			continue;
		}

		// 收缩后的边界
		const double q_low_safe = q_min + config.safety_distance;
		const double q_up_safe = q_max - config.safety_distance;

		// influence zone边界
		const double q_low_inf = q_min + config.influence_distance;
		const double q_up_inf = q_max - config.influence_distance;

		// 如果安全边界已经交叉，说明配置不合理，跳过该关节
		if (!(q_low_safe < q_up_safe - kEps)) {
			continue;
		}

		// Lower limit CBF
		// h_low(q) = q_i - q_min - safety_distance >= 0
		// dh/dt = qdot_i
		// 
		// CBF 约束: qdot_i >= -gamma_low * h_low(q)
		// 写成 lb <= A qdot <= ub 的形式:
		//   A(row, i) = 1
		//   lb(row) = -gamma_low * h_low(q)
		//   ub(row) = +inf
		if (q <= q_low_inf) {
			const double h_low = q - q_low_safe;
			const double lower_bound = -config.cbf_gain_lower * h_low;

			if (row >= A_qdot.rows()) {
				return row - start_row;
			}

			A_qdot.row(row).setZero();
			A_qdot(row, i) = 1.0;
			lb(row) = lower_bound;
			ub(row) = kInfinity;
			++row;
		}
		// Upper limit CBF
		// h_up(q) = q_max - safety_distance - q_i >= 0
		// dh/dt = -qdot_i
		//
		// CBF 约束: -qdot_i >= -gamma_up * h_up(q) <=> qdot_i <= gamma_up * h_up(q)
		// 写成 lb <= A qdot <= ub 的形式:
		//   A(row, i) = 1
		//   lb(row) = -inf
		//   ub(row) = gamma_up * h_up(q)
		if (q >= q_up_inf) {
			const double h_up = q_up_safe - q;
			const double upper_bound = config.cbf_gain_upper * h_up;

			if (row >= A_qdot.rows()) {
				return row - start_row;
			}
			A_qdot.row(row).setZero();
			A_qdot(row, i) = 1.0;
			lb(row) = -kInfinity;
			ub(row) = upper_bound;
			++row;
		}
	}
	return row - start_row;
}

}  // namespace arm_controller::algorithm::reactive_qp
