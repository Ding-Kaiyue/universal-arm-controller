#include "hessian_builder.hpp"
#include <cmath>

namespace arm_controller::algorithm::reactive_qp {

bool HessianBuilder::build(
    const HessianBuildInput& input,
    const HessianBuilderConfig& config,
    Eigen::MatrixXd& out_hessian,
    Eigen::VectorXd& out_gradient) {
		
	const int task_dim = static_cast<int>(input.jacobian_task.rows());
	const int dof = static_cast<int>(input.jacobian_task.cols());
	if (task_dim <= 0 || dof <= 0) {
		return false;
	}
	if (input.desired_twist.size() != task_dim) {
		return false;
	}
	if (input.q_current.size() != dof) {
		return false;
	}

	Eigen::VectorXd qdot_ref = input.posture_velocity_reference;
	if (qdot_ref.size() == 0) {
		qdot_ref = Eigen::VectorXd::Zero(dof);
	}
	if (qdot_ref.size() != dof) {
		return false;
	}

	Eigen::VectorXd qdot_prev = input.previous_qdot_reference;
	if (qdot_prev.size() == 0) {
		qdot_prev = Eigen::VectorXd::Zero(dof);
	}
	if (qdot_prev.size() != dof) {
		return false;
	}

	Eigen::VectorXd posture_w = input.posture_joint_weights;
	if (posture_w.size() == 0) {
		posture_w = Eigen::VectorXd::Ones(dof);
	}
	if (posture_w.size() != dof) {
		return false;
	}

	// grad_log_m := ∇log m(q)
	Eigen::VectorXd grad_log_m = input.manipulability_gradient;
	if (grad_log_m.size() == 0) {
		grad_log_m = Eigen::VectorXd::Zero(dof);
	}
	if (grad_log_m.size() != dof) {
		return false;
	}

	Eigen::RowVectorXd shell_jacobian = input.shell_jacobian;
	const bool shell_enabled =
		shell_jacobian.size() == dof &&
		shell_jacobian.allFinite() &&
		std::isfinite(input.shell_desired_rate) &&
		std::isfinite(input.shell_weight_scale) &&
		input.shell_weight_scale > 1e-9 &&
		config.shell_tracking_weight > 1e-9;
	if (shell_jacobian.size() != 0 && shell_jacobian.size() != dof) {
		return false;
	}
	Eigen::MatrixXd tangential_jacobian = input.tangential_jacobian;
	Eigen::VectorXd tangential_desired_velocity = input.tangential_desired_velocity;
	const bool tangential_enabled =
		tangential_jacobian.rows() > 0 &&
		tangential_jacobian.cols() == dof &&
		tangential_jacobian.allFinite() &&
		tangential_desired_velocity.size() == tangential_jacobian.rows() &&
		tangential_desired_velocity.allFinite() &&
		std::isfinite(input.tangential_weight_scale) &&
		input.tangential_weight_scale > 1e-9 &&
		config.shell_tracking_weight > 1e-9;
	if (tangential_jacobian.size() != 0 && tangential_jacobian.cols() != dof) {
		return false;
	}

	// Decision vector:
	//   x = [qdot; s], where qdot in R^dof, s in R^task_dim (task slack)
	const int nv = dof + task_dim;
	out_hessian = Eigen::MatrixXd::Zero(nv, nv);
	out_gradient = Eigen::VectorXd::Zero(nv);

	const double w_task = config.task_tracking_weight;
	const double w_qdot = config.joint_velocity_weight;
	const double w_slack = config.slack_weight;
	const double w_shell = config.shell_tracking_weight;
	const double w_smooth = config.qdot_smoothing_weight;
	const double w_posture = config.posture_weight;
	const double w_log_m = config.manipulability_weight;

	// Minimize:
	//   w_task * ||J*qdot + s - v_des||^2
	// + w_qdot * ||qdot||^2
	// + w_slack * ||s||^2
	// + w_smooth * ||qdot - qdot_prev||^2
	// + w_posture * ||W_posture * (qdot - qdot_ref)||^2
	// - w_log_m * (grad_log_m)^T * qdot
	out_hessian.topLeftCorner(dof, dof).noalias() +=
		2.0 * w_task * input.jacobian_task.transpose() * input.jacobian_task;
	out_hessian.topLeftCorner(dof, dof).diagonal().array() += 2.0 * w_qdot;
	out_hessian.topLeftCorner(dof, dof).diagonal().array() += 2.0 * w_smooth;
	out_gradient.head(dof).noalias() += -2.0 * w_smooth * qdot_prev;
	for (int i = 0; i < dof; ++i) {
		const double wi = (posture_w(i) > 0.0) ? posture_w(i) : 0.0;
		const double wi2 = wi * wi;
		out_hessian(i, i) += 2.0 * w_posture * wi2;
		out_gradient(i) += -2.0 * w_posture * wi2 * qdot_ref(i);
	}

	out_hessian.topRightCorner(dof, task_dim).noalias() +=
		2.0 * w_task * input.jacobian_task.transpose();
	out_hessian.bottomLeftCorner(task_dim, dof) =
		out_hessian.topRightCorner(dof, task_dim).transpose();

	out_hessian.bottomRightCorner(task_dim, task_dim).diagonal().array() +=
		2.0 * (w_task + w_slack);

	out_gradient.head(dof).noalias() +=
		-2.0 * w_task * input.jacobian_task.transpose() * input.desired_twist;
	out_gradient.tail(task_dim).noalias() += -2.0 * w_task * input.desired_twist;

	if (shell_enabled) {
		const double shell_weight = w_shell * input.shell_weight_scale;
		out_hessian.topLeftCorner(dof, dof).noalias() +=
			2.0 * shell_weight * shell_jacobian.transpose() * shell_jacobian;
		out_gradient.head(dof).noalias() +=
			-2.0 * shell_weight * shell_jacobian.transpose() * input.shell_desired_rate;
	}
	if (tangential_enabled) {
		const double tangential_weight = w_shell * input.tangential_weight_scale;
		out_hessian.topLeftCorner(dof, dof).noalias() +=
			2.0 * tangential_weight * tangential_jacobian.transpose() * tangential_jacobian;
		out_gradient.head(dof).noalias() +=
			-2.0 * tangential_weight *
			tangential_jacobian.transpose() * tangential_desired_velocity;
	}

	// Maximize log manipulability via linear term in qdot.
	out_gradient.head(dof).noalias() += -w_log_m * grad_log_m;

	return out_hessian.allFinite() && out_gradient.allFinite();
}

}  // namespace arm_controller::algorithm::reactive_qp
