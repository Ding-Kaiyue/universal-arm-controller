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

	// Decision vector:
	//   x = [qdot; s], where qdot in R^dof, s in R^task_dim (task slack)
	const int nv = dof + task_dim;
	out_hessian = Eigen::MatrixXd::Zero(nv, nv);
	out_gradient = Eigen::VectorXd::Zero(nv);

	const double w_task = config.task_tracking_weight;
	const double w_qdot = config.joint_velocity_weight;
	const double w_slack = config.slack_weight;
	const double w_posture = config.posture_weight;
	const double w_log_m = config.manipulability_weight;

	// Minimize:
	//   w_task * ||J*qdot + s - v_des||^2
	// + w_qdot * ||qdot||^2
	// + w_slack * ||s||^2
	// + w_posture * ||W_posture * (qdot - qdot_ref)||^2
	// - w_log_m * (grad_log_m)^T * qdot
	out_hessian.topLeftCorner(dof, dof).noalias() +=
		2.0 * w_task * input.jacobian_task.transpose() * input.jacobian_task;
	out_hessian.topLeftCorner(dof, dof).diagonal().array() += 2.0 * w_qdot;
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

	// Maximize log manipulability via linear term in qdot.
	out_gradient.head(dof).noalias() += -w_log_m * grad_log_m;

	return out_hessian.allFinite() && out_gradient.allFinite();
}

}  // namespace arm_controller::algorithm::reactive_qp
