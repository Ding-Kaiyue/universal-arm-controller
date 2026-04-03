#include "reactive_qp_validator.hpp"

#include <sstream>

namespace arm_controller::algorithm::reactive_qp {

bool ReactiveQpValidator::validateTaskInput(
    const Eigen::MatrixXd& jacobian_task,
    const Eigen::VectorXd& desired_twist,
    const Eigen::VectorXd& q_current,
    const Eigen::VectorXd& qd_min,
    const Eigen::VectorXd& qd_max,
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max,
    std::string* error) {

	auto fail = [&](const std::string& msg) {
		if (error != nullptr) {
		*error = msg;
		}
		return false;
	};

	if (jacobian_task.rows() <= 0 || jacobian_task.cols() <= 0) {
		return fail("Task Jacobian is empty.");
	}

	const int dof = jacobian_task.cols();
	const int task_dim = jacobian_task.rows();
	if (desired_twist.size() != task_dim) {
		return fail("Desired twist dimension mismatch.");
	}
	if (q_current.size() != dof || qd_min.size() != dof || qd_max.size() != dof ||
		q_min.size() != dof || q_max.size() != dof) {
		return fail("Joint vectors dimension mismatch with Jacobian columns.");
	}
	if (!jacobian_task.allFinite() || !desired_twist.allFinite() || !q_current.allFinite() ||
		!qd_min.allFinite() || !qd_max.allFinite() || !q_min.allFinite() || !q_max.allFinite()) {
		return fail("Task input contains non-finite values.");
	}

	for (int i = 0; i < dof; ++i) {
		if (qd_min(i) > qd_max(i)) {
		std::ostringstream oss;
		oss << "Invalid velocity limits at joint " << i << ".";
		return fail(oss.str());
		}
		if (q_min(i) > q_max(i)) {
		std::ostringstream oss;
		oss << "Invalid position limits at joint " << i << ".";
		return fail(oss.str());
		}
	}
	return true;
}

bool ReactiveQpValidator::validateProblem(
    const ReactiveQpProblem& problem,
    std::string* error) {
  	return problem.isWellFormed(error);
}

}  // namespace arm_controller::algorithm::reactive_qp
