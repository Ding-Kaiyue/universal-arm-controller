#include "manipulability_gradient.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>

namespace arm_controller::algorithm::reactive_qp {

ManipulabilityGradient::ManipulabilityGradient(
    std::shared_ptr<PointJacobianProvider> jacobian_provider)
    : jacobian_provider_(std::move(jacobian_provider)) {}

double ManipulabilityGradient::computeLogYoshikawaManipulability(
    const Eigen::MatrixXd& jacobian,
    const double determinant_damping) {
	if (jacobian.rows() <= 0 || jacobian.cols() <= 0) {
		return -std::numeric_limits<double>::infinity();;
	}
	
	Eigen::MatrixXd gram = jacobian * jacobian.transpose();
	gram.diagonal().array() += determinant_damping;

	if (!gram.allFinite()) {
		return -std::numeric_limits<double>::infinity();
	}

	// gram = L D L^T, where D is diagonal and L is unit lower triangular
	// det(gram) = prod(D_ii)
	// log m = 0.5 * log(det(gram)) 
	Eigen::LDLT<Eigen::MatrixXd> ldlt(gram);
	if (ldlt.info() != Eigen::Success) {
		return -std::numeric_limits<double>::infinity();
	}

	const auto& d = ldlt.vectorD();
	if (d.size() != gram.rows() || (d.array() <= 0.0).any()) {
		return -std::numeric_limits<double>::infinity();
	}

	double log_det = 0.0;
	for (int i = 0; i < d.size(); ++i) {
		if (!(d(i) > 0.0) || !std::isfinite(d(i))) {
			return -std::numeric_limits<double>::infinity();
		}
		log_det += std::log(d(i));
	}
	return 0.5 * log_det;
}

bool ManipulabilityGradient::compute(
    const Eigen::VectorXd& q,
    const ManipulabilityGradientConfig& config,
    Eigen::VectorXd& out_gradient,
    double* out_value) const {
	if (!jacobian_provider_) {
		return false;
	}
	if (q.size() <= 0 || config.finite_difference_step <= 0.0) {
		return false;
	}

	const Eigen::MatrixXd j_nominal = jacobian_provider_->computePointJacobian(
		q, config.link_name, config.point_in_link);
	if (j_nominal.cols() != q.size() || j_nominal.rows() <= 0 || !j_nominal.allFinite()) {
		return false;
	}

	const int task_dim = static_cast<int>(j_nominal.rows());
	const int dof = static_cast<int>(j_nominal.cols());

	Eigen::MatrixXd gram = j_nominal * j_nominal.transpose();
	gram.diagonal().array() += config.determinant_damping;
	if (!gram.allFinite()) {
		return false;
	}
	
	// 计算 log manipulability 在 q 处的数值
	const double log_m = computeLogYoshikawaManipulability(j_nominal, config.determinant_damping);
	if (!std::isfinite(log_m)) {
		return false;
	}
	if (out_value != nullptr) {
		*out_value = log_m;
	}

	// LDLT 用来求解 gram^{-1} * something, 避免显式 inverse
	Eigen::LDLT<Eigen::MatrixXd> ldlt(gram);
	if (ldlt.info() != Eigen::Success) {
		return false;
	}

	const auto& d = ldlt.vectorD();
	if (d.size() != task_dim) {
		return false;
	}
	for (int i = 0; i < d.size(); ++i) {
		if (!(d(i) > 0.0) || !std::isfinite(d(i))) {
			return false;
		}
	}

	out_gradient = Eigen::VectorXd::Zero(dof);
	const double h = config.finite_difference_step;

	for (int i = 0; i < dof; ++i) {
		Eigen::VectorXd q_plus = q;
		Eigen::VectorXd q_minus = q;
		q_plus(i) += h;
		q_minus(i) -= h;

		const Eigen::MatrixXd j_plus = jacobian_provider_->computePointJacobian(
			q_plus, config.link_name, config.point_in_link);
		const Eigen::MatrixXd j_minus = jacobian_provider_->computePointJacobian(
			q_minus, config.link_name, config.point_in_link);
		if (j_plus.rows() != task_dim || j_minus.rows() != task_dim || 
			j_plus.cols() != dof || j_minus.cols() != dof ||
			!j_plus.allFinite() || !j_minus.allFinite()) {
			return false;
		}

		// dJ/dq_i ≈ (J(q + h e_i) - J(q - h e_i)) / (2h)
		const Eigen::MatrixXd dJ_dqi = (j_plus - j_minus) / (2.0 * h);

		// grad_i = trace(gram^{-1} * dJ_dqi * J^T)
		//
		// 不显式求逆，先解：
		//   X = gram^{-1} * dJ_dqi
		// 再算：
		//   trace(X * J^T)
		const Eigen::MatrixXd X = ldlt.solve(dJ_dqi);
		if (ldlt.info() != Eigen::Success || !X.allFinite()) {
			return false;
		}
		const double trace_term = (X * j_nominal.transpose()).trace();
		if (!std::isfinite(trace_term)) {
			return false;
		}

		out_gradient(i) = trace_term;
	}
	return out_gradient.allFinite();
}

}  // namespace arm_controller::algorithm::reactive_qp
