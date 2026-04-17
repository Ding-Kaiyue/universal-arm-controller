#include "manipulability_gradient.hpp"
#include "manipulator_hessian_tensor.hpp"

#include <Eigen/Cholesky>
#include <cmath>
#include <limits>

namespace arm_controller::algorithm::reactive_qp {

ManipulabilityGradient::ManipulabilityGradient(
    std::shared_ptr<arm_controller::kinematics::JacobianProvider> jacobian_provider)
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
	if (q.size() <= 0) {
		return false;
	}

	const Eigen::MatrixXd j_nominal = jacobian_provider_->computeJacobian(
		q, config.link_name, config.point_in_link);
	if (j_nominal.cols() != q.size() || j_nominal.rows() <= 0 || !j_nominal.allFinite()) {
		return false;
	}
	// Analytic tensor builder assumes spatial Jacobian (6xn).
	if (j_nominal.rows() != 6) {
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

	std::vector<Eigen::MatrixXd> dJ_dq;
	if (!ManipulatorHessianTensorBuilder::buildFromJacobian(j_nominal, dJ_dq, nullptr)) {
		return false;
	}
	if (static_cast<int>(dJ_dq.size()) != dof) {
		return false;
	}

	out_gradient = Eigen::VectorXd::Zero(dof);
	for (int i = 0; i < dof; ++i) {
		const Eigen::MatrixXd& dJ_dqi = dJ_dq[static_cast<std::size_t>(i)];
		if (dJ_dqi.rows() != task_dim || dJ_dqi.cols() != dof || !dJ_dqi.allFinite()) {
			return false;
		}

		// For log-manipulability:
		//   grad_i = trace((J J^T + lambda I)^(-1) * (dJ/dq_i) * J^T)
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
