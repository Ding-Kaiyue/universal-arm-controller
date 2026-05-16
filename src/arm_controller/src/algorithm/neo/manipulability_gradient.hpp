#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>

#include "arm_controller/kinematics/jacobian_provider.hpp"

namespace arm_controller::algorithm::reactive_qp {

struct ManipulabilityGradientConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double finite_difference_step{1e-4};
  double determinant_damping{1e-8};
  std::string link_name;
  Eigen::Vector3d point_in_link{Eigen::Vector3d::Zero()};
};

class ManipulabilityGradient {
public:
	explicit ManipulabilityGradient(
        std::shared_ptr<arm_controller::kinematics::JacobianProvider> jacobian_provider);

	// 计算 log manipulability:
	//   log m(q) = 0.5 * log det(J(q) J(q)^T + lambda * I)
	// 
	// out_gradient 返回 ∇ log m(q)
	// out_value 若非空，则返回 log m(q) 的数值
	bool compute(
		const Eigen::VectorXd& q,
		const ManipulabilityGradientConfig& config,
		Eigen::VectorXd& out_gradient,
		double* out_value = nullptr) const;

private:
	// 返回 log manipulability:
	//   log m(q) = 0.5 * log det(J(q) J(q)^T + lambda * I)
	static double computeLogYoshikawaManipulability(
		const Eigen::MatrixXd& jacobian,
		double determinant_damping);

	std::shared_ptr<arm_controller::kinematics::JacobianProvider> jacobian_provider_;
};

}  // namespace arm_controller::algorithm::reactive_qp
