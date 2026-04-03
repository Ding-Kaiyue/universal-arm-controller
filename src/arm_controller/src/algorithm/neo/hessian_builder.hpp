#pragma once

#include <Eigen/Core>

namespace arm_controller::algorithm::reactive_qp {

struct HessianBuilderConfig {
	double task_tracking_weight{1.0};
	double joint_velocity_weight{1e-4};
	double slack_weight{1e2};
	// Weight for maximizing log-manipulability:
	// objective contains -w_log_m * (∇log m(q))^T * qdot
	// For log-gradient, start from a conservative non-zero weight.
	double manipulability_weight{0.02};
};

struct HessianBuildInput {
	Eigen::MatrixXd jacobian_task;
	Eigen::VectorXd desired_twist;
	// IMPORTANT:
	// This is gradient of log manipulability, i.e. ∇log m(q), not ∇m(q).
	Eigen::VectorXd manipulability_gradient;
};

class HessianBuilder {
 	public:
  	static bool build(
      const HessianBuildInput& input,
      const HessianBuilderConfig& config,
      Eigen::MatrixXd& out_hessian,
      Eigen::VectorXd& out_gradient);
};

}  // namespace arm_controller::algorithm::reactive_qp
