#pragma once

#include <Eigen/Core>

namespace arm_controller::algorithm::reactive_qp {

struct HessianBuilderConfig {
	double task_tracking_weight{1.0};
	double joint_velocity_weight{1e-4};
	double slack_weight{1e2};
	double shell_tracking_weight{0.0};
	double shell_centering_gain{1.0};
	double shell_target_velocity_limit{0.10};
	double qdot_smoothing_weight{0.0};
	// Weight for posture preference on joint velocity:
	//   w_posture * ||W_posture * (qdot - qdot_ref)||^2
	double posture_weight{0.0};
	// Weight for maximizing log-manipulability:
	// objective contains -w_log_m * (∇log m(q))^T * qdot
	// For log-gradient, start from a conservative non-zero weight.
	double manipulability_weight{0.02};
};

struct HessianBuildInput {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::MatrixXd jacobian_task;
	Eigen::VectorXd desired_twist;
	// Current joint position q in R^n.
	Eigen::VectorXd q_current;
	// Optional posture velocity reference qdot_ref in R^n.
	Eigen::VectorXd posture_velocity_reference;
	// Optional temporal smoothing reference qdot_prev in R^n:
	//   w_smooth * ||qdot - qdot_prev||^2
	Eigen::VectorXd previous_qdot_reference;
	// Optional non-negative per-joint posture weights in R^n.
	// If empty, all ones are used.
	Eigen::VectorXd posture_joint_weights;
	// IMPORTANT:
	// This is gradient of log manipulability, i.e. ∇log m(q), not ∇m(q).
	Eigen::VectorXd manipulability_gradient;
	// Optional shell-tracking term on obstacle clearance rate:
	//   shell_weight_scale * w_shell * ||nJ * qdot - d_dot_des||^2
	Eigen::RowVectorXd shell_jacobian;
	double shell_desired_rate{0.0};
	double shell_weight_scale{1.0};
	// Optional tangential tracking term on shell tangent plane:
	//   tangential_weight_scale * w_tan * ||J_tan * qdot - v_tan_des||^2
	Eigen::MatrixXd tangential_jacobian;
	Eigen::VectorXd tangential_desired_velocity;
	double tangential_weight_scale{1.0};
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
