#pragma once

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "algorithm/neo/manipulability_gradient.hpp"
#include "algorithm/neo/reactive_qp_builder.hpp"
#include "algorithm/neo/reactive_qp_solver.hpp"
#include "algorithm/neo/task_velocity_generator.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"
#include "controller/reactive_task/reactive_task_execution_context.hpp"
#include "controller/reactive_task/reactive_task_terminal_policy.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace rq = arm_controller::algorithm::reactive_qp;

class ReactiveTaskNeoPipeline {
public:
    struct PrepareInput {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::VectorXd q_now;
        Eigen::VectorXd qd_min;
        Eigen::VectorXd qd_max;
        rq::JointLimitData joint_limits;
        const arm_controller::kinematics::JacobianProvider* jacobian_provider{nullptr};
        const rq::TaskVelocityOutput* task_out{nullptr};
        const ReactiveTaskTerminalPolicy::PhaseFlags* phase_flags{nullptr};
        const ReactiveTaskExecutionContext* exec_ctx{nullptr};
        const rq::HumanLikeJointPreferenceConfig* joint_preference_cfg{nullptr};
        const cp::TimedCartesianSample* path_follow_joint_anchor_sample{nullptr};
        bool path_follow_joint_anchor_sample_valid{false};
        const cp::TimedCartesianSample* current_sample{nullptr};
        const Eigen::VectorXd* local_planner_joint_target{nullptr};
        double local_planner_joint_target_dt_sec{0.0};
        const rq::ManipulabilityGradient* manipulability_gradient{nullptr};
        const rq::ManipulabilityGradientConfig* manipulability_cfg{nullptr};
    };

    struct PrepareOutput {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool ok{false};
        Eigen::MatrixXd J;
        Eigen::MatrixXd J_task;
        Eigen::VectorXd manipulability_gradient;
        Eigen::VectorXd posture_qdot_ref;
        Eigen::VectorXd posture_joint_weights;
        rq::ReactiveQpBuildInput qp_input;
    };

    struct SolveInput {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::string mapping;
        int planner_tick{0};
        const rq::ReactiveQpBuildInput* qp_input{nullptr};
        const rq::ReactiveQpBuildConfig* qp_build_cfg{nullptr};
        rq::ReactiveQpSolver* solver{nullptr};
        Eigen::VectorXd qd_min;
        Eigen::VectorXd qd_max;
        const Eigen::MatrixXd* task_jacobian{nullptr};
        const Eigen::Matrix<double, 6, 1>* desired_twist{nullptr};
        rclcpp::Logger logger{rclcpp::get_logger("reactive_task_neo_pipeline")};
    };

    struct SolveOutput {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool ok{false};
        std::vector<double> qdot_cmd;
        Eigen::VectorXd qdot_eigen;
        Eigen::VectorXd task_pred;
        Eigen::VectorXd task_residual;
        double task_residual_norm{0.0};
    };

    bool prepare(const PrepareInput& input, PrepareOutput* output) const;
    bool solve(const SolveInput& input, SolveOutput* output) const;
};

}  // namespace arm_controller::controller::reactive_task
