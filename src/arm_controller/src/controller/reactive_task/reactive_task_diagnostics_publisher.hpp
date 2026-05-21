#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "algorithm/cartesian_path_planner/global_trajectory/global_trajectory_manager.hpp"
#include "algorithm/neo/body_obstacle_constraint_builder.hpp"
#include "arm_controller/kinematics/forward_kinematics.hpp"
#include "reactive_task_types.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace rq = arm_controller::algorithm::reactive_qp;

class ReactiveTaskDiagnosticsPublisher {
public:
    struct RuntimeVisualizationInput {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::string mapping;
        const arm_controller::kinematics::PinocchioForwardKinematics* fk_provider{nullptr};
        const rq::LinkCollisionEllipsoidList* collision_ellipsoids{nullptr};
        const Eigen::VectorXd* q_now{nullptr};
        const cp::GlobalTrajectoryManager* global_trajectory{nullptr};
        const cp::TimedCartesianSample* sample{nullptr};
        Eigen::Vector3d ee_position{Eigen::Vector3d::Zero()};
        ExecutionPhase phase{ExecutionPhase::Track};
        int planner_tick{0};
        int neo_iter{0};
        double path_progress{0.0};
        double pos_err_goal{0.0};
        double ori_err_goal{0.0};
        double qdot_norm{0.0};
        double qdot_max_abs{0.0};
        double qdot_delta_norm{0.0};
        bool qdot_limit_violation{false};
        double joint_limit_margin_min{0.0};
        double task_residual_norm{0.0};
        double ee_clearance{std::numeric_limits<double>::quiet_NaN()};
        WholeBodyStatusSnapshot whole_body_status{};
        double active_safety_distance{0.0};
        double obstacle_guidance_gate{0.0};
        double obstacle_min_distance{std::numeric_limits<double>::quiet_NaN()};
        int local_plan_sampled_steps{0};
        double local_plan_dt_sec{0.0};
        const Eigen::VectorXd* qdot{nullptr};
        const Eigen::Matrix<double, 6, 1>* v_des{nullptr};
        const Eigen::VectorXd* task_pred{nullptr};
        const Eigen::VectorXd* task_residual{nullptr};
    };

    explicit ReactiveTaskDiagnosticsPublisher(const rclcpp::Node::SharedPtr& node);

    void publishCollisionEllipsoids(
        const std::string& mapping,
        const Eigen::VectorXd& q_current,
        const arm_controller::kinematics::PinocchioForwardKinematics& fk_provider,
        const rq::LinkCollisionEllipsoidList& collision_ellipsoids);

    void clearCollisionEllipsoids(const std::string& mapping);

    void publishTrajectory(
        const std::string& mapping,
        const cp::GlobalTrajectoryManager& global_trajectory,
        const cp::TimedCartesianSample& sample,
        const Eigen::Vector3d& ee_position);

    void publishExecutionTrace(
        const std::string& mapping,
        const Eigen::Vector3d& ee_position);

    void clearTrajectory(const std::string& mapping);

    void clearVisuals(const std::string& mapping);

    void logPhaseTransition(
        const std::string& mapping,
        const ExecutionStatusOutput& phase_decision,
        double path_progress,
        double pos_err_goal,
        double ori_err_goal) const;

    void publishRuntimeCycle(const RuntimeVisualizationInput& input);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        collision_ellipsoid_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        trajectory_marker_pub_;
    std::mutex collision_ellipsoid_marker_mutex_;
    std::map<std::string, std::size_t> collision_ellipsoid_marker_counts_;
    std::mutex execution_trace_mutex_;
    std::map<std::string, std::vector<Eigen::Vector3d>> execution_traces_;
};

}  // namespace arm_controller::controller::reactive_task
