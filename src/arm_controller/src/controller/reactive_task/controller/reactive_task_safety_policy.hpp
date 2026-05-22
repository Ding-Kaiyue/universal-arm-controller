#pragma once

#include <Eigen/Core>

#include <optional>
#include <string>
#include <unordered_map>

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "algorithm/neo/body_obstacle_constraint_builder.hpp"
#include "algorithm/neo/obstacle_damper.hpp"
#include "algorithm/neo/reactive_qp_builder.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"
#include "controller/reactive_task/controller/reactive_task_execution_context.hpp"
#include "controller/reactive_task/controller/reactive_task_terminal_policy.hpp"
#include "controller/reactive_task/controller/reactive_task_types.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace rq = arm_controller::algorithm::reactive_qp;

class ReactiveTaskSafetyPolicy {
public:
    struct Input {
        bool enable_obstacle_constraints{false};
        int planner_tick{0};
        bool reference_finished{false};
        double path_progress{0.0};
        double hard_clearance{0.0};
        double safe_distance{0.0};
        double whole_body_min_margin{std::numeric_limits<double>::quiet_NaN()};
        ReactiveTaskExecutionContext* exec_ctx{nullptr};
        rq::ReactiveQpBuildConfig base_qp_build_cfg;
        rq::ReactiveQpBuildInput qp_input;
        ArmState arm_state;
        const arm_controller::kinematics::ForwardKinematicsOutput::LinkPoseMap*
            link_poses{nullptr};
        const rq::LinkCollisionEllipsoidList* collision_ellipsoids{nullptr};
        arm_controller::kinematics::JacobianProvider* jacobian_provider{nullptr};
        std::shared_ptr<const cp::DistanceFieldInterface> map;
        const ReactiveTaskTerminalPolicy::PhaseFlags* phase_flags{nullptr};
        rclcpp::Logger logger{rclcpp::get_logger("reactive_task_safety_policy")};
        rclcpp::Clock::SharedPtr clock;
        std::string mapping;
    };

    struct Output {
        rq::ReactiveQpBuildInput qp_input;
        rq::ReactiveQpBuildConfig qp_build_cfg;
        double obstacle_guidance_gate{0.0};
        double obstacle_min_distance{std::numeric_limits<double>::quiet_NaN()};
        double active_safety_distance{0.0};
    };

    bool apply(const Input& input, Output* output) const;

private:
    static double minFiniteObstacleDistance(
        const rq::ObstacleConstraintInputList& constraints);
};

}  // namespace arm_controller::controller::reactive_task
