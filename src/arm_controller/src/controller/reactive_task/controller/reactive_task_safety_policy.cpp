#include "controller/reactive_task/controller/reactive_task_safety_policy.hpp"

#include <algorithm>
#include <limits>

namespace arm_controller::controller::reactive_task {

bool ReactiveTaskSafetyPolicy::apply(const Input& input, Output* output) const {
    if (output == nullptr) {
        return false;
    }

    *output = Output{};
    output->qp_input = input.qp_input;
    output->qp_build_cfg = input.base_qp_build_cfg;

    double obstacle_min_distance = std::numeric_limits<double>::quiet_NaN();
    const bool obstacle_constraints_active = input.enable_obstacle_constraints;

    if (obstacle_constraints_active && input.link_poses != nullptr &&
        input.collision_ellipsoids != nullptr && input.jacobian_provider != nullptr && input.map) {
        std::string obstacle_error;
        const int generated = rq::BodyObstacleConstraintBuilder::appendLinkEllipsoidConstraints(
            input.arm_state.q,
            *input.link_poses,
            *input.collision_ellipsoids,
            *input.jacobian_provider,
            input.map,
            output->qp_input.obstacle_constraints,
            &obstacle_error);
        if (generated <= 0) {
            output->qp_input.obstacle_constraints.clear();
            if (input.clock) {
                RCLCPP_WARN_THROTTLE(
                    input.logger,
                    *input.clock,
                    2000,
                    "[%s] safety: obstacle constraints skipped at tick %d; no obstacle damper this cycle: %s",
                    input.mapping.c_str(),
                    input.planner_tick,
                    obstacle_error.c_str());
            }
        }
        obstacle_min_distance = minFiniteObstacleDistance(output->qp_input.obstacle_constraints);
    }

    if (output->qp_input.obstacle_constraints.size() > 3) {
        std::sort(
            output->qp_input.obstacle_constraints.begin(),
            output->qp_input.obstacle_constraints.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.distance < rhs.distance; });
        output->qp_input.obstacle_constraints.resize(3);
        obstacle_min_distance = minFiniteObstacleDistance(output->qp_input.obstacle_constraints);
    }

    double obstacle_guidance_gate_raw = 0.0;
    if (std::isfinite(obstacle_min_distance)) {
        const double obstacle_influence_distance =
            std::max(input.safe_distance, output->qp_build_cfg.obstacle_damper.influence_distance);
        const double obstacle_guidance_den =
            std::max(1e-6, obstacle_influence_distance - input.safe_distance);
        obstacle_guidance_gate_raw = std::clamp(
            (obstacle_influence_distance - obstacle_min_distance) / obstacle_guidance_den,
            0.0,
            1.0);
    }

    const double obstacle_guidance_gate = obstacle_guidance_gate_raw;
    const double active_safety_distance = input.safe_distance;
    output->qp_build_cfg.obstacle_damper.safety_distance = active_safety_distance;

    output->obstacle_guidance_gate = obstacle_guidance_gate;
    output->obstacle_min_distance = obstacle_min_distance;
    output->active_safety_distance = active_safety_distance;
    return true;
}

double ReactiveTaskSafetyPolicy::minFiniteObstacleDistance(
    const rq::ObstacleConstraintInputList& constraints) {
    double min_distance = std::numeric_limits<double>::infinity();
    for (const auto& constraint : constraints) {
        if (std::isfinite(constraint.distance)) {
            min_distance = std::min(min_distance, constraint.distance);
        }
    }
    if (!std::isfinite(min_distance)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return min_distance;
}

}  // namespace arm_controller::controller::reactive_task
