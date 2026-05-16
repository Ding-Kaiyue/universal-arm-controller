#include "controller/reactive_task/reactive_task_environment_probe.hpp"

#include <cmath>

namespace arm_controller::controller::reactive_task {

bool ReactiveTaskEnvironmentProbe::probe(
    const Input& input,
    Output* output) const {
    if (output == nullptr) {
        return false;
    }

    *output = Output{};
    output->whole_body_status.state =
        input.whole_body_validator != nullptr ? "ok" : "validator_unavailable";

    if (input.whole_body_validator != nullptr && input.q_now != nullptr) {
        const auto diag =
            input.whole_body_validator->diagnoseJointState(*input.q_now, input.safe_distance);
        output->whole_body_status.min_margin = diag.min_margin;
        output->whole_body_status.collision_free = diag.collision_free;
        output->whole_body_status.state = diag.reason;
        output->whole_body_status.worst_link = diag.worst_link_name;
        output->whole_body_status.worst_distance = diag.worst_distance;
        output->whole_body_status.worst_effective_radius = diag.worst_effective_radius;
        output->whole_body_status.required_clearance = diag.required_clearance;
        output->whole_body_status.safe_distance_used = diag.safe_distance_used;
        output->whole_body_status.worst_gradient_norm = diag.worst_gradient_norm;
        output->whole_body_status.worst_gradient_world = diag.worst_gradient_world;
        output->whole_body_status.worst_point_world = diag.worst_point_world;
        if (diag.worst_gradient_norm > 1e-6 &&
            std::isfinite(diag.worst_distance) &&
            diag.worst_point_world.allFinite() &&
            diag.worst_gradient_world.allFinite()) {
            const Eigen::Vector3d n_world =
                diag.worst_gradient_world / diag.worst_gradient_norm;
            output->whole_body_status.nearest_obstacle_point_world =
                diag.worst_point_world - diag.worst_distance * n_world;
            output->whole_body_status.nearest_obstacle_point_valid =
                output->whole_body_status.nearest_obstacle_point_world.allFinite();
        }
    }

    if (input.map != nullptr) {
        const auto query = input.map->queryDistanceAndGradient(input.ee_position);
        if (query.observed && query.distance_valid) {
            output->ee_clearance = query.distance - input.safe_distance;
        }
        if (query.observed && query.gradient_valid) {
            const double gradient_norm = query.gradient.norm();
            if (gradient_norm > 1e-6) {
                output->ee_escape_gradient = query.gradient / gradient_norm;
                output->ee_escape_gradient_valid = output->ee_escape_gradient.allFinite();
            }
        }
        return true;
    }

    if (input.map_config.enable_dummy_obstacle && input.map_config.dummy_obstacle_radius > 0.0) {
        const Eigen::Vector3d center = input.mapping == "right_arm"
                                           ? input.map_config.dummy_obstacle_center_right_arm
                                           : input.map_config.dummy_obstacle_center_left_arm;
        output->ee_clearance =
            (input.ee_position - center).norm() - input.map_config.dummy_obstacle_radius -
            input.safe_distance;
    }

    return true;
}

}  // namespace arm_controller::controller::reactive_task
