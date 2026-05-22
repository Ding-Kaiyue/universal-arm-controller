#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include "algorithm/cartesian_path_planner/collision/whole_body_ellipsoid_collision_checker.hpp"
#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "controller/reactive_task/controller/reactive_task_types.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;

class ReactiveTaskEnvironmentProbe {
public:
    struct MapConfig {
        bool enable_dummy_obstacle{false};
        double dummy_obstacle_radius{0.0};
        Eigen::Vector3d dummy_obstacle_center_left_arm{Eigen::Vector3d::Zero()};
        Eigen::Vector3d dummy_obstacle_center_right_arm{Eigen::Vector3d::Zero()};
    };

    struct Input {
        std::string mapping;
        const cp::WholeBodyEllipsoidCollisionChecker* whole_body_validator{nullptr};
        const Eigen::VectorXd* q_now{nullptr};
        double safe_distance{0.0};
        std::shared_ptr<const cp::DistanceFieldInterface> map;
        Eigen::Vector3d ee_position{Eigen::Vector3d::Zero()};
        MapConfig map_config;
    };

    struct Output {
        WholeBodyStatusSnapshot whole_body_status{};
        double ee_clearance{std::numeric_limits<double>::quiet_NaN()};
        Eigen::Vector3d ee_escape_gradient{Eigen::Vector3d::Zero()};
        bool ee_escape_gradient_valid{false};
    };

    bool probe(const Input& input, Output* output) const;
};

}  // namespace arm_controller::controller::reactive_task
