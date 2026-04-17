#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

#include "manipulability_gradient.hpp"
#include "reactive_qp_builder.hpp"
#include "task_velocity_generator.hpp"

namespace arm_controller::algorithm::reactive_qp {

struct HumanLikeJointPreference {
    std::string joint_name;
    bool enable_range_clamp{false};
    double preferred_min{0.0};
    double preferred_max{0.0};
    double preferred_center{0.0};
    double posture_weight{0.0};
};

struct HumanLikeJointPreferenceConfig {
    double posture_k{1.0};
    std::vector<HumanLikeJointPreference> joints;
};

class JointPreferenceLoader {
public:
    // YAML schema:
    // neo_joint_preference:
    //   posture_k: 1.0
    //   joints:
    //     <joint_name>:
    //       enable_range_clamp: true
    //       preferred_min: -1.0
    //       preferred_max:  1.0
    //       preferred_center: 0.0
    //       posture_weight: 0.5
    static bool loadFromYaml(
        const std::string& yaml_path,
        const std::vector<std::string>& joint_names,
        HumanLikeJointPreferenceConfig& out_config,
        std::string* error = nullptr);
};

struct ReactiveQpExampleConfig {
    TaskVelocityConfig task_velocity;
    Eigen::Vector3d target_translation{0.05, -0.02, 0.01};

    double obstacle_distance{0.12};

    ReactiveQpBuildConfig qp_build;
    ManipulabilityGradientConfig manipulability;
};

class ReactiveQpExampleConfigLoader {
public:
    // YAML schema:
    // neo_example:
    //   task_velocity: {...}
    //   target_translation_xyz: [x, y, z]
    //   enable_obstacle_damper: false
    //   obstacle_distance: 0.12
    //   qp_build: {...}
    //   manipulability: {...}
    static bool loadFromYaml(
        const std::string& yaml_path,
        ReactiveQpExampleConfig& out_config,
        std::string* error = nullptr);
};

}  // namespace arm_controller::algorithm::reactive_qp
