#include "joint_preference_loader.hpp"

#include <yaml-cpp/yaml.h>

namespace arm_controller::algorithm::reactive_qp {

namespace {

void setError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

void applyJointOverride(
    const YAML::Node& node,
    HumanLikeJointPreference& pref) {
    if (node["enable_range_clamp"]) {
        pref.enable_range_clamp = node["enable_range_clamp"].as<bool>();
    }
    if (node["preferred_min"]) {
        pref.preferred_min = node["preferred_min"].as<double>();
    }
    if (node["preferred_max"]) {
        pref.preferred_max = node["preferred_max"].as<double>();
    }
    if (node["preferred_center"]) {
        pref.preferred_center = node["preferred_center"].as<double>();
    }
    if (node["posture_weight"]) {
        pref.posture_weight = node["posture_weight"].as<double>();
    }
}

bool parseVec3(
    const YAML::Node& node,
    Eigen::Vector3d& out_v) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        return false;
    }
    out_v.x() = node[0].as<double>();
    out_v.y() = node[1].as<double>();
    out_v.z() = node[2].as<double>();
    return true;
}

}  // namespace

bool JointPreferenceLoader::loadFromYaml(
    const std::string& yaml_path,
    const std::vector<std::string>& joint_names,
    HumanLikeJointPreferenceConfig& out_config,
    std::string* error) {
    try {
        const YAML::Node root = YAML::LoadFile(yaml_path);
        const YAML::Node pref_root = root["neo_joint_preference"];
        if (!pref_root || !pref_root.IsMap()) {
            setError(error, "Missing 'neo_joint_preference' map in " + yaml_path);
            return false;
        }

        out_config.posture_k = pref_root["posture_k"] ? pref_root["posture_k"].as<double>() : 1.0;
        out_config.joints.clear();
        out_config.joints.reserve(joint_names.size());

        const YAML::Node joints_map = pref_root["joints"];
        if (!joints_map || !joints_map.IsMap()) {
            setError(error, "Missing 'neo_joint_preference.joints' map in " + yaml_path);
            return false;
        }

        for (const auto& joint_name : joint_names) {
            HumanLikeJointPreference pref;
            pref.joint_name = joint_name;
            if (const YAML::Node n = joints_map[joint_name]) {
                if (!n.IsMap()) {
                    setError(error, "Joint entry for '" + joint_name + "' must be a map.");
                    return false;
                }
                applyJointOverride(n, pref);
            }
            if (pref.preferred_min > pref.preferred_max) {
                setError(error, "Invalid preferred range for '" + joint_name + "': min > max.");
                return false;
            }
            out_config.joints.push_back(pref);
        }
        return true;
    } catch (const std::exception& e) {
        setError(error, e.what());
        return false;
    }
}

bool ReactiveQpExampleConfigLoader::loadFromYaml(
    const std::string& yaml_path,
    ReactiveQpExampleConfig& out_config,
    std::string* error) {
    try {
        const YAML::Node root = YAML::LoadFile(yaml_path);
        const YAML::Node ex = root["neo_example"];
        if (!ex || !ex.IsMap()) {
            setError(error, "Missing 'neo_example' map in " + yaml_path);
            return false;
        }

        if (const YAML::Node n = ex["task_velocity"]) {
            if (!n.IsMap()) {
                setError(error, "'neo_example.task_velocity' must be a map.");
                return false;
            }
            if (n["kp_pos"] && !parseVec3(n["kp_pos"], out_config.task_velocity.kp_pos)) {
                setError(error, "'neo_example.task_velocity.kp_pos' must be [x,y,z].");
                return false;
            }
            if (n["ko_ori"] && !parseVec3(n["ko_ori"], out_config.task_velocity.ko_ori)) {
                setError(error, "'neo_example.task_velocity.ko_ori' must be [x,y,z].");
                return false;
            }
            if (n["max_linear_speed"]) {
                out_config.task_velocity.max_linear_speed = n["max_linear_speed"].as<double>();
            }
            if (n["max_angular_speed"]) {
                out_config.task_velocity.max_angular_speed = n["max_angular_speed"].as<double>();
            }
            if (n["position_deadband"]) {
                out_config.task_velocity.position_deadband = n["position_deadband"].as<double>();
            }
            if (n["orientation_deadband"]) {
                out_config.task_velocity.orientation_deadband = n["orientation_deadband"].as<double>();
            }
        }

        if (ex["target_translation_xyz"] &&
            !parseVec3(ex["target_translation_xyz"], out_config.target_translation)) {
            setError(error, "'neo_example.target_translation_xyz' must be [x,y,z].");
            return false;
        }

        if (ex["obstacle_distance"]) {
            out_config.obstacle_distance = ex["obstacle_distance"].as<double>();
        }

        if (const YAML::Node n = ex["manipulability"]) {
            if (!n.IsMap()) {
                setError(error, "'neo_example.manipulability' must be a map.");
                return false;
            }
            if (n["finite_difference_step"]) {
                out_config.manipulability.finite_difference_step =
                    n["finite_difference_step"].as<double>();
            }
            if (n["determinant_damping"]) {
                out_config.manipulability.determinant_damping =
                    n["determinant_damping"].as<double>();
            }
            if (n["link_name"]) {
                out_config.manipulability.link_name = n["link_name"].as<std::string>();
            }
            if (n["point_in_link"] &&
                !parseVec3(n["point_in_link"], out_config.manipulability.point_in_link)) {
                setError(error, "'neo_example.manipulability.point_in_link' must be [x,y,z].");
                return false;
            }
        }

        if (const YAML::Node n = ex["qp_build"]) {
            if (!n.IsMap()) {
                setError(error, "'neo_example.qp_build' must be a map.");
                return false;
            }
            if (n["enable_joint_limit_damper"]) {
                out_config.qp_build.enable_joint_limit_damper =
                    n["enable_joint_limit_damper"].as<bool>();
            }
            if (n["enable_obstacle_damper"]) {
                out_config.qp_build.enable_obstacle_damper =
                    n["enable_obstacle_damper"].as<bool>();
            }
            if (n["slack_abs_bound"]) {
                out_config.qp_build.slack_abs_bound = n["slack_abs_bound"].as<double>();
            }

            if (const YAML::Node h = n["hessian"]) {
                if (!h.IsMap()) {
                    setError(error, "'neo_example.qp_build.hessian' must be a map.");
                    return false;
                }
                if (h["task_tracking_weight"]) {
                    out_config.qp_build.hessian.task_tracking_weight =
                        h["task_tracking_weight"].as<double>();
                }
                if (h["joint_velocity_weight"]) {
                    out_config.qp_build.hessian.joint_velocity_weight =
                        h["joint_velocity_weight"].as<double>();
                }
                if (h["slack_weight"]) {
                    out_config.qp_build.hessian.slack_weight = h["slack_weight"].as<double>();
                }
                if (h["shell_tracking_weight"]) {
                    out_config.qp_build.hessian.shell_tracking_weight =
                        h["shell_tracking_weight"].as<double>();
                }
                if (h["shell_centering_gain"]) {
                    out_config.qp_build.hessian.shell_centering_gain =
                        h["shell_centering_gain"].as<double>();
                }
                if (h["shell_target_velocity_limit"]) {
                    out_config.qp_build.hessian.shell_target_velocity_limit =
                        h["shell_target_velocity_limit"].as<double>();
                }
                if (h["qdot_smoothing_weight"]) {
                    out_config.qp_build.hessian.qdot_smoothing_weight =
                        h["qdot_smoothing_weight"].as<double>();
                }
                if (h["posture_weight"]) {
                    out_config.qp_build.hessian.posture_weight = h["posture_weight"].as<double>();
                }
                if (h["manipulability_weight"]) {
                    out_config.qp_build.hessian.manipulability_weight =
                        h["manipulability_weight"].as<double>();
                }
            }

            if (const YAML::Node j = n["joint_limit_damper"]) {
                if (!j.IsMap()) {
                    setError(error, "'neo_example.qp_build.joint_limit_damper' must be a map.");
                    return false;
                }
                if (j["safety_distance"]) {
                    out_config.qp_build.joint_limit_damper.safety_distance =
                        j["safety_distance"].as<double>();
                }
                if (j["influence_distance"]) {
                    out_config.qp_build.joint_limit_damper.influence_distance =
                        j["influence_distance"].as<double>();
                }
                if (j["cbf_gain_lower"]) {
                    out_config.qp_build.joint_limit_damper.cbf_gain_lower =
                        j["cbf_gain_lower"].as<double>();
                }
                if (j["cbf_gain_upper"]) {
                    out_config.qp_build.joint_limit_damper.cbf_gain_upper =
                        j["cbf_gain_upper"].as<double>();
                }
            }

            if (const YAML::Node o = n["obstacle_damper"]) {
                if (!o.IsMap()) {
                    setError(error, "'neo_example.qp_build.obstacle_damper' must be a map.");
                    return false;
                }
                if (o["safety_distance"]) {
                    out_config.qp_build.obstacle_damper.safety_distance =
                        o["safety_distance"].as<double>();
                }
                if (o["influence_distance"]) {
                    out_config.qp_build.obstacle_damper.influence_distance =
                        o["influence_distance"].as<double>();
                }
                if (o["cbf_gain"]) {
                    out_config.qp_build.obstacle_damper.cbf_gain = o["cbf_gain"].as<double>();
                }
            }
        }

        return true;
    } catch (const std::exception& e) {
        setError(error, e.what());
        return false;
    }
}

}  // namespace arm_controller::algorithm::reactive_qp
