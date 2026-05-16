#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>
#include <yaml-cpp/yaml.h>

#include "algorithm/neo/body_obstacle_constraint_builder.hpp"

namespace arm_controller::algorithm::sphere_model {

class LinkSphereModel {
public:
    // Build link-sphere model for a mapping from hardware_config.yaml.
    //
    // Priority:
    //   1) YAML explicit spheres under hardware.<mapping>.collision_spheres
    //   2) built-in preset by robot_type (REMANI-style)
    static bool buildForMapping(
        const std::string& hardware_config_path,
        const std::string& mapping,
        const pinocchio::Model& model,
        arm_controller::algorithm::reactive_qp::LinkCollisionSphereList& out_spheres,
        std::string* error = nullptr);

    // Build link-ellipsoid model for whole-body collision constraints.
    // YAML field:
    //   collision_spheres:
    //     - link_name: xxx
    //       center: [x, y, z]
    //       # Either:
    //       radius: r
    //       # Or:
    //       radii: [rx, ry, rz]
    static bool buildEllipsoidsForMapping(
        const std::string& hardware_config_path,
        const std::string& mapping,
        const pinocchio::Model& model,
        arm_controller::algorithm::reactive_qp::LinkCollisionEllipsoidList& out_ellipsoids,
        std::string* error = nullptr);

private:
    static bool buildFromYamlExplicit(
        const std::string& mapping,
        const pinocchio::Model& model,
        const YAML::Node& mapping_node,
        arm_controller::algorithm::reactive_qp::LinkCollisionSphereList& out_spheres,
        std::string* error);

    static bool buildEllipsoidsFromYamlExplicit(
        const std::string& mapping,
        const pinocchio::Model& model,
        const YAML::Node& mapping_node,
        arm_controller::algorithm::reactive_qp::LinkCollisionEllipsoidList& out_ellipsoids,
        std::string* error);

    static bool buildFromPreset(
        const std::string& robot_type,
        const std::vector<std::string>& joint_names,
        const pinocchio::Model& model,
        arm_controller::algorithm::reactive_qp::LinkCollisionSphereList& out_spheres,
        std::string* error);

    static bool buildEllipsoidsFromPreset(
        const std::string& robot_type,
        const std::vector<std::string>& joint_names,
        const pinocchio::Model& model,
        arm_controller::algorithm::reactive_qp::LinkCollisionEllipsoidList& out_ellipsoids,
        std::string* error);
};

}  // namespace arm_controller::algorithm::sphere_model
