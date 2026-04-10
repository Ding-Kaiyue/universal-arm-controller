#include "algorithm/sphere_model/link_sphere_model.hpp"

#include <algorithm>
#include <sstream>
#include <unordered_map>

namespace arm_controller::algorithm::sphere_model {

namespace {

using LinkSphere = arm_controller::algorithm::reactive_qp::LinkCollisionSphere;
using LinkEllipsoid = arm_controller::algorithm::reactive_qp::LinkCollisionEllipsoid;

std::string resolveBodyFrameForJoint(
    const pinocchio::Model& model,
    const std::string& joint_name) {
    if (!model.existJointName(joint_name)) {
        return "";
    }
    const pinocchio::JointIndex jid = model.getJointId(joint_name);
    for (pinocchio::FrameIndex fid = 0; fid < model.frames.size(); ++fid) {
        const auto& frame = model.frames[fid];
        if (frame.type == pinocchio::BODY && frame.parentJoint == jid) {
            return frame.name;
        }
    }
    return "";
}

void addSpheresForLink(
    const std::string& link_name,
    const std::vector<Eigen::Vector3d>& centers,
    double radius,
    std::vector<LinkSphere>& out_spheres) {
    for (std::size_t i = 0; i < centers.size(); ++i) {
        LinkSphere s;
        s.link_name = link_name;
        s.center_in_link = centers[i];
        s.radius = radius;
        s.debug_name = "sphere_" + link_name + "_" + std::to_string(i);
        out_spheres.push_back(std::move(s));
    }
}

void addEllipsoidsForLink(
    const std::string& link_name,
    const std::vector<Eigen::Vector3d>& centers,
    const Eigen::Vector3d& radii,
    std::vector<LinkEllipsoid>& out_ellipsoids) {
    for (std::size_t i = 0; i < centers.size(); ++i) {
        LinkEllipsoid e;
        e.link_name = link_name;
        e.center_in_link = centers[i];
        e.radii = radii;
        e.debug_name = "ellipsoid_" + link_name + "_" + std::to_string(i);
        out_ellipsoids.push_back(std::move(e));
    }
}

}  // namespace

bool LinkSphereModel::buildForMapping(
    const std::string& hardware_config_path,
    const std::string& mapping,
    const pinocchio::Model& model,
    std::vector<LinkSphere>& out_spheres,
    std::string* error) {
    out_spheres.clear();
    YAML::Node root;
    try {
        root = YAML::LoadFile(hardware_config_path);
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = std::string("Load hardware config failed: ") + e.what();
        }
        return false;
    }

    const YAML::Node hw = root["hardware"];
    if (!hw || !hw.IsMap()) {
        if (error != nullptr) {
            *error = "Missing 'hardware' map in hardware config.";
        }
        return false;
    }
    const YAML::Node mapping_node = hw[mapping];
    if (!mapping_node || !mapping_node.IsMap()) {
        if (error != nullptr) {
            *error = "Mapping '" + mapping + "' not found in hardware config.";
        }
        return false;
    }

    if (buildFromYamlExplicit(mapping, model, mapping_node, out_spheres, error)) {
        return true;
    }

    const std::string robot_type =
        mapping_node["robot_type"] ? mapping_node["robot_type"].as<std::string>() : "";
    std::vector<std::string> joint_names;
    if (mapping_node["joint_names"] && mapping_node["joint_names"].IsSequence()) {
        for (const auto& jn : mapping_node["joint_names"]) {
            joint_names.push_back(jn.as<std::string>());
        }
    }

    return buildFromPreset(robot_type, joint_names, model, out_spheres, error);
}

bool LinkSphereModel::buildEllipsoidsForMapping(
    const std::string& hardware_config_path,
    const std::string& mapping,
    const pinocchio::Model& model,
    std::vector<LinkEllipsoid>& out_ellipsoids,
    std::string* error) {
    out_ellipsoids.clear();
    YAML::Node root;
    try {
        root = YAML::LoadFile(hardware_config_path);
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = std::string("Load hardware config failed: ") + e.what();
        }
        return false;
    }

    const YAML::Node hw = root["hardware"];
    if (!hw || !hw.IsMap()) {
        if (error != nullptr) {
            *error = "Missing 'hardware' map in hardware config.";
        }
        return false;
    }
    const YAML::Node mapping_node = hw[mapping];
    if (!mapping_node || !mapping_node.IsMap()) {
        if (error != nullptr) {
            *error = "Mapping '" + mapping + "' not found in hardware config.";
        }
        return false;
    }

    if (buildEllipsoidsFromYamlExplicit(mapping, model, mapping_node, out_ellipsoids, error)) {
        return true;
    }

    const std::string robot_type =
        mapping_node["robot_type"] ? mapping_node["robot_type"].as<std::string>() : "";
    std::vector<std::string> joint_names;
    if (mapping_node["joint_names"] && mapping_node["joint_names"].IsSequence()) {
        for (const auto& jn : mapping_node["joint_names"]) {
            joint_names.push_back(jn.as<std::string>());
        }
    }

    return buildEllipsoidsFromPreset(robot_type, joint_names, model, out_ellipsoids, error);
}

bool LinkSphereModel::buildFromYamlExplicit(
    const std::string& mapping,
    const pinocchio::Model& model,
    const YAML::Node& mapping_node,
    std::vector<LinkSphere>& out_spheres,
    std::string* error) {
    const YAML::Node explicit_spheres = mapping_node["collision_spheres"];
    if (!explicit_spheres || !explicit_spheres.IsSequence()) {
        return false;
    }

    for (std::size_t i = 0; i < explicit_spheres.size(); ++i) {
        const YAML::Node n = explicit_spheres[i];
        if (!n.IsMap() || !n["link_name"] || !n["center"]) {
            continue;
        }
        LinkSphere s;
        s.link_name = n["link_name"].as<std::string>();
        const auto c = n["center"];
        if (!c.IsSequence() || c.size() != 3) {
            continue;
        }
        s.center_in_link = Eigen::Vector3d(c[0].as<double>(), c[1].as<double>(), c[2].as<double>());
        if (n["radius"]) {
            s.radius = n["radius"].as<double>();
        } else if (n["radii"] && n["radii"].IsSequence() && n["radii"].size() == 3) {
            const YAML::Node r = n["radii"];
            s.radius = std::max({r[0].as<double>(), r[1].as<double>(), r[2].as<double>()});
        } else {
            continue;
        }
        s.debug_name = n["debug_name"] ? n["debug_name"].as<std::string>()
                                       : ("sphere_" + s.link_name + "_" + std::to_string(i));
        if (!model.existFrame(s.link_name)) {
            continue;
        }
        out_spheres.push_back(std::move(s));
    }

    if (out_spheres.empty()) {
        if (error != nullptr) {
            *error = "Mapping '" + mapping +
                     "' has 'collision_spheres' but none are valid.";
        }
        return false;
    }
    return true;
}

bool LinkSphereModel::buildEllipsoidsFromYamlExplicit(
    const std::string& mapping,
    const pinocchio::Model& model,
    const YAML::Node& mapping_node,
    std::vector<LinkEllipsoid>& out_ellipsoids,
    std::string* error) {
    const YAML::Node explicit_spheres = mapping_node["collision_spheres"];
    if (!explicit_spheres || !explicit_spheres.IsSequence()) {
        return false;
    }

    for (std::size_t i = 0; i < explicit_spheres.size(); ++i) {
        const YAML::Node n = explicit_spheres[i];
        if (!n.IsMap() || !n["link_name"] || !n["center"]) {
            continue;
        }
        LinkEllipsoid e;
        e.link_name = n["link_name"].as<std::string>();
        const auto c = n["center"];
        if (!c.IsSequence() || c.size() != 3) {
            continue;
        }
        e.center_in_link = Eigen::Vector3d(c[0].as<double>(), c[1].as<double>(), c[2].as<double>());
        if (n["radii"] && n["radii"].IsSequence() && n["radii"].size() == 3) {
            const YAML::Node r = n["radii"];
            e.radii = Eigen::Vector3d(r[0].as<double>(), r[1].as<double>(), r[2].as<double>());
        } else if (n["radius"]) {
            const double r = n["radius"].as<double>();
            e.radii = Eigen::Vector3d::Constant(r);
        } else {
            continue;
        }
        e.debug_name = n["debug_name"] ? n["debug_name"].as<std::string>()
                                       : ("ellipsoid_" + e.link_name + "_" + std::to_string(i));
        if (!model.existFrame(e.link_name)) {
            continue;
        }
        if (!(e.radii.x() > 0.0) || !(e.radii.y() > 0.0) || !(e.radii.z() > 0.0)) {
            continue;
        }
        out_ellipsoids.push_back(std::move(e));
    }

    if (out_ellipsoids.empty()) {
        if (error != nullptr) {
            *error = "Mapping '" + mapping +
                     "' has 'collision_spheres' but none are valid for ellipsoid model.";
        }
        return false;
    }
    return true;
}

bool LinkSphereModel::buildFromPreset(
    const std::string& robot_type,
    const std::vector<std::string>& joint_names,
    const pinocchio::Model& model,
    std::vector<LinkSphere>& out_spheres,
    std::string* error) {
    if (joint_names.empty()) {
        if (error != nullptr) {
            *error = "No joint_names for preset-based sphere model.";
        }
        return false;
    }

    // REMANI-style defaults: spheres sampled along each link local axis.
    // We use a generic preset for 6-DOF arms; can be overridden via YAML.
    const double r_default = (robot_type == "dual_arm620") ? 0.03 : 0.035;
    const std::vector<std::vector<Eigen::Vector3d>> preset_centers = {
        {Eigen::Vector3d(0.00, 0.00, 0.00), Eigen::Vector3d(0.00, 0.00, 0.05), Eigen::Vector3d(0.00, 0.00, -0.05)},
        {Eigen::Vector3d(0.00, 0.00, 0.00), Eigen::Vector3d(0.07, 0.00, 0.00), Eigen::Vector3d(0.14, 0.00, 0.00), Eigen::Vector3d(0.21, 0.00, 0.00)},
        {Eigen::Vector3d(0.00, 0.00, 0.00)},
        {Eigen::Vector3d(0.00, 0.07, 0.00), Eigen::Vector3d(0.00, 0.14, 0.00), Eigen::Vector3d(0.00, 0.21, 0.00)},
        {Eigen::Vector3d(0.00, 0.00, 0.00)},
        {Eigen::Vector3d(0.00, 0.00, -0.10), Eigen::Vector3d(0.00, 0.04, -0.05), Eigen::Vector3d(0.00, -0.04, -0.05), Eigen::Vector3d(0.00, 0.05, 0.00), Eigen::Vector3d(0.00, -0.05, 0.00)},
    };

    const std::size_t n = std::min(joint_names.size(), preset_centers.size());
    for (std::size_t i = 0; i < n; ++i) {
        const std::string link_name = resolveBodyFrameForJoint(model, joint_names[i]);
        if (link_name.empty()) {
            continue;
        }
        addSpheresForLink(link_name, preset_centers[i], r_default, out_spheres);
    }

    if (out_spheres.empty()) {
        if (error != nullptr) {
            *error = "Preset sphere model generation failed for robot_type='" + robot_type + "'.";
        }
        return false;
    }
    return true;
}

bool LinkSphereModel::buildEllipsoidsFromPreset(
    const std::string& robot_type,
    const std::vector<std::string>& joint_names,
    const pinocchio::Model& model,
    std::vector<LinkEllipsoid>& out_ellipsoids,
    std::string* error) {
    if (joint_names.empty()) {
        if (error != nullptr) {
            *error = "No joint_names for preset-based ellipsoid model.";
        }
        return false;
    }

    const Eigen::Vector3d r_default =
        (robot_type == "dual_arm620")
            ? Eigen::Vector3d(0.028, 0.028, 0.050)
            : Eigen::Vector3d(0.030, 0.030, 0.055);
    const std::vector<std::vector<Eigen::Vector3d>> preset_centers = {
        {Eigen::Vector3d(0.00, 0.00, 0.04)},
        {Eigen::Vector3d(0.00, 0.15, 0.00)},
        {Eigen::Vector3d(0.00, -0.08, 0.01)},
        {Eigen::Vector3d(0.00, 0.00, 0.04)},
        {Eigen::Vector3d(0.00, 0.05, 0.00)},
        {Eigen::Vector3d(0.00, 0.00, 0.00)},
    };

    const std::size_t n = std::min(joint_names.size(), preset_centers.size());
    for (std::size_t i = 0; i < n; ++i) {
        const std::string link_name = resolveBodyFrameForJoint(model, joint_names[i]);
        if (link_name.empty()) {
            continue;
        }
        addEllipsoidsForLink(link_name, preset_centers[i], r_default, out_ellipsoids);
    }

    if (out_ellipsoids.empty()) {
        if (error != nullptr) {
            *error = "Preset ellipsoid model generation failed for robot_type='" + robot_type + "'.";
        }
        return false;
    }
    return true;
}

}  // namespace arm_controller::algorithm::sphere_model
