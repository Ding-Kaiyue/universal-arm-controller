#include "chassis_controller/config/chassis_controller_config.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace chassis_controller {
namespace {

std::vector<double> declareDoubleArray(rclcpp::Node& node, const std::string& name,
                                       const std::vector<double>& default_value) {
    return node.declare_parameter<std::vector<double>>(name, default_value);
}

std::vector<std::string> declareStringArray(rclcpp::Node& node, const std::string& name,
                                            const std::vector<std::string>& default_value) {
    return node.declare_parameter<std::vector<std::string>>(name, default_value);
}

std::vector<double> expandScalars(const std::vector<double>& values, const std::size_t size,
                                  const double default_value) {
    if (values.empty()) {
        return std::vector<double>(size, default_value);
    }
    if (values.size() == 1 && size > 1) {
        return std::vector<double>(size, values.front());
    }
    return values;
}

void requireSize(const std::string& name, const std::size_t actual, const std::size_t expected) {
    if (actual != expected) {
        throw std::runtime_error(name + " size mismatch: expected " + std::to_string(expected) +
                                 ", got " + std::to_string(actual));
    }
}

std::vector<ChassisModuleGeometry> loadModuleGeometry(rclcpp::Node& node) {
    const std::vector<std::string> module_names = declareStringArray(node, "module_names", {});
    const std::size_t module_count = module_names.size();

    if (module_count == 0) {
        return {};
    }

    const std::vector<std::string> steering_joints =
        declareStringArray(node, "steering_joints", std::vector<std::string>(module_count, ""));
    const std::vector<std::string> wheel_joints =
        declareStringArray(node, "wheel_joints", std::vector<std::string>(module_count, ""));
    const std::vector<double> module_x = declareDoubleArray(node, "module_x", {});
    const std::vector<double> module_y = declareDoubleArray(node, "module_y", {});
    const std::vector<double> drive_direction_x =
        expandScalars(declareDoubleArray(node, "drive_direction_x", {1.0}), module_count, 1.0);
    const std::vector<double> drive_direction_y =
        expandScalars(declareDoubleArray(node, "drive_direction_y", {0.0}), module_count, 0.0);
    const std::vector<double> steering_axis_sign =
        expandScalars(declareDoubleArray(node, "steering_axis_sign", {1.0}), module_count, 1.0);
    const std::vector<double> wheel_axis_sign =
        expandScalars(declareDoubleArray(node, "wheel_axis_sign", {1.0}), module_count, 1.0);

    requireSize("steering_joints", steering_joints.size(), module_count);
    requireSize("wheel_joints", wheel_joints.size(), module_count);
    requireSize("module_x", module_x.size(), module_count);
    requireSize("module_y", module_y.size(), module_count);
    requireSize("drive_direction_x", drive_direction_x.size(), module_count);
    requireSize("drive_direction_y", drive_direction_y.size(), module_count);
    requireSize("steering_axis_sign", steering_axis_sign.size(), module_count);
    requireSize("wheel_axis_sign", wheel_axis_sign.size(), module_count);

    std::vector<ChassisModuleGeometry> modules;
    modules.reserve(module_count);
    for (std::size_t i = 0; i < module_count; ++i) {
        modules.push_back(ChassisModuleGeometry{
            module_names[i],
            steering_joints[i],
            wheel_joints[i],
            module_x[i],
            module_y[i],
            drive_direction_x[i],
            drive_direction_y[i],
            steering_axis_sign[i],
            wheel_axis_sign[i],
        });
    }

    return modules;
}

}  // namespace

ChassisControllerConfig loadChassisControllerConfig(rclcpp::Node& node) {
    ChassisControllerConfig config;

    const std::string chassis_type = node.declare_parameter<std::string>("chassis_type", "fixed");
    config.model.type = chassisTypeFromString(chassis_type);
    config.model.name = chassis_type;

    config.input_topic = node.declare_parameter<std::string>("input_topic", config.input_topic);
    config.output_topic = node.declare_parameter<std::string>("output_topic", config.output_topic);
    config.joint_command_topic =
        node.declare_parameter<std::string>("joint_command_topic", config.joint_command_topic);
    config.publish_twist = node.declare_parameter<bool>("publish_twist", config.publish_twist);
    config.publish_joint_commands =
        node.declare_parameter<bool>("publish_joint_commands", config.publish_joint_commands);
    config.warn_on_unsupported_lateral = node.declare_parameter<bool>(
        "warn_on_unsupported_lateral", config.warn_on_unsupported_lateral);
    config.kinematics.model = config.model;
    config.kinematics.wheel_radius =
        node.declare_parameter<double>("wheel_radius", config.kinematics.wheel_radius);
    config.kinematics.track_width =
        node.declare_parameter<double>("track_width", config.kinematics.track_width);
    config.kinematics.wheel_base =
        node.declare_parameter<double>("wheel_base", config.kinematics.wheel_base);
    config.kinematics.modules = loadModuleGeometry(node);

    return config;
}

}  // namespace chassis_controller
