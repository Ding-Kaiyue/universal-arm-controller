#include "controller/reactive_task/reactive_task_controller.hpp"

#include <fstream>
#include <iterator>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "algorithm/global_planner/ompl_rrt_connect_global_planner.hpp"
#include "algorithm/sphere_model/link_sphere_model.hpp"

namespace rq = arm_controller::algorithm::reactive_qp;
namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace gp = arm_controller::algorithm::global_planner;

namespace {
struct RobotDescriptionPaths {
    std::string urdf_path;
    std::string srdf_path;
};

std::string normalizeArmTypeForTracIk(const std::string& robot_type) {
    if (robot_type == "dual_arm620" || robot_type == "dual_arm380") {
        return robot_type;
    }
    if (robot_type.find("620") != std::string::npos) {
        return "arm620";
    }
    if (robot_type.find("380") != std::string::npos) {
        return "arm380";
    }
    return robot_type.empty() ? "arm620" : robot_type;
}

RobotDescriptionPaths resolveRobotDescriptionPaths(const std::string& robot_type) {
    RobotDescriptionPaths paths;
    if (robot_type == "simple_omni_dual_arm" ||
        robot_type == "simple_omni_dual_arm_gazebo") {
        paths.urdf_path =
            ament_index_cpp::get_package_share_directory("whole_body_description") +
            "/urdf/simple_omni_dual_arm.urdf";
        paths.srdf_path =
            ament_index_cpp::get_package_share_directory("whole_body_config") +
            "/config/simple_omni_dual_arm.srdf";
        return paths;
    }

    paths.urdf_path =
        ament_index_cpp::get_package_share_directory("robot_description") +
        "/urdf/" + robot_type + ".urdf";
    try {
        paths.srdf_path =
            ament_index_cpp::get_package_share_directory(robot_type + "_config") +
            "/config/" + robot_type + ".srdf";
    } catch (const std::exception&) {
        paths.srdf_path.clear();
    }
    return paths;
}
}  // namespace

bool ReactiveTaskController::initializeMappingContext(const std::string& mapping, std::string* error) {
    std::lock_guard<std::mutex> lock(mapping_contexts_mutex_);
    auto& ctx = mapping_contexts_[mapping];
    if (ctx.initialized) {
        return true;
    }

    ctx.joint_names = hardware_manager_->get_joint_names(mapping);
    if (ctx.joint_names.empty()) {
        if (error != nullptr) {
            *error = "joint_names is empty";
        }
        return false;
    }

    ctx.robot_type = hardware_manager_->get_robot_type(mapping);
    if (ctx.robot_type.empty()) {
        if (error != nullptr) {
            *error = "robot_type is empty";
        }
        return false;
    }

    RobotDescriptionPaths description_paths;
    try {
        description_paths = resolveRobotDescriptionPaths(ctx.robot_type);
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = std::string("resolve robot description failed: ") + e.what();
        }
        return false;
    }

    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(description_paths.urdf_path, model);
        ctx.urdf_path = description_paths.urdf_path;
        ctx.srdf_path = description_paths.srdf_path;
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = std::string("build pinocchio model failed: ") + e.what();
        }
        return false;
    }

    std::vector<int> q_indices;
    std::vector<int> v_indices;
    q_indices.reserve(ctx.joint_names.size());
    v_indices.reserve(ctx.joint_names.size());
    for (const auto& joint_name : ctx.joint_names) {
        if (!model.existJointName(joint_name)) {
            if (error != nullptr) {
                *error = "joint not found in urdf: " + joint_name;
            }
            return false;
        }
        const auto jid = model.getJointId(joint_name);
        q_indices.push_back(static_cast<int>(model.joints[jid].idx_q()));
        v_indices.push_back(static_cast<int>(model.joints[jid].idx_v()));
    }

    pinocchio::FrameIndex ee_frame = 0;
    const auto last_joint = model.getJointId(ctx.joint_names.back());
    for (pinocchio::FrameIndex fid = 0; fid < model.frames.size(); ++fid) {
        if (model.frames[fid].parentJoint == last_joint) {
            ee_frame = fid;
        }
    }

    ctx.fk_provider = std::make_shared<arm_controller::kinematics::PinocchioForwardKinematics>(
        node_, model, q_indices, ee_frame);
    if (!ctx.fk_provider->initialize()) {
        if (error != nullptr) {
            *error = "fk provider initialize failed";
        }
        return false;
    }

    ctx.jacobian_provider = std::make_shared<arm_controller::kinematics::PinocchioJacobianProvider>(
        node_, model, q_indices, v_indices, ee_frame);
    if (!ctx.jacobian_provider->initialize()) {
        if (error != nullptr) {
            *error = "jacobian provider initialize failed";
        }
        return false;
    }

    ctx.manipulability_gradient =
        std::make_unique<rq::ManipulabilityGradient>(ctx.jacobian_provider);

    try {
        ctx.planning_group = hardware_manager_->get_planning_group(mapping);
        if (!ctx.planning_group.empty()) {
            ctx.moveit_adapter =
                std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, ctx.planning_group, "reactive_task");
            ctx.tracik_adapter =
                std::make_shared<trajectory_planning::infrastructure::integration::TracIKAdapter>(
                    node_, ctx.planning_group);

            if (ctx.moveit_adapter && ctx.tracik_adapter) {
                ctx.tracik_adapter->setMoveItAdapter(ctx.moveit_adapter.get());

                std::string urdf_xml = ctx.moveit_adapter->getURDFString(ctx.robot_type);
                if (urdf_xml.empty()) {
                    std::ifstream ifs(ctx.urdf_path);
                    urdf_xml.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
                }

                ctx.base_link = ctx.moveit_adapter->getBaseLink();
                ctx.tip_link = ctx.moveit_adapter->getEndEffectorLink();
                const bool kdl_ok =
                    !ctx.base_link.empty() && !ctx.tip_link.empty() &&
                    ctx.tracik_adapter->initializeKDLChain(urdf_xml, ctx.base_link, ctx.tip_link);
                const bool solver_ok =
                    kdl_ok &&
                    ctx.tracik_adapter->initializeSolver(normalizeArmTypeForTracIk(ctx.robot_type));
                ctx.tracik_ready = (kdl_ok && solver_ok);
            }
        }
    } catch (const std::exception&) {
        ctx.tracik_ready = false;
    }

    const std::string reactive_cfg_path =
        ament_index_cpp::get_package_share_directory("arm_controller") +
        "/config/reactive_task_config.yaml";
    std::string pref_error;
    if (!rq::JointPreferenceLoader::loadFromYaml(
            reactive_cfg_path, ctx.joint_names, ctx.joint_preference_cfg, &pref_error)) {
        if (error != nullptr) {
            *error = "load joint preference failed: " + pref_error;
        }
        return false;
    }

    const std::string hardware_cfg_path =
        ament_index_cpp::get_package_share_directory("arm_controller") + "/config/hardware_config.yaml";
    std::string collision_error;
    if (!arm_controller::algorithm::sphere_model::LinkSphereModel::buildEllipsoidsForMapping(
            hardware_cfg_path, mapping, model, ctx.collision_ellipsoids, &collision_error)) {
        if (error != nullptr) {
            *error = "build collision ellipsoids failed: " + collision_error;
        }
        return false;
    }
    if (ctx.collision_ellipsoids.empty()) {
        if (error != nullptr) {
            *error = "collision ellipsoid model is empty";
        }
        return false;
    }

    const int dof = static_cast<int>(ctx.joint_names.size());
    ctx.qd_min = Eigen::VectorXd::Zero(dof);
    ctx.qd_max = Eigen::VectorXd::Zero(dof);
    ctx.joint_limits.q_min = Eigen::VectorXd::Zero(dof);
    ctx.joint_limits.q_max = Eigen::VectorXd::Zero(dof);

    for (int i = 0; i < dof; ++i) {
        JointLimits limits;
        hardware_manager_->get_joint_limits(ctx.joint_names[static_cast<std::size_t>(i)], limits);
        ctx.joint_limits.q_min(i) = limits.min_position;
        ctx.joint_limits.q_max(i) = limits.max_position;
        const double vmax = (limits.has_velocity_limits && limits.max_velocity > 1e-6)
                                ? limits.max_velocity
                                : 1.0;
        ctx.qd_min(i) = -vmax;
        ctx.qd_max(i) = vmax;
    }

    ctx.initialized = true;
    return true;
}

std::shared_ptr<gp::GlobalPlannerInterface> ReactiveTaskController::buildPlanner() const {
    return std::make_shared<gp::OmplRrtConnectGlobalPlanner>(
        runtime_cfg_.planner_common,
        runtime_cfg_.planner_smoothing);
}
