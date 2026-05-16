#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace reactive_task_test {

inline std::string packageShareOrSourcePath(
    const std::string& package_name,
    const std::filesystem::path& source_relative_path) {
    try {
        return ament_index_cpp::get_package_share_directory(package_name);
    } catch (const std::exception&) {
        const std::filesystem::path arm_controller_root =
            std::filesystem::path(__FILE__).parent_path().parent_path();
        const std::filesystem::path workspace_src =
            arm_controller_root.parent_path();
        return (workspace_src / source_relative_path).string();
    }
}

struct RealArmModel {
    std::string robot_type{"dual_arm620"};
    std::string planning_group{"left_arm"};
    std::string base_link{"left_base_link"};
    std::string tip_link{"left_Link6"};
    std::string urdf_path;
    std::string srdf_path;
    std::vector<std::string> joint_names{
        "left_joint1", "left_joint2", "left_joint3",
        "left_joint4", "left_joint5", "left_joint6"};
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;
    pinocchio::Model model;
    std::vector<int> q_indices;
    pinocchio::FrameIndex tip_frame{0};

    static RealArmModel load() {
        RealArmModel arm;
        const std::string robot_desc_share = packageShareOrSourcePath(
            "robot_description", "trajectory_planning/robot_description");
        const std::string dual_arm_config_share = packageShareOrSourcePath(
            "dual_arm620_config",
            "trajectory_planning/robot_config/dual_arm620_config");
        arm.urdf_path = robot_desc_share + "/urdf/dual_arm620.urdf";
        arm.srdf_path = dual_arm_config_share + "/config/dual_arm620.srdf";

        pinocchio::urdf::buildModel(arm.urdf_path, arm.model);
        for (const auto& joint_name : arm.joint_names) {
            const auto joint_id = arm.model.getJointId(joint_name);
            arm.q_indices.push_back(static_cast<int>(arm.model.joints[joint_id].idx_q()));
        }
        arm.tip_frame = arm.model.getFrameId(arm.tip_link);
        arm.q_min = (Eigen::VectorXd(6) << -2.967, -1.5708, -1.5708, -2.967, -1.5708, -2.967).finished();
        arm.q_max = (Eigen::VectorXd(6) << 2.967, 1.5708, 1.5708, 2.967, 1.5708, 2.967).finished();
        return arm;
    }

    bool withinBounds(const Eigen::VectorXd& q) const {
        return q.size() == q_min.size() && q.allFinite() &&
               ((q.array() >= q_min.array() - 1e-9).all()) &&
               ((q.array() <= q_max.array() + 1e-9).all());
    }

    bool computePose(const Eigen::VectorXd& q, Eigen::Isometry3d* pose) const {
        if (pose == nullptr || q.size() != static_cast<Eigen::Index>(q_indices.size())) {
            return false;
        }
        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nq);
        for (Eigen::Index i = 0; i < q.size(); ++i) {
            q_full(q_indices[static_cast<std::size_t>(i)]) = q(i);
        }
        pinocchio::Data data(model);
        pinocchio::forwardKinematics(model, data, q_full);
        pinocchio::updateFramePlacements(model, data);
        const auto& placement = data.oMf[tip_frame];
        pose->setIdentity();
        pose->linear() = placement.rotation();
        pose->translation() = placement.translation();
        return pose->matrix().allFinite();
    }
};

inline Eigen::VectorXd defaultStartQ() {
    return (Eigen::VectorXd(6) << 0.0, -0.45, -0.55, 0.0, 0.45, 0.0).finished();
}

inline Eigen::VectorXd defaultGoalQ() {
    return (Eigen::VectorXd(6) << 0.30, -0.20, -0.90, 0.25, 0.70, 0.20).finished();
}

}  // namespace reactive_task_test

