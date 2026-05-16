#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arm_controller::kinematics {

struct ForwardKinematicsOutput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using LinkPoseMap = std::unordered_map<
        std::string,
        Eigen::Isometry3d,
        std::hash<std::string>,
        std::equal_to<std::string>,
        Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>>;

    // World/base pose of each link frame (BODY frames in Pinocchio model).
    LinkPoseMap link_poses;

    // End-effector pose in world/base frame.
    Eigen::Isometry3d ee_pose = Eigen::Isometry3d::Identity();
    Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d ee_rotation = Eigen::Matrix3d::Identity();
};

struct LinkPoseResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string link_name;
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};

using LinkPoseResultList =
    std::vector<LinkPoseResult, Eigen::aligned_allocator<LinkPoseResult>>;

class PinocchioForwardKinematics final {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PinocchioForwardKinematics(
        const rclcpp::Node::SharedPtr& node,
        const pinocchio::Model& model,
        const std::vector<int>& q_indices,
        pinocchio::FrameIndex default_ee_frame);

    bool initialize();

    // Compute all link poses (BODY frames) and end-effector pose.
    bool compute(
        const Eigen::VectorXd& q,
        ForwardKinematicsOutput& out,
        const std::string& ee_link_name = "") const;

    bool computeLinkPoses(
        const Eigen::VectorXd& q,
        const std::vector<std::string>& link_names,
        LinkPoseResultList& link_poses,
        Eigen::Vector3d* ee_position = nullptr,
        Eigen::Matrix3d* ee_rotation = nullptr,
        Eigen::Isometry3d* ee_pose = nullptr,
        const std::string& ee_link_name = "") const;

private:
    bool resolveFrameId(
        const std::string& link_name,
        pinocchio::FrameIndex& frame_id) const;

private:
    rclcpp::Node::SharedPtr node_;
    pinocchio::Model model_;
    std::vector<int> q_indices_;
    pinocchio::FrameIndex default_ee_frame_{0};

    mutable std::mutex data_mutex_;
    mutable bool initialized_{false};
    mutable std::unique_ptr<pinocchio::Data> data_;
};

}  // namespace arm_controller::kinematics
