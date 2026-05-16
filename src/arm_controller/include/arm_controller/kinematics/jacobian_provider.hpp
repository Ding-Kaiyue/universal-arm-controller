#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"

namespace arm_controller::kinematics {

class JacobianProvider {
public:
    virtual ~JacobianProvider() = default;

    virtual bool initialize() = 0;

    virtual Eigen::MatrixXd computeJacobian(
        const Eigen::VectorXd& q,
        const std::string& link_name,
        const Eigen::Vector3d& point_in_link) const = 0;
};

class MoveItJacobianProvider final : public JacobianProvider {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MoveItJacobianProvider(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>& moveit_adapter);

    bool initialize() override;

    Eigen::MatrixXd computeJacobian(
        const Eigen::VectorXd& q,
        const std::string& link_name,
        const Eigen::Vector3d& point_in_link) const override;

private:
    const moveit::core::LinkModel* resolveLinkModel(const std::string& link_name) const;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter> moveit_adapter_;

    mutable std::mutex cache_mutex_;
    mutable bool initialized_{false};
    mutable moveit::core::RobotModelPtr robot_model_;
    mutable std::shared_ptr<moveit::core::RobotState> robot_state_;
    mutable const moveit::core::JointModelGroup* joint_model_group_{nullptr};
    mutable const moveit::core::LinkModel* default_ee_link_model_{nullptr};
    mutable std::unordered_map<std::string, const moveit::core::LinkModel*> link_cache_;
};

class PinocchioJacobianProvider final : public JacobianProvider {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PinocchioJacobianProvider(
        const rclcpp::Node::SharedPtr& node,
        const pinocchio::Model& model,
        const std::vector<int>& q_indices,
        const std::vector<int>& v_indices,
        pinocchio::FrameIndex default_frame);

    bool initialize() override;

    Eigen::MatrixXd computeJacobian(
        const Eigen::VectorXd& q,
        const std::string& link_name,
        const Eigen::Vector3d& point_in_link) const override;

private:
    bool resolveFrameId(
        const std::string& link_name,
        pinocchio::FrameIndex& frame_id) const;

private:
    rclcpp::Node::SharedPtr node_;
    pinocchio::Model model_;
    std::vector<int> q_indices_;
    std::vector<int> v_indices_;
    pinocchio::FrameIndex default_frame_{0};

    mutable std::mutex data_mutex_;
    mutable bool initialized_{false};
    mutable std::unique_ptr<pinocchio::Data> data_;
};

}  // namespace arm_controller::kinematics
