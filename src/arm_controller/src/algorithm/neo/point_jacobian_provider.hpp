#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>

#include "arm_controller/kinematics/jacobian_provider.hpp"

namespace arm_controller::algorithm::reactive_qp {

class PointJacobianProvider {
public:
    virtual ~PointJacobianProvider() = default;

    virtual Eigen::MatrixXd computePointJacobian(
        const Eigen::VectorXd& q,
        const std::string& link_name,
        const Eigen::Vector3d& point_in_link) const = 0;
};

class KinematicsPointJacobianProvider final : public PointJacobianProvider {
public:
    explicit KinematicsPointJacobianProvider(
        std::shared_ptr<arm_controller::kinematics::JacobianProvider> provider);

    Eigen::MatrixXd computePointJacobian(
        const Eigen::VectorXd& q,
        const std::string& link_name,
        const Eigen::Vector3d& point_in_link) const override;

private:
    std::shared_ptr<arm_controller::kinematics::JacobianProvider> provider_;
};

}  // namespace arm_controller::algorithm::reactive_qp
