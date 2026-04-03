#include "point_jacobian_provider.hpp"

namespace arm_controller::algorithm::reactive_qp {

KinematicsPointJacobianProvider::KinematicsPointJacobianProvider(
    std::shared_ptr<arm_controller::kinematics::JacobianProvider> provider)
    : provider_(std::move(provider)) {}

Eigen::MatrixXd KinematicsPointJacobianProvider::computePointJacobian(
    const Eigen::VectorXd& q,
    const std::string& link_name,
    const Eigen::Vector3d& point_in_link) const {
    if (!provider_) {
        return Eigen::MatrixXd{};
    }
    return provider_->computeJacobian(q, link_name, point_in_link);
}

}  // namespace arm_controller::algorithm::reactive_qp
