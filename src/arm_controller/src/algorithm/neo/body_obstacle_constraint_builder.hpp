#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "arm_controller/kinematics/forward_kinematics.hpp"
#include "obstacle_damper.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"

namespace arm_controller::algorithm::reactive_qp {

struct LinkCollisionSphere {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string link_name;
    Eigen::Vector3d center_in_link{Eigen::Vector3d::Zero()};
    double radius{0.03};
    std::string debug_name;
};

struct LinkCollisionEllipsoid {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string link_name;
    Eigen::Vector3d center_in_link{Eigen::Vector3d::Zero()};
    // Half lengths in link-local x/y/z.
    Eigen::Vector3d radii{Eigen::Vector3d::Constant(0.03)};
    std::string debug_name;
};

struct DistanceQueryResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    double distance{0.0};
    Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
};

using LinkCollisionSphereList =
    std::vector<LinkCollisionSphere, Eigen::aligned_allocator<LinkCollisionSphere>>;
using LinkCollisionEllipsoidList =
    std::vector<LinkCollisionEllipsoid, Eigen::aligned_allocator<LinkCollisionEllipsoid>>;
using DistanceQueryResultList =
    std::vector<DistanceQueryResult, Eigen::aligned_allocator<DistanceQueryResult>>;

class BodyObstacleConstraintBuilder {
public:
    using DistanceFieldInterface =
        arm_controller::algorithm::cartesian_path_planner::DistanceFieldInterface;
    using LinkPoseMap = arm_controller::kinematics::ForwardKinematicsOutput::LinkPoseMap;

    // Build obstacle CBF inputs for all link spheres:
    //   h_i(q) = d_esdf(p_i(q)) - r_i - safety_distance >= 0
    //
    // For each sphere i, this function outputs:
    //   distance_i = d_esdf(p_i) - r_i
    //   normal_jacobian_i = n_i^T * Jv_i, where n_i is normalized ESDF gradient.
    static int appendLinkSphereConstraints(
        const Eigen::VectorXd& q_current,
        const LinkPoseMap& link_poses_world,
        const LinkCollisionSphereList& link_spheres,
        const arm_controller::kinematics::JacobianProvider& jacobian_provider,
        std::shared_ptr<const DistanceFieldInterface> distance_field,
        ObstacleConstraintInputList& out_constraints,
        std::string* error = nullptr);

    // Build obstacle CBF inputs for all link ellipsoids:
    //   h_i(q) = d_esdf(p_i(q)) - r_eff_i(n_i, q) - safety_distance >= 0
    //
    // r_eff_i is ellipsoid support radius in current obstacle normal direction:
    //   r_eff = sqrt((rx*n_x)^2 + (ry*n_y)^2 + (rz*n_z)^2), n in ellipsoid local frame.
    static int appendLinkEllipsoidConstraints(
        const Eigen::VectorXd& q_current,
        const LinkPoseMap& link_poses_world,
        const LinkCollisionEllipsoidList& link_ellipsoids,
        const arm_controller::kinematics::JacobianProvider& jacobian_provider,
        std::shared_ptr<const DistanceFieldInterface> distance_field,
        ObstacleConstraintInputList& out_constraints,
        std::string* error = nullptr);
};

}  // namespace arm_controller::algorithm::reactive_qp
