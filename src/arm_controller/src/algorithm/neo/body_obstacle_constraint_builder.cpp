#include "body_obstacle_constraint_builder.hpp"

#include <cmath>
#include <sstream>

namespace arm_controller::algorithm::reactive_qp {

namespace {

double effectiveEllipsoidRadiusAlongNormal(
    const Eigen::Matrix3d& R_world_link,
    const Eigen::Vector3d& radii_link,
    const Eigen::Vector3d& n_world) {
    const Eigen::Vector3d n_link = R_world_link.transpose() * n_world;
    const double x = radii_link.x() * n_link.x();
    const double y = radii_link.y() * n_link.y();
    const double z = radii_link.z() * n_link.z();
    const double v = x * x + y * y + z * z;
    return (v > 0.0) ? std::sqrt(v) : 0.0;
}

}  // namespace

BodyObstacleConstraintBuilder::DistanceQueryFn
BodyObstacleConstraintBuilder::makeDistanceQueryFromField(
    std::shared_ptr<const DistanceFieldInterface> distance_field) {
    return [distance_field = std::move(distance_field)](const Eigen::Vector3d& p) {
        DistanceQueryResult out;
        if (!distance_field) {
            return out;
        }
        if (!distance_field->isInsideMap(p)) {
            return out;
        }
        const double d = distance_field->getDistance(p);
        const Eigen::Vector3d g = distance_field->getGradient(p);
        if (!std::isfinite(d) || !g.allFinite()) {
            return out;
        }
        out.valid = true;
        out.distance = d;
        out.gradient = g;
        return out;
    };
}

int BodyObstacleConstraintBuilder::appendLinkSphereConstraints(
    const Eigen::VectorXd& q_current,
    const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses_world,
    const std::vector<LinkCollisionSphere>& link_spheres,
    const PointJacobianProvider& jacobian_provider,
    const DistanceQueryFn& distance_query,
    std::vector<ObstacleConstraintInput>& out_constraints,
    std::string* error) {
    std::vector<LinkCollisionEllipsoid> ellipsoids;
    ellipsoids.reserve(link_spheres.size());
    for (const auto& s : link_spheres) {
        LinkCollisionEllipsoid e;
        e.link_name = s.link_name;
        e.center_in_link = s.center_in_link;
        e.radii = Eigen::Vector3d::Constant(s.radius);
        e.debug_name = s.debug_name;
        ellipsoids.push_back(std::move(e));
    }
    return appendLinkEllipsoidConstraints(
        q_current,
        link_poses_world,
        ellipsoids,
        jacobian_provider,
        distance_query,
        out_constraints,
        error);
}

int BodyObstacleConstraintBuilder::appendLinkEllipsoidConstraints(
    const Eigen::VectorXd& q_current,
    const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses_world,
    const std::vector<LinkCollisionEllipsoid>& link_ellipsoids,
    const PointJacobianProvider& jacobian_provider,
    const DistanceQueryFn& distance_query,
    std::vector<ObstacleConstraintInput>& out_constraints,
    std::string* error) {
    if (q_current.size() <= 0) {
        if (error != nullptr) {
            *error = "q_current is empty.";
        }
        return 0;
    }
    if (!distance_query) {
        if (error != nullptr) {
            *error = "distance_query callback is empty.";
        }
        return 0;
    }

    int appended = 0;
    const int dof = static_cast<int>(q_current.size());

    for (std::size_t i = 0; i < link_ellipsoids.size(); ++i) {
        const auto& e = link_ellipsoids[i];
        if (!(e.radii.x() > 0.0) || !(e.radii.y() > 0.0) || !(e.radii.z() > 0.0) ||
            !e.radii.allFinite()) {
            continue;
        }

        const auto it = link_poses_world.find(e.link_name);
        if (it == link_poses_world.end()) {
            continue;
        }
        const Eigen::Isometry3d& T_world_link = it->second;
        const Eigen::Vector3d p_world = T_world_link * e.center_in_link;

        const DistanceQueryResult dq = distance_query(p_world);
        if (!dq.valid || !std::isfinite(dq.distance) || !dq.gradient.allFinite()) {
            continue;
        }

        Eigen::Vector3d n_world = dq.gradient;
        const double gn = n_world.norm();
        if (gn < 1e-9) {
            continue;
        }
        n_world /= gn;

        const double r_eff = effectiveEllipsoidRadiusAlongNormal(
            T_world_link.linear(), e.radii, n_world);
        if (!std::isfinite(r_eff)) {
            continue;
        }

        const Eigen::MatrixXd J_point =
            jacobian_provider.computePointJacobian(q_current, e.link_name, e.center_in_link);
        if (J_point.rows() < 3 || J_point.cols() != dof || !J_point.allFinite()) {
            continue;
        }

        ObstacleConstraintInput c;
        c.normal_jacobian = n_world.transpose() * J_point.topRows(3);
        c.distance = dq.distance - r_eff;
        c.debug_name = e.debug_name.empty()
                           ? ("link_ellipsoid_" + e.link_name + "_" + std::to_string(i))
                           : e.debug_name;

        if (!std::isfinite(c.distance) || !c.normal_jacobian.allFinite()) {
            continue;
        }
        out_constraints.push_back(std::move(c));
        ++appended;
    }

    if (appended == 0 && error != nullptr) {
        *error = "No valid link-ellipsoid obstacle constraints were generated.";
    }
    return appended;
}

}  // namespace arm_controller::algorithm::reactive_qp
