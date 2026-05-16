#include "body_obstacle_constraint_builder.hpp"

#include <cmath>
#include <sstream>

namespace arm_controller::algorithm::reactive_qp {

namespace {
constexpr double kSelfNearestRejectPaddingM = 0.05;

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

struct PreparedEllipsoidQuery {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const LinkCollisionEllipsoid* ellipsoid{nullptr};
    const Eigen::Isometry3d* link_pose_world{nullptr};
    Eigen::Vector3d point_world{Eigen::Vector3d::Zero()};
    std::size_t original_index{0};
};

using PreparedEllipsoidQueryList =
    std::vector<PreparedEllipsoidQuery, Eigen::aligned_allocator<PreparedEllipsoidQuery>>;

PreparedEllipsoidQueryList prepareEllipsoidQueries(
    const BodyObstacleConstraintBuilder::LinkPoseMap& link_poses_world,
    const LinkCollisionEllipsoidList& link_ellipsoids) {
    PreparedEllipsoidQueryList prepared;
    prepared.reserve(link_ellipsoids.size());
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

        PreparedEllipsoidQuery entry;
        entry.ellipsoid = &e;
        entry.link_pose_world = &it->second;
        entry.point_world = it->second * e.center_in_link;
        entry.original_index = i;
        prepared.push_back(entry);
    }
    return prepared;
}

bool pointInsideInflatedRobotEllipsoid(
    const Eigen::Vector3d& point_world,
    const PreparedEllipsoidQueryList& prepared_queries,
    const double padding_m) {
    if (!point_world.allFinite()) {
        return false;
    }
    for (const PreparedEllipsoidQuery& prepared : prepared_queries) {
        if (prepared.ellipsoid == nullptr || prepared.link_pose_world == nullptr ||
            !prepared.ellipsoid->radii.allFinite()) {
            continue;
        }
        const Eigen::Vector3d radii =
            (prepared.ellipsoid->radii +
             Eigen::Vector3d::Constant(std::max(0.0, padding_m)))
                .cwiseMax(Eigen::Vector3d::Constant(1e-4));
        const Eigen::Vector3d center_world =
            (*prepared.link_pose_world) * prepared.ellipsoid->center_in_link;
        const Eigen::Vector3d delta_link =
            prepared.link_pose_world->linear().transpose() *
            (point_world - center_world);
        const double normalized_sq =
            delta_link.cwiseQuotient(radii).squaredNorm();
        if (std::isfinite(normalized_sq) && normalized_sq <= 1.0) {
            return true;
        }
    }
    return false;
}

int appendPreparedEllipsoidConstraints(
    const Eigen::VectorXd& q_current,
    const PreparedEllipsoidQueryList& prepared_queries,
    const DistanceQueryResultList& distance_queries,
    const arm_controller::kinematics::JacobianProvider& jacobian_provider,
    ObstacleConstraintInputList& out_constraints) {
    const int dof = static_cast<int>(q_current.size());
    int appended = 0;

    const std::size_t query_count = std::min(prepared_queries.size(), distance_queries.size());
    for (std::size_t i = 0; i < query_count; ++i) {
        const PreparedEllipsoidQuery& prepared = prepared_queries[i];
        const LinkCollisionEllipsoid& e = *prepared.ellipsoid;
        const Eigen::Isometry3d& T_world_link = *prepared.link_pose_world;
        const DistanceQueryResult& dq = distance_queries[i];

        if (!dq.valid || !std::isfinite(dq.distance)) {
            continue;
        }

        Eigen::Vector3d n_world = dq.gradient;
        const double gn = n_world.norm();
        if (gn < 1e-9) {
            continue;
        }
        n_world /= gn;
        const Eigen::Vector3d nearest_obstacle_point =
            prepared.point_world - dq.distance * n_world;
        if (pointInsideInflatedRobotEllipsoid(
                nearest_obstacle_point,
                prepared_queries,
                kSelfNearestRejectPaddingM)) {
            continue;
        }

        const double r_eff = effectiveEllipsoidRadiusAlongNormal(
            T_world_link.linear(), e.radii, n_world);
        if (!std::isfinite(r_eff)) {
            continue;
        }

        const Eigen::MatrixXd J_point =
            jacobian_provider.computeJacobian(q_current, e.link_name, e.center_in_link);
        if (J_point.rows() < 3 || J_point.cols() != dof || !J_point.allFinite()) {
            continue;
        }

        ObstacleConstraintInput c;
        c.normal_jacobian = n_world.transpose() * J_point.topRows(3);
        c.linear_jacobian = J_point.topRows(3);
        c.normal_world = n_world;
        c.distance = dq.distance - r_eff;
        c.debug_name = e.debug_name.empty()
                           ? ("link_ellipsoid_" + e.link_name + "_" +
                              std::to_string(prepared.original_index))
                           : e.debug_name;

        if (!std::isfinite(c.distance) || !c.normal_jacobian.allFinite() ||
            !c.linear_jacobian.allFinite() || !c.normal_world.allFinite()) {
            continue;
        }
        out_constraints.push_back(std::move(c));
        ++appended;
    }

    return appended;
}

}  // namespace

int BodyObstacleConstraintBuilder::appendLinkSphereConstraints(
    const Eigen::VectorXd& q_current,
    const LinkPoseMap& link_poses_world,
    const LinkCollisionSphereList& link_spheres,
    const arm_controller::kinematics::JacobianProvider& jacobian_provider,
    std::shared_ptr<const DistanceFieldInterface> distance_field,
    ObstacleConstraintInputList& out_constraints,
    std::string* error) {
    LinkCollisionEllipsoidList ellipsoids;
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
        std::move(distance_field),
        out_constraints,
        error);
}

int BodyObstacleConstraintBuilder::appendLinkEllipsoidConstraints(
    const Eigen::VectorXd& q_current,
    const LinkPoseMap& link_poses_world,
    const LinkCollisionEllipsoidList& link_ellipsoids,
    const arm_controller::kinematics::JacobianProvider& jacobian_provider,
    std::shared_ptr<const DistanceFieldInterface> distance_field,
    ObstacleConstraintInputList& out_constraints,
    std::string* error) {
    if (q_current.size() <= 0) {
        if (error != nullptr) {
            *error = "q_current is empty.";
        }
        return 0;
    }
    if (!distance_field) {
        if (error != nullptr) {
            *error = "distance_field is empty.";
        }
        return 0;
    }

    const PreparedEllipsoidQueryList prepared_queries =
        prepareEllipsoidQueries(link_poses_world, link_ellipsoids);
    if (prepared_queries.empty()) {
        if (error != nullptr) {
            *error = "No valid link ellipsoids were available for distance queries.";
        }
        return 0;
    }

    cartesian_path_planner::Vector3dList query_points;
    query_points.reserve(prepared_queries.size());
    for (const auto& prepared : prepared_queries) {
        query_points.push_back(prepared.point_world);
    }

    const cartesian_path_planner::DistanceFieldQueryResultList field_queries =
        distance_field->queryDistanceAndGradientBatch(query_points);
    if (field_queries.size() != query_points.size()) {
        if (error != nullptr) {
            *error = "Distance-field batch query size mismatch.";
        }
        return 0;
    }

    DistanceQueryResultList distance_queries;
    distance_queries.reserve(prepared_queries.size());
    for (const auto& query : field_queries) {
        DistanceQueryResult out;
        if (query.observed && query.distance_valid) {
            out.valid = true;
            out.distance = query.distance;
            if (query.gradient_valid) {
                out.gradient = query.gradient;
            }
        }
        distance_queries.push_back(std::move(out));
    }

    const int appended = appendPreparedEllipsoidConstraints(
        q_current, prepared_queries, distance_queries, jacobian_provider, out_constraints);
    if (appended == 0 && error != nullptr) {
        *error = "No valid link-ellipsoid obstacle constraints were generated.";
    }
    return appended;
}

}  // namespace arm_controller::algorithm::reactive_qp
