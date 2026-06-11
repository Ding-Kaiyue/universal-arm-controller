#include "algorithm/cartesian_path_planner/map/composite_distance_field.hpp"

#include <limits>

namespace arm_controller::algorithm::cartesian_path_planner {

void CompositeDistanceField::addField(
    std::shared_ptr<const DistanceFieldInterface> field) {
    if (field) {
        fields_.push_back(std::move(field));
    }
}

bool CompositeDistanceField::isInsideMap(const Eigen::Vector3d& p) const {
    for (const auto& field : fields_) {
        if (field && field->isInsideMap(p)) {
            return true;
        }
    }
    return false;
}

double CompositeDistanceField::getDistance(const Eigen::Vector3d& p) const {
    const DistanceFieldQueryResult result = queryDistanceAndGradient(p);
    return result.distance_valid ? result.distance : -1.0;
}

Eigen::Vector3d CompositeDistanceField::getGradient(
    const Eigen::Vector3d& p) const {
    const DistanceFieldQueryResult result = queryDistanceAndGradient(p);
    return result.gradient_valid ? result.gradient : Eigen::Vector3d::Zero();
}

DistanceFieldQueryResult CompositeDistanceField::queryDistanceAndGradient(
    const Eigen::Vector3d& p) const {
    DistanceFieldQueryResult best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& field : fields_) {
        if (!field) {
            continue;
        }
        const DistanceFieldQueryResult query =
            field->queryDistanceAndGradient(p);
        if (!query.observed || !query.distance_valid) {
            continue;
        }
        if (query.distance < best_distance) {
            best_distance = query.distance;
            best = query;
        }
    }
    return best;
}

DistanceFieldQueryResultList CompositeDistanceField::queryDistanceAndGradientBatch(
    const Vector3dList& positions) const {
    DistanceFieldQueryResultList results(positions.size());
    std::vector<double> best_distances(
        positions.size(), std::numeric_limits<double>::infinity());
    for (const auto& field : fields_) {
        if (!field) {
            continue;
        }
        const DistanceFieldQueryResultList field_results =
            field->queryDistanceAndGradientBatch(positions);
        if (field_results.size() != positions.size()) {
            continue;
        }
        for (std::size_t i = 0; i < field_results.size(); ++i) {
            const DistanceFieldQueryResult& query = field_results[i];
            if (!query.observed || !query.distance_valid) {
                continue;
            }
            if (query.distance < best_distances[i]) {
                best_distances[i] = query.distance;
                results[i] = query;
            }
        }
    }
    return results;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
