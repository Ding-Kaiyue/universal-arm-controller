#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>

namespace arm_controller::algorithm::cartesian_path_planner {

using Vector3dList =
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

struct DistanceFieldQueryResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool observed{false};
    bool distance_valid{false};
    double distance{0.0};
    bool gradient_valid{false};
    Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
};

using DistanceFieldQueryResultList =
    std::vector<DistanceFieldQueryResult, Eigen::aligned_allocator<DistanceFieldQueryResult>>;

class DistanceFieldInterface {
public:
    virtual ~DistanceFieldInterface() = default;

    virtual bool isInsideMap(const Eigen::Vector3d& p) const = 0;
    virtual double getDistance(const Eigen::Vector3d& p) const = 0;
    virtual Eigen::Vector3d getGradient(const Eigen::Vector3d& p) const = 0;

    virtual DistanceFieldQueryResult queryDistanceAndGradient(
        const Eigen::Vector3d& p) const {
        DistanceFieldQueryResult result;
        result.observed = isInsideMap(p);
        if (!result.observed) {
            return result;
        }

        result.distance = getDistance(p);
        result.distance_valid = std::isfinite(result.distance);

        result.gradient = getGradient(p);
        result.gradient_valid = result.gradient.allFinite();
        return result;
    }

    virtual DistanceFieldQueryResultList queryDistanceAndGradientBatch(
        const Vector3dList& positions) const {
        DistanceFieldQueryResultList results;
        results.reserve(positions.size());
        for (const auto& p : positions) {
            results.push_back(queryDistanceAndGradient(p));
        }
        return results;
    }
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
