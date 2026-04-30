#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>

namespace arm_controller::algorithm::cartesian_path_planner {

struct DistanceFieldQueryResult {
    bool observed{false};
    bool distance_valid{false};
    double distance{0.0};
    bool gradient_valid{false};
    Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
};

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

    virtual std::vector<DistanceFieldQueryResult> queryDistanceAndGradientBatch(
        const std::vector<Eigen::Vector3d>& positions) const {
        std::vector<DistanceFieldQueryResult> results;
        results.reserve(positions.size());
        for (const auto& p : positions) {
            results.push_back(queryDistanceAndGradient(p));
        }
        return results;
    }
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
