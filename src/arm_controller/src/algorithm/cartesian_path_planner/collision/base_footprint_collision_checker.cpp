#include "algorithm/cartesian_path_planner/collision/base_footprint_collision_checker.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace arm_controller::algorithm::cartesian_path_planner {

BaseFootprintCollisionChecker::BaseFootprintCollisionChecker(
    std::shared_ptr<const DistanceFieldInterface> distance_field)
    : BaseFootprintCollisionChecker(std::move(distance_field), Config{}) {}

BaseFootprintCollisionChecker::BaseFootprintCollisionChecker(
    std::shared_ptr<const DistanceFieldInterface> distance_field,
    Config config)
    : distance_field_(std::move(distance_field)),
      config_(std::move(config)),
      local_footprint_samples_(makeLocalFootprintSamples()) {}

bool BaseFootprintCollisionChecker::isStateCollisionFree(
    const BaseState& state,
    const double safe_distance) const {
    return checkState(state, safe_distance).collision_free;
}

BaseFootprintCollisionChecker::Diagnostic BaseFootprintCollisionChecker::checkState(
    const BaseState& state,
    const double safe_distance) const {
    Diagnostic diag;
    diag.sample_sphere_radius = std::max(0.0, config_.sample_sphere_radius);
    diag.required_distance = safe_distance + diag.sample_sphere_radius;
    if (!distance_field_) {
        return diag;
    }

    const Vector3dList world_samples = makeWorldFootprintSamples(state);
    const DistanceFieldQueryResultList queries =
        distance_field_->queryDistanceAndGradientBatch(world_samples);
    if (queries.size() != world_samples.size()) {
        diag.collision_free = false;
        return diag;
    }

    for (std::size_t i = 0; i < queries.size(); ++i) {
        const DistanceFieldQueryResult& query = queries[i];
        if (!query.observed || !query.distance_valid) {
            if (config_.unknown_is_free) {
                continue;
            }
            diag.collision_free = false;
            diag.observed = query.observed;
            diag.distance_valid = query.distance_valid;
            if (i < world_samples.size()) {
                diag.worst_point_world = world_samples[i];
            }
            return diag;
        }
        diag.observed = true;
        diag.distance_valid = true;
        if (query.distance < diag.min_distance) {
            diag.min_distance = query.distance;
            if (i < world_samples.size()) {
                diag.worst_point_world = world_samples[i];
            }
        }
        if (query.distance < diag.required_distance) {
            diag.collision_free = false;
        }
    }
    return diag;
}

bool BaseFootprintCollisionChecker::isSegmentCollisionFree(
    const BaseState& from,
    const BaseState& to,
    const double safe_distance) const {
    return checkSegment(from, to, safe_distance).collision_free;
}

BaseFootprintCollisionChecker::Diagnostic BaseFootprintCollisionChecker::checkSegment(
    const BaseState& from,
    const BaseState& to,
    const double safe_distance) const {
    Diagnostic best_diag;
    best_diag.sample_sphere_radius = std::max(0.0, config_.sample_sphere_radius);
    best_diag.required_distance = safe_distance + best_diag.sample_sphere_radius;
    if (!distance_field_) {
        return best_diag;
    }

    const double xy_distance = std::hypot(to.x - from.x, to.y - from.y);
    const double yaw_distance = std::abs(angularDistance(from.yaw, to.yaw));
    const int steps = std::max(
        1,
        std::max(
            static_cast<int>(std::ceil(
                xy_distance / std::max(0.01, config_.segment_sample_resolution))),
            static_cast<int>(std::ceil(
                yaw_distance / std::max(0.02, config_.yaw_sample_resolution)))));

    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        BaseState sample;
        sample.x = from.x + t * (to.x - from.x);
        sample.y = from.y + t * (to.y - from.y);
        sample.yaw =
            normalizeAngle(from.yaw + t * angularDistance(from.yaw, to.yaw));
        Diagnostic sample_diag = checkState(sample, safe_distance);
        sample_diag.segment_t = t;
        if (!sample_diag.collision_free) {
            sample_diag.failed_on_segment_sample = true;
            return sample_diag;
        }
        if (sample_diag.min_distance < best_diag.min_distance) {
            best_diag = sample_diag;
            best_diag.segment_t = t;
        }
    }
    return best_diag;
}

Vector3dList BaseFootprintCollisionChecker::makeLocalFootprintSamples() const {
    Vector3dList samples;
    if (config_.sample_mode == Config::SampleMode::BoundarySpheres) {
        const double sx = std::max(0.01, config_.size.x());
        const double sy = std::max(0.01, config_.size.y());
        const double sz = std::max(0.01, config_.size.z());
        const double radius = std::max(
            0.02,
            config_.sample_sphere_radius > 0.0
                ? config_.sample_sphere_radius
                : config_.footprint_sample_resolution);
        const double z_min = radius;
        const double z_max = std::max(z_min, sz);
        const int nz = std::max(
            1,
            static_cast<int>(std::ceil((z_max - z_min) / radius)) + 1);

        auto pushPoint = [&](const Eigen::Vector2d& p, const double z) {
            samples.push_back(
                config_.center_in_base + Eigen::Vector3d(p.x(), p.y(), z));
        };
        for (int iz = 0; iz < nz; ++iz) {
            const double tz =
                nz <= 1 ? 0.0 : static_cast<double>(iz) / static_cast<double>(nz - 1);
            const double z = -0.5 * sz + z_min + tz * (z_max - z_min);
            const Eigen::Vector2d c1(
                0.5 * sx - radius, 0.5 * sy - radius);
            const Eigen::Vector2d c2(
                0.5 * sx - radius, -0.5 * sy + radius);
            const Eigen::Vector2d c3(
                -0.5 * sx + radius, -0.5 * sy + radius);
            const Eigen::Vector2d c4(
                -0.5 * sx + radius, 0.5 * sy - radius);
            const std::array<Eigen::Vector2d, 4> corners{c1, c2, c3, c4};
            for (int edge = 0; edge < 4; ++edge) {
                const Eigen::Vector2d from = corners[static_cast<std::size_t>(edge)];
                const Eigen::Vector2d to =
                    corners[static_cast<std::size_t>((edge + 1) % 4)];
                const double length = (to - from).norm();
                const int n = std::max(
                    1,
                    static_cast<int>(std::ceil(length / radius)));
                for (int i = 0; i <= n; ++i) {
                    if (edge > 0 && i == 0) {
                        continue;
                    }
                    const double t =
                        static_cast<double>(i) / static_cast<double>(n);
                    pushPoint((1.0 - t) * from + t * to, z);
                }
            }
        }
        if (samples.empty()) {
            samples.push_back(config_.center_in_base);
        }
        return samples;
    }

    const double sx = std::max(0.01, config_.size.x());
    const double sy = std::max(0.01, config_.size.y());
    const double sz = std::max(0.01, config_.size.z());
    const double resolution =
        std::max(0.02, config_.footprint_sample_resolution);
    const int nx = std::max(2, static_cast<int>(std::ceil(sx / resolution)) + 1);
    const int ny = std::max(2, static_cast<int>(std::ceil(sy / resolution)) + 1);
    const int nz = std::max(2, static_cast<int>(std::ceil(sz / resolution)) + 1);
    samples.reserve(static_cast<std::size_t>(nx * ny * nz));

    for (int ix = 0; ix < nx; ++ix) {
        const double x = -0.5 * sx +
                         sx * static_cast<double>(ix) /
                             static_cast<double>(nx - 1);
        for (int iy = 0; iy < ny; ++iy) {
            const double y = -0.5 * sy +
                             sy * static_cast<double>(iy) /
                                 static_cast<double>(ny - 1);
            for (int iz = 0; iz < nz; ++iz) {
                const double z = -0.5 * sz +
                                 sz * static_cast<double>(iz) /
                                     static_cast<double>(nz - 1);
                const bool on_surface =
                    ix == 0 || ix + 1 == nx || iy == 0 || iy + 1 == ny ||
                    iz == 0 || iz + 1 == nz;
                if (!on_surface) {
                    continue;
                }
                samples.push_back(
                    config_.center_in_base + Eigen::Vector3d(x, y, z));
            }
        }
    }

    if (samples.empty()) {
        samples.push_back(config_.center_in_base);
    }
    return samples;
}

Vector3dList BaseFootprintCollisionChecker::makeWorldFootprintSamples(
    const BaseState& state) const {
    Vector3dList samples;
    samples.reserve(local_footprint_samples_.size());
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d p_base(state.x, state.y, 0.0);
    for (const Eigen::Vector3d& local : local_footprint_samples_) {
        samples.push_back(p_base + R * local);
    }
    return samples;
}

double BaseFootprintCollisionChecker::normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double BaseFootprintCollisionChecker::angularDistance(
    const double from,
    const double to) {
    return normalizeAngle(to - from);
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
