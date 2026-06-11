#pragma once

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"

#include <Eigen/Core>

#include <memory>
#include <limits>

namespace arm_controller::algorithm::cartesian_path_planner {

class BaseFootprintCollisionChecker {
public:
    struct BaseState {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    struct Diagnostic {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool collision_free{true};
        bool observed{false};
        bool distance_valid{false};
        double min_distance{std::numeric_limits<double>::infinity()};
        double required_distance{0.0};
        double sample_sphere_radius{0.0};
        Eigen::Vector3d worst_point_world{Eigen::Vector3d::Zero()};
        double segment_t{0.0};
        bool failed_on_segment_sample{false};
    };

    struct Config {
        enum class SampleMode {
            SurfaceGrid,
            BoundarySpheres,
        };

        Eigen::Vector3d center_in_base{Eigen::Vector3d(0.0, 0.0, 0.22)};
        Eigen::Vector3d size{Eigen::Vector3d(0.64, 0.64, 0.24)};
        double footprint_sample_resolution{0.08};
        double segment_sample_resolution{0.08};
        double yaw_sample_resolution{0.20};
        SampleMode sample_mode{SampleMode::SurfaceGrid};
        double sample_sphere_radius{0.0};
        bool unknown_is_free{true};
    };

    explicit BaseFootprintCollisionChecker(
        std::shared_ptr<const DistanceFieldInterface> distance_field);

    BaseFootprintCollisionChecker(
        std::shared_ptr<const DistanceFieldInterface> distance_field,
        Config config);

    bool isStateCollisionFree(
        const BaseState& state,
        double safe_distance) const;

    Diagnostic checkState(
        const BaseState& state,
        double safe_distance) const;

    bool isSegmentCollisionFree(
        const BaseState& from,
        const BaseState& to,
        double safe_distance) const;

    Diagnostic checkSegment(
        const BaseState& from,
        const BaseState& to,
        double safe_distance) const;

    const Config& config() const { return config_; }

private:
    Vector3dList makeLocalFootprintSamples() const;
    Vector3dList makeWorldFootprintSamples(const BaseState& state) const;

    static double normalizeAngle(double angle);
    static double angularDistance(double from, double to);

    std::shared_ptr<const DistanceFieldInterface> distance_field_;
    Config config_;
    Vector3dList local_footprint_samples_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
