#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "algorithm/cartesian_path_planner/types.hpp"
#include "algorithm/neo/body_obstacle_constraint_builder.hpp"
#include "arm_controller/kinematics/forward_kinematics.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class WholeBodyEllipsoidPoseValidator {
public:
    using IkSolveFn = std::function<bool(
        const Eigen::Vector3d&,
        const Eigen::Matrix3d&,
        const std::optional<Eigen::VectorXd>&,
        Eigen::VectorXd&)>;

    struct Config {
        int ik_max_iterations{25};
        double ik_pos_tolerance_m{0.008};
        double ik_rot_tolerance_rad{0.10};
        double ik_damping{0.05};
        double ik_step_scale{0.6};
        int segment_substeps_min{1};
        double seed_pose_accept_pos_tolerance_m{0.02};
        double seed_pose_accept_rot_tolerance_rad{0.20};
        // Block only when margin falls below this threshold.
        // With margin = d - (safe_distance + r_eff):
        // - 0.0 means strict safety-buffer enforcement
        // - -safe_distance means allow consuming the safety buffer but still
        //   reject true ellipsoid-obstacle overlap
        double collision_blocking_margin_m{-0.015};
        double pose_cache_position_resolution_m{0.003};
        double pose_cache_quaternion_resolution{0.02};
        std::size_t pose_cache_max_entries{4096};
        std::size_t segment_cache_max_entries{4096};
        std::optional<Eigen::VectorXd> default_q_seed;
        // Optional external IK solver (e.g., TRAC-IK closest-to-seed).
        IkSolveFn ik_solver_fn;
    };

    using LinkCollisionEllipsoid =
        arm_controller::algorithm::reactive_qp::LinkCollisionEllipsoid;

    struct PoseDiagnostic {
        bool ik_ok{false};
        bool external_ik_ok{false};
        bool fallback_ik_used{false};
        bool collision_free{false};
        double min_margin{0.0};
        std::string reason;
        std::string worst_link_name;
        Eigen::VectorXd q_solution;
    };

    WholeBodyEllipsoidPoseValidator(
        Config cfg,
        std::shared_ptr<const DistanceFieldInterface> distance_field,
        std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics> fk_provider,
        std::shared_ptr<arm_controller::kinematics::JacobianProvider> jacobian_provider,
        std::vector<LinkCollisionEllipsoid> link_ellipsoids);

    bool validatePose(
        const Eigen::Vector3d& p_target,
        const Eigen::Matrix3d& R_target,
        double safe_distance,
        const std::optional<Eigen::VectorXd>& q_seed,
        Eigen::VectorXd& q_solution) const;

    bool validateSegment(
        const CartesianWaypoint& from,
        const CartesianWaypoint& to,
        double safe_distance,
        const std::optional<Eigen::VectorXd>& q_seed,
        Eigen::VectorXd& q_end) const;

    PoseDiagnostic diagnosePose(
        const Eigen::Vector3d& p_target,
        const Eigen::Matrix3d& R_target,
        double safe_distance,
        const std::optional<Eigen::VectorXd>& q_seed) const;

    PathPlanningInput::WholeBodyPoseValidatorFn makePoseValidatorFn() const;
    PathPlanningInput::WholeBodySegmentValidatorFn makeSegmentValidatorFn() const;
    PathPlanningInput::WholeBodyPoseDiagnosticFn makePoseDiagnosticFn() const;

private:
    struct IkSolveDiagnostic {
        bool final_ok{false};
        bool external_ik_ok{false};
        bool fallback_ik_used{false};
    };

    bool solveIk(
        const Eigen::Vector3d& p_target,
        const Eigen::Matrix3d& R_target,
        const std::optional<Eigen::VectorXd>& q_seed,
        Eigen::VectorXd& q_solution,
        IkSolveDiagnostic* ik_diag = nullptr) const;

    bool isWholeBodyCollisionFree(
        const Eigen::VectorXd& q,
        double safe_distance) const;

    struct PoseCacheKey {
        int px{0};
        int py{0};
        int pz{0};
        int qx{0};
        int qy{0};
        int qz{0};
        int qw{0};
        int sd{0};

        bool operator==(const PoseCacheKey& other) const {
            return px == other.px && py == other.py && pz == other.pz &&
                   qx == other.qx && qy == other.qy && qz == other.qz && qw == other.qw &&
                   sd == other.sd;
        }
    };

    struct PoseCacheKeyHash {
        std::size_t operator()(const PoseCacheKey& key) const noexcept;
    };

    struct PoseCacheValue {
        Eigen::VectorXd q_solution;
    };

    struct SegmentCacheKey {
        PoseCacheKey from;
        PoseCacheKey to;

        bool operator==(const SegmentCacheKey& other) const {
            return from == other.from && to == other.to;
        }
    };

    struct SegmentCacheKeyHash {
        std::size_t operator()(const SegmentCacheKey& key) const noexcept;
    };

    struct SegmentCacheValue {
        Eigen::VectorXd q_end;
    };

    PoseCacheKey makePoseCacheKey(
        const Eigen::Vector3d& p_target,
        const Eigen::Matrix3d& R_target,
        double safe_distance) const;

    bool lookupPoseCache(
        const PoseCacheKey& key,
        Eigen::VectorXd& q_solution) const;

    void storePoseCache(
        const PoseCacheKey& key,
        const Eigen::VectorXd& q_solution) const;

    SegmentCacheKey makeSegmentCacheKey(
        const CartesianWaypoint& from,
        const CartesianWaypoint& to,
        double safe_distance) const;

    bool lookupSegmentCache(
        const SegmentCacheKey& key,
        Eigen::VectorXd& q_end) const;

    void storeSegmentCache(
        const SegmentCacheKey& key,
        const Eigen::VectorXd& q_end) const;

private:
    Config cfg_;
    std::shared_ptr<const DistanceFieldInterface> distance_field_;
    std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics> fk_provider_;
    std::shared_ptr<arm_controller::kinematics::JacobianProvider> jacobian_provider_;
    std::vector<LinkCollisionEllipsoid> link_ellipsoids_;
    mutable std::mutex pose_cache_mutex_;
    mutable std::unordered_map<PoseCacheKey, PoseCacheValue, PoseCacheKeyHash> pose_cache_;
    mutable std::mutex segment_cache_mutex_;
    mutable std::unordered_map<SegmentCacheKey, SegmentCacheValue, SegmentCacheKeyHash> segment_cache_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
