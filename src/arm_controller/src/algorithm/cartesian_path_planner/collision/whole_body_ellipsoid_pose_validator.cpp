#include "algorithm/cartesian_path_planner/collision/whole_body_ellipsoid_pose_validator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace arm_controller::algorithm::cartesian_path_planner {

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

std::size_t WholeBodyEllipsoidPoseValidator::PoseCacheKeyHash::operator()(
    const PoseCacheKey& key) const noexcept {
    std::size_t h = std::hash<int>{}(key.px);
    h ^= (std::hash<int>{}(key.py) << 1);
    h ^= (std::hash<int>{}(key.pz) << 2);
    h ^= (std::hash<int>{}(key.qx) << 3);
    h ^= (std::hash<int>{}(key.qy) << 4);
    h ^= (std::hash<int>{}(key.qz) << 5);
    h ^= (std::hash<int>{}(key.qw) << 6);
    h ^= (std::hash<int>{}(key.sd) << 7);
    return h;
}

std::size_t WholeBodyEllipsoidPoseValidator::SegmentCacheKeyHash::operator()(
    const SegmentCacheKey& key) const noexcept {
    const PoseCacheKeyHash pose_hasher;
    std::size_t h = pose_hasher(key.from);
    h ^= (pose_hasher(key.to) << 1);
    return h;
}

WholeBodyEllipsoidPoseValidator::WholeBodyEllipsoidPoseValidator(
    Config cfg,
    std::shared_ptr<const DistanceFieldInterface> distance_field,
    std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics> fk_provider,
    std::shared_ptr<arm_controller::kinematics::JacobianProvider> jacobian_provider,
    std::vector<LinkCollisionEllipsoid> link_ellipsoids)
    : cfg_(cfg),
      distance_field_(std::move(distance_field)),
      fk_provider_(std::move(fk_provider)),
      jacobian_provider_(std::move(jacobian_provider)),
      link_ellipsoids_(std::move(link_ellipsoids)) {}

bool WholeBodyEllipsoidPoseValidator::validatePose(
    const Eigen::Vector3d& p_target,
    const Eigen::Matrix3d& R_target,
    const double safe_distance,
    const std::optional<Eigen::VectorXd>& q_seed,
    Eigen::VectorXd& q_solution) const {
    const PoseCacheKey cache_key = makePoseCacheKey(p_target, R_target, safe_distance);
    if (lookupPoseCache(cache_key, q_solution)) {
        return true;
    }
    if (!solveIk(p_target, R_target, q_seed, q_solution)) {
        return false;
    }
    if (!isWholeBodyCollisionFree(q_solution, safe_distance)) {
        return false;
    }
    storePoseCache(cache_key, q_solution);
    return true;
}

bool WholeBodyEllipsoidPoseValidator::validateSegment(
    const CartesianWaypoint& from,
    const CartesianWaypoint& to,
    const double safe_distance,
    const std::optional<Eigen::VectorXd>& q_seed,
    Eigen::VectorXd& q_end) const {
    const SegmentCacheKey cache_key = makeSegmentCacheKey(from, to, safe_distance);
    if (lookupSegmentCache(cache_key, q_end)) {
        return true;
    }

    const double dist = (to.position - from.position).norm();
    const int steps =
        std::max(cfg_.segment_substeps_min, static_cast<int>(std::ceil(dist / 0.03)));
    std::optional<Eigen::VectorXd> q_prev = q_seed;

    const Eigen::Quaterniond q0(from.orientation);
    const Eigen::Quaterniond q1(to.orientation);
    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Eigen::Vector3d p = (1.0 - t) * from.position + t * to.position;
        const Eigen::Matrix3d R = q0.slerp(t, q1).normalized().toRotationMatrix();
        Eigen::VectorXd q_i;
        if (!validatePose(p, R, safe_distance, q_prev, q_i)) {
            return false;
        }
        q_prev = q_i;
    }
    if (!q_prev.has_value()) {
        return false;
    }
    q_end = *q_prev;
    storeSegmentCache(cache_key, q_end);
    return true;
}

WholeBodyEllipsoidPoseValidator::PoseDiagnostic
WholeBodyEllipsoidPoseValidator::diagnosePose(
    const Eigen::Vector3d& p_target,
    const Eigen::Matrix3d& R_target,
    const double safe_distance,
    const std::optional<Eigen::VectorXd>& q_seed) const {
    PoseDiagnostic diag;

    IkSolveDiagnostic ik_diag;
    if (!solveIk(p_target, R_target, q_seed, diag.q_solution, &ik_diag)) {
        diag.ik_ok = false;
        diag.external_ik_ok = ik_diag.external_ik_ok;
        diag.fallback_ik_used = ik_diag.fallback_ik_used;
        diag.collision_free = false;
        diag.min_margin = -1.0;
        diag.reason = "ik_fail";
        return diag;
    }

    diag.ik_ok = true;
    diag.external_ik_ok = ik_diag.external_ik_ok;
    diag.fallback_ik_used = ik_diag.fallback_ik_used;

    if (!distance_field_ || !fk_provider_) {
        diag.collision_free = false;
        diag.min_margin = -1.0;
        diag.reason = "invalid_validator";
        return diag;
    }

    arm_controller::kinematics::ForwardKinematicsOutput fk_out;
    if (!fk_provider_->compute(diag.q_solution, fk_out)) {
        diag.collision_free = false;
        diag.min_margin = -1.0;
        diag.reason = "fk_fail";
        return diag;
    }

    diag.collision_free = true;
    diag.min_margin = std::numeric_limits<double>::infinity();
    diag.reason = "ok";
    for (const auto& e : link_ellipsoids_) {
        const auto it = fk_out.link_poses.find(e.link_name);
        if (it == fk_out.link_poses.end()) {
            continue;
        }
        const Eigen::Isometry3d& T_world_link = it->second;
        const Eigen::Vector3d p_world = T_world_link * e.center_in_link;
        if (!distance_field_->isInsideMap(p_world)) {
            diag.collision_free = false;
            diag.min_margin = -1.0;
            diag.reason = "out_of_map";
            diag.worst_link_name = e.debug_name.empty() ? e.link_name : e.debug_name;
            return diag;
        }
        const double d = distance_field_->getDistance(p_world);
        Eigen::Vector3d n_world = distance_field_->getGradient(p_world);
        if (!std::isfinite(d) || !n_world.allFinite()) {
            diag.collision_free = false;
            diag.min_margin = -1.0;
            diag.reason = "invalid_distance_field";
            diag.worst_link_name = e.debug_name.empty() ? e.link_name : e.debug_name;
            return diag;
        }
        const double gn = n_world.norm();
        if (gn < 1e-9) {
            diag.collision_free = false;
            diag.min_margin = -1.0;
            diag.reason = "invalid_gradient";
            diag.worst_link_name = e.debug_name.empty() ? e.link_name : e.debug_name;
            return diag;
        }
        n_world /= gn;
        const double r_eff =
            effectiveEllipsoidRadiusAlongNormal(T_world_link.linear(), e.radii, n_world);
        const double margin = d - (safe_distance + r_eff);
        if (margin < diag.min_margin) {
            diag.min_margin = margin;
            diag.worst_link_name = e.debug_name.empty() ? e.link_name : e.debug_name;
        }
        if (margin < cfg_.collision_blocking_margin_m) {
            diag.collision_free = false;
            diag.reason = "collision_fail";
        }
    }

    if (!std::isfinite(diag.min_margin)) {
        diag.min_margin = -1.0;
        if (diag.reason == "ok") {
            diag.reason = "no_ellipsoids";
        }
    }
    return diag;
}

PathPlanningInput::WholeBodyPoseValidatorFn
WholeBodyEllipsoidPoseValidator::makePoseValidatorFn() const {
    return [this](
               const Eigen::Vector3d& p,
               const Eigen::Matrix3d& R,
               const double safe_distance,
               const std::optional<Eigen::VectorXd>& q_seed,
               Eigen::VectorXd& q_solution) {
        return this->validatePose(p, R, safe_distance, q_seed, q_solution);
    };
}

PathPlanningInput::WholeBodySegmentValidatorFn
WholeBodyEllipsoidPoseValidator::makeSegmentValidatorFn() const {
    return [this](
               const CartesianWaypoint& from,
               const CartesianWaypoint& to,
               const double safe_distance,
               const std::optional<Eigen::VectorXd>& q_seed,
               Eigen::VectorXd& q_end) {
        return this->validateSegment(from, to, safe_distance, q_seed, q_end);
    };
}

PathPlanningInput::WholeBodyPoseDiagnosticFn
WholeBodyEllipsoidPoseValidator::makePoseDiagnosticFn() const {
    return [this](
               const Eigen::Vector3d& p,
               const Eigen::Matrix3d& R,
               const double safe_distance,
               const std::optional<Eigen::VectorXd>& q_seed) {
        const PoseDiagnostic diag = this->diagnosePose(p, R, safe_distance, q_seed);
        PathPlanningInput::WholeBodyPoseDiagnostic out;
        out.ik_ok = diag.ik_ok;
        out.external_ik_ok = diag.external_ik_ok;
        out.fallback_ik_used = diag.fallback_ik_used;
        out.collision_free = diag.collision_free;
        out.min_margin = diag.min_margin;
        out.reason = diag.reason;
        out.worst_link_name = diag.worst_link_name;
        out.q_solution = diag.q_solution;
        return out;
    };
}

bool WholeBodyEllipsoidPoseValidator::solveIk(
    const Eigen::Vector3d& p_target,
    const Eigen::Matrix3d& R_target,
    const std::optional<Eigen::VectorXd>& q_seed,
    Eigen::VectorXd& q_solution,
    IkSolveDiagnostic* ik_diag) const {
    IkSolveDiagnostic local_diag;
    if (ik_diag == nullptr) {
        ik_diag = &local_diag;
    }

    if (!cfg_.ik_solver_fn) {
        return false;
    }
    if (cfg_.ik_solver_fn(p_target, R_target, q_seed, q_solution)) {
        ik_diag->final_ok = true;
        ik_diag->external_ik_ok = true;
        return true;
    }
    return false;
}

bool WholeBodyEllipsoidPoseValidator::isWholeBodyCollisionFree(
    const Eigen::VectorXd& q,
    const double safe_distance) const {
    if (!distance_field_ || !fk_provider_) {
        return false;
    }
    arm_controller::kinematics::ForwardKinematicsOutput fk_out;
    if (!fk_provider_->compute(q, fk_out)) {
        return false;
    }

    for (const auto& e : link_ellipsoids_) {
        const auto it = fk_out.link_poses.find(e.link_name);
        if (it == fk_out.link_poses.end()) {
            continue;
        }
        const Eigen::Isometry3d& T_world_link = it->second;
        const Eigen::Vector3d p_world = T_world_link * e.center_in_link;
        if (!distance_field_->isInsideMap(p_world)) {
            return false;
        }
        const double d = distance_field_->getDistance(p_world);
        Eigen::Vector3d n_world = distance_field_->getGradient(p_world);
        if (!std::isfinite(d) || !n_world.allFinite()) {
            return false;
        }
        const double gn = n_world.norm();
        if (gn < 1e-9) {
            return false;
        }
        n_world /= gn;
        const double r_eff = effectiveEllipsoidRadiusAlongNormal(
            T_world_link.linear(), e.radii, n_world);
        if (!((d - (safe_distance + r_eff)) >= cfg_.collision_blocking_margin_m)) {
            return false;
        }
    }
    return true;
}

WholeBodyEllipsoidPoseValidator::PoseCacheKey
WholeBodyEllipsoidPoseValidator::makePoseCacheKey(
    const Eigen::Vector3d& p_target,
    const Eigen::Matrix3d& R_target,
    const double safe_distance) const {
    const double pos_res = std::max(1e-6, cfg_.pose_cache_position_resolution_m);
    const double quat_res = std::max(1e-6, cfg_.pose_cache_quaternion_resolution);
    Eigen::Quaterniond q(R_target);
    q.normalize();
    if (q.w() < 0.0) {
        q.coeffs() *= -1.0;
    }
    auto qbin = [](double v, double res) -> int {
        return static_cast<int>(std::llround(v / res));
    };
    return PoseCacheKey{
        qbin(p_target.x(), pos_res),
        qbin(p_target.y(), pos_res),
        qbin(p_target.z(), pos_res),
        qbin(q.x(), quat_res),
        qbin(q.y(), quat_res),
        qbin(q.z(), quat_res),
        qbin(q.w(), quat_res),
        qbin(safe_distance, pos_res),
    };
}

bool WholeBodyEllipsoidPoseValidator::lookupPoseCache(
    const PoseCacheKey& key,
    Eigen::VectorXd& q_solution) const {
    std::lock_guard<std::mutex> lock(pose_cache_mutex_);
    const auto it = pose_cache_.find(key);
    if (it == pose_cache_.end()) {
        return false;
    }
    q_solution = it->second.q_solution;
    return true;
}

void WholeBodyEllipsoidPoseValidator::storePoseCache(
    const PoseCacheKey& key,
    const Eigen::VectorXd& q_solution) const {
    std::lock_guard<std::mutex> lock(pose_cache_mutex_);
    if (cfg_.pose_cache_max_entries > 0 &&
        pose_cache_.size() >= cfg_.pose_cache_max_entries) {
        pose_cache_.clear();
    }
    pose_cache_[key] = PoseCacheValue{q_solution};
}

WholeBodyEllipsoidPoseValidator::SegmentCacheKey
WholeBodyEllipsoidPoseValidator::makeSegmentCacheKey(
    const CartesianWaypoint& from,
    const CartesianWaypoint& to,
    const double safe_distance) const {
    return SegmentCacheKey{
        makePoseCacheKey(from.position, from.orientation, safe_distance),
        makePoseCacheKey(to.position, to.orientation, safe_distance),
    };
}

bool WholeBodyEllipsoidPoseValidator::lookupSegmentCache(
    const SegmentCacheKey& key,
    Eigen::VectorXd& q_end) const {
    std::lock_guard<std::mutex> lock(segment_cache_mutex_);
    const auto it = segment_cache_.find(key);
    if (it == segment_cache_.end()) {
        return false;
    }
    q_end = it->second.q_end;
    return true;
}

void WholeBodyEllipsoidPoseValidator::storeSegmentCache(
    const SegmentCacheKey& key,
    const Eigen::VectorXd& q_end) const {
    std::lock_guard<std::mutex> lock(segment_cache_mutex_);
    if (cfg_.segment_cache_max_entries > 0 &&
        segment_cache_.size() >= cfg_.segment_cache_max_entries) {
        segment_cache_.clear();
    }
    segment_cache_[key] = SegmentCacheValue{q_end};
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
