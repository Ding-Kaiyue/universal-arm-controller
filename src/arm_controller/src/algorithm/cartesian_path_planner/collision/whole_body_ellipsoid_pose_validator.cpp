#include "algorithm/cartesian_path_planner/collision/whole_body_ellipsoid_pose_validator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <sstream>

namespace arm_controller::algorithm::cartesian_path_planner {

namespace {

constexpr double kGradientNormEps = 1e-9;
constexpr double kJointSegmentStepRad = 0.10;

Eigen::Vector3d rotationToRpyDeg(const Eigen::Matrix3d& R) {
    return R.eulerAngles(0, 1, 2) * (180.0 / M_PI);
}

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

double conservativeEllipsoidRadius(const Eigen::Vector3d& radii_link) {
    return radii_link.maxCoeff();
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
    const bool use_cache = shouldUseSeedAgnosticCache(q_seed);
    const PoseCacheKey cache_key = makePoseCacheKey(p_target, R_target, safe_distance);
    if (use_cache && lookupPoseCache(cache_key, q_solution)) {
        return true;
    }
    if (!solveIk(p_target, R_target, q_seed, q_solution)) {
        return false;
    }
    if (!isWholeBodyCollisionFree(q_solution, safe_distance)) {
        return false;
    }
    if (use_cache) {
        storePoseCache(cache_key, q_solution);
    }
    return true;
}

bool WholeBodyEllipsoidPoseValidator::validateSegment(
    const CartesianWaypoint& from,
    const CartesianWaypoint& to,
    const double safe_distance,
    const std::optional<Eigen::VectorXd>& q_seed,
    Eigen::VectorXd& q_end,
    PoseDiagnostic* failed_diag) const {
    const bool use_cache = shouldUseSeedAgnosticCache(q_seed);
    const SegmentCacheKey cache_key = makeSegmentCacheKey(from, to, safe_distance);
    if (use_cache && lookupSegmentCache(cache_key, q_end)) {
        return true;
    }

    const Eigen::Quaterniond R_from_q(from.orientation);
    const Eigen::Quaterniond R_to_q(to.orientation);
    auto interpolatePose = [&](const double t) {
        CartesianWaypoint pose;
        pose.position = (1.0 - t) * from.position + t * to.position;
        pose.orientation = R_from_q.slerp(t, R_to_q).normalized().toRotationMatrix();
        return pose;
    };
    auto failSegment = [&](PoseDiagnostic diag,
                           const int step_index,
                           const int total_steps,
                           const double t,
                           const CartesianWaypoint& fallback_pose) {
        diag.has_failed_pose = true;
        diag.failed_on_segment_sample = true;
        diag.failed_segment_t = t;
        if (!diag.failed_pose_world.allFinite()) {
            diag.failed_pose_world = fallback_pose.position;
        }
        if (!diag.failed_pose_orientation.allFinite()) {
            diag.failed_pose_orientation = fallback_pose.orientation;
        }
        if (failed_diag != nullptr) {
            *failed_diag = diag;
        }

        const bool has_actual_pose =
            diag.reason != "invalid_q" && diag.reason != "invalid_validator" &&
            diag.reason != "fk_fail" && diag.failed_pose_world.allFinite() &&
            diag.failed_pose_orientation.allFinite();
        const Eigen::Vector3d pose =
            has_actual_pose ? diag.failed_pose_world : fallback_pose.position;
        const Eigen::Matrix3d orientation =
            has_actual_pose ? diag.failed_pose_orientation : fallback_pose.orientation;
        const Eigen::Vector3d rpy_deg = rotationToRpyDeg(orientation);
        std::ostringstream oss;
        oss << "[segment_validator] step " << step_index << " / " << total_steps
            << " t=" << t
            << " pose=(" << pose.x() << ", " << pose.y() << ", " << pose.z() << ")"
            << " rpy_deg=(" << rpy_deg.x() << ", " << rpy_deg.y() << ", " << rpy_deg.z() << ")";
        if (!diag.ik_ok) {
            oss << " reason=ik_fail";
        } else if (!diag.collision_free) {
            oss << " reason=" << (diag.reason.empty() ? "collision_fail" : diag.reason)
                << " min_margin=" << diag.min_margin;
            if (!diag.worst_link_name.empty()) {
                oss << " worst_link=" << diag.worst_link_name;
            }
            oss << " worst_point=("
                << diag.worst_point_world.x() << ", "
                << diag.worst_point_world.y() << ", "
                << diag.worst_point_world.z() << ")"
                << " worst_distance=" << diag.worst_distance
                << " r_eff=" << diag.worst_effective_radius
                << " safe_distance=" << diag.safe_distance_used
                << " required_clearance=" << diag.required_clearance
                << " worst_gradient_norm=" << diag.worst_gradient_norm;
        } else {
            oss << " reason=segment_fail";
        }
        std::cout << oss.str() << std::endl;
        return false;
    };

    Eigen::VectorXd q_from;
    if (q_seed.has_value() && q_seed->size() > 0) {
        q_from = *q_seed;
        PoseDiagnostic start_diag = diagnoseConfiguration(q_from, safe_distance);
        if (!start_diag.collision_free) {
            start_diag.failed_pose_world = from.position;
            start_diag.failed_pose_orientation = from.orientation;
            return failSegment(start_diag, 0, 1, 0.0, from);
        }
    } else {
        PoseDiagnostic start_diag =
            diagnosePose(from.position, from.orientation, safe_distance, q_seed);
        if (!start_diag.ik_ok || !start_diag.collision_free) {
            start_diag.failed_pose_world = from.position;
            start_diag.failed_pose_orientation = from.orientation;
            return failSegment(start_diag, 0, 1, 0.0, from);
        }
        q_from = start_diag.q_solution;
    }

    IkSolveDiagnostic ik_diag;
    Eigen::VectorXd q_to;
    if (!solveIk(to.position, to.orientation, q_from, q_to, &ik_diag)) {
        PoseDiagnostic diag;
        diag.ik_ok = false;
        diag.external_ik_ok = ik_diag.external_ik_ok;
        diag.fallback_ik_used = ik_diag.fallback_ik_used;
        diag.collision_free = false;
        diag.min_margin = -1.0;
        diag.reason = "ik_fail";
        diag.failed_pose_world = to.position;
        diag.failed_pose_orientation = to.orientation;
        return failSegment(diag, 1, 1, 1.0, to);
    }

    PoseDiagnostic goal_diag = diagnoseConfiguration(q_to, safe_distance);
    goal_diag.ik_ok = true;
    goal_diag.external_ik_ok = ik_diag.external_ik_ok;
    goal_diag.fallback_ik_used = ik_diag.fallback_ik_used;
    goal_diag.q_solution = q_to;
    if (!goal_diag.collision_free) {
        return failSegment(goal_diag, 1, 1, 1.0, to);
    }

    const double cartesian_dist = (to.position - from.position).norm();
    const double check_step_m = std::max(1e-3, cfg_.segment_check_step_m);
    const int cartesian_steps =
        std::max(cfg_.segment_substeps_min, static_cast<int>(std::ceil(cartesian_dist / check_step_m)));
    const double max_joint_delta =
        (q_to.size() == q_from.size()) ? (q_to - q_from).cwiseAbs().maxCoeff() : 0.0;
    const int joint_steps = std::max(1, static_cast<int>(std::ceil(max_joint_delta / kJointSegmentStepRad)));
    const int steps = std::max(cartesian_steps, joint_steps);

    for (int i = 1; i < steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Eigen::VectorXd q_interp = (1.0 - t) * q_from + t * q_to;
        PoseDiagnostic diag = diagnoseConfiguration(q_interp, safe_distance);
        diag.q_solution = q_interp;
        if (!diag.collision_free) {
            return failSegment(diag, i, steps, t, interpolatePose(t));
        }
    }

    q_end = q_to;
    if (use_cache) {
        storeSegmentCache(cache_key, q_end);
    }
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
    PoseDiagnostic q_diag = diagnoseConfiguration(diag.q_solution, safe_distance);
    q_diag.ik_ok = true;
    q_diag.external_ik_ok = ik_diag.external_ik_ok;
    q_diag.fallback_ik_used = ik_diag.fallback_ik_used;
    q_diag.q_solution = diag.q_solution;
    return q_diag;
}

void WholeBodyEllipsoidPoseValidator::fillPlanningDiagnostic(
    const PoseDiagnostic& in,
    PathPlanningInput::WholeBodyPoseDiagnostic& out) {
    out.ik_ok = in.ik_ok;
    out.external_ik_ok = in.external_ik_ok;
    out.fallback_ik_used = in.fallback_ik_used;
    out.collision_free = in.collision_free;
    out.min_margin = in.min_margin;
    out.reason = in.reason;
    out.worst_link_name = in.worst_link_name;
    out.worst_point_world = in.worst_point_world;
    out.worst_distance = in.worst_distance;
    out.worst_effective_radius = in.worst_effective_radius;
    out.required_clearance = in.required_clearance;
    out.safe_distance_used = in.safe_distance_used;
    out.worst_gradient_norm = in.worst_gradient_norm;
    out.worst_gradient_world = in.worst_gradient_world;
    out.has_failed_pose = in.has_failed_pose;
    out.failed_on_segment_sample = in.failed_on_segment_sample;
    out.failed_segment_t = in.failed_segment_t;
    out.failed_pose_world = in.failed_pose_world;
    out.failed_pose_orientation = in.failed_pose_orientation;
    out.q_solution = in.q_solution;
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
               Eigen::VectorXd& q_end,
               PathPlanningInput::WholeBodyPoseDiagnostic* failed_diag) {
        PoseDiagnostic diag;
        const bool ok = this->validateSegment(
            from, to, safe_distance, q_seed, q_end, failed_diag != nullptr ? &diag : nullptr);
        if (!ok && failed_diag != nullptr) {
            fillPlanningDiagnostic(diag, *failed_diag);
        }
        return ok;
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
        fillPlanningDiagnostic(diag, out);
        return out;
    };
}

PathPlanningInput::JointStateValidatorFn
WholeBodyEllipsoidPoseValidator::makeJointStateValidatorFn() const {
    return [this](
               const Eigen::VectorXd& q,
               const double safe_distance,
               PathPlanningInput::WholeBodyPoseDiagnostic* diag_out) {
        const PoseDiagnostic diag = this->diagnoseJointState(q, safe_distance);
        if (diag_out != nullptr) {
            fillPlanningDiagnostic(diag, *diag_out);
        }
        return diag.collision_free;
    };
}

PathPlanningInput::JointSegmentValidatorFn
WholeBodyEllipsoidPoseValidator::makeJointSegmentValidatorFn() const {
    return [this](
               const Eigen::VectorXd& q_from,
               const Eigen::VectorXd& q_to,
               const double safe_distance,
               PathPlanningInput::WholeBodyPoseDiagnostic* diag_out) {
        if (q_from.size() == 0 || q_to.size() == 0 ||
            q_from.size() != q_to.size() || !q_from.allFinite() || !q_to.allFinite()) {
            PoseDiagnostic diag;
            diag.ik_ok = true;
            diag.collision_free = false;
            diag.min_margin = -1.0;
            diag.reason = "invalid_q_segment";
            if (diag_out != nullptr) {
                fillPlanningDiagnostic(diag, *diag_out);
            }
            return false;
        }

        const PoseDiagnostic start_diag = this->diagnoseJointState(q_from, safe_distance);
        if (!start_diag.collision_free) {
            if (diag_out != nullptr) {
                fillPlanningDiagnostic(start_diag, *diag_out);
            }
            return false;
        }

        const double max_joint_delta = (q_to - q_from).cwiseAbs().maxCoeff();
        const int steps = std::max(
            1,
            static_cast<int>(std::ceil(max_joint_delta / kJointSegmentStepRad)));
        for (int i = 1; i <= steps; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(steps);
            const Eigen::VectorXd q_interp = (1.0 - t) * q_from + t * q_to;
            const PoseDiagnostic diag = this->diagnoseJointState(q_interp, safe_distance);
            if (!diag.collision_free) {
                if (diag_out != nullptr) {
                    fillPlanningDiagnostic(diag, *diag_out);
                }
                return false;
            }
        }
        return true;
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

WholeBodyEllipsoidPoseValidator::PoseDiagnostic
WholeBodyEllipsoidPoseValidator::diagnoseConfiguration(
    const Eigen::VectorXd& q,
    const double safe_distance) const {
    PoseDiagnostic diag;
    diag.ik_ok = true;
    diag.external_ik_ok = true;
    diag.collision_free = true;
    diag.min_margin = std::numeric_limits<double>::infinity();
    diag.safe_distance_used = safe_distance;
    diag.reason = "ok";
    diag.q_solution = q;

    if (q.size() == 0 || !q.allFinite()) {
        diag.collision_free = false;
        diag.min_margin = -1.0;
        diag.reason = "invalid_q";
        return diag;
    }
    if (!distance_field_ || !fk_provider_) {
        diag.collision_free = false;
        diag.min_margin = -1.0;
        diag.reason = "invalid_validator";
        return diag;
    }

    arm_controller::kinematics::ForwardKinematicsOutput fk_out;
    if (!fk_provider_->compute(q, fk_out)) {
        diag.collision_free = false;
        diag.min_margin = -1.0;
        diag.reason = "fk_fail";
        return diag;
    }

    diag.failed_pose_world = fk_out.ee_position;
    diag.failed_pose_orientation = fk_out.ee_rotation;

    std::vector<Eigen::Vector3d> query_points;
    std::vector<const LinkCollisionEllipsoid*> query_ellipsoids;
    std::vector<const Eigen::Isometry3d*> query_link_poses;
    query_points.reserve(link_ellipsoids_.size());
    query_ellipsoids.reserve(link_ellipsoids_.size());
    query_link_poses.reserve(link_ellipsoids_.size());

    for (const auto& e : link_ellipsoids_) {
        const auto it = fk_out.link_poses.find(e.link_name);
        if (it == fk_out.link_poses.end()) {
            continue;
        }
        const Eigen::Isometry3d& T_world_link = it->second;
        query_points.push_back(T_world_link * e.center_in_link);
        query_ellipsoids.push_back(&e);
        query_link_poses.push_back(&T_world_link);
    }

    const std::vector<DistanceFieldQueryResult> queries =
        distance_field_->queryDistanceAndGradientBatch(query_points);
    for (std::size_t i = 0; i < query_ellipsoids.size(); ++i) {
        const auto& e = *query_ellipsoids[i];
        const Eigen::Isometry3d& T_world_link = *query_link_poses[i];
        const Eigen::Vector3d& p_world = query_points[i];
        const DistanceFieldQueryResult& query = queries[i];
        if (!query.observed) {
            continue;
        }

        const double d = query.distance;
        const Eigen::Vector3d raw_gradient =
            query.gradient_valid ? query.gradient : Eigen::Vector3d::Zero();
        const double gn = raw_gradient.norm();
        if (!query.distance_valid || !std::isfinite(d)) {
            diag.collision_free = false;
            diag.min_margin = -1.0;
            diag.reason = "invalid_distance_field";
            diag.worst_link_name = e.debug_name.empty() ? e.link_name : e.debug_name;
            diag.worst_point_world = p_world;
            diag.worst_distance = d;
            diag.worst_gradient_norm = gn;
            diag.worst_gradient_world = raw_gradient;
            return diag;
        }

        double r_eff = conservativeEllipsoidRadius(e.radii);
        if (query.gradient_valid && gn >= kGradientNormEps) {
            const Eigen::Vector3d n_world = raw_gradient / gn;
            r_eff = effectiveEllipsoidRadiusAlongNormal(
                T_world_link.linear(), e.radii, n_world);
        }

        const double margin = d - (safe_distance + r_eff);
        if (margin < diag.min_margin) {
            diag.min_margin = margin;
            diag.worst_link_name = e.debug_name.empty() ? e.link_name : e.debug_name;
            diag.worst_point_world = p_world;
            diag.worst_distance = d;
            diag.worst_effective_radius = r_eff;
            diag.required_clearance = safe_distance + r_eff;
            diag.safe_distance_used = safe_distance;
            diag.worst_gradient_norm = gn;
            diag.worst_gradient_world = raw_gradient;
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

WholeBodyEllipsoidPoseValidator::PoseDiagnostic
WholeBodyEllipsoidPoseValidator::diagnoseJointState(
    const Eigen::VectorXd& q,
    const double safe_distance) const {
    return diagnoseConfiguration(q, safe_distance);
}

bool WholeBodyEllipsoidPoseValidator::isWholeBodyCollisionFree(
    const Eigen::VectorXd& q,
    const double safe_distance) const {
    return diagnoseConfiguration(q, safe_distance).collision_free;
}

bool WholeBodyEllipsoidPoseValidator::shouldUseSeedAgnosticCache(
    const std::optional<Eigen::VectorXd>& q_seed) const {
    return !q_seed.has_value() || q_seed->size() == 0;
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
