#include "controller/reactive_task/reactive_task_controller.hpp"
#include "algorithm/cartesian_path_planner/collision/base_footprint_collision_checker.hpp"
#include "algorithm/cartesian_path_planner/map/composite_distance_field.hpp"
#include "algorithm/cartesian_path_planner/map/static_pillar_distance_field.hpp"
#include "algorithm/global_planner/base_guided_whole_body_planner.hpp"
#include "controller/reactive_task/goal/whole_body_goal_generator.hpp"
#include "controller/reactive_task/local_planner/reactive_task_whole_body_local_planner.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <thread>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace gp = arm_controller::algorithm::global_planner;
namespace rq = arm_controller::algorithm::reactive_qp;
namespace rt = arm_controller::controller::reactive_task;
namespace tpi = trajectory_planning::infrastructure::integration;

namespace {
constexpr int kBaseDof = 3;
constexpr int kArmDof = 12;
constexpr int kFullDof = kBaseDof + kArmDof;
constexpr double kBaseCandidateReachM = 0.32;
constexpr double kBaseBoundsMarginM = 1.50;
constexpr double kSegmentBaseStepM = 0.08;
constexpr double kSegmentYawStepRad = 0.20;
constexpr double kSegmentJointStepRad = 0.18;
constexpr double kWholeBodyTrackingKpBaseXy = 0.80;
constexpr double kWholeBodyTrackingKpBaseYaw = 1.20;
constexpr double kWholeBodyTrackingKpArm = 1.50;
constexpr double kWholeBodyTrackingMaxArmQdot = 0.45;
constexpr double kWholeBodyBaseMaxAx = 0.45;
constexpr double kWholeBodyBaseMaxAy = 0.45;
constexpr double kWholeBodyBaseMaxAwz = 0.90;
constexpr double kEsdfReadyWaitTimeoutSec = 2.0;
constexpr double kEsdfReadyPollPeriodSec = 0.02;
constexpr std::size_t kMaxWholeBodyGoalCandidates = 4;
constexpr std::size_t kMaxWholeBodyGoalSeeds = 36;
constexpr double kSelfNearestRejectPaddingM = 0.05;
constexpr double kWholeBodyLocalObstacleInfluenceM = 0.35;
constexpr double kWholeBodyLocalObstacleSafeMarginM = 0.18;
constexpr double kWholeBodyLocalPointObstacleRadiusM = 0.02;
constexpr int kWholeBodyLocalMinHorizonSteps = 24;

struct BaseState {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

struct WholeBodyReferenceSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BaseState base;
    Eigen::VectorXd arm;
    Eigen::VectorXd full_state;
    double time_from_start{0.0};
};

struct WholeBodyTrackingCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d base_body_twist{Eigen::Vector3d::Zero()};
    Eigen::VectorXd arm_qdot;
};

struct WholeBodyQpVelocityBounds {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::VectorXd qd_min;
    Eigen::VectorXd qd_max;
};

struct WholeBodyQpCollisionContext {
    const arm_controller::kinematics::PinocchioForwardKinematics* fk_provider{nullptr};
    const arm_controller::kinematics::JacobianProvider* jacobian_provider{nullptr};
    const rq::LinkCollisionEllipsoidList* collision_ellipsoids{nullptr};
};

struct EllipsoidObstacleSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d point_in_link{Eigen::Vector3d::Zero()};
    double radius{0.0};
    double weight{1.0};
};

struct ArmIkContext {
    std::shared_ptr<tpi::MoveItAdapter> moveit;
    std::shared_ptr<tpi::TracIKAdapter> tracik;
    std::string base_link;
    std::string tip_link;
    bool ready{false};
};

bool waitForCameraDriverEsdfSnapshot(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<const cp::CameraDriverEsdfMapClient>& esdf_map,
    const double timeout_sec) {
    if (!esdf_map) {
        return false;
    }
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(std::max(0.0, timeout_sec));
    while (std::chrono::steady_clock::now() < deadline) {
        if (esdf_map->isMapReady()) {
            return true;
        }
        std::this_thread::sleep_for(
            std::chrono::duration<double>(kEsdfReadyPollPeriodSec));
    }
    if (esdf_map->isMapReady()) {
        return true;
    }
    if (node) {
        RCLCPP_WARN(
            node->get_logger(),
            "[ReactiveTask] camera_driver ESDF SHM snapshot is still not ready after %.2fs: processed_frames=%d successful_queries=%zu failed_queries=%zu",
            timeout_sec,
            esdf_map->processedFrames(),
            esdf_map->successfulQueries(),
            esdf_map->failedQueries());
    }
    return false;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
makeBaseCollisionLocalSamples(
    const rq::LinkCollisionEllipsoidList* collision_ellipsoids) {
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> samples;
    if (collision_ellipsoids == nullptr) {
        return samples;
    }

    for (const rq::LinkCollisionEllipsoid& ellipsoid : *collision_ellipsoids) {
        if (ellipsoid.link_name != "base_link" || !ellipsoid.radii.allFinite()) {
            continue;
        }
        const Eigen::Vector3d radii =
            ellipsoid.radii.cwiseMax(Eigen::Vector3d::Constant(1e-4));
        const std::array<Eigen::Vector3d, 9> offsets = {
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(radii.x(), 0.0, 0.0),
            Eigen::Vector3d(-radii.x(), 0.0, 0.0),
            Eigen::Vector3d(0.0, radii.y(), 0.0),
            Eigen::Vector3d(0.0, -radii.y(), 0.0),
            Eigen::Vector3d(radii.x(), radii.y(), 0.0),
            Eigen::Vector3d(radii.x(), -radii.y(), 0.0),
            Eigen::Vector3d(-radii.x(), radii.y(), 0.0),
            Eigen::Vector3d(-radii.x(), -radii.y(), 0.0),
        };
        for (const Eigen::Vector3d& offset : offsets) {
            samples.push_back(ellipsoid.center_in_link + offset);
        }
        break;
    }
    return samples;
}

Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose& pose) {
    Eigen::Quaterniond q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    if (q.norm() < 1e-8) {
        q = Eigen::Quaterniond::Identity();
    } else {
        q.normalize();
    }

    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() =
        Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    t.linear() = q.toRotationMatrix();
    return t;
}

geometry_msgs::msg::Pose isoToPoseMsg(const Eigen::Isometry3d& t) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = t.translation().x();
    pose.position.y = t.translation().y();
    pose.position.z = t.translation().z();
    const Eigen::Quaterniond q(t.linear());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

Eigen::Isometry3d baseStateToIso(const BaseState& base) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(base.x, base.y, 0.0);
    t.linear() =
        Eigen::AngleAxisd(base.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return t;
}

BaseState baseStateFromQ(const Eigen::VectorXd& q) {
    BaseState base;
    if (q.size() >= kBaseDof) {
        base.x = q[0];
        base.y = q[1];
        base.yaw = q[2];
    }
    return base;
}

Eigen::VectorXd armStateFromFullState(const Eigen::VectorXd& q_full) {
    if (q_full.size() != kFullDof) {
        return {};
    }
    return q_full.segment(kBaseDof, kArmDof);
}

std::string formatVector(const Eigen::VectorXd& v) {
    std::ostringstream oss;
    oss << v.transpose().format(Eigen::IOFormat(4, 0, ", ", ", ", "[", "]"));
    return oss.str();
}

void logWholeBodyGlobalArmReference(
    rclcpp::Logger logger,
    const cp::TimedJointTrajectory& trajectory) {
    constexpr std::size_t kPrintEvery = 5;
    constexpr double kSignificantArmDeltaRad = 0.15;
    if (trajectory.joint_targets.empty()) {
        return;
    }
    RCLCPP_INFO(
        logger,
        "[dual_arm] global arm reference dump begin: states=%zu print_every=%zu significant_dq=%.3f",
        trajectory.joint_targets.size(),
        kPrintEvery,
        kSignificantArmDeltaRad);
    for (std::size_t i = 0; i < trajectory.joint_targets.size(); ++i) {
        const Eigen::VectorXd& q = trajectory.joint_targets[i];
        if (q.size() != kFullDof) {
            continue;
        }
        const double time =
            i < trajectory.cumulative_times.size()
                ? trajectory.cumulative_times[i]
                : 0.0;
        double dq_norm = 0.0;
        bool significant_delta = false;
        if (i > 0 && trajectory.joint_targets[i - 1u].size() == q.size()) {
            dq_norm =
                (q.segment(kBaseDof, kArmDof) -
                 trajectory.joint_targets[i - 1u].segment(kBaseDof, kArmDof))
                    .norm();
            significant_delta = dq_norm > kSignificantArmDeltaRad;
        }
        const bool print_sample =
            i == 0 || i + 1u == trajectory.joint_targets.size() ||
            i % kPrintEvery == 0u || significant_delta;
        if (!print_sample) {
            continue;
        }
        RCLCPP_INFO(
            logger,
            "[dual_arm] global_arm_ref[%zu] t=%.3f base=(%.3f, %.3f, %.3f) dq_norm=%.4f arm=%s",
            i,
            time,
            q[0],
            q[1],
            q[2],
            dq_norm,
            formatVector(q.segment(kBaseDof, kArmDof)).c_str());
    }
    RCLCPP_INFO(logger, "[dual_arm] global arm reference dump end");
}

std::optional<BaseState> readBaseStateFromTf(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& odom_frame,
    const std::string& base_frame,
    rclcpp::Logger logger) {
    try {
        const geometry_msgs::msg::TransformStamped tf =
            tf_buffer.lookupTransform(
                odom_frame,
                base_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(0.10));

        tf2::Quaternion q;
        tf2::fromMsg(tf.transform.rotation, q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        BaseState base;
        base.x = tf.transform.translation.x;
        base.y = tf.transform.translation.y;
        base.yaw = normalizeAngle(yaw);
        return base;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(
            logger,
            "[dual_arm] failed to read mobile base TF %s -> %s: %s",
            odom_frame.c_str(),
            base_frame.c_str(),
            ex.what());
        return std::nullopt;
    }
}

std::vector<BaseState> makeBaseGoalCandidates(
    const BaseState& start,
    const Eigen::Vector3d& left_goal,
    const Eigen::Vector3d& right_goal) {
    const Eigen::Vector2d target_mid =
        0.5 * (left_goal.head<2>() + right_goal.head<2>());
    const Eigen::Vector2d from_start =
        target_mid - Eigen::Vector2d(start.x, start.y);
    const double nominal_yaw =
        (from_start.norm() > 1e-4) ? std::atan2(from_start.y(), from_start.x())
                                   : start.yaw;

    std::vector<BaseState> candidates;
    candidates.push_back(start);

    const std::vector<double> yaw_offsets = {0.0, 0.35, -0.35, 0.70, -0.70,
                                             M_PI, M_PI + 0.35, M_PI - 0.35};
    const std::vector<double> range_offsets =
        {kBaseCandidateReachM, 0.24, 0.40, 0.55, 0.70};
    const std::vector<double> lateral_offsets =
        {0.0, 0.12, -0.12, 0.24, -0.24, 0.40, -0.40};

    for (const double yaw_offset : yaw_offsets) {
        const double yaw = normalizeAngle(nominal_yaw + yaw_offset);
        const Eigen::Vector2d forward(std::cos(yaw), std::sin(yaw));
        const Eigen::Vector2d lateral(-std::sin(yaw), std::cos(yaw));
        for (const double range : range_offsets) {
            for (const double lateral_offset : lateral_offsets) {
                const Eigen::Vector2d xy =
                    target_mid - range * forward + lateral_offset * lateral;
                candidates.push_back(BaseState{xy.x(), xy.y(), yaw});
            }
        }
    }

    return candidates;
}

void sortAndLimitBaseGoalCandidates(
    std::vector<BaseState>& candidates,
    const Eigen::Vector2d& target_mid) {
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [&target_mid](const BaseState& a, const BaseState& b) {
            const double da =
                (Eigen::Vector2d(a.x, a.y) - target_mid).squaredNorm();
            const double db =
                (Eigen::Vector2d(b.x, b.y) - target_mid).squaredNorm();
            const double ya = std::abs(normalizeAngle(a.yaw));
            const double yb = std::abs(normalizeAngle(b.yaw));
            return std::tie(da, ya) < std::tie(db, yb);
        });
    if (candidates.size() > kMaxWholeBodyGoalSeeds) {
        candidates.resize(kMaxWholeBodyGoalSeeds);
    }
}

bool initializeArmIkContext(
    const rclcpp::Node::SharedPtr& node,
    const std::string& group,
    const std::string& urdf_xml,
    ArmIkContext* out,
    std::string* error) {
    if (out == nullptr) {
        return false;
    }

    out->moveit = std::make_shared<tpi::MoveItAdapter>(
        node, group, "reactive_task_whole_body");
    out->tracik = std::make_shared<tpi::TracIKAdapter>(node, group);
    if (!out->moveit || !out->tracik) {
        if (error != nullptr) {
            *error = group + " adapter allocation failed";
        }
        return false;
    }

    out->tracik->setMoveItAdapter(out->moveit.get());
    out->base_link = out->moveit->getBaseLink();
    out->tip_link = out->moveit->getEndEffectorLink();
    if (out->base_link.empty() || out->tip_link.empty()) {
        if (error != nullptr) {
            *error = group + " missing base/tip link";
        }
        return false;
    }

    const bool kdl_ok =
        out->tracik->initializeKDLChain(urdf_xml, out->base_link, out->tip_link);
    const bool solver_ok = kdl_ok && out->tracik->initializeSolver("dual_arm620");
    out->ready = kdl_ok && solver_ok;
    if (!out->ready && error != nullptr) {
        *error = group + " TRAC-IK initialization failed";
    }
    return out->ready;
}

bool solveArmIkForBaseCandidate(
    const ArmIkContext& ctx,
    const Eigen::Isometry3d& T_world_base,
    const Eigen::Isometry3d& T_base_arm_mount,
    const Eigen::Isometry3d& T_world_goal,
    const std::vector<double>& seed,
    Eigen::VectorXd* q_solution,
    const Eigen::Isometry3d* T_world_relaxed_goal = nullptr,
    bool* used_relaxed_goal = nullptr) {
    if (!ctx.ready || !ctx.tracik || q_solution == nullptr ||
        seed.size() != 6u) {
        return false;
    }
    if (used_relaxed_goal != nullptr) {
        *used_relaxed_goal = false;
    }

    const Eigen::Isometry3d T_world_arm_base =
        T_world_base * T_base_arm_mount;
    const Eigen::Isometry3d T_arm_base_goal =
        T_world_arm_base.inverse() * T_world_goal;

    static std::atomic<int> debug_print_count{0};
    const int debug_index = debug_print_count.fetch_add(1);
    if (debug_index < 8) {
        const Eigen::Vector3d p = T_arm_base_goal.translation();
        RCLCPP_INFO(
            rclcpp::get_logger("reactive_task_dual_arm_ik"),
            "IK target in %s: p=(%.3f, %.3f, %.3f) norm=%.3f",
            ctx.base_link.c_str(),
            p.x(),
            p.y(),
            p.z(),
            p.norm());
    }

    std::vector<double> q_vec;
    bool ok = ctx.tracik->computeIKClosest(
                  isoToPoseMsg(T_arm_base_goal), seed, q_vec, 5, false) &&
              q_vec.size() == 6u;
    if (!ok && T_world_relaxed_goal != nullptr) {
        const Eigen::Isometry3d T_arm_base_relaxed_goal =
            T_world_arm_base.inverse() * (*T_world_relaxed_goal);
        ok = ctx.tracik->computeIKClosest(
                 isoToPoseMsg(T_arm_base_relaxed_goal),
                 seed,
                 q_vec,
                 5,
                 false) &&
             q_vec.size() == 6u;
        if (ok && used_relaxed_goal != nullptr) {
            *used_relaxed_goal = true;
        }
    }
    if (!ok) {
        return false;
    }

    *q_solution = Eigen::Map<const Eigen::VectorXd>(
        q_vec.data(), static_cast<Eigen::Index>(q_vec.size()));
    return true;
}

double ellipsoidSupportRadius(
    const Eigen::Vector3d& radii,
    const Eigen::Matrix3d& R_world_link,
    const Eigen::Vector3d& normal_world) {
    if (normal_world.norm() < 1e-8) {
        return radii.maxCoeff();
    }
    const Eigen::Vector3d n_link = R_world_link.transpose() * normal_world.normalized();
    return std::sqrt(
        std::pow(radii.x() * n_link.x(), 2) +
        std::pow(radii.y() * n_link.y(), 2) +
        std::pow(radii.z() * n_link.z(), 2));
}

std::vector<EllipsoidObstacleSample, Eigen::aligned_allocator<EllipsoidObstacleSample>>
makeEllipsoidObstacleSamples(const rq::LinkCollisionEllipsoid& ellipsoid) {
    std::vector<EllipsoidObstacleSample, Eigen::aligned_allocator<EllipsoidObstacleSample>>
        samples;
    if (!ellipsoid.radii.allFinite()) {
        return samples;
    }
    const Eigen::Vector3d radii =
        ellipsoid.radii.cwiseMax(Eigen::Vector3d::Constant(1e-4));
    const double min_radius = radii.minCoeff();
    const double point_radius =
        std::clamp(0.35 * min_radius, kWholeBodyLocalPointObstacleRadiusM, min_radius);

    auto push_sample = [&samples](
                           const Eigen::Vector3d& point_in_link,
                           const double radius,
                           const double weight) {
        EllipsoidObstacleSample sample;
        sample.point_in_link = point_in_link;
        sample.radius = radius;
        sample.weight = weight;
        samples.push_back(sample);
    };

    push_sample(ellipsoid.center_in_link, -1.0, 1.0);

    const std::array<Eigen::Vector3d, 6> axes = {
        Eigen::Vector3d(radii.x(), 0.0, 0.0),
        Eigen::Vector3d(-radii.x(), 0.0, 0.0),
        Eigen::Vector3d(0.0, radii.y(), 0.0),
        Eigen::Vector3d(0.0, -radii.y(), 0.0),
        Eigen::Vector3d(0.0, 0.0, radii.z()),
        Eigen::Vector3d(0.0, 0.0, -radii.z()),
    };
    for (const Eigen::Vector3d& axis : axes) {
        push_sample(ellipsoid.center_in_link + axis, point_radius, 0.55);
    }

    if (ellipsoid.link_name != "base_link") {
        const double xy_scale = 1.0 / std::sqrt(2.0);
        const std::array<Eigen::Vector3d, 4> diagonals = {
            Eigen::Vector3d(radii.x() * xy_scale, radii.y() * xy_scale, 0.0),
            Eigen::Vector3d(radii.x() * xy_scale, -radii.y() * xy_scale, 0.0),
            Eigen::Vector3d(-radii.x() * xy_scale, radii.y() * xy_scale, 0.0),
            Eigen::Vector3d(-radii.x() * xy_scale, -radii.y() * xy_scale, 0.0),
        };
        for (const Eigen::Vector3d& diagonal : diagonals) {
            push_sample(ellipsoid.center_in_link + diagonal, point_radius, 0.35);
        }
    }

    return samples;
}

bool pointInsideInflatedRobotEllipsoid(
    const Eigen::Vector3d& point_world,
    const std::vector<const rq::LinkCollisionEllipsoid*>& ellipsoids,
    const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& link_poses,
    const double padding_m) {
    if (!point_world.allFinite() || ellipsoids.size() != link_poses.size()) {
        return false;
    }
    for (std::size_t i = 0; i < ellipsoids.size(); ++i) {
        const rq::LinkCollisionEllipsoid* ellipsoid = ellipsoids[i];
        if (ellipsoid == nullptr || !ellipsoid->radii.allFinite()) {
            continue;
        }
        const Eigen::Vector3d radii =
            (ellipsoid->radii +
             Eigen::Vector3d::Constant(std::max(0.0, padding_m)))
                .cwiseMax(Eigen::Vector3d::Constant(1e-4));
        const Eigen::Vector3d center_world =
            link_poses[i] * ellipsoid->center_in_link;
        const Eigen::Vector3d delta_link =
            link_poses[i].linear().transpose() * (point_world - center_world);
        const double normalized_sq =
            delta_link.cwiseQuotient(radii).squaredNorm();
        if (std::isfinite(normalized_sq) && normalized_sq <= 1.0) {
            return true;
        }
    }
    return false;
}

template <typename MappingContextT>
cp::BaseFootprintCollisionChecker::Config makeBaseFootprintConfig(
    const MappingContextT& ctx) {
    cp::BaseFootprintCollisionChecker::Config cfg;
    for (const rq::LinkCollisionEllipsoid& ellipsoid : ctx.collision_ellipsoids) {
        if (ellipsoid.link_name != "base_link") {
            continue;
        }
        cfg.center_in_base = ellipsoid.center_in_link;
        cfg.size = 2.0 * ellipsoid.radii;
        cfg.footprint_sample_resolution = 0.08;
        cfg.segment_sample_resolution = kSegmentBaseStepM;
        cfg.yaw_sample_resolution = kSegmentYawStepRad;
        cfg.unknown_is_free = true;
        return cfg;
    }
    return cfg;
}

template <typename MappingContextT>
cp::BaseFootprintCollisionChecker::Config makeKinoBaseFootprintConfig(
    const MappingContextT& ctx) {
    cp::BaseFootprintCollisionChecker::Config cfg = makeBaseFootprintConfig(ctx);
    cfg.sample_mode =
        cp::BaseFootprintCollisionChecker::Config::SampleMode::BoundarySpheres;
    cfg.sample_sphere_radius = 0.06;
    cfg.footprint_sample_resolution = 0.12;
    cfg.segment_sample_resolution = 0.12;
    cfg.yaw_sample_resolution = 0.30;
    return cfg;
}

template <typename MappingContextT>
cp::PathPlanningInput::WholeBodyPoseDiagnostic diagnoseFullState(
    const Eigen::VectorXd& q_full,
    const MappingContextT& ctx,
    const std::shared_ptr<const cp::DistanceFieldInterface>& map,
    const double safe_distance) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
    diag.ik_ok = true;
    diag.external_ik_ok = true;
    diag.collision_free = false;
    diag.min_margin = std::numeric_limits<double>::infinity();
    diag.reason = "ok";
    diag.safe_distance_used = safe_distance;
    diag.q_solution = q_full;

    if (q_full.size() != kFullDof) {
        diag.min_margin = -1.0;
        diag.reason = "invalid_full_state_size";
        return diag;
    }
    if (!map) {
        diag.collision_free = true;
        diag.reason = "no_collision_map";
        return diag;
    }

    const Eigen::VectorXd q_arm = armStateFromFullState(q_full);
    arm_controller::kinematics::ForwardKinematicsOutput fk;
    if (!ctx.fk_provider || !ctx.fk_provider->compute(q_arm, fk)) {
        diag.min_margin = -1.0;
        diag.reason = "fk_fail";
        return diag;
    }

    const Eigen::Isometry3d T_world_base = baseStateToIso(baseStateFromQ(q_full));
    std::vector<const rq::LinkCollisionEllipsoid*> query_ellipsoids;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        query_link_poses;
    query_ellipsoids.reserve(ctx.collision_ellipsoids.size());
    query_link_poses.reserve(ctx.collision_ellipsoids.size());
    for (const rq::LinkCollisionEllipsoid& ellipsoid : ctx.collision_ellipsoids) {
        const auto pose_it = fk.link_poses.find(ellipsoid.link_name);
        if (pose_it == fk.link_poses.end()) {
            continue;
        }
        query_ellipsoids.push_back(&ellipsoid);
        query_link_poses.push_back(T_world_base * pose_it->second);
    }

    bool queried_any_observed = false;
    for (std::size_t i = 0; i < query_ellipsoids.size(); ++i) {
        const rq::LinkCollisionEllipsoid* ellipsoid = query_ellipsoids[i];
        if (ellipsoid == nullptr) {
            continue;
        }
        const Eigen::Isometry3d& T_world_link = query_link_poses[i];
        const Eigen::Vector3d p_world =
            T_world_link * ellipsoid->center_in_link;
        const cp::DistanceFieldQueryResult query =
            map->queryDistanceAndGradient(p_world);
        if (!query.observed || !query.distance_valid) {
            continue;
        }
        queried_any_observed = true;
        const double gradient_norm = query.gradient.norm();
        if (gradient_norm >= 1e-9) {
            const Eigen::Vector3d nearest_obstacle_point =
                p_world - query.distance * (query.gradient / gradient_norm);
            if (pointInsideInflatedRobotEllipsoid(
                    nearest_obstacle_point,
                    query_ellipsoids,
                    query_link_poses,
                    kSelfNearestRejectPaddingM)) {
                continue;
            }
        }
        const double effective_radius = ellipsoidSupportRadius(
            ellipsoid->radii, T_world_link.linear(), query.gradient);
        const double margin =
            query.distance - safe_distance - effective_radius;
        if (margin < diag.min_margin) {
            diag.min_margin = margin;
            diag.worst_link_name = ellipsoid->debug_name.empty()
                                       ? ellipsoid->link_name
                                       : ellipsoid->debug_name;
            diag.worst_point_world = p_world;
            diag.worst_distance = query.distance;
            diag.worst_effective_radius = effective_radius;
            diag.required_clearance = safe_distance + effective_radius;
            diag.worst_gradient_norm = gradient_norm;
            diag.worst_gradient_world = query.gradient;
        }
    }

    if (!queried_any_observed) {
        diag.min_margin = std::numeric_limits<double>::infinity();
        diag.collision_free = true;
        diag.reason = "no_observed_obstacles";
        return diag;
    }

    diag.collision_free = diag.min_margin >= 0.0;
    if (!diag.collision_free) {
        diag.reason = "collision_fail";
    }
    return diag;
}

template <typename MappingContextT>
rt::WholeBodyLbfgsOptimizer::CollisionCostGradient
computeFullStateCollisionCostGradient(
    const Eigen::VectorXd& q_full,
    const MappingContextT& ctx,
    const std::shared_ptr<const cp::DistanceFieldInterface>& map,
    const double safe_distance) {
    rt::WholeBodyLbfgsOptimizer::CollisionCostGradient out;
    out.gradient = Eigen::VectorXd::Zero(q_full.size());
    out.valid = false;

    if (q_full.size() != kFullDof || !map ||
        !ctx.fk_provider || !ctx.jacobian_provider) {
        return out;
    }

    const Eigen::VectorXd q_arm = armStateFromFullState(q_full);
    arm_controller::kinematics::ForwardKinematicsOutput fk;
    if (!ctx.fk_provider->compute(q_arm, fk)) {
        return out;
    }

    const BaseState base = baseStateFromQ(q_full);
    const Eigen::Isometry3d T_world_base = baseStateToIso(base);
    const Eigen::Matrix3d R_world_base = T_world_base.linear();
    const Eigen::Vector3d base_origin(base.x, base.y, 0.0);

    std::vector<const rq::LinkCollisionEllipsoid*> query_ellipsoids;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        query_link_poses;
    query_ellipsoids.reserve(ctx.collision_ellipsoids.size());
    query_link_poses.reserve(ctx.collision_ellipsoids.size());
    for (const rq::LinkCollisionEllipsoid& ellipsoid : ctx.collision_ellipsoids) {
        const auto pose_it = fk.link_poses.find(ellipsoid.link_name);
        if (pose_it == fk.link_poses.end()) {
            continue;
        }
        query_ellipsoids.push_back(&ellipsoid);
        query_link_poses.push_back(T_world_base * pose_it->second);
    }

    bool queried_any_observed = false;
    bool accumulated_any = false;
    for (std::size_t i = 0; i < query_ellipsoids.size(); ++i) {
        const rq::LinkCollisionEllipsoid* ellipsoid = query_ellipsoids[i];
        if (ellipsoid == nullptr) {
            continue;
        }
        const Eigen::Isometry3d& T_world_link = query_link_poses[i];
        const auto obstacle_samples = makeEllipsoidObstacleSamples(*ellipsoid);
        for (const EllipsoidObstacleSample& sample : obstacle_samples) {
            const Eigen::Vector3d p_world = T_world_link * sample.point_in_link;
            const cp::DistanceFieldQueryResult query =
                map->queryDistanceAndGradient(p_world);
            if (!query.observed || !query.distance_valid ||
                !std::isfinite(query.distance)) {
                continue;
            }
            queried_any_observed = true;

            const double gradient_norm = query.gradient.norm();
            if (gradient_norm < 1.0e-9 || !std::isfinite(gradient_norm)) {
                continue;
            }
            const Eigen::Vector3d normal_world = query.gradient / gradient_norm;
            const Eigen::Vector3d nearest_obstacle_point =
                p_world - query.distance * normal_world;
            if (pointInsideInflatedRobotEllipsoid(
                    nearest_obstacle_point,
                    query_ellipsoids,
                    query_link_poses,
                    kSelfNearestRejectPaddingM)) {
                continue;
            }

            const double effective_radius =
                sample.radius >= 0.0
                    ? sample.radius
                    : ellipsoidSupportRadius(
                          ellipsoid->radii, T_world_link.linear(), normal_world);
            const double margin =
                query.distance - safe_distance - effective_radius;
            const double activation =
                std::max(0.0, kWholeBodyLocalObstacleInfluenceM - margin);
            if (!(activation > 0.0)) {
                accumulated_any = true;
                continue;
            }

            const double sample_weight = std::max(0.0, sample.weight);
            out.cost += sample_weight * activation * activation * activation;
            const Eigen::Vector3d dcost_dp =
                -3.0 * sample_weight * activation * activation * normal_world;
            out.gradient[0] += dcost_dp.x();
            out.gradient[1] += dcost_dp.y();
            out.gradient[2] += dcost_dp.dot(
                Eigen::Vector3d::UnitZ().cross(p_world - base_origin));

            const Eigen::MatrixXd J_link =
                ctx.jacobian_provider->computeJacobian(
                    q_arm, ellipsoid->link_name, sample.point_in_link);
            if (J_link.rows() >= 3 && J_link.cols() == kArmDof) {
                const Eigen::MatrixXd J_world =
                    R_world_base * J_link.topRows(3);
                out.gradient.segment(kBaseDof, kArmDof) +=
                    J_world.transpose() * dcost_dp;
            }
            accumulated_any = true;
        }
    }

    out.valid = queried_any_observed || accumulated_any;
    return out;
}

template <typename MappingContextT>
bool validateFullStateSegment(
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    const MappingContextT& ctx,
    const std::shared_ptr<const cp::DistanceFieldInterface>& map,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* failed_diag) {
    if (q_from.size() != kFullDof || q_to.size() != kFullDof) {
        if (failed_diag != nullptr) {
            failed_diag->reason = "invalid_full_segment_size";
            failed_diag->min_margin = -1.0;
        }
        return false;
    }

    const double base_dist = (q_to.head<2>() - q_from.head<2>()).norm();
    const double yaw_dist = std::abs(normalizeAngle(q_to[2] - q_from[2]));
    const double joint_dist =
        (q_to.segment(kBaseDof, kArmDof) - q_from.segment(kBaseDof, kArmDof))
            .cwiseAbs()
            .maxCoeff();
    const int steps = std::max(
        1,
        std::max(
            static_cast<int>(std::ceil(base_dist / kSegmentBaseStepM)),
            std::max(
                static_cast<int>(std::ceil(yaw_dist / kSegmentYawStepRad)),
                static_cast<int>(std::ceil(joint_dist / kSegmentJointStepRad)))));

    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        Eigen::VectorXd q = (1.0 - t) * q_from + t * q_to;
        q[2] = normalizeAngle(
            q_from[2] + t * normalizeAngle(q_to[2] - q_from[2]));
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag =
            diagnoseFullState(q, ctx, map, safe_distance);
        if (!diag.collision_free) {
            if (failed_diag != nullptr) {
                diag.failed_on_segment_sample = true;
                diag.failed_segment_t = t;
                *failed_diag = diag;
            }
            return false;
        }
    }
    return true;
}

bool isPureMobileBaseCollisionDiagnostic(
    const cp::PathPlanningInput::WholeBodyPoseDiagnostic& diag) {
    if (diag.reason == "base_footprint_collision_fail") {
        return true;
    }
    return diag.reason == "collision_fail" &&
           (diag.worst_link_name == "base_link" ||
            diag.worst_link_name == "box_mobile_base");
}

void applyBaseFootprintDiagnostic(
    const cp::BaseFootprintCollisionChecker::Diagnostic& base_diag,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
    if (diag == nullptr) {
        return;
    }
    diag->collision_free = base_diag.collision_free;
    diag->min_margin = base_diag.min_distance - base_diag.required_distance;
    diag->reason = "base_footprint_collision_fail";
    diag->worst_link_name = "base_link";
    diag->worst_point_world = base_diag.worst_point_world;
    diag->worst_distance = base_diag.min_distance;
    diag->worst_effective_radius = base_diag.sample_sphere_radius;
    diag->required_clearance = base_diag.required_distance;
    diag->safe_distance_used = safe_distance;
    diag->failed_on_segment_sample = base_diag.failed_on_segment_sample;
    diag->failed_segment_t = base_diag.segment_t;
}

template <typename MappingContextT>
bool validateManipulatorStateForBaseGuide(
    const Eigen::VectorXd& q_full,
    const MappingContextT& ctx,
    const std::shared_ptr<const cp::DistanceFieldInterface>& map,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
    cp::PathPlanningInput::WholeBodyPoseDiagnostic local_diag =
        diagnoseFullState(q_full, ctx, map, safe_distance);
    if (!local_diag.collision_free &&
        isPureMobileBaseCollisionDiagnostic(local_diag)) {
        local_diag.collision_free = true;
        local_diag.reason = "base_collision_deferred";
        local_diag.min_margin = std::numeric_limits<double>::infinity();
    }
    if (diag != nullptr) {
        *diag = local_diag;
    }
    return local_diag.collision_free;
}

template <typename MappingContextT>
bool validateManipulatorSegmentForBaseGuide(
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    const MappingContextT& ctx,
    const std::shared_ptr<const cp::DistanceFieldInterface>& map,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* failed_diag) {
    if (q_from.size() != kFullDof || q_to.size() != kFullDof) {
        if (failed_diag != nullptr) {
            failed_diag->reason = "invalid_full_segment_size";
            failed_diag->min_margin = -1.0;
        }
        return false;
    }

    const double base_dist = (q_to.head<2>() - q_from.head<2>()).norm();
    const double yaw_dist = std::abs(normalizeAngle(q_to[2] - q_from[2]));
    const double joint_dist =
        (q_to.segment(kBaseDof, kArmDof) - q_from.segment(kBaseDof, kArmDof))
            .cwiseAbs()
            .maxCoeff();
    const int steps = std::max(
        1,
        std::max(
            static_cast<int>(std::ceil(base_dist / kSegmentBaseStepM)),
            std::max(
                static_cast<int>(std::ceil(yaw_dist / kSegmentYawStepRad)),
                static_cast<int>(std::ceil(joint_dist / kSegmentJointStepRad)))));

    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        Eigen::VectorXd q = (1.0 - t) * q_from + t * q_to;
        q[2] = normalizeAngle(
            q_from[2] + t * normalizeAngle(q_to[2] - q_from[2]));
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateManipulatorStateForBaseGuide(
                q, ctx, map, safe_distance, &diag)) {
            if (failed_diag != nullptr) {
                diag.failed_on_segment_sample = true;
                diag.failed_segment_t = t;
                *failed_diag = diag;
            }
            return false;
        }
    }
    return true;
}

Eigen::VectorXd interpolateFullState(
    const Eigen::VectorXd& q0,
    const Eigen::VectorXd& q1,
    const double ratio) {
    Eigen::VectorXd q = (1.0 - ratio) * q0 + ratio * q1;
    if (q.size() >= kBaseDof) {
        q[2] = normalizeAngle(q0[2] + ratio * normalizeAngle(q1[2] - q0[2]));
    }
    return q;
}

bool sampleWholeBodyTrajectory(
    const cp::TimedJointTrajectory& trajectory,
    const double t_query,
    Eigen::VectorXd* q_ref) {
    if (q_ref == nullptr || trajectory.empty() ||
        trajectory.joint_targets.size() != trajectory.cumulative_times.size()) {
        return false;
    }
    if (t_query <= 0.0) {
        *q_ref = trajectory.joint_targets.front();
        return true;
    }
    if (t_query >= trajectory.total_duration) {
        *q_ref = trajectory.joint_targets.back();
        return true;
    }

    auto upper = std::upper_bound(
        trajectory.cumulative_times.begin(),
        trajectory.cumulative_times.end(),
        t_query);
    if (upper == trajectory.cumulative_times.begin() ||
        upper == trajectory.cumulative_times.end()) {
        *q_ref = trajectory.joint_targets.back();
        return true;
    }
    const std::size_t i1 = static_cast<std::size_t>(
        std::distance(trajectory.cumulative_times.begin(), upper));
    const std::size_t i0 = i1 - 1u;
    const double t0 = trajectory.cumulative_times[i0];
    const double t1 = trajectory.cumulative_times[i1];
    const double ratio =
        (t1 > t0 + 1e-9) ? std::clamp((t_query - t0) / (t1 - t0), 0.0, 1.0)
                         : 0.0;
    *q_ref = interpolateFullState(
        trajectory.joint_targets[i0], trajectory.joint_targets[i1], ratio);
    return true;
}

bool splitWholeBodyReferenceState(
    const Eigen::VectorXd& q_full,
    const double time_from_start,
    WholeBodyReferenceSample* sample) {
    if (sample == nullptr || q_full.size() != kFullDof || !q_full.allFinite()) {
        return false;
    }

    sample->full_state = q_full;
    sample->base = baseStateFromQ(q_full);
    sample->arm = armStateFromFullState(q_full);
    sample->time_from_start = time_from_start;
    return sample->arm.size() == kArmDof && sample->arm.allFinite();
}

bool sampleWholeBodyReference(
    const cp::TimedJointTrajectory& trajectory,
    const double t_query,
    WholeBodyReferenceSample* sample) {
    Eigen::VectorXd q_ref;
    if (!sampleWholeBodyTrajectory(trajectory, t_query, &q_ref)) {
        return false;
    }
    return splitWholeBodyReferenceState(q_ref, t_query, sample);
}

double wholeBodyReferenceDistance(
    const Eigen::VectorXd& q_current,
    const Eigen::VectorXd& q_ref) {
    if (q_current.size() != kFullDof || q_ref.size() != kFullDof) {
        return std::numeric_limits<double>::infinity();
    }
    const double base_xy =
        (q_current.head<2>() - q_ref.head<2>()).squaredNorm();
    const double yaw =
        normalizeAngle(q_current[2] - q_ref[2]) *
        normalizeAngle(q_current[2] - q_ref[2]);
    return base_xy + 0.10 * yaw;
}

double projectWholeBodyStateToTrajectoryTime(
    const cp::TimedJointTrajectory& trajectory,
    const Eigen::VectorXd& q_current,
    const double time_hint_sec) {
    if (trajectory.empty() ||
        trajectory.joint_targets.size() != trajectory.cumulative_times.size() ||
        q_current.size() != kFullDof) {
        return std::clamp(time_hint_sec, 0.0, trajectory.total_duration);
    }

    const double clamped_hint =
        std::clamp(time_hint_sec, 0.0, trajectory.total_duration);
    const double search_radius_sec = 2.0;
    double best_time = clamped_hint;
    double best_cost = std::numeric_limits<double>::infinity();

    auto trySample = [&](const double t) {
        Eigen::VectorXd q_ref;
        if (!sampleWholeBodyTrajectory(trajectory, t, &q_ref)) {
            return;
        }
        const double cost = wholeBodyReferenceDistance(q_current, q_ref);
        if (cost < best_cost) {
            best_cost = cost;
            best_time = t;
        }
    };

    for (std::size_t i = 0; i < trajectory.cumulative_times.size(); ++i) {
        const double t = trajectory.cumulative_times[i];
        if (std::abs(t - clamped_hint) <= search_radius_sec) {
            trySample(t);
        }
    }
    const double sample_dt = 0.05;
    const double t_min =
        std::max(0.0, clamped_hint - search_radius_sec);
    const double t_max =
        std::min(trajectory.total_duration, clamped_hint + search_radius_sec);
    for (double t = t_min; t <= t_max + 1e-9; t += sample_dt) {
        trySample(t);
    }
    return best_time;
}

WholeBodyTrackingCommand buildWholeBodyTrackingCommand(
    const WholeBodyReferenceSample& ref,
    const WholeBodyReferenceSample& ref_next,
    const BaseState& base_now,
    const Eigen::VectorXd& q_arm_now,
    const double dt,
    const double kp_base_xy,
    const double kp_base_yaw,
    const double kp_arm) {
    WholeBodyTrackingCommand command;
    command.arm_qdot = Eigen::VectorXd::Zero(kArmDof);

    const double safe_dt = std::max(1e-3, dt);
    rq::TaskVelocityGenerator task_velocity_generator;
    rq::MobileBaseVelocityInput base_input;
    base_input.current_pose =
        Eigen::Vector3d(base_now.x, base_now.y, base_now.yaw);
    base_input.target_pose =
        Eigen::Vector3d(ref.base.x, ref.base.y, ref.base.yaw);
    base_input.next_target_pose =
        Eigen::Vector3d(ref_next.base.x, ref_next.base.y, ref_next.base.yaw);
    base_input.has_next_target_pose = true;
    base_input.dt_sec = safe_dt;
    rq::MobileBaseVelocityConfig base_cfg;
    base_cfg.kp_xy = kp_base_xy;
    base_cfg.kp_yaw = kp_base_yaw;
    base_cfg.max_vx = std::numeric_limits<double>::infinity();
    base_cfg.max_vy = std::numeric_limits<double>::infinity();
    base_cfg.max_wz = std::numeric_limits<double>::infinity();
    command.base_body_twist =
        task_velocity_generator.computeMobileBaseVelocity(base_input, base_cfg)
            .body_twist;

    if (q_arm_now.size() == kArmDof) {
        command.arm_qdot =
            (ref_next.arm - ref.arm) / safe_dt + kp_arm * (ref.arm - q_arm_now);
    }
    return command;
}

Eigen::VectorXd packWholeBodyVelocity(
    const WholeBodyTrackingCommand& command) {
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(kFullDof);
    qdot.head(kBaseDof) = command.base_body_twist;
    if (command.arm_qdot.size() == kArmDof) {
        qdot.segment(kBaseDof, kArmDof) = command.arm_qdot;
    }
    return qdot;
}

WholeBodyTrackingCommand unpackWholeBodyVelocity(
    const Eigen::VectorXd& qdot) {
    WholeBodyTrackingCommand command;
    command.arm_qdot = Eigen::VectorXd::Zero(kArmDof);
    if (qdot.size() == kFullDof) {
        command.base_body_twist = qdot.head(kBaseDof);
        command.arm_qdot = qdot.segment(kBaseDof, kArmDof);
    }
    return command;
}

Eigen::Vector3d limitBaseTwistRate(
    const Eigen::Vector3d& command,
    const Eigen::Vector3d& previous,
    const double dt,
    const bool previous_valid) {
    if (!previous_valid || !command.allFinite() || !previous.allFinite()) {
        return command;
    }
    const double safe_dt = std::max(1e-3, dt);
    Eigen::Vector3d limited = command;
    limited.x() = previous.x() + std::clamp(
        command.x() - previous.x(),
        -kWholeBodyBaseMaxAx * safe_dt,
        kWholeBodyBaseMaxAx * safe_dt);
    limited.y() = previous.y() + std::clamp(
        command.y() - previous.y(),
        -kWholeBodyBaseMaxAy * safe_dt,
        kWholeBodyBaseMaxAy * safe_dt);
    limited.z() = previous.z() + std::clamp(
        command.z() - previous.z(),
        -kWholeBodyBaseMaxAwz * safe_dt,
        kWholeBodyBaseMaxAwz * safe_dt);
    return limited;
}

Eigen::Vector3d suppressNegativeGlobalPathProgress(
    const Eigen::Vector3d& body_twist,
    const BaseState& base_now,
    const WholeBodyReferenceSample& ref,
    const WholeBodyReferenceSample& ref_next) {
    if (!body_twist.allFinite()) {
        return body_twist;
    }
    Eigen::Vector2d path_tangent(
        ref_next.base.x - ref.base.x,
        ref_next.base.y - ref.base.y);
    if (path_tangent.norm() < 1.0e-4) {
        path_tangent = Eigen::Vector2d(
            ref_next.base.x - base_now.x,
            ref_next.base.y - base_now.y);
    }
    const double tangent_norm = path_tangent.norm();
    if (tangent_norm < 1.0e-4) {
        return body_twist;
    }
    const Eigen::Vector2d tangent = path_tangent / tangent_norm;
    const Eigen::Rotation2Dd R_base(base_now.yaw);
    Eigen::Vector2d v_world = R_base * body_twist.head<2>();
    const double progress_speed = v_world.dot(tangent);
    if (progress_speed >= -0.01) {
        return body_twist;
    }
    v_world -= progress_speed * tangent;
    Eigen::Vector3d out = body_twist;
    out.head<2>() = R_base.inverse() * v_world;
    return out;
}

Eigen::VectorXd makeCurrentWholeBodyState(
    const BaseState& base_now,
    const Eigen::VectorXd& q_arm_now) {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(kFullDof);
    q[0] = base_now.x;
    q[1] = base_now.y;
    q[2] = base_now.yaw;
    if (q_arm_now.size() == kArmDof) {
        q.segment(kBaseDof, kArmDof) = q_arm_now;
    }
    return q;
}

void appendBaseFootprintObstacleConstraints(
    const BaseState& base_now,
    const WholeBodyQpCollisionContext& collision_ctx,
    const std::shared_ptr<const cp::DistanceFieldInterface>& collision_map,
    rq::ObstacleConstraintInputList* out_constraints) {
    if (out_constraints == nullptr || !collision_map) {
        return;
    }

    const auto local_samples =
        makeBaseCollisionLocalSamples(collision_ctx.collision_ellipsoids);
    if (local_samples.empty()) {
        return;
    }

    const Eigen::Matrix2d R_base_xy =
        Eigen::Rotation2Dd(base_now.yaw).toRotationMatrix();
    const Eigen::Matrix3d R_base =
        Eigen::AngleAxisd(base_now.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d p_base(base_now.x, base_now.y, 0.0);

    cp::Vector3dList world_samples;
    world_samples.reserve(local_samples.size());
    for (const Eigen::Vector3d& local : local_samples) {
        world_samples.push_back(p_base + R_base * local);
    }

    const cp::DistanceFieldQueryResultList queries =
        collision_map->queryDistanceAndGradientBatch(world_samples);
    if (queries.size() != world_samples.size()) {
        return;
    }

    for (std::size_t i = 0; i < queries.size(); ++i) {
        const cp::DistanceFieldQueryResult& query = queries[i];
        if (!query.observed || !query.distance_valid || !query.gradient_valid ||
            !std::isfinite(query.distance) || query.gradient.norm() < 1.0e-9) {
            continue;
        }
        const Eigen::Vector3d n = query.gradient.normalized();
        const Eigen::Vector2d n_xy(n.x(), n.y());
        const Eigen::Vector2d base_body_xy = R_base_xy.transpose() * n_xy;
        const Eigen::Vector2d p_rel_xy(
            world_samples[i].x() - base_now.x,
            world_samples[i].y() - base_now.y);

        rq::ObstacleConstraintInput c;
        c.normal_jacobian = Eigen::RowVectorXd::Zero(kFullDof);
        c.normal_jacobian[0] = base_body_xy.x();
        c.normal_jacobian[1] = base_body_xy.y();
        c.normal_jacobian[2] =
            n_xy.dot(Eigen::Vector2d(-p_rel_xy.y(), p_rel_xy.x()));
        c.linear_jacobian = Eigen::MatrixXd::Zero(3, kFullDof);
        c.linear_jacobian.block<2, 2>(0, 0) = R_base_xy;
        c.linear_jacobian(0, 2) = -p_rel_xy.y();
        c.linear_jacobian(1, 2) = p_rel_xy.x();
        c.point_world = world_samples[i];
        c.normal_world = n;
        c.distance = query.distance;
        c.debug_name = "whole_body_base_footprint_" + std::to_string(i);
        if (c.normal_jacobian.allFinite() && c.linear_jacobian.allFinite()) {
            out_constraints->push_back(std::move(c));
        }
    }
}

bool solveWholeBodyTrackingQp(
    const BaseState& base_now,
    const Eigen::VectorXd& q_arm_now,
    const WholeBodyTrackingCommand& nominal_command,
    const WholeBodyQpVelocityBounds& velocity_bounds,
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max,
    const WholeBodyQpCollisionContext& collision_ctx,
    const std::shared_ptr<const cp::DistanceFieldInterface>& collision_map,
    const double safe_distance,
    const Eigen::VectorXd& previous_qdot,
    const bool previous_qdot_valid,
    const rq::ReactiveQpBuildConfig& base_qp_cfg,
    rq::ReactiveQpSolver& solver,
    Eigen::VectorXd* qdot_solution,
    std::string* error) {
    if (qdot_solution == nullptr || q_arm_now.size() != kArmDof ||
        velocity_bounds.qd_min.size() != kFullDof ||
        velocity_bounds.qd_max.size() != kFullDof ||
        q_min.size() != kFullDof ||
        q_max.size() != kFullDof) {
        if (error != nullptr) {
            *error = "invalid whole-body QP input dimensions";
        }
        return false;
    }

    const Eigen::VectorXd q_current =
        makeCurrentWholeBodyState(base_now, q_arm_now);
    const Eigen::VectorXd desired_qdot =
        packWholeBodyVelocity(nominal_command);
    if (desired_qdot.size() != kFullDof || !desired_qdot.allFinite()) {
        if (error != nullptr) {
            *error = "invalid whole-body desired velocity";
        }
        return false;
    }

    rq::ReactiveQpBuildInput qp_input;
    qp_input.q_current = q_current;
    qp_input.jacobian_task = Eigen::MatrixXd::Identity(kFullDof, kFullDof);
    qp_input.desired_twist = desired_qdot;
    qp_input.manipulability_gradient = Eigen::VectorXd::Zero(kFullDof);
    qp_input.posture_velocity_reference = desired_qdot;
    if (previous_qdot_valid && previous_qdot.size() == kFullDof &&
        previous_qdot.allFinite()) {
        qp_input.previous_qdot_reference = previous_qdot;
    }
    qp_input.posture_joint_weights = Eigen::VectorXd::Ones(kFullDof);
    qp_input.qd_min = velocity_bounds.qd_min;
    qp_input.qd_max = velocity_bounds.qd_max;
    qp_input.joint_limits.q_min = Eigen::VectorXd::Constant(kFullDof, -1.0e6);
    qp_input.joint_limits.q_max = Eigen::VectorXd::Constant(kFullDof, 1.0e6);
    qp_input.joint_limits.q_min.segment(kBaseDof, kArmDof) =
        q_min.segment(kBaseDof, kArmDof);
    qp_input.joint_limits.q_max.segment(kBaseDof, kArmDof) =
        q_max.segment(kBaseDof, kArmDof);

    appendBaseFootprintObstacleConstraints(
        base_now, collision_ctx, collision_map, &qp_input.obstacle_constraints);

    if (collision_ctx.fk_provider && collision_ctx.jacobian_provider &&
        collision_ctx.collision_ellipsoids && collision_map &&
        !collision_ctx.collision_ellipsoids->empty()) {
        arm_controller::kinematics::ForwardKinematicsOutput fk;
        if (collision_ctx.fk_provider->compute(q_arm_now, fk)) {
            arm_controller::kinematics::ForwardKinematicsOutput::LinkPoseMap
                link_poses_world;
            const Eigen::Isometry3d T_world_base = baseStateToIso(base_now);
            for (const auto& item : fk.link_poses) {
                link_poses_world[item.first] = T_world_base * item.second;
            }

            rq::ObstacleConstraintInputList arm_constraints;
            std::string obstacle_error;
            rq::BodyObstacleConstraintBuilder::appendLinkEllipsoidConstraints(
                q_arm_now,
                link_poses_world,
                *collision_ctx.collision_ellipsoids,
                *collision_ctx.jacobian_provider,
                collision_map,
                arm_constraints,
                &obstacle_error);

            for (const rq::ObstacleConstraintInput& arm_constraint : arm_constraints) {
                if (arm_constraint.normal_jacobian.size() != kArmDof ||
                    !arm_constraint.normal_jacobian.allFinite() ||
                    !std::isfinite(arm_constraint.distance)) {
                    continue;
                }
                rq::ObstacleConstraintInput full_constraint = arm_constraint;
                full_constraint.normal_jacobian = Eigen::RowVectorXd::Zero(kFullDof);
                const Eigen::Vector3d n =
                    arm_constraint.normal_world.norm() > 1.0e-9
                        ? arm_constraint.normal_world.normalized()
                        : Eigen::Vector3d::Zero();
                const Eigen::Matrix2d R_base_xy =
                    Eigen::Rotation2Dd(base_now.yaw).toRotationMatrix();
                const Eigen::Vector2d n_xy(n.x(), n.y());
                const Eigen::Vector2d base_body_xy =
                    R_base_xy.transpose() * n_xy;
                const Eigen::Vector2d p_rel_xy(
                    arm_constraint.point_world.x() - base_now.x,
                    arm_constraint.point_world.y() - base_now.y);
                full_constraint.normal_jacobian[0] = base_body_xy.x();
                full_constraint.normal_jacobian[1] = base_body_xy.y();
                full_constraint.normal_jacobian[2] =
                    n_xy.dot(Eigen::Vector2d(-p_rel_xy.y(), p_rel_xy.x()));
                full_constraint.normal_jacobian.segment(kBaseDof, kArmDof) =
                    arm_constraint.normal_jacobian;
                full_constraint.debug_name =
                    "whole_body_" + arm_constraint.debug_name;
                qp_input.obstacle_constraints.push_back(std::move(full_constraint));
            }
        }
    }

    if (qp_input.obstacle_constraints.size() > 6u) {
        std::sort(
            qp_input.obstacle_constraints.begin(),
            qp_input.obstacle_constraints.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs.distance < rhs.distance;
            });
        qp_input.obstacle_constraints.resize(6u);
    }

    rq::ReactiveQpBuildConfig qp_cfg = base_qp_cfg;
    qp_cfg.enable_joint_limit_damper = base_qp_cfg.enable_joint_limit_damper;
    qp_cfg.enable_obstacle_damper =
        base_qp_cfg.enable_obstacle_damper &&
        !qp_input.obstacle_constraints.empty();
    qp_cfg.obstacle_damper.safety_distance = safe_distance;

    rq::ReactiveQpProblem problem;
    if (!rq::ReactiveQpBuilder::build(qp_input, qp_cfg, problem, error)) {
        return false;
    }

    Eigen::VectorXd solution;
    if (!solver.solve(problem, solution, error)) {
        return false;
    }
    if (solution.size() < kFullDof || !solution.head(kFullDof).allFinite()) {
        if (error != nullptr) {
            *error = "whole-body QP returned invalid solution";
        }
        return false;
    }

    *qdot_solution = solution.head(kFullDof);
    return true;
}

std::vector<double> clampArmVelocity(const Eigen::VectorXd& qdot_arm) {
    std::vector<double> out(static_cast<std::size_t>(qdot_arm.size()), 0.0);
    for (int i = 0; i < qdot_arm.size(); ++i) {
        out[static_cast<std::size_t>(i)] =
            std::clamp(qdot_arm[i],
                       -kWholeBodyTrackingMaxArmQdot,
                       kWholeBodyTrackingMaxArmQdot);
    }
    return out;
}

}  // namespace

bool ReactiveTaskController::executeDualArmTask(
    const std::string& mapping,
    const DualArmTarget& target) {
    if (mapping != "dual_arm") {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] dual-arm reactive task requires mapping='dual_arm'",
            mapping.c_str());
        last_execution_success_[mapping] = false;
        return false;
    }
    if (!runtime_cfg_.enable_mobile_base_in_planning) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[dual_arm] mobile-base planning is disabled; set whole_body.enable_mobile_base_in_planning=true");
        last_execution_success_[mapping] = false;
        return false;
    }

    std::string init_error;
    if (!initializeMappingContext(mapping, &init_error)) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[dual_arm] whole-body context init failed: %s",
            init_error.c_str());
        last_execution_success_[mapping] = false;
        return false;
    }

    MappingContext* ctx = nullptr;
    {
        std::lock_guard<std::mutex> lock(mapping_contexts_mutex_);
        auto it = mapping_contexts_.find(mapping);
        if (it != mapping_contexts_.end() && it->second.initialized) {
            ctx = &it->second;
        }
    }
    if (ctx == nullptr || ctx->joint_names.size() != kArmDof) {
        RCLCPP_ERROR(node_->get_logger(), "[dual_arm] invalid mapping context");
        last_execution_success_[mapping] = false;
        return false;
    }

    const std::optional<BaseState> base_now_opt = readBaseStateFromTf(
        *tf_buffer_,
        runtime_cfg_.mobile_base_odom_frame,
        runtime_cfg_.mobile_base_frame,
        node_->get_logger());
    if (!base_now_opt.has_value()) {
        last_execution_success_[mapping] = false;
        return false;
    }
    const BaseState base_now = *base_now_opt;

    const std::vector<double> q_current_vec =
        hardware_manager_->get_current_joint_positions_lockfree(mapping);
    if (q_current_vec.size() != kArmDof) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[dual_arm] current joint size mismatch: expected %d got %zu",
            kArmDof,
            q_current_vec.size());
        last_execution_success_[mapping] = false;
        return false;
    }

    const Eigen::VectorXd q_arm_start = Eigen::Map<const Eigen::VectorXd>(
        q_current_vec.data(), static_cast<Eigen::Index>(q_current_vec.size()));
    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(kFullDof);
    q_start[0] = base_now.x;
    q_start[1] = base_now.y;
    q_start[2] = base_now.yaw;
    q_start.segment(kBaseDof, kArmDof) = q_arm_start;

    arm_controller::kinematics::ForwardKinematicsOutput start_fk;
    if (!ctx->fk_provider || !ctx->fk_provider->compute(q_arm_start, start_fk)) {
        RCLCPP_ERROR(node_->get_logger(), "[dual_arm] FK failed at start");
        last_execution_success_[mapping] = false;
        return false;
    }

    auto findMount = [&](const std::string& link_name,
                         Eigen::Isometry3d* out) -> bool {
        const auto it = start_fk.link_poses.find(link_name);
        if (it == start_fk.link_poses.end() || out == nullptr) {
            return false;
        }
        *out = it->second;
        return true;
    };

    Eigen::Isometry3d T_base_left_mount = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_base_right_mount = Eigen::Isometry3d::Identity();
    if (!findMount("left_base_link", &T_base_left_mount) ||
        !findMount("right_base_link", &T_base_right_mount)) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[dual_arm] failed to resolve left/right arm mount poses from FK");
        last_execution_success_[mapping] = false;
        return false;
    }

    std::string urdf_xml;
    {
        std::ifstream ifs(ctx->urdf_path);
        urdf_xml.assign(
            std::istreambuf_iterator<char>(ifs),
            std::istreambuf_iterator<char>());
    }
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[dual_arm] failed to read URDF: %s",
            ctx->urdf_path.c_str());
        last_execution_success_[mapping] = false;
        return false;
    }

    ArmIkContext left_ik;
    ArmIkContext right_ik;
    std::string ik_error;
    if (!initializeArmIkContext(node_, "left_arm", urdf_xml, &left_ik, &ik_error) ||
        !initializeArmIkContext(node_, "right_arm", urdf_xml, &right_ik, &ik_error)) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[dual_arm] arm IK context init failed: %s",
            ik_error.c_str());
        last_execution_success_[mapping] = false;
        return false;
    }

    const Eigen::Isometry3d T_world_left_goal = poseMsgToIso(target.left);
    const Eigen::Isometry3d T_world_right_goal = poseMsgToIso(target.right);
    Eigen::Isometry3d T_world_left_relaxed_goal = T_world_left_goal;
    Eigen::Isometry3d T_world_right_relaxed_goal = T_world_right_goal;
    if (const auto it = start_fk.link_poses.find("left_Link6");
        it != start_fk.link_poses.end()) {
        T_world_left_relaxed_goal.linear() =
            (baseStateToIso(base_now) * it->second).linear();
    }
    if (const auto it = start_fk.link_poses.find("right_Link6");
        it != start_fk.link_poses.end()) {
        T_world_right_relaxed_goal.linear() =
            (baseStateToIso(base_now) * it->second).linear();
    }

    ensureCameraDriverDistanceFieldInitialized();
    std::shared_ptr<const cp::DistanceFieldInterface> collision_map;
    std::shared_ptr<const cp::CameraDriverPointcloudMapAdapter> pointcloud_map;
    std::shared_ptr<const cp::CameraDriverEsdfMapClient> esdf_map;
    {
        std::lock_guard<std::mutex> lock(live_distance_field_mutex_);
        pointcloud_map = camera_driver_pointcloud_map_;
        esdf_map = camera_driver_esdf_map_;
        if (runtime_cfg_.distance_field_source == "camera_driver_esdf") {
            collision_map =
                std::static_pointer_cast<const cp::DistanceFieldInterface>(
                    camera_driver_esdf_map_);
        } else if (runtime_cfg_.collision_map_source == "camera_driver_esdf") {
            collision_map =
                std::static_pointer_cast<const cp::DistanceFieldInterface>(
                    camera_driver_esdf_map_);
        } else if (runtime_cfg_.collision_map_source == "camera_driver_pointcloud") {
            collision_map =
                std::static_pointer_cast<const cp::DistanceFieldInterface>(
                    camera_driver_pointcloud_map_);
        } else if (runtime_cfg_.distance_field_source == "camera_driver_pointcloud") {
            collision_map =
                std::static_pointer_cast<const cp::DistanceFieldInterface>(
                    camera_driver_pointcloud_map_);
        }
    }
    if ((runtime_cfg_.distance_field_source == "camera_driver_esdf" ||
         runtime_cfg_.collision_map_source == "camera_driver_esdf") &&
        !waitForCameraDriverEsdfSnapshot(
            node_, esdf_map, kEsdfReadyWaitTimeoutSec)) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[dual_arm] whole-body planning aborted: camera_driver ESDF SHM snapshot is not ready");
        last_execution_success_[mapping] = false;
        return false;
    }
    if (collision_map) {
        auto composite_map = std::make_shared<cp::CompositeDistanceField>();
        composite_map->addField(collision_map);
        auto static_pillar_map = cp::StaticPillarDistanceField::makeDefaultPillarWorld();
        composite_map->addField(static_pillar_map);
        collision_map =
            std::static_pointer_cast<const cp::DistanceFieldInterface>(
                composite_map);
        RCLCPP_INFO(
            node_->get_logger(),
            "[dual_arm] static pillar distance field merged into whole-body planning map.");
    }

    const Eigen::Vector3d left_goal = T_world_left_goal.translation();
    const Eigen::Vector3d right_goal = T_world_right_goal.translation();
    const Eigen::Vector2d target_mid =
        0.5 * (left_goal.head<2>() + right_goal.head<2>());
    std::vector<BaseState> base_candidates =
        makeBaseGoalCandidates(base_now, left_goal, right_goal);
    sortAndLimitBaseGoalCandidates(base_candidates, target_mid);

    const std::vector<double> left_seed(
        q_current_vec.begin(), q_current_vec.begin() + 6);
    const std::vector<double> right_seed(
        q_current_vec.begin() + 6, q_current_vec.end());

    cp::PathPlanningInput input;
    input.q_start_seed = q_start;
    input.q_min = Eigen::VectorXd::Zero(kFullDof);
    input.q_max = Eigen::VectorXd::Zero(kFullDof);
    input.q_min[0] = std::min(base_now.x, target_mid.x()) - kBaseBoundsMarginM;
    input.q_max[0] = std::max(base_now.x, target_mid.x()) + kBaseBoundsMarginM;
    input.q_min[1] = std::min(base_now.y, target_mid.y()) - kBaseBoundsMarginM;
    input.q_max[1] = std::max(base_now.y, target_mid.y()) + kBaseBoundsMarginM;
    input.q_min[2] = base_now.yaw - M_PI;
    input.q_max[2] = base_now.yaw + M_PI;
    input.q_min.segment(kBaseDof, kArmDof) = ctx->joint_limits.q_min;
    input.q_max.segment(kBaseDof, kArmDof) = ctx->joint_limits.q_max;
    input.safe_distance =
        std::max(runtime_cfg_.request_safe_distance,
                 runtime_cfg_.request_planning_safe_distance);
    input.feasibility_safe_distance = 0.0;
    input.hard_clearance = runtime_cfg_.request_hard_clearance;
    input.goal_tolerance = runtime_cfg_.request_goal_tolerance;
    input.p_start = baseStateToIso(base_now).translation();
    input.p_goal = Eigen::Vector3d(target_mid.x(), target_mid.y(), base_now.yaw);

    auto base_footprint_checker =
        std::make_shared<cp::BaseFootprintCollisionChecker>(
            collision_map, makeBaseFootprintConfig(*ctx));
    const cp::BaseFootprintCollisionChecker::Config kino_footprint_cfg =
        makeKinoBaseFootprintConfig(*ctx);
    auto kino_base_footprint_checker =
        std::make_shared<cp::BaseFootprintCollisionChecker>(
            collision_map, kino_footprint_cfg);
    input.joint_state_validator =
        [ctx, collision_map, base_footprint_checker](
            const Eigen::VectorXd& q,
            const double safe_distance,
            cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            cp::PathPlanningInput::WholeBodyPoseDiagnostic local_diag =
                diagnoseFullState(q, *ctx, collision_map, safe_distance);
            if (local_diag.collision_free && q.size() >= kBaseDof) {
                const cp::BaseFootprintCollisionChecker::Diagnostic base_diag =
                    base_footprint_checker->checkState(
                        cp::BaseFootprintCollisionChecker::BaseState{
                            q[0], q[1], q[2]},
                        safe_distance);
                if (!base_diag.collision_free) {
                    applyBaseFootprintDiagnostic(
                        base_diag, safe_distance, &local_diag);
                }
            }
            if (diag != nullptr) {
                *diag = local_diag;
            }
            return local_diag.collision_free;
        };
    input.manipulator_state_validator =
        [ctx, collision_map](
            const Eigen::VectorXd& q,
            const double safe_distance,
            cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            return validateManipulatorStateForBaseGuide(
                q, *ctx, collision_map, safe_distance, diag);
        };
    input.mobile_base_state_validator =
        [base_footprint_checker](
            const double x,
            const double y,
            const double yaw,
            const double safe_distance) {
            return base_footprint_checker->isStateCollisionFree(
                cp::BaseFootprintCollisionChecker::BaseState{x, y, yaw},
                safe_distance);
        };
    input.mobile_base_segment_validator =
        [base_footprint_checker](
            const double from_x,
            const double from_y,
            const double from_yaw,
            const double to_x,
            const double to_y,
            const double to_yaw,
            const double safe_distance) {
            return base_footprint_checker->isSegmentCollisionFree(
                cp::BaseFootprintCollisionChecker::BaseState{
                    from_x, from_y, from_yaw},
                cp::BaseFootprintCollisionChecker::BaseState{
                    to_x, to_y, to_yaw},
                safe_distance);
        };
    input.kino_mobile_base_state_validator =
        [kino_base_footprint_checker](
            const double x,
            const double y,
            const double yaw,
            const double safe_distance) {
            return kino_base_footprint_checker->isStateCollisionFree(
                cp::BaseFootprintCollisionChecker::BaseState{x, y, yaw},
                safe_distance);
        };
    input.kino_mobile_base_segment_validator =
        [kino_base_footprint_checker](
            const double from_x,
            const double from_y,
            const double from_yaw,
            const double to_x,
            const double to_y,
            const double to_yaw,
            const double safe_distance) {
            return kino_base_footprint_checker->isSegmentCollisionFree(
                cp::BaseFootprintCollisionChecker::BaseState{
                    from_x, from_y, from_yaw},
                cp::BaseFootprintCollisionChecker::BaseState{
                    to_x, to_y, to_yaw},
                safe_distance);
        };

    RCLCPP_INFO(
        node_->get_logger(),
        "[dual_arm] building whole-body TrajOpt-style goal candidates: base_candidates=%zu left_goal=(%.3f, %.3f, %.3f) right_goal=(%.3f, %.3f, %.3f)",
        base_candidates.size(),
        left_goal.x(),
        left_goal.y(),
        left_goal.z(),
        right_goal.x(),
        right_goal.y(),
        right_goal.z());

    rt::WholeBodyGoalGenerator::Config goal_gen_cfg;
    goal_gen_cfg.position_tolerance_m = runtime_cfg_.goal_position_tolerance;
    goal_gen_cfg.orientation_tolerance_rad =
        runtime_cfg_.goal_orientation_tolerance_rad;
    goal_gen_cfg.safe_distance = input.safe_distance;
    goal_gen_cfg.hard_clearance = input.hard_clearance;
    goal_gen_cfg.max_goal_candidates = kMaxWholeBodyGoalCandidates;
    goal_gen_cfg.optimizer_iterations = 0;
    goal_gen_cfg.require_strict_orientation = true;
    rt::WholeBodyGoalGenerator goal_generator(goal_gen_cfg);

    rt::WholeBodyGoalGenerator::Input goal_gen_input;
    goal_gen_input.q_start = q_start;
    goal_gen_input.q_min = input.q_min;
    goal_gen_input.q_max = input.q_max;
    goal_gen_input.q_nominal = q_start;
    goal_gen_input.left_target = T_world_left_goal;
    goal_gen_input.right_target = T_world_right_goal;
    goal_gen_input.joint_state_validator = input.joint_state_validator;
    goal_gen_input.seeds.reserve(base_candidates.size());
    for (const BaseState& candidate : base_candidates) {
        rt::WholeBodyGoalGenerator::Seed seed;
        seed.base = rt::WholeBodyGoalGenerator::BaseState{
            candidate.x, candidate.y, candidate.yaw};
        seed.arm_seed = q_arm_start;
        goal_gen_input.seeds.push_back(std::move(seed));
    }
    goal_gen_input.seed_projector =
        [left_ik,
         right_ik,
         T_base_left_mount,
         T_base_right_mount,
         T_world_left_goal,
         T_world_right_goal,
         left_seed,
         right_seed](
            const rt::WholeBodyGoalGenerator::BaseState& base,
            const Eigen::VectorXd& arm_seed,
            Eigen::VectorXd* arm_solution) -> bool {
            if (arm_solution == nullptr) {
                return false;
            }
            const BaseState candidate{base.x, base.y, base.yaw};
            const Eigen::Isometry3d T_world_base = baseStateToIso(candidate);
            std::vector<double> left_project_seed = left_seed;
            std::vector<double> right_project_seed = right_seed;
            if (arm_seed.size() == kArmDof) {
                left_project_seed.assign(
                    arm_seed.data(), arm_seed.data() + 6);
                right_project_seed.assign(
                    arm_seed.data() + 6, arm_seed.data() + kArmDof);
            }

            Eigen::VectorXd q_left;
            Eigen::VectorXd q_right;
            if (!solveArmIkForBaseCandidate(
                    left_ik,
                    T_world_base,
                    T_base_left_mount,
                    T_world_left_goal,
                    left_project_seed,
                    &q_left) ||
                !solveArmIkForBaseCandidate(
                    right_ik,
                    T_world_base,
                    T_base_right_mount,
                    T_world_right_goal,
                    right_project_seed,
                    &q_right)) {
                return false;
            }
            *arm_solution = Eigen::VectorXd::Zero(kArmDof);
            arm_solution->segment(0, 6) = q_left;
            arm_solution->segment(6, 6) = q_right;
            return true;
        };
    goal_gen_input.full_state_fk =
        [ctx](const Eigen::VectorXd& q_full,
              Eigen::Isometry3d* left_pose,
              Eigen::Isometry3d* right_pose) -> bool {
            if (q_full.size() != kFullDof || left_pose == nullptr ||
                right_pose == nullptr || !ctx || !ctx->fk_provider) {
                return false;
            }
            const Eigen::VectorXd q_arm =
                q_full.segment(kBaseDof, kArmDof);
            arm_controller::kinematics::ForwardKinematicsOutput fk;
            if (!ctx->fk_provider->compute(q_arm, fk)) {
                return false;
            }
            const auto left_it = fk.link_poses.find("left_Link6");
            const auto right_it = fk.link_poses.find("right_Link6");
            if (left_it == fk.link_poses.end() ||
                right_it == fk.link_poses.end()) {
                return false;
            }
            const BaseState base = baseStateFromQ(q_full);
            const Eigen::Isometry3d T_world_base = baseStateToIso(base);
            *left_pose = T_world_base * left_it->second;
            *right_pose = T_world_base * right_it->second;
            return true;
        };

    const rt::WholeBodyGoalGenerator::Output goal_gen_output =
        goal_generator.generate(goal_gen_input);
    input.q_goal_candidates = goal_gen_output.q_goal_candidates;

    std::size_t valid_diag_count = 0;
    std::size_t collision_free_diag_count = 0;
    for (std::size_t i = 0; i < goal_gen_output.diagnostics.size(); ++i) {
        const auto& diag = goal_gen_output.diagnostics[i];
        valid_diag_count += diag.valid ? 1u : 0u;
        collision_free_diag_count += diag.collision_free ? 1u : 0u;
        if (i < 8u || diag.valid) {
            RCLCPP_INFO(
                node_->get_logger(),
                "[dual_arm] q_goal_diag[%zu]: valid=%s collision_free=%s reason=%s left_pos=%.4f left_rot=%.4f right_pos=%.4f right_rot=%.4f margin=%.5f",
                i,
                diag.valid ? "true" : "false",
                diag.collision_free ? "true" : "false",
                diag.reason.c_str(),
                diag.left_position_error_m,
                diag.left_orientation_error_rad,
                diag.right_position_error_m,
                diag.right_orientation_error_rad,
                diag.min_margin);
        }
    }
    for (std::size_t i = 0; i < input.q_goal_candidates.size() && i < 3u; ++i) {
        RCLCPP_INFO(
            node_->get_logger(),
            "[dual_arm] verified whole-body q_goal_candidate[%zu]=%s",
            i,
            formatVector(input.q_goal_candidates[i]).c_str());
    }

    if (input.q_goal_candidates.empty()) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[dual_arm] failed to build verified whole-body goal candidates: base_candidates=%zu diagnostics=%zu valid=%zu collision_free=%zu",
            base_candidates.size(),
            goal_gen_output.diagnostics.size(),
            valid_diag_count,
            collision_free_diag_count);
        last_execution_success_[mapping] = false;
        return false;
    }
    input.joint_segment_validator =
        [ctx, collision_map, base_footprint_checker](
            const Eigen::VectorXd& q_from,
            const Eigen::VectorXd& q_to,
            const double safe_distance,
            cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            if (!validateFullStateSegment(
                    q_from, q_to, *ctx, collision_map, safe_distance, diag)) {
                return false;
            }
            if (q_from.size() < kBaseDof || q_to.size() < kBaseDof) {
                if (diag != nullptr) {
                    diag->reason = "invalid_full_segment_size";
                    diag->min_margin = -1.0;
                }
                return false;
            }
            const cp::BaseFootprintCollisionChecker::Diagnostic base_diag =
                base_footprint_checker->checkSegment(
                    cp::BaseFootprintCollisionChecker::BaseState{
                        q_from[0], q_from[1], q_from[2]},
                    cp::BaseFootprintCollisionChecker::BaseState{
                        q_to[0], q_to[1], q_to[2]},
                    safe_distance);
            if (!base_diag.collision_free) {
                if (diag != nullptr) {
                    applyBaseFootprintDiagnostic(
                        base_diag, safe_distance, diag);
                }
                return false;
            }
            return true;
        };
    input.manipulator_segment_validator =
        [ctx, collision_map](
            const Eigen::VectorXd& q_from,
            const Eigen::VectorXd& q_to,
            const double safe_distance,
            cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            return validateManipulatorSegmentForBaseGuide(
                q_from, q_to, *ctx, collision_map, safe_distance, diag);
        };
    input.joint_to_pose_fn =
        [](const Eigen::VectorXd& q, cp::CartesianWaypoint& wp) -> bool {
            if (q.size() != kFullDof) {
                return false;
            }
            wp.position = Eigen::Vector3d(q[0], q[1], 0.0);
            wp.orientation =
                Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
            return true;
        };

    RCLCPP_INFO(
        node_->get_logger(),
        "[dual_arm] whole-body base-guided planner using %zu verified q_goal candidates from %zu base seeds; q_start=%s",
        input.q_goal_candidates.size(),
        base_candidates.size(),
        formatVector(q_start).c_str());

    gp::BaseGuidedWholeBodyPlanner base_guided_planner(runtime_cfg_.planner_common);
    cp::TimedJointTrajectory trajectory = base_guided_planner.planTrajectory(input);
    if (trajectory.empty()) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[dual_arm] base-guided whole-body planner failed to produce a REMANI-style 15D global reference; not falling back to unguided 15D RRTConnect");
        last_execution_success_[mapping] = false;
        return false;
    }

    const Eigen::VectorXd& q_final = trajectory.joint_targets.back();
    RCLCPP_INFO(
        node_->get_logger(),
        "[dual_arm] whole-body global reference ready: states=%zu duration=%.3f final_base=(%.3f, %.3f, %.3f) final_arm=%s",
        trajectory.joint_targets.size(),
        trajectory.total_duration,
        q_final[0],
        q_final[1],
        q_final[2],
        formatVector(q_final.segment(kBaseDof, kArmDof)).c_str());
    logWholeBodyGlobalArmReference(node_->get_logger(), trajectory);

    const bool executed =
        executeWholeBodyReference(mapping, trajectory, input, *ctx, collision_map);
    last_execution_success_[mapping] = executed;
    return executed;
}

bool ReactiveTaskController::executeWholeBodyReference(
    const std::string& mapping,
    const cp::TimedJointTrajectory& trajectory,
    const cp::PathPlanningInput& planning_input,
    const MappingContext& ctx,
    std::shared_ptr<const cp::DistanceFieldInterface> collision_map) {
    if (mapping != "dual_arm" || trajectory.empty()) {
        return false;
    }
    if (runtime_cfg_.command_output != "gazebo") {
        RCLCPP_WARN(
            node_->get_logger(),
            "[dual_arm] 15D whole-body reference execution is currently wired for command_output=gazebo; command_output=%s",
            runtime_cfg_.command_output.c_str());
    }

    const auto start_time = std::chrono::steady_clock::now();
    const double dt = std::max(1e-3, runtime_cfg_.neo_control_cycle_sec);
    const double kp_base_xy = kWholeBodyTrackingKpBaseXy;
    const double kp_base_yaw = kWholeBodyTrackingKpBaseYaw;
    const double kp_arm = kWholeBodyTrackingKpArm;
    const double max_duration =
        trajectory.total_duration + std::max(1.0, 2.0 * dt);
    WholeBodyQpVelocityBounds velocity_bounds;
    velocity_bounds.qd_min = Eigen::VectorXd::Zero(kFullDof);
    velocity_bounds.qd_max = Eigen::VectorXd::Zero(kFullDof);
    velocity_bounds.qd_min[0] = -runtime_cfg_.base_max_vx;
    velocity_bounds.qd_max[0] = runtime_cfg_.base_max_vx;
    velocity_bounds.qd_min[1] = -runtime_cfg_.base_max_vy;
    velocity_bounds.qd_max[1] = runtime_cfg_.base_max_vy;
    velocity_bounds.qd_min[2] = -runtime_cfg_.base_max_wz;
    velocity_bounds.qd_max[2] = runtime_cfg_.base_max_wz;
    velocity_bounds.qd_min.segment(kBaseDof, kArmDof).setConstant(
        -kWholeBodyTrackingMaxArmQdot);
    velocity_bounds.qd_max.segment(kBaseDof, kArmDof).setConstant(
        kWholeBodyTrackingMaxArmQdot);
    rq::ReactiveQpSolver whole_body_solver;
    WholeBodyQpCollisionContext qp_collision_ctx;
    qp_collision_ctx.fk_provider = ctx.fk_provider.get();
    qp_collision_ctx.jacobian_provider = ctx.jacobian_provider.get();
    qp_collision_ctx.collision_ellipsoids = &ctx.collision_ellipsoids;
    rt::WholeBodyLocalPlanner::Config whole_body_local_cfg;
    whole_body_local_cfg.horizon_steps =
        std::max(kWholeBodyLocalMinHorizonSteps,
                 runtime_cfg_.local_planner.horizon_steps);
    whole_body_local_cfg.dt_sec =
        std::max(1e-3, runtime_cfg_.local_planner.dt_sec);
    whole_body_local_cfg.optimization_iterations = 32;
    whole_body_local_cfg.reference_weight = 0.005;
    whole_body_local_cfg.base_reference_weight = 1.5;
    whole_body_local_cfg.yaw_reference_weight = 0.3;
    whole_body_local_cfg.base_progress_weight = 35000.0;
    whole_body_local_cfg.smoothness_weight = 0.20;
    whole_body_local_cfg.current_state_weight = 0.20;
    whole_body_local_cfg.collision_weight = 650000.0;
    whole_body_local_cfg.time_weight = 1.0;
    whole_body_local_cfg.obstacle_safe_margin =
        std::max(planning_input.safe_distance, kWholeBodyLocalObstacleSafeMarginM);
    whole_body_local_cfg.constrain_points_per_piece = 24;
    whole_body_local_cfg.collision_repair_step = 0.04;
    whole_body_local_cfg.collision_repair_samples = 6;
    whole_body_local_cfg.max_base_vx = runtime_cfg_.base_max_vx;
    whole_body_local_cfg.max_base_vy = runtime_cfg_.base_max_vy;
    whole_body_local_cfg.max_base_wz = runtime_cfg_.base_max_wz;
    whole_body_local_cfg.max_arm_qdot = kWholeBodyTrackingMaxArmQdot;
    rt::WholeBodyLocalPlanner whole_body_local_planner(whole_body_local_cfg);
    Eigen::VectorXd previous_qdot = Eigen::VectorXd::Zero(kFullDof);
    bool previous_qdot_valid = false;
    double tracking_time_sec = 0.0;
    auto last_cycle_time = start_time;
    Eigen::Vector3d previous_base_twist = Eigen::Vector3d::Zero();
    bool previous_base_twist_valid = false;

    RCLCPP_INFO(
        node_->get_logger(),
        "[dual_arm] executing 15D whole-body reference through NEO QP: duration=%.3f dt=%.4f command_output=%s",
        trajectory.total_duration,
        dt,
        runtime_cfg_.command_output.c_str());

    bool reached = false;
    while (rclcpp::ok() && is_active(mapping)) {
        const auto cycle_start = std::chrono::steady_clock::now();
        const double cycle_dt =
            std::max(1e-3, std::chrono::duration<double>(
                               cycle_start - last_cycle_time)
                               .count());
        last_cycle_time = cycle_start;
        const double elapsed =
            std::chrono::duration<double>(cycle_start - start_time).count();
        if (elapsed > max_duration) {
            break;
        }

        const std::optional<BaseState> base_now_opt = readBaseStateFromTf(
            *tf_buffer_,
            runtime_cfg_.mobile_base_odom_frame,
            runtime_cfg_.mobile_base_frame,
            node_->get_logger());
        if (!base_now_opt.has_value()) {
            break;
        }
        const BaseState base_now = *base_now_opt;

        const std::vector<double> q_now_vec =
            hardware_manager_->get_current_joint_positions_lockfree(mapping);
        if (q_now_vec.size() != kArmDof) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[dual_arm] whole-body execution joint feedback size mismatch: expected %d got %zu",
                kArmDof,
                q_now_vec.size());
            break;
        }
        const Eigen::VectorXd q_arm_now = Eigen::Map<const Eigen::VectorXd>(
            q_now_vec.data(), static_cast<Eigen::Index>(q_now_vec.size()));

        const Eigen::VectorXd q_current_full =
            makeCurrentWholeBodyState(base_now, q_arm_now);
        const double base_projected_time = projectWholeBodyStateToTrajectoryTime(
            trajectory, q_current_full, tracking_time_sec);
        const double max_tracking_lead =
            std::max(whole_body_local_cfg.dt_sec,
                     0.5 * whole_body_local_cfg.dt_sec *
                         static_cast<double>(whole_body_local_cfg.horizon_steps));
        const double predicted_time = std::min(
            trajectory.total_duration,
            std::max(tracking_time_sec, base_projected_time) + cycle_dt);
        const double projection_lead_limit =
            base_projected_time + max_tracking_lead;
        tracking_time_sec = std::clamp(
            std::min(predicted_time, projection_lead_limit),
            0.0,
            trajectory.total_duration);

        WholeBodyReferenceSample ref;
        WholeBodyReferenceSample ref_next;
        if (!sampleWholeBodyReference(trajectory, tracking_time_sec, &ref) ||
            !sampleWholeBodyReference(
                trajectory, tracking_time_sec + dt, &ref_next)) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[dual_arm] whole-body reference sample failed at tracking_t=%.3f elapsed=%.3f",
                tracking_time_sec,
                elapsed);
            break;
        }
        const WholeBodyReferenceSample global_ref_for_progress = ref;
        const WholeBodyReferenceSample global_ref_next_for_progress = ref_next;

        rt::WholeBodyLocalPlanner::Output local_output;
        rt::WholeBodyLocalPlanner::Input local_input;
        local_input.q_current = q_current_full;
        local_input.global_reference = trajectory;
        local_input.global_time_sec = tracking_time_sec;
        local_input.safe_distance = std::max(0.0, planning_input.hard_clearance);
        local_input.q_min = planning_input.q_min;
        local_input.q_max = planning_input.q_max;
        local_input.joint_state_validator = planning_input.joint_state_validator;
        local_input.joint_segment_validator = planning_input.joint_segment_validator;
        local_input.collision_cost_gradient_fn =
            [&ctx, collision_map](
                const Eigen::VectorXd& q,
                const double safe_distance) {
                return computeFullStateCollisionCostGradient(
                    q, ctx, collision_map, safe_distance);
            };
        const bool local_ok =
            whole_body_local_planner.compute(local_input, &local_output);
        double command_dt = dt;
        if (local_ok && local_output.joint_targets.size() >= 2u) {
            const std::size_t ref_index =
                std::min<std::size_t>(1u, local_output.joint_targets.size() - 1u);
            const std::size_t next_index =
                std::min<std::size_t>(ref_index + 1u,
                                      local_output.joint_targets.size() - 1u);
            command_dt = std::max(1e-3, local_output.dt_sec);
            splitWholeBodyReferenceState(
                local_output.joint_targets[ref_index],
                tracking_time_sec + command_dt * static_cast<double>(ref_index),
                &ref);
            splitWholeBodyReferenceState(
                local_output.joint_targets[next_index],
                tracking_time_sec + command_dt * static_cast<double>(next_index),
                &ref_next);
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                1000,
                "[dual_arm] whole-body local planner active: steps=%d dt=%.3f replanned=%d repaired=%d truncated=%d",
                local_output.sampled_steps,
                local_output.dt_sec,
                local_output.replanned_window ? 1 : 0,
                local_output.repaired_states,
                local_output.truncated_by_collision ? 1 : 0);
            if (local_output.used_frontend_fallback) {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    1000,
                    "[dual_arm] whole-body local planner using validated frontend window: %s",
                    local_output.error.c_str());
            } else if (local_output.used_optimized_trajectory) {
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    1000,
                    "[dual_arm] whole-body local planner using optimized MINCO trajectory");
            }
        } else {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[dual_arm] whole-body local planner failed; stopping instead of tracking unsafe global reference directly: %s",
                local_output.error.c_str());
            stop_whole_body_motion();
            break;
        }

        const Eigen::Vector2d world_error(
            ref.base.x - base_now.x,
            ref.base.y - base_now.y);
        const WholeBodyTrackingCommand command =
            buildWholeBodyTrackingCommand(
                ref,
                ref_next,
                base_now,
                q_arm_now,
                command_dt,
                kp_base_xy,
                kp_base_yaw,
                kp_arm);

        Eigen::VectorXd qdot_solution;
        std::string qp_error;
        if (!solveWholeBodyTrackingQp(
                base_now,
                q_arm_now,
                command,
                velocity_bounds,
                planning_input.q_min,
                planning_input.q_max,
                qp_collision_ctx,
                collision_map,
                planning_input.safe_distance,
                previous_qdot,
                previous_qdot_valid,
                reactive_cfg_.qp_build,
                whole_body_solver,
                &qdot_solution,
                &qp_error)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "[dual_arm] whole-body NEO QP failed, falling back to nominal tracking command: %s",
                qp_error.c_str());
            qdot_solution = packWholeBodyVelocity(command);
        }
        WholeBodyTrackingCommand qp_command = unpackWholeBodyVelocity(qdot_solution);
        qp_command.base_body_twist = limitBaseTwistRate(
            qp_command.base_body_twist,
            previous_base_twist,
            cycle_dt,
            previous_base_twist_valid);
        qp_command.base_body_twist =
            suppressNegativeGlobalPathProgress(
                qp_command.base_body_twist,
                base_now,
                global_ref_for_progress,
                global_ref_next_for_progress);
        previous_base_twist = qp_command.base_body_twist;
        previous_base_twist_valid = true;
        qdot_solution.head(kBaseDof) = qp_command.base_body_twist;
        previous_qdot = qdot_solution;
        previous_qdot_valid = qdot_solution.size() == kFullDof &&
                              qdot_solution.allFinite();
        publish_chassis_twist(qp_command.base_body_twist);

        const std::vector<double> arm_cmd_vec =
            clampArmVelocity(qp_command.arm_qdot);
        if (!send_gazebo_arm_joint_velocities(mapping, arm_cmd_vec)) {
            break;
        }

        const double base_pos_err = world_error.norm();
        const double base_yaw_err =
            std::abs(normalizeAngle(ref.base.yaw - base_now.yaw));
        const double arm_err = (ref.arm - q_arm_now).norm();
        const double arm_cmd_norm =
            qp_command.arm_qdot.size() == kArmDof ? qp_command.arm_qdot.norm() : 0.0;
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            500,
            "[dual_arm] whole-body track t=%.2f/%.2f elapsed=%.2f base_err=%.3f yaw_err=%.3f arm_err=%.3f arm_cmd_norm=%.3f cmd_base=(%.3f %.3f %.3f)",
            tracking_time_sec,
            trajectory.total_duration,
            elapsed,
            base_pos_err,
            base_yaw_err,
            arm_err,
            arm_cmd_norm,
            qp_command.base_body_twist.x(),
            qp_command.base_body_twist.y(),
            qp_command.base_body_twist.z());

        if (tracking_time_sec >= trajectory.total_duration &&
            base_pos_err < 0.08 &&
            base_yaw_err < 0.12 &&
            arm_err < 0.18) {
            reached = true;
            break;
        }

        const double cycle_elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - cycle_start)
                .count();
        if (cycle_elapsed < dt) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(dt - cycle_elapsed));
        }
    }

    stop_whole_body_motion();
    if (reached) {
        RCLCPP_INFO(node_->get_logger(), "[dual_arm] whole-body reference reached.");
    } else {
        RCLCPP_WARN(node_->get_logger(), "[dual_arm] whole-body reference execution stopped before tolerance.");
    }
    return reached;
}
