#include "algorithm/global_planner/base_guided_whole_body_planner.hpp"

#include "algorithm/cartesian_path_planner/base/kino_astar_base_planner.hpp"
#include "algorithm/global_planner/layer_gap_rrt_connector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <random>

namespace arm_controller::algorithm::global_planner {

namespace {
constexpr int kBaseDof = 3;
constexpr int kMinWholeBodyDof = 4;
constexpr double kMinBaseStepM = 0.04;
constexpr double kMaxBaseStepM = 0.16;
constexpr double kMaxYawStepRad = 0.18;
constexpr double kMinSegmentDt = 1e-3;
constexpr std::size_t kMaxArmCandidatesPerLayer = 64;
constexpr std::size_t kMaxGoalCandidatesToTry = 4;
constexpr std::size_t kMaxRoutesPerGoalToTry = 8;
constexpr int kLayerRrtMaxIterations = 12000;
constexpr double kLayerRrtOppositeTreeBias = 0.20;
constexpr double kLayerRrtEndpointBias = 0.10;
constexpr double kLayerRrtMaxJointVelRadPerSec = 0.90;
constexpr double kTrajectoryArmMaxJointVelRadPerSec = 0.45;
constexpr double kTrajectoryArmMaxJointAccRadPerSec2 = 0.45;
constexpr double kLayerRrtExtensionVelocityScale = 0.30;
constexpr double kLayerRrtTimeScale = 3.0;
constexpr double kMinArmLayerDt = 0.16;
constexpr double kLayerRrtMeetToleranceRad = 1e-4;
constexpr double kLayerEdgeCheckBaseStepM = 0.03;
constexpr double kLayerEdgeCheckYawStepRad = 0.10;
constexpr double kLayerEdgeCheckJointStepRad = 0.08;
constexpr double kKinoBaseClearanceBufferM = 0.12;
constexpr int kArmPathSmoothingIterations = 8;
constexpr int kLayerExplorationSamples = 48;
constexpr int kLayerSeedExplorationSamples = 12;
constexpr int kSampleManiRandomStateAttempts = 80;
constexpr int kRemaniCarStateCheckNum = 5;
constexpr int kSampleManiEndpointPullLayerBand = 3;
constexpr double kSampleManiPreferredClearanceM = 0.18;
constexpr double kSampleManiEarlyAcceptClearanceM = 0.16;
constexpr double kSampleManiMinimumAcceptClearanceM = 0.0;
constexpr double kSampleManiPlanningClearanceBufferM = 0.10;
constexpr double kSampleManiClearanceDeficitWeight = 12000.0;
constexpr double kSampleManiNodeClearanceCostScale = 0.60;

enum class LayerTreeState {
    Unvisited,
    Forward,
    Backward,
};

struct LayerTreeNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::VectorXd arm;
    int layer{0};
    int parent{-1};
    double g_score{0.0};
    double clearance_margin{0.0};
    LayerTreeState state{LayerTreeState::Unvisited};
};

std::string layerTreeNodeKey(const int layer, const Eigen::VectorXd& arm) {
    std::string key = std::to_string(layer);
    key.push_back(':');
    for (Eigen::Index i = 0; i < arm.size(); ++i) {
        key += std::to_string(static_cast<int>(std::llround(arm[i] * 100.0)));
        key.push_back(',');
    }
    return key;
}

double feasibilitySafeDistance(const cp::PathPlanningInput& input) {
    if (std::isfinite(input.feasibility_safe_distance) &&
        input.feasibility_safe_distance > 0.0) {
        return std::min(input.safe_distance, input.feasibility_safe_distance);
    }
    if (std::isfinite(input.hard_clearance)) {
        return std::max(0.0, input.hard_clearance);
    }
    return 0.0;
}

bool isWholeBodyInput(const cp::PathPlanningInput& input) {
    return input.q_start_seed.has_value() &&
           input.q_start_seed->size() >= kMinWholeBodyDof &&
           !input.q_goal_candidates.empty() &&
           input.joint_state_validator &&
           input.joint_segment_validator &&
           input.mobile_base_state_validator &&
           input.mobile_base_segment_validator;
}

bool validateLayerState(
    const cp::PathPlanningInput& input,
    const Eigen::VectorXd& q,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
    const auto& validator =
        input.manipulator_state_validator ? input.manipulator_state_validator
                                          : input.joint_state_validator;
    return validator && validator(q, safe_distance, diag);
}

bool validateLayerSegment(
    const cp::PathPlanningInput& input,
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
    const auto& validator =
        input.manipulator_segment_validator ? input.manipulator_segment_validator
                                            : input.joint_segment_validator;
    return validator && validator(q_from, q_to, safe_distance, diag);
}

double normalizeAngleLocal(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

BaseGuidedWholeBodyPlanner::BaseState interpolateBaseState(
    const BaseGuidedWholeBodyPlanner::BaseState& from,
    const BaseGuidedWholeBodyPlanner::BaseState& to,
    const double t) {
    BaseGuidedWholeBodyPlanner::BaseState base;
    base.x = from.x + t * (to.x - from.x);
    base.y = from.y + t * (to.y - from.y);
    base.yaw = normalizeAngleLocal(
        from.yaw + t * normalizeAngleLocal(to.yaw - from.yaw));
    return base;
}

bool validateLayerEdgeAlongBase(
    const std::vector<BaseGuidedWholeBodyPlanner::BaseState>& route,
    const std::vector<std::vector<BaseGuidedWholeBodyPlanner::BaseState>>&
        segment_check_states,
    const int from_layer,
    const int to_layer,
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    const cp::PathPlanningInput& input,
    const double safe_distance,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* failed_diag) {
    if (q_from.size() != q_to.size() || q_from.size() <= kBaseDof ||
        std::abs(to_layer - from_layer) != 1 ||
        from_layer < 0 || to_layer < 0 ||
        from_layer >= static_cast<int>(route.size()) ||
        to_layer >= static_cast<int>(route.size())) {
        if (failed_diag != nullptr) {
            failed_diag->reason = "invalid_layer_edge";
            failed_diag->min_margin = -1.0;
        }
        return false;
    }

    const int segment_index = std::min(from_layer, to_layer);
    if (segment_index >= 0 &&
        segment_index < static_cast<int>(segment_check_states.size()) &&
        !segment_check_states[static_cast<std::size_t>(segment_index)].empty()) {
        const auto& checks =
            segment_check_states[static_cast<std::size_t>(segment_index)];
        Eigen::VectorXd previous_q;
        for (std::size_t i = 0; i < checks.size(); ++i) {
            const double raw_t =
                checks.size() <= 1u
                    ? 1.0
                    : static_cast<double>(i) /
                          static_cast<double>(checks.size() - 1u);
            const double local_t =
                from_layer < to_layer ? raw_t : 1.0 - raw_t;
            Eigen::VectorXd q = q_from + local_t * (q_to - q_from);
            const BaseGuidedWholeBodyPlanner::BaseState& base = checks[i];
            q[0] = base.x;
            q[1] = base.y;
            q[2] = base.yaw;

            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (!validateLayerState(input, q, safe_distance, &diag)) {
                if (failed_diag != nullptr) {
                    diag.failed_on_segment_sample = true;
                    diag.failed_segment_t = local_t;
                    *failed_diag = diag;
                }
                return false;
            }
            if (i > 0) {
                cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
                if (!validateLayerSegment(
                        input, previous_q, q, safe_distance, &segment_diag)) {
                    if (failed_diag != nullptr) {
                        segment_diag.failed_on_segment_sample = true;
                        const double inner_t =
                            std::clamp(segment_diag.failed_segment_t, 0.0, 1.0);
                        segment_diag.failed_segment_t =
                            (static_cast<double>(i - 1u) + inner_t) /
                            static_cast<double>(checks.size() - 1u);
                        *failed_diag = segment_diag;
                    }
                    return false;
                }
            }
            previous_q = std::move(q);
        }
        return true;
    }

    const BaseGuidedWholeBodyPlanner::BaseState& base_from =
        route[static_cast<std::size_t>(segment_index)];
    const BaseGuidedWholeBodyPlanner::BaseState& base_to =
        route[static_cast<std::size_t>(segment_index + 1)];
    const double base_dist =
        std::hypot(base_to.x - base_from.x, base_to.y - base_from.y);
    const double yaw_dist =
        std::abs(normalizeAngleLocal(base_to.yaw - base_from.yaw));
    const double joint_dist =
        (q_to.segment(kBaseDof, q_to.size() - kBaseDof) -
         q_from.segment(kBaseDof, q_from.size() - kBaseDof))
            .cwiseAbs()
            .maxCoeff();
    const int steps = std::max(
        2,
        std::max(
            static_cast<int>(std::ceil(base_dist / kLayerEdgeCheckBaseStepM)),
            std::max(
                static_cast<int>(std::ceil(yaw_dist / kLayerEdgeCheckYawStepRad)),
                static_cast<int>(std::ceil(joint_dist / kLayerEdgeCheckJointStepRad)))));

    Eigen::VectorXd previous_q;
    for (int i = 0; i <= steps; ++i) {
        const double local_t =
            static_cast<double>(i) / static_cast<double>(steps);
        const double t = from_layer < to_layer ? local_t : 1.0 - local_t;
        const BaseGuidedWholeBodyPlanner::BaseState base =
            interpolateBaseState(base_from, base_to, t);
        Eigen::VectorXd q = q_from + local_t * (q_to - q_from);
        q[0] = base.x;
        q[1] = base.y;
        q[2] = base.yaw;

        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateLayerState(input, q, safe_distance, &diag)) {
            if (failed_diag != nullptr) {
                diag.failed_on_segment_sample = true;
                diag.failed_segment_t = local_t;
                *failed_diag = diag;
            }
            return false;
        }
        if (i > 0) {
            cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
            if (!validateLayerSegment(
                    input, previous_q, q, safe_distance, &segment_diag)) {
                if (failed_diag != nullptr) {
                    segment_diag.failed_on_segment_sample = true;
                    const double inner_t =
                        std::clamp(segment_diag.failed_segment_t, 0.0, 1.0);
                    segment_diag.failed_segment_t =
                        (static_cast<double>(i - 1) + inner_t) /
                        static_cast<double>(steps);
                    *failed_diag = segment_diag;
                }
                return false;
            }
        }
        previous_q = std::move(q);
    }
    return true;
}

double jointMotionCost(const Eigen::VectorXd& from, const Eigen::VectorXd& to) {
    if (from.size() != to.size() || from.size() <= kBaseDof) {
        return std::numeric_limits<double>::infinity();
    }
    const double base_motion = (to.head<2>() - from.head<2>()).norm();
    const double yaw_motion = std::abs(normalizeAngleLocal(to[2] - from[2]));
    const double arm_motion =
        (to.segment(kBaseDof, to.size() - kBaseDof) -
         from.segment(kBaseDof, from.size() - kBaseDof))
            .norm();
    return base_motion + 0.20 * yaw_motion + 0.15 * arm_motion;
}

double armTransitionCost(
    const Eigen::VectorXd& from_arm,
    const Eigen::VectorXd& to_arm,
    const double dt) {
    if (from_arm.size() != to_arm.size() || from_arm.size() <= 0) {
        return std::numeric_limits<double>::infinity();
    }
    const double safe_dt = std::max(1e-3, dt);
    return (to_arm - from_arm).lpNorm<1>() / safe_dt;
}

double sampleManiNodeClearancePenalty(const double margin) {
    if (!std::isfinite(margin)) {
        return kSampleManiClearanceDeficitWeight *
               kSampleManiPreferredClearanceM *
               kSampleManiPreferredClearanceM *
               kSampleManiNodeClearanceCostScale;
    }
    const double deficit =
        std::max(0.0, kSampleManiPreferredClearanceM - margin);
    return kSampleManiClearanceDeficitWeight * deficit * deficit *
           kSampleManiNodeClearanceCostScale;
}

bool armVelocityFeasible(
    const Eigen::VectorXd& from_arm,
    const Eigen::VectorXd& to_arm,
    const double dt) {
    if (from_arm.size() != to_arm.size() || from_arm.size() <= 0) {
        return false;
    }
    const double safe_dt = std::max(1e-3, dt);
    const double max_delta =
        kLayerRrtMaxJointVelRadPerSec * safe_dt;
    return (to_arm - from_arm).cwiseAbs().maxCoeff() <= max_delta;
}

int frontierLayer(
    const std::vector<LayerTreeNode>& nodes,
    const LayerTreeState state,
    const bool forward_tree) {
    int layer = forward_tree ? 0 : std::numeric_limits<int>::max();
    bool found = false;
    for (const LayerTreeNode& node : nodes) {
        if (node.state != state) {
            continue;
        }
        found = true;
        if (forward_tree) {
            layer = std::max(layer, node.layer);
        } else {
            layer = std::min(layer, node.layer);
        }
    }
    if (!found) {
        return 0;
    }
    return layer;
}

std::vector<Eigen::VectorXd> reconstructLayerTreePath(
    const std::vector<LayerTreeNode>& nodes,
    const std::vector<BaseGuidedWholeBodyPlanner::BaseState>& route,
    int forward_index,
    int backward_index) {
    std::vector<Eigen::VectorXd> prefix;
    for (int index = forward_index; index >= 0; index = nodes[index].parent) {
        const LayerTreeNode& node = nodes[static_cast<std::size_t>(index)];
        Eigen::VectorXd q(kBaseDof + node.arm.size());
        const auto& base = route[static_cast<std::size_t>(node.layer)];
        q[0] = base.x;
        q[1] = base.y;
        q[2] = base.yaw;
        q.segment(kBaseDof, node.arm.size()) = node.arm;
        prefix.push_back(q);
    }
    std::reverse(prefix.begin(), prefix.end());

    std::vector<Eigen::VectorXd> suffix;
    for (int index = backward_index; index >= 0; index = nodes[index].parent) {
        const LayerTreeNode& node = nodes[static_cast<std::size_t>(index)];
        Eigen::VectorXd q(kBaseDof + node.arm.size());
        const auto& base = route[static_cast<std::size_t>(node.layer)];
        q[0] = base.x;
        q[1] = base.y;
        q[2] = base.yaw;
        q.segment(kBaseDof, node.arm.size()) = node.arm;
        suffix.push_back(q);
    }

    std::vector<Eigen::VectorXd> path = std::move(prefix);
    path.insert(path.end(), suffix.begin(), suffix.end());
    return path;
}

Eigen::VectorXd composeLayerState(
    const std::vector<BaseGuidedWholeBodyPlanner::BaseState>& route,
    const LayerTreeNode& node) {
    Eigen::VectorXd q(kBaseDof + node.arm.size());
    const auto& base = route[static_cast<std::size_t>(node.layer)];
    q[0] = base.x;
    q[1] = base.y;
    q[2] = base.yaw;
    q.segment(kBaseDof, node.arm.size()) = node.arm;
    return q;
}

std::vector<Eigen::VectorXd> composeFullPathFromArmPath(
    const std::vector<BaseGuidedWholeBodyPlanner::BaseState>& route,
    const std::vector<Eigen::VectorXd>& arm_path,
    const std::vector<int>& path_layers) {
    std::vector<Eigen::VectorXd> full_path;
    if (arm_path.size() != path_layers.size()) {
        return full_path;
    }
    full_path.reserve(arm_path.size());
    for (std::size_t i = 0; i < arm_path.size(); ++i) {
        const int layer = path_layers[i];
        if (layer < 0 || layer >= static_cast<int>(route.size())) {
            return {};
        }
        Eigen::VectorXd q(kBaseDof + arm_path[i].size());
        const auto& base = route[static_cast<std::size_t>(layer)];
        q[0] = base.x;
        q[1] = base.y;
        q[2] = base.yaw;
        q.segment(kBaseDof, arm_path[i].size()) = arm_path[i];
        full_path.push_back(std::move(q));
    }
    return full_path;
}

double deterministicUnitNoise(const double seed) {
    const double value = std::sin(seed * 12.9898 + 78.233) * 43758.5453;
    return value - std::floor(value);
}

double deterministicSignedNoise(
    const std::size_t layer_hint,
    const int sample_index,
    const Eigen::Index joint_index) {
    const double seed = static_cast<double>((layer_hint + 1u) * 1009u) +
                        static_cast<double>((sample_index + 1) * 917) +
                        static_cast<double>((joint_index + 1) * 101);
    return 2.0 * deterministicUnitNoise(seed) - 1.0;
}

double armPathMotionCost(
    const std::vector<Eigen::VectorXd>& path) {
    if (path.size() < 2) {
        return 0.0;
    }
    double cost = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (path[i].size() != path[i - 1].size() ||
            path[i].size() <= kBaseDof) {
            return std::numeric_limits<double>::infinity();
        }
        cost += (path[i].segment(kBaseDof, path[i].size() - kBaseDof) -
                 path[i - 1].segment(kBaseDof, path[i - 1].size() - kBaseDof))
                    .norm();
    }
    return cost;
}

double armPathAccelerationCost(
    const std::vector<Eigen::VectorXd>& path) {
    if (path.size() < 3) {
        return 0.0;
    }
    double cost = 0.0;
    for (std::size_t i = 1; i + 1u < path.size(); ++i) {
        if (path[i - 1].size() != path[i].size() ||
            path[i + 1].size() != path[i].size() ||
            path[i].size() <= kBaseDof) {
            return std::numeric_limits<double>::infinity();
        }
        const Eigen::Index arm_dof = path[i].size() - kBaseDof;
        const Eigen::VectorXd accel =
            path[i + 1].segment(kBaseDof, arm_dof) -
            2.0 * path[i].segment(kBaseDof, arm_dof) +
            path[i - 1].segment(kBaseDof, arm_dof);
        cost += accel.squaredNorm();
    }
    return cost;
}

double armPathSelectionCost(
    const std::vector<Eigen::VectorXd>& path) {
    return 0.35 * armPathMotionCost(path) +
           0.10 * armPathAccelerationCost(path);
}

struct PathClearanceCost {
    double cost{0.0};
    double min_margin{std::numeric_limits<double>::infinity()};
    std::string worst_link;
};

void accumulateClearanceDiagnostic(
    const cp::PathPlanningInput::WholeBodyPoseDiagnostic& diag,
    PathClearanceCost* out) {
    if (out == nullptr || !std::isfinite(diag.min_margin)) {
        return;
    }
    if (diag.min_margin < out->min_margin) {
        out->min_margin = diag.min_margin;
        out->worst_link = diag.worst_link_name;
    }
    const double deficit =
        std::max(0.0, kSampleManiPreferredClearanceM - diag.min_margin);
    out->cost +=
        kSampleManiClearanceDeficitWeight * deficit * deficit;
}

PathClearanceCost sampledPathClearanceCost(
    const std::vector<Eigen::VectorXd>& path,
    const cp::PathPlanningInput& input,
    const double safe_distance) {
    PathClearanceCost out;
    if (path.empty()) {
        out.cost = std::numeric_limits<double>::infinity();
        return out;
    }
    for (std::size_t i = 0; i < path.size(); ++i) {
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateLayerState(input, path[i], safe_distance, &diag)) {
            out.cost = std::numeric_limits<double>::infinity();
            out.min_margin = std::min(out.min_margin, diag.min_margin);
            out.worst_link = diag.worst_link_name;
            return out;
        }
        accumulateClearanceDiagnostic(diag, &out);
        if (i == 0) {
            continue;
        }

        const Eigen::VectorXd& from = path[i - 1u];
        const Eigen::VectorXd& to = path[i];
        if (from.size() != to.size() || from.size() <= kBaseDof) {
            out.cost = std::numeric_limits<double>::infinity();
            return out;
        }
        const double base_dist = (to.head<2>() - from.head<2>()).norm();
        const double yaw_dist = std::abs(normalizeAngleLocal(to[2] - from[2]));
        const double joint_dist =
            (to.segment(kBaseDof, to.size() - kBaseDof) -
             from.segment(kBaseDof, from.size() - kBaseDof))
                .cwiseAbs()
                .maxCoeff();
        const int steps = std::max(
            1,
            std::max(
                static_cast<int>(std::ceil(base_dist / kLayerEdgeCheckBaseStepM)),
                std::max(
                    static_cast<int>(std::ceil(yaw_dist / kLayerEdgeCheckYawStepRad)),
                    static_cast<int>(std::ceil(joint_dist / kLayerEdgeCheckJointStepRad)))));
        for (int step = 1; step < steps; ++step) {
            const double t =
                static_cast<double>(step) / static_cast<double>(steps);
            Eigen::VectorXd q = (1.0 - t) * from + t * to;
            q[2] = normalizeAngleLocal(
                from[2] + t * normalizeAngleLocal(to[2] - from[2]));
            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag_mid;
            if (!validateLayerState(input, q, safe_distance, &diag_mid)) {
                out.cost = std::numeric_limits<double>::infinity();
                out.min_margin = std::min(out.min_margin, diag_mid.min_margin);
                out.worst_link = diag_mid.worst_link_name;
                return out;
            }
            accumulateClearanceDiagnostic(diag_mid, &out);
        }
    }
    return out;
}

double clearanceAwarePathSelectionCost(
    const std::vector<Eigen::VectorXd>& path,
    const cp::PathPlanningInput& input,
    const double safe_distance,
    PathClearanceCost* clearance_out = nullptr) {
    PathClearanceCost clearance =
        sampledPathClearanceCost(path, input, safe_distance);
    if (clearance_out != nullptr) {
        *clearance_out = clearance;
    }
    if (!std::isfinite(clearance.cost)) {
        return std::numeric_limits<double>::infinity();
    }
    const double critical_deficit =
        std::max(0.0, kSampleManiEarlyAcceptClearanceM - clearance.min_margin);
    const double preferred_deficit =
        std::max(0.0, kSampleManiPreferredClearanceM - clearance.min_margin);
    return armPathSelectionCost(path) + clearance.cost +
           50000.0 * critical_deficit * critical_deficit +
           20000.0 * preferred_deficit * preferred_deficit;
}

std::vector<Eigen::VectorXd> densifyPathForSegmentValidation(
    const std::vector<Eigen::VectorXd>& path,
    const cp::PathPlanningInput& input,
    const double safe_distance) {
    if (path.size() < 2) {
        return path;
    }
    std::vector<Eigen::VectorXd> out;
    out.reserve(path.size());
    out.push_back(path.front());
    for (std::size_t i = 1; i < path.size(); ++i) {
        const Eigen::VectorXd& from = out.back();
        const Eigen::VectorXd& to = path[i];
        if (from.size() != to.size() || from.size() <= kBaseDof) {
            return {};
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic direct_diag;
        if (validateLayerSegment(input, from, to, safe_distance, &direct_diag)) {
            out.push_back(to);
            continue;
        }

        const double base_dist = (to.head<2>() - from.head<2>()).norm();
        const double yaw_dist =
            std::abs(normalizeAngleLocal(to[2] - from[2]));
        const double joint_dist =
            (to.segment(kBaseDof, to.size() - kBaseDof) -
             from.segment(kBaseDof, from.size() - kBaseDof))
                .cwiseAbs()
                .maxCoeff();
        const int steps = std::max(
            2,
            std::max(
                static_cast<int>(std::ceil(base_dist / kLayerEdgeCheckBaseStepM)),
                std::max(
                    static_cast<int>(std::ceil(yaw_dist / kLayerEdgeCheckYawStepRad)),
                    static_cast<int>(std::ceil(joint_dist / kLayerEdgeCheckJointStepRad)))));

        for (int step = 1; step <= steps; ++step) {
            const double t =
                static_cast<double>(step) / static_cast<double>(steps);
            Eigen::VectorXd q = (1.0 - t) * from + t * to;
            q[2] = normalizeAngleLocal(
                from[2] + t * normalizeAngleLocal(to[2] - from[2]));
            cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
            if (!validateLayerState(input, q, safe_distance, &state_diag)) {
                return {};
            }
            cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
            if (!validateLayerSegment(
                    input, out.back(), q, safe_distance, &segment_diag)) {
                return {};
            }
            out.push_back(std::move(q));
        }
    }
    return out;
}

}  // namespace

BaseGuidedWholeBodyPlanner::BaseGuidedWholeBodyPlanner(
    const cp::PlannerCommonConfig& common_cfg)
    : common_cfg_(common_cfg) {}

cp::TimedJointTrajectory BaseGuidedWholeBodyPlanner::planTrajectory(
    const cp::PathPlanningInput& input) {
    if (!isWholeBodyInput(input)) {
        std::printf(
            "[base_guided_planner] skipped: missing 15D start/goals or validators\n");
        return {};
    }

    const Eigen::VectorXd& q_start = *input.q_start_seed;
    const double safe_distance = feasibilitySafeDistance(input);
    cp::PathPlanningInput::WholeBodyPoseDiagnostic start_diag;
    if (!input.joint_state_validator(q_start, safe_distance, &start_diag)) {
        std::printf(
            "[base_guided_planner] start rejected: margin=%.5f reason=%s\n",
            start_diag.min_margin,
            start_diag.reason.c_str());
        return {};
    }

    const BaseState start_base = baseFromQ(q_start);
    JointPath best_path;
    std::vector<double> best_times;
    double best_score = std::numeric_limits<double>::infinity();

    for (std::size_t goal_index = 0;
         goal_index < input.q_goal_candidates.size() &&
         goal_index < kMaxGoalCandidatesToTry;
         ++goal_index) {
        const Eigen::VectorXd& q_goal = input.q_goal_candidates[goal_index];
        if (q_goal.size() != q_start.size() || !q_goal.allFinite()) {
            continue;
        }

        cp::PathPlanningInput::WholeBodyPoseDiagnostic goal_diag;
        if (!input.joint_state_validator(q_goal, safe_distance, &goal_diag)) {
            std::printf(
                "[base_guided_planner] goal=%zu rejected before routing: margin=%.5f reason=%s\n",
                goal_index,
                goal_diag.min_margin,
                goal_diag.reason.c_str());
            continue;
        }

        const BaseState goal_base = baseFromQ(q_goal);
        std::printf(
            "[base_guided_planner] goal=%zu route generation begin\n",
            goal_index);
        std::fflush(stdout);
        const std::vector<BaseRoute> routes =
            makeBaseRoutes(start_base, goal_base, q_start, input, safe_distance);
        std::printf(
            "[base_guided_planner] goal=%zu route generation done routes=%zu\n",
            goal_index,
            routes.size());
        std::fflush(stdout);
        for (std::size_t route_index = 0;
             route_index < routes.size() && route_index < kMaxRoutesPerGoalToTry;
             ++route_index) {
            std::printf(
                "[base_guided_planner] goal=%zu route=%zu layered graph begin layers=%zu\n",
                goal_index,
                route_index,
                routes[route_index].size());
            std::fflush(stdout);
            JointPath path = planLayeredArmGraph(
                routes[route_index], q_start, q_goal, input, safe_distance);
            std::printf(
                "[base_guided_planner] goal=%zu route=%zu layered graph done path_samples=%zu\n",
                goal_index,
                route_index,
                path.size());
            std::fflush(stdout);
            if (path.size() < 2) {
                continue;
            }
            path = densifyPathForSegmentValidation(path, input, safe_distance);
            if (path.size() < 2) {
                continue;
            }
            if (!validateLayerPath(path, input, safe_distance)) {
                std::printf(
                    "[base_guided_planner] goal=%zu route=%zu rejected by REMANI-style final validation; starting layer-RRT repair on same base route\n",
                    goal_index,
                    route_index);
                path = repairPathWithLayerRrt(
                    routes[route_index],
                    path,
                    q_start,
                    q_goal,
                    input,
                    safe_distance);
                path = densifyPathForSegmentValidation(path, input, safe_distance);
                std::printf(
                    "[base_guided_planner] goal=%zu route=%zu final-validation repair done path_samples=%zu\n",
                    goal_index,
                    route_index,
                    path.size());
                if (path.size() < 2 ||
                    !validateLayerPath(path, input, safe_distance)) {
                    continue;
                }
            }
            path = smoothArmPathConservatively(
                routes[route_index], std::move(path), input, safe_distance);
            if (!validateLayerPath(path, input, safe_distance)) {
                continue;
            }

            PathClearanceCost clearance;
            double score = 0.02 * static_cast<double>(path.size()) +
                           clearanceAwarePathSelectionCost(
                               path, input, safe_distance, &clearance);
            for (std::size_t i = 1; i < path.size(); ++i) {
                score += jointMotionCost(path[i - 1], path[i]);
            }
            if (score < best_score) {
                best_score = score;
                best_path = std::move(path);
                best_times = estimateWholeBodyPathSegmentTimes(
                    best_path, common_cfg_.default_segment_speed);
                std::printf(
                    "[base_guided_planner] best-so-far goal=%zu route=%zu layered_graph_samples=%zu duration=%.3f score=%.3f min_margin=%.4f worst=%s\n",
                    goal_index,
                    route_index,
                    best_path.size(),
                    std::accumulate(best_times.begin(), best_times.end(), 0.0),
                    best_score,
                    clearance.min_margin,
                    clearance.worst_link.empty() ? "none" : clearance.worst_link.c_str());
            }
        }
    }

    if (best_path.empty()) {
        std::printf(
            "[base_guided_planner] failed: no base-guided layered whole-body route passed segment validation\n");
        return {};
    }
    PathClearanceCost final_clearance;
    (void)clearanceAwarePathSelectionCost(
        best_path, input, safe_distance, &final_clearance);
    std::printf(
        "[base_guided_planner] selected safest candidate samples=%zu duration=%.3f score=%.3f min_margin=%.4f preferred=%.4f worst=%s\n",
        best_path.size(),
        std::accumulate(best_times.begin(), best_times.end(), 0.0),
        best_score,
        final_clearance.min_margin,
        kSampleManiPreferredClearanceM,
        final_clearance.worst_link.empty() ? "none" : final_clearance.worst_link.c_str());
    std::fflush(stdout);
    return buildTrajectory(best_path, best_times);
}

std::vector<BaseGuidedWholeBodyPlanner::BaseRoute>
BaseGuidedWholeBodyPlanner::makeBaseRoutes(
    const BaseState& start,
    const BaseState& goal,
    const Eigen::VectorXd& q_start,
    const cp::PathPlanningInput& input,
    const double safe_distance) const {
    std::vector<BaseRoute> routes;
    routes.reserve(1u);

    const auto& kino_state_validator =
        input.kino_mobile_base_state_validator
            ? input.kino_mobile_base_state_validator
            : input.mobile_base_state_validator;
    const auto& kino_segment_validator =
        input.kino_mobile_base_segment_validator
            ? input.kino_mobile_base_segment_validator
            : input.mobile_base_segment_validator;

    if (q_start.size() >= 3 && kino_state_validator) {
        cp::KinoAstarBasePlanner::Config kino_cfg;
        kino_cfg.kinematic_model =
            cp::KinoAstarBasePlanner::KinematicModel::Omnidirectional;
        kino_cfg.xy_resolution =
            std::clamp(common_cfg_.path_resolution * 5.0, 0.08, 0.14);
        kino_cfg.yaw_resolution = 3.15;
        kino_cfg.primitive_arc_length =
            std::clamp(common_cfg_.path_resolution * 16.0, 0.28, 0.55);
        kino_cfg.primitive_duration = std::clamp(
            kino_cfg.primitive_arc_length /
                std::max(1e-3, common_cfg_.default_segment_speed),
            0.25,
            0.80);
        kino_cfg.wheel_base = 0.40;
        kino_cfg.max_steer_angle = 0.70;
        kino_cfg.heuristic_weight = 2.5;
        kino_cfg.max_velocity = std::max(0.20, common_cfg_.default_segment_speed);
        kino_cfg.max_lateral_velocity = kino_cfg.max_velocity;
        kino_cfg.max_yaw_rate = 0.80;
        kino_cfg.forward_penalty = 1.0;
        kino_cfg.backward_penalty = 1.0;
        kino_cfg.reverse_penalty = 1.0;
        kino_cfg.gear_switch_penalty = 15.0;
        kino_cfg.steer_penalty = 0.50;
        kino_cfg.steer_change_penalty = 0.0;
        kino_cfg.oneshot_range = 0.0;
        kino_cfg.oneshot_check_len = 0.16;
        kino_cfg.reeds_shepp_turning_radii = {0.4, 0.2, 0.1};
        kino_cfg.goal_xy_tolerance = std::max(0.10, kino_cfg.xy_resolution * 1.5);
        kino_cfg.goal_yaw_tolerance = 3.15;
        const double bound_margin = 2.0;
        kino_cfg.map_min_x = std::min(start.x, goal.x) - bound_margin;
        kino_cfg.map_max_x = std::max(start.x, goal.x) + bound_margin;
        kino_cfg.map_min_y = std::min(start.y, goal.y) - bound_margin;
        kino_cfg.map_max_y = std::max(start.y, goal.y) + bound_margin;
        kino_cfg.clearance = safe_distance + kKinoBaseClearanceBufferM;
        kino_cfg.max_expansions = 80000;
        kino_cfg.max_solve_time_sec = std::clamp(
            0.75 * std::max(1.0, common_cfg_.joint_space_sampling_time_budget_sec),
            4.0,
            12.0);

        cp::KinoAstarBasePlanner::Input kino_input;
        kino_input.start = cp::KinoAstarBasePlanner::BaseState{
            start.x, start.y, start.yaw};
        kino_input.goal = cp::KinoAstarBasePlanner::BaseState{
            goal.x, goal.y, goal.yaw};
        kino_input.state_validator =
            [&kino_state_validator](
                const cp::KinoAstarBasePlanner::BaseState& state,
                const double clearance) {
                return kino_state_validator(
                    state.x, state.y, state.yaw, clearance);
            };
        if (kino_segment_validator) {
            kino_input.segment_validator =
                [&kino_segment_validator](
                    const cp::KinoAstarBasePlanner::BaseState& from,
                    const cp::KinoAstarBasePlanner::BaseState& to,
                    const double clearance) {
                    return kino_segment_validator(
                    from.x,
                    from.y,
                    from.yaw,
                    to.x,
                    to.y,
                    to.yaw,
                    clearance);
                };
        }

        auto make_remani_route =
            [&](const cp::KinoAstarBasePlanner::Result& kino_result) {
                BaseRoute route;
                if (!kino_result.success || kino_result.path.size() < 2u) {
                    return route;
                }

                std::vector<double> segment_times = kino_result.segment_times;
                if (segment_times.size() + 1u != kino_result.path.size()) {
                    segment_times.clear();
                    segment_times.reserve(kino_result.path.size() - 1u);
                    for (std::size_t i = 1; i < kino_result.path.size(); ++i) {
                        const auto& from = kino_result.path[i - 1u];
                        const auto& to = kino_result.path[i];
                        const double distance =
                            std::hypot(to.x - from.x, to.y - from.y);
                        segment_times.push_back(
                            std::max(
                                kMinSegmentDt,
                                distance /
                                    std::max(
                                        1e-3,
                                        common_cfg_.default_segment_speed)));
                    }
                }

                route.states.reserve(kino_result.path.size());
                route.segment_times.reserve(kino_result.path.size() - 1u);
                route.segment_check_states.reserve(kino_result.path.size() - 1u);
                route.dense_check_states.clear();
                route.singularities.clear();
                route.states.push_back(
                    BaseState{kino_result.path.front().x,
                              kino_result.path.front().y,
                              kino_result.path.front().yaw});

                for (std::size_t i = 1; i < kino_result.path.size(); ++i) {
                    const auto& from_raw = kino_result.path[i - 1u];
                    const auto& to_raw = kino_result.path[i];
                    const BaseState from{
                        from_raw.x, from_raw.y, from_raw.yaw};
                    const BaseState to{to_raw.x, to_raw.y, to_raw.yaw};
                    const double duration =
                        std::max(kMinSegmentDt, segment_times[i - 1u]);
                    const int singularity =
                        i - 1u < kino_result.singularities.size()
                            ? kino_result.singularities[i - 1u]
                            : 1;

                    std::vector<BaseState> checks;
                    checks.reserve(kRemaniCarStateCheckNum + 1);
                    for (int k = 0; k <= kRemaniCarStateCheckNum; ++k) {
                        const double alpha =
                            static_cast<double>(k) /
                            static_cast<double>(kRemaniCarStateCheckNum);
                        checks.push_back(interpolateBaseState(from, to, alpha));
                    }
                    route.dense_check_states.insert(
                        route.dense_check_states.end(),
                        checks.begin(),
                        checks.end());
                    route.segment_check_states.push_back(std::move(checks));
                    route.segment_times.push_back(duration);
                    route.singularities.push_back(singularity == 0 ? 1 : singularity);
                    route.states.push_back(to);
                }
                return route;
            };

        std::printf("[base_guided_planner] kino_astar base route begin\n");
        std::fflush(stdout);
        const cp::KinoAstarBasePlanner kino_planner(kino_cfg);
        const cp::KinoAstarBasePlanner::Result kino_result =
            kino_planner.plan(kino_input);
        if (kino_result.success && kino_result.path.size() >= 2) {
            BaseRoute route = make_remani_route(kino_result);
            if (route.size() < 2u ||
                route.segment_times.size() + 1u != route.size() ||
                route.segment_check_states.size() + 1u != route.size()) {
                std::printf(
                    "[base_guided_planner] kino_astar route rejected: invalid REMANI timing layers=%zu times=%zu checks=%zu\n",
                    route.size(),
                    route.segment_times.size(),
                    route.segment_check_states.size());
                return routes;
            }
            routes.push_back(std::move(route));
            std::printf(
                "[base_guided_planner] kino_astar base route accepted samples=%zu remani_layers=%zu check_edges=%zu expanded=%d cost=%.3f solve_time=%.3f\n",
                kino_result.path.size(),
                routes.back().size(),
                routes.back().segment_check_states.size(),
                kino_result.expanded_nodes,
                kino_result.cost,
                kino_result.solve_time_sec);
        } else {
            std::printf(
                "[base_guided_planner] kino_astar base route failed expanded=%d solve_time=%.3f timed_out=%s; no non-REMANI fallback route\n",
                kino_result.expanded_nodes,
                kino_result.solve_time_sec,
                kino_result.timed_out ? "true" : "false");
        }
    } else {
        std::printf(
            "[base_guided_planner] kino_astar skipped: missing mobile base footprint validator\n");
    }
    return routes;
}

BaseGuidedWholeBodyPlanner::JointPath
BaseGuidedWholeBodyPlanner::planLayeredArmGraph(
    const BaseRoute& route,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const cp::PathPlanningInput& input,
    const double safe_distance) const {
    if (route.size() < 2 || q_start.size() != q_goal.size()) {
        return {};
    }

    const std::vector<double> segment_times = estimateArmLayerSegmentTimes(
        route, common_cfg_.default_segment_speed);
    if (segment_times.size() + 1u != route.size()) {
        return {};
    }
    std::vector<Layer> sample_mani_layers(route.size());
    sample_mani_layers.front().push_back(q_start);
    sample_mani_layers.back().push_back(q_goal);

    std::printf(
        "[base_guided_planner] REMANI-style SampleMani search begin layers=%zu\n",
        route.size());
    std::fflush(stdout);
    JointPath sample_mani_path = planBidirectionalLayerRrt(
        route, sample_mani_layers, q_start, q_goal, input, safe_distance);
    std::printf(
        "[base_guided_planner] REMANI-style SampleMani search done path_samples=%zu\n",
        sample_mani_path.size());
    std::fflush(stdout);
    return sample_mani_path;
}

BaseGuidedWholeBodyPlanner::JointPath
BaseGuidedWholeBodyPlanner::planBidirectionalLayerRrt(
    const BaseRoute& route,
    const std::vector<Layer>& layers,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const cp::PathPlanningInput& input,
    const double safe_distance) const {
    if (route.size() < 2 || layers.size() != route.size() ||
        q_start.size() != q_goal.size()) {
        return {};
    }

    const std::vector<double> raw_segment_times = estimateRouteSegmentTimes(
        route, common_cfg_.default_segment_speed);
    const std::vector<double> arm_segment_times = estimateArmLayerSegmentTimes(
        route, common_cfg_.default_segment_speed);
    if (raw_segment_times.size() + 1 != route.size() ||
        arm_segment_times.size() + 1 != route.size()) {
        return {};
    }
    const double sample_mani_safe_distance =
        safe_distance + kSampleManiPlanningClearanceBufferM;

    std::vector<LayerTreeNode> nodes;
    nodes.reserve(layers.size() * kMaxArmCandidatesPerLayer + 128u);
    std::map<std::string, int> node_pool;
    std::vector<std::vector<int>> nodes_by_layer(route.size());
    auto stateClearanceMargin = [&](const Eigen::VectorXd& q) {
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateLayerState(input, q, sample_mani_safe_distance, &diag)) {
            return -std::numeric_limits<double>::infinity();
        }
        return diag.min_margin;
    };
    auto addNode = [&](const Eigen::VectorXd& q,
                       const int layer,
                       const int parent,
                       const double g_score,
                       const LayerTreeState state) -> int {
        const Eigen::VectorXd arm = q.segment(kBaseDof, q.size() - kBaseDof);
        const double clearance_margin = stateClearanceMargin(q);
        const std::string key = layerTreeNodeKey(layer, arm);
        const auto existing = node_pool.find(key);
        if (existing != node_pool.end()) {
            LayerTreeNode& node =
                nodes[static_cast<std::size_t>(existing->second)];
            node.clearance_margin =
                std::max(node.clearance_margin, clearance_margin);
            if (node.state == LayerTreeState::Unvisited) {
                node.state = state;
                node.parent = parent;
                node.g_score = g_score;
            } else if (node.state == state && g_score < node.g_score) {
                node.parent = parent;
                node.g_score = g_score;
            }
            return existing->second;
        }
        LayerTreeNode node;
        node.arm = arm;
        node.layer = layer;
        node.parent = parent;
        node.g_score = g_score;
        node.clearance_margin = clearance_margin;
        node.state = state;
        nodes.push_back(std::move(node));
        const int index = static_cast<int>(nodes.size() - 1u);
        node_pool.emplace(key, index);
        if (layer >= 0 && layer < static_cast<int>(nodes_by_layer.size())) {
            nodes_by_layer[static_cast<std::size_t>(layer)].push_back(index);
        }
        return index;
    };

    const int start_index = addNode(q_start, 0, -1, 0.0, LayerTreeState::Forward);
    const int goal_index = addNode(
        q_goal,
        static_cast<int>(route.size() - 1u),
        -1,
        0.0,
        LayerTreeState::Backward);
    (void)start_index;
    (void)goal_index;

    std::uint32_t seed = 2166136261u;
    for (Eigen::Index i = 0; i < q_start.size(); ++i) {
        seed ^= static_cast<std::uint32_t>(
            std::llround((q_start[i] + 17.0) * 10000.0));
        seed *= 16777619u;
        seed ^= static_cast<std::uint32_t>(
            std::llround((q_goal[i] + 19.0) * 10000.0));
        seed *= 16777619u;
    }
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    std::uniform_int_distribution<int> layer_dist(
        1, std::max(1, static_cast<int>(route.size()) - 2));
    const auto search_start = std::chrono::steady_clock::now();
    const double search_time_budget_sec = std::clamp(
        common_cfg_.joint_space_sampling_time_budget_sec, 0.5, 20.0);
    auto elapsedSearchSec = [&]() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - search_start)
            .count();
    };
    std::printf(
        "[base_guided_planner] SampleMani config layers=%zu segments=%zu time_scale=%.2f vel_scale=%.2f max_joint_vel=%.3f budget=%.3f opposite_bias=%.2f endpoint_bias=%.2f safe=%.4f sample_safe=%.4f preferred_clearance=%.4f min_accept=%.4f\n",
        route.size(),
        arm_segment_times.size(),
        kLayerRrtTimeScale,
        kLayerRrtExtensionVelocityScale,
        kLayerRrtMaxJointVelRadPerSec,
        search_time_budget_sec,
        kLayerRrtOppositeTreeBias,
        kLayerRrtEndpointBias,
        safe_distance,
        sample_mani_safe_distance,
        kSampleManiPreferredClearanceM,
        kSampleManiMinimumAcceptClearanceM);
    std::fflush(stdout);

    auto makeRandomLayerCandidate =
        [&](const int layer_index) -> std::optional<Eigen::VectorXd> {
        const bool has_limits =
            input.q_min.size() == q_start.size() &&
            input.q_max.size() == q_start.size();
        Eigen::VectorXd best_q;
        double best_margin = -std::numeric_limits<double>::infinity();
        if (has_limits) {
            for (int attempt = 0; attempt < kSampleManiRandomStateAttempts;
                 ++attempt) {
                Eigen::VectorXd q = q_start;
                q[0] = route[static_cast<std::size_t>(layer_index)].x;
                q[1] = route[static_cast<std::size_t>(layer_index)].y;
                q[2] = route[static_cast<std::size_t>(layer_index)].yaw;
                for (Eigen::Index joint = kBaseDof; joint < q.size(); ++joint) {
                    const double lo =
                        std::min(input.q_min[joint], input.q_max[joint]);
                    const double hi =
                        std::max(input.q_min[joint], input.q_max[joint]);
                    if (std::isfinite(lo) && std::isfinite(hi) && hi > lo) {
                        q[joint] = lo + unit_dist(rng) * (hi - lo);
                    }
                }
                cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
                if (validateLayerState(input, q, sample_mani_safe_distance, &diag)) {
                    if (diag.min_margin > best_margin) {
                        best_margin = diag.min_margin;
                        best_q = q;
                    }
                    if (diag.min_margin >= kSampleManiPreferredClearanceM) {
                        break;
                    }
                }
            }
            if (best_margin > -std::numeric_limits<double>::infinity()) {
                return best_q;
            }
        }

        const double t = static_cast<double>(layer_index) /
                         static_cast<double>(route.size() - 1u);
        for (int attempt = 0; attempt < kSampleManiRandomStateAttempts / 4;
             ++attempt) {
            Eigen::VectorXd q = interpolateState(
                q_start,
                q_goal,
                route[static_cast<std::size_t>(layer_index)],
                t);
            for (Eigen::Index joint = kBaseDof; joint < q.size(); ++joint) {
                const double lo =
                    has_limits ? std::min(input.q_min[joint], input.q_max[joint])
                               : q[joint] - M_PI;
                const double hi =
                    has_limits ? std::max(input.q_min[joint], input.q_max[joint])
                               : q[joint] + M_PI;
                const double span = std::max(0.2, hi - lo);
                q[joint] += (unit_dist(rng) - 0.5) * span;
                q[joint] = std::clamp(q[joint], lo, hi);
            }
            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (validateLayerState(input, q, sample_mani_safe_distance, &diag)) {
                if (diag.min_margin > best_margin) {
                    best_margin = diag.min_margin;
                    best_q = q;
                }
            }
        }
        if (best_margin > -std::numeric_limits<double>::infinity()) {
            return best_q;
        }
        return std::nullopt;
    };

    auto edgeValid = [&](const Eigen::VectorXd& from,
                         const Eigen::VectorXd& to,
                         const int from_layer,
                         const int to_layer,
                         cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
        if (std::abs(to_layer - from_layer) != 1) {
            if (diag != nullptr) {
                diag->reason = "non_adjacent_layer_edge";
                diag->min_margin = -1.0;
            }
            return false;
        }
        const int segment_index = std::min(from_layer, to_layer);
        const double dt =
            arm_segment_times[static_cast<std::size_t>(segment_index)];
        if (!armVelocityFeasible(
                from.segment(kBaseDof, from.size() - kBaseDof),
                to.segment(kBaseDof, to.size() - kBaseDof),
                dt)) {
            if (diag != nullptr) {
                diag->reason = "arm_velocity";
                diag->min_margin = -1.0;
            }
            return false;
        }
        return validateLayerEdgeAlongBase(
            route.states,
            route.segment_check_states,
            from_layer,
            to_layer,
            from,
            to,
            input,
            sample_mani_safe_distance,
            diag);
    };

    auto extendOneStep = [&](const int near_index,
                             const Eigen::VectorXd& q_target,
                             const int target_layer,
                             const LayerTreeState tree_state,
                             int* out_index,
                             cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag)
        -> bool {
        if (out_index == nullptr || near_index < 0) {
            if (diag != nullptr) {
                diag->reason = "missing_nearest";
                diag->min_margin = -1.0;
            }
            return false;
        }
        const LayerTreeNode& near = nodes[static_cast<std::size_t>(near_index)];
        const int next_layer =
            tree_state == LayerTreeState::Forward ? near.layer + 1
                                                  : near.layer - 1;
        if (next_layer < 0 || next_layer >= static_cast<int>(route.size())) {
            if (diag != nullptr) {
                diag->reason = "next_layer_out_of_range";
                diag->min_margin = -1.0;
            }
            return false;
        }

        const int segment_index = std::min(near.layer, next_layer);
        const double dt =
            arm_segment_times[static_cast<std::size_t>(segment_index)];
        Eigen::VectorXd q_new = q_target;
        q_new[0] = route[static_cast<std::size_t>(next_layer)].x;
        q_new[1] = route[static_cast<std::size_t>(next_layer)].y;
        q_new[2] = route[static_cast<std::size_t>(next_layer)].yaw;

        const Eigen::Index arm_dof = q_new.size() - kBaseDof;
        const int first_time_index = std::min(near.layer, target_layer);
        const int last_time_index = std::max(near.layer, target_layer);
        double total_dt = 0.0;
        for (int i = first_time_index; i < last_time_index; ++i) {
            if (i >= 0 && i < static_cast<int>(arm_segment_times.size())) {
                total_dt += arm_segment_times[static_cast<std::size_t>(i)];
            }
        }
        if (total_dt <= 1e-6) {
            total_dt = dt;
        }
        Eigen::VectorXd arm_velocity =
            q_target.segment(kBaseDof, arm_dof) -
            near.arm;
        arm_velocity /= std::max(1e-6, total_dt);
        const double max_delta =
            kLayerRrtExtensionVelocityScale *
            kLayerRrtMaxJointVelRadPerSec * dt;
        const double max_velocity =
            kLayerRrtExtensionVelocityScale *
            kLayerRrtMaxJointVelRadPerSec;
        for (Eigen::Index i = 0; i < arm_dof; ++i) {
            arm_velocity[i] =
                std::clamp(arm_velocity[i], -max_velocity, max_velocity);
        }
        q_new.segment(kBaseDof, arm_dof) =
            near.arm + arm_velocity * dt;

        if (next_layer == 0 ||
            next_layer == static_cast<int>(route.size()) - 1) {
            const Eigen::VectorXd& boundary =
                next_layer == 0 ? q_start : q_goal;
            if ((boundary.segment(kBaseDof, arm_dof) -
                 q_new.segment(kBaseDof, arm_dof))
                    .cwiseAbs()
                    .maxCoeff() <= max_delta) {
                q_new.segment(kBaseDof, arm_dof) =
                    boundary.segment(kBaseDof, arm_dof);
            }
        }

        if (input.q_min.size() == q_new.size() &&
            input.q_max.size() == q_new.size()) {
            for (Eigen::Index i = kBaseDof; i < q_new.size(); ++i) {
                q_new[i] = std::clamp(
                    q_new[i],
                    std::min(input.q_min[i], input.q_max[i]),
                    std::max(input.q_min[i], input.q_max[i]));
            }
        }

        cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
        if (!validateLayerState(input, q_new, sample_mani_safe_distance, &state_diag)) {
            if (diag != nullptr) {
                *diag = state_diag;
            }
            return false;
        }
        const Eigen::VectorXd q_near_full = composeLayerState(route.states, near);
        if (!edgeValid(q_near_full, q_new, near.layer, next_layer, diag)) {
            return false;
        }

        const double g_score =
            near.g_score +
            armTransitionCost(near.arm, q_new.segment(kBaseDof, arm_dof), dt) +
            sampleManiNodeClearancePenalty(state_diag.min_margin);
        *out_index = addNode(q_new, next_layer, near_index, g_score, tree_state);
        (void)target_layer;
        return true;
    };

    JointPath best_connected_path;
    double best_connected_cost = std::numeric_limits<double>::infinity();
    PathClearanceCost best_connected_clearance;
    std::size_t extend_rejects = 0;
    std::size_t pull_rejects = 0;
    std::size_t merge_attempts = 0;
    std::map<std::string, std::size_t> extend_reasons;
    std::map<std::string, std::size_t> pull_reasons;

    auto addRejectReason =
        [](std::map<std::string, std::size_t>* reasons,
           const cp::PathPlanningInput::WholeBodyPoseDiagnostic& diag) {
            if (reasons == nullptr) {
                return;
            }
            std::string reason = diag.reason.empty() ? "unknown" : diag.reason;
            if (!diag.worst_link_name.empty()) {
                reason += ":";
                reason += diag.worst_link_name;
            }
            ++(*reasons)[reason];
        };

    auto summarizeRejectReasons =
        [](const std::map<std::string, std::size_t>& reasons) {
            std::string summary = "none";
            std::size_t count = 0;
            for (const auto& entry : reasons) {
                if (entry.second > count) {
                    summary = entry.first;
                    count = entry.second;
                }
            }
            if (count == 0) {
                return summary;
            }
            return summary + "(" + std::to_string(count) + ")";
        };

    auto treeCount = [&](const LayerTreeState state) {
        return static_cast<int>(std::count_if(
            nodes.begin(), nodes.end(), [&](const LayerTreeNode& node) {
                return node.state == state;
            }));
    };

    auto nearestNodeInLayerFast =
        [&](const int layer,
            const LayerTreeState state,
            const Eigen::VectorXd& q_target) {
            if (layer < 0 || layer >= static_cast<int>(nodes_by_layer.size())) {
                return -1;
            }
            int best = -1;
            double best_cost = std::numeric_limits<double>::infinity();
            const Eigen::Index arm_dof = q_target.size() - kBaseDof;
            if (arm_dof <= 0) {
                return -1;
            }
            const Eigen::VectorXd target_arm =
                q_target.segment(kBaseDof, arm_dof);
            for (const int index :
                 nodes_by_layer[static_cast<std::size_t>(layer)]) {
                if (index < 0 || index >= static_cast<int>(nodes.size())) {
                    continue;
                }
                const LayerTreeNode& node =
                    nodes[static_cast<std::size_t>(index)];
                if (node.state != state || node.arm.size() != arm_dof) {
                    continue;
                }
                const double cost =
                    (node.arm - target_arm).squaredNorm() +
                    0.02 * node.g_score +
                    sampleManiNodeClearancePenalty(node.clearance_margin);
                if (cost < best_cost) {
                    best_cost = cost;
                    best = index;
                }
            }
            return best;
        };

    auto propagateGScore = [&](auto&& self, const int parent_index) -> void {
        if (parent_index < 0 ||
            parent_index >= static_cast<int>(nodes.size())) {
            return;
        }
        const LayerTreeNode parent =
            nodes[static_cast<std::size_t>(parent_index)];
        const int first_child_layer = parent.layer - 1;
        const int last_child_layer = parent.layer + 1;
        for (int layer = first_child_layer; layer <= last_child_layer; ++layer) {
            if (layer < 0 ||
                layer >= static_cast<int>(nodes_by_layer.size())) {
                continue;
            }
            for (const int child_index :
                 nodes_by_layer[static_cast<std::size_t>(layer)]) {
                LayerTreeNode& child =
                    nodes[static_cast<std::size_t>(child_index)];
                if (child.parent != parent_index ||
                    child.state != parent.state ||
                    child.arm.size() != parent.arm.size()) {
                    continue;
                }
                const int segment_index = std::clamp(
                    std::min(child.layer, parent.layer),
                    0,
                    static_cast<int>(arm_segment_times.size()) - 1);
                const double edge_cost =
                    armTransitionCost(
                        parent.arm,
                        child.arm,
                        arm_segment_times[static_cast<std::size_t>(segment_index)]) +
                    sampleManiNodeClearancePenalty(child.clearance_margin);
                child.g_score = parent.g_score + edge_cost;
                self(self, child_index);
            }
        }
    };

    auto linkNode = [&](const int parent_index, const int child_index) {
        if (parent_index < 0 || child_index < 0 ||
            parent_index >= static_cast<int>(nodes.size()) ||
            child_index >= static_cast<int>(nodes.size()) ||
            parent_index == child_index) {
            return false;
        }
        LayerTreeNode& parent = nodes[static_cast<std::size_t>(parent_index)];
        LayerTreeNode& child = nodes[static_cast<std::size_t>(child_index)];
        if (parent.state == LayerTreeState::Unvisited ||
            child.state != parent.state ||
            std::abs(child.layer - parent.layer) != 1 ||
            parent.arm.size() != child.arm.size()) {
            return false;
        }
        const int segment_index = std::clamp(
            std::min(child.layer, parent.layer),
            0,
            static_cast<int>(arm_segment_times.size()) - 1);
        const double edge_cost =
            armTransitionCost(
                parent.arm,
                child.arm,
                arm_segment_times[static_cast<std::size_t>(segment_index)]) +
            sampleManiNodeClearancePenalty(child.clearance_margin);
        child.parent = parent_index;
        child.g_score = parent.g_score + edge_cost;
        propagateGScore(propagateGScore, child_index);
        return true;
    };

    auto nearestForRemaniSample = [&](const int sample_layer,
                                      const LayerTreeState state,
                                      const Eigen::VectorXd& q_target) {
        const bool forward = state == LayerTreeState::Forward;
        int predecessor_layer = forward ? sample_layer - 1 : sample_layer + 1;
        const int frontier = frontierLayer(nodes, state, forward);
        if (forward) {
            predecessor_layer = std::min(predecessor_layer, frontier);
        } else {
            predecessor_layer = std::max(predecessor_layer, frontier);
        }
        predecessor_layer = std::clamp(
            predecessor_layer, 0, static_cast<int>(route.size()) - 1);
        return nearestNodeInLayerFast(predecessor_layer, state, q_target);
    };

    auto adjustTree = [&](const int new_index) {
        if (new_index < 0 || new_index >= static_cast<int>(nodes.size())) {
            return;
        }
        const LayerTreeNode q_new = nodes[static_cast<std::size_t>(new_index)];
        if (q_new.state == LayerTreeState::Unvisited) {
            return;
        }
        const bool forward = q_new.state == LayerTreeState::Forward;
        const int next_layer = forward ? q_new.layer + 1 : q_new.layer - 1;
        if (next_layer < 0 || next_layer >= static_cast<int>(route.size())) {
            return;
        }
        const int segment_index = std::min(q_new.layer, next_layer);
        const double dt =
            arm_segment_times[static_cast<std::size_t>(segment_index)];
        const Eigen::VectorXd q_new_full = composeLayerState(route.states, q_new);
        for (const int candidate_index :
             nodes_by_layer[static_cast<std::size_t>(next_layer)]) {
            if (candidate_index == new_index || candidate_index < 0 ||
                candidate_index >= static_cast<int>(nodes.size())) {
                continue;
            }
            LayerTreeNode& candidate =
                nodes[static_cast<std::size_t>(candidate_index)];
            if (candidate.state != q_new.state ||
                candidate.arm.size() != q_new.arm.size()) {
                continue;
            }
            const double candidate_cost =
                q_new.g_score +
                armTransitionCost(q_new.arm, candidate.arm, dt) +
                sampleManiNodeClearancePenalty(candidate.clearance_margin);
            if (candidate_cost >= candidate.g_score) {
                continue;
            }
            const Eigen::VectorXd q_candidate_full =
                composeLayerState(route.states, candidate);
            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (!armVelocityFeasible(q_new.arm, candidate.arm, dt) ||
                !edgeValid(
                    q_new_full,
                    q_candidate_full,
                    q_new.layer,
                    candidate.layer,
                    &diag)) {
                continue;
            }
            linkNode(new_index, candidate_index);
        }
    };

    auto tryMergeSameLayer = [&](const int a_index,
                                 const int b_index) -> JointPath {
        if (a_index < 0 || b_index < 0) {
            return JointPath{};
        }
        const LayerTreeNode& a = nodes[static_cast<std::size_t>(a_index)];
        const LayerTreeNode& b = nodes[static_cast<std::size_t>(b_index)];
        if (a.layer != b.layer || a.arm.size() != b.arm.size()) {
            return JointPath{};
        }
        ++merge_attempts;
        const double meet_error =
            (a.arm - b.arm).cwiseAbs().maxCoeff();
        if (meet_error > kLayerRrtMeetToleranceRad) {
            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            const int bridge_segment = std::clamp(
                std::min(
                    a.layer,
                    static_cast<int>(arm_segment_times.size()) - 1),
                0,
                static_cast<int>(arm_segment_times.size()) - 1);
            const int previous_segment = std::clamp(
                bridge_segment - 1,
                0,
                static_cast<int>(arm_segment_times.size()) - 1);
            const double bridge_dt = std::max(
                arm_segment_times[static_cast<std::size_t>(bridge_segment)],
                arm_segment_times[static_cast<std::size_t>(previous_segment)]);
            const Eigen::VectorXd q_a = composeLayerState(route.states, a);
            const Eigen::VectorXd q_b = composeLayerState(route.states, b);
            if (!armVelocityFeasible(a.arm, b.arm, bridge_dt) ||
                !validateLayerSegment(input, q_a, q_b, sample_mani_safe_distance, &diag)) {
                return JointPath{};
            }
        }
        const int forward_index =
            a.state == LayerTreeState::Forward ? a_index : b_index;
        const int backward_index =
            a.state == LayerTreeState::Backward ? a_index : b_index;
        return reconstructLayerTreePath(
            nodes, route.states, forward_index, backward_index);
    };

    auto tryBridgeTrees = [&](const int a_index,
                              const int b_index) -> JointPath {
        if (a_index < 0 || b_index < 0) {
            return JointPath{};
        }
        const LayerTreeNode& a = nodes[static_cast<std::size_t>(a_index)];
        const LayerTreeNode& b = nodes[static_cast<std::size_t>(b_index)];
        if (a.state == b.state || a.arm.size() != b.arm.size()) {
            return JointPath{};
        }
        if (a.layer == b.layer) {
            return tryMergeSameLayer(a_index, b_index);
        }
        if (std::abs(a.layer - b.layer) != 1) {
            return JointPath{};
        }
        ++merge_attempts;
        const int segment_index = std::min(a.layer, b.layer);
        const double dt =
            arm_segment_times[static_cast<std::size_t>(segment_index)];
        const Eigen::VectorXd q_a = composeLayerState(route.states, a);
        const Eigen::VectorXd q_b = composeLayerState(route.states, b);
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!armVelocityFeasible(a.arm, b.arm, dt) ||
            !edgeValid(q_a, q_b, a.layer, b.layer, &diag)) {
            return JointPath{};
        }

        const int forward_index =
            a.state == LayerTreeState::Forward ? a_index : b_index;
        const int backward_index =
            a.state == LayerTreeState::Backward ? a_index : b_index;
        std::vector<Eigen::VectorXd> prefix;
        for (int index = forward_index; index >= 0;
             index = nodes[static_cast<std::size_t>(index)].parent) {
            const LayerTreeNode& node = nodes[static_cast<std::size_t>(index)];
            prefix.push_back(composeLayerState(route.states, node));
        }
        std::reverse(prefix.begin(), prefix.end());

        std::vector<Eigen::VectorXd> suffix;
        for (int index = backward_index; index >= 0;
             index = nodes[static_cast<std::size_t>(index)].parent) {
            const LayerTreeNode& node = nodes[static_cast<std::size_t>(index)];
            suffix.push_back(composeLayerState(route.states, node));
        }

        std::vector<Eigen::VectorXd> path = std::move(prefix);
        path.insert(path.end(), suffix.begin(), suffix.end());
        return path;
    };

    auto acceptMergeCandidate = [&](const int a_index, const int b_index) {
        JointPath path = tryBridgeTrees(a_index, b_index);
        const bool complete_endpoint_path =
            path.size() >= 2 &&
            path.front().size() == q_start.size() &&
            path.back().size() == q_goal.size() &&
            path.front().isApprox(q_start, 1e-6) &&
            path.back().isApprox(q_goal, 1e-6);
        if (!complete_endpoint_path ||
            !validateLayerPath(path, input, sample_mani_safe_distance)) {
            return false;
        }
        PathClearanceCost clearance;
        const double path_cost =
            clearanceAwarePathSelectionCost(
                path, input, sample_mani_safe_distance, &clearance);
        if (path_cost < best_connected_cost) {
            best_connected_cost = path_cost;
            best_connected_clearance = clearance;
            best_connected_path = std::move(path);
        }
        return true;
    };

    auto pullOppositeTree = [&](const int target_index,
                                const LayerTreeState opposite) {
        bool connected = false;
        int near_index = nearestForRemaniSample(
            nodes[static_cast<std::size_t>(target_index)].layer,
            opposite,
            composeLayerState(
                route.states,
                nodes[static_cast<std::size_t>(target_index)]));
        if (near_index < 0) {
            return false;
        }
        while (near_index >= 0 &&
               nodes[static_cast<std::size_t>(near_index)].layer !=
                   nodes[static_cast<std::size_t>(target_index)].layer) {
            int pulled_index = -1;
            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (!extendOneStep(
                    near_index,
                    composeLayerState(
                        route.states,
                        nodes[static_cast<std::size_t>(target_index)]),
                    nodes[static_cast<std::size_t>(target_index)].layer,
                    opposite,
                    &pulled_index,
                    &diag)) {
                ++pull_rejects;
                addRejectReason(&pull_reasons, diag);
                break;
            }
            connected |= acceptMergeCandidate(target_index, pulled_index);
            if (nodes[static_cast<std::size_t>(pulled_index)].state == opposite) {
                adjustTree(pulled_index);
            }
            near_index = pulled_index;
        }
        if (near_index >= 0 &&
            nodes[static_cast<std::size_t>(near_index)].layer ==
                nodes[static_cast<std::size_t>(target_index)].layer) {
            connected |= acceptMergeCandidate(target_index, near_index);
        }
        return connected;
    };

    auto pullEndpoint = [&](const int seed_index,
                            const LayerTreeState state) {
        int current_index = seed_index;
        bool advanced = false;
        const Eigen::VectorXd& endpoint =
            state == LayerTreeState::Forward ? q_goal : q_start;
        const int endpoint_layer =
            state == LayerTreeState::Forward
                ? static_cast<int>(route.size()) - 1
                : 0;
        while (current_index >= 0 &&
               current_index < static_cast<int>(nodes.size()) &&
               nodes[static_cast<std::size_t>(current_index)].layer !=
                   endpoint_layer) {
            int next_index = -1;
            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (!extendOneStep(
                    current_index,
                    endpoint,
                    endpoint_layer,
                    state,
                    &next_index,
                    &diag)) {
                ++extend_rejects;
                addRejectReason(&extend_reasons, diag);
                break;
            }
            advanced = true;
            current_index = next_index;
            adjustTree(current_index);
            const LayerTreeState opposite =
                state == LayerTreeState::Forward ? LayerTreeState::Backward
                                                 : LayerTreeState::Forward;
            const LayerTreeNode& node =
                nodes[static_cast<std::size_t>(current_index)];
            const int same_layer_opposite =
                nearestNodeInLayerFast(
                    node.layer,
                    opposite,
                    composeLayerState(route.states, node));
            if (same_layer_opposite >= 0) {
                acceptMergeCandidate(current_index, same_layer_opposite);
            }
        }
        return advanced;
    };

    int completed_iterations = 0;
    for (int iteration = 0; iteration < kLayerRrtMaxIterations; ++iteration) {
        completed_iterations = iteration + 1;
        const double elapsed_sec = elapsedSearchSec();
        if (elapsed_sec > search_time_budget_sec) {
            break;
        }
        if (iteration > 0 && iteration % 500 == 0) {
            std::printf(
                "[base_guided_planner] SampleMani progress iter=%d elapsed=%.3f/%.3f nodes=%zu best=%s score=%.3f min_margin=%.4f tree_front=%d anti_front=%d extend_rejects=%zu top_extend=%s pull_rejects=%zu top_pull=%s merges=%zu\n",
                iteration,
                elapsed_sec,
                search_time_budget_sec,
                nodes.size(),
                best_connected_path.empty() ? "false" : "true",
                best_connected_cost,
                best_connected_path.empty()
                    ? 0.0
                    : best_connected_clearance.min_margin,
                frontierLayer(nodes, LayerTreeState::Forward, true),
                frontierLayer(nodes, LayerTreeState::Backward, false),
                extend_rejects,
                summarizeRejectReasons(extend_reasons).c_str(),
                pull_rejects,
                summarizeRejectReasons(pull_reasons).c_str(),
                merge_attempts);
            std::fflush(stdout);
        }
        const int forward_count = treeCount(LayerTreeState::Forward);
        const int backward_count = treeCount(LayerTreeState::Backward);
        const bool expand_forward = forward_count <= backward_count;
        const LayerTreeState active =
            expand_forward ? LayerTreeState::Forward : LayerTreeState::Backward;
        const LayerTreeState opposite =
            expand_forward ? LayerTreeState::Backward : LayerTreeState::Forward;
        const int active_frontier =
            frontierLayer(nodes, active, expand_forward);
        const int endpoint_layer =
            expand_forward ? static_cast<int>(route.size()) - 1 : 0;
        const bool endpoint_band_reached =
            std::abs(endpoint_layer - active_frontier) <=
            kSampleManiEndpointPullLayerBand;

        int sample_layer = -1;
        Eigen::VectorXd q_rand;
        const double sample_mode = unit_dist(rng);
        if (sample_mode < kLayerRrtOppositeTreeBias) {
            std::vector<int> opposite_indices;
            opposite_indices.reserve(nodes.size());
            for (std::size_t i = 0; i < nodes.size(); ++i) {
                if (nodes[i].state == opposite) {
                    opposite_indices.push_back(static_cast<int>(i));
                }
            }
            if (!opposite_indices.empty()) {
                std::uniform_int_distribution<std::size_t> opposite_dist(
                    0, opposite_indices.size() - 1u);
                const LayerTreeNode& target_node =
                    nodes[static_cast<std::size_t>(
                        opposite_indices[opposite_dist(rng)])];
                if (std::abs(target_node.layer - active_frontier) <=
                    kSampleManiEndpointPullLayerBand + 1) {
                    sample_layer = target_node.layer;
                    q_rand = composeLayerState(route.states, target_node);
                }
            }
        }
        if (sample_layer < 0 &&
            sample_mode <
                kLayerRrtOppositeTreeBias + kLayerRrtEndpointBias &&
            endpoint_band_reached) {
            sample_layer = expand_forward
                               ? static_cast<int>(route.size()) - 1
                               : 0;
            q_rand = expand_forward ? q_goal : q_start;
        }
        if (sample_layer < 0 && route.size() > 2) {
            sample_layer = layer_dist(rng);
            std::optional<Eigen::VectorXd> random_q =
                makeRandomLayerCandidate(sample_layer);
            if (!random_q.has_value()) {
                ++extend_rejects;
                cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
                diag.reason = "sample_mani_random_sample";
                diag.min_margin = -1.0;
                addRejectReason(&extend_reasons, diag);
                continue;
            }
            q_rand = std::move(*random_q);
        }
        if (sample_layer < 0) {
            sample_layer = expand_forward
                               ? static_cast<int>(route.size()) - 1
                               : 0;
            q_rand = expand_forward ? q_goal : q_start;
        }

        int near_index =
            nearestForRemaniSample(sample_layer, active, q_rand);
        if (near_index < 0) {
            continue;
        }

        int new_index = -1;
        cp::PathPlanningInput::WholeBodyPoseDiagnostic extend_diag;
        if (!extendOneStep(
                near_index,
                q_rand,
                sample_layer,
                active,
                &new_index,
                &extend_diag)) {
            ++extend_rejects;
            addRejectReason(&extend_reasons, extend_diag);
            continue;
        }

        const LayerTreeNode q_new = nodes[static_cast<std::size_t>(new_index)];
        if (nodes[static_cast<std::size_t>(new_index)].state == active) {
            adjustTree(new_index);
        }
        const int opposite_index =
            nearestNodeInLayerFast(
                q_new.layer, opposite, composeLayerState(route.states, q_new));
        if (opposite_index >= 0) {
            acceptMergeCandidate(new_index, opposite_index);
        }
        const int bridge_layer =
            active == LayerTreeState::Forward ? q_new.layer + 1 : q_new.layer - 1;
        if (bridge_layer >= 0 && bridge_layer < static_cast<int>(route.size())) {
            const int bridge_index =
                nearestNodeInLayerFast(
                    bridge_layer,
                    opposite,
                    composeLayerState(route.states, q_new));
            if (bridge_index >= 0) {
                acceptMergeCandidate(new_index, bridge_index);
            }
        }
        pullOppositeTree(new_index, opposite);
        if (endpoint_band_reached &&
            sample_mode <
                kLayerRrtOppositeTreeBias + kLayerRrtEndpointBias) {
            pullEndpoint(new_index, active);
        }

        if (!best_connected_path.empty() &&
            best_connected_clearance.min_margin >=
                kSampleManiEarlyAcceptClearanceM &&
            iteration > std::max(200, kLayerRrtMaxIterations / 5)) {
            std::printf(
                "[base_guided_planner] SampleMani connected iterations=%d elapsed=%.3f nodes=%zu path_samples=%zu score=%.3f min_margin=%.4f worst=%s tree_front=%d anti_front=%d merges=%zu\n",
                iteration + 1,
                elapsedSearchSec(),
                nodes.size(),
                best_connected_path.size(),
                best_connected_cost,
                best_connected_clearance.min_margin,
                best_connected_clearance.worst_link.empty()
                    ? "none"
                    : best_connected_clearance.worst_link.c_str(),
                frontierLayer(nodes, LayerTreeState::Forward, true),
                frontierLayer(nodes, LayerTreeState::Backward, false),
                merge_attempts);
            std::fflush(stdout);
            return best_connected_path;
        }
    }

    if (!best_connected_path.empty()) {
        if (best_connected_clearance.min_margin <
            kSampleManiMinimumAcceptClearanceM) {
            std::printf(
                "[base_guided_planner] SampleMani budget best rejected: margin=%.4f < required=%.4f score=%.3f worst=%s; trying gap-RRT/other routes instead\n",
                best_connected_clearance.min_margin,
                kSampleManiMinimumAcceptClearanceM,
                best_connected_cost,
                best_connected_clearance.worst_link.empty()
                    ? "none"
                    : best_connected_clearance.worst_link.c_str());
            std::fflush(stdout);
        } else {
        std::printf(
            "[base_guided_planner] SampleMani connected at budget iterations=%d elapsed=%.3f/%.3f nodes=%zu path_samples=%zu score=%.3f hard_margin=%.4f hard_safe=%.4f worst=%s tree_front=%d anti_front=%d merges=%zu\n",
            completed_iterations,
            elapsedSearchSec(),
            search_time_budget_sec,
            nodes.size(),
            best_connected_path.size(),
            best_connected_cost,
            best_connected_clearance.min_margin,
            sample_mani_safe_distance,
            best_connected_clearance.worst_link.empty()
                ? "none"
                : best_connected_clearance.worst_link.c_str(),
            frontierLayer(nodes, LayerTreeState::Forward, true),
            frontierLayer(nodes, LayerTreeState::Backward, false),
            merge_attempts);
        std::fflush(stdout);
        return best_connected_path;
        }
    }

    std::printf(
        "[base_guided_planner] SampleMani failed nodes=%zu iterations=%d elapsed=%.3f/%.3f tree_count=%d anti_tree_count=%d tree_front=%d anti_front=%d extend_rejects=%zu top_extend=%s pull_rejects=%zu top_pull=%s merges=%zu; starting gap-RRT connector\n",
        nodes.size(),
        completed_iterations,
        elapsedSearchSec(),
        search_time_budget_sec,
        treeCount(LayerTreeState::Forward),
        treeCount(LayerTreeState::Backward),
        frontierLayer(nodes, LayerTreeState::Forward, true),
        frontierLayer(nodes, LayerTreeState::Backward, false),
        extend_rejects,
        summarizeRejectReasons(extend_reasons).c_str(),
        pull_rejects,
        summarizeRejectReasons(pull_reasons).c_str(),
        merge_attempts);
    std::fflush(stdout);
    LayerGapRrtConnector::Config gap_cfg;
    gap_cfg.max_iterations = 1600;
    gap_cfg.goal_bias = 0.40;
    gap_cfg.max_joint_velocity_rad_per_sec = kLayerRrtMaxJointVelRadPerSec;
    gap_cfg.time_scale = kLayerRrtTimeScale;
    gap_cfg.meet_tolerance_rad = kLayerRrtMeetToleranceRad;
    gap_cfg.require_complete_endpoint_path = true;
    LayerGapRrtConnector gap_connector(gap_cfg);
    LayerGapRrtConnector::Input gap_input;
    gap_input.planning_input = &input;
    gap_input.arm_start = q_start.segment(kBaseDof, q_start.size() - kBaseDof);
    gap_input.arm_goal = q_goal.segment(kBaseDof, q_goal.size() - kBaseDof);
    gap_input.safe_distance = sample_mani_safe_distance;
    gap_input.segment_times = raw_segment_times;
    gap_input.segment_check_states.reserve(route.segment_check_states.size());
    for (const auto& segment_checks : route.segment_check_states) {
        std::vector<LayerGapRrtConnector::BaseState> converted;
        converted.reserve(segment_checks.size());
        for (const BaseState& state : segment_checks) {
            converted.push_back(
                LayerGapRrtConnector::BaseState{state.x, state.y, state.yaw});
        }
        gap_input.segment_check_states.push_back(std::move(converted));
    }
    gap_input.route.reserve(route.size());
    for (const BaseState& state : route.states) {
        gap_input.route.push_back(
            LayerGapRrtConnector::BaseState{state.x, state.y, state.yaw});
    }
    gap_input.start_seeds.reserve(nodes.size());
    gap_input.goal_seeds.reserve(nodes.size());
    for (const LayerTreeNode& node : nodes) {
        if (node.state == LayerTreeState::Forward) {
            gap_input.start_seeds.push_back(
                {node.arm, node.layer});
        } else if (node.state == LayerTreeState::Backward) {
            gap_input.goal_seeds.push_back(
                {node.arm, node.layer});
        }
    }
    const LayerGapRrtConnector::Result gap_result =
        gap_connector.connect(gap_input);
    if (gap_result.success && gap_result.path.size() >= 2 &&
        gap_result.start_layer == 0 &&
        gap_result.end_layer == static_cast<int>(route.size()) - 1) {
        JointPath gap_full_path = composeFullPathFromArmPath(
            route.states, gap_result.path, gap_result.path_layers);
        PathClearanceCost gap_clearance;
        (void)clearanceAwarePathSelectionCost(
            gap_full_path, input, sample_mani_safe_distance, &gap_clearance);
        if (gap_clearance.min_margin < kSampleManiMinimumAcceptClearanceM) {
            std::printf(
                "[base_guided_planner] gap-RRT connector rejected low-clearance path margin=%.4f < required=%.4f worst=%s\n",
                gap_clearance.min_margin,
                kSampleManiMinimumAcceptClearanceM,
                gap_clearance.worst_link.empty() ? "none" : gap_clearance.worst_link.c_str());
            return {};
        }
        std::printf(
            "[base_guided_planner] gap-RRT connector succeeded iterations=%d nodes=%zu path_samples=%zu layers=%d->%d min_margin=%.4f\n",
            gap_result.iterations,
            gap_result.node_count,
            gap_result.path.size(),
            gap_result.start_layer,
            gap_result.end_layer,
            gap_clearance.min_margin);
        return gap_full_path;
    }
    std::printf(
        "[base_guided_planner] gap-RRT connector failed iterations=%d nodes=%zu layers=%d->%d\n",
        gap_result.iterations,
        gap_result.node_count,
        gap_result.start_layer,
        gap_result.end_layer);
    return {};
}

BaseGuidedWholeBodyPlanner::JointPath
BaseGuidedWholeBodyPlanner::repairPathWithLayerRrt(
    const BaseRoute& route,
    const JointPath& seed_path,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const cp::PathPlanningInput& input,
    const double safe_distance) const {
    if (route.size() < 2 || q_start.size() != q_goal.size() ||
        seed_path.size() != route.size()) {
        return {};
    }

    const std::vector<double> segment_times = estimateRouteSegmentTimes(
        route, common_cfg_.default_segment_speed);
    if (segment_times.size() + 1u != route.size()) {
        return {};
    }

    int prefix_end = -1;
    for (std::size_t i = 0; i < seed_path.size(); ++i) {
        cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
        if (!validateLayerState(input, seed_path[i], safe_distance, &state_diag)) {
            break;
        }
        if (i > 0) {
            cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
            if (!validateLayerSegment(
                    input,
                    seed_path[i - 1u],
                    seed_path[i],
                    safe_distance,
                    &segment_diag)) {
                break;
            }
        }
        prefix_end = static_cast<int>(i);
    }

    int suffix_start = static_cast<int>(seed_path.size());
    for (int i = static_cast<int>(seed_path.size()) - 1; i >= 0; --i) {
        cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
        if (!validateLayerState(
                input,
                seed_path[static_cast<std::size_t>(i)],
                safe_distance,
                &state_diag)) {
            break;
        }
        if (i + 1 < static_cast<int>(seed_path.size())) {
            cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
            if (!validateLayerSegment(
                    input,
                    seed_path[static_cast<std::size_t>(i)],
                    seed_path[static_cast<std::size_t>(i + 1)],
                    safe_distance,
                    &segment_diag)) {
                break;
            }
        }
        suffix_start = i;
    }

    if (prefix_end < 0 ||
        suffix_start >= static_cast<int>(seed_path.size()) ||
        prefix_end >= suffix_start) {
        std::printf(
            "[base_guided_planner] same-route repair skipped: prefix_end=%d suffix_start=%d\n",
            prefix_end,
            suffix_start);
        return {};
    }

    LayerGapRrtConnector::Config gap_cfg;
    gap_cfg.max_iterations = std::max(2400, 2 * kLayerRrtMaxIterations);
    gap_cfg.goal_bias = 0.40;
    gap_cfg.max_joint_velocity_rad_per_sec = kLayerRrtMaxJointVelRadPerSec;
    gap_cfg.time_scale = kLayerRrtTimeScale;
    gap_cfg.meet_tolerance_rad = kLayerRrtMeetToleranceRad;
    gap_cfg.require_complete_endpoint_path = false;
    LayerGapRrtConnector gap_connector(gap_cfg);

    LayerGapRrtConnector::Input gap_input;
    gap_input.planning_input = &input;
    gap_input.arm_start = q_start.segment(kBaseDof, q_start.size() - kBaseDof);
    gap_input.arm_goal = q_goal.segment(kBaseDof, q_goal.size() - kBaseDof);
    gap_input.safe_distance = safe_distance;
    gap_input.segment_times = segment_times;
    gap_input.segment_check_states.reserve(route.segment_check_states.size());
    for (const auto& segment_checks : route.segment_check_states) {
        std::vector<LayerGapRrtConnector::BaseState> converted;
        converted.reserve(segment_checks.size());
        for (const BaseState& state : segment_checks) {
            converted.push_back(
                LayerGapRrtConnector::BaseState{state.x, state.y, state.yaw});
        }
        gap_input.segment_check_states.push_back(std::move(converted));
    }
    gap_input.route.reserve(route.size());
    for (const BaseState& state : route.states) {
        gap_input.route.push_back(
            LayerGapRrtConnector::BaseState{state.x, state.y, state.yaw});
    }

    for (int i = 0; i <= prefix_end; ++i) {
        const Eigen::VectorXd& q = seed_path[static_cast<std::size_t>(i)];
        gap_input.start_seeds.push_back(
            {q.segment(kBaseDof, q.size() - kBaseDof), i});
    }
    for (int i = suffix_start; i < static_cast<int>(seed_path.size()); ++i) {
        const Eigen::VectorXd& q = seed_path[static_cast<std::size_t>(i)];
        gap_input.goal_seeds.push_back(
            {q.segment(kBaseDof, q.size() - kBaseDof), i});
    }

    const LayerGapRrtConnector::Result gap_result =
        gap_connector.connect(gap_input);
    if (!gap_result.success || gap_result.path.size() < 2) {
        std::printf(
            "[base_guided_planner] same-route gap repair failed prefix_end=%d suffix_start=%d iterations=%d nodes=%zu\n",
            prefix_end,
            suffix_start,
            gap_result.iterations,
            gap_result.node_count);
        return {};
    }

    const int gap_start_layer = gap_result.start_layer;
    const int gap_end_layer = gap_result.end_layer;
    if (gap_start_layer < 0 || gap_end_layer < gap_start_layer ||
        gap_start_layer > prefix_end || gap_end_layer < suffix_start) {
        std::printf(
            "[base_guided_planner] same-route gap repair produced inconsistent layers start=%d end=%d prefix_end=%d suffix_start=%d\n",
            gap_start_layer,
            gap_end_layer,
            prefix_end,
            suffix_start);
        return {};
    }

    JointPath repaired;
    repaired.reserve(seed_path.size() + gap_result.path.size());
    for (int i = 0; i < gap_start_layer; ++i) {
        repaired.push_back(seed_path[static_cast<std::size_t>(i)]);
    }
    JointPath gap_full_path = composeFullPathFromArmPath(
        route.states, gap_result.path, gap_result.path_layers);
    repaired.insert(repaired.end(), gap_full_path.begin(), gap_full_path.end());
    for (int i = gap_end_layer + 1;
         i < static_cast<int>(seed_path.size());
         ++i) {
        repaired.push_back(seed_path[static_cast<std::size_t>(i)]);
    }

    if (validateLayerPath(repaired, input, safe_distance)) {
        PathClearanceCost repaired_clearance;
        (void)clearanceAwarePathSelectionCost(
            repaired, input, safe_distance, &repaired_clearance);
        if (repaired_clearance.min_margin < kSampleManiMinimumAcceptClearanceM) {
            std::printf(
                "[base_guided_planner] same-route gap repair rejected low-clearance path margin=%.4f < required=%.4f worst=%s\n",
                repaired_clearance.min_margin,
                kSampleManiMinimumAcceptClearanceM,
                repaired_clearance.worst_link.empty()
                    ? "none"
                    : repaired_clearance.worst_link.c_str());
            return {};
        }
        std::printf(
            "[base_guided_planner] REMANI-style same-route gap repair succeeded prefix_end=%d suffix_start=%d gap=%d->%d path_samples=%zu min_margin=%.4f\n",
            prefix_end,
            suffix_start,
            gap_start_layer,
            gap_end_layer,
            repaired.size(),
            repaired_clearance.min_margin);
        return repaired;
    }

    std::printf(
        "[base_guided_planner] same-route gap repair path failed final validation path_samples=%zu\n",
        repaired.size());
    return {};
}

BaseGuidedWholeBodyPlanner::Layer
BaseGuidedWholeBodyPlanner::buildArmCandidatesForLayer(
    const BaseState& base,
    const std::size_t layer_index,
    const double t,
    const bool is_start,
    const bool is_goal,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const cp::PathPlanningInput& input,
    const double safe_distance,
    LayerRejectStats* reject_stats) const {
    Layer candidates;
    if (q_start.size() != q_goal.size()) {
        return candidates;
    }

    auto tryAdd = [&](Eigen::VectorXd q) {
        if (q.size() != q_start.size() || !q.allFinite()) {
            return;
        }
        if (reject_stats != nullptr) {
            ++reject_stats->state_attempts;
        }
        q[0] = base.x;
        q[1] = base.y;
        q[2] = base.yaw;
        if (input.q_min.size() == q.size() && input.q_max.size() == q.size()) {
            for (Eigen::Index i = 0; i < q.size(); ++i) {
                q[i] = std::clamp(
                    q[i],
                    std::min(input.q_min[i], input.q_max[i]),
                    std::max(input.q_min[i], input.q_max[i]));
            }
        }
        for (const Eigen::VectorXd& existing : candidates) {
            if ((existing - q).norm() < 1e-4) {
                if (reject_stats != nullptr) {
                    ++reject_stats->duplicate_rejects;
                }
                return;
            }
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateLayerState(input, q, safe_distance, &diag)) {
            if (reject_stats != nullptr) {
                reject_stats->addStateReject(diag);
            }
            return;
        }
        if (reject_stats != nullptr) {
            ++reject_stats->state_accepts;
        }
        candidates.push_back(std::move(q));
    };

    if (is_start) {
        tryAdd(q_start);
        return candidates;
    }
    if (is_goal) {
        tryAdd(q_goal);
        return candidates;
    }

    const std::vector<double> schedule_gammas = {1.0, 0.85, 1.15};
    for (const double gamma : schedule_gammas) {
        const double alpha = std::clamp(std::pow(t, gamma), 0.0, 1.0);
        tryAdd(interpolateState(q_start, q_goal, base, alpha));
    }

    const Eigen::VectorXd q_linear = interpolateState(q_start, q_goal, base, t);
    const Eigen::Index arm_dof = q_start.size() - kBaseDof;
    const bool has_limits =
        input.q_min.size() == q_start.size() && input.q_max.size() == q_start.size();
    const Eigen::VectorXd arm_delta =
        q_goal.segment(kBaseDof, arm_dof) - q_start.segment(kBaseDof, arm_dof);
    const double bump = std::sin(M_PI * t);
    const std::vector<double> perturb_scales = {0.025, -0.025};
    for (const double scale : perturb_scales) {
        Eigen::VectorXd q = q_linear;
        for (Eigen::Index i = 0; i < arm_dof; ++i) {
            const double sign = (i % 2 == 0) ? 1.0 : -1.0;
            q[kBaseDof + i] += sign * scale * bump;
        }
        tryAdd(std::move(q));
    }

    for (Eigen::Index i = 0; i < arm_dof &&
                            candidates.size() < kMaxArmCandidatesPerLayer;
         ++i) {
        if (std::abs(arm_delta[i]) < 0.05) {
            continue;
        }
        for (const double offset : {0.04, -0.04}) {
            Eigen::VectorXd q = q_linear;
            q[kBaseDof + i] += offset * bump;
            tryAdd(std::move(q));
            if (candidates.size() >= kMaxArmCandidatesPerLayer) {
                break;
            }
        }
    }

    for (int sample = 0;
         sample < kLayerExplorationSamples &&
         candidates.size() < kMaxArmCandidatesPerLayer;
         ++sample) {
        Eigen::VectorXd q = q_linear;
        const double global_scale =
            0.04 + 0.18 * deterministicUnitNoise(
                       static_cast<double>((layer_index + 1u) * 811u + sample * 37));
        const double alpha_bias = deterministicUnitNoise(
            static_cast<double>((layer_index + 1u) * 331u + sample * 53));
        const double alpha =
            std::clamp(t + (alpha_bias - 0.5) * 0.12 * bump, 0.0, 1.0);
        q = interpolateState(q_start, q_goal, base, alpha);

        for (Eigen::Index joint = 0; joint < arm_dof; ++joint) {
            const double noise =
                deterministicSignedNoise(layer_index, sample, joint);
            q[kBaseDof + joint] += global_scale * bump * noise;
        }
        tryAdd(std::move(q));
    }

    if (has_limits) {
        for (int sample = 0;
             sample < kLayerExplorationSamples &&
             candidates.size() < kMaxArmCandidatesPerLayer;
             ++sample) {
            Eigen::VectorXd q = q_linear;
            const double blend_to_random =
                0.35 + 0.40 * deterministicUnitNoise(
                           static_cast<double>((layer_index + 1u) * 7919u) +
                           static_cast<double>((sample + 1) * 3571));
            for (Eigen::Index joint = 0; joint < arm_dof; ++joint) {
                const double lo = std::min(
                    input.q_min[kBaseDof + joint],
                    input.q_max[kBaseDof + joint]);
                const double hi = std::max(
                    input.q_min[kBaseDof + joint],
                    input.q_max[kBaseDof + joint]);
                if (std::isfinite(lo) && std::isfinite(hi) && hi > lo) {
                    const double u = deterministicUnitNoise(
                        static_cast<double>((layer_index + 1u) * 10007u) +
                        static_cast<double>((sample + 1) * 2593) +
                        static_cast<double>((joint + 1) * 733));
                    const double random_joint = lo + u * (hi - lo);
                    q[kBaseDof + joint] =
                        (1.0 - blend_to_random) * q_linear[kBaseDof + joint] +
                        blend_to_random * random_joint;
                }
            }
            tryAdd(std::move(q));
        }
    }

    if (candidates.size() > kMaxArmCandidatesPerLayer) {
        candidates.resize(kMaxArmCandidatesPerLayer);
    }
    return candidates;
}

BaseGuidedWholeBodyPlanner::Layer
BaseGuidedWholeBodyPlanner::buildArmCandidatesFromSeedsForLayer(
    const BaseState& base,
    const std::size_t layer_index,
    const double t,
    const bool is_goal,
    const Layer& seed_layer,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const cp::PathPlanningInput& input,
    const double safe_distance,
    const double segment_dt,
    LayerRejectStats* reject_stats) const {
    Layer candidates;
    if (q_start.size() != q_goal.size() || q_start.size() <= kBaseDof) {
        return candidates;
    }

    const Eigen::Index arm_dof = q_start.size() - kBaseDof;
    const bool has_limits =
        input.q_min.size() == q_start.size() && input.q_max.size() == q_start.size();
    const double max_delta =
        std::max(
            0.02,
            kLayerRrtMaxJointVelRadPerSec * std::max(1e-3, segment_dt));
    const double bump = std::max(0.15, std::sin(M_PI * std::clamp(t, 0.0, 1.0)));

    auto clampAndSetBase = [&](Eigen::VectorXd& q) {
        q[0] = base.x;
        q[1] = base.y;
        q[2] = base.yaw;
        if (has_limits) {
            for (Eigen::Index i = kBaseDof; i < q.size(); ++i) {
                q[i] = std::clamp(
                    q[i],
                    std::min(input.q_min[i], input.q_max[i]),
                    std::max(input.q_min[i], input.q_max[i]));
            }
        }
    };

    auto tryAdd = [&](Eigen::VectorXd q) {
        if (q.size() != q_start.size() || !q.allFinite() ||
            candidates.size() >= kMaxArmCandidatesPerLayer) {
            return;
        }
        if (reject_stats != nullptr) {
            ++reject_stats->state_attempts;
        }
        clampAndSetBase(q);
        for (const Eigen::VectorXd& existing : candidates) {
            if ((existing.segment(kBaseDof, arm_dof) -
                 q.segment(kBaseDof, arm_dof))
                    .cwiseAbs()
                    .maxCoeff() < 1e-4) {
                if (reject_stats != nullptr) {
                    ++reject_stats->duplicate_rejects;
                }
                return;
            }
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateLayerState(input, q, safe_distance, &diag)) {
            if (reject_stats != nullptr) {
                reject_stats->addStateReject(diag);
            }
            return;
        }
        if (reject_stats != nullptr) {
            ++reject_stats->state_accepts;
        }
        candidates.push_back(std::move(q));
    };

    if (is_goal) {
        tryAdd(q_goal);
        return candidates;
    }

    const Eigen::VectorXd q_interpolated =
        interpolateState(q_start, q_goal, base, t);

    auto tryAddReachableFromSeed = [&](const Eigen::VectorXd& seed,
                                       Eigen::VectorXd q) {
        if (seed.size() != q_start.size() || q.size() != q_start.size() ||
            candidates.size() >= kMaxArmCandidatesPerLayer) {
            return;
        }
        Eigen::VectorXd arm_delta =
            q.segment(kBaseDof, arm_dof) -
            seed.segment(kBaseDof, arm_dof);
        for (Eigen::Index joint = 0; joint < arm_dof; ++joint) {
            arm_delta[joint] =
                std::clamp(arm_delta[joint], -max_delta, max_delta);
        }
        q.segment(kBaseDof, arm_dof) =
            seed.segment(kBaseDof, arm_dof) + arm_delta;
        tryAdd(std::move(q));
    };

    for (const Eigen::VectorXd& seed : seed_layer) {
        if (seed.size() != q_start.size() ||
            candidates.size() >= kMaxArmCandidatesPerLayer) {
            continue;
        }

        tryAddReachableFromSeed(seed, seed);

        std::vector<Eigen::VectorXd> local_targets;
        local_targets.reserve(4u);
        local_targets.push_back(q_goal);
        local_targets.push_back(q_interpolated);
        local_targets.push_back(interpolateState(
            q_start,
            q_goal,
            base,
            std::clamp(t - 0.08 * bump, 0.0, 1.0)));
        local_targets.push_back(interpolateState(
            q_start,
            q_goal,
            base,
            std::clamp(t + 0.08 * bump, 0.0, 1.0)));

        for (const Eigen::VectorXd& target : local_targets) {
            if (target.size() != seed.size() ||
                candidates.size() >= kMaxArmCandidatesPerLayer) {
                continue;
            }
            Eigen::VectorXd step = target;
            step.segment(kBaseDof, arm_dof) =
                target.segment(kBaseDof, arm_dof);
            tryAddReachableFromSeed(seed, step);

            Eigen::VectorXd blend = seed;
            blend.segment(kBaseDof, arm_dof) =
                0.65 * step.segment(kBaseDof, arm_dof) +
                0.35 * q_interpolated.segment(kBaseDof, arm_dof);
            tryAddReachableFromSeed(seed, blend);
        }

        Eigen::VectorXd step = seed;
        Eigen::VectorXd arm_delta =
            q_goal.segment(kBaseDof, arm_dof) -
            seed.segment(kBaseDof, arm_dof);
        for (Eigen::Index joint = 0; joint < arm_dof; ++joint) {
            arm_delta[joint] = std::clamp(
                arm_delta[joint], -max_delta, max_delta);
        }
        step.segment(kBaseDof, arm_dof) =
            seed.segment(kBaseDof, arm_dof) + arm_delta;

        for (int sample = 0;
             sample < kLayerSeedExplorationSamples &&
             candidates.size() < kMaxArmCandidatesPerLayer;
             ++sample) {
            Eigen::VectorXd q = step;
            const double scale =
                std::min(
                    0.45 * max_delta,
                    (0.015 + 0.045 *
                            deterministicUnitNoise(
                                static_cast<double>(
                                    (layer_index + 1u) * 6151u + sample * 97))) *
                        bump);
            for (Eigen::Index joint = 0; joint < arm_dof; ++joint) {
                q[kBaseDof + joint] +=
                    scale * deterministicSignedNoise(layer_index, sample, joint);
            }
            tryAddReachableFromSeed(seed, std::move(q));
        }

        for (Eigen::Index joint = 0;
             joint < arm_dof && candidates.size() < kMaxArmCandidatesPerLayer;
             ++joint) {
            const double delta = 0.55 * max_delta;
            for (const double sign : {1.0, -1.0}) {
                Eigen::VectorXd q = seed;
                q[kBaseDof + joint] += sign * delta;
                if (has_limits) {
                    const double lo = std::min(
                        input.q_min[kBaseDof + joint],
                        input.q_max[kBaseDof + joint]);
                    const double hi = std::max(
                        input.q_min[kBaseDof + joint],
                        input.q_max[kBaseDof + joint]);
                    q[kBaseDof + joint] = std::clamp(q[kBaseDof + joint], lo, hi);
                }
                tryAddReachableFromSeed(seed, std::move(q));
                if (candidates.size() >= kMaxArmCandidatesPerLayer) {
                    break;
                }
            }
        }
    }
    if (candidates.empty()) {
        tryAdd(q_interpolated);
    }
    return candidates;
}

bool BaseGuidedWholeBodyPlanner::validatePath(
    const JointPath& path,
    const cp::PathPlanningInput& input,
    const double safe_distance) const {
    for (std::size_t i = 0; i < path.size(); ++i) {
        cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
        if (!input.joint_state_validator(path[i], safe_distance, &state_diag)) {
            std::printf(
                "[base_guided_planner] state rejected idx=%zu margin=%.5f reason=%s worst_link=%s worst_distance=%.5f required=%.5f\n",
                i,
                state_diag.min_margin,
                state_diag.reason.c_str(),
                state_diag.worst_link_name.c_str(),
                state_diag.worst_distance,
                state_diag.required_clearance);
            return false;
        }
        if (i == 0) {
            continue;
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
        if (!input.joint_segment_validator(
                path[i - 1], path[i], safe_distance, &segment_diag)) {
            std::printf(
                "[base_guided_planner] segment rejected idx=%zu->%zu margin=%.5f reason=%s worst_link=%s segment_t=%.3f worst_distance=%.5f required=%.5f\n",
                i - 1,
                i,
                segment_diag.min_margin,
                segment_diag.reason.c_str(),
                segment_diag.worst_link_name.c_str(),
                segment_diag.failed_segment_t,
                segment_diag.worst_distance,
                segment_diag.required_clearance);
            return false;
        }
    }
    return true;
}

bool BaseGuidedWholeBodyPlanner::validateLayerPath(
    const JointPath& path,
    const cp::PathPlanningInput& input,
    const double safe_distance) const {
    for (std::size_t i = 0; i < path.size(); ++i) {
        cp::PathPlanningInput::WholeBodyPoseDiagnostic state_diag;
        if (!validateLayerState(input, path[i], safe_distance, &state_diag)) {
            return false;
        }
        if (i == 0) {
            continue;
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic segment_diag;
        if (!validateLayerSegment(
                input, path[i - 1], path[i], safe_distance, &segment_diag)) {
            return false;
        }
    }
    return true;
}

BaseGuidedWholeBodyPlanner::JointPath
BaseGuidedWholeBodyPlanner::smoothArmPathConservatively(
    const BaseRoute& route,
    JointPath path,
    const cp::PathPlanningInput& input,
    const double safe_distance) const {
    if (path.size() < 3 || route.size() != path.size()) {
        return path;
    }

    const double initial_cost = armPathMotionCost(path);
    for (int iter = 0; iter < kArmPathSmoothingIterations; ++iter) {
        bool changed = false;
        for (std::size_t i = 1; i + 1u < path.size(); ++i) {
            if (path[i - 1].size() != path[i].size() ||
                path[i + 1].size() != path[i].size() ||
                path[i].size() <= kBaseDof) {
                continue;
            }
            const Eigen::Index arm_dof = path[i].size() - kBaseDof;
            Eigen::VectorXd candidate = path[i];
            candidate.segment(kBaseDof, arm_dof) =
                0.25 * path[i - 1].segment(kBaseDof, arm_dof) +
                0.50 * path[i].segment(kBaseDof, arm_dof) +
                0.25 * path[i + 1].segment(kBaseDof, arm_dof);
            candidate[0] = route[i].x;
            candidate[1] = route[i].y;
            candidate[2] = route[i].yaw;
            if (input.q_min.size() == candidate.size() &&
                input.q_max.size() == candidate.size()) {
                for (Eigen::Index joint = kBaseDof; joint < candidate.size(); ++joint) {
                    candidate[joint] = std::clamp(
                        candidate[joint],
                        std::min(input.q_min[joint], input.q_max[joint]),
                        std::max(input.q_min[joint], input.q_max[joint]));
                }
            }

            const double before =
                (path[i].segment(kBaseDof, arm_dof) -
                 path[i - 1].segment(kBaseDof, arm_dof))
                    .norm() +
                (path[i + 1].segment(kBaseDof, arm_dof) -
                 path[i].segment(kBaseDof, arm_dof))
                    .norm();
            const double after =
                (candidate.segment(kBaseDof, arm_dof) -
                 path[i - 1].segment(kBaseDof, arm_dof))
                    .norm() +
                (path[i + 1].segment(kBaseDof, arm_dof) -
                 candidate.segment(kBaseDof, arm_dof))
                    .norm();
            if (after >= before - 1e-6) {
                continue;
            }

            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (!validateLayerState(input, candidate, safe_distance, &diag)) {
                continue;
            }
            const std::vector<Eigen::VectorXd> before_window = {
                path[i - 1], path[i], path[i + 1]};
            const std::vector<Eigen::VectorXd> after_window = {
                path[i - 1], candidate, path[i + 1]};
            PathClearanceCost before_clearance;
            PathClearanceCost after_clearance;
            (void)clearanceAwarePathSelectionCost(
                before_window, input, safe_distance, &before_clearance);
            (void)clearanceAwarePathSelectionCost(
                after_window, input, safe_distance, &after_clearance);
            if (!std::isfinite(after_clearance.cost) ||
                after_clearance.min_margin + 1e-4 <
                    before_clearance.min_margin) {
                continue;
            }
            if (!validateLayerEdgeAlongBase(
                    route.states,
                    route.segment_check_states,
                    static_cast<int>(i - 1u),
                    static_cast<int>(i),
                    path[i - 1],
                    candidate,
                    input,
                    safe_distance,
                    &diag)) {
                continue;
            }
            if (!validateLayerEdgeAlongBase(
                    route.states,
                    route.segment_check_states,
                    static_cast<int>(i),
                    static_cast<int>(i + 1u),
                    candidate,
                    path[i + 1],
                    input,
                    safe_distance,
                    &diag)) {
                continue;
            }
            path[i] = std::move(candidate);
            changed = true;
        }
        if (!changed) {
            break;
        }
    }

    const double final_cost = armPathMotionCost(path);
    if (std::isfinite(initial_cost) && std::isfinite(final_cost) &&
        final_cost + 1e-6 < initial_cost) {
        std::printf(
            "[base_guided_planner] smoothed global arm path motion %.3f -> %.3f\n",
            initial_cost,
            final_cost);
    }
    return path;
}

cp::TimedJointTrajectory BaseGuidedWholeBodyPlanner::buildTrajectory(
    const JointPath& path,
    const std::vector<double>& segment_times) const {
    cp::TimedJointTrajectory traj;
    if (path.size() < 2) {
        return traj;
    }
    traj.joint_targets = path;
    traj.segment_kind = cp::PlannedSegmentKind::Goal;
    traj.segment_durations.reserve(path.size() - 1);
    traj.cumulative_times.reserve(path.size());
    traj.cumulative_times.push_back(0.0);

    double total_time = 0.0;
    const bool use_supplied_times =
        segment_times.size() + 1u == path.size();
    for (std::size_t i = 1; i < path.size(); ++i) {
        const double base_motion =
            (path[i].head<2>() - path[i - 1].head<2>()).norm();
        const double yaw_motion =
            std::abs(angularDistance(path[i - 1][2], path[i][2]));
        const double arm_motion =
            (path[i].segment(kBaseDof, path[i].size() - kBaseDof) -
             path[i - 1].segment(kBaseDof, path[i - 1].size() - kBaseDof))
                .norm();
        const double blended_motion =
            base_motion + 0.20 * yaw_motion + 0.15 * arm_motion;
        const double dt =
            use_supplied_times
                ? std::max(kMinSegmentDt, segment_times[i - 1u])
                : std::max(
                      kMinSegmentDt,
                      blended_motion /
                          std::max(1e-3, common_cfg_.default_segment_speed));
        traj.segment_durations.push_back(dt);
        total_time += dt;
        traj.cumulative_times.push_back(total_time);
    }
    traj.total_duration = total_time;
    return traj;
}

BaseGuidedWholeBodyPlanner::BaseState BaseGuidedWholeBodyPlanner::baseFromQ(
    const Eigen::VectorXd& q) {
    BaseState base;
    if (q.size() >= kBaseDof) {
        base.x = q[0];
        base.y = q[1];
        base.yaw = normalizeAngle(q[2]);
    }
    return base;
}

double BaseGuidedWholeBodyPlanner::normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double BaseGuidedWholeBodyPlanner::angularDistance(
    const double from,
    const double to) {
    return normalizeAngle(to - from);
}

Eigen::VectorXd BaseGuidedWholeBodyPlanner::interpolateState(
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const BaseState& base,
    const double arm_alpha) {
    Eigen::VectorXd q = q_start;
    q[0] = base.x;
    q[1] = base.y;
    q[2] = base.yaw;
    if (q.size() > kBaseDof) {
        q.segment(kBaseDof, q.size() - kBaseDof) =
            (1.0 - arm_alpha) * q_start.segment(kBaseDof, q.size() - kBaseDof) +
            arm_alpha * q_goal.segment(kBaseDof, q.size() - kBaseDof);
    }
    return q;
}

std::vector<double> BaseGuidedWholeBodyPlanner::estimateRouteSegmentTimes(
    const BaseRoute& route,
    const double default_segment_speed) {
    std::vector<double> times;
    if (route.size() < 2) {
        return times;
    }
    if (route.segment_times.size() + 1u == route.size()) {
        return route.segment_times;
    }
    times.reserve(route.size() - 1u);
    const double speed = std::max(1e-3, default_segment_speed);
    for (std::size_t i = 1; i < route.size(); ++i) {
        const double base_motion =
            std::hypot(route[i].x - route[i - 1].x, route[i].y - route[i - 1].y);
        const double yaw_motion =
            std::abs(angularDistance(route[i - 1].yaw, route[i].yaw));
        times.push_back(
            std::max(kMinSegmentDt, (base_motion + 0.20 * yaw_motion) / speed));
    }
    return times;
}

std::vector<double> BaseGuidedWholeBodyPlanner::estimateArmLayerSegmentTimes(
    const BaseRoute& route,
    const double default_segment_speed) {
    std::vector<double> times =
        estimateRouteSegmentTimes(route, default_segment_speed);
    for (double& dt : times) {
        dt = std::max(kMinArmLayerDt, kLayerRrtTimeScale * dt);
    }
    return times;
}

std::vector<double> BaseGuidedWholeBodyPlanner::estimateWholeBodyPathSegmentTimes(
    const JointPath& path,
    const double default_segment_speed) {
    std::vector<double> times;
    if (path.size() < 2u) {
        return times;
    }
    times.reserve(path.size() - 1u);
    const double base_speed = std::max(1e-3, default_segment_speed);
    const double max_joint_vel =
        std::max(1e-3, kTrajectoryArmMaxJointVelRadPerSec);
    const double max_joint_acc =
        std::max(1e-3, kTrajectoryArmMaxJointAccRadPerSec2);
    const double t_acc = max_joint_vel / max_joint_acc;
    const double dist_acc = max_joint_acc * t_acc * t_acc;

    for (std::size_t i = 1u; i < path.size(); ++i) {
        if (path[i - 1u].size() != path[i].size() ||
            path[i].size() < kBaseDof) {
            return {};
        }
        const double base_motion =
            (path[i].head<2>() - path[i - 1u].head<2>()).norm();
        const double yaw_motion =
            std::abs(angularDistance(path[i - 1u][2], path[i][2]));
        double dt =
            std::max(kMinSegmentDt,
                     (base_motion + 0.20 * yaw_motion) / base_speed);

        const bool endpoint_segment =
            i == 1u || i + 1u == path.size();
        for (Eigen::Index j = kBaseDof; j < path[i].size(); ++j) {
            const double err = std::abs(path[i][j] - path[i - 1u][j]);
            double arm_dt = 0.0;
            if (endpoint_segment) {
                arm_dt = err <= dist_acc
                             ? std::sqrt(err / max_joint_acc)
                             : (err - dist_acc) / max_joint_vel + 2.0 * t_acc;
            } else {
                arm_dt = err / max_joint_vel;
            }
            dt = std::max(dt, arm_dt);
        }
        times.push_back(std::max(kMinArmLayerDt, dt));
    }

    if (!times.empty()) {
        times.front() *= 1.5;
        times.back() *= 1.5;
    }
    return times;
}

}  // namespace arm_controller::algorithm::global_planner
