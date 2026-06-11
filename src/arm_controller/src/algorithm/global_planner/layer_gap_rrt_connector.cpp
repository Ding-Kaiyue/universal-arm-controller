#include "algorithm/global_planner/layer_gap_rrt_connector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <string>

namespace arm_controller::algorithm::global_planner {

namespace {
constexpr int kBaseDof = 3;
constexpr double kLayerEdgeCheckBaseStepM = 0.03;
constexpr double kLayerEdgeCheckYawStepRad = 0.10;
constexpr double kLayerEdgeCheckJointStepRad = 0.08;

enum class TreeState {
    Forward,
    Backward,
};

struct TreeNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::VectorXd arm;
    int layer{0};
    int parent{-1};
    TreeState tree{TreeState::Forward};
    double g_score{0.0};
};

std::string nodeKey(
    const TreeState tree,
    const int layer,
    const Eigen::VectorXd& arm) {
    std::string key = tree == TreeState::Forward ? "F:" : "B:";
    key += std::to_string(layer);
    key += ":";
    for (Eigen::Index i = 0; i < arm.size(); ++i) {
        key += std::to_string(static_cast<long long>(std::llround(arm[i] * 100.0)));
        key += ",";
    }
    return key;
}

Eigen::VectorXd composeFullState(
    const std::vector<LayerGapRrtConnector::BaseState>& route,
    const int layer,
    const Eigen::VectorXd& arm) {
    Eigen::VectorXd q(kBaseDof + arm.size());
    const auto& base = route[static_cast<std::size_t>(layer)];
    q[0] = base.x;
    q[1] = base.y;
    q[2] = base.yaw;
    q.segment(kBaseDof, arm.size()) = arm;
    return q;
}

int nearestNodeInLayer(
    const std::vector<TreeNode>& nodes,
    const int layer,
    const TreeState tree,
    const Eigen::VectorXd& arm_target) {
    int best = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const TreeNode& node = nodes[i];
        if (node.layer != layer || node.tree != tree ||
            node.arm.size() != arm_target.size()) {
            continue;
        }
        const double cost = (node.arm - arm_target).squaredNorm();
        if (cost < best_cost) {
            best_cost = cost;
            best = static_cast<int>(i);
        }
    }
    return best;
}

int frontierLayer(
    const std::vector<TreeNode>& nodes,
    const TreeState tree,
    const bool forward_tree) {
    int layer = forward_tree ? 0 : std::numeric_limits<int>::max();
    bool found = false;
    for (const TreeNode& node : nodes) {
        if (node.tree != tree) {
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
        return forward_tree ? 0 : 0;
    }
    return layer;
}

double edgeCost(const TreeNode& from, const TreeNode& to) {
    if (from.arm.size() != to.arm.size()) {
        return std::numeric_limits<double>::infinity();
    }
    return (from.arm - to.arm).norm() +
           1.0e-3 * static_cast<double>(std::abs(from.layer - to.layer));
}

LayerGapRrtConnector::Result reconstructPath(
    const std::vector<TreeNode>& nodes,
    const int forward_index,
    const int backward_index) {
    std::vector<Eigen::VectorXd> prefix_arms;
    std::vector<int> prefix_layers;
    for (int index = forward_index; index >= 0; index = nodes[index].parent) {
        const TreeNode& node = nodes[static_cast<std::size_t>(index)];
        prefix_arms.push_back(node.arm);
        prefix_layers.push_back(node.layer);
    }
    std::reverse(prefix_arms.begin(), prefix_arms.end());
    std::reverse(prefix_layers.begin(), prefix_layers.end());

    std::vector<Eigen::VectorXd> suffix_arms;
    std::vector<int> suffix_layers;
    for (int index = backward_index; index >= 0; index = nodes[index].parent) {
        const TreeNode& node = nodes[static_cast<std::size_t>(index)];
        suffix_arms.push_back(node.arm);
        suffix_layers.push_back(node.layer);
    }

    LayerGapRrtConnector::Result result;
    result.path = std::move(prefix_arms);
    result.path.insert(result.path.end(), suffix_arms.begin(), suffix_arms.end());
    result.path_layers = std::move(prefix_layers);
    result.path_layers.insert(
        result.path_layers.end(), suffix_layers.begin(), suffix_layers.end());
    return result;
}

std::uint32_t makeSeed(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    std::uint32_t seed = 2166136261u;
    for (Eigen::Index i = 0; i < a.size(); ++i) {
        seed ^= static_cast<std::uint32_t>(
            std::llround((a[i] + 23.0) * 10000.0));
        seed *= 16777619u;
        seed ^= static_cast<std::uint32_t>(
            std::llround((b[i] + 29.0) * 10000.0));
        seed *= 16777619u;
    }
    return seed;
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

LayerGapRrtConnector::BaseState interpolateBaseState(
    const LayerGapRrtConnector::BaseState& from,
    const LayerGapRrtConnector::BaseState& to,
    const double t) {
    LayerGapRrtConnector::BaseState base;
    base.x = from.x + t * (to.x - from.x);
    base.y = from.y + t * (to.y - from.y);
    base.yaw = normalizeAngleLocal(
        from.yaw + t * normalizeAngleLocal(to.yaw - from.yaw));
    return base;
}

bool validateLayerEdgeAlongBase(
    const LayerGapRrtConnector::Input& input,
    const int from_layer,
    const int to_layer,
    const Eigen::VectorXd& arm_from,
    const Eigen::VectorXd& arm_to,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* failed_diag) {
    if (input.planning_input == nullptr ||
        arm_from.size() != arm_to.size() || arm_from.size() <= 0 ||
        std::abs(to_layer - from_layer) != 1 ||
        from_layer < 0 || to_layer < 0 ||
        from_layer >= static_cast<int>(input.route.size()) ||
        to_layer >= static_cast<int>(input.route.size())) {
        if (failed_diag != nullptr) {
            failed_diag->reason = "invalid_layer_edge";
            failed_diag->min_margin = -1.0;
        }
        return false;
    }

    const int segment_index = std::min(from_layer, to_layer);
    if (segment_index >= 0 &&
        segment_index < static_cast<int>(input.segment_check_states.size()) &&
        !input.segment_check_states[static_cast<std::size_t>(segment_index)].empty()) {
        const auto& checks =
            input.segment_check_states[static_cast<std::size_t>(segment_index)];
        Eigen::VectorXd previous_q;
        for (std::size_t i = 0; i < checks.size(); ++i) {
            const double raw_t =
                checks.size() <= 1u
                    ? 1.0
                    : static_cast<double>(i) /
                          static_cast<double>(checks.size() - 1u);
            const double local_t =
                from_layer < to_layer ? raw_t : 1.0 - raw_t;
            const LayerGapRrtConnector::BaseState& base = checks[i];
            Eigen::VectorXd arm = arm_from + local_t * (arm_to - arm_from);
            Eigen::VectorXd q(kBaseDof + arm.size());
            q[0] = base.x;
            q[1] = base.y;
            q[2] = base.yaw;
            q.segment(kBaseDof, arm.size()) = arm;

            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (!validateLayerState(
                    *input.planning_input, q, input.safe_distance, &diag)) {
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
                        *input.planning_input,
                        previous_q,
                        q,
                        input.safe_distance,
                        &segment_diag)) {
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

    const LayerGapRrtConnector::BaseState& base_from =
        input.route[static_cast<std::size_t>(segment_index)];
    const LayerGapRrtConnector::BaseState& base_to =
        input.route[static_cast<std::size_t>(segment_index + 1)];
    const double base_dist =
        std::hypot(base_to.x - base_from.x, base_to.y - base_from.y);
    const double yaw_dist =
        std::abs(normalizeAngleLocal(base_to.yaw - base_from.yaw));
    const double joint_dist =
        (arm_to - arm_from).cwiseAbs().maxCoeff();
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
        const LayerGapRrtConnector::BaseState base =
            interpolateBaseState(base_from, base_to, t);
        Eigen::VectorXd arm = arm_from + local_t * (arm_to - arm_from);
        Eigen::VectorXd q(kBaseDof + arm.size());
        q[0] = base.x;
        q[1] = base.y;
        q[2] = base.yaw;
        q.segment(kBaseDof, arm.size()) = arm;

        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateLayerState(
                *input.planning_input, q, input.safe_distance, &diag)) {
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
                    *input.planning_input,
                    previous_q,
                    q,
                    input.safe_distance,
                    &segment_diag)) {
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

bool validateWholePath(
    const std::vector<Eigen::VectorXd>& path,
    const std::vector<int>& path_layers,
    const LayerGapRrtConnector::Input& connector_input,
    const cp::PathPlanningInput& input,
    const double safe_distance) {
    if (path.size() < 2 || path.size() != path_layers.size()) {
        return false;
    }
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (path_layers[i] < 0 ||
            path_layers[i] >= static_cast<int>(connector_input.route.size())) {
            return false;
        }
        const Eigen::VectorXd q = composeFullState(
            connector_input.route, path_layers[i], path[i]);
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateLayerState(input, q, safe_distance, &diag)) {
            return false;
        }
        if (i == 0) {
            continue;
        }
        if (std::abs(path_layers[i] - path_layers[i - 1u]) != 1) {
            if (!validateLayerSegment(
                    input,
                    composeFullState(
                        connector_input.route, path_layers[i - 1u], path[i - 1u]),
                    q,
                    safe_distance,
                    &diag)) {
                return false;
            }
            continue;
        }
        if (!validateLayerEdgeAlongBase(
                connector_input,
                path_layers[i - 1u],
                path_layers[i],
                path[i - 1u],
                path[i],
                &diag)) {
            return false;
        }
    }
    return true;
}

bool isCompleteEndpointPath(
    const std::vector<Eigen::VectorXd>& path,
    const Eigen::VectorXd& arm_start,
    const Eigen::VectorXd& arm_goal,
    const double tolerance) {
    if (path.size() < 2 || path.front().size() != arm_start.size() ||
        path.back().size() != arm_goal.size()) {
        return false;
    }
    const double start_err =
        (path.front() - arm_start).cwiseAbs().maxCoeff();
    const double goal_err =
        (path.back() - arm_goal).cwiseAbs().maxCoeff();
    return start_err <= tolerance && goal_err <= tolerance;
}

LayerGapRrtConnector::Result shortcutPath(
    LayerGapRrtConnector::Result path_result,
    const LayerGapRrtConnector::Input& connector_input,
    const bool preserve_layer_order) {
    if (preserve_layer_order) {
        return path_result;
    }
    if (path_result.path.size() < 3 ||
        path_result.path.size() != path_result.path_layers.size()) {
        return path_result;
    }
    std::vector<Eigen::VectorXd> shortened;
    std::vector<int> shortened_layers;
    shortened.reserve(path_result.path.size());
    shortened_layers.reserve(path_result.path_layers.size());
    std::size_t i = 0;
    shortened.push_back(path_result.path.front());
    shortened_layers.push_back(path_result.path_layers.front());
    while (i + 1 < path_result.path.size()) {
        std::size_t best = i + 1;
        for (std::size_t j = path_result.path.size() - 1; j > i + 1; --j) {
            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (std::abs(path_result.path_layers[j] -
                         path_result.path_layers[i]) == 1 &&
                validateLayerEdgeAlongBase(
                    connector_input,
                    path_result.path_layers[i],
                    path_result.path_layers[j],
                    path_result.path[i],
                    path_result.path[j],
                    &diag)) {
                best = j;
                break;
            }
        }
        shortened.push_back(path_result.path[best]);
        shortened_layers.push_back(path_result.path_layers[best]);
        i = best;
    }
    path_result.path = std::move(shortened);
    path_result.path_layers = std::move(shortened_layers);
    return path_result;
}

void addRejectReason(
    std::map<std::string, std::size_t>* reasons,
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
}

std::string summarizeRejectReasons(
    const std::map<std::string, std::size_t>& reasons) {
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
}

}  // namespace

LayerGapRrtConnector::LayerGapRrtConnector()
    : LayerGapRrtConnector(Config{}) {}

LayerGapRrtConnector::LayerGapRrtConnector(Config config)
    : config_(config) {}

LayerGapRrtConnector::Result LayerGapRrtConnector::connect(
    const Input& input) const {
    Result result;
    if (input.planning_input == nullptr || input.route.size() < 2 ||
        input.segment_times.size() + 1 != input.route.size() ||
        input.arm_start.size() != input.arm_goal.size() ||
        input.arm_start.size() <= 0 ||
        (!input.planning_input->manipulator_state_validator &&
         !input.planning_input->joint_state_validator) ||
        (!input.planning_input->manipulator_segment_validator &&
         !input.planning_input->joint_segment_validator)) {
        return result;
    }

    const cp::PathPlanningInput& planning_input = *input.planning_input;
    std::vector<TreeNode> nodes;
    nodes.reserve(static_cast<std::size_t>(config_.max_iterations) + 2u);
    std::map<std::string, int> node_pool;

    auto addNode = [&](const Eigen::VectorXd& arm,
                       const int layer,
                       const int parent,
                       const TreeState tree) {
        const std::string key = nodeKey(tree, layer, arm);
        const auto existing = node_pool.find(key);
        if (existing != node_pool.end()) {
            const int existing_index = existing->second;
            if (parent >= 0 && existing_index >= 0 &&
                existing_index < static_cast<int>(nodes.size())) {
                TreeNode& existing_node =
                    nodes[static_cast<std::size_t>(existing_index)];
                if (existing_node.parent < 0) {
                    existing_node.parent = parent;
                }
            }
            return existing_index;
        }
        TreeNode node;
        node.arm = arm;
        node.layer = layer;
        node.parent = parent;
        node.tree = tree;
        node.g_score =
            parent >= 0 && parent < static_cast<int>(nodes.size())
                ? nodes[static_cast<std::size_t>(parent)].g_score +
                      edgeCost(nodes[static_cast<std::size_t>(parent)], node)
                : 0.0;
        nodes.push_back(std::move(node));
        const int index = static_cast<int>(nodes.size() - 1u);
        node_pool.emplace(key, index);
        return index;
    };

    auto addSeedSet = [&](const std::vector<Input::SeedState>& seeds,
                          const TreeState tree,
                          const Eigen::VectorXd& fallback_q,
                          const int fallback_layer) {
        if (seeds.empty()) {
            addNode(fallback_q, fallback_layer, -1, tree);
            return;
        }
        int added = 0;
        for (const Input::SeedState& seed_state : seeds) {
            if (seed_state.arm.size() != fallback_q.size() ||
                seed_state.layer < 0 ||
                seed_state.layer >= static_cast<int>(input.route.size())) {
                continue;
            }
            addNode(seed_state.arm, seed_state.layer, -1, tree);
            ++added;
        }
        if (added == 0) {
            addNode(fallback_q, fallback_layer, -1, tree);
        }
    };
    addSeedSet(input.start_seeds, TreeState::Forward, input.arm_start, 0);
    addSeedSet(
        input.goal_seeds,
        TreeState::Backward,
        input.arm_goal,
        static_cast<int>(input.route.size() - 1u));

    std::mt19937 rng(makeSeed(input.arm_start, input.arm_goal));
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    std::uniform_int_distribution<int> layer_dist(
        1, std::max(1, static_cast<int>(input.route.size()) - 2));
    std::map<std::string, std::size_t> extend_reasons;

    auto makeTarget = [&](const bool expand_forward) {
        Eigen::VectorXd arm_target =
            expand_forward ? input.arm_goal : input.arm_start;
        int layer = expand_forward ? static_cast<int>(input.route.size()) - 1 : 0;
        if (input.route.size() > 2 && unit_dist(rng) > config_.goal_bias) {
            layer = layer_dist(rng);
            const double t = static_cast<double>(layer) /
                             static_cast<double>(input.route.size() - 1u);
            const double alpha =
                std::clamp(t + (unit_dist(rng) - 0.5) * 0.12, 0.0, 1.0);
            arm_target = (1.0 - alpha) * input.arm_start + alpha * input.arm_goal;
            for (Eigen::Index i = 0; i < arm_target.size(); ++i) {
                arm_target[i] += (unit_dist(rng) - 0.5) * 0.04;
            }
        }
        return std::pair<int, Eigen::VectorXd>(layer, arm_target);
    };

    auto extendOneStep = [&](const int near_index,
                             const Eigen::VectorXd& arm_target,
                             const TreeState tree,
                             int* out_index,
                             cp::PathPlanningInput::WholeBodyPoseDiagnostic* reject_diag) {
        if (near_index < 0 || out_index == nullptr) {
            if (reject_diag != nullptr) {
                reject_diag->reason = "missing_nearest";
                reject_diag->min_margin = -1.0;
            }
            return false;
        }
        const TreeNode& near = nodes[static_cast<std::size_t>(near_index)];
        const int next_layer =
            tree == TreeState::Forward ? near.layer + 1 : near.layer - 1;
        if (next_layer < 0 || next_layer >= static_cast<int>(input.route.size())) {
            if (reject_diag != nullptr) {
                reject_diag->reason = "next_layer_out_of_range";
                reject_diag->min_margin = -1.0;
            }
            return false;
        }

        const int segment_index = std::min(near.layer, next_layer);
        const double dt =
            config_.time_scale *
            input.segment_times[static_cast<std::size_t>(segment_index)];
        const double max_delta =
            std::max(1e-6, config_.max_joint_velocity_rad_per_sec * dt);

        Eigen::VectorXd arm_delta =
            arm_target - near.arm;
        for (Eigen::Index i = 0; i < arm_delta.size(); ++i) {
            arm_delta[i] = std::clamp(arm_delta[i], -max_delta, max_delta);
        }
        Eigen::VectorXd arm_new = near.arm + arm_delta;

        if (next_layer == 0 || next_layer == static_cast<int>(input.route.size()) - 1) {
            const Eigen::VectorXd& boundary =
                next_layer == 0 ? input.arm_start : input.arm_goal;
            if ((boundary - arm_new).cwiseAbs().maxCoeff() <= max_delta) {
                arm_new = boundary;
            }
        }

        if (planning_input.q_min.size() == kBaseDof + arm_new.size() &&
            planning_input.q_max.size() == kBaseDof + arm_new.size()) {
            for (Eigen::Index i = 0; i < arm_new.size(); ++i) {
                const Eigen::Index full_i = kBaseDof + i;
                arm_new[i] = std::clamp(
                    arm_new[i],
                    std::min(planning_input.q_min[full_i], planning_input.q_max[full_i]),
                    std::max(planning_input.q_min[full_i], planning_input.q_max[full_i]));
            }
        } else if (planning_input.q_min.size() == arm_new.size() &&
                   planning_input.q_max.size() == arm_new.size()) {
            for (Eigen::Index i = 0; i < arm_new.size(); ++i) {
                arm_new[i] = std::clamp(
                    arm_new[i],
                    std::min(planning_input.q_min[i], planning_input.q_max[i]),
                    std::max(planning_input.q_min[i], planning_input.q_max[i]));
            }
        }

        const Eigen::VectorXd q_new =
            composeFullState(input.route, next_layer, arm_new);
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        if (!validateLayerState(
                planning_input, q_new, input.safe_distance, &diag)) {
            if (reject_diag != nullptr) {
                *reject_diag = diag;
            }
            return false;
        }
        if (!validateLayerEdgeAlongBase(
                input, near.layer, next_layer, near.arm, arm_new, &diag)) {
            if (reject_diag != nullptr) {
                *reject_diag = diag;
            }
            return false;
        }

        *out_index = addNode(arm_new, next_layer, near_index, tree);
        return true;
    };

    auto adjacentEdgeFeasible = [&](const int from_index, const int to_index) {
        if (from_index < 0 || to_index < 0 ||
            from_index >= static_cast<int>(nodes.size()) ||
            to_index >= static_cast<int>(nodes.size())) {
            return false;
        }
        const TreeNode& from = nodes[static_cast<std::size_t>(from_index)];
        const TreeNode& to = nodes[static_cast<std::size_t>(to_index)];
        if (from.tree != to.tree || from.arm.size() != to.arm.size() ||
            std::abs(from.layer - to.layer) != 1) {
            return false;
        }
        const int segment_index = std::min(from.layer, to.layer);
        if (segment_index < 0 ||
            segment_index >= static_cast<int>(input.segment_times.size())) {
            return false;
        }
        const double dt =
            config_.time_scale *
            input.segment_times[static_cast<std::size_t>(segment_index)];
        const double max_delta =
            std::max(1.0e-6, config_.max_joint_velocity_rad_per_sec * dt);
        if ((to.arm - from.arm).cwiseAbs().maxCoeff() > max_delta) {
            return false;
        }
        cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
        return validateLayerEdgeAlongBase(
            input, from.layer, to.layer, from.arm, to.arm, &diag);
    };

    auto adjustTree = [&](const int new_index) {
        if (new_index < 0 ||
            new_index >= static_cast<int>(nodes.size())) {
            return;
        }
        const TreeNode q_new = nodes[static_cast<std::size_t>(new_index)];
        const int next_layer =
            q_new.tree == TreeState::Forward ? q_new.layer + 1 : q_new.layer - 1;
        if (next_layer < 0 ||
            next_layer >= static_cast<int>(input.route.size())) {
            return;
        }
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            if (static_cast<int>(i) == new_index) {
                continue;
            }
            TreeNode& candidate = nodes[i];
            if (candidate.tree != q_new.tree || candidate.layer != next_layer ||
                candidate.arm.size() != q_new.arm.size()) {
                continue;
            }
            const double new_cost =
                q_new.g_score + edgeCost(q_new, candidate);
            if (new_cost + 1.0e-9 >= candidate.g_score) {
                continue;
            }
            if (!adjacentEdgeFeasible(new_index, static_cast<int>(i))) {
                continue;
            }
            candidate.parent = new_index;
            candidate.g_score = new_cost;
        }
    };

    auto tryMerge = [&](const int a_index, const int b_index) {
        if (a_index < 0 || b_index < 0) {
            return LayerGapRrtConnector::Result{};
        }
        const TreeNode& a = nodes[static_cast<std::size_t>(a_index)];
        const TreeNode& b = nodes[static_cast<std::size_t>(b_index)];
        if (a.layer != b.layer || a.arm.size() != b.arm.size()) {
            return LayerGapRrtConnector::Result{};
        }
        const double err = (a.arm - b.arm).cwiseAbs().maxCoeff();
        if (err > config_.meet_tolerance_rad) {
            const int bridge_segment = std::clamp(
                std::min(
                    a.layer,
                    static_cast<int>(input.segment_times.size()) - 1),
                0,
                static_cast<int>(input.segment_times.size()) - 1);
            const int previous_segment = std::clamp(
                bridge_segment - 1,
                0,
                static_cast<int>(input.segment_times.size()) - 1);
            const double bridge_dt =
                config_.time_scale *
                std::max(
                    input.segment_times[static_cast<std::size_t>(bridge_segment)],
                    input.segment_times[static_cast<std::size_t>(previous_segment)]);
            const double max_delta =
                std::max(
                    1e-6,
                    config_.max_joint_velocity_rad_per_sec * bridge_dt);
            if (err > max_delta) {
                return LayerGapRrtConnector::Result{};
            }
            cp::PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (!validateLayerSegment(
                    planning_input,
                    composeFullState(input.route, a.layer, a.arm),
                    composeFullState(input.route, b.layer, b.arm),
                    input.safe_distance,
                    &diag)) {
                return LayerGapRrtConnector::Result{};
            }
        }
        const int forward_index =
            a.tree == TreeState::Forward ? a_index : b_index;
        const int backward_index =
            a.tree == TreeState::Backward ? a_index : b_index;
        return reconstructPath(nodes, forward_index, backward_index);
    };

    for (int iter = 0; iter < config_.max_iterations; ++iter) {
        result.iterations = iter + 1;
        const auto forward_count = std::count_if(
            nodes.begin(), nodes.end(), [](const TreeNode& node) {
                return node.tree == TreeState::Forward;
            });
        const auto backward_count = static_cast<int>(nodes.size()) - forward_count;
        const bool expand_forward = forward_count <= backward_count;
        const TreeState active =
            expand_forward ? TreeState::Forward : TreeState::Backward;
        const TreeState opposite =
            expand_forward ? TreeState::Backward : TreeState::Forward;

        const auto [target_layer, arm_target] = makeTarget(expand_forward);
        int predecessor_layer = expand_forward ? target_layer - 1
                                               : target_layer + 1;
        if (expand_forward) {
            predecessor_layer = std::min(
                predecessor_layer,
                frontierLayer(nodes, active, true));
        } else {
            predecessor_layer = std::max(
                predecessor_layer,
                frontierLayer(nodes, active, false));
        }
        predecessor_layer = std::clamp(
            predecessor_layer, 0, static_cast<int>(input.route.size()) - 1);
        const int near_index =
            nearestNodeInLayer(nodes, predecessor_layer, active, arm_target);
        int new_index = -1;
        cp::PathPlanningInput::WholeBodyPoseDiagnostic extend_diag;
        if (!extendOneStep(
                near_index, arm_target, active, &new_index, &extend_diag)) {
            ++result.extend_rejects;
            addRejectReason(&extend_reasons, extend_diag);
            continue;
        }
        adjustTree(new_index);

        const TreeNode q_new = nodes[static_cast<std::size_t>(new_index)];
        int opposite_predecessor_layer =
            expand_forward ? q_new.layer + 1 : q_new.layer - 1;
        if (opposite_predecessor_layer >= 0 &&
            opposite_predecessor_layer < static_cast<int>(input.route.size())) {
            const int opposite_near = nearestNodeInLayer(
                nodes, opposite_predecessor_layer, opposite, q_new.arm);
            int opposite_new = -1;
            cp::PathPlanningInput::WholeBodyPoseDiagnostic opposite_diag;
            if (extendOneStep(
                    opposite_near, q_new.arm, opposite, &opposite_new, &opposite_diag)) {
                adjustTree(opposite_new);
                LayerGapRrtConnector::Result path_result =
                    tryMerge(new_index, opposite_new);
                if ((!path_result.path.empty()) &&
                    (!config_.require_complete_endpoint_path ||
                     isCompleteEndpointPath(
                         path_result.path,
                         input.arm_start,
                         input.arm_goal,
                         config_.meet_tolerance_rad)) &&
                    validateWholePath(
                        path_result.path,
                        path_result.path_layers,
                        input,
                        planning_input,
                        input.safe_distance)) {
                    result = shortcutPath(
                        std::move(path_result),
                        input,
                        config_.require_complete_endpoint_path);
                    result.success = true;
                    result.start_layer = config_.require_complete_endpoint_path
                                             ? 0
                                             : nodes[static_cast<std::size_t>(
                                                   new_index)]
                                                   .layer;
                    result.end_layer =
                        config_.require_complete_endpoint_path
                            ? static_cast<int>(input.route.size()) - 1
                            : nodes[static_cast<std::size_t>(opposite_new)].layer;
                    result.node_count = nodes.size();
                    result.top_extend_reject_reason =
                        summarizeRejectReasons(extend_reasons);
                    return result;
                }
            } else {
                ++result.extend_rejects;
                addRejectReason(&extend_reasons, opposite_diag);
            }
        }

        const int same_layer_opposite =
            nearestNodeInLayer(nodes, q_new.layer, opposite, q_new.arm);
        LayerGapRrtConnector::Result path_result =
            tryMerge(new_index, same_layer_opposite);
        if ((!path_result.path.empty()) &&
            (!config_.require_complete_endpoint_path ||
             isCompleteEndpointPath(
                 path_result.path,
                 input.arm_start,
                 input.arm_goal,
                 config_.meet_tolerance_rad)) &&
            validateWholePath(
                path_result.path,
                path_result.path_layers,
                input,
                planning_input,
                input.safe_distance)) {
            result = shortcutPath(
                std::move(path_result),
                input,
                config_.require_complete_endpoint_path);
            result.success = true;
            result.start_layer = config_.require_complete_endpoint_path
                                     ? 0
                                     : nodes[static_cast<std::size_t>(new_index)].layer;
            result.end_layer =
                config_.require_complete_endpoint_path
                    ? static_cast<int>(input.route.size()) - 1
                    : nodes[static_cast<std::size_t>(same_layer_opposite)].layer;
            result.node_count = nodes.size();
            result.top_extend_reject_reason =
                summarizeRejectReasons(extend_reasons);
            return result;
        }
    }

    result.node_count = nodes.size();
    result.top_extend_reject_reason = summarizeRejectReasons(extend_reasons);
    return result;
}

}  // namespace arm_controller::algorithm::global_planner
