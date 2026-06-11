#include "algorithm/cartesian_path_planner/base/kino_astar_base_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

namespace arm_controller::algorithm::cartesian_path_planner {

namespace {
constexpr double kEps = 1e-9;

struct QueueItem {
    int node_index{0};
    double f_score{0.0};
};

struct QueueCompare {
    bool operator()(const QueueItem& a, const QueueItem& b) const {
        return a.f_score > b.f_score;
    }
};

}  // namespace

struct KinoAstarBasePlanner::NodeKey {
    int ix{0};
    int iy{0};
    int iyaw{0};

    bool operator==(const NodeKey& other) const {
        return ix == other.ix && iy == other.iy && iyaw == other.iyaw;
    }
};

struct KinoAstarBasePlanner::Node {
    BaseState state;
    NodeKey key;
    int parent{-1};
    double g_score{std::numeric_limits<double>::infinity()};
    double f_score{std::numeric_limits<double>::infinity()};
    double steer{0.0};
    double signed_arc{0.0};
    double vx_body{0.0};
    double vy_body{0.0};
    double yaw_rate{0.0};
    double segment_time{0.0};
    int singularity{0};
    bool closed{false};
};

struct KinoAstarBasePlanner::NodeKeyHash {
    std::size_t operator()(const KinoAstarBasePlanner::NodeKey& key) const noexcept {
        std::size_t h = std::hash<int>{}(key.ix);
        h ^= std::hash<int>{}(key.iy + 0x9e3779b9 + (h << 6) + (h >> 2));
        h ^= std::hash<int>{}(key.iyaw + 0x9e3779b9 + (h << 6) + (h >> 2));
        return h;
    }
};

KinoAstarBasePlanner::KinoAstarBasePlanner()
    : KinoAstarBasePlanner(Config{}) {}

KinoAstarBasePlanner::KinoAstarBasePlanner(Config config)
    : config_(std::move(config)) {}

KinoAstarBasePlanner::Result KinoAstarBasePlanner::plan(
    const Input& input) const {
    Result result;
    if (!inBounds(input.start) || !inBounds(input.goal) ||
        !isStateValid(input, input.start) || !isStateValid(input, input.goal)) {
        return result;
    }

    std::vector<Node> nodes;
    nodes.reserve(static_cast<std::size_t>(std::max(16, config_.max_expansions)));
    std::unordered_map<NodeKey, int, NodeKeyHash> node_index_by_key;
    std::priority_queue<QueueItem, std::vector<QueueItem>, QueueCompare> open;

    Node start;
    start.state = input.start;
    start.state.yaw = normalizeAngle(start.state.yaw);
    start.key = keyFromState(start.state);
    start.g_score = 0.0;
    start.f_score = config_.heuristic_weight * heuristic(start.state, input.goal);
    start.steer = input.initial_steer;
    start.singularity = input.start_singularity != 0
                            ? input.start_singularity
                            : singularityFromVelocity(
                                  input.start_velocity,
                                  config_.non_singular_velocity);
    nodes.push_back(start);
    node_index_by_key[start.key] = 0;
    open.push(QueueItem{0, start.f_score});

    const std::vector<double> steer_values = {
        -config_.max_steer_angle,
        -0.5 * config_.max_steer_angle,
        0.0,
        0.5 * config_.max_steer_angle,
        config_.max_steer_angle};
    const std::vector<double> directions =
        config_.allow_reverse ? std::vector<double>{1.0, -1.0}
                              : std::vector<double>{1.0};
    const double omni_dt = std::max(0.05, config_.primitive_duration);
    const std::vector<Eigen::Vector2d> omni_directions = {
        {1.0, 0.0},
        {1.0, 1.0},
        {0.0, 1.0},
        {-1.0, 1.0},
        {-1.0, 0.0},
        {-1.0, -1.0},
        {0.0, -1.0},
        {1.0, -1.0},
    };

    int best_index = 0;
    double best_h = heuristic(start.state, input.goal);
    const auto start_time = std::chrono::steady_clock::now();
    const double max_solve_time_sec = std::max(0.0, config_.max_solve_time_sec);

    while (!open.empty() && result.expanded_nodes < config_.max_expansions) {
        if (max_solve_time_sec > 0.0) {
            const double elapsed =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start_time)
                    .count();
            if (elapsed >= max_solve_time_sec) {
                result.timed_out = true;
                result.solve_time_sec = elapsed;
                break;
            }
        }
        const QueueItem item = open.top();
        open.pop();
        if (item.node_index < 0 ||
            item.node_index >= static_cast<int>(nodes.size())) {
            continue;
        }
        Node& current = nodes[static_cast<std::size_t>(item.node_index)];
        if (current.closed) {
            continue;
        }
        current.closed = true;
        ++result.expanded_nodes;

        const double h = heuristic(current.state, input.goal);
        if (h < best_h) {
            best_h = h;
            best_index = item.node_index;
        }
        if (config_.oneshot_range > 0.0 &&
            std::hypot(current.state.x - input.goal.x,
                       current.state.y - input.goal.y) <= config_.oneshot_range) {
            BasePath shot_path;
            if (tryOneShot(input, current.state, input.goal, &shot_path)) {
                result.success = true;
                result.cost = current.g_score + heuristic(current.state, input.goal);
                reconstructPathAndMetadata(
                    nodes, item.node_index, current.state, result);
                if (!shot_path.empty() && result.path.size() >= 1u) {
                    const std::size_t old_size = result.path.size();
                    result.path.insert(
                        result.path.end(),
                        shot_path.begin() + 1,
                        shot_path.end());
                    for (std::size_t i = old_size; i < result.path.size(); ++i) {
                        const BaseState& from = result.path[i - 1u];
                        const BaseState& to = result.path[i];
                        const double dist = std::hypot(to.x - from.x, to.y - from.y);
                        result.segment_times.push_back(
                            std::max(1e-3, dist / std::max(1e-3, config_.max_velocity)));
                        const Eigen::Vector2d heading(
                            std::cos(from.yaw), std::sin(from.yaw));
                        const Eigen::Vector2d delta(to.x - from.x, to.y - from.y);
                        result.singularities.push_back(
                            heading.dot(delta) >= 0.0 ? 1 : -1);
                    }
                }
                populatePathMetadata(result);
                result.solve_time_sec =
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - start_time)
                        .count();
                return result;
            }
        }
        if (nearGoal(current.state, input.goal) &&
            isSegmentValid(input, current.state, input.goal)) {
            result.success = true;
            result.cost = current.g_score + heuristic(current.state, input.goal);
            reconstructPathAndMetadata(
                nodes, item.node_index, input.goal, result);
            populatePathMetadata(result);
            result.solve_time_sec =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start_time)
                    .count();
            return result;
        }

        if (config_.kinematic_model == KinematicModel::Omnidirectional) {
            std::vector<Eigen::Vector3d> controls;
            controls.reserve(10);
            for (const double speed_scale : {1.0, 0.5}) {
                for (const Eigen::Vector2d& direction : omni_directions) {
                    if (!config_.allow_reverse && direction.x() < -1e-6) {
                        continue;
                    }
                    Eigen::Vector2d normalized = direction;
                    if (normalized.norm() > 1.0) {
                        normalized.normalize();
                    }
                    controls.emplace_back(
                        speed_scale * normalized.x() * config_.max_velocity,
                        speed_scale * normalized.y() * config_.max_lateral_velocity,
                        0.0);
                }
            }
            const double yaw_error =
                angularDistance(current.state.yaw, input.goal.yaw);
            if (std::abs(yaw_error) > 0.5 * config_.yaw_resolution) {
                const double yaw_rate =
                    std::copysign(config_.max_yaw_rate, yaw_error);
                controls.emplace_back(0.0, 0.0, yaw_rate);
                controls.emplace_back(0.0, 0.0, -yaw_rate);
            }

            for (const Eigen::Vector3d& control : controls) {
                const double vx = control.x();
                const double vy = control.y();
                const double wz = control.z();
                BaseState next =
                    propagateOmni(current.state, vx, vy, wz, omni_dt);
                if (!inBounds(next)) {
                    continue;
                }
                if (input.segment_validator) {
                    if (!isSegmentValid(input, current.state, next)) {
                        continue;
                    }
                } else if (!isStateValid(input, next) ||
                           !isSegmentValid(input, current.state, next)) {
                    continue;
                }

                const NodeKey next_key = keyFromState(next);
                const double tentative_g =
                    current.g_score +
                    omniTransitionCost(
                        current.state, next, vx, vy, wz, omni_dt);

                auto index_it = node_index_by_key.find(next_key);
                if (index_it == node_index_by_key.end()) {
                    Node node;
                    node.state = next;
                    node.key = next_key;
                    node.parent = item.node_index;
                    node.g_score = tentative_g;
                    node.f_score = tentative_g +
                                   config_.heuristic_weight *
                                       heuristic(next, input.goal);
                    node.vx_body = vx;
                    node.vy_body = vy;
                    node.yaw_rate = wz;
                    node.segment_time = omni_dt;
                    node.singularity = singularityFromVelocity(
                        vx, config_.non_singular_velocity);
                    const int node_index = static_cast<int>(nodes.size());
                    nodes.push_back(std::move(node));
                    node_index_by_key[next_key] = node_index;
                    open.push(QueueItem{node_index, nodes.back().f_score});
                } else {
                    Node& existing =
                        nodes[static_cast<std::size_t>(index_it->second)];
                    if (!existing.closed &&
                        tentative_g + kEps < existing.g_score) {
                        existing.state = next;
                        existing.parent = item.node_index;
                        existing.g_score = tentative_g;
                        existing.f_score = tentative_g +
                                           config_.heuristic_weight *
                                               heuristic(next, input.goal);
                        existing.vx_body = vx;
                        existing.vy_body = vy;
                        existing.yaw_rate = wz;
                        existing.segment_time = omni_dt;
                        existing.singularity = singularityFromVelocity(
                            vx, config_.non_singular_velocity);
                        open.push(QueueItem{
                            index_it->second, existing.f_score});
                    }
                }
            }
            continue;
        }

        for (const double direction : directions) {
            const double signed_arc = direction * config_.primitive_arc_length;
            const int next_singularity = singularityFromArc(signed_arc);
            for (const double steer : steer_values) {
                if (std::abs(steer) > 0.9 * config_.max_steer_angle &&
                    std::abs(signed_arc) > 0.9 * config_.primitive_arc_length) {
                    continue;
                }
                BaseState next = propagate(current.state, steer, signed_arc);
                if (!inBounds(next)) {
                    continue;
                }
                if (input.segment_validator) {
                    if (!isSegmentValid(input, current.state, next)) {
                        continue;
                    }
                } else if (!isStateValid(input, next) ||
                           !isSegmentValid(input, current.state, next)) {
                    continue;
                }

                const NodeKey next_key = keyFromState(next);
                const double tentative_g =
                    current.g_score +
                    transitionCost(
                        current.state,
                        next,
                        steer,
                        current.steer,
                        signed_arc,
                        current.singularity);

                auto index_it = node_index_by_key.find(next_key);
                if (index_it == node_index_by_key.end()) {
                    Node node;
                    node.state = next;
                    node.key = next_key;
                    node.parent = item.node_index;
                    node.g_score = tentative_g;
                    node.f_score = tentative_g +
                                   config_.heuristic_weight *
                                       heuristic(next, input.goal);
                    node.steer = steer;
                    node.signed_arc = signed_arc;
                    node.segment_time =
                        std::max(1e-3,
                                 std::abs(signed_arc) /
                                     std::max(1e-3, config_.max_velocity));
                    node.singularity = next_singularity;
                    const int node_index = static_cast<int>(nodes.size());
                    nodes.push_back(std::move(node));
                    node_index_by_key[next_key] = node_index;
                    open.push(QueueItem{node_index, nodes.back().f_score});
                } else {
                    Node& existing =
                        nodes[static_cast<std::size_t>(index_it->second)];
                    if (!existing.closed && tentative_g + kEps < existing.g_score) {
                        existing.state = next;
                        existing.parent = item.node_index;
                        existing.g_score = tentative_g;
                        existing.f_score = tentative_g +
                                           config_.heuristic_weight *
                                               heuristic(next, input.goal);
                        existing.steer = steer;
                        existing.signed_arc = signed_arc;
                        existing.segment_time =
                            std::max(1e-3,
                                     std::abs(signed_arc) /
                                         std::max(1e-3, config_.max_velocity));
                        existing.singularity = next_singularity;
                        open.push(QueueItem{index_it->second, existing.f_score});
                    }
                }
            }
        }
    }

    reconstructPathAndMetadata(nodes, best_index, input.goal, result);
    result.cost = nodes.empty() ? 0.0 : nodes[static_cast<std::size_t>(best_index)].g_score;
    result.success = false;
    populatePathMetadata(result);
    if (result.solve_time_sec <= 0.0) {
        result.solve_time_sec =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time)
                .count();
    }
    return result;
}

bool KinoAstarBasePlanner::isStateValid(
    const Input& input,
    const BaseState& state) const {
    return !input.state_validator ||
           input.state_validator(state, config_.clearance);
}

bool KinoAstarBasePlanner::isSegmentValid(
    const Input& input,
    const BaseState& from,
    const BaseState& to) const {
    if (input.segment_validator) {
        return input.segment_validator(from, to, config_.clearance);
    }
    const double distance =
        std::hypot(to.x - from.x, to.y - from.y) +
        0.25 * std::abs(angularDistance(from.yaw, to.yaw));
    const int samples =
        std::max(2, static_cast<int>(
                        std::ceil(distance / std::max(0.02, config_.xy_resolution))));
    for (int i = 1; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        BaseState sample;
        sample.x = from.x + t * (to.x - from.x);
        sample.y = from.y + t * (to.y - from.y);
        sample.yaw =
            normalizeAngle(from.yaw + t * angularDistance(from.yaw, to.yaw));
        if (!isStateValid(input, sample)) {
            return false;
        }
    }
    return true;
}

bool KinoAstarBasePlanner::inBounds(const BaseState& state) const {
    return std::isfinite(state.x) && std::isfinite(state.y) &&
           std::isfinite(state.yaw) && state.x >= config_.map_min_x &&
           state.x <= config_.map_max_x && state.y >= config_.map_min_y &&
           state.y <= config_.map_max_y;
}

bool KinoAstarBasePlanner::nearGoal(
    const BaseState& state,
    const BaseState& goal) const {
    return std::hypot(state.x - goal.x, state.y - goal.y) <=
               config_.goal_xy_tolerance &&
           std::abs(angularDistance(state.yaw, goal.yaw)) <=
               config_.goal_yaw_tolerance;
}

double KinoAstarBasePlanner::heuristic(
    const BaseState& state,
    const BaseState& goal) const {
    return std::hypot(state.x - goal.x, state.y - goal.y) +
           config_.yaw_weight * std::abs(angularDistance(state.yaw, goal.yaw));
}

double KinoAstarBasePlanner::transitionCost(
    const BaseState& from,
    const BaseState& to,
    const double steer,
    const double previous_steer,
    const double signed_arc,
    const int previous_singularity) const {
    const double distance = std::max(
        std::abs(signed_arc),
        std::hypot(to.x - from.x, to.y - from.y));
    const int singularity = singularityFromArc(signed_arc);
    double cost = distance *
                  (singularity < 0
                       ? std::max(config_.backward_penalty, config_.reverse_penalty)
                       : config_.forward_penalty);
    if (singularity * previous_singularity < 0) {
        cost += config_.gear_switch_penalty;
    }
    cost += config_.steer_penalty * std::abs(steer) * distance;
    cost += config_.steer_change_penalty * std::abs(steer - previous_steer);
    cost += config_.yaw_weight * std::abs(angularDistance(from.yaw, to.yaw));
    return cost;
}

double KinoAstarBasePlanner::omniTransitionCost(
    const BaseState& from,
    const BaseState& to,
    const double vx_body,
    const double vy_body,
    const double yaw_rate,
    const double duration) const {
    const double planar_motion = std::hypot(to.x - from.x, to.y - from.y);
    const double yaw_motion = std::abs(angularDistance(from.yaw, to.yaw));
    const double lateral_ratio =
        std::abs(vy_body) / std::max(1e-3, config_.max_lateral_velocity);
    const double reverse_ratio = vx_body < -1e-6 ? 1.0 : 0.0;
    const double yaw_ratio =
        std::abs(yaw_rate) / std::max(1e-3, config_.max_yaw_rate);
    return std::max(1e-3, duration) +
           planar_motion +
           config_.yaw_weight * yaw_motion +
           0.03 * reverse_ratio +
           0.05 * lateral_ratio +
           0.03 * yaw_ratio;
}

KinoAstarBasePlanner::BaseState KinoAstarBasePlanner::propagate(
    const BaseState& state,
    const double steer,
    const double signed_arc) const {
    BaseState next = state;
    if (std::abs(steer) < 1e-5) {
        next.x += signed_arc * std::cos(state.yaw);
        next.y += signed_arc * std::sin(state.yaw);
        next.yaw = normalizeAngle(state.yaw);
        return next;
    }

    const double wheel_base = std::max(1e-4, config_.wheel_base);
    const double turning_radius = wheel_base / std::tan(steer);
    const double d_yaw = signed_arc / turning_radius;
    const double radius = turning_radius;
    next.x += radius * (std::sin(state.yaw + d_yaw) - std::sin(state.yaw));
    next.y += -radius * (std::cos(state.yaw + d_yaw) - std::cos(state.yaw));
    next.yaw = normalizeAngle(state.yaw + d_yaw);
    return next;
}

KinoAstarBasePlanner::BaseState KinoAstarBasePlanner::propagateOmni(
    const BaseState& state,
    const double vx_body,
    const double vy_body,
    const double yaw_rate,
    const double duration) const {
    BaseState next = state;
    const double dt = std::max(0.0, duration);
    const double c = std::cos(state.yaw);
    const double s = std::sin(state.yaw);
    next.x += (c * vx_body - s * vy_body) * dt;
    next.y += (s * vx_body + c * vy_body) * dt;
    next.yaw = normalizeAngle(state.yaw + yaw_rate * dt);
    return next;
}

KinoAstarBasePlanner::NodeKey KinoAstarBasePlanner::keyFromState(
    const BaseState& state) const {
    NodeKey key;
    key.ix = static_cast<int>(
        std::floor((state.x - config_.map_min_x) / config_.xy_resolution));
    key.iy = static_cast<int>(
        std::floor((state.y - config_.map_min_y) / config_.xy_resolution));
    const double yaw = normalizeAngle(state.yaw);
    key.iyaw = static_cast<int>(
        std::floor((yaw + M_PI) / config_.yaw_resolution));
    return key;
}

KinoAstarBasePlanner::BasePath KinoAstarBasePlanner::reconstructPath(
    const std::vector<Node>& nodes,
    const int node_index,
    const BaseState& goal) const {
    BasePath reversed;
    int index = node_index;
    while (index >= 0 && index < static_cast<int>(nodes.size())) {
        reversed.push_back(nodes[static_cast<std::size_t>(index)].state);
        index = nodes[static_cast<std::size_t>(index)].parent;
    }
    std::reverse(reversed.begin(), reversed.end());
    if (!reversed.empty()) {
        const BaseState& back = reversed.back();
        if (std::hypot(back.x - goal.x, back.y - goal.y) >
                config_.goal_xy_tolerance ||
            std::abs(angularDistance(back.yaw, goal.yaw)) >
                config_.goal_yaw_tolerance) {
            reversed.push_back(goal);
        } else {
            reversed.back() = goal;
        }
    }
    return reversed;
}

void KinoAstarBasePlanner::reconstructPathAndMetadata(
    const std::vector<Node>& nodes,
    const int node_index,
    const BaseState& goal,
    Result& result) const {
    result.path.clear();
    result.dense_check_path.clear();
    result.segment_times.clear();
    result.singularities.clear();

    std::vector<int> reversed_indices;
    int index = node_index;
    while (index >= 0 && index < static_cast<int>(nodes.size())) {
        reversed_indices.push_back(index);
        index = nodes[static_cast<std::size_t>(index)].parent;
    }
    std::reverse(reversed_indices.begin(), reversed_indices.end());
    result.path.reserve(reversed_indices.size() + 1u);
    for (const int node_idx : reversed_indices) {
        result.path.push_back(nodes[static_cast<std::size_t>(node_idx)].state);
    }

    if (result.path.empty()) {
        return;
    }

    const BaseState& back = result.path.back();
    const bool append_goal =
        std::hypot(back.x - goal.x, back.y - goal.y) >
            config_.goal_xy_tolerance ||
        std::abs(angularDistance(back.yaw, goal.yaw)) >
            config_.goal_yaw_tolerance;
    if (append_goal) {
        result.path.push_back(goal);
    } else {
        result.path.back() = goal;
    }

    result.segment_times.reserve(result.path.size() - 1u);
    result.singularities.reserve(result.path.size() - 1u);
    for (std::size_t i = 1; i < reversed_indices.size(); ++i) {
        const Node& node =
            nodes[static_cast<std::size_t>(reversed_indices[i])];
        result.segment_times.push_back(
            std::max(1e-3, node.segment_time));
        result.singularities.push_back(node.singularity == 0 ? 1 : node.singularity);
    }
    if (append_goal && result.path.size() >= 2u) {
        const BaseState& from = result.path[result.path.size() - 2u];
        const BaseState& to = result.path.back();
        const double planar_motion = std::hypot(to.x - from.x, to.y - from.y);
        const double yaw_motion = std::abs(angularDistance(from.yaw, to.yaw));
        result.segment_times.push_back(
            std::max(
                1e-3,
                std::max(
                    planar_motion / std::max(1e-3, config_.max_velocity),
                    yaw_motion / std::max(1e-3, config_.max_yaw_rate))));
        const Eigen::Vector2d heading(std::cos(from.yaw), std::sin(from.yaw));
        const Eigen::Vector2d delta(to.x - from.x, to.y - from.y);
        result.singularities.push_back(heading.dot(delta) >= 0.0 ? 1 : -1);
    }
}

bool KinoAstarBasePlanner::tryOneShot(
    const Input& input,
    const BaseState& from,
    const BaseState& goal,
    BasePath* shot_path) const {
    if (shot_path == nullptr || config_.reeds_shepp_turning_radii.empty()) {
        return false;
    }

    namespace ob = ompl::base;
    for (const double radius : config_.reeds_shepp_turning_radii) {
        if (!(radius > 1.0e-6) || !std::isfinite(radius)) {
            continue;
        }
        auto space = std::make_shared<ob::ReedsSheppStateSpace>(radius);
        ob::ScopedState<> s_from(space);
        ob::ScopedState<> s_goal(space);
        ob::ScopedState<> s(space);
        s_from[0] = from.x;
        s_from[1] = from.y;
        s_from[2] = from.yaw;
        s_goal[0] = goal.x;
        s_goal[1] = goal.y;
        s_goal[2] = goal.yaw;
        const double length = space->distance(s_from.get(), s_goal.get());
        if (!(length > 1.0e-9) || !std::isfinite(length)) {
            continue;
        }
        const int samples =
            std::max(2, static_cast<int>(
                            std::ceil(length / std::max(0.02, config_.oneshot_check_len))));
        BasePath candidate;
        candidate.reserve(static_cast<std::size_t>(samples + 1));
        bool valid = true;
        for (int i = 0; i <= samples; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(samples);
            space->interpolate(s_from.get(), s_goal.get(), ratio, s.get());
            const auto* se2 = s.get()->as<ob::SE2StateSpace::StateType>();
            BaseState state;
            state.x = se2->getX();
            state.y = se2->getY();
            state.yaw = normalizeAngle(se2->getYaw());
            if (!inBounds(state) || !isStateValid(input, state)) {
                valid = false;
                break;
            }
            if (!candidate.empty() &&
                !isSegmentValid(input, candidate.back(), state)) {
                valid = false;
                break;
            }
            candidate.push_back(state);
        }
        if (valid && candidate.size() >= 2) {
            *shot_path = std::move(candidate);
            return true;
        }
    }
    return false;
}

void KinoAstarBasePlanner::populatePathMetadata(Result& result) const {
    result.dense_check_path.clear();
    if (result.path.empty()) {
        return;
    }

    const bool has_dynamic_times =
        result.segment_times.size() + 1u == result.path.size();
    const bool has_singularities =
        result.singularities.size() + 1u == result.path.size();
    if (!has_dynamic_times) {
        result.segment_times.clear();
        result.segment_times.reserve(result.path.size() - 1u);
    }
    if (!has_singularities) {
        result.singularities.clear();
        result.singularities.reserve(result.path.size() - 1u);
    }

    result.dense_check_path.push_back(result.path.front());
    for (std::size_t i = 1; i < result.path.size(); ++i) {
        const BaseState& from = result.path[i - 1];
        const BaseState& to = result.path[i];
        const double distance = std::hypot(to.x - from.x, to.y - from.y);
        const double yaw_distance = std::abs(angularDistance(from.yaw, to.yaw));
        const double length = distance + config_.yaw_weight * yaw_distance;
        const double max_velocity = std::max(
            1e-3,
            std::max(config_.nominal_speed, config_.max_velocity));
        const double max_acceleration = std::max(1e-3, config_.max_acceleration);
        const double accel_time = max_velocity / max_acceleration;
        const double accel_distance = max_velocity * accel_time;
        const double duration =
            length <= accel_distance
                ? std::max(1e-3, 2.0 * std::sqrt(length / max_acceleration))
                : std::max(1e-3, length / max_velocity + accel_time);
        if (!has_dynamic_times) {
            result.segment_times.push_back(duration);
        }

        const Eigen::Vector2d heading(std::cos(from.yaw), std::sin(from.yaw));
        const Eigen::Vector2d delta(to.x - from.x, to.y - from.y);
        const int singularity = heading.dot(delta) >= 0.0 ? 1 : -1;
        if (!has_singularities) {
            result.singularities.push_back(singularity);
        }

        const int samples = std::max(
            1,
            static_cast<int>(std::ceil(
                (distance + 0.25 * yaw_distance) /
                std::max(0.01, config_.dense_check_resolution))));
        for (int j = 1; j <= samples; ++j) {
            const double t = static_cast<double>(j) /
                             static_cast<double>(samples);
            BaseState sample;
            sample.x = from.x + t * (to.x - from.x);
            sample.y = from.y + t * (to.y - from.y);
            sample.yaw = normalizeAngle(
                from.yaw + t * angularDistance(from.yaw, to.yaw));
            result.dense_check_path.push_back(sample);
        }
    }
}

int KinoAstarBasePlanner::singularityFromArc(const double signed_arc) {
    if (signed_arc > 1e-6) {
        return 1;
    }
    if (signed_arc < -1e-6) {
        return -1;
    }
    return 0;
}

int KinoAstarBasePlanner::singularityFromVelocity(
    const double velocity,
    const double threshold) {
    const double eps = std::max(0.0, threshold);
    if (std::abs(velocity) <= eps) {
        return 0;
    }
    return velocity >= 0.0 ? 1 : -1;
}

double KinoAstarBasePlanner::normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double KinoAstarBasePlanner::angularDistance(
    const double from,
    const double to) {
    return normalizeAngle(to - from);
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
