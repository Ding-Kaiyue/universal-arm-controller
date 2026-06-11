#pragma once

#include <Eigen/Core>

#include <functional>
#include <vector>

namespace arm_controller::algorithm::cartesian_path_planner {

class KinoAstarBasePlanner {
public:
    enum class KinematicModel {
        Bicycle,
        Omnidirectional,
    };

    struct BaseState {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    using BasePath = std::vector<BaseState>;
    using StateValidator =
        std::function<bool(const BaseState& state, double clearance)>;
    using SegmentValidator =
        std::function<bool(const BaseState& from,
                           const BaseState& to,
                           double clearance)>;

    struct Config {
        KinematicModel kinematic_model{KinematicModel::Bicycle};
        double xy_resolution{0.10};
        double yaw_resolution{0.35};
        double primitive_arc_length{0.18};
        double primitive_duration{0.35};
        double max_steer_angle{0.70};
        double wheel_base{0.40};
        double goal_xy_tolerance{0.12};
        double goal_yaw_tolerance{0.30};
        double map_min_x{-5.0};
        double map_max_x{5.0};
        double map_min_y{-5.0};
        double map_max_y{5.0};
        double clearance{0.05};
        double forward_penalty{1.0};
        double backward_penalty{1.0};
        double reverse_penalty{1.80};
        double gear_switch_penalty{15.0};
        double steer_penalty{0.08};
        double steer_change_penalty{0.05};
        double yaw_weight{0.20};
        double heuristic_weight{1.20};
        double nominal_speed{0.20};
        double max_velocity{0.50};
        double max_lateral_velocity{0.50};
        double max_yaw_rate{0.80};
        double max_acceleration{0.50};
        double non_singular_velocity{0.01};
        double dense_check_resolution{0.05};
        int max_expansions{12000};
        double max_solve_time_sec{2.0};
        double oneshot_range{5.0};
        double oneshot_check_len{0.20};
        std::vector<double> reeds_shepp_turning_radii{0.4, 0.2, 0.1};
        bool allow_reverse{true};
    };

    struct Input {
        BaseState start;
        BaseState goal;
        double start_velocity{0.0};
        int start_singularity{0};
        double initial_steer{0.0};
        StateValidator state_validator;
        SegmentValidator segment_validator;
    };

    struct Result {
        bool success{false};
        BasePath path;
        BasePath dense_check_path;
        std::vector<double> segment_times;
        std::vector<int> singularities;
        int expanded_nodes{0};
        double cost{0.0};
        double solve_time_sec{0.0};
        bool timed_out{false};
    };

    KinoAstarBasePlanner();
    explicit KinoAstarBasePlanner(Config config);

    Result plan(const Input& input) const;

    const Config& config() const { return config_; }

private:
    struct Node;
    struct NodeKey;
    struct NodeKeyHash;

    bool isStateValid(const Input& input, const BaseState& state) const;
    bool isSegmentValid(
        const Input& input,
        const BaseState& from,
        const BaseState& to) const;
    bool inBounds(const BaseState& state) const;
    bool nearGoal(const BaseState& state, const BaseState& goal) const;
    double heuristic(const BaseState& state, const BaseState& goal) const;
    double transitionCost(
        const BaseState& from,
        const BaseState& to,
        double steer,
        double previous_steer,
        double signed_arc,
        int previous_singularity) const;
    double omniTransitionCost(
        const BaseState& from,
        const BaseState& to,
        double vx_body,
        double vy_body,
        double yaw_rate,
        double duration) const;
    BaseState propagate(
        const BaseState& state,
        double steer,
        double signed_arc) const;
    BaseState propagateOmni(
        const BaseState& state,
        double vx_body,
        double vy_body,
        double yaw_rate,
        double duration) const;
    static int singularityFromArc(double signed_arc);
    static int singularityFromVelocity(double velocity, double threshold);
    NodeKey keyFromState(const BaseState& state) const;
    BasePath reconstructPath(
        const std::vector<Node>& nodes,
        int node_index,
        const BaseState& goal) const;
    void reconstructPathAndMetadata(
        const std::vector<Node>& nodes,
        int node_index,
        const BaseState& goal,
        Result& result) const;
    bool tryOneShot(
        const Input& input,
        const BaseState& from,
        const BaseState& goal,
        BasePath* shot_path) const;
    void populatePathMetadata(Result& result) const;

    static double normalizeAngle(double angle);
    static double angularDistance(double from, double to);

    Config config_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
