#include "trajectory_smoother.hpp"
#include <csaps.h>

// ============= CSAPSSmoother 实现 =============

CSAPSSmoother::CSAPSSmoother(rclcpp::Node::SharedPtr node)
    : node_(node) {}

trajectory_interpolator::Trajectory CSAPSSmoother::smooth(
    const std::vector<double>& times,
    const std::vector<std::vector<double>>& positions,
    const std::vector<std::vector<double>>& velocities,
    const std::vector<std::vector<double>>& efforts,
    const std::vector<std::string>& joint_names) {

    trajectory_interpolator::Trajectory result;
    result.joint_names = joint_names;

    if (positions.empty()) {
        RCLCPP_WARN(node_->get_logger(), "❎ CSAPS: Empty trajectory");
        return result;
    }

    int num_joints = joint_names.size();

    RCLCPP_INFO(node_->get_logger(), "CSAPS smoothing: %zu points, smoothing_factor=%.2f",
                positions.size(), smoothing_factor_);

    try {
        // 转换数据格式
        csaps::DoubleArray2D pos_data(positions.size(), num_joints);
        csaps::DoubleArray2D vel_data(velocities.size(), num_joints);
        csaps::DoubleArray2D acc_data(efforts.size(), num_joints);

        for (size_t i = 0; i < positions.size(); ++i) {
            for (int j = 0; j < num_joints && j < 6; ++j) {
                pos_data(i, j) = positions[i][j];
                vel_data(i, j) = velocities[i][j];
                acc_data(i, j) = efforts[i][j];
            }
        }

        // 转换时间数组
        csaps::DoubleArray x(times.size());
        for (size_t i = 0; i < times.size(); ++i) {
            x(i) = times[i];
        }

        // 创建样条曲线并平滑
        csaps::MultivariateCubicSmoothingSpline pos_spline(x, pos_data, smoothing_factor_);
        auto smoothed_positions = pos_spline(x);

        csaps::MultivariateCubicSmoothingSpline vel_spline(x, vel_data, smoothing_factor_);
        auto smoothed_velocities = vel_spline(x);

        csaps::MultivariateCubicSmoothingSpline acc_spline(x, acc_data, smoothing_factor_);
        auto smoothed_accelerations = acc_spline(x);

        // 构建轨迹点
        for (size_t i = 0; i < times.size(); ++i) {
            trajectory_interpolator::TrajectoryPoint point;
            point.time_from_start = times[i];

            for (int j = 0; j < num_joints && j < 6; ++j) {
                point.positions.push_back(smoothed_positions(i, j));

                // 首末点速度为0
                if (i == 0 || i == times.size() - 1) {
                    point.velocities.push_back(0.0);
                } else {
                    point.velocities.push_back(smoothed_velocities(i, j));
                }

                point.accelerations.push_back(smoothed_accelerations(i, j));
            }

            result.points.push_back(point);
        }

        RCLCPP_INFO(node_->get_logger(), "✅ CSAPS smoothing completed: %zu points", result.points.size());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ CSAPS smoothing failed: %s", e.what());
        return RawSmoother(node_).smooth(times, positions, velocities, efforts, joint_names);
    }

    return result;
}

// ============= RawSmoother 实现 =============

RawSmoother::RawSmoother(rclcpp::Node::SharedPtr node)
    : node_(node) {}

trajectory_interpolator::Trajectory RawSmoother::smooth(
    const std::vector<double>& times,
    const std::vector<std::vector<double>>& positions,
    const std::vector<std::vector<double>>& velocities,
    const std::vector<std::vector<double>>& efforts,
    const std::vector<std::string>& joint_names) {

    trajectory_interpolator::Trajectory result;
    result.joint_names = joint_names;

    RCLCPP_INFO(node_->get_logger(), "Using raw trajectory: %zu points (no smoothing)", positions.size());

    for (size_t i = 0; i < times.size(); ++i) {
        auto point = build_trajectory_point(i, times, positions, velocities, efforts);
        result.points.push_back(point);
    }

    return result;
}

trajectory_interpolator::TrajectoryPoint RawSmoother::build_trajectory_point(
    size_t idx,
    const std::vector<double>& times,
    const std::vector<std::vector<double>>& positions,
    const std::vector<std::vector<double>>& velocities,
    const std::vector<std::vector<double>>& efforts) {

    trajectory_interpolator::TrajectoryPoint point;
    point.time_from_start = times[idx];

    int num_joints = positions[idx].size();
    for (int j = 0; j < num_joints && j < 6; ++j) {
        point.positions.push_back(positions[idx][j]);

        // 首末点速度为0
        if (idx == 0 || idx == times.size() - 1) {
            point.velocities.push_back(0.0);
        } else {
            point.velocities.push_back(velocities[idx][j]);
        }

        point.accelerations.push_back(efforts[idx][j]);
    }

    return point;
}

// ============= TrajectorySmoother 实现 =============

TrajectorySmoother::TrajectorySmoother(rclcpp::Node::SharedPtr node)
    : node_(node) {
    csaps_smoother_ = std::make_shared<CSAPSSmoother>(node);
    raw_smoother_ = std::make_shared<RawSmoother>(node);
}

trajectory_interpolator::Trajectory TrajectorySmoother::smooth(
    const std::vector<double>& times,
    const std::vector<std::vector<double>>& positions,
    const std::vector<std::vector<double>>& velocities,
    const std::vector<std::vector<double>>& efforts,
    const std::vector<std::string>& joint_names,
    bool use_csaps,
    bool skip_large_datasets) {

    // 检查是否应该跳过大数据集的平滑
    if (skip_large_datasets && positions.size() > large_dataset_threshold_) {
        RCLCPP_INFO(node_->get_logger(), "Large dataset (%zu points): Skipping smoothing for performance",
                    positions.size());
        return raw_smoother_->smooth(times, positions, velocities, efforts, joint_names);
    }

    // 选择平滑策略
    if (use_csaps) {
        return csaps_smoother_->smooth(times, positions, velocities, efforts, joint_names);
    } else {
        return raw_smoother_->smooth(times, positions, velocities, efforts, joint_names);
    }
}
