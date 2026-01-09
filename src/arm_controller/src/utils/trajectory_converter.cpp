#include "arm_controller/utils/trajectory_converter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace arm_controller::utils {

trajectory_interpolator::Trajectory TrajectoryConverter::convertPlanningToInterpolator(
    const trajectory_planning::domain::entities::Trajectory& planning_trajectory,
    const std::vector<std::string>& joint_names) {

    trajectory_interpolator::Trajectory interpolator_trajectory;
    interpolator_trajectory.joint_names = joint_names;

    for (const auto& planning_point : planning_trajectory.points()) {
        trajectory_interpolator::TrajectoryPoint interpolator_point;

        // 转换时间
        interpolator_point.time_from_start = planning_point.time_from_start.seconds();

        // 转换位置：弧度 → 度数 (轨迹插值器期望度数)
        const auto& positions_rad = planning_point.position.values();
        interpolator_point.positions.reserve(positions_rad.size());
        for (double pos_rad : positions_rad) {
            interpolator_point.positions.push_back(pos_rad * 180.0 / M_PI);
        }

        // 转换速度：弧度/秒 → 度/秒
        const auto& velocities_rad = planning_point.velocity.values();
        interpolator_point.velocities.reserve(velocities_rad.size());
        for (double vel_rad : velocities_rad) {
            interpolator_point.velocities.push_back(vel_rad * 180.0 / M_PI);
        }

        // 转换加速度：弧度/秒² → 度/秒²
        const auto& accelerations_rad = planning_point.acceleration.values();
        interpolator_point.accelerations.reserve(accelerations_rad.size());
        for (double acc_rad : accelerations_rad) {
            interpolator_point.accelerations.push_back(acc_rad * 180.0 / M_PI);
        }

        interpolator_trajectory.points.push_back(interpolator_point);
    }

    return interpolator_trajectory;
}

trajectory_msgs::msg::JointTrajectory TrajectoryConverter::convertInterpolatorToRos(
    const trajectory_interpolator::Trajectory& interpolator_trajectory) {

    trajectory_msgs::msg::JointTrajectory ros_trajectory;
    ros_trajectory.joint_names = interpolator_trajectory.joint_names;

    for (const auto& interpolator_point : interpolator_trajectory.points) {
        trajectory_msgs::msg::JointTrajectoryPoint ros_point;

        ros_point.positions = interpolator_point.positions;
        ros_point.velocities = interpolator_point.velocities;
        ros_point.accelerations = interpolator_point.accelerations;
        ros_point.time_from_start = rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(interpolator_point.time_from_start * 1e9));

        ros_trajectory.points.push_back(ros_point);
    }

    return ros_trajectory;
}

trajectory_msgs::msg::JointTrajectory TrajectoryConverter::convertPlanningToRos(
    const trajectory_planning::domain::entities::Trajectory& planning_trajectory,
    const std::vector<std::string>& joint_names) {

    trajectory_msgs::msg::JointTrajectory ros_trajectory;
    ros_trajectory.joint_names = joint_names;

    for (const auto& planning_point : planning_trajectory.points()) {
        trajectory_msgs::msg::JointTrajectoryPoint ros_point;

        ros_point.positions = planning_point.position.values();
        ros_point.velocities = planning_point.velocity.values();
        ros_point.accelerations = planning_point.acceleration.values();
        ros_point.time_from_start = rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(planning_point.time_from_start.seconds() * 1e9));

        ros_trajectory.points.push_back(ros_point);
    }

    return ros_trajectory;
}

trajectory_interpolator::Trajectory TrajectoryConverter::convertRosToInterpolator(
    const trajectory_msgs::msg::JointTrajectory& ros_trajectory) {

    trajectory_interpolator::Trajectory interpolator_trajectory;
    interpolator_trajectory.joint_names = ros_trajectory.joint_names;

    for (size_t point_idx = 0; point_idx < ros_trajectory.points.size(); ++point_idx) {
        const auto& ros_point = ros_trajectory.points[point_idx];
        trajectory_interpolator::TrajectoryPoint interpolator_point;

        interpolator_point.time_from_start = ros_point.time_from_start.sec +
                                           ros_point.time_from_start.nanosec * 1e-9;

        // 转换位置：弧度 → 度数 (避免浮点数误差累积)
        interpolator_point.positions.reserve(ros_point.positions.size());
        for (double pos_rad : ros_point.positions) {
            interpolator_point.positions.push_back(pos_rad * 180.0 / M_PI);
        }

        // 转换速度：弧度/秒 → 度/秒
        interpolator_point.velocities.reserve(ros_point.velocities.size());
        for (double vel_rad : ros_point.velocities) {
            interpolator_point.velocities.push_back(vel_rad * 180.0 / M_PI);
        }

        // 转换加速度：弧度/秒² → 度/秒²
        interpolator_point.accelerations.reserve(ros_point.accelerations.size());
        for (double acc_rad : ros_point.accelerations) {
            interpolator_point.accelerations.push_back(acc_rad * 180.0 / M_PI);
        }

        interpolator_trajectory.points.push_back(interpolator_point);
    }

    return interpolator_trajectory;
}

TrajectoryConverter::TrajectoryDynamics TrajectoryConverter::analyzeTrajectoryDynamics(
    const trajectory_planning::domain::entities::Trajectory& trajectory) {

    TrajectoryDynamics dynamics;

    for (const auto& point : trajectory.points()) {
        // 分析速度
        const auto& vel_values = point.velocity.values();
        for (size_t i = 0; i < vel_values.size(); ++i) {
            dynamics.max_velocity = std::max(dynamics.max_velocity, std::abs(vel_values[i]));
        }

        // 分析加速度
        const auto& acc_values = point.acceleration.values();
        for (size_t i = 0; i < acc_values.size(); ++i) {
            dynamics.max_acceleration = std::max(dynamics.max_acceleration, std::abs(acc_values[i]));
        }
    }

    // 估算最大加加速度（基于加速度变化率）
    if (trajectory.points().size() > 1) {
        double max_jerk = 0.0;
        for (size_t i = 1; i < trajectory.points().size(); ++i) {
            const auto& prev_acc = trajectory.points()[i-1].acceleration.values();
            const auto& curr_acc = trajectory.points()[i].acceleration.values();
            double dt = trajectory.points()[i].time_from_start.seconds() -
                       trajectory.points()[i-1].time_from_start.seconds();

            if (dt > 0.0) {
                for (size_t j = 0; j < prev_acc.size() && j < curr_acc.size(); ++j) {
                    double jerk = std::abs(curr_acc[j] - prev_acc[j]) / dt;
                    max_jerk = std::max(max_jerk, jerk);
                }
            }
        }
        dynamics.max_jerk = max_jerk;
    }

    return dynamics;
}

TrajectoryConverter::TrajectoryDynamics TrajectoryConverter::analyzeTrajectoryDynamics(
    const trajectory_msgs::msg::JointTrajectory& trajectory) {

    TrajectoryDynamics dynamics;

    for (const auto& point : trajectory.points) {
        // 分析速度
        for (double vel : point.velocities) {
            dynamics.max_velocity = std::max(dynamics.max_velocity, std::abs(vel));
        }

        // 分析加速度
        for (double acc : point.accelerations) {
            dynamics.max_acceleration = std::max(dynamics.max_acceleration, std::abs(acc));
        }
    }

    // 估算最大加加速度（基于加速度变化率）
    if (trajectory.points.size() > 1) {
        double max_jerk = 0.0;
        for (size_t i = 1; i < trajectory.points.size(); ++i) {
            const auto& prev_acc = trajectory.points[i-1].accelerations;
            const auto& curr_acc = trajectory.points[i].accelerations;

            double prev_time = trajectory.points[i-1].time_from_start.sec +
                              trajectory.points[i-1].time_from_start.nanosec * 1e-9;
            double curr_time = trajectory.points[i].time_from_start.sec +
                              trajectory.points[i].time_from_start.nanosec * 1e-9;
            double dt = curr_time - prev_time;

            if (dt > 0.0) {
                for (size_t j = 0; j < prev_acc.size() && j < curr_acc.size(); ++j) {
                    double jerk = std::abs(curr_acc[j] - prev_acc[j]) / dt;
                    max_jerk = std::max(max_jerk, jerk);
                }
            }
        }
        dynamics.max_jerk = max_jerk;
    }

    return dynamics;
}

TrajectoryConverter::TrajectoryDynamics TrajectoryConverter::calculateSafeInterpolationParams(
    const TrajectoryDynamics& dynamics,
    double safety_margin,
    double hardware_velocity_limit) {

    TrajectoryDynamics safe_params;

    // 如果轨迹已经有速度/加速度信息，直接使用该信息
    if (dynamics.max_velocity > 0.001) {
        // 轨迹有动力学信息，直接使用
        safe_params.max_velocity = dynamics.max_velocity * safety_margin;
        safe_params.max_acceleration = dynamics.max_acceleration * safety_margin;
        safe_params.max_jerk = dynamics.max_jerk * safety_margin;

        // 应用硬件限制
        safe_params.max_velocity = std::min(safe_params.max_velocity, hardware_velocity_limit);
    } else {
        // 轨迹没有动力学信息（如 MoveIt Cartesian 规划），使用最小的温和参数
        // 这样插值器会生成合理数量的点，避免过度插值
        safe_params.max_velocity = 0.2;       // 非常温和的速度
        safe_params.max_acceleration = 0.3;   // 非常温和的加速度
        safe_params.max_jerk = 0.5;           // 非常温和的加加速度
    }

    return safe_params;
}

Trajectory TrajectoryConverter::convertInterpolatorToHardwareDriver(
    const trajectory_interpolator::Trajectory& interpolator_trajectory) {

    Trajectory hw_trajectory;
    hw_trajectory.joint_names = interpolator_trajectory.joint_names;

    for (const auto& interpolator_point : interpolator_trajectory.points) {
        TrajectoryPoint hw_point;

        hw_point.time_from_start = interpolator_point.time_from_start;
        hw_point.positions = interpolator_point.positions;
        hw_point.velocities = interpolator_point.velocities;
        hw_point.accelerations = interpolator_point.accelerations;

        hw_trajectory.points.push_back(hw_point);
    }

    return hw_trajectory;
}

} // namespace arm_controller::utils