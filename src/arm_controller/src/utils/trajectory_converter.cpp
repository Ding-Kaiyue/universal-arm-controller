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
        auto positions_rad = planning_point.position.values();
        interpolator_point.positions.reserve(positions_rad.size());
        for (double pos_rad : positions_rad) {
            interpolator_point.positions.push_back(pos_rad * 180.0 / M_PI);
        }

        // 转换速度：弧度/秒 → 度/秒
        auto velocities_rad = planning_point.velocity.values();
        interpolator_point.velocities.reserve(velocities_rad.size());
        for (double vel_rad : velocities_rad) {
            interpolator_point.velocities.push_back(vel_rad * 180.0 / M_PI);
        }

        // 转换加速度：弧度/秒² → 度/秒²
        auto accelerations_rad = planning_point.acceleration.values();
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
        const auto& velocities = point.velocity.values();
        for (double vel : velocities) {
            dynamics.max_velocity = std::max(dynamics.max_velocity, std::abs(vel));
        }

        // 分析加速度
        const auto& accelerations = point.acceleration.values();
        for (double acc : accelerations) {
            dynamics.max_acceleration = std::max(dynamics.max_acceleration, std::abs(acc));
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

    // 如果轨迹动力学参数为0或过小，使用合理的默认值
    double base_velocity = std::max(dynamics.max_velocity, 1.0);  // 至少1.0 rad/s
    double base_acceleration = std::max(dynamics.max_acceleration, 2.0);  // 至少2.0 rad/s²
    double base_jerk = std::max(dynamics.max_jerk, 4.0);  // 至少4.0 rad/s³

    // 应用安全余量（保持原有逻辑，但提高基础值）
    safe_params.max_velocity = base_velocity * safety_margin;
    safe_params.max_acceleration = base_acceleration * safety_margin;
    safe_params.max_jerk = base_jerk * safety_margin;

    // 应用硬件限制
    safe_params.max_velocity = std::min(safe_params.max_velocity, hardware_velocity_limit);

    // 确保最小值合理（不要过慢）
    safe_params.max_velocity = std::max(safe_params.max_velocity, 0.8);  // 最低0.8 rad/s
    safe_params.max_acceleration = std::max(safe_params.max_acceleration, 1.5);  // 最低1.5 rad/s²
    safe_params.max_jerk = std::max(safe_params.max_jerk, 3.0);  // 最低3.0 rad/s³

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