#ifndef __TRAJECTORY_CONVERTER_HPP__
#define __TRAJECTORY_CONVERTER_HPP__

#include "trajectory_planning_v3/domain/entities/trajectory.hpp"
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "hardware_driver/interface/robot_hardware.hpp"
#include <vector>
#include <string>

namespace arm_controller::utils {

/**
 * @brief 轨迹数据类型转换工具类
 * 提供各种轨迹数据格式之间的转换功能
 */
class TrajectoryConverter {
public:
    /**
     * @brief 将trajectory_planning的轨迹转换为trajectory_interpolator的轨迹格式
     * @param planning_trajectory trajectory_planning的轨迹
     * @param joint_names 关节名称列表
     * @return trajectory_interpolator的轨迹格式
     */
    static trajectory_interpolator::Trajectory convertPlanningToInterpolator(
        const trajectory_planning::domain::entities::Trajectory& planning_trajectory,
        const std::vector<std::string>& joint_names);

    /**
     * @brief 将trajectory_interpolator的轨迹转换为ROS轨迹消息
     * @param interpolator_trajectory trajectory_interpolator的轨迹
     * @return ROS轨迹消息
     */
    static trajectory_msgs::msg::JointTrajectory convertInterpolatorToRos(
        const trajectory_interpolator::Trajectory& interpolator_trajectory);

    /**
     * @brief 将trajectory_planning的轨迹直接转换为ROS轨迹消息
     * @param planning_trajectory trajectory_planning的轨迹
     * @param joint_names 关节名称列表
     * @return ROS轨迹消息
     */
    static trajectory_msgs::msg::JointTrajectory convertPlanningToRos(
        const trajectory_planning::domain::entities::Trajectory& planning_trajectory,
        const std::vector<std::string>& joint_names);

    /**
     * @brief 将ROS轨迹消息转换为trajectory_interpolator的轨迹格式
     * @param ros_trajectory ROS轨迹消息
     * @return trajectory_interpolator的轨迹格式
     */
    static trajectory_interpolator::Trajectory convertRosToInterpolator(
        const trajectory_msgs::msg::JointTrajectory& ros_trajectory);

    /**
     * @brief 将trajectory_interpolator的轨迹转换为hardware_driver的轨迹格式
     * @param interpolator_trajectory trajectory_interpolator的轨迹
     * @return hardware_driver的轨迹格式
     */
    static Trajectory convertInterpolatorToHardwareDriver(
        const trajectory_interpolator::Trajectory& interpolator_trajectory);

    /**
     * @brief 分析轨迹的动力学参数
     * @param trajectory trajectory_planning的轨迹
     * @return 包含最大速度、加速度的结构体
     */
    struct TrajectoryDynamics {
        double max_velocity = 0.0;
        double max_acceleration = 0.0;
        double max_jerk = 0.0;
    };

    static TrajectoryDynamics analyzeTrajectoryDynamics(
        const trajectory_planning::domain::entities::Trajectory& trajectory);

    /**
     * @brief 分析ROS轨迹的动力学参数
     * @param trajectory ROS轨迹消息
     * @return 包含最大速度、加速度的结构体
     */
    static TrajectoryDynamics analyzeTrajectoryDynamics(
        const trajectory_msgs::msg::JointTrajectory& trajectory);

    /**
     * @brief 基于轨迹动力学计算安全的插值参数
     * @param dynamics 轨迹动力学分析结果
     * @param safety_margin 安全余量系数（默认1.2）
     * @param hardware_velocity_limit 硬件速度限制
     * @return 安全的插值参数
     */
    static TrajectoryDynamics calculateSafeInterpolationParams(
        const TrajectoryDynamics& dynamics,
        double safety_margin = 1.0,
        double hardware_velocity_limit = 5.0);

private:
    TrajectoryConverter() = delete; // 静态工具类，禁止实例化
};

} // namespace arm_controller::utils

#endif // __TRAJECTORY_CONVERTER_HPP__