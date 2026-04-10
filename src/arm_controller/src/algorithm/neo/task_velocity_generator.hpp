#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

namespace arm_controller::algorithm::reactive_qp {

struct TaskVelocityConfig {
    // 位置误差反馈增益
    Eigen::Vector3d kp_pos{3.0, 3.0, 3.0};

    // 姿态误差反馈增益
    Eigen::Vector3d ko_ori{3.0, 3.0, 3.0};

    // 输出限幅，避免v_des过激
    double max_linear_speed{0.5};   // m/s
    double max_angular_speed{1.0};  // rad/s

    // 当误差非常小时，直接清零，避免抖动
    double position_deadband{1e-4};     // m
    double orientation_deadband{1e-4};  // rad
};

struct TaskVelocityInput {
    // 当前末端位姿（建议统一在base/world坐标系）
    Eigen::Isometry3d T_current = Eigen::Isometry3d::Identity();

    // 目标位姿
    bool has_target_pose{false};
    Eigen::Isometry3d T_target = Eigen::Isometry3d::Identity();

    // 目标空间速度 [vx vy vz wx wy wz]
    bool has_target_twist{false};
    Eigen::Matrix<double, 6, 1> target_twist =
        Eigen::Matrix<double, 6, 1>::Zero();

    // 可选调试标签
    std::string debug_name;
};

struct TaskVelocityOutput {
    Eigen::Matrix<double, 6, 1> v_des =
        Eigen::Matrix<double, 6, 1>::Zero();

    Eigen::Vector3d e_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d e_ori = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 6, 1> v_ff =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> v_fb =
        Eigen::Matrix<double, 6, 1>::Zero();
};

class TaskVelocityGenerator {
    public:
    TaskVelocityOutput compute(
        const TaskVelocityInput& in,
        const TaskVelocityConfig& cfg) const;

    private:
    // 计算姿态误差，返回“当前姿态到目标姿态”的旋转向量（在基坐标系表达）
    static Eigen::Vector3d computeOrientationError(
        const Eigen::Matrix3d& R_current,
        const Eigen::Matrix3d& R_target);

    static Eigen::Vector3d clampNorm(
        const Eigen::Vector3d& v,
        double max_norm);

    static Eigen::Vector3d applyDeadband(
        const Eigen::Vector3d& v,
        double threshold);
};

// Frame conversion helpers for feeding world-frame references into NEO.
// If NEO uses base frame, call with T_base_world.
class TaskVelocityFrameAdapter {
public:
    // Pose conversion: T_dst_obj = T_dst_src * T_src_obj
    static Eigen::Isometry3d transformPose(
        const Eigen::Isometry3d& T_dst_src,
        const Eigen::Isometry3d& T_src_obj);

    // Twist conversion by frame-rotation only (same EE reference point).
    // V = [v; w], both linear and angular are rotated to dst frame.
    static Eigen::Matrix<double, 6, 1> rotateTwist(
        const Eigen::Matrix<double, 6, 1>& V_src,
        const Eigen::Matrix3d& R_dst_src);

    // Convert a full task input from src frame to dst frame.
    static TaskVelocityInput transformInput(
        const TaskVelocityInput& in_src,
        const Eigen::Isometry3d& T_dst_src);
};

}  // namespace arm_controller::algorithm::reactive_qp
