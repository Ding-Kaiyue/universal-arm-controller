#include "task_velocity_generator.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::algorithm::reactive_qp {

Eigen::Vector3d TaskVelocityGenerator::computeOrientationError(
    const Eigen::Matrix3d& R_current,
    const Eigen::Matrix3d& R_target) {
    // 误差旋转：从当前旋转到目标旋转
    const Eigen::Matrix3d R_err = R_current.transpose() * R_target;

    Eigen::AngleAxisd aa(R_err);

    // 数值保护：当角度极小，axis可能不稳定
    if (std::abs(aa.angle()) < 1e-12) {
        return Eigen::Vector3d::Zero();
    }

    // 先在当前末端局部坐标系得到误差旋转向量
    const Eigen::Vector3d e_body = aa.axis() * aa.angle();

    // 再映射到基坐标系，确保与通常Jacobian表达坐标系一致
    return R_current * e_body;
}

Eigen::Vector3d TaskVelocityGenerator::clampNorm(
    const Eigen::Vector3d& v,
    double max_norm) {
    const double n = v.norm();
    if (n < 1e-12 || n <= max_norm) {
        return v;
    }
    return v * (max_norm / n);
}

Eigen::Vector3d TaskVelocityGenerator::applyDeadband(
    const Eigen::Vector3d& v,
    double threshold) {
    if (v.norm() < threshold) {
        return Eigen::Vector3d::Zero();
    }
    return v;
}

TaskVelocityOutput TaskVelocityGenerator::compute(
    const TaskVelocityInput& in,
    const TaskVelocityConfig& cfg) const {

    TaskVelocityOutput out;

    // 1) 前馈项：直接用目标twist
    if (in.has_target_twist) {
        out.v_ff = in.target_twist;
    }

    // 2) 反馈项：由目标pose误差生成
    if (in.has_target_pose) {
        const Eigen::Vector3d p_cur = in.T_current.translation();
        const Eigen::Vector3d p_tgt = in.T_target.translation();

        out.e_pos = p_tgt - p_cur;
        out.e_ori = computeOrientationError(
            in.T_current.rotation(),
            in.T_target.rotation());

        out.e_pos = applyDeadband(out.e_pos, cfg.position_deadband);
        out.e_ori = applyDeadband(out.e_ori, cfg.orientation_deadband);

        const Eigen::Vector3d v_fb_linear =
            cfg.kp_pos.asDiagonal() * out.e_pos;
        const Eigen::Vector3d v_fb_angular =
            cfg.ko_ori.asDiagonal() * out.e_ori;

        out.v_fb.head<3>() = v_fb_linear;
        out.v_fb.tail<3>() = v_fb_angular;
    }

    // 3) 合成期望速度
    out.v_des = out.v_ff + out.v_fb;

    // 4) 分别对线速度/角速度限幅
    out.v_des.head<3>() =
        clampNorm(out.v_des.head<3>(), cfg.max_linear_speed);
    out.v_des.tail<3>() =
        clampNorm(out.v_des.tail<3>(), cfg.max_angular_speed);

    return out;
}

Eigen::Isometry3d TaskVelocityFrameAdapter::transformPose(
    const Eigen::Isometry3d& T_dst_src,
    const Eigen::Isometry3d& T_src_obj) {
    return T_dst_src * T_src_obj;
}

Eigen::Matrix<double, 6, 1> TaskVelocityFrameAdapter::rotateTwist(
    const Eigen::Matrix<double, 6, 1>& V_src,
    const Eigen::Matrix3d& R_dst_src) {
    Eigen::Matrix<double, 6, 1> V_dst = Eigen::Matrix<double, 6, 1>::Zero();
    V_dst.head<3>() = R_dst_src * V_src.head<3>();
    V_dst.tail<3>() = R_dst_src * V_src.tail<3>();
    return V_dst;
}

TaskVelocityInput TaskVelocityFrameAdapter::transformInput(
    const TaskVelocityInput& in_src,
    const Eigen::Isometry3d& T_dst_src) {
    TaskVelocityInput out = in_src;
    out.T_current = transformPose(T_dst_src, in_src.T_current);

    if (in_src.has_target_pose) {
        out.T_target = transformPose(T_dst_src, in_src.T_target);
    }
    if (in_src.has_target_twist) {
        out.target_twist = rotateTwist(in_src.target_twist, T_dst_src.linear());
    }
    return out;
}

}  // namespace arm_controller::algorithm::reactive_qp
