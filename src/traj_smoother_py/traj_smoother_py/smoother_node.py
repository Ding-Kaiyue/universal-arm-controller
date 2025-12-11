#!/usr/bin/env python3
"""
轨迹平滑节点
使用 csaps 库对轨迹进行样条平滑处理
平滑后的轨迹发布到 /smoothed_joint_trajectory，由 TrajectoryReplayController 直接执行
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from csaps import csaps
import numpy as np


class TrajectorySmoother(Node):
    """轨迹平滑节点"""

    def __init__(self):
        super().__init__('trajectory_smoother')

        # 订阅原始轨迹
        self.sub = self.create_subscription(
            JointTrajectory,
            '/raw_joint_trajectory',
            self.traj_callback,
            10)

        # 发布平滑后的轨迹（TrajectoryReplayController 会订阅并直接执行）
        self.pub = self.create_publisher(
            JointTrajectory,
            '/smoothed_joint_trajectory',
            10)

        # 可调参数
        self.smooth_factor = 0.95  # 平滑系数（0-1，越大越平滑）

        self.get_logger().info('Trajectory smoother node started.')
        self.get_logger().info(f'  - Input topic: /raw_joint_trajectory')
        self.get_logger().info(f'  - Output topic: /smoothed_joint_trajectory')
        self.get_logger().info(f'  - Smooth factor: {self.smooth_factor}')

    def traj_callback(self, msg: JointTrajectory):
        """轨迹回调函数"""
        self.get_logger().info(f'Received trajectory with {len(msg.points)} points.')

        # 检查点数是否足够
        if len(msg.points) < 5:
            self.get_logger().warn('Too few points (< 5), skipping smoothing.')
            self.pub.publish(msg)
            return

        # 提取时间序列
        t = np.array([
            p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            for p in msg.points
        ])

        n_joints = len(msg.joint_names)

        # 准备输出消息
        smoothed = JointTrajectory()
        smoothed.joint_names = msg.joint_names
        smoothed.header = msg.header

        # 对每个关节进行平滑
        all_pos = []
        all_vel = []

        for i in range(n_joints):
            # 提取位置数据
            pos = np.array([p.positions[i] for p in msg.points])

            # 使用 csaps 进行样条平滑
            spline_func = csaps(t, pos, smooth=self.smooth_factor)

            # 计算平滑位置
            sm_pos = spline_func(t)

            # 计算一阶导（速度）
            sm_vel = spline_func(t, nu=1)  # nu=1 表示求一阶导

            all_pos.append(sm_pos)
            all_vel.append(sm_vel)

        # 转换为合适的形状 (N, n_joints)
        all_pos = np.array(all_pos).T
        all_vel = np.array(all_vel).T

        # 强制首尾速度为零（避免轨迹执行完成后仍有速度，触发安全保护）
        all_vel[0, :] = 0.0
        all_vel[-1, :] = 0.0

        # 构建平滑后的轨迹点
        for idx, ti in enumerate(t):
            pt = JointTrajectoryPoint()
            pt.positions = all_pos[idx, :].tolist()
            pt.velocities = all_vel[idx, :].tolist()
            pt.effort = [0.0] * n_joints

            # 设置时间戳
            pt.time_from_start.sec = int(ti)
            pt.time_from_start.nanosec = int((ti - int(ti)) * 1e9)

            smoothed.points.append(pt)

        # 发布平滑后的轨迹（TrajectoryReplayController 会订阅并直接通过 hardware_manager 执行）
        self.get_logger().info(f'Publishing smoothed trajectory ({len(smoothed.points)} points)')
        self.pub.publish(smoothed)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = TrajectorySmoother()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
