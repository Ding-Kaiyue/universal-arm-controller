#!/usr/bin/env python3
"""
重力补偿节点
使用 Pinocchio 库计算机器人重力补偿力矩
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import pinocchio as pin
import numpy as np


class GravityCompensator(Node):
    """重力补偿计算器节点"""

    def __init__(self):
        super().__init__('gravity_compensator')

        # 从参数服务器读取 robot_description（URDF 字符串）
        self.declare_parameter('robot_description', '')
        urdf_string = self.get_parameter('robot_description').get_parameter_value().string_value

        if not urdf_string:
            self.get_logger().error("参数 'robot_description' 未设置！")
            return

        try:
            # 从 URDF 字符串构建 Pinocchio 模型
            self.model = pin.buildModelFromXML(urdf_string)
            self.data = self.model.createData()

            # 获取关节名称（排除 universe 和固定关节）
            self.joint_names = [
                name for idx, name in enumerate(self.model.names)
                if idx > 0 and self.model.joints[idx].nq > 0
            ]

            self.get_logger().info(f"成功加载模型，关节数量: {len(self.joint_names)}")
            self.get_logger().info(f"关节名称: {self.joint_names}")

        except Exception as e:
            self.get_logger().error(f"模型构建失败: {str(e)}")
            return

        # 当前关节位置
        self.q = np.zeros(self.model.nq)

        # 笛卡尔速度控制相关
        self.dq_max = 2.0  # 最大关节速度限制（rad/s）

        # === 订阅与发布 ===
        # 订阅关节状态
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # 发布重力补偿力矩
        self.torque_pub = self.create_publisher(
            JointState,
            '/gravity_torque',
            10
        )

        # 订阅笛卡尔速度命令（用于拖动示教）
        self.create_subscription(
            Twist,
            '/cartesian_velocity_cmd',
            self.velocity_callback,
            10
        )

        # 发布关节速度（从笛卡尔速度转换）
        self.joint_velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocity',
            10
        )

        self.get_logger().info("重力补偿节点已启动")

    def joint_callback(self, msg: JointState):
        """
        关节状态回调函数
        接收关节状态，计算并发布重力补偿力矩
        """
        try:
            q = np.zeros(self.model.nq)

            # 从消息中提取关节位置
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    q[i] = msg.position[idx]

            # 保存当前关节位置（用于笛卡尔速度控制）
            self.q = q

            # 使用 Pinocchio 计算重力补偿力矩
            g = pin.computeGeneralizedGravity(self.model, self.data, q)

            # 发布重力补偿力矩
            torque_msg = JointState()
            torque_msg.header.stamp = self.get_clock().now().to_msg()
            torque_msg.name = self.joint_names
            torque_msg.effort = g.tolist()
            self.torque_pub.publish(torque_msg)

        except Exception as e:
            self.get_logger().error(f"关节状态处理失败: {e}")

    def velocity_callback(self, msg: Twist):
        """
        笛卡尔速度回调函数
        将笛卡尔空间速度转换为关节空间速度
        """
        try:
            # 构建笛卡尔速度向量 [vx, vy, vz, wx, wy, wz]
            v_ee = np.array([
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
            ])

            if len(v_ee) != 6:
                self.get_logger().error("输入笛卡尔速度不是六维向量！")
                return

            # 使用 Pinocchio 计算雅可比矩阵
            # 使用末端执行器帧（通常是最后一个关节）
            ee_frame = self.model.njoints - 1
            J = pin.computeJointJacobian(self.model, self.data, self.q, ee_frame)  # 6xN

            # 通过雅可比矩阵的伪逆求解关节速度
            dq = np.linalg.pinv(J) @ v_ee

            # 限幅
            dq = np.clip(dq, -self.dq_max, self.dq_max)

            # 发布关节速度
            out_msg = Float64MultiArray()
            out_msg.data = dq.tolist()
            self.joint_velocity_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"计算关节速度失败: {e}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = GravityCompensator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
