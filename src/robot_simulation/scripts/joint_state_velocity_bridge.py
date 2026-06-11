#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointStateVelocityBridge(Node):
    def __init__(self):
        super().__init__("joint_state_velocity_bridge")
        self.declare_parameter(
            "joints",
            [
                "front_left_wheel_joint",
                "front_right_wheel_joint",
                "rear_left_wheel_joint",
                "rear_right_wheel_joint",
            ],
        )
        self.declare_parameter("input_topic", "chassis_joint_commands")
        self.declare_parameter(
            "output_topic", "/chassis_wheel_velocity_controller/commands"
        )

        self.joints = list(self.get_parameter("joints").value)
        self.publisher = self.create_publisher(
            Float64MultiArray, self.get_parameter("output_topic").value, 10
        )
        self.subscription = self.create_subscription(
            JointState,
            self.get_parameter("input_topic").value,
            self.handle_joint_state,
            10,
        )

        self.get_logger().info(
            "joint velocity bridge ready: %s -> %s"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value,
            )
        )

    def handle_joint_state(self, msg):
        velocity_by_name = {}
        for index, name in enumerate(msg.name):
            if index >= len(msg.velocity):
                continue
            value = msg.velocity[index]
            if math.isnan(value):
                continue
            velocity_by_name[name] = value

        command = Float64MultiArray()
        command.data = [velocity_by_name.get(joint, 0.0) for joint in self.joints]
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = JointStateVelocityBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
