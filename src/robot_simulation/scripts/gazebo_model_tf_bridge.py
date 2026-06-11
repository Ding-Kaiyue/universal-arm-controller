#!/usr/bin/env python3

import math

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GazeboModelTfBridge(Node):
    def __init__(self):
        super().__init__("gazebo_model_tf_bridge")
        self.declare_parameter("model_name", "simple_omni_dual_arm")
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("project_to_planar", True)
        self.declare_parameter("base_z", 0.0)

        self.model_name = self.get_parameter("model_name").value
        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.model_states_topic = self.get_parameter("model_states_topic").value
        self.project_to_planar = bool(self.get_parameter("project_to_planar").value)
        self.base_z = float(self.get_parameter("base_z").value)

        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self.handle_model_states,
            10,
        )
        self.missing_warned = False

        self.get_logger().info(
            "gazebo model TF bridge ready: %s -> %s from model '%s' on %s"
            % (self.world_frame, self.base_frame, self.model_name, self.model_states_topic)
        )

    def handle_model_states(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            if not self.missing_warned:
                self.get_logger().warn(
                    "model '%s' not found in %s"
                    % (self.model_name, self.model_states_topic)
                )
                self.missing_warned = True
            return

        pose = msg.pose[index]
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y

        if self.project_to_planar:
            transform.transform.translation.z = self.base_z
            yaw = quaternion_to_yaw(pose.orientation)
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
        else:
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation

        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GazeboModelTfBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
