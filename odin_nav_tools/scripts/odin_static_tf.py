#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class OdinStaticTf(Node):
    def __init__(self):
        super().__init__("odin_static_tf")
        self.declare_parameter("enabled", False)
        self.declare_parameter("parent_frame", "odin1_base_link")
        self.declare_parameter("child_frame", "base_footprint")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)

        if not self.get_parameter("enabled").value:
            self.get_logger().warn("Odin static TF is disabled; no transform is being published.")
            return

        roll = self.get_parameter("roll").value
        pitch = self.get_parameter("pitch").value
        yaw = self.get_parameter("yaw").value
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.get_parameter("parent_frame").value
        transform.child_frame_id = self.get_parameter("child_frame").value
        transform.transform.translation.x = self.get_parameter("x").value
        transform.transform.translation.y = self.get_parameter("y").value
        transform.transform.translation.z = self.get_parameter("z").value
        transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
        transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
        transform.transform.rotation.z = cr * cp * sy - sr * sp * cy
        transform.transform.rotation.w = cr * cp * cy + sr * sp * sy

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transform)
        self.get_logger().info(
            "Publishing static TF %s -> %s: xyz=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
            transform.header.frame_id, transform.child_frame_id,
            transform.transform.translation.x, transform.transform.translation.y,
            transform.transform.translation.z, roll, pitch, yaw)


def main():
    rclpy.init()
    node = OdinStaticTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
