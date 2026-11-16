#!/usr/bin/env python3

import math
import sys
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from base_interfaces_demo.msg import MpcPositionCommand, PositionCommand


class MincoControllerSmokeTest(Node):
    def __init__(self):
        super().__init__("minco_controller_smoke_test")
        transient_local = QoSProfile(depth=1)
        transient_local.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_local.reliability = ReliabilityPolicy.RELIABLE

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", transient_local)
        self.odom_pub = self.create_publisher(Odometry, "/odin1/odometry", 10)
        self.opt_pub = self.create_publisher(MpcPositionCommand, "/opt_path", 10)
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.on_cmd, 10
        )
        self.mpc_cmd_sub = self.create_subscription(
            Twist, "/cmd_vel_mpc", self.on_mpc_cmd, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.follow_path = ActionClient(self, FollowPath, "/follow_path")
        self.last_cmd: Optional[Twist] = None
        self.started_at = self.get_clock().now()
        self.goal_sent = False
        self.result_received = False
        self.mpc_cmd: Optional[Twist] = None
        self.finished = False
        self.timer = self.create_timer(0.05, self.tick)

    frame = "odom"

    def publish_tf(self):
        stamp = self.get_clock().now().to_msg()
        for parent, child in (("map", "odom"), ("odom", "gimbal_yaw_fake")):
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.info.resolution = 0.05
        msg.info.width = 200
        msg.info.height = 200
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -5.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (msg.info.width * msg.info.height)
        self.map_pub.publish(msg)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.child_frame_id = "gimbal_yaw_fake"
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(msg)

    def publish_opt_path(self):
        msg = MpcPositionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.mpc_horizon = 10
        msg.command_flag = MpcPositionCommand.NORMAL_COMMAND
        for i in range(31):
            cmd = PositionCommand()
            cmd.position.x = 0.05 * i
            cmd.position.y = 0.0
            cmd.position.z = 0.0
            cmd.velocity.x = 0.5
            cmd.velocity.y = 0.0
            cmd.acceleration.x = 0.0
            cmd.acceleration.y = 0.0
            cmd.jerk.x = 0.0
            cmd.jerk.y = 0.0
            cmd.yaw = 0.0
            cmd.yaw_dot = 0.0
            cmd.trajectory_id = 1
            msg.cmds.append(cmd)
        self.opt_pub.publish(msg)

    def send_goal(self):
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds * 1.0e-9
        if elapsed < 8.0 or self.goal_sent or not self.follow_path.wait_for_server(timeout_sec=0.0):
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame
        for x in (0.0, 1.0):
            pose = __import__("geometry_msgs.msg", fromlist=["PoseStamped"]).PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "MincoMpc"
        goal.goal_checker_id = "general_goal_checker"
        self.goal_sent = True
        future = self.follow_path.send_goal_async(goal)
        future.add_done_callback(self.goal_response)
        self.get_logger().info("Sent FollowPath goal with controller_id=MincoMpc")

    def goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("FollowPath goal was rejected")
            return
        self.get_logger().info("FollowPath goal accepted")

    def on_cmd(self, msg):
        self.last_cmd = msg
        if abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z) > 1.0e-3:
            self.result_received = True

    def on_mpc_cmd(self, msg):
        self.mpc_cmd = msg
        if abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z) > 1.0e-3:
            self.result_received = True

    def tick(self):
        self.publish_tf()
        self.publish_map()
        self.publish_odom()
        self.publish_opt_path()
        self.send_goal()

        elapsed = (self.get_clock().now() - self.started_at).nanoseconds * 1.0e-9
        if self.result_received:
            cmd = self.mpc_cmd or self.last_cmd
            self.get_logger().info(
                "PASS: MincoMpc produced velocity "
                f"vx={cmd.linear.x:.3f} vy={cmd.linear.y:.3f} wz={cmd.angular.z:.3f}"
            )
            self.finished = True
        elif elapsed > 20.0:
            self.get_logger().error(
                "FAIL: no non-zero /cmd_vel_nav2_result within 20 seconds"
            )
            self.finished = True


def main():
    rclpy.init()
    node = MincoControllerSmokeTest()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.result_received else 1


if __name__ == "__main__":
    sys.exit(main())
