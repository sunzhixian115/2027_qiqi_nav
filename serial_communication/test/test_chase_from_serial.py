#!/usr/bin/env python3
"""
测试: 串口模拟 → /track_target → chase → /chase_point
==============================================
模拟串口收到 enemy_distance_=5, enemy_yaw_=0.3
发布到 /track_target，监听 /chase_point 是否有输出

用法:
  终端1: ros2 launch 你的launch文件
  终端2: python3 test_chase_from_serial.py
"""

import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Target
from geometry_msgs.msg import Point
import math
import time


class ChaseFromSerialTest(Node):
    def __init__(self):
        super().__init__('test_chase_from_serial')

        # 模拟串口数据
        self.enemy_distance_ = 5.0
        self.enemy_yaw_ = 0.3  # 约17度

        # 发布者: 模拟 serial_communication 发布 /track_target
        self.track_target_pub_ = self.create_publisher(
            Target, '/track_target', 10)

        # 订阅者: 监听 chase 节点发布的 /chase_point
        self.chase_point_sub_ = self.create_subscription(
            Point, '/chase_point', self.chase_point_callback, 10)

        self.received_chase_point = False
        self.publish_count = 0

        # 定时发布, 模拟串口每100ms发一次
        self.timer = self.create_timer(0.1, self.publish_track_target)

        self.get_logger().info('=== 测试开始 ===')
        self.get_logger().info(f'模拟串口: enemy_distance={self.enemy_distance_}, enemy_yaw={self.enemy_yaw_}')
        self.get_logger().info('正在发布 /track_target, 等待 /chase_point 回应...')

    def publish_track_target(self):
        """模拟串口发布 enemy_distance_ 和 enemy_yaw_"""
        msg = Target()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'chassis'
        msg.distance = self.enemy_distance_
        msg.yaw = self.enemy_yaw_
        msg.tracking = (self.enemy_distance_ > 2.0 and self.enemy_distance_ < 10.0)
        msg.position.x = self.enemy_distance_ * math.cos(self.enemy_yaw_)
        msg.position.y = self.enemy_distance_ * math.sin(self.enemy_yaw_)
        msg.position.z = 0.0

        self.track_target_pub_.publish(msg)
        self.publish_count += 1

        if self.publish_count <= 3 or self.publish_count % 20 == 0:
            self.get_logger().info(
                f'[TX #{self.publish_count}] /track_target: '
                f'dist={msg.distance:.2f} yaw={msg.yaw:.2f} '
                f'tracking={msg.tracking} '
                f'pos=({msg.position.x:.2f}, {msg.position.y:.2f})')

    def chase_point_callback(self, msg: Point):
        """收到 chase 节点发布的追击点"""
        if msg.x == 0.0 and msg.y == 0.0:
            self.get_logger().warn(
                f'[RX] /chase_point: ({msg.x:.2f}, {msg.y:.2f}) '
                f'--- (0,0) 说明chase没算出有效点!')
        else:
            self.get_logger().info(
                f'[RX] /chase_point: ({msg.x:.2f}, {msg.y:.2f}) '
                f'--- 有效追击点!')
            self.received_chase_point = True


def main():
    rclpy.init()
    node = ChaseFromSerialTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('=== 测试结束 ===')
        if node.received_chase_point:
            node.get_logger().info('结果: /chase_point 有效, 追击链路通了!')
        else:
            node.get_logger().warn('结果: 没收到有效 /chase_point, 检查:')
            node.get_logger().warn('  1. chase节点是否启动')
            node.get_logger().warn('  2. TF树是否通 (odom→chassis)')
            node.get_logger().warn('  3. costmap是否有数据')
            node.get_logger().warn('  4. /track_target的frame_id是否和chase节点匹配')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
