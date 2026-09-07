#!/usr/bin/env python3
"""
模拟障碍物数据流测试
发布假的 /pc_detector/rel_yaw_dist 数据,
观察 red 节点是否正确接收并计算全局坐标
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import math
import time


class ObstacleFlowTest(Node):
    def __init__(self):
        super().__init__('obstacle_flow_test')

        # 发布假的 rel_yaw_dist 数据
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/pc_detector/rel_yaw_dist',
            10
        )

        # 订阅 red 节点发给电控的数据 (没法直接看串口, 但可以观察)
        self.get_logger().info('=== 障碍物数据流测试 ===')

        self.timer = self.create_timer(0.5, self.publish_test)

        self.test_cases = [
            # (描述, dist, yaw_rad)
            ("正前方2m",       2.0, 0.0),
            ("右前方1.5m@30°", 1.5, -math.radians(30)),
            ("左后方3m@120°",  3.0, math.radians(120)),
            ("右侧1m@90°",     1.0, -math.radians(90)),
            ("无障碍物",        0.0, 0.0),
        ]
        self.case_idx = 0

    def publish_test(self):
        if self.case_idx >= len(self.test_cases):
            self.get_logger().info('=== 测试用例发送完毕 ===')
            self.case_idx = 0  # 循环

        desc, dist, yaw = self.test_cases[self.case_idx]

        # 期望的 livox_frame 下坐标
        expected_x = dist * math.cos(yaw)
        expected_y = dist * math.sin(yaw)

        self.get_logger().info(
            f'[{self.case_idx}] {desc}: '
            f'dist={dist:.2f}, yaw={yaw:.3f}rad → '
            f'livox_frame: ({expected_x:.2f}, {expected_y:.2f}) → '
            f'TF → map: (?, ?) [由 red 节点计算]'
        )

        # rel_yaw_dist_node 发的格式: [dist, yaw, dist, yaw, ...]
        msg = Float32MultiArray()
        if dist > 0.01:
            msg.data = [float(dist), float(yaw)]
        else:
            msg.data = []  # 空 = 无障碍物

        self.pub.publish(msg)
        self.case_idx += 1


def main():
    rclpy.init()
    node = ObstacleFlowTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
