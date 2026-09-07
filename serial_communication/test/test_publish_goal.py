#!/usr/bin/env python3
"""
测试 publishGoal 去重效果
用法: ros2 run serial_communication test_publish_goal
或者: python3 test_publish_goal.py

监控 /goal_pose 话题，统计收到 goal 的频率
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalMonitor(Node):
    def __init__(self):
        super().__init__('goal_monitor')
        self.sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.cb, 10)
        self.count = 0
        self.last_x = None
        self.last_y = None
        self.dup_count = 0
        self.start_time = time.time()
        self.last_report = self.start_time

    def cb(self, msg):
        self.count += 1
        x = msg.pose.position.x
        y = msg.pose.position.y

        # 检查是否重复点
        if self.last_x is not None:
            if abs(x - self.last_x) < 0.01 and abs(y - self.last_y) < 0.01:
                self.dup_count += 1

        self.last_x = x
        self.last_y = y

        now = time.time()
        # 每5秒报告一次
        if now - self.last_report >= 5.0:
            elapsed = now - self.start_time
            rate = self.count / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f'[统计] 总收={self.count} | 重复点={self.dup_count} | '
                f'频率={rate:.2f}Hz | 运行={elapsed:.0f}s')
            self.last_report = now


def main():
    rclpy.init()
    node = GoalMonitor()
    node.get_logger().info('=== Goal 监控已启动, Ctrl+C 退出 ===')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        elapsed = time.time() - node.start_time
        rate = node.count / elapsed if elapsed > 0 else 0
        node.get_logger().info(
            f'\n=== 最终统计 ===\n'
            f'总收到: {node.count}\n'
            f'重复点: {node.dup_count}\n'
            f'平均频率: {rate:.2f}Hz\n'
            f'运行时长: {elapsed:.1f}s')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
