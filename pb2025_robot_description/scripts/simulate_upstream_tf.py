#!/usr/bin/env python3
"""无雷达离线测试：模拟 Odin 输入，经原有节点生成 TF，导出 PDF/GV。"""

import argparse
from collections import defaultdict
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
from zoneinfo import ZoneInfo


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--domain-id", type=int, default=174)
    parser.add_argument("--duration", type=float, default=12.0)
    args = parser.parse_args()
    if args.duration < 8:
        parser.error("duration 至少 8 秒，以等待 TF 和节点发现")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    # 所有测试进程和模拟数据仅存在于本测试域。
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformBroadcaster, TransformListener, StaticTransformBroadcaster
    import yaml

    rclpy.init()
    node = rclpy.create_node("simulated_odin_upstream")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    dynamic = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    pubs = {
        "odom": node.create_publisher(Odometry, "/odin1/odometry", 10),
        "highfreq": node.create_publisher(Odometry, "/odin1/odometry_highfreq", 10),
        "cloud": node.create_publisher(PointCloud2, "/odin1/cloud_slam", 10),
    }
    received = defaultdict(list)
    observed = defaultdict(list)
    counts = defaultdict(int)

    def transform(parent, child, stamp, xyz=(0, 0, 0), yaw=0):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        return t

    static.sendTransform(transform("odin1_base_link", "lidar", node.get_clock().now().to_msg()))
    start = time.monotonic()

    def odometry(stamp, elapsed):
        yaw = 0.2 * math.sin(elapsed * 0.5)
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = "odom"
        m.child_frame_id = "odin1_base_link"
        m.pose.pose.position.x = 0.1 * math.sin(elapsed * 0.3)
        m.pose.pose.position.y = 0.05 * math.sin(elapsed * 0.4)
        m.pose.pose.position.z = 0.426
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        m.twist.twist.angular.z = 0.1 * math.cos(elapsed * 0.5)
        # 将 odom 系速度转到消息规定的 child 系。
        vx, vy = 0.03 * math.cos(elapsed * 0.3), 0.02 * math.cos(elapsed * 0.4)
        m.twist.twist.linear.x = math.cos(yaw)*vx + math.sin(yaw)*vy
        m.twist.twist.linear.y = -math.sin(yaw)*vx + math.cos(yaw)*vy
        return m

    points = b"".join(struct.pack("<fffI", 2*math.cos(i*math.tau/360),
                                 2*math.sin(i*math.tau/360), 0.15+(i % 20)*0.02,
                                 0x00A0C0FF) for i in range(360))

    def publish_standard():
        stamp = node.get_clock().now().to_msg()
        pubs["odom"].publish(odometry(stamp, time.monotonic()-start))
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "odom"
        cloud.height, cloud.width = 1, 360
        cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(["x", "y", "z", "rgb"])]
        cloud.point_step, cloud.row_step = 16, len(points)
        cloud.is_dense = True
        cloud.data = points
        pubs["cloud"].publish(cloud)
        # 只模拟驱动负责的边；不代替下游节点发布机体或云台 TF。
        # lidar->imu 使用明确的虚拟外参，不冒充这台设备的实际标定。
        dynamic.sendTransform([
            transform("map", "odom", stamp, (0.3, -0.2, 0), 0.05),
            transform("lidar", "imu", stamp, (0, 0, -0.02)),
        ])
        counts["standard_odom"] += 1
        counts["cloud"] += 1

    def publish_highfreq():
        pubs["highfreq"].publish(odometry(node.get_clock().now().to_msg(), time.monotonic()-start))
        counts["highfreq_odom"] += 1

    timers = [node.create_timer(0.1, publish_standard), node.create_timer(0.0025, publish_highfreq)]

    def on_tf(message):
        for t in message.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            received[key].append(time.monotonic())

    def on_message(message, topic):
        observed[topic].append(time.monotonic())

    durable = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(TFMessage, "/tf", on_tf, qos_profile_sensor_data),
        node.create_subscription(TFMessage, "/tf_static", on_tf, durable),
    ]
    for topic, message_type in [("/sensor_scan", PointCloud2), ("/odometry", Odometry),
                                ("/odin1/cloud_slam_xyzi", PointCloud2)]:
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda m, t=topic: on_message(m, t), qos_profile_sensor_data))

    params = str(Path(get_package_share_directory("pb2025_nav_bringup")) /
                 "config/reality/nav2_params.yaml")
    commands = [
        ["ros2", "launch", "pb2025_robot_description", "robot_description_launch.py",
         "use_rviz:=False", "use_sim_time:=False"],
        ["ros2", "run", "odin_nav_tools", "odin_cloud_to_xyzi"],
        ["ros2", "run", "sensor_scan_generation", "sensor_scan_generation_node", "--ros-args",
         "-r", "__node:=sensor_scan_generation", "--params-file", params,
         "-r", "lidar_odometry:=/odin1/odometry", "-r", "registered_scan:=/odin1/cloud_slam_xyzi"],
        ["ros2", "run", "fake_vel_transform", "fake_vel_transform_node", "--ros-args",
         "-r", "__node:=fake_vel_transform", "--params-file", params],
    ]
    processes, logs = [], []
    try:
        for i, command in enumerate(commands):
            log = (args.output_dir / f"node-{i}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        while time.monotonic()-start < args.duration:
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError("测试节点提前退出，请检查 node-*.log")
            rclpy.spin_once(node, timeout_sec=0.01)

        expected = {
            ("map", "odom"), ("odom", "base_footprint"), ("base_footprint", "chassis"),
            ("chassis", "odin1_base_link"), ("odin1_base_link", "lidar"), ("lidar", "imu"),
            ("chassis", "gimbal_yaw_odom"), ("gimbal_yaw_odom", "gimbal_pitch_odom"),
            ("gimbal_pitch_odom", "gimbal_yaw"), ("gimbal_yaw", "gimbal_pitch"),
            ("gimbal_yaw", "gimbal_yaw_fake"), ("gimbal_pitch", "speed_monitor"),
            ("gimbal_pitch", "front_industrial_camera"),
            ("front_industrial_camera", "front_industrial_camera_optical_frame"),
        }
        if set(received) != expected:
            raise AssertionError(f"TF 缺少 {expected-set(received)}；多出 {set(received)-expected}")
        if any(len(observed[t]) < 20 for t in ["/sensor_scan", "/odometry", "/odin1/cloud_slam_xyzi"]):
            raise AssertionError("原有下游节点没有持续输出")
        # 所有节点必须连到 map，每个 child 只有一个 parent。
        parents = {child: parent for parent, child in expected}
        if len(parents) != len(expected):
            raise AssertionError("出现多个父节点")
        for child in parents:
            current, seen = child, set()
            while current in parents:
                if current in seen:
                    raise AssertionError("TF 有环")
                seen.add(current)
                current = parents[current]
            if current != "map":
                raise AssertionError("TF 未连到 map")
            buffer.lookup_transform("map", child, rclpy.time.Time())

        graph = yaml.safe_load(buffer.all_frames_as_yaml())
        (args.output_dir / "tf-buffer.yaml").write_text(yaml.safe_dump(graph, sort_keys=True))
        simulated = {("map", "odom"), ("odin1_base_link", "lidar"), ("lidar", "imu")}
        stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y-%m-%d %H:%M:%S CST")
        dot = ["digraph TF {", 'graph [rankdir=TB, bgcolor="white", pad=0.3, nodesep=0.25, ranksep=0.40,',
               f'label="Odin1 TF tree - OFFLINE SIMULATION\\n{stamp} | ROS domain {args.domain_id}\\nBlue dashed edges: mock driver | Other edges: existing ROS nodes", labelloc=t, fontname="DejaVu Sans", fontsize=15];',
               'node [shape=box, style="rounded,filled", fillcolor="#F1F5F9", color="#94A3B8", fontname="DejaVu Sans", fontsize=11];',
               'edge [fontname="DejaVu Sans", fontsize=9, color="#475569"];']
        for parent, child in sorted(expected):
            info = graph[child]
            rate = "static" if float(info["buffer_length"]) == 0 else f'{info["rate"]:.1f} Hz'
            source = "mock driver" if (parent, child) in simulated else (
                "sensor_scan_generation" if parent == "odom" else
                "fake_vel_transform" if child == "gimbal_yaw_fake" else "robot_state_publisher")
            style = ', color="#2563EB", style=dashed' if (parent, child) in simulated else ""
            dot.append(f'"{parent}" -> "{child}" [label="{source}\\n{rate}"{style}];')
        dot.append("}")
        gv = args.output_dir / "odin_tf_simulated.gv"
        gv.write_text("\n".join(dot)+"\n")
        subprocess.run(["dot", "-Tpdf", str(gv), "-o", str(gv.with_suffix(".pdf"))], check=True)

        def hz(times):
            return round((len(times)-1)/(times[-1]-times[0]), 2) if len(times)>1 else None

        report = {"passed": True, "mode": "OFFLINE SIMULATION - not hardware validation",
                  "timestamp": stamp, "ros_domain_id": args.domain_id,
                  "simulated_extrinsics": "lidar->imu: (0,0,-0.02)m, identity rotation; not device calibration",
                  "publish_counts": dict(counts),
                  "downstream_hz": {t: hz(v) for t, v in observed.items()},
                  "tf_edges": [{"parent": p, "child": c, "measured_receive_hz": hz(received[p,c]),
                                "mock": (p,c) in simulated} for p,c in sorted(expected)],
                  "checks": ["exact retained branches", "no deleted frames", "single root map",
                             "no cycle", "all map-to-frame lookups succeed", "real downstream outputs"]}
        (args.output_dir / "simulation-check.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
