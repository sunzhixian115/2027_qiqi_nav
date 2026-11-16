#!/usr/bin/env python3
"""检查精简后的实车模型，并在独立 ROS 域内验证机器人描述与 TF。"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


KEEP_LINKS = {
    "base_footprint", "chassis", "odin1_base_link", "gimbal_yaw_odom",
    "gimbal_pitch_odom", "gimbal_yaw", "gimbal_pitch", "speed_monitor",
    "front_industrial_camera", "front_industrial_camera_optical_frame",
}
REMOVED_LINKS = {
    "front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel",
    "front_rplidar_a2", "light_indicator",
} | {f"armor_{i}" for i in range(4)} | {f"armor_support_frame_{i}" for i in range(4)}


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def canonical(element):
    return ET.canonicalize(ET.tostring(element, encoding="unicode"), strip_text=True)


def edge(joint):
    return joint.find("parent").attrib["link"], joint.find("child").attrib["link"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path,
                        help="修改前生成的 URDF，用于核对保留部分的全部参数")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--domain-id", type=int, default=173,
                        help="测试域，不启动雷达、导航或底盘")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory("pb2025_robot_description"))
    macro = XMLMacro4sdf()
    macro.set_xml_file(str(share / "resource/xmacro/pb2025_sentry_robot.sdf.xmacro"))
    macro.generate()
    sdf_text = macro.to_string()
    generator = UrdfGenerator()
    generator.parse_from_sdf_string(sdf_text)
    urdf_text = generator.to_string()
    robot = ET.fromstring(urdf_text)
    baseline = ET.parse(args.baseline).getroot()
    links = {item.attrib["name"]: item for item in robot.findall("link")}
    joints = {item.attrib["name"]: item for item in robot.findall("joint")}
    old_links = {item.attrib["name"]: item for item in baseline.findall("link")}
    old_joints = {item.attrib["name"]: item for item in baseline.findall("joint")}
    require(set(links) == KEEP_LINKS, f"保留节点不符：{set(links) ^ KEEP_LINKS}")
    require(set(old_links) - set(links) == REMOVED_LINKS, "删除名单不符")
    require(len(joints) == len(links) - 1, "机器人描述不是预期的树")
    for name, item in links.items():
        require(canonical(item) == canonical(old_links[name]), f"保留模型被改变：{name}")
    for name, item in joints.items():
        require(canonical(item) == canonical(old_joints[name]), f"保留关节被改变：{name}")
        require(set(edge(item)).issubset(links), f"关节引用不存在的模型：{name}")
    for plugin in ET.fromstring(sdf_text).findall(".//plugin"):
        for visual in plugin.findall("link_visual"):
            require(visual.text.split("/")[0] in links, "插件仍引用已删除模型")
    require("MecanumDrive2" not in sdf_text, "轮子驱动插件未清理")
    (args.output_dir / "sentry-after.urdf").write_text(urdf_text)
    (args.output_dir / "sentry-after.sdf").write_text(sdf_text)

    # 仅启动原有机器人描述 launch；隔离域内不会接入实际导航和设备。
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("check_sentry_tf")
    received = {}
    descriptions = []

    def on_tf(message):
        for transform in message.transforms:
            received[(transform.header.frame_id, transform.child_frame_id)] = transform

    durable = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(TFMessage, "/tf", on_tf, qos_profile_sensor_data),
        node.create_subscription(TFMessage, "/tf_static", on_tf, durable),
        node.create_subscription(String, "/robot_description", descriptions.append, durable),
    ]
    expected_edges = {edge(item) for item in joints.values()}
    log_path = args.output_dir / "robot-description-test.log"
    process = None
    try:
        with log_path.open("w") as logfile:
            process = subprocess.Popen(
                ["ros2", "launch", "pb2025_robot_description", "robot_description_launch.py",
                 "use_rviz:=False", "use_sim_time:=False"],
                stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True,
            )
            deadline = time.monotonic() + 15
            ready_since = None
            while time.monotonic() < deadline:
                require(process.poll() is None, f"机器人描述提前退出，见 {log_path}")
                rclpy.spin_once(node, timeout_sec=0.05)
                if set(received) == expected_edges and descriptions:
                    ready_since = ready_since or time.monotonic()
                    if time.monotonic() - ready_since > 1:
                        break
            require(set(received) == expected_edges,
                    f"实际 TF 不符；缺少 {expected_edges - set(received)}，多出 {set(received) - expected_edges}")
            require(descriptions, "未收到 robot_description")
            require(canonical(ET.fromstring(descriptions[-1].data)) == canonical(robot),
                    "实际发布的模型与源码生成结果不同")
            # joint_state_publisher 默认零位时，核对每条边的位置及旋转。
            for item in joints.values():
                transform = received[edge(item)].transform
                origin = item.find("origin")
                xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
                roll, pitch, yaw = [float(v) for v in origin.get("rpy", "0 0 0").split()]
                cr, sr = math.cos(roll / 2), math.sin(roll / 2)
                cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
                cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
                expected_q = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                              cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
                actual_xyz = [transform.translation.x, transform.translation.y, transform.translation.z]
                q = transform.rotation
                actual_q = [q.x, q.y, q.z, q.w]
                require(max(abs(a-b) for a, b in zip(xyz, actual_xyz)) < 1e-6,
                        f"实际平移不符：{edge(item)}")
                require(abs(abs(sum(a*b for a, b in zip(expected_q, actual_q))) - 1) < 1e-6,
                        f"实际旋转不符：{edge(item)}")
            # 生成标准 TF 树文件，作为运行证据保存。
            view = subprocess.Popen(
                ["ros2", "run", "tf2_tools", "view_frames"], cwd=args.output_dir,
                stdout=logfile, stderr=subprocess.STDOUT,
            )
            try:
                require(view.wait(timeout=20) == 0, "view_frames 执行失败")
            except subprocess.TimeoutExpired:
                view.terminate()
                view.wait(timeout=5)
                raise
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()

    report = {
        "passed": True,
        "retained_links": sorted(links),
        "removed_links": sorted(REMOVED_LINKS),
        "retained_models_and_joint_parameters_unchanged": True,
        "actual_tf_matches_generated_urdf": True,
        "actual_tf_zero_joint_poses_verified": True,
        "observed_tf_edges": sorted([list(pair) for pair in received]),
        "ros_domain_id": args.domain_id,
        "test_launch_exit_code": process.returncode,
        "scope": "仅机器人描述；map/odom 和驱动生成的 TF 未在本测试中启动",
    }
    (args.output_dir / "tf-cleanup-check.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
