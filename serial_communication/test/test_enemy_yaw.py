#!/usr/bin/env python3
"""
测试 enemy_yaw 计算逻辑
模拟不同车头朝向和敌人位置，验证夹角计算是否正确
"""

import math

def get_enemy_yaw(gimble_yaw, robot_x, robot_y, enemy_x, enemy_y):
    """复刻 C++ 的 get_enemy_yaw 逻辑"""
    if enemy_x < -0.5 and enemy_y < -0.5:
        print("  → 无合适目标，返回 0.0")
        return 0.0
    
    d_x = enemy_x - robot_x
    d_y = enemy_y - robot_y
    yaw = math.atan2(d_y, d_x)
    turn_yaw = gimble_yaw - yaw
    
    # 归一化到 [-π, π]
    while turn_yaw > math.pi:
        turn_yaw -= 2.0 * math.pi
    while turn_yaw < -math.pi:
        turn_yaw += 2.0 * math.pi
    
    print(f"  → robot=({robot_x:.1f},{robot_y:.1f}) enemy=({enemy_x:.1f},{enemy_y:.1f})")
    print(f"  → gimble_yaw={gimble_yaw:.3f} rad ({math.degrees(gimble_yaw):.1f}°)")
    print(f"  → target_yaw={yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
    print(f"  → turn_yaw={turn_yaw:.3f} rad ({math.degrees(turn_yaw):.1f}°)")
    return turn_yaw

def check(name, expected, actual, tol=0.01):
    ok = abs(expected - actual) < tol
    status = "✅" if ok else "❌"
    print(f"  {status} {name}: expected={expected:.3f} got={actual:.3f}")

print("=" * 60)
print("场景1: 敌人在车头正前方")
print("  车朝右(0°)，敌人在右边")
yaw = get_enemy_yaw(0.0, 5.0, 5.0, 8.0, 5.0)
check("正前方", 0.0, yaw)

print()
print("场景2: 敌人在车头左边90°")
print("  车朝右(0°)，敌人在上方")
yaw = get_enemy_yaw(0.0, 5.0, 5.0, 5.0, 8.0)
check("左90°", -math.pi/2, yaw)  # atan2(3,0)=pi/2, turn=0-pi/2=-pi/2

print()
print("场景3: 敌人在车头右边90°")
print("  车朝右(0°)，敌人在下方")
yaw = get_enemy_yaw(0.0, 5.0, 5.0, 5.0, 2.0)
check("右90°", math.pi/2, yaw)  # atan2(-3,0)=-pi/2, turn=0-(-pi/2)=pi/2

print()
print("场景4: 敌人在车头正后方")
print("  车朝右(0°)，敌人在左边")
yaw = get_enemy_yaw(0.0, 5.0, 5.0, 2.0, 5.0)
check("正后方", math.pi, yaw)  # atan2(0,-3)=pi, turn=0-pi=-pi → normalize to pi

print()
print("场景5: 车朝上(π/2)，敌人在右边")
yaw = get_enemy_yaw(math.pi/2, 5.0, 5.0, 8.0, 5.0)
check("车朝上敌在右", math.pi/2, yaw)  # target=0, turn=pi/2-0=pi/2

print()
print("场景6: 跨越±π边界")
print("  车朝左略偏上(3.0)，敌人在左略偏下(-3.0)")
yaw = get_enemy_yaw(3.0, 5.0, 5.0, 2.0, 2.0)
# gimble=3.0, target=atan2(-3,-3)=-2.356, turn=3.0-(-2.356)=5.356 → normalize
print(f"  → 归一化后 turn_yaw={yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
print(f"  → 在[-π,π]范围内: {'✅' if -math.pi <= yaw <= math.pi else '❌'}")

print()
print("场景7: 无效目标(-1,-1)")
yaw = get_enemy_yaw(0.5, 5.0, 5.0, -1.0, -1.0)

print()
print("=" * 60)
print("场景8: 模拟数据流 - updateBattleInfo → sendLoop")
print("  串口传来敌人真实坐标(10.0, 8.0)")
print("  origin偏移: x=-3.8, y=-7.5")

# 模拟 updateBattleInfo 的坐标偏移
origin_x, origin_y = -3.8, -7.5
enemy_real_x, enemy_real_y = 10.0, 8.0
enemy_slam_x = enemy_real_x - origin_x  # 13.8
enemy_slam_y = enemy_real_y - origin_y  # 15.5
print(f"  SLAM坐标: ({enemy_slam_x:.1f}, {enemy_slam_y:.1f})")

# 模拟 robot_pos_ 从 TF 拿到的 SLAM 坐标
robot_slam_x, robot_slam_y = 5.0, 8.0
gimble_yaw = 1.0  # 车头朝向

print(f"  车位置(SLAM): ({robot_slam_x:.1f}, {robot_slam_y:.1f})")
print(f"  车头朝向: {gimble_yaw:.3f} rad ({math.degrees(gimble_yaw):.1f}°)")

yaw = get_enemy_yaw(gimble_yaw, robot_slam_x, robot_slam_y, enemy_slam_x, enemy_slam_y)
print(f"  → 发给电控的 frame.yaw = {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")

print()
print("全部测试完成！")
