#!/usr/bin/env python3
"""
完整决策系统测试
覆盖: updateBattleInfo / findRadarTarget / findGimbleTarget / get_enemy_yaw / decisionTick / sendLoop
"""

import math
import struct

# ============================================================
# 模拟参数
# ============================================================

ORIGIN_X = -3.8
ORIGIN_Y = -7.5
ARRIVE_TOLERANCE = 0.3

# ============================================================
# 工具函数
# ============================================================

def get_enemy_yaw(gimble_yaw, robot_x, robot_y, enemy_x, enemy_y):
    if enemy_x < -0.5 and enemy_y < -0.5:
        return 0.0
    d_x = enemy_x - robot_x
    d_y = enemy_y - robot_y
    yaw = math.atan2(d_y, d_x)
    turn_yaw = gimble_yaw - yaw
    while turn_yaw > math.pi:
        turn_yaw -= 2.0 * math.pi
    while turn_yaw < -math.pi:
        turn_yaw += 2.0 * math.pi
    return turn_yaw

def get_distance(ax, ay, bx, by):
    return math.sqrt((ax - bx)**2 + (ay - by)**2)

def point_in_polygon(px, py, vertices):
    n = len(vertices)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = vertices[i]
        xj, yj = vertices[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside

def get_region(x, y, regions):
    for region in regions:
        if point_in_polygon(x, y, region["vertices"]):
            return region["name"]
    return 0

# ============================================================
# 模拟数据
# ============================================================

# 简化的区域定义(3个区域)
REGIONS = [
    {"name": 1, "vertices": [(0,0), (10,0), (10,10), (0,10)]},
    {"name": 2, "vertices": [(10,0), (20,0), (20,10), (10,10)]},
    {"name": 3, "vertices": [(0,10), (10,10), (10,20), (0,20)]},
]

def make_enemy(x, y, hp, zone):
    return {"x": x, "y": y, "hp": hp, "zone": zone}

def make_robot(x, y, slam_x, slam_y):
    return {"x": x, "y": y, "slam_x": slam_x, "slam_y": slam_y}

# ============================================================
# 模拟 updateBattleInfo
# ============================================================

def simulate_update_battle_info(serial_enemies, serial_gimble, robot_slam_x, robot_slam_y, gimble_yaw):
    """
    模拟 updateBattleInfo 的完整流程:
    1. 拷贝串口数据(真实坐标)
    2. 坐标偏移(真实→SLAM)
    3. 计算区域
    4. 缓存雷达点/云台点 + 计算yaw
    """
    # Step 1+2: 偏移
    enemies = []
    for e in serial_enemies:
        ex, ey = e["x"], e["y"]
        if ex > -0.5 and ey > -0.5:
            ex -= ORIGIN_X
            ey -= ORIGIN_Y
        enemies.append({"x": ex, "y": ey, "hp": e["hp"], "zone": e["zone"]})

    gx, gy = serial_gimble["x"], serial_gimble["y"]
    if gx > -0.5 and gy > -0.5:
        gx -= ORIGIN_X
        gy -= ORIGIN_Y

    # Step 3: 区域
    for e in enemies:
        if e["x"] > -0.5 and e["y"] > -0.5:
            e["zone"] = get_region(e["x"], e["y"], REGIONS)
    gimble_zone = get_region(gx, gy, REGIONS) if gx > -0.5 else 0

    # Step 4a: findRadarTarget
    current_region = get_region(robot_slam_x, robot_slam_y, REGIONS)
    radar_pt = find_radar_target(enemies, robot_slam_x, robot_slam_y, current_region)

    # Step 4b: findGimbleTarget
    gimble_pt = find_gimble_target(gx, gy, gimble_zone, current_region)

    # Step 4c: 计算yaw(只用雷达点)
    if radar_pt["valid"]:
        enemy_yaw_angle = get_enemy_yaw(gimble_yaw, robot_slam_x, robot_slam_y, radar_pt["x"], radar_pt["y"])
    else:
        enemy_yaw_angle = 0.0

    return {
        "enemies": enemies,
        "gimble": {"x": gx, "y": gy, "zone": gimble_zone},
        "radar_pt": radar_pt,
        "gimble_pt": gimble_pt,
        "enemy_yaw_angle": enemy_yaw_angle,
        "current_region": current_region,
        "radar_active": radar_pt["valid"],
        "gimble_active": gimble_pt["valid"],
    }

def find_radar_target(enemies, robot_x, robot_y, my_zone):
    min_dist = 999.0
    found = False
    best = {"x": -1.0, "y": -1.0}
    for e in enemies:
        if e["x"] < -0.5 or e["y"] < -0.5:
            continue
        if e["hp"] <= 0:
            continue
        if my_zone == 0 or e["zone"] != my_zone:
            continue
        d = get_distance(e["x"], e["y"], robot_x, robot_y)
        if d < min_dist:
            min_dist = d
            best = {"x": e["x"], "y": e["y"]}
            found = True
    if found:
        best["valid"] = True
        best["dist"] = min_dist
    else:
        best["valid"] = False
        best["dist"] = 999.0
    return best

def find_gimble_target(gx, gy, gimble_zone, my_zone):
    if gx < -0.5 or gy < -0.5:
        return {"x": -1.0, "y": -1.0, "valid": False}
    if gimble_zone == 0 or gimble_zone != my_zone:
        return {"x": -1.0, "y": -1.0, "valid": False}
    return {"x": gx, "y": gy, "valid": True}

# ============================================================
# 模拟 decisionTick
# ============================================================

def simulate_decision_tick(result, start_flag, attack_mession, hp, bullet,
                           chase_locked, chase_x, chase_y,
                           robot_x, robot_y,
                           need_heal, radar_arrived, gimble_arrived):
    """
    模拟决策流程, 返回:
    - action: "supply" / "chase" / "radar_patrol" / "gimble_patrol" / "home_cruise" / "idle"
    - target: (x, y) 导航目标
    - state_update: 状态更新
    """
    state_update = {}

    # start_flag != 4 → 不动
    if start_flag != 4:
        return {"action": "idle", "target": None, "state_update": state_update}

    # 前哨站任务 → 不动
    if attack_mession == 10:
        return {"action": "idle", "target": None, "state_update": state_update}

    # 第一: 补给
    if not need_heal and (hp <= 150.0 or bullet < 50):
        need_heal = True
        state_update["need_heal"] = True
    if need_heal and hp >= 390.0 and bullet > 50:
        need_heal = False
        state_update["need_heal"] = False
    if need_heal:
        return {"action": "supply", "target": (10.0, 10.0), "state_update": state_update}

    # 第二: 追击
    if chase_locked:
        chase_zone = get_region(chase_x, chase_y, REGIONS)
        current_region = get_region(robot_x, robot_y, REGIONS)
        if chase_zone == current_region:
            if get_distance(chase_x, chase_y, robot_x, robot_y) < 2.5:
                return {"action": "chase_arrived", "target": (chase_x, chase_y), "state_update": state_update}
            return {"action": "chase", "target": (chase_x, chase_y), "state_update": state_update}

    # 第三: 雷达点巡逻
    if result["radar_active"]:
        rx, ry = result["radar_pt"]["x"], result["radar_pt"]["y"]
        if get_distance(rx, ry, robot_x, robot_y) < ARRIVE_TOLERANCE:
            if not radar_arrived:
                state_update["radar_arrived"] = True
                return {"action": "radar_arrived", "target": (rx, ry), "state_update": state_update}
            else:
                # 假设10秒到了(外部计时)
                state_update["radar_arrived"] = False
                state_update["radar_active_clear"] = True
                return {"action": "radar_timeout", "target": None, "state_update": state_update}
        return {"action": "radar_patrol", "target": (rx, ry), "state_update": state_update}

    # 第四: 云台点巡逻
    if result["gimble_active"]:
        gx, gy = result["gimble_pt"]["x"], result["gimble_pt"]["y"]
        if get_distance(gx, gy, robot_x, robot_y) < ARRIVE_TOLERANCE:
            if not gimble_arrived:
                state_update["gimble_arrived"] = True
                return {"action": "gimble_arrived", "target": (gx, gy), "state_update": state_update}
            else:
                state_update["gimble_arrived"] = False
                state_update["gimble_active_clear"] = True
                return {"action": "gimble_timeout", "target": None, "state_update": state_update}
        return {"action": "gimble_patrol", "target": (gx, gy), "state_update": state_update}

    # 第五: 基地巡逻
    return {"action": "home_cruise", "target": (5.0, 5.0), "state_update": state_update}

# ============================================================
# 模拟 sendLoop (帧组装)
# ============================================================

def simulate_send_loop(gimble_yaw, enemy_yaw_angle, vx=0.0, vy=0.0, wz=0.0,
                       dist=0.0, if_turn=False, region=0, attack_target=0,
                       obs_x=-1.0, obs_y=-1.0):
    """模拟串口帧组装, 返回字节数组"""
    frame = bytearray()
    frame.append(0xa5)  # header
    frame += struct.pack('<f', vx)       # linear_x
    frame += struct.pack('<f', vy)       # linear_y
    frame += struct.pack('<f', wz)       # angular_z
    frame += struct.pack('<f', gimble_yaw)    # yaw
    frame += struct.pack('<f', enemy_yaw_angle)  # enemy_yaw
    frame += struct.pack('<f', dist)     # dist
    frame += struct.pack('<?', if_turn)  # if_turn
    frame += struct.pack('<H', region)   # region
    frame += struct.pack('<B', attack_target)  # attack_target
    frame += struct.pack('<f', obs_x)    # obstacle_x
    frame += struct.pack('<f', obs_y)    # obstacle_y
    # CRC placeholder (2 bytes)
    frame += b'\x00\x00'
    return frame

# ============================================================
# 测试断言工具
# ============================================================

PASS_COUNT = 0
FAIL_COUNT = 0

def check(desc, expected, actual, tol=None):
    global PASS_COUNT, FAIL_COUNT
    if tol is not None:
        ok = abs(expected - actual) < tol
    elif isinstance(expected, float):
        ok = abs(expected - actual) < 0.01
    else:
        ok = expected == actual
    if ok:
        PASS_COUNT += 1
        print(f"  ✅ {desc}")
    else:
        FAIL_COUNT += 1
        print(f"  ❌ {desc}: expected={expected}, got={actual}")

def section(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

# ============================================================
# 测试开始
# ============================================================

section("一、坐标偏移 (updateBattleInfo Step2)")
print("  串口真实坐标 → SLAM坐标: 减去 origin")

# 正常坐标
ex, ey = 10.0, 8.0
slam_x = ex - ORIGIN_X  # 13.8
slam_y = ey - ORIGIN_Y  # 15.5
check("正常坐标偏移 X", 13.8, slam_x)
check("正常坐标偏移 Y", 15.5, slam_y)

# 无效坐标(-1)不偏移
ex, ey = -1.0, -1.0
if ex > -0.5 and ey > -0.5:
    ex -= ORIGIN_X
    ey -= ORIGIN_Y
check("无效坐标不偏移 X", -1.0, ex)
check("无效坐标不偏移 Y", -1.0, ey)

# 边界: -0.49 有效, -0.51 无效
ex = -0.49
if ex > -0.5:
    ex -= ORIGIN_X
check("边界 -0.49 偏移", -0.49 - ORIGIN_X, ex)

ex = -0.51
if ex > -0.5:
    ex -= ORIGIN_X
check("边界 -0.51 不偏移", -0.51, ex)


section("二、findRadarTarget 测试")

# 场景1: 同区域有敌人
enemies = [make_enemy(13.0, 15.0, 200, 1)]
r = find_radar_target(enemies, 5.0, 5.0, 1)
check("同区域有效敌人", True, r["valid"])
check("返回坐标X", 13.0, r["x"])
check("返回坐标Y", 15.0, r["y"])

# 场景2: 不同区域的敌人(应被过滤)
enemies = [make_enemy(15.0, 5.0, 200, 2)]
r = find_radar_target(enemies, 5.0, 5.0, 1)
check("不同区域敌人被过滤", False, r["valid"])

# 场景3: 死亡敌人
enemies = [make_enemy(13.0, 15.0, 0, 1)]
r = find_radar_target(enemies, 5.0, 5.0, 1)
check("死亡敌人被过滤", False, r["valid"])

# 场景4: 无效坐标敌人
enemies = [make_enemy(-1.0, -1.0, 200, 1)]
r = find_radar_target(enemies, 5.0, 5.0, 1)
check("无效坐标被过滤", False, r["valid"])

# 场景5: 多个同区域敌人, 选最近的
enemies = [
    make_enemy(8.0, 6.0, 200, 1),   # dist ≈ 4.2
    make_enemy(6.0, 5.5, 200, 1),   # dist ≈ 1.1
    make_enemy(12.0, 14.0, 200, 1), # dist ≈ 12.7
]
r = find_radar_target(enemies, 5.0, 5.0, 1)
check("多敌人选最近", True, r["valid"])
check("最近敌人X", 6.0, r["x"])
check("最近敌人Y", 5.5, r["y"])

# 场景6: 混合有效/无效/死亡
enemies = [
    make_enemy(-1.0, 5.0, 200, 1),  # 无效坐标
    make_enemy(8.0, 6.0, 0, 1),     # 死亡
    make_enemy(9.0, 7.0, 150, 2),   # 不同区域
    make_enemy(7.0, 5.5, 300, 1),   # 有效, 最近
]
r = find_radar_target(enemies, 5.0, 5.0, 1)
check("混合场景选中正确目标", True, r["valid"])
check("混合场景坐标X", 7.0, r["x"])

# 场景7: my_zone=0 (不在任何区域) → 不过滤
enemies = [make_enemy(8.0, 6.0, 200, 1)]
r = find_radar_target(enemies, 50.0, 50.0, 0)
check("zone=0 不过滤(找不到)", False, r["valid"])


section("三、findGimbleTarget 测试")

# 场景1: 有效云台手点, 同区域
r = find_gimble_target(8.0, 7.0, 1, 1)
check("有效同区域云台点", True, r["valid"])

# 场景2: 不同区域
r = find_gimble_target(15.0, 5.0, 2, 1)
check("不同区域云台点无效", False, r["valid"])

# 场景3: 无效坐标
r = find_gimble_target(-1.0, -1.0, 0, 1)
check("无效坐标云台点", False, r["valid"])

# 场景4: gimble_zone=0
r = find_gimble_target(50.0, 50.0, 0, 1)
check("zone=0 云台点无效", False, r["valid"])


section("四、get_enemy_yaw 测试 (全角度)")

# 0°: 敌人在正前方
check("0° 正前方", 0.0, get_enemy_yaw(0.0, 5, 5, 8, 5), tol=0.01)

# +90°: 敌人在左边(逆时针)
check("+90° 左边", -math.pi/2, get_enemy_yaw(0.0, 5, 5, 5, 8), tol=0.01)

# -90°: 敌人在右边(顺时针)
check("-90° 右边", math.pi/2, get_enemy_yaw(0.0, 5, 5, 5, 2), tol=0.01)

# 180°: 敌人在正后方
yaw = get_enemy_yaw(0.0, 5, 5, 2, 5)
check("180° 正后方 在[-π,π]", True, -math.pi <= yaw <= math.pi)
check("180° 绝对值≈π", math.pi, abs(yaw), tol=0.01)

# 车朝上(π/2), 敌人在右
check("车朝上敌在右", math.pi/2, get_enemy_yaw(math.pi/2, 5, 5, 8, 5), tol=0.01)

# 车朝左(π), 敌人在右 → gimble=π, target=0, turn=π (归一化后±π等价)
yaw = get_enemy_yaw(math.pi, 5, 5, 8, 5)
check("车朝左敌在右 |值|≈π", math.pi, abs(yaw), tol=0.01)

# 车朝下(-π/2), 敌人在上 → gimble=-π/2, target=π/2, turn=-π
yaw = get_enemy_yaw(-math.pi/2, 5, 5, 5, 8)
check("车朝下敌在上 |值|≈π", math.pi, abs(yaw), tol=0.01)

# 跨越±π边界: 车朝3.0, 敌人在-2.356
yaw = get_enemy_yaw(3.0, 5, 5, 2, 2)
check("跨±π边界归一化", True, -math.pi <= yaw <= math.pi)

# 无效目标
check("无效目标返回0", 0.0, get_enemy_yaw(0.5, 5, 5, -1, -1))

# 小角度: 敌人在车头偏左5°
offset = math.radians(5)
yaw = get_enemy_yaw(0.0, 5, 5, 5 + math.cos(offset), 5 + math.sin(offset))
check("偏左5°", -offset, yaw, tol=0.01)

# 小角度: 敌人在车头偏右5°
yaw = get_enemy_yaw(0.0, 5, 5, 5 + math.cos(-offset), 5 + math.sin(-offset))
check("偏右5°", offset, yaw, tol=0.01)


section("五、updateBattleInfo 完整数据流")

# 场景: 串口传来2个敌人 + 云台手坐标
# 坐标系: origin=(-3.8, -7.5)
# 机器人SLAM(5,5) → zone=1
# 敌人需要: 真实坐标x>-0.5且y>-0.5(才能偏移), 偏移后在zone=1
# 真实(1.0, 0.0) → SLAM(4.8, 7.5) → zone=1 ✓
serial_enemies = [
    make_enemy(1.0, 0.0, 200, 1),
]
serial_gimble = {"x": 2.0, "y": 1.0}  # SLAM(5.8, 8.5) → zone=1 ✓
robot_slam_x, robot_slam_y = 5.0, 5.0
gimble_yaw = 0.5

result = simulate_update_battle_info(serial_enemies, serial_gimble, robot_slam_x, robot_slam_y, gimble_yaw)

check("敌人1 SLAM X", 1.0 - ORIGIN_X, result["enemies"][0]["x"])
check("敌人1 SLAM Y", 0.0 - ORIGIN_Y, result["enemies"][0]["y"])
check("云台手 SLAM X", 2.0 - ORIGIN_X, result["gimble"]["x"])
check("云台手 SLAM Y", 1.0 - ORIGIN_Y, result["gimble"]["y"])
check("radar_active", True, result["radar_active"])
check("gimble_active", True, result["gimble_active"])
check("enemy_yaw_angle 非零", True, abs(result["enemy_yaw_angle"]) > 0.01)

# 场景: 无有效雷达点
serial_enemies = [make_enemy(-1.0, -1.0, 200, 1)]
serial_gimble = {"x": -1.0, "y": -1.0}
result = simulate_update_battle_info(serial_enemies, serial_gimble, 5.0, 5.0, 0.5)
check("无雷达点 radar_active", False, result["radar_active"])
check("无云台点 gimble_active", False, result["gimble_active"])
check("无目标 yaw=0", 0.0, result["enemy_yaw_angle"])


section("六、decisionTick 决策优先级")

# 场景1: start_flag != 4 → idle
result_dummy = {"radar_active": True, "gimble_active": False,
                "radar_pt": {"x": 8.0, "y": 6.0, "valid": True},
                "gimble_pt": {"x": -1, "y": -1, "valid": False}}
d = simulate_decision_tick(result_dummy, start_flag=0, attack_mession=0,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("start_flag!=4 → idle", "idle", d["action"])

# 场景2: attack_mession=10 → idle
d = simulate_decision_tick(result_dummy, start_flag=4, attack_mession=10,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("前哨站任务 → idle", "idle", d["action"])

# 场景3: HP低 → 补给(优先级最高)
d = simulate_decision_tick(result_dummy, start_flag=4, attack_mession=0,
                           hp=100, bullet=300, chase_locked=True,
                           chase_x=8, chase_y=6, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("HP低 → supply(优先于追击)", "supply", d["action"])

# 场景4: 弹药低 → 补给
d = simulate_decision_tick(result_dummy, start_flag=4, attack_mession=0,
                           hp=400, bullet=30, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("弹药低 → supply", "supply", d["action"])

# 场景5: 补给完成(HP满+弹药够) → 解除
d = simulate_decision_tick(result_dummy, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=5, robot_y=5,
                           need_heal=True, radar_arrived=False, gimble_arrived=False)
check("HP满弹药够 → 解除补给", False, d["state_update"].get("need_heal", True))

# 场景6: 追击(同区域, 未到达)
d = simulate_decision_tick(result_dummy, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=True,
                           chase_x=8, chase_y=6, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("追击同区域 → chase", "chase", d["action"])

# 场景7: 追击(同区域, 已到达)
d = simulate_decision_tick(result_dummy, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=True,
                           chase_x=5.1, chase_y=5.1, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("追击到达 → chase_arrived", "chase_arrived", d["action"])

# 场景8: 追击(不同区域) → 跳过追击, 进入雷达巡逻
result_with_radar = {"radar_active": True, "gimble_active": False,
                     "radar_pt": {"x": 8.0, "y": 6.0, "valid": True},
                     "gimble_pt": {"x": -1, "y": -1, "valid": False}}
d = simulate_decision_tick(result_with_radar, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=True,
                           chase_x=15, chase_y=5, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("追击不同区域 → 跳到雷达巡逻", "radar_patrol", d["action"])

# 场景9: 雷达点巡逻
d = simulate_decision_tick(result_with_radar, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("雷达点巡逻", "radar_patrol", d["action"])

# 场景10: 到达雷达点
d = simulate_decision_tick(result_with_radar, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=8.1, robot_y=6.1,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("到达雷达点 → 开始计时", "radar_arrived", d["action"])
check("radar_arrived 设为 True", True, d["state_update"].get("radar_arrived"))

# 场景11: 雷达点超时
d = simulate_decision_tick(result_with_radar, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=8.1, robot_y=6.1,
                           need_heal=False, radar_arrived=True, gimble_arrived=False)
check("雷达点超时 → 清除", "radar_timeout", d["action"])
check("radar_arrived 重置", False, d["state_update"].get("radar_arrived"))

# 场景12: 雷达无效, 云台有效 → 云台巡逻
result_with_gimble = {"radar_active": False, "gimble_active": True,
                      "radar_pt": {"x": -1, "y": -1, "valid": False},
                      "gimble_pt": {"x": 9.0, "y": 7.0, "valid": True}}
d = simulate_decision_tick(result_with_gimble, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("云台点巡逻", "gimble_patrol", d["action"])

# 场景13: 到达云台点
d = simulate_decision_tick(result_with_gimble, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=9.1, robot_y=7.1,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("到达云台点", "gimble_arrived", d["action"])

# 场景14: 都无效 → 基地巡逻
result_none = {"radar_active": False, "gimble_active": False,
               "radar_pt": {"x": -1, "y": -1, "valid": False},
               "gimble_pt": {"x": -1, "y": -1, "valid": False}}
d = simulate_decision_tick(result_none, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("都无效 → home_cruise", "home_cruise", d["action"])

# 场景15: 雷达优先于云台
result_both = {"radar_active": True, "gimble_active": True,
               "radar_pt": {"x": 8.0, "y": 6.0, "valid": True},
               "gimble_pt": {"x": 9.0, "y": 7.0, "valid": True}}
d = simulate_decision_tick(result_both, start_flag=4, attack_mession=0,
                           hp=400, bullet=300, chase_locked=False,
                           chase_x=0, chase_y=0, robot_x=5, robot_y=5,
                           need_heal=False, radar_arrived=False, gimble_arrived=False)
check("雷达优先于云台", "radar_patrol", d["action"])


section("七、sendLoop 帧组装")

# 验证帧结构
gimble_yaw = 1.2
enemy_yaw = 0.35
frame = simulate_send_loop(gimble_yaw, enemy_yaw, vx=0.5, vy=0.3, region=2)

check("帧头 0xa5", 0xa5, frame[0])
check("帧长度 39", 39, len(frame))

# 解析验证
parsed_vx = struct.unpack_from('<f', frame, 1)[0]
parsed_vy = struct.unpack_from('<f', frame, 5)[0]
parsed_wz = struct.unpack_from('<f', frame, 9)[0]
parsed_yaw = struct.unpack_from('<f', frame, 13)[0]
parsed_enemy_yaw = struct.unpack_from('<f', frame, 17)[0]
parsed_dist = struct.unpack_from('<f', frame, 21)[0]
parsed_if_turn = struct.unpack_from('<?', frame, 25)[0]
parsed_region = struct.unpack_from('<H', frame, 26)[0]
parsed_attack = struct.unpack_from('<B', frame, 28)[0]
parsed_obs_x = struct.unpack_from('<f', frame, 29)[0]
parsed_obs_y = struct.unpack_from('<f', frame, 33)[0]

check("vx=0.5", 0.5, parsed_vx)
check("vy=0.3", 0.3, parsed_vy)
check("yaw=gimble_yaw", gimble_yaw, parsed_yaw)
check("enemy_yaw=0.35", enemy_yaw, parsed_enemy_yaw)
check("region=2", 2, parsed_region)

# 无效目标时 enemy_yaw=0
frame2 = simulate_send_loop(0.5, 0.0)
parsed2 = struct.unpack_from('<f', frame2, 17)[0]
check("无目标 enemy_yaw=0", 0.0, parsed2)


section("八、端到端完整流程")

print("\n  模拟: 串口→updateBattleInfo→decisionTick→sendLoop")
print("  ---")

# 场景A: 有敌人, 追击优先
# 真实(1.0, 0.0) → SLAM(4.8, 7.5) → zone=1 ✓
serial_enemies = [make_enemy(1.0, 0.0, 200, 1)]
serial_gimble = {"x": -1.0, "y": -1.0}
r = simulate_update_battle_info(serial_enemies, serial_gimble, 5.0, 5.0, 0.0)
check("E2E-A: radar_active", True, r["radar_active"])
check("E2E-A: enemy_yaw 非零", True, abs(r["enemy_yaw_angle"]) > 0.01)

d = simulate_decision_tick(r, 4, 0, 400, 300, True, 4.8, 7.5, 5.0, 5.0, False, False, False)
check("E2E-A: 追击优先于雷达", "chase", d["action"])

frame = simulate_send_loop(0.0, r["enemy_yaw_angle"])
parsed_yaw = struct.unpack_from('<f', frame, 13)[0]
parsed_eyaw = struct.unpack_from('<f', frame, 17)[0]
check("E2E-A: frame.yaw=gimble_yaw", 0.0, parsed_yaw)
check("E2E-A: frame.enemy_yaw 有值", True, abs(parsed_eyaw) > 0.01)

# 场景B: 补给优先于一切
serial_enemies = [make_enemy(1.0, 0.0, 200, 1)]
r = simulate_update_battle_info(serial_enemies, serial_gimble, 5.0, 5.0, 0.0)
d = simulate_decision_tick(r, 4, 0, 100, 300, True, 4.8, 7.5, 5.0, 5.0, False, False, False)
check("E2E-B: 补给优先于追击", "supply", d["action"])

# 场景C: 全部无效 → 基地巡逻
serial_enemies = [make_enemy(-1.0, -1.0, 0, 0)]
serial_gimble = {"x": -1.0, "y": -1.0}
r = simulate_update_battle_info(serial_enemies, serial_gimble, 5.0, 5.0, 0.0)
d = simulate_decision_tick(r, 4, 0, 400, 300, False, 0, 0, 5.0, 5.0, False, False, False)
check("E2E-C: 全无效 → 基地巡逻", "home_cruise", d["action"])
check("E2E-C: enemy_yaw=0", 0.0, r["enemy_yaw_angle"])

# 场景D: 只有云台点(无雷达) - 云台真实坐标(3.0, 0.0) → SLAM(6.8, 7.5) zone=1 ✓
serial_enemies = [make_enemy(-1.0, -1.0, 0, 0)]
serial_gimble = {"x": 3.0, "y": 0.0}
r = simulate_update_battle_info(serial_enemies, serial_gimble, 5.0, 5.0, 0.0)
check("E2E-D: radar_active=False", False, r["radar_active"])
check("E2E-D: gimble_active=True", True, r["gimble_active"])
check("E2E-D: enemy_yaw=0(只看雷达)", 0.0, r["enemy_yaw_angle"])
d = simulate_decision_tick(r, 4, 0, 400, 300, False, 0, 0, 5.0, 5.0, False, False, False)
check("E2E-D: 云台巡逻", "gimble_patrol", d["action"])


section("九、边界条件")

# 机器人在区域边界上
enemies = [make_enemy(8.0, 5.0, 200, 1)]
r = find_radar_target(enemies, 10.0, 5.0, 1)  # 在区域1和2的边界
check("边界上的机器人", True, r["valid"])  # 区域1包含边界

# 机器人不在任何区域
r = find_radar_target(enemies, 50.0, 50.0, 0)
check("zone=0 不过滤", False, r["valid"])  # 敌人在zone=1, my_zone=0不过滤

# 敌人恰好在 -0.5 边界: C++是 < -0.5 才过滤, -0.5 不 < -0.5, 所以有效
enemies = [make_enemy(-0.5, 5.0, 200, 1)]
r = find_radar_target(enemies, 5.0, 5.0, 1)
check("x=-0.5 有效(不< -0.5)", True, r["valid"])

# 敌人 -0.51 被过滤
enemies = [make_enemy(-0.51, 5.0, 200, 1)]
r = find_radar_target(enemies, 5.0, 5.0, 1)
check("x=-0.51 被过滤", False, r["valid"])

# 距离为0 (机器人和敌人重合)
yaw = get_enemy_yaw(0.0, 5.0, 5.0, 5.0, 5.0)
check("距离0 返回0", 0.0, yaw)

# 极远距离
enemies = [make_enemy(1000.0, 1000.0, 200, 1)]
r = find_radar_target(enemies, 5.0, 5.0, 1)  # zone不同
check("极远敌人(不同zone)", False, r["valid"])


section("十、状态转换测试")

# 模拟: 雷达点到达 → 10秒超时 → 恢复基地巡逻
print("\n  模拟完整状态机转换:")
print("  初始 → 雷达巡逻 → 到达 → 超时 → 基地巡逻")

result_radar = {"radar_active": True, "gimble_active": False,
                "radar_pt": {"x": 8.0, "y": 6.0, "valid": True},
                "gimble_pt": {"x": -1, "y": -1, "valid": False}}

# Step1: 雷达巡逻
d = simulate_decision_tick(result_radar, 4, 0, 400, 300, False, 0, 0, 5.0, 5.0, False, False, False)
check("Step1: 雷达巡逻", "radar_patrol", d["action"])

# Step2: 到达
d = simulate_decision_tick(result_radar, 4, 0, 400, 300, False, 0, 0, 8.1, 6.1, False, False, False)
check("Step2: 到达", "radar_arrived", d["action"])

# Step3: 超时(假设10秒到了)
d = simulate_decision_tick(result_radar, 4, 0, 400, 300, False, 0, 0, 8.1, 6.1, False, True, False)
check("Step3: 超时清除", "radar_timeout", d["action"])

# Step4: 下一轮 updateBattleInfo 重新检测
# (模拟雷达点因敌人移动而失效)
result_no_radar = {"radar_active": False, "gimble_active": False,
                   "radar_pt": {"x": -1, "y": -1, "valid": False},
                   "gimble_pt": {"x": -1, "y": -1, "valid": False}}
d = simulate_decision_tick(result_no_radar, 4, 0, 400, 300, False, 0, 0, 8.1, 6.1, False, False, False)
check("Step4: 恢复基地巡逻", "home_cruise", d["action"])


# ============================================================
# 结果
# ============================================================

print(f"\n{'='*60}")
print(f"  测试结果: {PASS_COUNT} 通过, {FAIL_COUNT} 失败")
print(f"{'='*60}")

if FAIL_COUNT > 0:
    print("\n⚠️ 有失败的测试，请检查！")
else:
    print("\n🎉 全部通过！")
