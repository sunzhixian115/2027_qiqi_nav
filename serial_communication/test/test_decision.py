#!/usr/bin/env python3
"""
decisionTick 模拟测试
模拟不同数据流，验证决策逻辑是否正确
"""

import math
import time

# ============================================================
# 模拟数据结构
# ============================================================

class Vec2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

class SimState:
    """模拟 RefereeSystemNode 的状态"""
    def __init__(self):
        # 串口数据
        self.start_flag = 4
        self.attack_mession = 1
        self.second = 420
        self.bullet = 200
        self.my_hp = 400.0
        self.gimble_x = -1.0
        self.gimble_y = -1.0

        # 敌方数据
        self.enemies = []

        # 机器人状态
        self.robot_pos = Vec2(3.0, 5.0)
        self.current_region = 1

        # 决策状态
        self.decision_mode = "BASE_MODE"
        self.need_heal = False
        self.chase_locked = False
        self.chase_point = Vec2()
        self.chase_region = 0

        # 雷达/云台状态
        self.radar_active = False
        self.radar_arrive_time = 0.0
        self.radar_saved_point = Vec2(-1.0, -1.0)

        self.gimble_active = False
        self.gimble_arrive_time = 0.0
        self.gimble_saved_point = Vec2(-1.0, -1.0)

        # 点位
        self.supply_point = Vec2(0.5, 0.0)
        self.pre_point = Vec2(4.89, -2.08)
        self.z1 = Vec2(6.5, -1.0)
        self.z2 = Vec2(9.0, 3.0)
        self.back_point = Vec2(11.0, 5.5)
        self.attack_points = [Vec2(12.0, -3.0), Vec2(11.7, 0.0), Vec2(16.4, -2.5)]
        self.high_points = [Vec2(12.0, 6.0), Vec2(14.0, 5.0), Vec2(13.0, 8.0)]

        # 结果记录
        self.actions = []

    def get_distance(self, a, b):
        dx = a.x - b.x
        dy = a.y - b.y
        return math.sqrt(dx*dx + dy*dy)

    def get_region(self, p):
        # 简化：根据x坐标判断区域
        if p.x < 5.0:
            return 1  # 基地区
        elif p.x < 9.0:
            return 2  # 通道
        else:
            return 3  # 高地

    def find_radar_target(self, my_zone):
        """找同区域最近敌人"""
        min_dist = 999.0
        best = Vec2(-1.0, -1.0)
        for e in self.enemies:
            if e['x'] < -0.5 or e['y'] < -0.5:
                continue
            if e['hp'] <= 0:
                continue
            if e['zone'] != my_zone:
                continue
            dx = e['x'] - self.robot_pos.x
            dy = e['y'] - self.robot_pos.y
            d = math.sqrt(dx*dx + dy*dy)
            if d < min_dist:
                min_dist = d
                best = Vec2(e['x'], e['y'])
        return best

    def find_gimble_target(self, my_zone):
        if self.gimble_x < -0.5 or self.gimble_y < -0.5:
            return self.robot_pos
        gimble_zone = self.get_region(Vec2(self.gimble_x, self.gimble_y))
        if gimble_zone == 0 or gimble_zone != my_zone:
            return self.robot_pos
        return Vec2(self.gimble_x, self.gimble_y)

    def publish_goal(self, p, label=""):
        self.actions.append(f"  publishGoal({p.x:.1f}, {p.y:.1f}) {label}")

    def log(self, msg):
        self.actions.append(f"  [LOG] {msg}")


def decision_tick(s: SimState):
    """模拟 decisionTick 逻辑"""
    s.actions = []

    if s.start_flag != 4:
        s.actions.append("→ start_flag != 4, return")
        return

    if s.attack_mession == 10:
        s.actions.append("→ attack_mession == 10, return")
        return

    current_region = s.get_region(s.robot_pos)

    if s.decision_mode == "BASE_MODE":
        if s.second < 60:
            s.actions.append(f"→ second={s.second} < 60, go_to_high()")
            return

        # 补给
        if not s.need_heal and (s.my_hp <= 150.0 or s.bullet < 50):
            s.need_heal = True
        if s.need_heal and s.my_hp >= 390.0 and s.bullet > 50:
            s.need_heal = False
        if s.need_heal:
            s.publish_goal(s.supply_point, "补给")
            return

        # 追击
        if s.chase_locked:
            if s.chase_region == current_region:
                s.publish_goal(s.chase_point, "追击")
                if s.get_distance(s.chase_point, s.robot_pos) < 2.5:
                    s.log("到达追击点")
                return

        # 雷达
        radar_pt = s.find_radar_target(current_region)
        if radar_pt.x > -0.5 and radar_pt.y > -0.5:
            s.radar_saved_point = radar_pt

        if s.radar_saved_point.x > -0.5 and s.radar_saved_point.y > -0.5:
            s.publish_goal(s.radar_saved_point, "雷达")
            if s.get_distance(s.radar_saved_point, s.robot_pos) < 0.7:
                now = time.time()
                if not s.radar_active:
                    s.radar_arrive_time = now
                    s.radar_active = True
                    s.log("已到达雷达点，开始计时")
                elif now - s.radar_arrive_time >= 5.0:  # 测试用5秒
                    s.log("雷达点10秒结束")
                    s.radar_active = False
                    s.radar_saved_point = Vec2(-1.0, -1.0)
            return

        # 云台
        if s.gimble_x > -0.5 and s.gimble_y > -0.5:
            s.gimble_saved_point = s.find_gimble_target(current_region)

        if s.gimble_saved_point.x > -0.5 and s.gimble_saved_point.y > -0.5:
            s.publish_goal(s.gimble_saved_point, "云台")
            if s.get_distance(s.gimble_saved_point, s.robot_pos) < 0.7:
                now = time.time()
                if not s.gimble_active:
                    s.gimble_arrive_time = now
                    s.gimble_active = True
                    s.log("已到达云台点，开始计时")
                elif now - s.gimble_arrive_time >= 5.0:  # 测试用5秒
                    s.log("云台点10秒结束")
                    s.gimble_active = False
                    s.gimble_saved_point = Vec2(-1.0, -1.0)
            return

        # 巡航
        choice = ((420 - s.second) // 30) % 3
        s.publish_goal(s.attack_points[choice], f"homeCruise choice={choice}")

    else:  # HIGH_MODE
        s.actions.append("→ HIGH_MODE 巡航")
        choice = ((420 - s.second) // 30) % 3
        s.publish_goal(s.high_points[choice], f"highCruise choice={choice}")


# ============================================================
# 测试场景
# ============================================================

def test_scene(name, setup_fn):
    s = SimState()
    setup_fn(s)
    print(f"\n{'='*50}")
    print(f"场景: {name}")
    print(f"{'='*50}")
    decision_tick(s)
    for a in s.actions:
        print(a)
    if not s.actions:
        print("  (无动作)")


# 场景1: 比赛未开始
test_scene("比赛未开始 (start_flag=0)",
    lambda s: setattr(s, 'start_flag', 0))

# 场景2: 前哨站模式不动
test_scene("前哨站模式 (attack_mession=10)",
    lambda s: [setattr(s, 'attack_mession', 10)])

# 场景3: 剩余60秒上高地
test_scene("剩余50秒 → go_to_high",
    lambda s: setattr(s, 'second', 50))

# 场景4: HP低去补给
test_scene("HP=100 → 补给",
    lambda s: setattr(s, 'my_hp', 100.0))

# 场景5: 弹药低去补给
test_scene("弹药=30 → 补给",
    lambda s: setattr(s, 'bullet', 30))

# 场景6: 追击同区域
def setup_chase(s):
    s.chase_locked = True
    s.chase_point = Vec2(4.0, 6.0)
    s.chase_region = s.get_region(s.chase_point)
test_scene("追击同区域", setup_chase)

# 场景7: 追击不同区域
def setup_chase_diff(s):
    s.chase_locked = True
    s.chase_point = Vec2(12.0, 6.0)
    s.chase_region = 3
test_scene("追击不同区域(跳过)", setup_chase_diff)

# 场景8: 雷达点有效
def setup_radar(s):
    s.enemies = [
        {'x': 8.0, 'y': 5.0, 'hp': 200, 'zone': 1},
        {'x': 12.0, 'y': 3.0, 'hp': 150, 'zone': 3},
    ]
test_scene("雷达: 同区域最近敌人(8,5)", setup_radar)

# 场景9: 雷达点全无效
def setup_radar_invalid(s):
    s.enemies = [
        {'x': -1.0, 'y': -1.0, 'hp': 200, 'zone': 1},
        {'x': 12.0, 'y': 3.0, 'hp': 0, 'zone': 1},  # 死了
    ]
test_scene("雷达: 全无效(死亡/非法)", setup_radar_invalid)

# 场景10: 云台点有效
def setup_gimble(s):
    s.gimble_x = 4.5
    s.gimble_y = 6.0
test_scene("云台点有效(4.5, 6.0)", setup_gimble)

# 场景11: 云台点无效
test_scene("云台点无效(-1, -1)",
    lambda s: None)

# 场景12: 雷达+云台同时有效(雷达优先)
def setup_both(s):
    s.enemies = [{'x': 7.0, 'y': 4.0, 'hp': 200, 'zone': 1}]
    s.gimble_x = 4.5
    s.gimble_y = 6.0
test_scene("雷达+云台同时有效(雷达优先)", setup_both)

# 场景13: 都没有 → 巡航
test_scene("无目标 → homeCruise",
    lambda s: setattr(s, 'second', 300))

# 场景14: 雷达到达+等待
def setup_radar_arrive(s):
    s.enemies = [{'x': 3.0, 'y': 5.0, 'hp': 200, 'zone': 1}]  # 敌人在机器人位置
test_scene("雷达: 到达(敌人在机器人位置)", setup_radar_arrive)

# 场景15: 云台到达+等待
def setup_gimble_arrive(s):
    s.gimble_x = 3.0
    s.gimble_y = 5.0  # 云台点在机器人位置
test_scene("云台: 到达(在机器人位置)", setup_gimble_arrive)

# 场景16: 连续多帧模拟(雷达到达→10秒→清除)
def setup_radar_multi(s):
    s.enemies = [{'x': 3.0, 'y': 5.0, 'hp': 200, 'zone': 1}]
test_scene("雷达连续帧: 到达→等待→清除", setup_radar_multi)
# 模拟后续帧
s = SimState()
s.enemies = [{'x': 3.0, 'y': 5.0, 'hp': 200, 'zone': 1}]
decision_tick(s)
print("  --- 第2帧(到达中) ---")
decision_tick(s)
print("  --- 第3帧(雷达缓存还在) ---")
s.enemies = [{'x': -1.0, 'y': -1.0, 'hp': 200, 'zone': 1}]  # 敌人消失
decision_tick(s)

print("\n" + "="*50)
print("所有场景测试完成")
print("="*50)
