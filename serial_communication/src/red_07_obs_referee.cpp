#include "red.h"

// ============================================================
// 发布观察空间 (72维)
// ============================================================

void RefereeSystemNode::publishObservation()
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data.reserve(72);

  // 计算自身到各点的距离 (非法点返回-1)
  auto dist = [this](float x, float y) -> float {
    if (x < -0.5f || y < -0.5f) return -1.0f;
    float dx = robot_pos_.x - x;
    float dy = robot_pos_.y - y;
    return std::sqrt(dx * dx + dy * dy);
  };

  // === 自身 (5维) ===
  msg.data.push_back(static_cast<float>(robot_pos_.x));
  msg.data.push_back(static_cast<float>(robot_pos_.y));
  msg.data.push_back(static_cast<float>(own_zone_));
  msg.data.push_back(my_hp_);
  msg.data.push_back(static_cast<float>(bullet_));

  // === 3队友 (15维) ===  my_engineer, my_infantry_3, my_infantry_4
  // 队友1: my_engineer
  msg.data.push_back(my_engineer_x_);
  msg.data.push_back(my_engineer_y_);
  msg.data.push_back(static_cast<float>(my_engineer_zone_));
  msg.data.push_back(static_cast<float>(my_engineer_hp_));
  msg.data.push_back(dist(my_engineer_x_, my_engineer_y_));

  // 队友2: my_infantry_3
  msg.data.push_back(my_infantry_3_x_);
  msg.data.push_back(my_infantry_3_y_);
  msg.data.push_back(static_cast<float>(my_infantry_3_zone_));
  msg.data.push_back(static_cast<float>(my_infantry_3_hp_));
  msg.data.push_back(dist(my_infantry_3_x_, my_infantry_3_y_));

  // 队友3: my_infantry_4
  msg.data.push_back(my_infantry_4_x_);
  msg.data.push_back(my_infantry_4_y_);
  msg.data.push_back(static_cast<float>(my_infantry_4_zone_));
  msg.data.push_back(static_cast<float>(my_infantry_4_hp_));
  msg.data.push_back(dist(my_infantry_4_x_, my_infantry_4_y_));

  // === 己方英雄 (10维) ===
  float hero_dist_to_self = dist(base_x_, base_y_);

  // 找离英雄最近的敌方
  float near_enemy_dist_to_hero = -1.0f;
  float near_enemy_dist_to_self = -1.0f;
  float near_enemy_x = -1.0f;
  float near_enemy_y = -1.0f;
  float near_enemy_zone = -1.0f;

  struct Enemy { float x, y; int hp, zone; };
  Enemy enemies[] = {
    {enemy_hero_x_, enemy_hero_y_, enemy_hero_hp_, enemy_hero_zone_},
    {enemy_engineer_x_, enemy_engineer_y_, enemy_engineer_hp_, enemy_engineer_zone_},
    {enemy_infantry_3_x_, enemy_infantry_3_y_, enemy_infantry_3_hp_, enemy_infantry_3_zone_},
    {enemy_infantry_4_x_, enemy_infantry_4_y_, enemy_infantry_4_hp_, enemy_infantry_4_zone_},
    {enemy_sentry_x_, enemy_sentry_y_, enemy_sentry_hp_, enemy_sentry_zone_}
  };

  for (const auto& e : enemies) {
    if (e.hp <= 0) continue;
    float dx = e.x - my_hero_x_;
    float dy = e.y - my_hero_y_;
    float d_to_hero = std::sqrt(dx * dx + dy * dy);
    if (near_enemy_dist_to_hero < 0 || d_to_hero < near_enemy_dist_to_hero) {
      near_enemy_dist_to_hero = d_to_hero;
      near_enemy_dist_to_self = dist(e.x, e.y);
      near_enemy_x = e.x;
      near_enemy_y = e.y;
      near_enemy_zone = static_cast<float>(e.zone);
    }
  }

  msg.data.push_back(my_hero_x_);
  msg.data.push_back(my_hero_y_);
  msg.data.push_back(static_cast<float>(my_hero_zone_));
  msg.data.push_back(static_cast<float>(my_hero_hp_));
  msg.data.push_back(hero_dist_to_self);
  msg.data.push_back(near_enemy_dist_to_hero);
  msg.data.push_back(near_enemy_dist_to_self);
  msg.data.push_back(near_enemy_x);
  msg.data.push_back(near_enemy_y);
  msg.data.push_back(near_enemy_zone);

  // === 5敌方 (25维) ===
  for (const auto& e : enemies) {
    msg.data.push_back(e.x);
    msg.data.push_back(e.y);
    msg.data.push_back(static_cast<float>(e.zone));
    msg.data.push_back(static_cast<float>(e.hp));
    msg.data.push_back(dist(e.x, e.y));
  }

  // === 前哨站 (5维) ===
  float outpost_x = static_cast<float>(outpost_x_);
  float outpost_y = static_cast<float>(outpost_y_);
  msg.data.push_back(outpost_x);
  msg.data.push_back(outpost_y);
  msg.data.push_back(static_cast<float>(outpost_hp_));
  msg.data.push_back(outpost_stopped_ ? 1.0f : 0.0f);
  msg.data.push_back(dist(outpost_x, outpost_y));

  // === 基地 (5维) ===
  float base_x = static_cast<float>(base_x_);
  float base_y = static_cast<float>(base_y_);
  msg.data.push_back(base_x);
  msg.data.push_back(base_y);
  msg.data.push_back(static_cast<float>(base_hp_));
  msg.data.push_back(base_exposed_ ? 1.0f : 0.0f);
  msg.data.push_back(dist(base_x, base_y));

  // === 能量机关 (4维) ===
  float energy_x = static_cast<float>(energy_x_);
  float energy_y = static_cast<float>(energy_y_);
  msg.data.push_back(energy_x);
  msg.data.push_back(energy_y);
  msg.data.push_back(energy_activated_ ? 1.0f : 0.0f);
  msg.data.push_back(dist(energy_x, energy_y));

  // === 全局 (3维) ===
  msg.data.push_back(static_cast<float>(team_economy_));
  msg.data.push_back(static_cast<float>(revive_count_));
  msg.data.push_back(static_cast<float>(revive_cost_));

  obs_pub_->publish(msg);
}

void RefereeSystemNode::publishRefereeStatus()
{
  // helper: fill RobotStatus
  auto fill_robot = [&](rmoss_interfaces::msg::RobotStatus &s,
                        const std::string &name, int id,
                        float x, float y, int hp, int max_hp,
                        int total_proj, int used_proj) {
    s.id = id;
    s.level = 0;
    s.name = name;
    s.remain_hp = hp;
    s.max_hp = max_hp;
    s.total_projectiles = total_proj;
    s.used_projectiles = used_proj;
    s.hit_projectiles = 0;
    s.gt_tf.translation.x = x;
    s.gt_tf.translation.y = y;
    s.gt_tf.translation.z = 0.0;
    s.gt_tf.rotation.w = 1.0;
  };

  // helper: SLAM坐标 → 真实坐标
  auto slam_to_real = [this](float x, float y) -> std::pair<float,float> {
    if (x < -0.5f || y < -0.5f) return {-1.0f, -1.0f};
    return {x + origin_x_, y + origin_y_};
  };

  // 自身
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(robot_pos_.x, robot_pos_.y);
    fill_robot(s, "red_standard_robot1", 1,
               rx, ry,
               static_cast<int>(my_hp_), 400,
               300, 300 - static_cast<int>(bullet_));
    ref_self_pub_->publish(s);
  }

  // 英雄
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(my_hero_x_, my_hero_y_);
    fill_robot(s, "red_hero_robot1", 0,
               rx, ry, my_hero_hp_, 400, 0, 0);
    ref_hero_pub_->publish(s);
  }

  // 工程
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(my_engineer_x_, my_engineer_y_);
    fill_robot(s, "red_engineer_robot1", 2,
               rx, ry, my_engineer_hp_, 400, 0, 0);
    ref_engineer_pub_->publish(s);
  }

  // 步兵3
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(my_infantry_3_x_, my_infantry_3_y_);
    fill_robot(s, "red_standard_robot2", 3,
               rx, ry, my_infantry_3_hp_, 400, 0, 0);
    ref_infantry3_pub_->publish(s);
  }

  // 步兵4
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(my_infantry_4_x_, my_infantry_4_y_);
    fill_robot(s, "red_standard_robot3", 4,
               rx, ry, my_infantry_4_hp_, 400, 0, 0);
    ref_infantry4_pub_->publish(s);
  }

  // 敌方英雄
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(enemy_hero_x_, enemy_hero_y_);
    fill_robot(s, "blue_hero_robot1", 5,
               rx, ry, enemy_hero_hp_, 400, 0, 0);
    ref_enemy_hero_pub_->publish(s);
  }

  // 敌方工程
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(enemy_engineer_x_, enemy_engineer_y_);
    fill_robot(s, "blue_engineer_robot1", 6,
               rx, ry, enemy_engineer_hp_, 400, 0, 0);
    ref_enemy_engineer_pub_->publish(s);
  }

  // 敌方步兵3
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(enemy_infantry_3_x_, enemy_infantry_3_y_);
    fill_robot(s, "blue_standard_robot1", 7,
               rx, ry, enemy_infantry_3_hp_, 400, 0, 0);
    ref_enemy_infantry3_pub_->publish(s);
  }

  // 敌方步兵4
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(enemy_infantry_4_x_, enemy_infantry_4_y_);
    fill_robot(s, "blue_standard_robot2", 8,
               rx, ry, enemy_infantry_4_hp_, 400, 0, 0);
    ref_enemy_infantry4_pub_->publish(s);
  }

  // 敌方哨兵
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(enemy_sentry_x_, enemy_sentry_y_);
    fill_robot(s, "blue_sentry_robot1", 9,
               rx, ry, enemy_sentry_hp_, 400, 0, 0);
    ref_enemy_sentry_pub_->publish(s);
  }

  // 前哨站
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(outpost_x_, outpost_y_);
    fill_robot(s, "outpost", 10,
               rx, ry,
               outpost_hp_, 1500, 0, 0);
    ref_outpost_pub_->publish(s);
  }

  // 基地
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(base_x_, base_y_);
    fill_robot(s, "base", 11,
               rx, ry,
               base_hp_, 5000, 0, 0);
    ref_base_pub_->publish(s);
  }

  // 能量机关
  {
    rmoss_interfaces::msg::RobotStatus s;
    auto [rx, ry] = slam_to_real(energy_x_, energy_y_);
    fill_robot(s, "energy", 12,
               rx, ry,
               energy_activated_ ? 1 : 0, 1, 0, 0);
    ref_energy_pub_->publish(s);
  }

  // 游戏状态
  {
    rmoss_interfaces::msg::GameStatus g;
    g.status = 2; // RUNNING
    g.total_time = 0;
    g.current_time = static_cast<int>(second_);
    g.result = 0;
    ref_game_pub_->publish(g);
  }
}

