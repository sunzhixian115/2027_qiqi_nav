#include "red.h"

// 更新战场信息
// ============================================================

void RefereeSystemNode::updateBattleInfo()
{
  // ====================================================
  // 串口基础数据
  // ====================================================

  enemy_distance_ = serial_receiver_.enemy_distance_;
  enemy_yaw_ = serial_receiver_.enemy_yaw_;
  my_hp_ = serial_receiver_.my_hp_;
  second_ = serial_receiver_.second_;
  bullet_ = serial_receiver_.bullet_;
  rune_mode_ = serial_receiver_.rune_mode_;
  attack_mession_ = serial_receiver_.attack_mession_;//10是前哨
  if_fly_ = serial_receiver_.if_fly_;
  


  // ====================================================
  // RL观察空间: 同步串口数据到成员变量
  // ====================================================

  // 自身zone (robot_pos_已经是SLAM坐标)
  own_zone_ = getRegion(robot_pos_);

  // ====================================================
  // 第一步: 拷贝串口数据
  // ====================================================

  // 己方英雄
  my_hero_x_ = serial_receiver_.my_hero_x_;
  my_hero_y_ = serial_receiver_.my_hero_y_;
  my_hero_hp_ = serial_receiver_.my_hero_hp_;

  // 己方工程
  my_engineer_x_ = serial_receiver_.my_engineer_x_;
  my_engineer_y_ = serial_receiver_.my_engineer_y_;
  my_engineer_hp_ = serial_receiver_.my_engineer_hp_;

  // 己方步兵3
  my_infantry_3_x_ = serial_receiver_.my_infantry3_x_;
  my_infantry_3_y_ = serial_receiver_.my_infantry3_y_;
  my_infantry_3_hp_ = serial_receiver_.my_infantry3_hp_;

  // 己方步兵4
  my_infantry_4_x_ = serial_receiver_.my_infantry4_x_;
  my_infantry_4_y_ = serial_receiver_.my_infantry4_y_;
  my_infantry_4_hp_ = serial_receiver_.my_infantry4_hp_;

  // 敌方英雄
  enemy_hero_x_ = serial_receiver_.enemy_hero_x_;
  enemy_hero_y_ = serial_receiver_.enemy_hero_y_;
  enemy_hero_hp_ = serial_receiver_.enemy_hero_hp_;

  // 敌方工程
  enemy_engineer_x_ = serial_receiver_.enemy_engineer_x_;
  enemy_engineer_y_ = serial_receiver_.enemy_engineer_y_;
  enemy_engineer_hp_ = serial_receiver_.enemy_engineer_hp_;

  // 敌方步兵3
  enemy_infantry_3_x_ = serial_receiver_.enemy_infantry3_x_;
  enemy_infantry_3_y_ = serial_receiver_.enemy_infantry3_y_;
  enemy_infantry_3_hp_ = serial_receiver_.enemy_infantry3_hp_;

  // 敌方步兵4
  enemy_infantry_4_x_ = serial_receiver_.enemy_infantry4_x_;
  enemy_infantry_4_y_ = serial_receiver_.enemy_infantry4_y_;
  enemy_infantry_4_hp_ = serial_receiver_.enemy_infantry4_hp_;

  // 敌方哨兵
  enemy_sentry_x_ = serial_receiver_.enemy_sentry_x_;
  enemy_sentry_y_ = serial_receiver_.enemy_sentry_y_;
  enemy_sentry_hp_ = serial_receiver_.enemy_sentry_hp_;



  // ====================================================
  // 第二步: 坐标偏移 (真实→SLAM)
  // 有效的xy(>-0.5)减去偏移量, 无效的(-1)不动
  // ====================================================
  if (my_hero_x_ > -0.5f && my_hero_y_ > -0.5f) {
    my_hero_x_ -= origin_x_; my_hero_y_ -= origin_y_;
  }
  if (my_engineer_x_ > -0.5f && my_engineer_y_ > -0.5f) {
    my_engineer_x_ -= origin_x_; my_engineer_y_ -= origin_y_;
  }
  if (my_infantry_3_x_ > -0.5f && my_infantry_3_y_ > -0.5f) {
    my_infantry_3_x_ -= origin_x_; my_infantry_3_y_ -= origin_y_;
  }
  if (my_infantry_4_x_ > -0.5f && my_infantry_4_y_ > -0.5f) {
    my_infantry_4_x_ -= origin_x_; my_infantry_4_y_ -= origin_y_;
  }
  if (enemy_hero_x_ > -0.5f && enemy_hero_y_ > -0.5f) {
    enemy_hero_x_ -= origin_x_; enemy_hero_y_ -= origin_y_;
  }
  if (enemy_engineer_x_ > -0.5f && enemy_engineer_y_ > -0.5f) {
    enemy_engineer_x_ -= origin_x_; enemy_engineer_y_ -= origin_y_;
  }
  if (enemy_infantry_3_x_ > -0.5f && enemy_infantry_3_y_ > -0.5f) {
    enemy_infantry_3_x_ -= origin_x_; enemy_infantry_3_y_ -= origin_y_;
  }
  if (enemy_infantry_4_x_ > -0.5f && enemy_infantry_4_y_ > -0.5f) {
    enemy_infantry_4_x_ -= origin_x_; enemy_infantry_4_y_ -= origin_y_;
  }
  if (enemy_sentry_x_ > -0.5f && enemy_sentry_y_ > -0.5f) {
    enemy_sentry_x_ -= origin_x_; enemy_sentry_y_ -= origin_y_;
  }


  // ====================================================
  // 第三步: 计算区域 (偏移后的SLAM坐标)
  // ====================================================
  { geometry_msgs::msg::Point p; p.x=my_hero_x_; p.y=my_hero_y_; my_hero_zone_ = getRegion(p); }
  { geometry_msgs::msg::Point p; p.x=my_engineer_x_; p.y=my_engineer_y_; my_engineer_zone_ = getRegion(p); }
  { geometry_msgs::msg::Point p; p.x=my_infantry_3_x_; p.y=my_infantry_3_y_; my_infantry_3_zone_ = getRegion(p); }
  { geometry_msgs::msg::Point p; p.x=my_infantry_4_x_; p.y=my_infantry_4_y_; my_infantry_4_zone_ = getRegion(p); }
  { geometry_msgs::msg::Point p; p.x=enemy_hero_x_; p.y=enemy_hero_y_; enemy_hero_zone_ = getRegion(p); }
  { geometry_msgs::msg::Point p; p.x=enemy_engineer_x_; p.y=enemy_engineer_y_; enemy_engineer_zone_ = getRegion(p); }
  { geometry_msgs::msg::Point p; p.x=enemy_infantry_3_x_; p.y=enemy_infantry_3_y_; enemy_infantry_3_zone_ = getRegion(p); }
  { geometry_msgs::msg::Point p; p.x=enemy_infantry_4_x_; p.y=enemy_infantry_4_y_; enemy_infantry_4_zone_ = getRegion(p); }
  { geometry_msgs::msg::Point p; p.x=enemy_sentry_x_; p.y=enemy_sentry_y_; enemy_sentry_zone_ = getRegion(p); }

  // 前哨站/基地/能量机关状态
  // 前哨站: 比赛7分钟，开始3分钟后停转 → 剩余<=240秒时停转
  outpost_stopped_ = (second_ <= 240);
  base_exposed_ = serial_receiver_.base_exposed_;
  energy_activated_ = serial_receiver_.energy_activated_;
  outpost_hp_ = serial_receiver_.outpost_hp_;
  base_hp_ = serial_receiver_.base_hp_;

  // ====================================================
  // 第四步: 缓存雷达点和云台点 + 计算yaw
  // ====================================================

  const int current_region = getRegion(robot_pos_);

  // 雷达点
  auto radar_pt = findRadarTarget(current_region);
  if (radar_pt.x > -0.5f && radar_pt.y > -0.5f) {
    radar_saved_point_ = radar_pt;
    radar_active_ = true;// 雷达点有效
  } else {
    radar_saved_point_.x = -1.0f;
    radar_saved_point_.y = -1.0f;
    radar_active_ = false;// 雷达点无效
  }

  // 计算车头到敌人的夹角 (只用雷达点)
  if (radar_active_) {
    enemy_yaw_angle_ = get_enemy_yaw(gimble_yaw_, robot_pos_, radar_saved_point_);
  } else {
    enemy_yaw_angle_ = 0.0f;
  }

  // 发布追踪目标到 /track_target
  {
    base_interfaces_demo::msg::Target msg;
    msg.header.header.stamp = this->now();
    msg.header.header.frame_id = "chassis";
    msg.distance = enemy_distance_;
    msg.yaw = enemy_yaw_;
    track_target_pub_->publish(msg);
  }

  
}

// ============================================================
// 决策
// ============================================================

