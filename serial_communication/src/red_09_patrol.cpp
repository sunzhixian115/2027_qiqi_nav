#include "red.h"

// ============================================================
// findRadarTarget
//
// 功能：找同区域最近的存活敌人，返回其坐标
// 返回：目标点（SLAM坐标），没找到返回当前位置
// ============================================================

geometry_msgs::msg::Point RefereeSystemNode::findRadarTarget(int my_zone)
{
  struct Enemy
  {
    float x;
    float y;
    int hp;
    int zone;
  };

  std::vector<Enemy> enemies =
  {
    {enemy_hero_x_, enemy_hero_y_, enemy_hero_hp_, enemy_hero_zone_},
    {enemy_engineer_x_, enemy_engineer_y_, enemy_engineer_hp_, enemy_engineer_zone_},
    {enemy_infantry_3_x_, enemy_infantry_3_y_, enemy_infantry_3_hp_, enemy_infantry_3_zone_},
    {enemy_infantry_4_x_, enemy_infantry_4_y_, enemy_infantry_4_hp_, enemy_infantry_4_zone_},
    {enemy_sentry_x_, enemy_sentry_y_, enemy_sentry_hp_, enemy_sentry_zone_}
  };

  float min_dist = 999.0f;
  bool found = false;
  geometry_msgs::msg::Point best_target;

  for(const auto & e : enemies)
  {
    // 非法点过滤
    if(e.x < -0.5f || e.y < -0.5f) continue;

    // 死亡过滤
    if(e.hp <= 0) continue;

    // 区域过滤
    if(my_zone == 0 || e.zone != my_zone) continue;

    // 距离计算
    float dx = e.x - robot_pos_.x;
    float dy = e.y - robot_pos_.y;
    float d = std::sqrt(dx * dx + dy * dy);

    if(d < min_dist)
    {
      min_dist = d;
      best_target.x = e.x;
      best_target.y = e.y;
      best_target.z = 0.0;
      found = true;
    }
  }

  // 没找到返回无效点（不能返回robot_pos_，否则会原地打转）
  if(!found)
  {
    geometry_msgs::msg::Point invalid;
    invalid.x = -1.0f;
    invalid.y = -1.0f;
    invalid.z = 0.0;
    return invalid;
  }

  RCLCPP_INFO(
    get_logger(),
    "Radar 目标 -> (%.2f, %.2f) dist=%.2f",
    best_target.x,
    best_target.y,
    min_dist);

  return best_target;
}



float RefereeSystemNode::get_enemy_yaw( const float gimble_yaw_,const geometry_msgs::msg::Point robot_pos_,const geometry_msgs::msg::Point enemy_pos_ )//计算敌人与雷达朝向之间的夹角
{//与地图x轴夹角，本车位姿，敌人位姿
  if(enemy_pos_.x <-0.5 && enemy_pos_.y < -0.5)
  {
    RCLCPP_INFO(get_logger(),"无合适目标");
    return 0.0f;
  }
  float d_x = enemy_pos_.x - robot_pos_.x; 
  float d_y = enemy_pos_.y - robot_pos_.y;
  float yaw = atan2(d_y , d_x);
  float turn_yaw = gimble_yaw_ - yaw;
  // 归一化到 [-π, π]
  while (turn_yaw > M_PI)  turn_yaw -= 2.0f * M_PI;
  while (turn_yaw < -M_PI) turn_yaw += 2.0f * M_PI;
  RCLCPP_INFO(get_logger(),"转角是%.2f",turn_yaw); 
  return turn_yaw ;
}

// ============================================================
// main
// ============================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<RefereeSystemNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
