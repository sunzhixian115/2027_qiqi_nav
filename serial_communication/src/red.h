#ifndef RED_H
#define RED_H

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <stdint.h>

#include <vector>
#include <utility>
#include <string>
#include <iostream>
#include <cstring>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "base_interfaces_demo/msg/target.hpp"
#include "rmoss_interfaces/msg/robot_status.hpp"
#include "rmoss_interfaces/msg/game_status.hpp"
#include "crc16.h"

using namespace std::chrono_literals;

// ============================================================
// 状态机
// ============================================================

enum class Stage
{
  IDLE = 0,

  GO_PRE_POINT,
  GO_Z1,
  GO_Z2,
  GO_BACK_POINT,

  FINISHED
};

enum class DecisionMode
{
  BASE_MODE = 0,
  HIGH_MODE
};

// ============================================================
// 电控发送数据帧
// 哨兵 -> STM32  (25+2=27 字节)
// ============================================================

#pragma pack(push, 1)

struct SerialOutFrame
{
  uint8_t header;

  float linear_x;
  float linear_y;
  float angular_z;

  float yaw;

  float dist;

  bool if_turn;

  uint16_t region;

  uint8_t attack_target;


  float obstacle_x;  // 给雷达站传的
  float obstacle_y; 

  uint8_t if_arrive_base_point;//0表示还没到，1表示到了

  uint16_t crc;
};

#pragma pack(pop)

static_assert(
  sizeof(SerialOutFrame) == 36,
  "SerialOutFrame size error");

// ============================================================
// 电控接收数据帧
// STM32 -> 哨兵
// ============================================================

#pragma pack(push, 1)

struct SerialInFrame
{
  uint8_t header;

  uint8_t start_flag;

  float enemy_distance;//自瞄敌人距离

  float enemy_yaw;//自瞄敌人相对云台的角度

  float my_hp;

  uint16_t second;

  uint16_t bullet;

  bool rune_mode;//是否在符文模式

  uint16_t attack_mession;//1英雄 2工程 34步兵 10前哨 11基地 

  float my_hero_x;//咱英雄
  float my_hero_y;
  int my_hero_hp;

  float my_engineer_x;//咱工程
  float my_engineer_y;
  int my_engineer_hp;

  float my_infantry3_x;//咱步兵
  float my_infantry3_y;
  int my_infantry3_hp;

  float my_infantry4_x;//同上
  float my_infantry4_y;
  int my_infantry4_hp;

  float enemy_hero_x;//敌人英雄
  float enemy_hero_y;
  int enemy_hero_hp;

  float enemy_engineer_x;//敌人工程
  float enemy_engineer_y;
  int enemy_engineer_hp;

  float enemy_infantry3_x;//敌人步兵
  float enemy_infantry3_y;
  int enemy_infantry3_hp;

  float enemy_infantry4_x;//敌人步兵
  float enemy_infantry4_y;
  int enemy_infantry4_hp;

  

  float enemy_sentry_x;//敌人哨兵
  float enemy_sentry_y;
  int enemy_sentry_hp;

  bool base_exposed;//基地是否被暴露
  bool energy_activated;//能量机关是否被激活

  int outpost_hp;//前哨站血量
  int base_hp;//基地血量



  uint8_t if_fly;//是否在飞行状态  0是没起飞 1是起飞了
  uint8_t gimble_status;//云台手状态 默认为0 0是前哨没死 1是前哨死翘翘了，我解放了可以乱跑了 2是回到基地点猥琐的扫描 3是绕基地遛弯 


  uint16_t crc;
};

#pragma pack(pop)

// static_assert 暂时注释: 结构体已扩展，帧长度待电控确认
static_assert(
  sizeof(SerialInFrame) == 143,
  "SerialInFrame size error");

// ============================================================
// 串口接收类
// 电控 -> 哨兵
// ============================================================

class SerialReceiver
{
public:

  explicit SerialReceiver(serial::Serial & serial_port)
  : serial_port_(serial_port)
  {
  }

  void readSerialData();

  // ==========================================================
  // 数据
  // ==========================================================

  uint8_t start_flag_ = 0;

  uint16_t second_ = 420;

  uint16_t bullet_ = 300;

  float my_hp_ = 400.0f;

  float enemy_distance_ = 0.0f;

  float enemy_yaw_ = 0.0f;

  bool rune_mode_ = false;

  uint16_t attack_mession_ = 0;

  uint8_t if_arrive_base_point_;//0表示还没到，1表示到了


  // 己方机器人
  float my_hero_x_ = -1.0f;
  float my_hero_y_ = -1.0f;
  int my_hero_hp_ = -1;

  float my_engineer_x_ = -1.0f;
  float my_engineer_y_ = -1.0f;
  int my_engineer_hp_ = -1;

  float my_infantry3_x_ = -1.0f;
  float my_infantry3_y_ = -1.0f;
  int my_infantry3_hp_ = -1;

  float my_infantry4_x_ = -1.0f;
  float my_infantry4_y_ = -1.0f;
  int my_infantry4_hp_ = -1;

  // 敌方机器人
  float enemy_hero_x_ = -1.0f;
  float enemy_hero_y_ = -1.0f;
  int enemy_hero_hp_ = -1;

  float enemy_engineer_x_ = -1.0f;
  float enemy_engineer_y_ = -1.0f;
  int enemy_engineer_hp_ = -1;

  float enemy_infantry3_x_ = -1.0f;
  float enemy_infantry3_y_ = -1.0f;
  int enemy_infantry3_hp_ = -1;

  float enemy_infantry4_x_ = -1.0f;
  float enemy_infantry4_y_ = -1.0f;
  int enemy_infantry4_hp_ = -1;

  float enemy_sentry_x_ = -1.0f;
  float enemy_sentry_y_ = -1.0f;
  int enemy_sentry_hp_ = -1;

  // 场地状态
  bool base_exposed_ = false;
  bool energy_activated_ = false;

  int outpost_hp_ = -1;
  int base_hp_ = -1;

  // 云台坐标
  uint8_t if_fly_ = 0;//飞机没飞
  uint8_t gimble_status_ = 0;//前哨没死翘翘 1是死翘翘

private:

  void processBuffer();

  serial::Serial & serial_port_;

  std::vector<uint8_t> buffer_;
};

// ============================================================
// 主节点
// ============================================================

class RefereeSystemNode : public rclcpp::Node
{
public:

  RefereeSystemNode();

private:

  // ==========================================================
  // 参数加载
  // ==========================================================

  void loadPointParameter(
    const std::string & name,
    geometry_msgs::msg::Point & point);

  // ==========================================================
  // 回调函数
  // ==========================================================

  void cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg);

  void gimbleYawCallback(
    const std_msgs::msg::Float64::SharedPtr msg);

  void chasePointCallback(
    const geometry_msgs::msg::Point::SharedPtr msg);

  void relYawDistCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // RL动作回调
  void rlCmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg);

  void rlShootCallback(
    const std_msgs::msg::UInt16::SharedPtr msg);

  void rlReviveCallback(
    const std_msgs::msg::Bool::SharedPtr msg);

  void rlRemoteAmmoCallback(
    const std_msgs::msg::Bool::SharedPtr msg);

  void rlLocalAmmoCallback(
    const std_msgs::msg::Bool::SharedPtr msg);

  // ==========================================================
  // 主循环
  // ==========================================================

  void readSerialLoop();

  void decisionLoop();

  void sendLoop();

  // ==========================================================
  // 更新战场信息
  // ==========================================================

  void updateBattleInfo();

  // ==========================================================
  // 决策
  // ==========================================================

  void decisionTick();

  void homeCruise();//前哨站还没爆炸的巡航

  void highCruise();//高地巡航

  void baseCruise();//基地巡航四个角乱跑的巡航

  void no_out_poset_Cruise();//前哨站爆炸了的巡航



  void go_to_high();

  void go_to_base();



  void publishObservation();
  void publishRefereeStatus();

  // ==========================================================
  // 辅助函数
  // ==========================================================

  void publishGoal(
    const geometry_msgs::msg::Point & p);

  bool getRobotPosition(
    geometry_msgs::msg::Point & position);

  float getDistance(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);

  static bool pointInPolygon(
    double px, double py,
    const std::vector<
      std::pair<double, double>> & vertices);

  int getRegion(
    const geometry_msgs::msg::Point & p);

  geometry_msgs::msg::Point predictPoint();

  geometry_msgs::msg::Point findRadarTarget(int my_zone);

  float get_enemy_yaw( const float gimble_yaw_,const geometry_msgs::msg::Point robot_pos_,const geometry_msgs::msg::Point enemy_pos_ );  // 计算敌人与雷达朝向之间的夹角


  // ==========================================================
  // 区域数据
  // ==========================================================

  struct Region
  {
    int name;

    std::vector<
      std::pair<double, double>>
        vertices;
  };

  std::vector<Region> regions_;

  // ==========================================================
  // 串口
  // ==========================================================

  serial::Serial serial_port_;

  SerialReceiver serial_receiver_;

  // ==========================================================
  // timer
  // ==========================================================

  rclcpp::TimerBase::SharedPtr read_timer_;

  rclcpp::TimerBase::SharedPtr decision_timer_;

  rclcpp::TimerBase::SharedPtr send_timer_;

  // ==========================================================
  // publisher
  // ==========================================================

  rclcpp::Publisher<
    base_interfaces_demo::msg::Target>::SharedPtr track_target_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;

  // referee_system 话题 (供 RL 推理节点)
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_self_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_hero_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_engineer_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_infantry3_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_infantry4_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_enemy_hero_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_enemy_engineer_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_enemy_infantry3_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_enemy_infantry4_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_enemy_sentry_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_outpost_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_base_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::RobotStatus>::SharedPtr ref_energy_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::GameStatus>::SharedPtr ref_game_pub_;

  // ==========================================================
  // subscriber
  // ==========================================================

  rclcpp::Subscription<
    geometry_msgs::msg::Point>::SharedPtr chase_point_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float64>::SharedPtr gimble_yaw_sub_;

  // 雷达障碍物(dist, yaw)
  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr rel_yaw_dist_sub_;

  // RL动作订阅
  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr rl_cmd_vel_sub_;

  rclcpp::Subscription<
    std_msgs::msg::UInt16>::SharedPtr rl_shoot_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr rl_revive_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr rl_remote_ammo_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr rl_local_ammo_sub_;

  // ==========================================================
  // tf
  // ==========================================================

  std::shared_ptr<tf2_ros::Buffer>
    tf_buffer_;

  std::shared_ptr<
    tf2_ros::TransformListener>
      tf_listener_;

  // ==========================================================
  // frame
  // ==========================================================

  std::string map_frame_;

  std::string base_frame_;

  std::string lidar_frame_;

  // ==========================================================
  // 状态
  // ==========================================================

  bool chase_locked_ = false;

  bool high_ground_locked_ = false;

  bool need_heal_ = false;

  bool going_home_ = false;

  bool if_turn_ = false;

  // 雷达点状态
  bool radar_active_ = false;
  bool radar_arrived_ = false;  // 是否已到达雷达点并开始计时
  rclcpp::Time radar_arrive_time_;

  // 缓存点
  geometry_msgs::msg::Point radar_saved_point_;

  Stage stage_ = Stage::IDLE;

  DecisionMode decision_mode_ = DecisionMode::BASE_MODE;

  // ==========================================================
  // 串口数据缓存
  // ==========================================================

  float enemy_distance_ = 0.0f;
  float enemy_yaw_ = 0.0f;
  float my_hp_ = 400.0f;
  uint16_t second_ = 420;
  uint16_t bullet_ = 300;
  bool rune_mode_ = false;
  uint16_t attack_mession_ = 0;
  

  // ==========================================================
  // RL动作缓存
  // ==========================================================

  geometry_msgs::msg::Twist rl_twist_;
  uint8_t rl_shoot_ = 0;
  bool rl_revive_ = false;
  bool rl_remote_ammo_ = false;
  bool rl_local_ammo_ = false;

  bool rl_active_ = false;  // RL是否激活

  // ==========================================================
  // 电控发送缓存
  // ==========================================================

  geometry_msgs::msg::Twist current_twist_;

  float gimble_yaw_ = 0.0f;

  // 雷达障碍物(雷达坐标系)
  float obs_dist_ = 0.0f;
  float obs_yaw_ = 0.0f;
  bool has_obs_ = false;

  float target_distance_ = 0.0f;
  float enemy_yaw_angle_ = 0.0f;  // 车头到敌人的夹角(弧度)

  // region_num_ 已删除，改用 getRegion(robot_pos_) 实时计算

  uint8_t attack_target_ = 0;
  uint8_t if_arrive_base_point_ = 0;  // 0未到达 1已到达
  uint8_t if_fly_ = 0;

  // ==========================================================
  // 点位
  // ==========================================================

  geometry_msgs::msg::Point robot_pos_;

  geometry_msgs::msg::Point chase_point_;

  geometry_msgs::msg::Point current_target_point_;

  // publishGoal 去重 + 兜底重发
  bool goal_ever_published_ = false;
  geometry_msgs::msg::Point last_published_goal_;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  geometry_msgs::msg::Point attack_point_1_;
  geometry_msgs::msg::Point attack_point_2_;

  geometry_msgs::msg::Point high_point_1_;
  geometry_msgs::msg::Point high_point_2_;
  geometry_msgs::msg::Point high_point_3_;

  geometry_msgs::msg::Point pre_point_;

  geometry_msgs::msg::Point z1_;

  geometry_msgs::msg::Point z2_;

  geometry_msgs::msg::Point back_point_;

  geometry_msgs::msg::Point supply_point_;

  geometry_msgs::msg::Point base_point_;
  geometry_msgs::msg::Point home_point_1_;
  geometry_msgs::msg::Point home_point_2_;
  geometry_msgs::msg::Point home_point_3_;
  geometry_msgs::msg::Point home_point_4_;
  geometry_msgs::msg::Point no_outpost_point_1_;
  geometry_msgs::msg::Point no_outpost_point_2_;

  // ==========================================================
  // 其他
  // ==========================================================

  geometry_msgs::msg::PoseStamped goal_msg_;

  base_interfaces_demo::msg::Target target_msg_;

  double arrive_tolerance_ = 0.5;

  // ========================================================
  // RL观察空间: 固定目标坐标 (config)
  // ========================================================

  double outpost_x_ = 11.0;
  double outpost_y_ = 11.35;
  double base_x_ = 2.4;
  double base_y_ = 7.5;
  double energy_x_ = 14.0;
  double energy_y_ = 7.5;

  // ========================================================
  // 地图偏移 (地图yaml origin 取反)
  // 读进来(真实→SLAM): xy -= origin_
  // 发出去(SLAM→真实): xy += origin_
  // ========================================================

  double origin_x_ = -3.8;
  double origin_y_ = -7.5;

  // ========================================================
  // RL观察空间: 全局状态 (成员变量)
  // ========================================================

  int team_economy_ = 400;
  int revive_count_ = -1;
  int revive_cost_ = -1;

  // ========================================================
  // RL观察空间: 10个机器人状态 (成员变量)
  // xy默认-1, zone默认-1, hp默认-1
  // ========================================================

  // 自身 (xy从robot_pos_取, hp从串口取, ammo从串口取)
  int own_zone_ = -1;

  // 己方英雄
  float my_hero_x_ = -1.0f;
  float my_hero_y_ = -1.0f;
  int my_hero_hp_ = -1;
  int my_hero_zone_ = -1;

  // 己方工程
  float my_engineer_x_ = -1.0f;
  float my_engineer_y_ = -1.0f;
  int my_engineer_hp_ = -1;
  int my_engineer_zone_ = -1;

  // 己方步兵3
  float my_infantry_3_x_ = -1.0f;
  float my_infantry_3_y_ = -1.0f;
  int my_infantry_3_hp_ = -1;
  int my_infantry_3_zone_ = -1;

  // 己方步兵4
  float my_infantry_4_x_ = -1.0f;
  float my_infantry_4_y_ = -1.0f;
  int my_infantry_4_hp_ = -1;
  int my_infantry_4_zone_ = -1;

  // 敌方英雄
  float enemy_hero_x_ = -1.0f;
  float enemy_hero_y_ = -1.0f;
  int enemy_hero_hp_ = -1;
  int enemy_hero_zone_ = -1;

  // 敌方工程
  float enemy_engineer_x_ = -1.0f;
  float enemy_engineer_y_ = -1.0f;
  int enemy_engineer_hp_ = -1;
  int enemy_engineer_zone_ = -1;

  // 敌方步兵3
  float enemy_infantry_3_x_ = -1.0f;
  float enemy_infantry_3_y_ = -1.0f;
  int enemy_infantry_3_hp_ = -1;
  int enemy_infantry_3_zone_ = -1;

  // 敌方步兵4
  float enemy_infantry_4_x_ = -1.0f;
  float enemy_infantry_4_y_ = -1.0f;
  int enemy_infantry_4_hp_ = -1;
  int enemy_infantry_4_zone_ = -1;

  // 敌方哨兵
  float enemy_sentry_x_ = -1.0f;
  float enemy_sentry_y_ = -1.0f;
  int enemy_sentry_hp_ = -1;
  int enemy_sentry_zone_ = -1;

  // 前哨站/基地/能量机关状态
  int outpost_hp_ = -1;
  int base_hp_ = -1;
  bool outpost_stopped_ = false;
  bool base_exposed_ = false;
  bool energy_activated_ = false;
};

#endif // RED_H
