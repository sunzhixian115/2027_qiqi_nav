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

  uint8_t if_arrive_base_point;//0表示还没到，1表示到了

  uint16_t crc;
};

#pragma pack(pop)

static_assert(
  sizeof(SerialOutFrame) == 28,
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

  uint16_t attack_mession;//1英雄 2工程 34步兵 10前哨 11基地 

  uint16_t crc;
};

#pragma pack(pop)

// static_assert 暂时注释: 结构体已扩展，帧长度待电控确认
static_assert(sizeof(SerialInFrame) == 22, "SerialInFrame size error");

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

  uint16_t attack_mession_ = 0;

  uint8_t if_arrive_base_point_;//0表示还没到，1表示到了


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

  // 保留内部兼容接口，当前不参与任何 RL 或发布链路。
  geometry_msgs::msg::Point predictPoint();


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


  // ==========================================================
  // subscriber
  // ==========================================================

  rclcpp::Subscription<
    geometry_msgs::msg::Point>::SharedPtr chase_point_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float64>::SharedPtr gimble_yaw_sub_;



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


  // ==========================================================
  // 状态
  // ==========================================================

  bool chase_locked_ = false;

  bool high_ground_locked_ = false;

  bool need_heal_ = false;

  bool going_home_ = false;

  bool if_turn_ = false;

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

};

#endif // RED_H
