#include "red.h"

// ============================================================
// 主循环
// ============================================================

void RefereeSystemNode::readSerialLoop()
{
  serial_receiver_.readSerialData();
}

void RefereeSystemNode::decisionLoop()
{
  // 更新TF
  getRobotPosition(robot_pos_);

  // 更新战场信息
  updateBattleInfo();

  // 发布观察空间
  publishObservation();

  // 发布 referee_system 话题
  publishRefereeStatus();

  // 决策
  decisionTick();
}

void RefereeSystemNode::sendLoop()
{
  SerialOutFrame frame;

  frame.header = 0xa5;

  frame.linear_x =
    current_twist_.linear.x;

  frame.linear_y =
    current_twist_.linear.y;

  frame.angular_z =
    current_twist_.angular.z;

    // frame.linear_x =rl_twist_.linear.x*cos(gimble_yaw_);
    // frame.linear_y =rl_twist_.linear.y*sin(gimble_yaw_);
    // frame.angular_z =rl_twist_.angular.z;


  // 直接用 getRegion 计算当前区域
  uint16_t current_region = getRegion(robot_pos_);
  if(current_region == 2)
  {
    frame.linear_x *= 2.0f;
    frame.linear_y *= 2.0f;
  }



  frame.yaw =
    gimble_yaw_;

  // frame.enemy_yaw =
  //   enemy_yaw_angle_;  // 暂时不传给电控

  frame.dist =
    target_distance_;

  frame.if_turn =
    if_turn_;

  frame.region =
    current_region;

  frame.attack_target =
    attack_target_;

  frame.if_arrive_base_point =
    if_arrive_base_point_;  // 0未到达 1已到达

  // ====== 障碍物全局坐标(TF变换, 同chase方法) ======
  if (has_obs_ && obs_dist_ > 0.01f)
  {
    // dist/yaw 是 livox_frame 下的极坐标
    // 先转成 livox_frame 下的 xyz
    geometry_msgs::msg::PointStamped obs_in_lidar;
    obs_in_lidar.header.frame_id = lidar_frame_;
    obs_in_lidar.header.stamp = rclcpp::Time(0);
    obs_in_lidar.point.x =
      obs_dist_ * std::cos(obs_yaw_);
    obs_in_lidar.point.y =
      obs_dist_ * std::sin(obs_yaw_);
    obs_in_lidar.point.z = 0.0;

    // 用 TF 变到 map 坐标系
    try
    {
      geometry_msgs::msg::PointStamped obs_in_map =
        tf_buffer_->transform(
          obs_in_lidar,
          map_frame_,
          tf2::durationFromSec(0.1));

      // SLAM坐标 → 真实坐标 (加回偏移)
      frame.obstacle_x =
        static_cast<float>(obs_in_map.point.x) + origin_x_;
      frame.obstacle_y =
        static_cast<float>(obs_in_map.point.y) + origin_y_;
    }
    catch (tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "障碍物TF变换失败: %s",
        ex.what());
      frame.obstacle_x = -1.0f;
      frame.obstacle_y = -1.0f;
    }
  }
  else
  {
    frame.obstacle_x = -1.0f;
    frame.obstacle_y = -1.0f;
    // -1无效值, 不做偏移
  }

  append_CRC16_check_sum(
    reinterpret_cast<uint8_t*>(&frame),
    sizeof(frame));

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "发送: vx=%.2f vy=%.2f wz=%.2f region=%u",
    frame.linear_x,
    frame.linear_y,
    frame.angular_z,
    frame.region);

  // dump发送帧(hex) - 只在前3次打印，之后每200次打印一次
  {
    static int tx_count = 0;
    tx_count++;
    if (tx_count <= 3 || tx_count % 200 == 0) {
      const uint8_t* fp = reinterpret_cast<const uint8_t*>(&frame);
      std::string hex;
      for (size_t i = 0; i < sizeof(frame); ++i) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%02X ", fp[i]);
        hex += buf;
      }
      RCLCPP_INFO(
        get_logger(),
        "TX#%d vx=%.2f vy=%.2f: %s",
        tx_count, frame.linear_x, frame.linear_y, hex.c_str());
    }
  }

  try
  {
    serial_port_.write(
      reinterpret_cast<
        const uint8_t *>(&frame),
      sizeof(frame));
  }
  catch(serial::IOException & e)
  {
    RCLCPP_ERROR(
      get_logger(),
      "串口发送失败: %s",
      e.what());
  }



}

// ============================================================
