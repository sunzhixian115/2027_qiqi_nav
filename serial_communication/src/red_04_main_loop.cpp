#include "red.h"

// ============================================================
// 主循环
// ============================================================

void RefereeSystemNode::readSerialLoop()
{
  serial_receiver_.readSerialData();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RefereeSystemNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void RefereeSystemNode::decisionLoop()
{
  // 更新TF
  getRobotPosition(robot_pos_);

  // 更新战场信息
  updateBattleInfo();

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

  if (!serial_port_.isOpen()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "串口未打开，跳过发送");
    return;
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
