#include "red.h"

void RefereeSystemNode::loadPointParameter(// 加载点参数
  const std::string & name,
  geometry_msgs::msg::Point & point)
{
  get_parameter(name + ".x", point.x);
  get_parameter(name + ".y", point.y);
}

// ============================================================
// 回调函数
// ============================================================

void RefereeSystemNode::cmdVelCallback(// 命令速度回调
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_twist_ = *msg;
}

void RefereeSystemNode::gimbleYawCallback(// 云台角度回调
  const std_msgs::msg::Float64::SharedPtr msg)
{
  gimble_yaw_ = msg->data;
}

void RefereeSystemNode::chasePointCallback(// 追逐点回调
  const geometry_msgs::msg::Point::SharedPtr msg)
{
  chase_point_ = *msg;

  chase_locked_ =
    !(std::abs(msg->x) < 1e-2 &&
      std::abs(msg->y) < 1e-2);
}
