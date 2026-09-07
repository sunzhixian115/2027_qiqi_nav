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

// 雷达障碍物回调: Float32MultiArray [dist, yaw, dist, yaw, ...]
void RefereeSystemNode::relYawDistCallback(// 雷达障碍物回调
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  // 取最近的一个障碍物
  float min_dist = 999.0f;
  float nearest_yaw = 0.0f;
  bool found = false;

  for (size_t i = 0; i + 1 < msg->data.size(); i += 2)
  {
    float d = msg->data[i];
    float y = msg->data[i + 1];
    if (d > 0.01f && d < min_dist)
    {
      min_dist = d;
      nearest_yaw = y;
      found = true;
    }
  }

  if (found)
  {
    obs_dist_ = min_dist;
    obs_yaw_ = nearest_yaw;
    has_obs_ = true;
  }
  else
  {
    has_obs_ = false;// 没有障碍物
  }
}

// ============================================================
// RL动作回调
// ============================================================

void RefereeSystemNode::rlCmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  rl_twist_ = *msg;
  rl_active_ = true;
}

void RefereeSystemNode::rlShootCallback(
  const std_msgs::msg::UInt16::SharedPtr msg)
{
  rl_shoot_ = msg->data;
}

void RefereeSystemNode::rlReviveCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  rl_revive_ = msg->data;
}

void RefereeSystemNode::rlRemoteAmmoCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  rl_remote_ammo_ = msg->data;
}

void RefereeSystemNode::rlLocalAmmoCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  rl_local_ammo_ = msg->data;
}

