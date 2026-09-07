#include "red.h"

// ============================================================
// 辅助函数
// ============================================================

void RefereeSystemNode::publishGoal(
  const geometry_msgs::msg::Point & p)
{
  current_target_point_ = p;

  // 目标没变 → 不重复发布
  const bool same_point =
    std::abs(p.x - last_published_goal_.x) < 0.01f &&
    std::abs(p.y - last_published_goal_.y) < 0.01f;

  if (goal_ever_published_ && same_point)
  {
    // 但最长 2 秒重发一次，防止 nav 启动时序丢目标
    if ((now() - last_publish_time_).seconds() < 2.0)
    {
      return;
    }
  }

  goal_msg_.header.stamp = now();
  goal_msg_.header.frame_id = map_frame_;
  goal_msg_.pose.position.x = p.x;
  goal_msg_.pose.position.y = p.y;
  goal_msg_.pose.position.z = 0.0;
  goal_msg_.pose.orientation.w = 1.0;
  goal_pub_->publish(goal_msg_);

  goal_ever_published_ = true;
  last_published_goal_ = p;
  last_publish_time_ = now();
}

bool RefereeSystemNode::getRobotPosition(
  geometry_msgs::msg::Point & position)
{
  try
  {
    auto t =
      tf_buffer_->lookupTransform(
        map_frame_,
        base_frame_,
        tf2::TimePointZero);

    position.x =
      t.transform.translation.x;

    position.y =
      t.transform.translation.y;

    position.z = 0.0;

    return true;
  }
  catch(tf2::TransformException & ex)
  {
    RCLCPP_WARN(
      get_logger(),
      "TF查询失败: %s",
      ex.what());

    return false;
  }
}

float RefereeSystemNode::getDistance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  float dx = a.x - b.x;
  float dy = a.y - b.y;

  return std::sqrt(dx * dx + dy * dy);
}

bool RefereeSystemNode::pointInPolygon(
  double px, double py,
  const std::vector<
    std::pair<double, double>> & vertices)
{
  const size_t n = vertices.size();

  bool inside = false;

  size_t j = n - 1;

  for(size_t i = 0; i < n; ++i)
  {
    const double xi = vertices[i].first;
    const double yi = vertices[i].second;

    const double xj = vertices[j].first;
    const double yj = vertices[j].second;

    if(
      ((yi > py) != (yj > py)) &&
      (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }

    j = i;
  }

  return inside;
}

int RefereeSystemNode::getRegion(
  const geometry_msgs::msg::Point & p)
{
  for(const auto & region : regions_)
  {
    if(
      pointInPolygon(
        p.x, p.y, region.vertices))
    {
      return region.name;
    }
  }

  return 0;
}

// ============================================================
// 速度积分预测
// 输入: rl_twist_(RL速度), robot_pos_(当前位置), gimble_yaw_(朝向)
// 输出: 0.25秒后的预测点
// 距离<0.5米时返回当前点(不移动)
// ============================================================

geometry_msgs::msg::Point RefereeSystemNode::predictPoint()
{
  constexpr double PREDICT_T = 0.25;  // = 1/4Hz

  float x0 = robot_pos_.x;
  float y0 = robot_pos_.y;

  float vx = rl_twist_.linear.x;
  float vy = rl_twist_.linear.y;

  float cos_yaw = std::cos(gimble_yaw_);
  float sin_yaw = std::sin(gimble_yaw_);

  geometry_msgs::msg::Point predicted;
  predicted.x = x0 + (vx * cos_yaw - vy * sin_yaw) * PREDICT_T;
  predicted.y = y0 + (vx * sin_yaw + vy * cos_yaw) * PREDICT_T;
  predicted.z = 0.0;

  // 距离太近返回当前点
  float dx = predicted.x - x0;
  float dy = predicted.y - y0;
  if (std::sqrt(dx * dx + dy * dy) < 0.3f)
  {
    predicted.x = x0;
    predicted.y = y0;
  }

  return predicted;
}


