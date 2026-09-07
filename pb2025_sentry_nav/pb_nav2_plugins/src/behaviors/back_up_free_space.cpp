// Copyright 2024 Polaris Xia
// Copyright 2025 Lihan Chen
// Modified with Centroid Logic and Robust Parameter Handling

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace pb_nav2_behaviors
{

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 1. 安全声明参数（给定默认值和明确的初始类型）
  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, "cost_threshold", rclcpp::ParameterValue(10.0)); // 默认 double
  nav2_util::declare_parameter_if_not_declared(node, "free_threshold", rclcpp::ParameterValue(10));   // 默认 int

  // 2. 获取基础参数
  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("visualize", visualize_);
  node->get_parameter("free_threshold", free_threshold_);

  // 3. 核心修复：健壮地获取 cost_threshold (解决 expected double got integer 报错)
  rclcpp::Parameter cost_param;
  if (node->get_parameter("cost_threshold", cost_param)) {
    if (cost_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      cost_threshold_ = cost_param.as_double();
    } else if (cost_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      // 如果 YAML 里写了 10，这里会自动转成 10.0
      cost_threshold_ = static_cast<double>(cost_param.as_int());
      RCLCPP_DEBUG(logger_, "Converting cost_threshold from integer to double.");
    }
  }

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
  
  RCLCPP_INFO(logger_, "BackUpFreeSpace configured with cost_threshold: %.2f", cost_threshold_);
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  // 等待地图服务
  if (!costmap_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(logger_, "Costmap service %s not available.", service_name_.c_str());
    return nav2_behaviors::Status::FAILED;
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  
  if (result.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Timeout calling costmap service.");
    return nav2_behaviors::Status::FAILED;
  }

  auto costmap = result.get()->map;

  if (!nav2_util::getCurrentPose(initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Could not get current robot pose.");
    return nav2_behaviors::Status::FAILED;
  }

  double pose_x = initial_pose_.pose.position.x;
  double pose_y = initial_pose_.pose.position.y;
  double yaw = tf2::getYaw(initial_pose_.pose.orientation);

  // 质心算法寻找空旷地带
  bool free_space_found = false;
  double current_search_r = 0.3; // 初始探测半径
  std::vector<geometry_msgs::msg::Point> free_points;

  while (!free_space_found && current_search_r <= max_radius_) {
    free_points.clear();
    for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
      for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
        auto idx = i + j * costmap.metadata.size_x;
        auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
        auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
        
        if (std::hypot(x - pose_x, y - pose_y) <= current_search_r) {
          if (static_cast<double>(costmap.data[idx]) <= cost_threshold_) {
            geometry_msgs::msg::Point p;
            p.x = x; p.y = y;
            free_points.push_back(p);
          }
        }
      }
    }

    if (static_cast<int>(free_points.size()) >= free_threshold_) {
      free_space_found = true;
    } else {
      current_search_r += 0.2; // 步进扩大搜索范围
    }
  }

  if (!free_space_found) {
    RCLCPP_WARN(logger_, "No free space found with cost <= %.2f", cost_threshold_);
    return nav2_behaviors::Status::FAILED;
  }

  // 计算目标点（空地的几何中心）
  double avg_x = 0.0, avg_y = 0.0;
  for (const auto& p : free_points) {
    avg_x += p.x; avg_y += p.y;
  }
  avg_x /= free_points.size();
  avg_y /= free_points.size();

  // 计算航向角
  double target_yaw = std::atan2(avg_y - pose_y, avg_x - pose_x);
  double angle_diff = target_yaw - yaw;
  
  // 角度归一化到 [-PI, PI]
  angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

  // 设定速度分量
  twist_x_ = std::cos(angle_diff) * command->speed;
  twist_y_ = std::sin(angle_diff) * command->speed;
  
  command_x_ = command->target.x; 
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;

  RCLCPP_INFO(logger_, "Recovery started: Target direction %.2f deg relative to robot", angle_diff * 180.0 / M_PI);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  // 超时检查
  if ((end_time_ - clock_->now()).seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "Recovery behavior timed out.");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    return nav2_behaviors::Status::FAILED;
  }

  double dist_travelled = std::hypot(initial_pose_.pose.position.x - current_pose.pose.position.x,
                                     initial_pose_.pose.position.y - current_pose.pose.position.y);

  feedback_->distance_traveled = static_cast<float>(dist_travelled);
  action_server_->publish_feedback(feedback_);

  // 到达距离目标
  if (dist_travelled >= std::fabs(command_x_)) {
    stopRobot();
    RCLCPP_INFO(logger_, "Recovery distance reached.");
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = twist_x_;
  cmd_vel->linear.y = twist_y_;

  // 运行中的实时碰撞检测
  geometry_msgs::msg::Pose2D cur_pose2d;
  cur_pose2d.x = current_pose.pose.position.x;
  cur_pose2d.y = current_pose.pose.position.y;
  cur_pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(dist_travelled, cmd_vel.get(), cur_pose2d)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision detected during recovery move! Aborting.");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));
  return nav2_behaviors::Status::RUNNING;
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)