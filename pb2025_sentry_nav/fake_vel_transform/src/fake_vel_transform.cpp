#include "fake_vel_transform/fake_vel_transform.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "FakeVelTransform Started! Frequency: 200Hz");

  // 参数声明与获取
  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel_nav");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "cmd_vel");
  this->declare_parameter<float>("init_spin_speed", 0.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);

  // 初始化 TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 发布器
  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);
  gimble_to_map_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("gimble_to_map_yaw", 10);

  // 订阅器
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));

  local_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    local_plan_topic_, 10, std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // 核心改动：定时器设为 5ms (200Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  
  // 转换速度并发布到底盘（底盘速度依然可以跟随导航频率，或者你也可以移入定时器）
  geometry_msgs::msg::Twist aft_tf_vel = transformVelocity(msg, current_robot_base_angle_);
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  
  last_controller_activate_time_ = this->get_clock()->now();
  // 注意：这里不再调用 publishGimbalYaw()，由定时器统一接管
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::SharedPtr /*msg*/)
{
  last_controller_activate_time_ = this->get_clock()->now();
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

// 专门负责计算并发布 Yaw 的函数
void FakeVelTransform::publishGimbalYaw()
{
  try {
    // 从 TF 树获取 map 到云台的实时姿态
    auto transform_stamped = tf_buffer_->lookupTransform(
      "map", robot_base_frame_, tf2::TimePointZero);
    
    double yaw = tf2::getYaw(transform_stamped.transform.rotation);
    std_msgs::msg::Float64 msg;
    msg.data = yaw;
    gimble_to_map_yaw_pub_->publish(msg);
  } catch (const tf2::TransformException & ex) {
    // 启动初期如果 TF 还没广播出来，抑制报错
  }
}

// 200Hz 定时器回调
void FakeVelTransform::publishTransform()
{
  // 1. 发布针对视觉/导航的 Fake TF
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);

  // 2. 关键：在这里主动调用发布函数，确保 Yaw 以 1000Hz 发布
  publishGimbalYaw();
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)