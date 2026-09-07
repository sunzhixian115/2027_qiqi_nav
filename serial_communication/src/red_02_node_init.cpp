#include "red.h"

// ============================================================
// RefereeSystemNode 构造函数
// ============================================================

RefereeSystemNode::RefereeSystemNode()
: Node("referee_system_node"),
  serial_receiver_(serial_port_)
{
  // ========================================================
  // 参数声明
  // ========================================================

  declare_parameter<std::string>(
    "map_frame",
    "map");

  declare_parameter<std::string>(
    "base_frame",
    "base_link");

  declare_parameter<std::string>(
    "lidar_frame",
    "livox_frame");

  declare_parameter<double>(
    "arrive_tolerance",
    0.5);

  declare_parameter<double>("attack_point_1.x", 0.0);
  declare_parameter<double>("attack_point_1.y", 0.0);

  declare_parameter<double>("attack_point_2.x", 0.0);
  declare_parameter<double>("attack_point_2.y", 0.0);

  declare_parameter<double>("high_point_1.x", 0.0);
  declare_parameter<double>("high_point_1.y", 0.0);

  declare_parameter<double>("high_point_2.x", 0.0);
  declare_parameter<double>("high_point_2.y", 0.0);

  declare_parameter<double>("high_point_3.x", 0.0);
  declare_parameter<double>("high_point_3.y", 0.0);

  declare_parameter<double>("pre_point.x", 0.0);
  declare_parameter<double>("pre_point.y", 0.0);

  declare_parameter<double>("z1.x", 0.0);
  declare_parameter<double>("z1.y", 0.0);

  declare_parameter<double>("z2.x", 0.0);
  declare_parameter<double>("z2.y", 0.0);

  declare_parameter<double>("back_point.x", 0.0);
  declare_parameter<double>("back_point.y", 0.0);

  declare_parameter<double>("supply_point.x", 0.0);
  declare_parameter<double>("supply_point.y", 0.0);

  declare_parameter<double>("base_point.x", 0.0);
  declare_parameter<double>("base_point.y", 0.0);

  declare_parameter<double>("home_point_1.x", 0.0);
  declare_parameter<double>("home_point_1.y", 0.0);

  declare_parameter<double>("home_point_2.x", 0.0);
  declare_parameter<double>("home_point_2.y", 0.0);

  declare_parameter<double>("home_point_3.x", 0.0);
  declare_parameter<double>("home_point_3.y", 0.0);

  declare_parameter<double>("home_point_4.x", 0.0);
  declare_parameter<double>("home_point_4.y", 0.0);

  declare_parameter<double>("no_outpost_point_1.x", 0.0);
  declare_parameter<double>("no_outpost_point_1.y", 0.0);

  declare_parameter<double>("no_outpost_point_2.x", 0.0);
  declare_parameter<double>("no_outpost_point_2.y", 0.0);

  // RL观察空间: 固定目标坐标
  declare_parameter<double>("outpost_x", 11.0);
  declare_parameter<double>("outpost_y", 11.35);
  declare_parameter<double>("base_x", 2.4);
  declare_parameter<double>("base_y", 7.5);
  declare_parameter<double>("energy_x", 14.0);
  declare_parameter<double>("energy_y", 7.5);

  // 地图偏移量 (地图yaml origin 取反)
  declare_parameter<double>("origin_x", -3.8);
  declare_parameter<double>("origin_y", -7.5);

  for(int i = 1; i <= 5; ++i)
  {
    declare_parameter(
      "region_" + std::to_string(i),
      std::vector<double>{});
  }

  // ========================================================
  // 参数获取
  // ========================================================

  get_parameter("map_frame", map_frame_);

  get_parameter("base_frame", base_frame_);

  get_parameter("lidar_frame", lidar_frame_);

  get_parameter(
    "arrive_tolerance",
    arrive_tolerance_);

  loadPointParameter("attack_point_1", attack_point_1_);
  loadPointParameter("attack_point_2", attack_point_2_);

  loadPointParameter("high_point_1", high_point_1_);
  loadPointParameter("high_point_2", high_point_2_);
  loadPointParameter("high_point_3", high_point_3_);

  loadPointParameter("pre_point", pre_point_);

  loadPointParameter("z1", z1_);

  loadPointParameter("z2", z2_);

  loadPointParameter("back_point", back_point_);

  loadPointParameter("supply_point", supply_point_);

  loadPointParameter("base_point", base_point_);
  loadPointParameter("home_point_1", home_point_1_);
  loadPointParameter("home_point_2", home_point_2_);
  loadPointParameter("home_point_3", home_point_3_);
  loadPointParameter("home_point_4", home_point_4_);
  loadPointParameter("no_outpost_point_1", no_outpost_point_1_);
  loadPointParameter("no_outpost_point_2", no_outpost_point_2_);

  // RL观察空间: 固定目标坐标

  get_parameter("outpost_x", outpost_x_);
  get_parameter("outpost_y", outpost_y_);
  get_parameter("base_x", base_x_);
  get_parameter("base_y", base_y_);
  get_parameter("energy_x", energy_x_);
  get_parameter("energy_y", energy_y_);

  // 地图偏移
  get_parameter("origin_x", origin_x_);
  get_parameter("origin_y", origin_y_);
  RCLCPP_INFO(
    get_logger(),
    "地图偏移: origin_x=%.2f origin_y=%.2f",
    origin_x_, origin_y_);


  // 区域顶点列表
  for(int i = 1; i <= 5; ++i)
  {
    std::vector<double> raw;
    if(
      get_parameter(
        "region_" + std::to_string(i), raw) &&
      raw.size() >= 3)
    {
      Region r;
      r.name = static_cast<int>(raw[0]);
      for(size_t j = 1; j + 1 < raw.size(); j += 2)
      {
        r.vertices.emplace_back(
          raw[j], raw[j + 1]);
      }
      regions_.push_back(std::move(r));
    }
  }

  // ========================================================
  // 串口
  // ========================================================

  declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  std::string serial_port_name;
  get_parameter("serial_port", serial_port_name);

  try
  {
    serial_port_.setPort(serial_port_name);

    serial_port_.setBaudrate(115200);

    serial::Timeout timeout =
      serial::Timeout::simpleTimeout(1000);

    serial_port_.setTimeout(timeout);

    serial_port_.open();
  }
  catch(serial::IOException & e)
  {
    RCLCPP_ERROR(
      get_logger(),
      "串口打开失败: %s",
      e.what());
  }

  if(serial_port_.isOpen())
  {
    RCLCPP_INFO(
      get_logger(),
      "串口已打开");
  }

  // ========================================================
  // publisher
  // ========================================================

  track_target_pub_ =
    create_publisher<
      base_interfaces_demo::msg::Target>(
        "/track_target",
        10);

  goal_pub_ =
    create_publisher<
      geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10);

  obs_pub_ =
    create_publisher<
      std_msgs::msg::Float32MultiArray>(
        "/rl_obs",
        10);

  // referee_system 话题
  ref_self_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/red_standard_robot1", 10);
  ref_hero_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/red_hero_robot1", 10);
  ref_engineer_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/red_engineer_robot1", 10);
  ref_infantry3_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/red_standard_robot2", 10);
  ref_infantry4_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/red_standard_robot3", 10);
  ref_enemy_hero_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/blue_hero_robot1", 10);
  ref_enemy_engineer_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/blue_engineer_robot1", 10);
  ref_enemy_infantry3_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/blue_standard_robot1", 10);
  ref_enemy_infantry4_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/blue_standard_robot2", 10);
  ref_enemy_sentry_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/robot_status/blue_sentry_robot1", 10);
  ref_outpost_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/outpost_status", 10);
  ref_base_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/base_status", 10);
  ref_energy_pub_ = create_publisher<rmoss_interfaces::msg::RobotStatus>(
    "/referee_system/energy_status", 10);
  ref_game_pub_ = create_publisher<rmoss_interfaces::msg::GameStatus>(
    "/referee_system/game_status", 10);

  // ========================================================
  // subscriber
  // ========================================================

  cmd_vel_sub_ =
    create_subscription<
      geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(
          &RefereeSystemNode::cmdVelCallback,
          this,
          std::placeholders::_1));

  chase_point_sub_ =
    create_subscription<
      geometry_msgs::msg::Point>(
        "/chase_point",
        10,
        std::bind(
          &RefereeSystemNode::chasePointCallback,
          this,
          std::placeholders::_1));

  gimble_yaw_sub_ =
    create_subscription<
      std_msgs::msg::Float64>(
        "/gimble_to_map_yaw",
        10,
        std::bind(
          &RefereeSystemNode::gimbleYawCallback,
          this,
          std::placeholders::_1));

  // 雷达障碍物(dist, yaw) - 雷达坐标系
  rel_yaw_dist_sub_ =
    create_subscription<
      std_msgs::msg::Float32MultiArray>(
        "/pc_detector/rel_yaw_dist",
        10,
        std::bind(
          &RefereeSystemNode::relYawDistCallback,
          this,
          std::placeholders::_1));

  // ========================================================
  // RL动作订阅
  // ========================================================

  rl_cmd_vel_sub_ =
    create_subscription<
      geometry_msgs::msg::Twist>(
        "/rl_cmd_vel",
        10,
        std::bind(
          &RefereeSystemNode::rlCmdVelCallback,
          this,
          std::placeholders::_1));

  rl_shoot_sub_ =
    create_subscription<
      std_msgs::msg::UInt16>(
        "/rl_shoot",
        10,
        std::bind(
          &RefereeSystemNode::rlShootCallback,
          this,
          std::placeholders::_1));

  rl_revive_sub_ =
    create_subscription<
      std_msgs::msg::Bool>(
        "/rl_revive_with_coins",
        10,
        std::bind(
          &RefereeSystemNode::rlReviveCallback,
          this,
          std::placeholders::_1));

  rl_remote_ammo_sub_ =
    create_subscription<
      std_msgs::msg::Bool>(
        "/rl_remote_ammo_exchange",
        10,
        std::bind(
          &RefereeSystemNode::rlRemoteAmmoCallback,
          this,
          std::placeholders::_1));

  rl_local_ammo_sub_ =
    create_subscription<
      std_msgs::msg::Bool>(
        "/rl_local_ammo_exchange",
        10,
        std::bind(
          &RefereeSystemNode::rlLocalAmmoCallback,
          this,
          std::placeholders::_1));

  // ========================================================
  // tf
  // ========================================================

  tf_buffer_ =
    std::make_shared<tf2_ros::Buffer>(
      get_clock());

  tf_listener_ =
    std::make_shared<
      tf2_ros::TransformListener>(
        *tf_buffer_);

  // ========================================================
  // timer
  // ========================================================

  read_timer_ =
    create_wall_timer(
      100ms,
      std::bind(
        &RefereeSystemNode::readSerialLoop,
        this));

  decision_timer_ =
    create_wall_timer(
      100ms,
      std::bind(
        &RefereeSystemNode::decisionLoop,
        this));

  send_timer_ =
    create_wall_timer(
      50ms,
      std::bind(
        &RefereeSystemNode::sendLoop,
        this));

  // 初始化缓存为无效值，防止开机默认(0,0)被当成有效目标
  radar_saved_point_.x = -1.0f;
  radar_saved_point_.y = -1.0f;

  RCLCPP_INFO(
    get_logger(),
    "裁判系统节点已启动");
}

// ============================================================
// 参数加载
// ============================================================

