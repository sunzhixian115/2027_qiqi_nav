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

  declare_parameter<std::string>("serial_port", "/dev/chassis");
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

  RCLCPP_INFO(
    get_logger(),
    "裁判系统节点已启动");
}

// ============================================================
// 参数加载
// ============================================================
