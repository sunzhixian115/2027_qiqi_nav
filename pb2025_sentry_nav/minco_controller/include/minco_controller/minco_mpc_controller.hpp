#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "base_interfaces_demo/msg/mpc_position_command.hpp"

#include "log.hpp"
#include "minco_controller/mpc_solver.hpp"
#include "minco_controller/performance/mpc_performance_monitor.hpp"

namespace minco_controller {

class MincoMpcController : public nav2_core::Controller
{
public:
  // === Constructor & Lifecycle ===
  MincoMpcController() = default;
  ~MincoMpcController() override;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;//配置控制器

  void cleanup() override;//清理控制器
  void activate() override;//激活控制器
  void deactivate() override;//停用控制器

  // === Core Planning Interfaces ===
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;//计算速度命令

  void applyGravityCompensation(const nav_msgs::msg::Odometry::SharedPtr & odom, double & vx, double & vy);//应用重力补偿

  void setPlan(const nav_msgs::msg::Path & path) override;//设置计划

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;//设置速度限制

private:
  // === Callbacks ===
  void onOptPath(const base_interfaces_demo::msg::MpcPositionCommand::SharedPtr msg);//处理优化路径
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);//处理里程计数据

  // === Utility & Helper Functions ===
  void configureMpcPerfLogging(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);//配置 MPC 性能日志

  // --- Reference and Path Processing ---
  // Find the nearest point on cached trajectory and build horizon reference sequence.//从优化路径构建参考轨迹
  // @param curr 当前状态 (map)
  // @param out_ref 输出参考轨迹 (horizon 个点)
  // @return 是否成功构建参考轨迹
  bool buildReferenceFromOptPath(const State & curr, std::vector<ReferencePoint> & out_ref) const;//从优化路径构建参考轨迹

  bool transformPathToOdom(const base_interfaces_demo::msg::MpcPositionCommand::SharedPtr & opt,
    std::vector<base_interfaces_demo::msg::PositionCommand> & out_cmds) const;//将优化路径转换为里程计坐标系

  // --- Motion and Frame Utilities ---
  void compensateLeverArm(double v_lidar_x,
    double v_lidar_y,
    double omega_z,
    double yaw,
    double & vx_global,
    double & vy_global,
    double & omega_global) const;//补偿杠杆臂

  void extractGlobalVelocityAndYaw(const nav_msgs::msg::Odometry::SharedPtr & odom,
    double & vx_global,
    double & vy_global,
    double & omega_global,
    double & yaw_global) const;//从里程计数据中提取全局速度和航向角

  // --- Interpolation Utilities ---
  static double normalizeYaw(double yaw);//归航向角
  inline static double interpolateYaw(double yaw1, double yaw2, double alpha)//插值航向角
  {
    double diff = std::atan2(std::sin(yaw2 - yaw1), std::cos(yaw2 - yaw1));
    return yaw1 + diff * alpha;
  }

  inline static double interpolate(double v1, double v2, double alpha) { return v1 + (v2 - v1) * alpha; }//插值标量

  inline static Eigen::Vector2d interpolate(
    const Eigen::Vector2d & v1, const Eigen::Vector2d & v2, double alpha)//插值向量
  {
    return v1 + (v2 - v1) * alpha;
  }

  // --- Visualization ---
  void publishVisualization(const std::vector<State> & pred_path, const State & curr_state);

  // === ROS 2 Interfaces (Publishers, Subscribers, Timers) ===
  rclcpp::Subscription<base_interfaces_demo::msg::MpcPositionCommand>::SharedPtr opt_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_predict_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_real_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_mpc_pub_;

  // === TF & Costmap & Frames ===
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoMpcController")};
  std::string name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string global_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  // === State Variables & Caches ===
  // --- Visualization Cache ---
  std::vector<geometry_msgs::msg::PoseStamped> real_path_history_;
  rclcpp::Time last_real_path_pub_time_;

  // --- Runtime Data Cache ---
  mutable std::mutex data_mtx_;
  base_interfaces_demo::msg::MpcPositionCommand::SharedPtr latest_opt_path_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  // --- Reference Tracking State ---
  // Track reference index over time when trajectory is not updated to avoid jumping back to start.
  mutable double tracked_ref_idx_{0.0};
  mutable rclcpp::Time tracked_ref_time_;
  mutable uint32_t tracked_opt_traj_id_{0};
  mutable bool has_tracked_ref_{false};

  // --- Nav2 Plan Cache ---
  nav_msgs::msg::Path global_plan_;
  mutable std::mutex plan_mtx_;

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  MPCConfig mpc_config_;
  std::unique_ptr<MpcSolver> solver_;

  // === Configurations & Parameters ===
  // Dynamic speed limit from Nav2 setSpeedLimit().
  double speed_limit_{0.0};
  bool speed_limit_percentage_{false};

  // Other controller parameters.
  double fixed_wz_{0.0};
  double deadzone_speed_threshold_{0.02};
  double control_delay_compensation_{0.25};
  double lidar_offset_x_{0.0};
  double lidar_offset_y_{0.0};
  double lidar_roll_offset_ = 0.0;
  bool use_small_gyro_mode_{true};
  MpcPerformanceMonitor mpc_perf_monitor_;
  long long last_mpc_perf_stamp_ns_{0};

  // Low-pass filter for MPC output to prevent stick-slip oscillation on slopes.
  mutable Eigen::Vector3d prev_u_global_{0.0, 0.0, 0.0};
  double vel_smooth_alpha_{0.3};

  std::string opt_path_topic_{"/opt_path"};
  std::string odom_topic_{"/odin1/odometry"};

  // P0: Integral action for unmodeled disturbances (gravity on slopes, slip).
  // Leaky integrators of along-track and cross-track position errors.
  mutable Eigen::Vector2d integral_err_{0.0, 0.0};
  double k_i_along_{1.5};  // integral gain: along-track (boost on slopes)
  double k_i_cross_{3.0};  // integral gain: cross-track (precision on curves)
  double integral_decay_{0.98};
};

}  // namespace minco_controller
