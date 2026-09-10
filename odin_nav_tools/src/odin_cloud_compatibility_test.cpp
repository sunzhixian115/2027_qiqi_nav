#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class OdinCloudCompatibilityTest : public rclcpp::Node
{
public:
  OdinCloudCompatibilityTest() : Node("odin_cloud_compatibility_test")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/odin1/cloud_slam");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odin1/odometry");

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OdinCloudCompatibilityTest::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&OdinCloudCompatibilityTest::odomCallback, this, std::placeholders::_1));
    terrain_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "terrain_map", rclcpp::QoS(10),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr) { ++terrain_messages_; });
    sensor_scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "sensor_scan", rclcpp::QoS(10),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr) { ++sensor_scan_messages_; });
    chassis_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr) { ++chassis_odom_messages_; });
    report_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&OdinCloudCompatibilityTest::report, this));

    RCLCPP_INFO(get_logger(), "Checking %s with the same PointXYZI conversion used by terrain analysis.", cloud_topic_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    ++cloud_messages_;
    converted_points_ += cloud.points.size();
    last_cloud_frame_ = msg->header.frame_id;
    last_cloud_stamp_ = rclcpp::Time(msg->header.stamp);

    if (!printed_fields_) {
      std::ostringstream fields;
      for (const auto & field : msg->fields) {
        if (fields.tellp() > 0) {
          fields << ", ";
        }
        fields << field.name;
      }
      RCLCPP_INFO(
        get_logger(), "PointCloud2 fields: [%s], frame_id: %s, input points: %u, PointXYZI points: %zu",
        fields.str().c_str(), msg->header.frame_id.c_str(), msg->width * msg->height, cloud.points.size());
      printed_fields_ = true;
    }

    for (const auto & point : cloud.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        ++finite_points_;
      }
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    ++odom_messages_;
    last_odom_frame_ = msg->header.frame_id;
    last_child_frame_ = msg->child_frame_id;
  }

  void report()
  {
    if (cloud_messages_ == 0) {
      RCLCPP_WARN(get_logger(), "No cloud messages received from %s.", cloud_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Cloud conversion OK: %zu messages, %zu/%zu finite XYZ points, frame=%s, latest stamp=%.3f",
        cloud_messages_, finite_points_, converted_points_, last_cloud_frame_.c_str(), last_cloud_stamp_.seconds());
    }

    if (odom_messages_ == 0) {
      RCLCPP_WARN(get_logger(), "No odometry messages received from %s.", odom_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "Odometry OK: %zu messages, %s -> %s.", odom_messages_,
        last_odom_frame_.c_str(), last_child_frame_.c_str());
    }
    RCLCPP_INFO(
      get_logger(), "Downstream output: terrain_map=%zu, sensor_scan=%zu, odometry=%zu messages.",
      terrain_messages_, sensor_scan_messages_, chassis_odom_messages_);
  }

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string last_cloud_frame_;
  std::string last_odom_frame_;
  std::string last_child_frame_;
  rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
  bool printed_fields_{false};
  size_t cloud_messages_{0};
  size_t odom_messages_{0};
  size_t converted_points_{0};
  size_t finite_points_{0};
  size_t terrain_messages_{0};
  size_t sensor_scan_messages_{0};
  size_t chassis_odom_messages_{0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr chassis_odom_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdinCloudCompatibilityTest>());
  rclcpp::shutdown();
  return 0;
}
