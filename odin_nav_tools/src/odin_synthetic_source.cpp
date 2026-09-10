#include <chrono>
#include <cmath>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class OdinSyntheticSource : public rclcpp::Node
{
public:
  OdinSyntheticSource() : Node("odin_synthetic_source")
  {
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/odin1/cloud_slam", rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odin1/odometry", rclcpp::QoS(10));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdinSyntheticSource::publish, this));
    RCLCPP_INFO(get_logger(), "Publishing synthetic Odin1 XYZRGB cloud and odometry at 10 Hz.");
  }

private:
  void publish()
  {
    const auto stamp = now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "odin1_base_link";
    odom.pose.pose.orientation.w = 1.0;
    odom_pub_->publish(odom);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "odom";
    cloud.height = 1;
    cloud.width = 400;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(cloud.width);

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> rgb(cloud, "rgb");
    for (uint32_t i = 0; i < cloud.width; ++i, ++x, ++y, ++z, ++rgb) {
      const float angle = static_cast<float>(i) * 0.0157F;
      *x = 2.0F * std::cos(angle);
      *y = 2.0F * std::sin(angle);
      *z = 0.15F + static_cast<float>(i % 20) * 0.02F;
      *rgb = 0.0F;
    }
    cloud_pub_->publish(cloud);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdinSyntheticSource>());
  rclcpp::shutdown();
  return 0;
}
