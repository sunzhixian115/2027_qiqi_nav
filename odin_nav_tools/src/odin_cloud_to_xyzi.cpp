#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct Point
{
  float x;
  float y;
  float z;
  float rgb;
  float distance_to_voxel_center;
};

class OdinCloudToXYZI final : public rclcpp::Node
{
public:
  OdinCloudToXYZI()
  : Node("odin_cloud_to_xyzi")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odin1/cloud_slam");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odin1/cloud_slam_xyzi");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odin1/odometry");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.05);
    crop_enabled_ = declare_parameter<bool>("crop_enabled", true);
    crop_size_x_ = declare_parameter<double>("crop_size_x", 2.0);
    crop_size_y_ = declare_parameter<double>("crop_size_y", 0.5);
    crop_size_z_ = declare_parameter<double>("crop_size_z", 0.5);
    min_range_ = declare_parameter<double>("min_range", 0.05);

    if (voxel_size_ <= 0.0) {
      throw std::invalid_argument("voxel_size must be positive");
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        odom_z_ = msg->pose.pose.position.z;
        have_odom_ = true;
      });


    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(10));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OdinCloudToXYZI::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Odin cloud adapter: %s -> %s, voxel=%.3f m, crop=%s [%.2f, %.2f, %.2f] m",
      input_topic_.c_str(), output_topic_.c_str(), voxel_size_, crop_enabled_ ? "on" : "off",
      crop_size_x_, crop_size_y_, crop_size_z_);
  }

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr input)
  {
    if (!input || input->height == 0 || input->width == 0) {
      return;
    }

    double odom_x;
    double odom_y;
    double odom_z;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!have_odom_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for Odin odometry before filtering the cloud");
        return;
      }
      odom_x = odom_x_;
      odom_y = odom_y_;
      odom_z = odom_z_;
    }

    sensor_msgs::PointCloud2ConstIterator<float> in_x(*input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*input, "z");
    sensor_msgs::PointCloud2ConstIterator<float> in_rgb(*input, "rgb");


    std::unordered_map<VoxelKey, Point, VoxelKeyHash> voxels;
    const std::size_t count = static_cast<std::size_t>(input->width) * input->height;
    voxels.reserve(count / 2U + 1U);

    const double half_x = crop_size_x_ * 0.5;
    const double half_y = crop_size_y_ * 0.5;
    const double half_z = crop_size_z_ * 0.5;
    const double min_range_sq = min_range_ * min_range_;
    std::size_t valid_points = 0;

    for (std::size_t n = 0; n < count; ++n, ++in_x, ++in_y, ++in_z, ++in_rgb) {
      const float x = *in_x;
      const float y = *in_y;
      const float z = *in_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const double dx = static_cast<double>(x) - odom_x;
      const double dy = static_cast<double>(y) - odom_y;
      const double dz = static_cast<double>(z) - odom_z;
      if (dx * dx + dy * dy + dz * dz < min_range_sq) {
        continue;
      }
      if (crop_enabled_ &&
        (std::abs(dx) > half_x || std::abs(dy) > half_y || std::abs(dz) > half_z))
      {
        continue;
      }

      ++valid_points;
      const VoxelKey key{
        static_cast<std::int64_t>(std::floor(static_cast<double>(x) / voxel_size_)),
        static_cast<std::int64_t>(std::floor(static_cast<double>(y) / voxel_size_)),
        static_cast<std::int64_t>(std::floor(static_cast<double>(z) / voxel_size_))};
      const double center_x = (static_cast<double>(key.x) + 0.5) * voxel_size_;
      const double center_y = (static_cast<double>(key.y) + 0.5) * voxel_size_;
      const double center_z = (static_cast<double>(key.z) + 0.5) * voxel_size_;
      const float distance = static_cast<float>(
        (static_cast<double>(x) - center_x) * (static_cast<double>(x) - center_x) +
        (static_cast<double>(y) - center_y) * (static_cast<double>(y) - center_y) +
        (static_cast<double>(z) - center_z) * (static_cast<double>(z) - center_z));
      const Point point{x, y, z, *in_rgb, distance};
      const auto found = voxels.find(key);
      if (found == voxels.end() || distance < found->second.distance_to_voxel_center) {
        voxels[key] = point;
      }
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = input->header;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(voxels.size());
    output.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(voxels.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> out_i(output, "intensity");
    sensor_msgs::PointCloud2Iterator<float> out_rgb(output, "rgb");
    for (const auto & entry : voxels) {
      const Point & point = entry.second;
      *out_x = point.x;
      *out_y = point.y;
      *out_z = point.z;
      *out_i = 1.0F;
      *out_rgb = point.rgb;
      ++out_x;
      ++out_y;
      ++out_z;
      ++out_i;
      ++out_rgb;
    }
    pub_->publish(std::move(output));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Odin cloud: input=%zu, after_filter=%zu, after_voxel=%zu",
      count, valid_points, voxels.size());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_topic_;
  double voxel_size_{0.05};
  bool crop_enabled_{true};
  double crop_size_x_{2.0};
  double crop_size_y_{0.5};
  double crop_size_z_{0.5};
  double min_range_{0.05};
  std::mutex odom_mutex_;
  bool have_odom_{false};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_z_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdinCloudToXYZI>());
  rclcpp::shutdown();
  return 0;
}
