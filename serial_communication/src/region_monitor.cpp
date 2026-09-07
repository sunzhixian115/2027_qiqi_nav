#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// 射线法判定点是否在多边形内
static bool pointInPolygon(double px, double py,
                           const std::vector<std::pair<double, double>> & vertices) {
  const size_t n = vertices.size();
  bool inside = false;
  size_t j = n - 1;
  for (size_t i = 0; i < n; ++i) {
    const double xi = vertices[i].first;
    const double yi = vertices[i].second;
    const double xj = vertices[j].first;
    const double yj = vertices[j].second;
    if (((yi > py) != (yj > py)) &&
        (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
    j = i;
  }
  return inside;
}

class RegionMonitorNode : public rclcpp::Node
{
public:
  RegionMonitorNode()
  : Node("region_monitor_node")
  {
    loadRegionsFromParams();

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/region_markers", 10);
    region_pub_ = create_publisher<std_msgs::msg::UInt16>("/region_num", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&RegionMonitorNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "区域监控节点已启动，加载了 %zu 个区域", regions_.size());
  }

private:
  struct Region {
    int name;
    std::vector<std::pair<double, double>> vertices;
  };

  void loadRegionsFromParams() {
    for (int i = 1; i <= 20; ++i) {
      std::string key = "region_" + std::to_string(i);
      declare_parameter(key, std::vector<double>{});

      std::vector<double> raw;
      if (!get_parameter(key, raw) || raw.size() < 6) {
        continue;
      }

      Region r;
      r.name = static_cast<int>(raw[0]);
      for (size_t j = 1; j + 1 < raw.size(); j += 2) {
        r.vertices.emplace_back(raw[j], raw[j + 1]);
      }

      RCLCPP_INFO(get_logger(), "加载区域 %d: %zu 个顶点", r.name, r.vertices.size());
      regions_.push_back(std::move(r));
    }
  }

  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "等待 TF: %s", e.what());
      return;
    }

    const double robot_x = t.transform.translation.x;
    const double robot_y = t.transform.translation.y;

    int current_region = 0;
    visualization_msgs::msg::MarkerArray marker_array;
    const auto stamp = now();

    for (size_t i = 0; i < regions_.size(); ++i) {
      const auto & region = regions_[i];
      const auto & verts = region.vertices;
      const bool is_inside = pointInPolygon(robot_x, robot_y, verts);

      if (is_inside) {
        current_region = region.name;
      }

      // 画区域边框
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = stamp;
      marker.ns = "region_monitor";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.05;

      marker.color.r = is_inside ? 1.0 : 0.0;
      marker.color.g = is_inside ? 0.0 : 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker.points.reserve(verts.size() + 1);
      for (const auto & v : verts) {
        geometry_msgs::msg::Point p;
        p.x = v.first;
        p.y = v.second;
        p.z = 0.0;
        marker.points.push_back(p);
      }
      geometry_msgs::msg::Point p0;
      p0.x = verts.front().first;
      p0.y = verts.front().second;
      p0.z = 0.0;
      marker.points.push_back(p0);

      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker_array.markers.push_back(marker);

      // 文字标签
      visualization_msgs::msg::Marker text;
      text.header.frame_id = "map";
      text.header.stamp = stamp;
      text.ns = "region_labels";
      text.id = static_cast<int>(i);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.scale.z = 0.6;
      text.color.r = is_inside ? 1.0 : 1.0;
      text.color.g = is_inside ? 0.0 : 1.0;
      text.color.b = is_inside ? 0.0 : 1.0;
      text.color.a = 1.0;

      double cx = 0.0, cy = 0.0;
      for (const auto & v : verts) {
        cx += v.first;
        cy += v.second;
      }
      cx /= static_cast<double>(verts.size());
      cy /= static_cast<double>(verts.size());

      text.pose.position.x = cx;
      text.pose.position.y = cy;
      text.pose.position.z = 0.3;
      text.pose.orientation.w = 1.0;
      text.text = "R" + std::to_string(region.name);

      text.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker_array.markers.push_back(text);
    }

    marker_pub_->publish(marker_array);

    if (current_region > 0) {
      std_msgs::msg::UInt16 msg;
      msg.data = static_cast<uint16_t>(current_region);
      region_pub_->publish(msg);
    }
  }

  std::vector<Region> regions_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr region_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RegionMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
