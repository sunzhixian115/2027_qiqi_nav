#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class OdinCloudToXYZI final : public rclcpp::Node {
public:
  OdinCloudToXYZI() : Node("odin_cloud_to_xyzi") {
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "odin1/cloud_slam_xyzi", rclcpp::QoS(10));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "odin1/cloud_slam", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
        if (!input || input->height == 0 || input->width == 0) return;
        sensor_msgs::msg::PointCloud2 output;
        output.header = input->header;
        output.height = input->height;
        output.width = input->width;
        output.is_dense = input->is_dense;
        sensor_msgs::PointCloud2Modifier modifier(output);
        modifier.setPointCloud2Fields(
          5,
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
          "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(output.height * output.width);

        sensor_msgs::PointCloud2ConstIterator<float> in_x(*input, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*input, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*input, "z");
        sensor_msgs::PointCloud2ConstIterator<float> in_rgb(*input, "rgb");
        sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
        sensor_msgs::PointCloud2Iterator<float> out_i(output, "intensity");
        sensor_msgs::PointCloud2Iterator<float> out_rgb(output, "rgb");
        const auto count = static_cast<std::size_t>(input->height) * input->width;
        for (std::size_t n = 0; n < count;
             ++n, ++in_x, ++in_y, ++in_z, ++in_rgb,
             ++out_x, ++out_y, ++out_z, ++out_i, ++out_rgb) {
          *out_x = *in_x;
          *out_y = *in_y;
          *out_z = *in_z;
          *out_rgb = *in_rgb;
          // Keep RGB while providing the scalar field expected by terrain/Nav2.
          *out_i = 1.0f;
        }
        pub_->publish(std::move(output));
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdinCloudToXYZI>());
  rclcpp::shutdown();
  return 0;
}
