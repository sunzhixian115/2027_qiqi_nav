#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class PcdPublisher : public rclcpp::Node
{
public:
    PcdPublisher()
    : Node("pcd_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "pcd_cloud", 10);

        this->declare_parameter<std::string>("pcd_file", "");
        const std::string pcd_file = this->get_parameter("pcd_file").as_string();
        if (pcd_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Set the pcd_file parameter before starting pcd_publisher");
            return;
        }
        
        // 加载PCD文件
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", pcd_file.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded PCD file with %lu points", cloud_->points.size());
        
        // 创建定时器，定期发布点云
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 每100ms发布一次
            std::bind(&PcdPublisher::publish_pcd, this));
    }

private:
    void publish_pcd()
    {
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_, output);
        output.header.frame_id = "map";  // 设置坐标系
        output.header.stamp = this->now();  // 设置时间戳
        
        publisher_->publish(output);
        RCLCPP_INFO_ONCE(this->get_logger(), "Published PCD data to topic: pcd_cloud");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZ>};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcdPublisher>());
    rclcpp::shutdown();
    return 0;
}
