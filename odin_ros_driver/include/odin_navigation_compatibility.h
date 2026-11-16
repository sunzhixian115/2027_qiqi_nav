#pragma once

#include <array>
#include <cmath>
#include <vector>
#include <mutex>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odin_ros_driver {

// 旧导航以 odin1_base_link 表示雷达参考点；新版 SDK 的位姿参考点是 IMU。
// 在驱动出口完成转换，保留旧导航源码和安装标定。
class NavigationCompatibility {
public:
    using Odometry = nav_msgs::msg::Odometry;
    using Publisher = rclcpp::Publisher<Odometry>::SharedPtr;

    NavigationCompatibility(rclcpp::Node::SharedPtr node,
                            Publisher standard, Publisher highfreq)
        : node_(node), output_{standard, highfreq}, static_tf_(node), dynamic_tf_(node) {
        native_[0] = node_->create_publisher<Odometry>(
            "odin1/odometry_imu", rclcpp::SensorDataQoS());
        native_[1] = node_->create_publisher<Odometry>(
            "odin1/odometry_highfreq_imu", rclcpp::SensorDataQoS());
    }

    void setExtrinsics(const Eigen::Matrix4d& imu_from_lidar) {
        std::lock_guard<std::mutex> lock(mutex_);
        const Eigen::Matrix3d rotation = imu_from_lidar.block<3, 3>(0, 0);
        // 使用设备实际在线标定，不把官网名义外参当作这台设备的精确标定。
        const bool valid = imu_from_lidar.allFinite() &&
            (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-3 &&
            std::abs(rotation.determinant() - 1.0) < 1e-3 &&
            (imu_from_lidar.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-6;
        if (!valid) {
            ready_ = false;
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "设备 IMU/LiDAR 外参无效，停止输出导航兼容里程计");
            return;
        }
        imu_from_lidar_ = imu_from_lidar;
        if (ready_) return;
        ready_ = true;
        previous_ = {};

        geometry_msgs::msg::TransformStamped lidar;
        lidar.header.stamp = node_->now();
        lidar.header.frame_id = "odin1_base_link";
        lidar.child_frame_id = "lidar";
        lidar.transform.rotation.w = 1.0;
        static_tf_.sendTransform(lidar);
        RCLCPP_INFO(node_->get_logger(),
            "导航兼容已启用：IMU 位姿转换到 odin1_base_link；传感器 TF 由驱动发布");
    }

    void publish(const Odometry& native, bool highfreq) {
        const size_t stream = highfreq ? 1 : 0;
        native_[stream]->publish(native);
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ready_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "等待设备 WIWC 外参，暂不输出导航兼容里程计");
            return;
        }
        const auto& p = native.pose.pose.position;
        const auto& orientation = native.pose.pose.orientation;
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-6 ||
            !Eigen::Vector3d(p.x, p.y, p.z).allFinite()) return;
        q.normalize();
        Eigen::Matrix4d odom_from_imu = Eigen::Matrix4d::Identity();
        odom_from_imu.block<3, 3>(0, 0) = q.toRotationMatrix();
        odom_from_imu.block<3, 1>(0, 3) = Eigen::Vector3d(p.x, p.y, p.z);
        const Eigen::Matrix4d odom_from_lidar = odom_from_imu * imu_from_lidar_;
        const Eigen::Vector3d position = odom_from_lidar.block<3, 1>(0, 3);
        Eigen::Quaterniond rotation(odom_from_lidar.block<3, 3>(0, 0));
        rotation.normalize();

        auto output = native;
        output.child_frame_id = "odin1_base_link";
        output.pose.pose.position.x = position.x();
        output.pose.pose.position.y = position.y();
        output.pose.pose.position.z = position.z();
        output.pose.pose.orientation.w = rotation.w();
        output.pose.pose.orientation.x = rotation.x();
        output.pose.pose.orientation.y = rotation.y();
        output.pose.pose.orientation.z = rotation.z();
        // SDK 的 covariance 数组装的是外参，不是统计协方差。
        output.pose.covariance.fill(0.0);
        output.twist.covariance.fill(0.0);
        output.twist.twist = geometry_msgs::msg::Twist();

        // 用各流自己的消息时间计算雷达参考点速度，包含转动产生的杆臂位移。
        const int64_t stamp = rclcpp::Time(native.header.stamp).nanoseconds();
        auto& previous = previous_[stream];
        const double dt = (stamp - previous.stamp) * 1e-9;
        if (previous.valid && dt <= 0.0) return;
        if (previous.valid && dt <= 1.0) {
            const Eigen::Vector3d linear = rotation.conjugate() *
                ((position - previous.position) / dt);
            Eigen::Quaterniond delta = previous.rotation.conjugate() * rotation;
            if (delta.w() < 0.0) delta.coeffs() *= -1.0;
            const Eigen::AngleAxisd angle(delta);
            const Eigen::Vector3d angular = rotation.conjugate() *
                (previous.rotation * (angle.axis() * (angle.angle() / dt)));
            output.twist.twist.linear.x = linear.x();
            output.twist.twist.linear.y = linear.y();
            output.twist.twist.linear.z = linear.z();
            output.twist.twist.angular.x = angular.x();
            output.twist.twist.angular.y = angular.y();
            output.twist.twist.angular.z = angular.z();
        }
        if (!highfreq) {
            // WIWC 是在线标定，lidar→imu 随实际里程计时间更新，不能伪装成恒定外参。
            geometry_msgs::msg::TransformStamped imu;
            imu.header = native.header;
            imu.header.frame_id = "lidar";
            imu.child_frame_id = "imu";
            const Eigen::Matrix4d lidar_from_imu = imu_from_lidar_.inverse();
            Eigen::Quaterniond q_il(lidar_from_imu.block<3, 3>(0, 0));
            q_il.normalize();
            imu.transform.translation.x = lidar_from_imu(0, 3);
            imu.transform.translation.y = lidar_from_imu(1, 3);
            imu.transform.translation.z = lidar_from_imu(2, 3);
            imu.transform.rotation.w = q_il.w();
            imu.transform.rotation.x = q_il.x();
            imu.transform.rotation.y = q_il.y();
            imu.transform.rotation.z = q_il.z();
            dynamic_tf_.sendTransform(imu);
        }
        previous = {true, stamp, position, rotation};
        output_[stream]->publish(output);
    }

private:
    struct Previous {
        bool valid = false;
        int64_t stamp = 0;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    };
    rclcpp::Node::SharedPtr node_;
    std::array<Publisher, 2> output_, native_;
    tf2_ros::StaticTransformBroadcaster static_tf_;
    tf2_ros::TransformBroadcaster dynamic_tf_;
    std::mutex mutex_;
    bool ready_ = false;
    Eigen::Matrix4d imu_from_lidar_ = Eigen::Matrix4d::Identity();
    std::array<Previous, 2> previous_;
};
}  // namespace odin_ros_driver
