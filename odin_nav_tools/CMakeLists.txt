cmake_minimum_required(VERSION 3.8)
project(odin_nav_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)

add_executable(odin_cloud_compatibility_test src/odin_cloud_compatibility_test.cpp)
ament_target_dependencies(odin_cloud_compatibility_test
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions)
target_include_directories(odin_cloud_compatibility_test PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(odin_cloud_compatibility_test ${PCL_LIBRARIES})

add_executable(odin_synthetic_source src/odin_synthetic_source.cpp)
ament_target_dependencies(odin_synthetic_source rclcpp sensor_msgs nav_msgs)

add_executable(odin_cloud_to_xyzi src/odin_cloud_to_xyzi.cpp)
ament_target_dependencies(odin_cloud_to_xyzi rclcpp sensor_msgs nav_msgs)

install(TARGETS odin_cloud_compatibility_test odin_synthetic_source odin_cloud_to_xyzi DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/odin_static_tf.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
