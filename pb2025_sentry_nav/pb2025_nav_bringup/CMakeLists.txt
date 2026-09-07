cmake_minimum_required(VERSION 3.8)
project(pb2025_nav_bringup)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)


if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

# Add executable for PCD publisher
if(PCL_FOUND)
  add_executable(pcd_publisher src/pcd_publisher.cpp)
  ament_target_dependencies(pcd_publisher
    rclcpp
    sensor_msgs
    pcl_conversions
    pcl_ros)
  target_include_directories(pcd_publisher PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_definitions(pcd_publisher PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(pcd_publisher ${PCL_LIBRARIES})

  install(TARGETS
    pcd_publisher
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  pcd
  config
  behavior_trees
  rviz
)