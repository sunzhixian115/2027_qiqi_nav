cmake_minimum_required(VERSION 3.8)
project(serial_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(base_interfaces_demo REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 源文件缺失，暂时注释
# add_executable(rm_serial_desicion src/rm_serial_desicion.cpp)
# add_executable(rm_go_to_high src/rm_go_to_high.cpp)
# add_executable(rm_go_to_home src/rm_go_to_home.cpp)
# add_executable(rm_rviz_test src/rm_rviz_test.cpp)
# add_executable(blue src/blue.cpp)

add_executable(nav_goal_guard src/nav_goal_guard.cpp)
add_executable(red
  src/red_01_serial_receiver.cpp
  src/red_02_node_init.cpp
  src/red_03_callbacks.cpp
  src/red_04_main_loop.cpp
  src/red_05_battle_info.cpp
  src/red_06_decision.cpp
  src/red_08_utils.cpp
)
add_executable(region_monitor src/region_monitor.cpp)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
  serial
  rclcpp_action
  nav2_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  base_interfaces_demo
  std_msgs
)

ament_target_dependencies(nav_goal_guard ${COMMON_DEPS})
ament_target_dependencies(red ${COMMON_DEPS})
ament_target_dependencies(region_monitor
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS
  nav_goal_guard
  red
  region_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
