


./szx.sh



导航栈
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=07154 slam:=False use_robot_state_pub:=True



串口包

source install/setup.bash
ros2 launch serial_communication red.launch.py





追击包

source install/setup.bash
ros2 launch chase chase_launch.py



区块检测
source install/setup.bash

ros2 launch pb2025_region_monitor monitor_launch.py

录制
source install/setup.bash
ros2 bag record /livox/lidar /livox/imu /tf /tf_static /odometry -o my_map_bag



回放
source install/setup.bash

ros2 bag play my_map_bag --clock

pcd可视化

source install/setup.bash
ros2 launch pb2025_nav_bringup pcd_publisher_launch.py





查看TF树
ros2 run tf2_tools view_frames

视奸车体模型：
source install/setup.bash
ros2 launch pb2025_robot_description robot_description_launch.py


仿真导航
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmuc_2025 \
slam:=False


启动仿真
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py


ros2 launch small_point_lio small_point_lio.launch.py



建图模式：

source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True

建图

source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True

保存栅格地图：
ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1
我喜欢用的保存地图：
ros2 run nav2_map_server map_saver_cli -f 地图名字 -t /map

















