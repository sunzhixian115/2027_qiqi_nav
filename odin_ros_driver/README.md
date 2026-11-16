> 本包为 odin_0915 集成的 Odin1 0.14.4 驱动：在驱动内部兼容旧导航位姿接口，保留 map→odom 的求逆修正。
> 详细启动方式、坐标系和旧地图验证限制见 [导航接入说明](docs/导航接入说明.md)。
> 当前默认配置关闭厂商机体 TF；厂商 demo 中建议开启该 TF 的说明不适用于本导航配置。

# Odin_ROS_Driver 使用说明

Odin 传感器模块（Manifold Tech Ltd.）的 ROS 驱动套件。

[Odin1 官方 Wiki](https://manifoldtechltd.github.io/wiki/Odin1/Cover.html)

## 兼容性

- ROS 1：推荐使用长期支持版本 Noetic。
- ROS 2：推荐使用长期支持版本 Humble。

## 重要说明

本驱动包提供点云 SLAM 应用的核心功能，面向特定使用场景，供进行二次开发的专业技术人员使用。实际部署时，用户需要根据应用场景进行优化和定制开发，以满足运行要求。

## 1. 版本

当前驱动版本：**v0.14.4**。

要求的设备固件版本：**v0.14.0**。

## 2. 环境准备

### 2.1 操作系统要求

- Ubuntu 20.04：适用于 ROS Noetic 和 ROS 2 Foxy。
- Ubuntu 22.04：适用于 ROS 2 Humble。
- Ubuntu 18.04：目前不支持。
- Ubuntu 24.04：官方暂不支持，经过部分修改后可能可以运行。

### 2.2 依赖项

- OpenCV ≥ 4.2.0，推荐 4.5.5 或 4.8.0。请确保驱动和 ROS 使用一致的 OpenCV 版本，避免多个版本混用。
- yaml-cpp。
- 线程库。
- OpenSSL。
- Eigen3。

### 2.3 安装依赖

#### 2.3.1 系统依赖

```shell
sudo apt update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

#### 2.3.2 yaml-cpp

```shell
sudo apt update
sudo apt install -y libyaml-cpp-dev
```

#### 2.3.3 libusb

```shell
sudo apt update
sudo apt install -y libusb-1.0-0-dev
```

#### 2.3.4 OpenCV

```shell
sudo apt update
sudo apt-get install libopencv-dev
```

#### 2.3.5 ROS

根据使用的发行版参考安装说明：

- [ROS Noetic 安装说明](https://wiki.ros.org/noetic/Installation)。
- [ROS 2 Foxy 安装说明](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)。
- [ROS 2 Humble 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)。

## 3. 编译与运行

### 3.1 创建 udev 规则

```shell
sudo vim /etc/udev/rules.d/99-odin-usb.rules
```

向 `99-odin-usb.rules` 文件添加以下内容：

```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```

重新加载规则，然后重新插拔设备：

```shell
sudo udevadm control --reload
sudo udevadm trigger
```

### 3.2 获取源码

```shell
git clone https://github.com/manifoldsdk/odin_ros_driver.git catkin_ws/src/odin_ros_driver
```

请将源码放在 `[ros_workspace]/src/` 目录下，否则可能出现编译错误。

### 3.3 编译

#### 3.3.1 ROS 1（以 Noetic 为例）

```shell
source /opt/ros/noetic/setup.bash
./script/build_ros.sh
```

#### 3.3.2 ROS 2（以 Foxy 为例）

```shell
source /opt/ros/foxy/setup.bash
./script/build_ros2.sh
```

### 3.4 运行

#### 3.4.1 ROS 1（以 Noetic 为例）

```shell
source [ros_workspace]/devel/setup.bash
roslaunch odin_ros_driver [launch file]
```

- `odin_ros_driver`：功能包名称。
- `[launch file]`：启动文件名称。
- `[ros_workspace]`：用户的 ROS 工作空间。

示例：

```shell
roslaunch odin_ros_driver odin1_ros1.launch
```

#### 3.4.2 ROS 2（以 Foxy 为例）

```shell
source [ros2_workspace]/install/setup.bash
ros2 launch odin_ros_driver [launch file]
```

- `odin_ros_driver`：功能包名称。
- `[launch file]`：启动文件名称。
- `[ros2_workspace]`：用户的 ROS 2 工作空间。

ROS 2 示例启动命令：

```shell
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

### 3.5 工作模式

通过 `config/control_command.yaml` 中的 `custom_map_mode` 参数选择工作模式。

#### 里程计模式

设置 `custom_map_mode: 0`，启用纯里程计模式。该模式下，`map` 和 `odom` 坐标系的位姿相同。

如果里程计发生漂移，可以执行 `./set_param.sh algo_reset 1` 动态重置算法。

#### SLAM 建图模式

设置 `custom_map_mode: 1`，启用 SLAM 模式。该模式在里程计模式基础上增加**回环检测**和**地图保存**功能。

启动驱动后，Odin1 自动开始建图并缓存地图数据。采集完成后，在驱动源码目录执行 `./set_param.sh save_map 1`，保存本次程序启动以来采集的地图。保存目录和文件名分别由 `mapping_result_dest_dir`、`mapping_result_file_name` 指定；未指定时采用默认值。

地图保存目录留空时，默认保存到驱动目录的 `map/{driver_start_time}/`，不使用 `.ros`。使用 symlink-install 时对应驱动源码的 map 目录；可通过 `mapping_result_dest_dir` 显式指定其他目录。

首次保存后，可以再次执行命令保存新的地图。每次保存均生成一个新文件，**连续两次保存至少间隔 5 秒**。

地图原点对应程序启动时的 `odom` 坐标系原点。

#### 重定位模式

设置 `custom_map_mode: 2`，并通过 `relocalization_map_abs_path` 指定已有地图的**绝对路径**。

启动后，Odin1 根据当前视角和指定地图进行重定位。为提高成功率，建议初始位置距离原建图轨迹不超过 1 米，姿态误差不超过 ±10°。

重定位效果高度依赖环境。在特征明显的场景中，超出 1 米或 10° 也可能成功；部分环境则要求更严格的初始位置和姿态。应在目标场景中测试实际容差。

如果初始重定位失败，系统暂时进入备用 SLAM 模式，此时不支持保存地图。用户可以移动 Odin1，系统会在后台持续尝试重定位。成功后，将发布 `map` 与 `odom` 之间的 TF。初始化后轻微晃动或移动设备可能有助于重定位。

原文列出的 `/odin1/cloud_slam`、`/odin1/odom`、`/odin1/highodom`、`/odin1/path` 数据位于 `odom` 坐标系下。如需转换到 `map` 坐标系，应应用相应 TF。当前版本里程计的实际话题名称见 4.3 节。

## 4. 文件结构与数据格式

### 4.1 文件结构

```shell
Odin_ROS_Driver/                // ROS 1 / ROS 2 驱动包
    3rdparty/                   // 第三方库
    src/
        host_sdk_sample.cpp     // 示例源代码
        yaml_parser.cpp         // YAML 参数读取实现
        rawCloudRender.cpp      // 彩色点云渲染实现
        depth_image_ros_node.cpp // ROS 1 深度图节点
        depth_image_ros2_node.cpp // ROS 2 深度图节点
        pcd2depth_ros.cpp       // ROS 1 点云转深度图节点
        pcd2depth_ros2.cpp      // ROS 2 点云转深度图节点
        pointcloud_depth_converter.cpp // 点云转深度图实现
        cloud_reprojection_ros.cpp // 点云重投影节点（ROS 1 / ROS 2）
        cloud_reprojector.cpp   // 点云重投影核心逻辑
    lib/
        liblydHostApi_amd.a     // x86_64 平台静态库
        liblydHostApi_arm.a     // ARM 平台静态库
    include/
        host_sdk_sample.h       // 示例头文件
        lidar_api_type.h        // API 数据结构定义
        lidar_api.h             // API 函数声明
        yaml_parser.h           // 参数文件读取接口
        rawCloudRender.h        // 彩色点云渲染接口
        data_logger.h           // 数据保存与日志接口
        depth_image_ros_node.hpp // ROS 1 深度图节点
        depth_image_ros2_node.hpp // ROS 2 深度图节点
        pointcloud_depth_converter.hpp // 点云转深度图接口
        cloud_reprojection_ros_node.hpp // 点云重投影节点接口（ROS 1 / ROS 2）
        cloud_reprojector.hpp   // 点云重投影核心类
    config/
        control_command.yaml    // 驱动控制参数文件
        calib.yaml              // 设备专用标定文件，连接时从设备获取
    launch_ROS1/
        odin1_ros1.launch       // ROS 1 启动文件
    launch_ROS2/
        odin1_ros2.launch.py    // ROS 2 启动文件
    script/
        build_ros1.sh           // ROS 1 编译安装脚本
        build_ros2.sh           // ROS 2 编译安装脚本
    recorddata/                 // 可导入 MindCloud 的录制数据
    log/                        // 日志目录
        Driver_{timestamp}/     // 每次驱动启动对应的日志目录
            Conn_{timestamp}/   // 每次设备连接对应的日志目录
                dev_status.csv  // 设备状态日志
    README.md                   // 使用说明
    CMakeLists.txt              // CMake 构建文件
    License                     // 许可证文件
```

### 4.2 启动文件

| 启动文件 | 用途 |
|---|---|
| `odin1_ros1.launch` | ROS 1 的 Odin1 基础操作示例 |
| `odin1_ros2.launch.py` | ROS 2 的 Odin1 基础操作示例 |

### 4.3 ROS 话题

驱动内部参数位于 `config/control_command.yaml`。下表列出常用话题及控制参数；部分原文话题名与当前源码不同，以下按当前版本源码列出。

| 话题 | 控制参数 | 说明 |
|---|---|---|
| `odin1/imu` | `sendimu` | IMU 数据 |
| `odin1/image` | `sendrgb` | 从设备原始 JPEG 解码得到的 RGB 相机图像，格式为 `bgr8` |
| `odin1/image/undistorted` | `sendrgbundistort` | 使用设备标定文件去畸变后的 RGB 图像 |
| `odin1/image/compressed` | `sendrgbcompressed` | 设备原始 JPEG 压缩图像 |
| `odin1/cloud_raw` | `senddtof` | 原始点云 |
| `odin1/cloud_render` | `sendcloudrender` | 主机结合原始点云、RGB 图像和设备标定生成的彩色点云 |
| `odin1/cloud_slam` | `sendcloudslam` | SLAM 点云 |
| `odin1/odometry` | `sendodom` | 里程计 |
| `odin1/odometry_highfreq` | `sendodom` | 高频里程计 |
| `odin1/path` | `showpath` | 里程计轨迹 |
| `tf` | `sendodom` | TF 坐标变换树 |
| `odin1/depth_img_competetion` | `senddepth` | 稠密深度图示例，计算开销较大。与去畸变图像对应，建议直接订阅，不建议使用 `echo`；原始数值已经是深度，无需再次转换 |
| `odin1/depth_img_competetion_cloud` | `senddepth` | 稠密深度点云示例，计算开销较大 |
| `odin1/reprojected_image` | `sendreprojection` | 主机利用里程计将 `cloud_slam` 重投影到相机图像上的结果 |

### 4.4 数据格式

#### 1. 原始点云 `cloud_raw`

包含以下字段：

```
float32 x             // X 轴坐标，单位：米
float32 y             // Y 轴坐标，单位：米
float32 z             // Z 轴坐标，单位：米
uint8  intensity      // 反射强度，范围：0–255
uint16 confidence     // 点置信度，典型范围约 0–1300，越大越可靠；建议过滤阈值 30–35，按场景调整
float32 offset_time   // 相对于基准时间戳的偏移量，单位：秒
```

在 PCL 中使用该自定义格式时，先定义点类型：

```cpp
/*** LS ***/
namespace ls_ros {
    struct EIGEN_ALIGN16 Point {
        float x;
        float y;
        float z;
        uint8_t intensity;
        uint16_t confidence;
        float offset_time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // ls_ros 命名空间

POINT_CLOUD_REGISTER_POINT_STRUCT(ls_ros::Point,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (uint8_t, intensity, intensity)
      (uint16_t, confidence, confidence)
      (float offset_time , offset_time)
)
```

然后将 ROS 的 `sensor_msgs::PointCloud2` 消息转换成 PCL 点云：

```
pcl::PointCloud<ls_ros::Point> ls_cloud;
pcl::fromROSMsg(*msg, ls_cloud);
```

#### 2. SLAM 点云与彩色点云

`cloud_slam` 和 `cloud_render` 包含以下字段：

```
float32 x             // X 轴坐标，单位：米
float32 y             // Y 轴坐标，单位：米
float32 z             // Z 轴坐标，单位：米
float32 rgb           // RGB 颜色值
```

### 4.5 其他功能参数

| 参数 | 说明 |
|---|---|
| `use_host_ros_time` | 时间同步模式：`0` 使用 Odin 内部系统时间，通常推荐；`1` 使用主机收到数据时的 ROS 时间，仅用于特定需求；`2` 通过类似 NTP 的方式，将 Odin1 时间对齐到主机时间轴 |
| `strict_usb3.0_check` | 严格检查 USB 3.0；关闭后允许在低于 USB 3.0 的连接上运行 |
| `recorddata` | 以专有格式录制数据，可导入 MindCloud(TM) 后处理。存储开销较大，测试中 10 分钟数据约占 9.5 GB。IMU、图像、点云、位姿、旋转数据的时间戳与 `use_host_ros_time` 采用相同的对齐策略；原文将 `1`、`2` 均称为 NTP 模式，具体时间语义以本表对应模式的说明及实际代码为准 |
| `devstatuslog` | 记录设备状态，包括 SoC 温度、CPU 和内存占用、dToF 温度、数据收发速率等，保存到日志目录中的 CSV 文件。每次启动创建新的日志文件 |
| `showcamerapose` | 显示相机位姿和视场角 |
| `custom_map_mode` | `0`：纯里程计；`1`：带回环检测的建图模式，可保存地图；`2`：重定位模式，需指定地图绝对路径，成功后输出 `map` 与 `odom` 之间的 TF |
| `custom_init_pos` | 初始化位姿，原文标注为暂未使用 |
| `relocalization_map_abs_path` | 重定位所用地图的绝对路径 |
| `mapping_result_dest_dir`、`mapping_result_file_name` | 建图模式下地图保存目录和文件名；未指定时使用默认值 |

### 4.6 通过 ROS 服务在线调节 AE/AWB

驱动提供 4 个 ROS 服务，可在主数据流持续运行时，从另一终端调节相机自动曝光（AE）和自动白平衡（AWB）。底层 SDK 调用与驱动主控制路径共享互斥锁，可与正常运行并发调用。

#### 服务列表

| 服务名称 | 类型 | 用途 |
|---|---|---|
| `/odin1/get_ae` | `odin_ros_driver/srv/GetAe` | 查询当前 AE 状态 |
| `/odin1/get_awb` | `odin_ros_driver/srv/GetAwb` | 查询当前 AWB 状态 |
| `/odin1/set_ae` | `odin_ros_driver/srv/SetAe` | 设置 AE 模式，以及手动曝光时间和增益 |
| `/odin1/set_awb` | `odin_ros_driver/srv/SetAwb` | 设置 AWB 模式，以及手动 R、B 通道增益 |

#### 4.6.1 请求字段、范围与物理含义

**`SetAe.Request`**

| 字段 | 范围 | 含义 |
|---|---|---|
| `mode` | `0`（AUTO）或 `1`（MANUAL） | `0`：设备自动调节曝光，忽略下方两个浮点参数；`1`：锁定自动曝光，应用指定参数 |
| `exposure_time` | `0.0001`～`0.033` 秒，仅手动模式 | 每帧传感器曝光时间；越长越亮，但运动模糊也越明显 |
| `gain` | `1.0`～`64.0`，仅手动模式 | 模拟增益；越大越亮，但信噪比越差 |

**`SetAwb.Request`**

| 字段 | 范围 | 含义 |
|---|---|---|
| `mode` | `0`（AUTO）或 `1`（MANUAL） | `0`：设备自动调节白平衡；`1`：锁定白平衡，应用指定增益 |
| `rgain` | `0.1`～`4.0`，仅手动模式 | R 通道增益；相对 `bgain` 越大，画面越偏暖、偏黄红 |
| `bgain` | `0.1`～`4.0`，仅手动模式 | B 通道增益；相对 `rgain` 越大，画面越偏冷、偏蓝 |

Gr、Gb 通道增益由设备固定为 `1.0`，不可调节。

#### 4.6.2 响应字段

4 个服务均返回 `success`（布尔值）和 `rc`（32 位整数）；查询服务还返回当前状态。

**`GetAe.Response`**

| 字段 | 典型范围 | 含义 |
|---|---|---|
| `exposure_time` | `0.0001`～`0.033` 秒 | 当前曝光时间 |
| `gain` | `1.0`～`64.0` | 当前模拟增益 |
| `iso` | `100`～`6400` | 等效 ISO |
| `brightness` | `0`～`255` | 平均帧亮度 |
| `is_converged` | `0` 或 `1` | `1` 表示 AE 已收敛 |
| `env_lv` | `0`～`15` | 环境亮度指数，越大表示越亮 |
| `fps` | 约 `10`、`14.5`、`29` | 当前帧率 |

**`GetAwb.Response`**

| 字段 | 典型范围 | 含义 |
|---|---|---|
| `rgain`、`bgain` | `0.1`～`4.0` | R、B 通道增益 |
| `grgain`、`gbgain` | 固定为 `1.0` | 由设备固定的 Gr、Gb 通道增益 |
| `cct` | `2500`～`8000` K | 相关色温 |
| `ccri` | `-50`～`50` | 色温偏离指数，`0` 表示位于普朗克轨迹上 |
| `is_converged` | `0` 或 `1` | `1` 表示 AWB 已收敛 |

#### 4.6.3 `rc` 返回码

| `rc` | 含义 |
|---|---|
| `0` | 成功 |
| `400` | 设备载荷过短 |
| `401` | 设备不支持该操作码 |
| `402` | 参数长度错误 |
| `403` | **参数越界**，常见于手动参数超出上述范围 |
| `404` | 设备端套接字错误 |
| `405` | 设备端 `ae_control` 无应答，请确认 `lydapp` 已运行 |
| `255`（`0xFF`） | `ae_control` 返回未知操作码 |
| `-1` | SDK 未初始化 |
| `-2`～`-5` | USB 传输异常、超时或应答格式错误 |
| `-100` | **驱动尚未打开设备**，请等待连接成功后再调用 |

#### 4.6.4 调用示例

ROS 2 Humble：在一个终端启动驱动，在另一个终端执行：

```bash
source install/setup.bash

# 查询当前状态
ros2 service call /odin1/get_ae  odin_ros_driver/srv/GetAe
ros2 service call /odin1/get_awb odin_ros_driver/srv/GetAwb

# 设置 AE 为自动
ros2 service call /odin1/set_ae odin_ros_driver/srv/SetAe "{mode: 0}"

# 设置 AE 为手动，10 毫秒曝光，增益 4.0
ros2 service call /odin1/set_ae odin_ros_driver/srv/SetAe \
  "{mode: 1, exposure_time: 0.010, gain: 4.0}"

# 设置 AWB 为手动，rgain=1.5、bgain=2.0
ros2 service call /odin1/set_awb odin_ros_driver/srv/SetAwb \
  "{mode: 1, rgain: 1.5, bgain: 2.0}"

# 恢复自动模式
ros2 service call /odin1/set_ae  odin_ros_driver/srv/SetAe  "{mode: 0}"
ros2 service call /odin1/set_awb odin_ros_driver/srv/SetAwb "{mode: 0}"

# 查看服务完整定义
ros2 interface show odin_ros_driver/srv/SetAe
```

ROS 1 Noetic：启动驱动后，在另一个终端执行：

```bash
source devel/setup.bash

# 查询当前状态
rosservice call /odin1/get_ae
rosservice call /odin1/get_awb

# 设置 AE 为手动
rosservice call /odin1/set_ae  "{mode: 1, exposure_time: 0.010, gain: 4.0}"

# 设置 AWB 为手动
rosservice call /odin1/set_awb "{mode: 1, rgain: 1.5, bgain: 2.0}"

# 恢复自动模式（ROS 1 要求填齐全部字段）
rosservice call /odin1/set_ae  "{mode: 0, exposure_time: 0.0, gain: 0.0}"
rosservice call /odin1/set_awb "{mode: 0, rgain: 0.0, bgain: 0.0}"

# 查看服务完整定义
rossrv show odin_ros_driver/SetAe
```

#### 4.6.5 不同场景的推荐起始参数

**AE 参数：`exposure_time` 和 `gain`**

| 场景 | `exposure_time` | `gain` |
|---|---|---|
| 明亮室外 | `0.001`～`0.005` 秒 | `1.0`～`2.0` |
| 普通室内 | `0.008`～`0.015` 秒 | `2.0`～`8.0` |
| 暗光环境 | `0.020`～`0.030` 秒 | `8.0`～`32.0` |
| 极暗环境 | `0.033` 秒 | `32.0`～`64.0` |

**AWB 参数：`rgain` 和 `bgain`**

| 目标色调 | `rgain` | `bgain` |
|---|---|---|
| 暖色，如钨丝灯、夕阳 | `2.0`～`2.5` | `1.0`～`1.2` |
| 中性，如 D65 日光 | `1.5`～`1.7` | `1.8`～`2.0` |
| 冷色，如阴天、荧光灯 | `1.2`～`1.4` | `2.2`～`2.6` |
| 极冷色 | `1.0` | `3.0`～`4.0` |

#### 4.6.6 注意事项

- 服务等待设备应答时最长可能阻塞约 10 秒，正常延迟为几十毫秒。
- 手动模式不会跨驱动或设备重启保留，每次重新连接默认恢复 AUTO。
- `rc = -100` 表示驱动尚未打开设备，应等待日志显示设备连接成功后再调用。
- 最大曝光时间受帧周期 `1/fps` 限制。在约 29 Hz、周期约 34 毫秒的情况下，`0.033` 秒已接近帧周期上限。原文此处写作 `dtof_fps = 290`；当前配置的约 29 Hz RGB 档位使用 `rgb_fps: 290`、`dtof_fps: 100`，请按配置文件中的配对表设置。

### 4.7 运行时设备控制

除 AE/AWB 服务外，驱动还提供其他运行时 ROS 服务，并支持 `set_param.sh` 命令文件接口，可用于保存地图、重置算法和获取设备日志等操作。

完整服务列表、请求与响应字段，以及命令文件接口的用法见 [运行时服务操作说明](docs/runtime_service_operations.md)。

## 5. 常见问题

### 5.1 重新启动主机 SDK 时出现段错误

**错误信息**

`No device connected after 60 seconds`

**处理方法**

1. 断开并重新接通 Odin 模块电源。
2. 等设备重新启动后，再启动 SDK。

### 5.2 编译时库链接失败

**错误信息**

`ld: cannot find -llydHostApi`，或出现符号查找错误。

**处理方法**

1. 清理旧编译产物。以下是原文提供的命令。

ROS 1：

```shell
rm -rf devel/ build/  
```

ROS 2：

```shell
rm -rf devel/ install/ log/ 
```

2. 重新运行编译安装脚本。

### 5.3 Docker 无法显示图形界面

**错误信息**

`Unable to open X display` 或 `No protocol specified`。

**处理方法**

```shell
xhost + # 允许 Docker 容器访问图形显示服务
```

### 5.4 驱动读取版本失败后退出

**错误信息**

```shell
<ERROR><api.cpp:lidar_get_version:672>: get device version fail.
get version failed.
```

**处理方法**

原文将该问题归因为设备固件过旧，建议升级到匹配的固件版本。实际排查时也应确认 USB 通信和驱动、固件兼容性。

### 5.5 RViz 长时间无响应

**现象**

RViz 无响应，随后终端输出 `Device disconnected, waiting for reconnection...`。

**处理方法**

断开并重新接通 Odin 模块电源。

### 5.6 设备无响应

**错误信息**

`Missed ok response from device,probably wrong interaction procedure.`

**处理方法**

按 5.1 节的方法重新上电并初始化。

### 5.7 缺少相机标定文件或标定内容不完整

**错误信息**

`ERROR：Missing camera node 'cam_0'`

**处理方法**

重新插拔 USB，重新获取设备标定文件。

### 5.8 启动数据流后立即报告设备断开

**错误信息**

```shell
Device ready and streams activated
Device detaching...
Wating for device reconnection...
Device disconnected, waiting for reconnection...
```

**原因**

在连接办公 Wi-Fi、有线网络等复杂网络环境的 ROS 2 系统中较常见。ROS 2 默认使用广播发现，复杂网络可能导致 ROS 发布阻塞，进一步引起设备连接异常。

**处理方法**

如果不需要跨设备通信，将 ROS 2 通信限制在本机：

```shell
export ROS_LOCALHOST_ONLY=1
```

如果需要跨设备通信，应尽量简化网络环境，推荐使用仅包含所需设备的小型局域网。

### 5.9 启动数据流后驱动立即崩溃

**错误信息**

```shell
Device ready and streams activated
[host_sdk_sample-2] process has died ......
```

**排查方法**

将 `control_command.yaml` 中的 `sendrgb` 设为 `0`，关闭 `odin1/image` 后重试。如果此时驱动可以运行，可能是系统同时安装多个 OpenCV 版本，导致库混用。

**处理方法**

卸载不需要的 OpenCV 版本，保留一套完整且一致的版本，然后重新编译驱动。需要保留多个版本时，参见 5.16 节。

### 5.10 出现 `TF_OLD_DATA ignoring data` 警告

**错误信息**

```shell
[rviz2-3] Warning: TF_OLD_DATA ignoring data from the past for frame odin1_base_link at time 20.547632 according to authority Authority undetectable
[rviz2-3] Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained
[rviz2-3]          at line 294 in ./src/buffer_core.cpp
```

**原因**

ROS 和 RViz 检测到 TF 时间戳冲突，忽略了部分数据。如果保持驱动运行时给 Odin 重新上电，设备内部时间会归零，新数据时间戳便可能与 RViz 缓存的旧数据冲突。

**处理方法**

点击 RViz 界面底部的重置按钮，清空内部状态。

### 5.11 出现 `unknown cmd code: xx` 错误

**错误信息**

```shell
<ERROR><api.cpp:cmd_data_deal:418>: unknow command code 21.
```

**原因**

驱动与设备固件版本不匹配，导致驱动无法解码新固件增加的数据。

**处理方法**

使用相互匹配的驱动和设备固件版本。

### 5.12 USB 访问错误：`LIBUSB_ERROR_BUSY` 或 `LIBUSB_ERROR_ACCESS`

**错误信息**

```shell
libusb: error [udev_hotplug_event] ignoring udev action bind
LIBUSB_ERROR_BUSY
```

或：

```shell
libusb: error [_get_usbfs_fd] libusb couldn't open USB device /dev/bus/usb/xxx/xxx, errno=13
LIBUSB_ERROR_ACCESS
```

**原因**

- `LIBUSB_ERROR_BUSY`：其他进程已占用 USB 设备，例如同时运行多个驱动实例，或先前崩溃的程序仍持有设备句柄。
- `LIBUSB_ERROR_ACCESS`：当前用户没有 USB 访问权限，常见于缺少 udev 规则或用户权限不足。

**处理 `LIBUSB_ERROR_BUSY`**

1. 检查是否有其他驱动实例：

```shell
ps aux | grep host_sdk_sample
```

2. 结束已有驱动实例：

```shell
killall host_sdk_sample
```

3. 如果问题仍存在，重新插拔 USB 以重置设备状态。

**处理 `LIBUSB_ERROR_ACCESS`**

1. 创建 `/etc/udev/rules.d/99-odin.rules`，写入设备访问规则：

```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```

2. 重新加载规则：

```shell
sudo udevadm control --reload-rules
sudo udevadm trigger
```

3. 也可临时以 `sudo` 启动驱动，但不推荐在生产环境中使用。原文命令如下，启动文件名应按实际文件调整：

```shell
sudo -E ros2 launch odin_ros_driver odin_ros_driver.launch.py
```

4. 确保当前用户属于 `plugdev` 组：

```shell
sudo usermod -aG plugdev $USER
```

修改用户组后，注销并重新登录使其生效。

### 5.13 ros2 bag 录制丢失高频话题

**现象**

使用 `ros2 bag record` 录制时，低频话题（点云、图像、普通里程计、`wiwc`）完整，但 `/odin1/imu` 和 `/odin1/odometry_highfreq` 等 400 Hz 话题出现丢帧。离线分析可见消息间隔达到正常周期的两倍以上，而 SDK 或独立运行的 `ros2 topic hz` 不报告丢帧。

**原因**

驱动以 `RELIABLE` QoS 发布这两个高频话题。原文描述的录制订阅配置为 `history = keep_last`、`depth = 10`，在 400 Hz 下只能缓存约 25 毫秒数据。录制端短暂阻塞，例如磁盘刷新、mcap/sqlite 分块写入或调度抖动，都可能使订阅队列溢出，在订阅端丢弃最旧数据。SDK 和发布端不受影响，因此其日志和独立订阅者不一定能观察到丢帧。

**处理方法**

使用 `script/rosbag2_qos.yaml` 增大录制端高频话题订阅队列：

```yaml
# script/rosbag2_qos.yaml
/odin1/imu:
  reliability: reliable
  history: keep_last
  depth: 4000

/odin1/odometry_highfreq:
  reliability: reliable
  history: keep_last
  depth: 4000
```

录制时通过 `--qos-profile-overrides-path` 应用该配置：

```shell
ros2 bag record -a \
    --qos-profile-overrides-path src/odin_ros_driver/script/rosbag2_qos.yaml \
    -o my_bag
```

也可以只录制指定话题：

```shell
ros2 bag record \
    --qos-profile-overrides-path src/odin_ros_driver/script/rosbag2_qos.yaml \
    -o my_bag \
    /odin1/imu /odin1/odometry_highfreq /odin1/odometry /odin1/wiwc /odin1/cloud_raw
```

**进一步优化**

如果应用配置后仍丢帧，尤其是磁盘较慢时，可以使用 mcap 和更大的内部缓存，并增大内核 UDP 套接字缓冲区：

```shell
# 使用 mcap 后端和更大的内部缓存（比 sqlite3 快）。
ros2 bag record -s mcap --max-cache-size 1073741824 \
    --qos-profile-overrides-path src/odin_ros_driver/script/rosbag2_qos.yaml \
    -o my_bag \
    /odin1/imu /odin1/odometry_highfreq ...

# 增大内核 UDP 套接字缓冲区（400 Hz 可靠传输的常见瓶颈，默认仅 208 KB）。
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.wmem_max=33554432
```

**ROS 1 是否存在同样的问题？**

原文说明，ROS 1 使用基于 TCP 的发布和订阅，各端以 `queue_size` 控制队列，不存在相同的 ROS 2 QoS 配置匹配问题。此驱动的 ROS 1 分支已将 IMU 和高频里程计发布队列设为 `4000`（见 `include/host_sdk_sample.h` 的 `initialize_publishers`），因此不需要上述 ROS 2 专用配置。这不代表 ROS 1 在任意负载下都不会丢失录制数据。

### 5.14 Ubuntu 24.04 / ROS 2 Jazzy 下 RViz 点云与 TF 转换失败

**现象**

RViz 的 Fixed Frame 设为 `odom` 时，`lidar` 坐标系下的 `cloud_raw` 无法显示，`odom → imu → lidar` 链条出现变换或时间外推错误，而相同配置在 Ubuntu 22.04 / Humble 下可以运行。

**原因**

设备点云时间戳通常比里程计时间戳超前约 100 毫秒。原文说明 Jazzy 的 tf2 时间戳查找更严格；如果 TF 只随里程计发布，缓存中可能没有覆盖点云时间戳的变换，导致查找失败。

**处理方法**

在 `config/control_command.yaml` 中将 `tf_extra_publish_rate` 设为大于 `0` 的数值，例如 `100`。驱动会启用额外定时器，按指定频率补发 `odom → imu` 和 `imu → lidar` 变换，推进时间戳以覆盖点云数据时间。

```yaml
# config/control_command.yaml
tf_extra_publish_rate: 100  # 0：关闭；大于 0：补发频率，单位 Hz（ROS 2 Jazzy / Ubuntu 24.04 推荐启用）
```

Ubuntu 22.04 / Humble 和 ROS 1 通常不需要此设置，可以保持为 `0`。

### 5.15 低性能主机出现 USB 内存或传输错误

**错误信息**

```shell
<ERROR>: submit x transfer fail LIBUSB_ERROR_NO_MEM.
<ERROR>: transfer error: LIBUSB_TRANSFER_ERROR.
```

**现象**

驱动在性能较强的 PC 上正常，但部署到性能较弱的主机，例如嵌入式 ARM 板，运行一段时间后，SDK 先报告 `LIBUSB_ERROR_NO_MEM`，随后连续出现 `transfer error`。设备发送速率未变化，变化的是运行 SDK 的主机。

**原因**

libusb 提交到内核的异步传输会占用有限的 usbfs 内存池，其配置位于 `/sys/module/usbcore/parameters/usbfs_memory_mb`，原文默认值为 16 MB。较慢的主机可能无法及时处理完成事件，导致未完成的 OUT 发送请求堆积，最终返回 `LIBUSB_ERROR_NO_MEM`，随后产生 `LIBUSB_TRANSFER_ERROR`。

**处理方法**

1. 运行时增大 usbfs 内存上限，立即生效，重启后恢复：

```shell
# 查看当前值（默认 16 MB）
cat /sys/module/usbcore/parameters/usbfs_memory_mb

# 调大到 128 MB
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

2. 如需重启后仍保留配置，可选择以下一种方式：

```shell
# 方式 A：内核启动参数（编辑 GRUB / 板子的 bootargs）
usbcore.usbfs_memory_mb=128

# 方式 B：modprobe 配置
echo "options usbcore usbfs_memory_mb=128" | sudo tee /etc/modprobe.d/usbcore.conf
```

**说明**

增大 `usbfs_memory_mb` 是提高缓冲余量的缓解措施。SDK 的 USB 层已限制在途 OUT 请求数量，并在各终止状态释放传输缓冲，避免暂时错误演变成持续内存泄漏。对于较慢主机或大文件传输，例如重定位地图，仍建议提供较大的 usbfs 内存池，如 128 MB。

### 5.16 RGB 发布时因 cv_bridge / OpenCV 不匹配而崩溃

**现象**

驱动可以连接设备并激活数据流，但发布第一帧 RGB 时就发生段错误或堆损坏。这与 5.9 节的问题原因相同：ROS 的 `cv_bridge` 使用一种 OpenCV 编译，驱动却链接另一种版本。

**原因**

启用 `sendrgb: 1` 后，主机使用 `cv::imdecode(...)` 解码图像，经 `cv_bridge::toImageMsg()` 发布。同一进程若加载两套 `libopencv_core`，`cv::Mat` 可能由一套库创建、另一套库释放，跨越 ABI 边界而导致内存损坏。

**受影响的 ROS 版本**

这不是某个 ROS 版本独有的问题。无论 ROS 1 或 ROS 2，只要 `cv_bridge` 与驱动使用的 OpenCV 不一致，就可能受影响。Jetson 等嵌入式平台较常见，因为预装 OpenCV 可能与 ROS 软件包编译时的版本不同。

下表保留原文列出的发行版、分支和典型版本，具体可用分支与版本应以实际仓库及系统为准。

| ROS 发行版 | 类型 | `vision_opencv` 分支 | 典型系统 OpenCV |
|---|---|---|---|
| Kinetic（Ubuntu 16.04） | ROS 1 | `kinetic` | 3.3.1 |
| Melodic（Ubuntu 18.04） | ROS 1 | `melodic` | 3.2 / 3.4 |
| Noetic（Ubuntu 20.04） | ROS 1 | `noetic` | 4.2.0（apt）/ 4.5.4（Jetson） |
| Foxy（Ubuntu 20.04） | ROS 2 | `foxy` | 4.2.0 |
| Galactic（Ubuntu 20.04） | ROS 2 | `galactic` | 4.2.0 |
| Humble（Ubuntu 22.04） | ROS 2 | `humble` | 4.5.x |
| Iron（Ubuntu 22.04） | ROS 2 | `iron` | 4.5.x |
| Jazzy（Ubuntu 24.04） | ROS 2 | `rolling` | 4.5.x / 4.6.x |

如果仅使用一套 OpenCV，且与 `cv_bridge` 一致，就不会触发这里描述的版本混用问题。

**处理方法**

5.9 节建议卸载多余的 OpenCV。如果需要保留多个版本，可以在工作空间内基于指定 OpenCV 重新编译 `cv_bridge`，使它和驱动使用同一版本，编译产物保存在工作空间内。

原文称驱动会读取 `cv_bridge-extras.cmake` 并自动锁定 OpenCV 版本；当前压缩包的 `CMakeLists.txt` 实际仅调用 `find_package(OpenCV REQUIRED)`，不能依赖该自动锁定行为。编译时应显式指定匹配的 `OpenCV_DIR`，并通过下述 `ldd` 检查最终加载的库。

**步骤 0：确认 ROS 发行版**

```shell
echo $ROS_DISTRO
# 示例输出：noetic（ROS 1）
# 示例输出：foxy（ROS 2）
```

下方命令以 `$ROS_DISTRO` 表示实际发行版，请先加载对应 ROS 环境，或替换成发行版名称。

**步骤 1：下载对应分支的 `vision_opencv`**

将仓库克隆到工作空间 `src/` 下，与 `odin_ros_driver` 并列：

```shell
cd <your_workspace>/src
git clone -b $ROS_DISTRO https://github.com/ros-perception/vision_opencv.git
```

如果发行版名称与分支名称不同，需替换成实际分支。原文以 Jazzy 对应 `rolling` 为例，实际选择时应确认当前仓库的分支兼容性。

**步骤 2：先编译 cv_bridge，再编译驱动**

必须先构建工作空间中的 `cv_bridge`，让驱动可以找到它。仅调整构建顺序不能保证 OpenCV 一致，还应显式指定同一套 OpenCV 配置。

ROS 1（`catkin_make`）示例：

```shell
# 先将 cv_bridge 编译到工作空间。
source /opt/ros/$ROS_DISTRO/setup.bash
cd <your_catkin_ws>
catkin_make -DBUILD_SYSTEM=ROS1 -DCATKIN_WHITELIST_PACKAGES="cv_bridge" -j$(nproc)

# 再编译驱动，使用工作空间中的 cv_bridge（此时不要清理 build/devel）。
# 请另行确保编译时指定的 OpenCV 与 cv_bridge 一致。
cd <your_catkin_ws>/src/odin_ros_driver/script
./build_ros.sh
```

完整构建也会按依赖关系先构建 `cv_bridge`，再构建驱动：

```shell
cd <your_catkin_ws> && catkin_make -DBUILD_SYSTEM=ROS1 -j$(nproc)
```

ROS 2（`colcon`）示例：

```shell
# 先将 cv_bridge 编译到工作空间。
source /opt/ros/$ROS_DISTRO/setup.bash
cd <your_ros2_ws>
colcon build --packages-select cv_bridge

# 先加载工作空间环境，使驱动找到其中的 cv_bridge，然后编译驱动。
source <your_ros2_ws>/install/setup.bash
colcon build --packages-select odin_ros_driver
```

**验证**

```shell
# ROS 1
source /opt/ros/$ROS_DISTRO/setup.bash
source <your_catkin_ws>/devel/setup.bash
ldd <your_catkin_ws>/devel/lib/odin_ros_driver/host_sdk_sample | grep -iE 'cv_bridge|opencv_core'

# ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash
source <your_ros2_ws>/install/setup.bash
ldd <your_ros2_ws>/install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample | grep -iE 'cv_bridge|opencv_core'
```

修复前，驱动可能从 `/opt/ros/$ROS_DISTRO/lib/` 加载 `libcv_bridge.so`，同时混入不匹配的 OpenCV。使用工作空间重编译方案后，应从工作空间加载 `cv_bridge`，并确保所有 `libopencv_*` 来自同一兼容版本。如果直接让驱动匹配系统 `cv_bridge`，则无需更换 `cv_bridge` 的加载路径。

运行驱动，确认 `/odin1/image` 持续发布且不再崩溃。

请在一致的 ROS 环境中运行，避免同时加载不同发行版，例如同时加载 Noetic 和 Foxy，从而使 `LD_LIBRARY_PATH` 混入不匹配的库。

## 6. 联系方式

技术支持邮箱：support@manifoldtech.cn。

为便于现场应用工程师（FAE）诊断问题，请提供：

1. 当前驱动和设备固件版本。原文给出的日志示例如下；注意其中 `ros_driver_version` 是主机驱动版本，设备固件请同时提供 `soc_version`、`slam_version` 等字段。

```shell
[device_version_capture]: ros_driver_version: [Version Number]
```

2. 使用的电源适配器和转接线照片。
3. 问题是偶发还是持续出现。
4. 问题现场的图片。
5. 第 5 节排查方法是否有效。
6. 期望的问题解决时间。
