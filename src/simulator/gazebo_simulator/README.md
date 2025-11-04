# Gazebo Simulator

## 简介

`gazebo_simulator` 是一个基于 Gazebo 的 ROS2 仿真包，用于模拟机器人在虚拟环境中的运行。该包提供了完整的仿真环境配置、机器人模型、传感器模拟以及点云数据处理功能。

主要功能：
- **Gazebo 仿真环境**：提供多个预配置的仿真世界和机器人模型
- **点云坐标变换**：将传感器点云数据转换到指定坐标系
- **机器人巡逻控制**：控制机器人沿预设路径巡逻，并处理传感器数据
- **多传感器支持**：支持激光雷达、点云等多种传感器类型

## 包结构

```
gazebo_simulator/
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # ROS2 包描述文件
├── README.md                   # 本文档
├── src/                        # 源代码目录
│   ├── pointcloud_transformer.cpp      # 点云坐标变换节点
│   └── scout_patrol_node.cpp           # 侦察兵巡逻节点
├── launch/                     # 启动文件目录
│   ├── entrance_simulator.launch.py    # 入口场景仿真启动文件
│   ├── pointcloud_transformer.launch.py # 点云变换启动文件
│   └── scout_patrol_launch.py          # 巡逻节点启动文件
├── config/                     # 配置文件目录
│   ├── entrance_pointcloud_transformer.yaml  # 入口场景点云变换配置
│   ├── scout_params.yaml                     # 侦察兵参数配置
│   └── topic_transform.yaml                  # 话题变换配置
├── urdf/                       # 机器人模型描述文件
│   ├── entrance/               # 入口场景机器人模型
│   └── scout_robot/            # 侦察兵机器人模型
├── world/                      # Gazebo 世界文件
│   ├── entrance/               # 入口场景世界文件
│   └── scout_robot/            # 侦察兵场景世界文件
└── rviz/                       # RViz 配置文件
```

## 核心功能

### 1. 点云坐标变换节点 (pointcloud_transformer)

该节点负责将传感器发布的点云数据从传感器坐标系转换到目标坐标系（如 `base_link`）。

**功能特点**：
- 支持 PointCloud2 和 LaserScan 两种消息类型
- 支持静态坐标变换和 TF 树变换
- 动态订阅和发布话题管理
- 可配置多个传感器的同时变换

**参数配置**：
```yaml
pointcloud_transformer:
  ros__parameters:
    - original_topic: "/sensor/points"      # 原始点云话题
      transformed_topic: "/sensor/points_tf" # 变换后话题
      frame_id: "base_link"                  # 目标坐标系
      tf: [x, y, z, qx, qy, qz, qw]          # 静态变换（可选）
```

### 2. 侦察兵巡逻节点 (scout_patrol_node)

该节点控制机器人在 Gazebo 中沿预定义路径巡逻，同时处理传感器数据并发布变换后的点云。

**功能特点**：
- 通过 Gazebo 服务控制机器人位姿
- 支持循环或单次巡逻模式
- 实时采集和转换传感器点云数据
- 可选点云数据保存功能（PCD 格式）
- 发布机器人当前位姿信息

**参数配置**：
```yaml
pointcloud_transformer:
  ros__parameters:
    robot_name: "scout_bot"                 # 机器人名称
    patrol_speed: 0.5                       # 巡逻速度 (m/s)
    pose_publish_rate: 10.0                 # 位姿发布频率 (Hz)
    save_pointclouds: false                 # 是否保存点云
    save_directory: "/path/to/save"         # 点云保存路径
    
    sensors:                                # 传感器配置
      - name: "sensor_name"
        original_topic: "/sensor/scan"
        transformed_topic: "/sensor/scan_tf"
        frame_id: "base_link"
    
    start_pose: [x, y, z, qx, qy, qz, qw]   # 起始位姿
    
    patrol_route:                           # 巡逻路径
      is_loop: true                         # 是否循环
      points:
        - [x, y, z, qx, qy, qz, qw]         # 路径点1
        - [x, y, z, qx, qy, qz, qw]         # 路径点2
```

## 使用方法

### 1. 编译包

```bash
cd ~/your_ros2_workspace
colcon build --packages-select gazebo_simulator
source install/setup.bash
```

### 2. 启动入口场景仿真

启动完整的入口场景仿真环境（包括 Gazebo、机器人模型、RViz）：

```bash
ros2 launch gazebo_simulator entrance_simulator.launch.py
```

### 3. 启动点云变换节点

单独启动点云坐标变换节点：

```bash
ros2 launch gazebo_simulator pointcloud_transformer.launch.py
```

或使用自定义配置文件：

```bash
ros2 run gazebo_simulator pointcloud_transformer --ros-args --params-file config/topic_transform.yaml
```

### 4. 启动侦察兵巡逻节点

启动机器人巡逻节点：

```bash
ros2 launch gazebo_simulator scout_patrol_launch.py
```

或使用自定义配置：

```bash
ros2 run gazebo_simulator scout_patrol_node --ros-args --params-file config/scout_params.yaml
```

## 配置文件说明

### topic_transform.yaml

配置多个传感器的点云坐标变换参数：

```yaml
pointcloud_transformer:
  ros__parameters:
    - original_topic: "/robosense_airy_lf_points/scan"
      transformed_topic: "/robosense_airy_lf_points/scan_tf"
      frame_id: "base_link"
      tf: [-3.0, -2.1, 0.01, 0, 0, 0, 1]  # [x, y, z, qx, qy, qz, qw]
```

**参数说明**：
- `original_topic`: 原始传感器话题
- `transformed_topic`: 变换后发布的话题
- `frame_id`: 目标坐标系 ID
- `tf`: 从传感器坐标系到目标坐标系的静态变换（7个值：平移x,y,z + 四元数qx,qy,qz,qw）

### scout_params.yaml

配置侦察兵机器人的巡逻参数：

```yaml
pointcloud_transformer:
  ros__parameters:
    robot_name: "scout_bot"
    patrol_speed: 0.5                    # 巡逻速度
    pose_publish_rate: 10.0              # 位姿发布频率
    save_pointclouds: false              # 是否保存点云
    save_directory: "/home/user/pcd_out" # 保存路径
    
    sensors:                             # 传感器列表
      - name: "robosense_airy_scout"
        original_topic: "/robosense_airy_center_points/scan"
        transformed_topic: "/robosense_airy_center_points/scan_tf"
        frame_id: "base_link"
    
    start_pose: [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]
    
    patrol_route:
      is_loop: true                      # 循环巡逻
      points:
        - [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]
        - [0.0, 15.0, 0.1, 0.0, 0.0, 0.0, 1.0]
        - [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]
        - [0.0, -25.0, 0.1, 0.0, 0.0, 0.0, 1.0]
```

## 话题接口

### 订阅的话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/sensor/points` (可配置) | `sensor_msgs/PointCloud2` | 原始点云数据 |
| `/sensor/scan` (可配置) | `sensor_msgs/LaserScan` | 原始激光扫描数据 |

### 发布的话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/sensor/points_tf` (可配置) | `sensor_msgs/PointCloud2` | 变换后的点云数据 |
| `/scout_pose` | `geometry_msgs/Pose` | 机器人当前位姿 |

### 服务客户端

| 服务名 | 服务类型 | 说明 |
|--------|----------|------|
| `/gazebo/get_entity_state` | `gazebo_msgs/GetEntityState` | 获取实体状态 |
| `/gazebo/set_entity_state` | `gazebo_msgs/SetEntityState` | 设置实体状态 |
| `/gazebo/get_model_list` | `gazebo_msgs/GetModelList` | 获取模型列表 |

## 依赖项

- **rclcpp**: ROS2 C++ 客户端库
- **sensor_msgs**: 传感器消息定义
- **geometry_msgs**: 几何消息定义
- **tf2**: 坐标变换库
- **tf2_ros**: TF2 ROS 集成
- **tf2_sensor_msgs**: 传感器消息的 TF2 支持
- **gazebo_msgs**: Gazebo 消息和服务定义
- **laser_geometry**: 激光扫描转点云工具
- **yaml-cpp**: YAML 配置文件解析
- **gazebo_ros**: Gazebo ROS2 集成

## 开发说明

### 添加新的机器人模型

1. 在 `urdf/` 目录下创建新的机器人描述文件（URDF/XACRO）
2. 在 `world/` 目录下创建对应的 Gazebo 世界文件
3. 创建相应的启动文件在 `launch/` 目录

### 添加新的传感器

1. 在 URDF 文件中定义传感器插件
2. 在配置文件中添加传感器的坐标变换参数
3. 更新启动文件以加载新的配置

### 修改巡逻路径

编辑 `config/scout_params.yaml` 文件中的 `patrol_route` 部分，添加或修改路径点。

## 常见问题

### 1. Gazebo 无法启动

**问题**：启动 launch 文件后 Gazebo 无法打开。

**解决方案**：
- 确保已安装 `gazebo_ros_pkgs`
- 检查 Gazebo 环境变量是否正确配置
- 尝试单独启动 Gazebo：`gazebo --verbose`

### 2. 点云数据无法转换

**问题**：变换后的点云话题没有数据输出。

**解决方案**：
- 检查原始话题是否有数据发布：`ros2 topic echo /original_topic`
- 验证配置文件中的话题名称是否正确
- 确认 TF 树是否完整：`ros2 run tf2_tools view_frames`

### 3. 机器人不移动

**问题**：巡逻节点运行但机器人不移动。

**解决方案**：
- 确认 Gazebo 服务是否可用：`ros2 service list | grep gazebo`
- 检查机器人名称是否与配置文件中的 `robot_name` 匹配
- 查看节点日志：`ros2 run gazebo_simulator scout_patrol_node --ros-args --log-level debug`

### 4. 点云保存失败

**问题**：启用 `save_pointclouds` 后无法保存点云文件。

**解决方案**：
- 确认保存目录存在且有写权限
- 检查磁盘空间是否充足
- 验证点云数据是否有效

## 维护者

- **维护者**: anykaka
- **邮箱**: xxx@163.com

## 许可证

TODO: 添加许可证声明

## 参考资料

- [Gazebo 官方文档](http://gazebosim.org/)
- [ROS2 TF2 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [sensor_msgs 文档](https://docs.ros2.org/latest/api/sensor_msgs/index.html)
- [Gazebo ROS2 集成](https://github.com/gazebosim/gazebo_ros_pkgs)

## 更新日志

- **v0.0.0**: 初始版本
  - 实现点云坐标变换功能
  - 实现机器人巡逻控制
  - 支持多传感器配置
  - 集成 Gazebo 仿真环境
