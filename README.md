# SLAM-14 学习实践工程

## 简介

本项目是基于《视觉SLAM十四讲》的学习实践工程，使用 ROS2 框架搭建，旨在通过实际代码实现来深入理解和掌握 SLAM（Simultaneous Localization and Mapping，同时定位与建图）的核心算法和技术。

## 工程结构

```
SLAM-14/
├── src/                        # 源代码目录
│   └── simulator/              # 仿真相关包
│       └── gazebo_simulator/   # Gazebo 仿真环境
├── interface/                  # 公共接口包
│   ├── include/                # 公共头文件
│   └── src/                    # 自定义消息/服务/动作
├── doc/                        # 文档目录
├── build/                      # 编译输出目录
├── install/                    # 安装目录
└── log/                        # 日志目录
```

## 功能模块

### 1. Gazebo 仿真器 (gazebo_simulator)

提供基于 Gazebo 的机器人仿真环境，包括：
- 多种仿真场景和机器人模型
- 点云坐标变换功能
- 机器人巡逻控制
- 多传感器数据采集

详见：[gazebo_simulator/README.md](src/simulator/gazebo_simulator/README.md)

### 2. 接口包 (interface)

集中管理项目的公共资源：
- 通用 C++ 头文件和工具类
- ROS2 自定义消息类型（msg）
- ROS2 自定义服务类型（srv）
- ROS2 自定义动作类型（action）

详见：[interface/README.md](interface/README.md)

## 环境要求

- **操作系统**: Ubuntu 22.04 或更高版本
- **ROS2 版本**: Humble Hawksbill 或更高版本
- **Gazebo**: Gazebo 11 或更高版本
- **编译器**: GCC 9.0 或更高版本，支持 C++17

## 依赖安装

### 安装 ROS2

```bash
# 根据官方教程安装 ROS2 Humble
# https://docs.ros.org/en/humble/Installation.html
```

### 安装 Gazebo 和相关包

```bash
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    libyaml-cpp-dev
```

## 快速开始

### 1. 克隆并编译工程

```bash
# 进入工作空间
cd ~/Develop/ROS2/SLAM-14

# 编译所有包
colcon build

# 加载环境变量
source install/setup.bash
```

### 2. 运行仿真环境

```bash
# 启动 Gazebo 仿真环境
ros2 launch gazebo_simulator entrance_simulator.launch.py
```

### 3. 运行点云变换节点

```bash
# 在新终端中运行点云坐标变换
source install/setup.bash
ros2 launch gazebo_simulator pointcloud_transformer.launch.py
```

### 4. 运行机器人巡逻

```bash
# 在新终端中启动机器人巡逻节点
source install/setup.bash
ros2 launch gazebo_simulator scout_patrol_launch.py
```

## 学习路线

本项目按照《视觉SLAM十四讲》的章节顺序组织，建议按以下顺序学习：

1. **第一讲：预备知识** - 了解 SLAM 基本概念
2. **第二讲：初识 SLAM** - 熟悉 ROS2 和 Gazebo 仿真环境
3. **第三讲：三维空间刚体运动** - 理解坐标变换和 TF2
4. **第四讲：李群与李代数** - 学习旋转表示方法
5. **第五讲：相机与图像** - 处理相机模型和图像数据
6. **第六讲：非线性优化** - 实现优化算法
7. **第七讲：视觉里程计** - 实现前端视觉里程计
8. **第八讲：后端优化** - 实现后端优化
9. **第九讲：回环检测** - 实现闭环检测
10. **第十讲：建图** - 实现地图构建
11. **第十一讲：SLAM 系统** - 集成完整系统
12. **第十二讲：实践** - 实际应用和调优
13. **第十三讲：深度学习与 SLAM** - 探索深度学习方法
14. **第十四讲：未来展望** - 了解 SLAM 发展趋势

## 常用命令

### 编译相关

```bash
# 编译所有包
colcon build

# 编译指定包
colcon build --packages-select gazebo_simulator

# 清理编译文件
rm -rf build/ install/ log/
```

### 运行和调试

```bash
# 查看话题列表
ros2 topic list

# 查看话题数据
ros2 topic echo /topic_name

# 查看节点列表
ros2 node list

# 查看节点信息
ros2 node info /node_name

# 查看 TF 树
ros2 run tf2_tools view_frames

# 启动 RViz 可视化
rviz2
```

### 日志查看

```bash
# 查看节点日志
ros2 run gazebo_simulator scout_patrol_node --ros-args --log-level debug

# 查看编译日志
cat log/latest_build/gazebo_simulator/stdout_stderr.log
```

## 项目进度

- [x] 仿真环境搭建
- [x] 点云数据采集和处理
- [x] 机器人运动控制
- [ ] 视觉里程计实现
- [ ] 后端优化实现
- [ ] 回环检测实现
- [ ] 地图构建实现
- [ ] 完整 SLAM 系统集成

## 参考资料

- **《视觉SLAM十四讲》** - 高翔 著
- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [Gazebo 官方文档](http://gazebosim.org/)
- [SLAM 十四讲 GitHub](https://github.com/gaoxiang12/slambook2)

## 开发者

- **作者**: zx
- **邮箱**: lz2297519360@outlook.com

## 许可证

TODO: 添加许可证声明

## 贡献指南

欢迎提交 Issue 和 Pull Request！

## 更新日志

### 2025-11-04
- 初始化项目结构
- 完成 Gazebo 仿真环境搭建
- 实现点云坐标变换功能
- 实现机器人巡逻控制功能
