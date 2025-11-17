#!/bin/bash

clear

# ------------------------------------------------------------------------------------------------- #
# 功能包创建，以第一次创建为例
# --build-type: 指定包的构建类型 ament_cmake C++、ament_python Python
# --node-name: 指定节点名称
# --dependencies: 指定包的依赖 rclcpp ROS2 C++、rclpy ROS2 Python
# 注：创建包时，包名不能包含特殊符号：-、

#ros2 pkg create --build-type ament_cmake class_3_cpp --dependencies rclcpp

# ------------------------------------------------------------------------------------------------- #
# 配置ROS2包依赖环境
# rosdep install -i --from-paths src --rosdistro $ROS_DISTRO -y

# ------------------------------------------------------------------------------------------------- #
# 编译ROS2包
# colcon 常用参数说明：
#   --build-base: 指定编译输出目录
#   --intsall-base: 指定安装输出目录
#   --symlink-install: 使用符号链接安装包
#   --packages-select: 指定编译的包
#   --packages-ignore: 指定忽略的包
#   --continue-on-error: 编译错误继续编译
#   --cmake-args ，--ament-cmake-args， --catkin-cmake-args: 传递给CMake的参数
#   --packages-up-to: 编译指定包及其依赖的所有包
#   --packages-up-to-select: 编译指定包及其依赖的所有包
#   --merge-install: 合并安装目录
#   --parallel-workers: 指定并行编译的线程数，默认是CPU核心数

# OpenVINO环境配置
# sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
# echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino.list
# sudo apt-get update
# sudo apt install openvino-2025.3.0

# ROS2 格式化终端打印
export RCUTILS_COLORIZED_OUTPUT=1
# 用占位符自定义输出字段和顺序，例如显示时间、级别、节点名和消息
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{name}]: {message}"


# rm -rf ./install
rm -rf ./log
colcon build --symlink-install --continue-on-error --merge-install && \
source ./install/setup.bash && \
echo "====================  ROS2包编译完成  ====================" && \
ros2 run lecture_4 lecture_4_test_node && \
ros2 run lecture_5 lecture_5_test_node && \
ros2 run lecture_6 lecture_6_test_node && \
echo "====================  ROS2包运行完成  ===================="
