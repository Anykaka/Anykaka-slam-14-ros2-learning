# Interface

## 简介

`interface` 是一个 ROS2 接口包，用于集中管理和存放项目中的公共资源，包括：

- **公共头文件**：通用的 C++ 头文件和工具类
- **自定义消息（msg）**：ROS2 自定义消息类型定义
- **自定义服务（srv）**：ROS2 自定义服务类型定义
- **自定义动作（action）**：ROS2 自定义动作类型定义

## 包结构

```
interface/
├── CMakeLists.txt          # CMake 构建配置文件
├── package.xml             # ROS2 包描述文件
├── README.md               # 本文档
├── include/                # 公共头文件目录
│   └── interface/
│       └── common/         # 通用头文件
└── src/                    # 消息定义目录
    ├── msg/                # 自定义消息定义
    ├── srv/                # 自定义服务定义
    └── action/             # 自定义动作定义
```

## 使用方法

### 1. 添加自定义消息（msg）

在 `src/msg/` 目录下创建 `.msg` 文件，例如 `CustomMessage.msg`：

```msg
# 示例消息定义
std_msgs/Header header
float64 x
float64 y
float64 theta
```

### 2. 添加自定义服务（srv）

在 `src/srv/` 目录下创建 `.srv` 文件，例如 `CustomService.srv`：

```srv
# 请求部分
string command
---
# 响应部分
bool success
string message
```

### 3. 添加自定义动作（action）

在 `src/action/` 目录下创建 `.action` 文件，例如 `CustomAction.action`：

```action
# 目标
float64 target_position
---
# 结果
bool success
float64 final_position
---
# 反馈
float64 current_position
float64 progress
```

### 4. 添加公共头文件

在 `include/interface/common/` 目录下创建 `.h` 或 `.hpp` 文件，例如：

```cpp
// include/interface/common/utils.hpp
#ifndef INTERFACE__COMMON__UTILS_HPP_
#define INTERFACE__COMMON__UTILS_HPP_

namespace interface {
namespace common {

// 在这里添加通用工具函数或类

}  // namespace common
}  // namespace interface

#endif  // INTERFACE__COMMON__UTILS_HPP_
```

## 构建配置

### 启用消息生成

如需生成自定义消息、服务或动作，请在 `CMakeLists.txt` 中取消以下代码的注释：

```cmake
FILE(GLOB_RECURSE msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/msg/*.msg")
FILE(GLOB_RECURSE srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/srv/*.srv")
FILE(GLOB_RECURSE action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)
```

### 添加依赖

如果自定义消息依赖其他消息类型，需要在 `package.xml` 中添加相应的依赖：

```xml
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 在其他包中使用

### 使用自定义消息

在其他 ROS2 包中使用此包定义的消息：

1. 在 `package.xml` 中添加依赖：
```xml
<depend>interface</depend>
```

2. 在 `CMakeLists.txt` 中添加：
```cmake
find_package(interface REQUIRED)
ament_target_dependencies(your_node interface)
```

3. 在代码中包含消息头文件：
```cpp
#include "interface/msg/custom_message.hpp"
```

### 使用公共头文件

在其他包中使用公共头文件：

1. 在 `package.xml` 中添加依赖：
```xml
<depend>interface</depend>
```

2. 在 `CMakeLists.txt` 中添加：
```cmake
find_package(interface REQUIRED)
include_directories(${interface_INCLUDE_DIRS})
```

3. 在代码中包含头文件：
```cpp
#include "interface/common/utils.hpp"
```

## 编译

在工作空间根目录下执行：

```bash
colcon build --packages-select interface
source install/setup.bash
```

## 维护者

- **维护者**: zx
- **邮箱**: lz2297519360@outlook.com

## 许可证

TODO: 添加许可证信息

## 注意事项

1. 消息、服务和动作的命名应遵循 ROS2 命名规范（使用 PascalCase）
2. 公共头文件应添加适当的头文件保护宏
3. 修改消息定义后需要重新编译包和所有依赖此包的包
4. 建议为每个消息、服务和动作添加详细的注释说明

## 相关链接

- [ROS2 接口定义文档](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
- [自定义消息创建教程](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
