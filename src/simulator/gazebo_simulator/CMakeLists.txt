cmake_minimum_required(VERSION 3.8)
project(gazebo_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# basic build deps for our C++ node
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  urdf
  rviz
  launch
  world
  config
  DESTINATION share/${PROJECT_NAME})

# Build pointcloud transformer executable
add_executable(pointcloud_transformer src/pointcloud_transformer.cpp)
ament_target_dependencies(pointcloud_transformer
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  laser_geometry
  tf2_geometry_msgs
)

if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(pointcloud_transformer yaml-cpp::yaml-cpp)
else()
  # Some systems/installations provide yaml-cpp without the namespaced target.
  # Try linking the plain library name as a fallback.
  target_link_libraries(pointcloud_transformer yaml-cpp)
endif()

install(TARGETS pointcloud_transformer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# scout patrol node: sets model pose in gazebo, patrols along waypoints, transforms and forwards pointclouds
add_executable(scout_patrol_node src/scout_patrol_node.cpp)
ament_target_dependencies(scout_patrol_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  gazebo_msgs
)

if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(scout_patrol_node yaml-cpp::yaml-cpp)
else()
  target_link_libraries(scout_patrol_node yaml-cpp)
endif()

install(TARGETS scout_patrol_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
