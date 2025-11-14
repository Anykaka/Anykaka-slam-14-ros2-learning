cmake_minimum_required(VERSION 3.8)
project(interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  /usr/include
  ${CMAKE_CURRENT_LIST_DIR}/include/
  ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}
)

FILE(GLOB_RECURSE msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
# FILE(GLOB_RECURSE srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")
# FILE(GLOB_RECURSE action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "action/*.action")

# FILE(GLOB_RECURSE proto_srcs
#   "${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/proto/*.cc"
#   "${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/proto/*.cpp"
# )

# 生成消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
#   ${srv_files}
#   ${action_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_library(${PROJECT_NAME}_lib SHARED src/main.cc) # ${proto_srcs}
target_link_libraries(${PROJECT_NAME}_lib ${DEPEND_PROJECT_LIBRARIES_LIST}
  # /lib/x86_64-linux-gnu/libprotobuf.so.23
)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp)

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

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_lib_export
)

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME})

# install(DIRECTORY
#   include/${PROJECT_NAME}/proto
#   DESTINATION include/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}_lib_export)

ament_package()
