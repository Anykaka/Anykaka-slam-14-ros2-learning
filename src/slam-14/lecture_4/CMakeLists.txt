cmake_minimum_required(VERSION 3.8)
project(lecture_4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(PACKAGE_NAME=\"${PROJECT_NAME}\")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/include/
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_test_node src/main.cc)
target_include_directories(${PROJECT_NAME}_test_node PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include/
)
ament_target_dependencies(${PROJECT_NAME}_test_node rclcpp interface)
target_link_libraries(${PROJECT_NAME}_test_node Sophus::Sophus)

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

install(TARGETS
        ${PROJECT_NAME}_test_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME})

ament_package()
