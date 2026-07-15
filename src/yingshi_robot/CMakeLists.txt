cmake_minimum_required(VERSION 3.8)
project(yingshi_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
# Fields2Cover: 预编译在 install/fields2cover/ 下
list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../install/fields2cover")
find_package(Fields2Cover REQUIRED)

# 查找 Fields2Cover 自带的 OR-Tools
# CMAKE_CURRENT_SOURCE_DIR = .../src/yingshi_robot → .. = .../src → Fields2Cover 的父目录
list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Fields2Cover/third_party/ortools-src")
find_package(ortools CONFIG REQUIRED)

# ── 模块化重构：主节点 + 5 个独立模块 ──
add_executable(polygon_planner_node
  src/polygon_planner_node.cpp
  src/decomposer.cpp
  src/swath_generator.cpp
  src/boundary_filler.cpp
  src/path_planner.cpp
)
ament_target_dependencies(polygon_planner_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  nav_msgs
  std_srvs
  visualization_msgs
)
target_link_libraries(polygon_planner_node
  Fields2Cover::Fields2Cover
  ortools::ortools
)
target_include_directories(polygon_planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS
  polygon_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
