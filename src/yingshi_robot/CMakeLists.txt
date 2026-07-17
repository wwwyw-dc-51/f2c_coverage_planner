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

# ── 纯规划算法库：不依赖 ROS、评估器或文件输出 ──
add_library(coverage_planner_core STATIC
  src/decomposer.cpp
  src/swath_generator.cpp
  src/boundary_filler.cpp
  src/path_planner.cpp
)
add_library(${PROJECT_NAME}::coverage_planner_core ALIAS coverage_planner_core)
target_link_libraries(coverage_planner_core PUBLIC
  Fields2Cover::Fields2Cover
)
target_include_directories(coverage_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── ROS 适配层：消息转换、参数和发布 ──
add_executable(polygon_planner_node
  src/polygon_planner_node.cpp
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
  coverage_planner_core
  ortools::ortools
)
target_include_directories(polygon_planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(path_geometry_test
    test/path_geometry_test.cpp
  )
  target_link_libraries(path_geometry_test
    Fields2Cover::Fields2Cover
  )
  target_include_directories(path_geometry_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  ament_add_gtest(decomposer_test
    test/decomposer_test.cpp
  )
  target_link_libraries(decomposer_test
    coverage_planner_core
  )
  target_include_directories(decomposer_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(planning_invariants_test
    test/planning_invariants_test.cpp
  )
  target_link_libraries(planning_invariants_test
    coverage_planner_core
  )
  target_include_directories(planning_invariants_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(coverage_evaluator_test
    test/coverage_evaluator_test.cpp
  )
  target_link_libraries(coverage_evaluator_test
    Fields2Cover::Fields2Cover
  )
  target_include_directories(coverage_evaluator_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
endif()

install(TARGETS
  coverage_planner_core
  EXPORT export_coverage_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# 保持 ROS 可执行文件的标准安装位置。
install(TARGETS
  polygon_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_coverage_planner_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Fields2Cover)

ament_package()
