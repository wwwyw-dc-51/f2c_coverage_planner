cmake_minimum_required(VERSION 3.8)
project(yingshi_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
# 优先使用调用方显式提供的安装前缀；仓库内目录只作为开发环境兼容回退。
set(YINGSHI_FIELDS2COVER_PREFIX "" CACHE PATH
  "Fields2Cover installation prefix")
if(YINGSHI_FIELDS2COVER_PREFIX)
  list(PREPEND CMAKE_PREFIX_PATH "${YINGSHI_FIELDS2COVER_PREFIX}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../install/fields2cover")
  list(PREPEND CMAKE_PREFIX_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../../install/fields2cover")
endif()
find_package(Fields2Cover REQUIRED)

# OR-Tools 同样允许使用系统安装或交叉编译 sysroot 中的包配置。
set(YINGSHI_ORTOOLS_PREFIX "" CACHE PATH
  "OR-Tools installation prefix")
if(YINGSHI_ORTOOLS_PREFIX)
  list(PREPEND CMAKE_PREFIX_PATH "${YINGSHI_ORTOOLS_PREFIX}")
elseif(EXISTS
    "${CMAKE_CURRENT_SOURCE_DIR}/../Fields2Cover/third_party/ortools-src")
  list(PREPEND CMAKE_PREFIX_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../Fields2Cover/third_party/ortools-src")
endif()
find_package(ortools CONFIG REQUIRED)

# ── 纯规划算法库：不依赖 ROS、评估器或文件输出 ──
add_library(coverage_planner_core STATIC
  src/planner_config.cpp
  src/decomposer.cpp
  src/swath_generator.cpp
  src/boundary_filler.cpp
  src/geometry_normalizer.cpp
  src/path_planner.cpp
  src/physical_footprint.cpp
  src/traversability.cpp
  src/planner_core.cpp
)
add_library(${PROJECT_NAME}::coverage_planner_core ALIAS coverage_planner_core)
target_link_libraries(coverage_planner_core PUBLIC
  Fields2Cover::Fields2Cover
)
target_include_directories(coverage_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── 离线测评接口：评分/网格报告不得成为规划核心的依赖 ──
# 实车部署时设置 -DYINGSHI_ENABLE_EVALUATION=OFF 即可从生产二进制中完全移除。
option(YINGSHI_ENABLE_EVALUATION
  "Build offline coverage evaluation (scoring, grid, JSON reports)" ON)

if(YINGSHI_ENABLE_EVALUATION)
  add_library(coverage_planner_evaluation INTERFACE)
  add_library(${PROJECT_NAME}::coverage_planner_evaluation ALIAS
    coverage_planner_evaluation)
  target_link_libraries(coverage_planner_evaluation INTERFACE
    Fields2Cover::Fields2Cover
  )
  target_include_directories(coverage_planner_evaluation INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  )
endif()

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
if(YINGSHI_ENABLE_EVALUATION)
  target_link_libraries(polygon_planner_node
    coverage_planner_evaluation
  )
  target_compile_definitions(polygon_planner_node PRIVATE
    YINGSHI_EVAL_ENABLED=1
  )
endif()
target_include_directories(polygon_planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  if(YINGSHI_ENABLE_EVALUATION)
    ament_add_gtest(path_geometry_test
      test/path_geometry_test.cpp
    )
    target_link_libraries(path_geometry_test
      coverage_planner_evaluation
    )
  endif()

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

  ament_add_gtest(planner_core_test
    test/planner_core_test.cpp
  )
  target_link_libraries(planner_core_test
    coverage_planner_core
  )
  target_include_directories(planner_core_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  if(YINGSHI_ENABLE_EVALUATION)
    ament_add_gtest(coverage_evaluator_test
      test/coverage_evaluator_test.cpp
    )
    target_link_libraries(coverage_evaluator_test
      coverage_planner_evaluation
    )
  endif()

  ament_add_gtest(planner_config_test
    test/planner_config_test.cpp
  )
  target_link_libraries(planner_config_test
    coverage_planner_core
  )

  ament_add_gtest(traversability_test
    test/traversability_test.cpp
  )
  target_link_libraries(traversability_test
    coverage_planner_core
  )

  ament_add_gtest(physical_footprint_test
    test/physical_footprint_test.cpp
  )
  target_link_libraries(physical_footprint_test
    coverage_planner_core
  )

  ament_add_gtest(artifact_sink_test
    test/artifact_sink_test.cpp
  )
  target_include_directories(artifact_sink_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
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
