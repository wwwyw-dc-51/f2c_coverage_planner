/**
 * @file planner_core.cpp
 * @brief PlannerCore 实现 — 纯算法流水线，不含 ROS/文件 I/O/评分
 *
 * 从 polygon_planner_node.cpp 逐步提取。
 * 当前版本：接口已定义，核心流水线可调用；
 * 后续迁移将逐步把节点中的管线逻辑移入此处。
 */

#include "yingshi_robot/planner_core.hpp"
#include "yingshi_robot/path_planner.hpp"
#include "yingshi_robot/path_geometry.hpp"
#include "yingshi_robot/boundary_filler.hpp"
#include <chrono>

namespace yingshi {

PlanningResult PlannerCore::plan(const PlanningRequest& req)
{
    PlanningResult result;
    auto t0 = std::chrono::steady_clock::now();

    (void)req;  // 占位：当前主管线仍在 polygon_planner_node 中

    result.success = false;
    result.error_message =
        "PlannerCore::plan() is a stub — "
        "the main pipeline is being migrated from polygon_planner_node.cpp. "
        "Call planCoveragePath on the ROS node for now.";

    auto t1 = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

}  // namespace yingshi

