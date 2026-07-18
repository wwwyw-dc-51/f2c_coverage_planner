#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <fields2cover.h>

namespace yingshi {

struct TraversabilityParams {
    double robot_width = 0.75;
    double clearance_margin = 0.0;
    double max_excluded_area_ratio = 0.05;
    double area_tolerance = 1e-6;
    double ratio_tolerance = 1e-9;
};

struct TraversabilityResult {
    // 机器人中心可以到达的 C-space；每个 Cell 是一个独立子任务。
    f2c::types::Cells center_space;
    // 与 center_space 一一对应，保留断开子任务，禁止 union 后丢失拓扑信息。
    std::vector<f2c::types::Cells> coverage_components;
    // C-space 按机器人半宽恢复后，与原始自由空间相交得到的覆盖目标。
    f2c::types::Cells reachable_coverage;
    // 原始自由空间中无法由机器人足迹触达的区域。
    f2c::types::Cells excluded_regions;

    double original_area = 0.0;
    double reachable_area = 0.0;
    double excluded_area = 0.0;
    double excluded_area_ratio = 0.0;
    std::size_t component_count = 0;

    bool has_excluded_regions = false;
    bool requires_repositioning = false;
    bool exclusion_limit_exceeded = false;
    bool analysis_valid = false;
    std::string error_message;
};

// 用圆形保守足迹（半径 = robot_width / 2 + clearance_margin）分析可通行性。
// 不修改输入场景，也不把排除面积伪装成已覆盖；调用方必须单独报告结果。
TraversabilityResult analyzeTraversability(
    const f2c::types::Cell& free_space,
    const TraversabilityParams& params);

}  // namespace yingshi
