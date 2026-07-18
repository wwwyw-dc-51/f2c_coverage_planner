#pragma once

#include <string>
#include <vector>

#include "yingshi_robot/planner_params.hpp"

namespace yingshi {

// 节点运行时参数。它们不直接属于某个规划模块，但同样必须在生效前校验。
struct PlannerRuntimeParams {
    double decomposition_angle = 0.0;
    double mid_hl_width_ratio = 0.2;
    double no_hl_width_ratio = 0.0;
    double min_hole_area = 1.0;
    double eval_grid_resolution = 0.1;
    double eval_coverage_threshold = 0.99;
};

// 完整规划配置的聚合边界，集中校验后再交给规划器。
struct PlannerConfig {
    DecomposerParams decomposer;
    SwathParams swath;
    FillParams fill;
    PathParams path;
    PlannerRuntimeParams runtime;
};

enum class PlannerConfigErrorCode {
    kNonFinite,
    kOutOfRange,
    kUnsupportedValue,
    kInconsistentValue,
};

struct PlannerConfigIssue {
    PlannerConfigErrorCode code;
    std::string field;
    std::string message;
};

// 返回全部配置问题；空列表表示配置可安全交给规划器。
// 函数只诊断、不修改输入，避免参数服务器与实际运行值不一致。
std::vector<PlannerConfigIssue> validatePlannerConfig(
    const PlannerConfig& config);

}  // namespace yingshi
