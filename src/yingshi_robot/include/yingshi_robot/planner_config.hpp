#pragma once

#include <string>
#include <vector>

#include "yingshi_robot/planner_params.hpp"

namespace yingshi {

// 完整规划配置的聚合边界。暂时包裹现有参数结构，便于渐进迁移节点。
struct PlannerConfig {
    DecomposerParams decomposer;
    SwathParams swath;
    FillParams fill;
    PathParams path;
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
// 函数只诊断，不修改输入，避免参数服务器值与实际运行值不一致。
std::vector<PlannerConfigIssue> validatePlannerConfig(
    const PlannerConfig& config);

}  // namespace yingshi
