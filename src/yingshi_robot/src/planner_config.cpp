#include "yingshi_robot/planner_config.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>

namespace yingshi {
namespace {

constexpr double kConsistencyTolerance = 1e-9;

bool contains(
    const std::string& value,
    std::initializer_list<const char*> allowed)
{
    return std::any_of(
        allowed.begin(), allowed.end(),
        [&value](const char* candidate) { return value == candidate; });
}

void addIssue(
    std::vector<PlannerConfigIssue>& issues,
    PlannerConfigErrorCode code,
    const std::string& field,
    const std::string& message)
{
    issues.push_back({code, field, message});
}

void requireFinite(
    std::vector<PlannerConfigIssue>& issues,
    const std::string& field,
    double value)
{
    if (!std::isfinite(value)) {
        addIssue(
            issues, PlannerConfigErrorCode::kNonFinite, field,
            "参数必须是有限数");
    }
}

void requireRange(
    std::vector<PlannerConfigIssue>& issues,
    const std::string& field,
    double value,
    double minimum,
    double maximum)
{
    if (!std::isfinite(value)) {
        addIssue(
            issues, PlannerConfigErrorCode::kNonFinite, field,
            "参数必须是有限数");
        return;
    }
    if (value < minimum || value > maximum) {
        addIssue(
            issues, PlannerConfigErrorCode::kOutOfRange, field,
            "参数超出允许范围");
    }
}

void requirePositive(
    std::vector<PlannerConfigIssue>& issues,
    const std::string& field,
    double value)
{
    if (!std::isfinite(value)) {
        addIssue(
            issues, PlannerConfigErrorCode::kNonFinite, field,
            "参数必须是有限数");
        return;
    }
    if (value <= 0.0) {
        addIssue(
            issues, PlannerConfigErrorCode::kOutOfRange, field,
            "参数必须大于零");
    }
}

bool nearlyEqual(double lhs, double rhs)
{
    return std::isfinite(lhs) && std::isfinite(rhs) &&
           std::abs(lhs - rhs) <= kConsistencyTolerance;
}

}  // namespace

std::vector<PlannerConfigIssue> validatePlannerConfig(
    const PlannerConfig& config)
{
    std::vector<PlannerConfigIssue> issues;

    requireRange(
        issues, "decomposer.merge_angle_threshold",
        config.decomposer.merge_angle_threshold, 0.0, 90.0);
    requireRange(
        issues, "decomposer.min_cell_area_ratio",
        config.decomposer.min_cell_area_ratio, 0.0,
        std::numeric_limits<double>::max());

    requirePositive(
        issues, "swath.coverage_width", config.swath.coverage_width);
    requireRange(
        issues, "swath.swath_overlap_ratio",
        config.swath.swath_overlap_ratio, 0.0, 0.5);
    requireRange(
        issues, "swath.min_swath_length",
        config.swath.min_swath_length, 0.0,
        std::numeric_limits<double>::max());
    requireFinite(
        issues, "swath.endpoint_shrink", config.swath.endpoint_shrink);

    requirePositive(
        issues, "fill.coverage_width", config.fill.coverage_width);
    requireFinite(
        issues, "fill.boundary_margin", config.fill.boundary_margin);
    requireFinite(
        issues, "fill.open_default_margin", config.fill.open_default_margin);

    requirePositive(
        issues, "path.robot_width", config.path.robot_width);
    requirePositive(
        issues, "path.coverage_width", config.path.coverage_width);
    requirePositive(
        issues, "path.path_resolution", config.path.path_resolution);
    requirePositive(
        issues, "path.max_diff_curv", config.path.max_diff_curv);
    requireRange(
        issues, "path.min_turning_radius",
        config.path.min_turning_radius, 0.0,
        std::numeric_limits<double>::max());
    requireRange(
        issues, "path.swath_overlap_ratio",
        config.path.swath_overlap_ratio, 0.0, 0.5);
    requireRange(
        issues, "path.simplify_tolerance",
        config.path.simplify_tolerance, 0.0,
        std::numeric_limits<double>::max());
    requireRange(
        issues, "path.simplify_turn_threshold",
        config.path.simplify_turn_threshold, 0.0,
        std::numeric_limits<double>::max());
    requireFinite(
        issues, "path.endpoint_shrink", config.path.endpoint_shrink);
    requireFinite(
        issues, "path.boundary_margin", config.path.boundary_margin);

    if (!contains(config.fill.boundary_type, {"closed", "open", "custom"})) {
        addIssue(
            issues, PlannerConfigErrorCode::kUnsupportedValue,
            "fill.boundary_type", "未知边界策略");
    }
    if (!contains(
            config.path.boundary_type, {"closed", "open", "custom"})) {
        addIssue(
            issues, PlannerConfigErrorCode::kUnsupportedValue,
            "path.boundary_type", "未知边界策略");
    }
    if (!contains(
            config.path.swath_order_type,
            {"boustrophedon", "snake", "spiral", "none"})) {
        addIssue(
            issues, PlannerConfigErrorCode::kUnsupportedValue,
            "path.swath_order_type", "未知 Swath 排序策略");
    }
    if (!contains(
            config.path.turn_planner_type,
            {"direct", "dubins", "dubins_cc", "reeds_shepp"})) {
        addIssue(
            issues, PlannerConfigErrorCode::kUnsupportedValue,
            "path.turn_planner_type", "未知转弯规划器");
    }

    if (!nearlyEqual(
            config.swath.coverage_width, config.fill.coverage_width) ||
        !nearlyEqual(
            config.swath.coverage_width, config.path.coverage_width)) {
        addIssue(
            issues, PlannerConfigErrorCode::kInconsistentValue,
            "coverage_width", "三个模块的覆盖宽度必须一致");
    }
    if (!nearlyEqual(
            config.swath.swath_overlap_ratio,
            config.path.swath_overlap_ratio)) {
        addIssue(
            issues, PlannerConfigErrorCode::kInconsistentValue,
            "swath_overlap_ratio", "Swath 与路径模块的重叠率必须一致");
    }
    if (config.fill.boundary_type != config.path.boundary_type) {
        addIssue(
            issues, PlannerConfigErrorCode::kInconsistentValue,
            "boundary_type", "补线与路径模块的边界策略必须一致");
    }
    if (!nearlyEqual(
            config.swath.endpoint_shrink, config.path.endpoint_shrink)) {
        addIssue(
            issues, PlannerConfigErrorCode::kInconsistentValue,
            "endpoint_shrink", "Swath 与路径模块的端点缩进必须一致");
    }
    if (!nearlyEqual(
            config.fill.boundary_margin, config.path.boundary_margin)) {
        addIssue(
            issues, PlannerConfigErrorCode::kInconsistentValue,
            "boundary_margin", "补线与路径模块的边界余量必须一致");
    }

    return issues;
}

}  // namespace yingshi
