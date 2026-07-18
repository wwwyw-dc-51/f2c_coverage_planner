#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "yingshi_robot/planner_config.hpp"

namespace {

bool hasIssueFor(
    const std::vector<yingshi::PlannerConfigIssue>& issues,
    const std::string& field)
{
    for (const auto& issue : issues) {
        if (issue.field == field) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST(PlannerConfig, AcceptsDefaultConfiguration)
{
    const yingshi::PlannerConfig config;

    EXPECT_TRUE(yingshi::validatePlannerConfig(config).empty());
}

TEST(PlannerConfig, RejectsUnsafeNumericValuesWithoutSilentlyClamping)
{
    yingshi::PlannerConfig config;
    config.path.robot_width = 0.0;
    config.path.path_resolution = -0.1;
    config.swath.swath_overlap_ratio = 0.75;
    config.fill.boundary_margin = std::numeric_limits<double>::quiet_NaN();
    config.swath.endpoint_shrink = -0.01;
    config.path.endpoint_shrink = -0.01;

    const auto issues = yingshi::validatePlannerConfig(config);

    EXPECT_TRUE(hasIssueFor(issues, "path.robot_width"));
    EXPECT_TRUE(hasIssueFor(issues, "path.path_resolution"));
    EXPECT_TRUE(hasIssueFor(issues, "swath.swath_overlap_ratio"));
    EXPECT_TRUE(hasIssueFor(issues, "fill.boundary_margin"));
    EXPECT_TRUE(hasIssueFor(issues, "swath.endpoint_shrink"));
    EXPECT_TRUE(hasIssueFor(issues, "path.endpoint_shrink"));
}

TEST(PlannerConfig, RejectsEmptyAngleCandidate)
{
    yingshi::PlannerConfig config;
    config.swath.angle_candidates = "0, 45,";

    const auto issues = yingshi::validatePlannerConfig(config);

    EXPECT_TRUE(hasIssueFor(issues, "swath.angle_candidates"));
}

TEST(PlannerConfig, RejectsInvalidRuntimeOnlyParameters)
{
    yingshi::PlannerConfig config;
    config.runtime.decomposition_angle =
        std::numeric_limits<double>::quiet_NaN();
    config.runtime.mid_hl_width_ratio = -0.1;
    config.runtime.no_hl_width_ratio =
        std::numeric_limits<double>::infinity();
    config.runtime.min_hole_area = -0.1;
    config.runtime.eval_grid_resolution = 0.0;
    config.runtime.eval_coverage_threshold = 1.1;

    const auto issues = yingshi::validatePlannerConfig(config);

    EXPECT_TRUE(hasIssueFor(issues, "runtime.decomposition_angle"));
    EXPECT_TRUE(hasIssueFor(issues, "runtime.mid_hl_width_ratio"));
    EXPECT_TRUE(hasIssueFor(issues, "runtime.no_hl_width_ratio"));
    EXPECT_TRUE(hasIssueFor(issues, "runtime.min_hole_area"));
    EXPECT_TRUE(hasIssueFor(issues, "runtime.eval_grid_resolution"));
    EXPECT_TRUE(hasIssueFor(issues, "runtime.eval_coverage_threshold"));
}

TEST(PlannerConfig, RejectsConflictingCopiesOfSharedPhysicalParameters)
{
    yingshi::PlannerConfig config;
    config.fill.coverage_width = 0.90;
    config.path.coverage_width = 0.90;

    const auto issues = yingshi::validatePlannerConfig(config);

    EXPECT_TRUE(hasIssueFor(issues, "coverage_width"));
}

TEST(PlannerConfig, RejectsOpenBoundaryFallbackThatShrinksInward)
{
    yingshi::PlannerConfig config;
    config.fill.boundary_type = "open";
    config.path.boundary_type = "open";
    config.fill.boundary_margin = 0.0;
    config.path.boundary_margin = 0.0;
    config.fill.open_default_margin = 0.2;

    const auto issues = yingshi::validatePlannerConfig(config);

    EXPECT_TRUE(hasIssueFor(issues, "fill.open_default_margin"));
}

TEST(PlannerConfig, RejectsUnsupportedStrategyNames)
{
    yingshi::PlannerConfig config;
    config.fill.boundary_type = "soft-ish";
    config.path.swath_order_type = "nearest-ish";
    config.path.turn_planner_type = "teleport";
    config.swath.angle_candidates = "0, 45, invalid";

    const auto issues = yingshi::validatePlannerConfig(config);

    EXPECT_TRUE(hasIssueFor(issues, "fill.boundary_type"));
    EXPECT_TRUE(hasIssueFor(issues, "path.swath_order_type"));
    EXPECT_TRUE(hasIssueFor(issues, "path.turn_planner_type"));
    EXPECT_TRUE(hasIssueFor(issues, "swath.angle_candidates"));
}
