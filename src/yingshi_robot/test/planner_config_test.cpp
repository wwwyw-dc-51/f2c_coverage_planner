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

    const auto issues = yingshi::validatePlannerConfig(config);

    EXPECT_TRUE(hasIssueFor(issues, "path.robot_width"));
    EXPECT_TRUE(hasIssueFor(issues, "path.path_resolution"));
    EXPECT_TRUE(hasIssueFor(issues, "swath.swath_overlap_ratio"));
    EXPECT_TRUE(hasIssueFor(issues, "fill.boundary_margin"));
}

TEST(PlannerConfig, RejectsConflictingCopiesOfSharedPhysicalParameters)
{
    yingshi::PlannerConfig config;
    config.fill.coverage_width = 0.90;
    config.path.coverage_width = 0.90;

    const auto issues = yingshi::validatePlannerConfig(config);

    EXPECT_TRUE(hasIssueFor(issues, "coverage_width"));
}

TEST(PlannerConfig, RejectsUnsupportedStrategyNames)
{
    yingshi::PlannerConfig config;
    config.fill.boundary_type = "soft-ish";
    config.path.swath_order_type = "nearest-ish";
    config.path.turn_planner_type = "teleport";

    const auto issues = yingshi::validatePlannerConfig(config);

    EXPECT_TRUE(hasIssueFor(issues, "fill.boundary_type"));
    EXPECT_TRUE(hasIssueFor(issues, "path.swath_order_type"));
    EXPECT_TRUE(hasIssueFor(issues, "path.turn_planner_type"));
}
