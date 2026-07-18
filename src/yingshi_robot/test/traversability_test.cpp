#include <gtest/gtest.h>

#include <algorithm>
#include <initializer_list>
#include <limits>
#include <string>
#include <utility>

#include "yingshi_robot/traversability.hpp"

namespace {

f2c::types::LinearRing makeRing(
    std::initializer_list<std::pair<double, double>> vertices)
{
    f2c::types::LinearRing ring;
    for (const auto& [x, y] : vertices) {
        ring.addPoint(f2c::types::Point(x, y));
    }
    const auto& first = *vertices.begin();
    ring.addPoint(f2c::types::Point(first.first, first.second));
    return ring;
}

f2c::types::Cell makeCell(
    std::initializer_list<std::pair<double, double>> vertices)
{
    f2c::types::Cell cell;
    cell.addRing(makeRing(vertices));
    return cell;
}

void expectAreaConserved(const yingshi::TraversabilityResult& result)
{
    EXPECT_NEAR(
        result.reachable_area + result.excluded_area,
        result.original_area,
        1e-6 * std::max(1.0, result.original_area));
}

}  // namespace

TEST(Traversability, KeepsOneWideReachableComponent)
{
    const auto field = makeCell({
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}, {0.0, 5.0}});
    yingshi::TraversabilityParams params;
    params.robot_width = 1.0;
    params.max_excluded_area_ratio = 0.05;

    const auto result = yingshi::analyzeTraversability(field, params);

    ASSERT_TRUE(result.analysis_valid) << result.error_message;
    EXPECT_EQ(result.center_space.size(), 1U);
    EXPECT_EQ(result.component_count, 1U);
    EXPECT_NEAR(result.original_area, 50.0, 1e-6);
    EXPECT_GT(result.reachable_area, 49.0);
    EXPECT_LT(result.excluded_area_ratio, 0.02);
    EXPECT_FALSE(result.exclusion_limit_exceeded);
    expectAreaConserved(result);
}

TEST(Traversability, SplitsRoomsConnectedBySubRobotWidthCorridor)
{
    const auto field = makeCell({
        {0.0, 0.0}, {4.0, 0.0}, {4.0, 1.75},
        {6.0, 1.75}, {6.0, 0.0}, {10.0, 0.0},
        {10.0, 4.0}, {6.0, 4.0}, {6.0, 2.25},
        {4.0, 2.25}, {4.0, 4.0}, {0.0, 4.0}});
    yingshi::TraversabilityParams params;
    params.robot_width = 1.0;
    params.max_excluded_area_ratio = 0.20;

    const auto result = yingshi::analyzeTraversability(field, params);

    ASSERT_TRUE(result.analysis_valid) << result.error_message;
    EXPECT_EQ(result.component_count, 2U);
    EXPECT_EQ(result.center_space.size(), 2U);
    EXPECT_EQ(result.coverage_components.size(), 2U);
    EXPECT_GT(result.excluded_area, 0.0);
    EXPECT_FALSE(result.exclusion_limit_exceeded);
    EXPECT_TRUE(result.requires_repositioning);
    expectAreaConserved(result);
}

TEST(Traversability, PreservesASmallPhysicalObstacle)
{
    auto field = makeCell({
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
    field.addRing(makeRing({
        {4.7, 4.7}, {5.3, 4.7}, {5.3, 5.3}, {4.7, 5.3}}));
    yingshi::TraversabilityParams params;
    params.robot_width = 0.75;

    const auto result = yingshi::analyzeTraversability(field, params);

    ASSERT_TRUE(result.analysis_valid) << result.error_message;
    ASSERT_EQ(result.component_count, 1U);
    EXPECT_GT(result.center_space.getGeometry(0).size(), 1U);
    EXPECT_NEAR(result.original_area, 99.64, 1e-6);
    EXPECT_LE(result.reachable_area, result.original_area);
    EXPECT_FALSE(result.requires_repositioning);
    expectAreaConserved(result);
}

TEST(Traversability, GatesTheSameGeometryAtAStricterLimit)
{
    const auto field = makeCell({
        {0.0, 0.0}, {4.0, 0.0}, {4.0, 1.75},
        {6.0, 1.75}, {6.0, 0.0}, {10.0, 0.0},
        {10.0, 4.0}, {6.0, 4.0}, {6.0, 2.25},
        {4.0, 2.25}, {4.0, 4.0}, {0.0, 4.0}});
    yingshi::TraversabilityParams params;
    params.robot_width = 1.0;
    params.max_excluded_area_ratio = 0.001;

    const auto result = yingshi::analyzeTraversability(field, params);

    ASSERT_TRUE(result.analysis_valid) << result.error_message;
    EXPECT_GT(result.excluded_area_ratio, params.max_excluded_area_ratio);
    EXPECT_TRUE(result.exclusion_limit_exceeded);
}

TEST(Traversability, ReportsPositiveClearanceAsExcludedCoverage)
{
    const auto field = makeCell({
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}, {0.0, 5.0}});
    yingshi::TraversabilityParams params;
    params.robot_width = 1.0;
    params.clearance_margin = 0.2;
    params.max_excluded_area_ratio = 0.20;

    const auto result = yingshi::analyzeTraversability(field, params);

    ASSERT_TRUE(result.analysis_valid) << result.error_message;
    EXPECT_EQ(result.component_count, 1U);
    EXPECT_GT(result.excluded_area, 5.0);
    EXPECT_GT(result.excluded_area_ratio, 0.10);
    EXPECT_FALSE(result.exclusion_limit_exceeded);
    expectAreaConserved(result);
}

TEST(Traversability, ReportsAnEntirelyNonTraversableStrip)
{
    const auto field = makeCell({
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 0.6}, {0.0, 0.6}});
    yingshi::TraversabilityParams params;
    params.robot_width = 1.0;
    params.max_excluded_area_ratio = 0.10;

    const auto result = yingshi::analyzeTraversability(field, params);

    ASSERT_TRUE(result.analysis_valid) << result.error_message;
    EXPECT_EQ(result.component_count, 0U);
    EXPECT_NEAR(result.excluded_area, result.original_area, 1e-6);
    EXPECT_NEAR(result.excluded_area_ratio, 1.0, 1e-9);
    EXPECT_TRUE(result.exclusion_limit_exceeded);
    expectAreaConserved(result);
}

TEST(Traversability, AllowsTotalExclusionWhenTheConfiguredLimitIsOne)
{
    const auto field = makeCell({
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 0.6}, {0.0, 0.6}});
    yingshi::TraversabilityParams params;
    params.robot_width = 1.0;
    params.max_excluded_area_ratio = 1.0;

    const auto result = yingshi::analyzeTraversability(field, params);

    ASSERT_TRUE(result.analysis_valid) << result.error_message;
    EXPECT_NEAR(result.excluded_area_ratio, 1.0, 1e-9);
    EXPECT_FALSE(result.exclusion_limit_exceeded);
}

TEST(Traversability, RejectsInvalidAnalysisParameters)
{
    const auto field = makeCell({
        {0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}});
    yingshi::TraversabilityParams params;
    params.robot_width = 0.0;
    params.max_excluded_area_ratio = 1.5;

    const auto result = yingshi::analyzeTraversability(field, params);

    EXPECT_FALSE(result.analysis_valid);
    EXPECT_FALSE(result.error_message.empty());
}

TEST(Traversability, RejectsANegativeClearanceMargin)
{
    const auto field = makeCell({
        {0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}});
    yingshi::TraversabilityParams params;
    params.clearance_margin = -0.01;

    const auto result = yingshi::analyzeTraversability(field, params);

    EXPECT_FALSE(result.analysis_valid);
    EXPECT_NE(result.error_message.find("clearance_margin"), std::string::npos);
}

TEST(Traversability, RejectsOverflowInDerivedCenterClearance)
{
    const auto field = makeCell({
        {0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}});
    yingshi::TraversabilityParams params;
    params.robot_width = std::numeric_limits<double>::max();
    params.clearance_margin = std::numeric_limits<double>::max();

    const auto result = yingshi::analyzeTraversability(field, params);

    EXPECT_FALSE(result.analysis_valid);
    EXPECT_NE(result.error_message.find("center clearance"), std::string::npos);
}

TEST(Traversability, RejectsAnUnboundedRatioTolerance)
{
    const auto field = makeCell({
        {0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}});
    yingshi::TraversabilityParams params;
    params.ratio_tolerance = 2.0;

    const auto result = yingshi::analyzeTraversability(field, params);

    EXPECT_FALSE(result.analysis_valid);
    EXPECT_NE(result.error_message.find("ratio_tolerance"), std::string::npos);
}
