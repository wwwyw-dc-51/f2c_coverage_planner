#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <utility>

#include "yingshi_robot/planner_core.hpp"

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

yingshi::PlanningRequest makeNotchedRequest()
{
    yingshi::PlanningRequest request;
    request.polygon.addRing(makeRing({
        {0.0, 0.0}, {6.0, 0.0}, {6.0, 12.0},
        {25.0, 12.0}, {25.0, 25.0}, {0.0, 25.0}}));

    auto hole = makeRing({
        {9.0, 13.5}, {19.0, 13.5},
        {19.0, 23.5}, {9.0, 23.5}});
    request.polygon.addRing(hole);
    request.holes.push_back(hole);

    request.robot_width = 0.75;
    request.coverage_width = 0.90;
    request.mid_hl_width_ratio = 0.20;
    request.no_hl_width_ratio = 0.0;
    request.swath_overlap_ratio = 0.03;
    request.swath_endpoint_shrink_distance = 0.03;
    request.min_swath_length = 0.5;
    request.decomposition_enabled = true;
    request.use_sweep_decomp = true;
    request.merge_angle_threshold = 60.0;
    request.swath_angle_optimization = true;
    request.swath_order_type = "boustrophedon";
    request.turn_planner_type = "direct";
    request.filter_tiny_cells = true;
    request.path_simplify_enabled = true;
    request.path_simplify_tolerance = 0.05;
    request.path_simplify_turn_threshold = 0.15;
    request.boundary_type = "closed";
    return request;
}

double pathLength(const std::vector<f2c::types::Point>& points)
{
    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += std::hypot(
            points[i].getX() - points[i - 1].getX(),
            points[i].getY() - points[i - 1].getY());
    }
    return length;
}

TEST(PlannerCore, UsesValidatedV97PhysicalDefaults)
{
    const yingshi::PlanningRequest request;

    EXPECT_DOUBLE_EQ(request.robot_width, 0.75);
    EXPECT_DOUBLE_EQ(request.coverage_width, 0.90);
    EXPECT_DOUBLE_EQ(request.mid_hl_width_ratio, 0.20);
    EXPECT_DOUBLE_EQ(request.no_hl_width_ratio, 0.0);
    EXPECT_DOUBLE_EQ(request.swath_overlap_ratio, 0.03);
    EXPECT_DOUBLE_EQ(request.min_swath_length, 0.5);
    EXPECT_TRUE(request.use_sweep_decomp);
    EXPECT_TRUE(request.swath_angle_optimization);
    EXPECT_TRUE(request.filter_tiny_cells);
    EXPECT_TRUE(request.path_simplify_enabled);
}

TEST(PlannerCore, MatchesValidatedNotchedRouteBaseline)
{
    yingshi::PlannerCore planner;

    const auto result = planner.plan(makeNotchedRequest());

    ASSERT_TRUE(result.success) << result.error_message;
    EXPECT_EQ(result.total_swaths, 48U);
    EXPECT_NEAR(pathLength(result.path_points), 454.51, 2.0);
    EXPECT_FALSE(result.path_has_crossings);
}

}  // namespace
