#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <string>
#include <utility>

#include "yingshi_robot/boundary_filler.hpp"
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

f2c::types::LinearRing makeRing(
    const std::vector<std::pair<double, double>>& vertices)
{
    f2c::types::LinearRing ring;
    for (const auto& [x, y] : vertices) {
        ring.addPoint(f2c::types::Point(x, y));
    }
    if (!vertices.empty()) {
        ring.addPoint(f2c::types::Point(
            vertices.front().first, vertices.front().second));
    }
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
    // 该用例锁定 v9.7 路由基线；C-space 行为由独立用例验证。
    request.traversability_enabled = false;
    return request;
}

yingshi::PlanningRequest makeSplitRoomRequest()
{
    yingshi::PlanningRequest request;
    request.polygon.addRing(makeRing({
        {0.0, 0.0}, {4.0, 0.0}, {4.0, 1.75},
        {6.0, 1.75}, {6.0, 0.0}, {10.0, 0.0},
        {10.0, 4.0}, {6.0, 4.0}, {6.0, 2.25},
        {4.0, 2.25}, {4.0, 4.0}, {0.0, 4.0}}));
    request.robot_width = 1.0;
    request.coverage_width = 0.5;
    request.max_excluded_area_ratio = 0.20;
    request.decomposition_enabled = false;
    request.filter_tiny_cells = false;
    request.swath_angle_optimization = false;
    request.min_swath_length = 0.1;
    request.path_simplify_enabled = false;
    request.traversability_enabled = true;
    return request;
}

yingshi::PlanningRequest makeFactoryWorkshopRequest()
{
    yingshi::PlanningRequest request;
    request.polygon.addRing(makeRing({
        {0.0, 0.0}, {30.0, 0.0}, {30.0, 20.0}, {0.0, 20.0}}));
    const std::vector<std::vector<std::pair<double, double>>> hole_vertices {
        {{4.0, 4.0}, {4.6, 4.0}, {4.6, 4.6}, {4.0, 4.6}},
        {{15.0, 4.5}, {15.6, 4.5}, {15.6, 5.1}, {15.0, 5.1}},
        {{25.5, 3.0}, {26.1, 3.0}, {26.1, 3.6}, {25.5, 3.6}},
        {{3.5, 15.0}, {4.1, 15.0}, {4.1, 15.6}, {3.5, 15.6}},
        {{26.0, 14.0}, {26.6, 14.0}, {26.6, 14.6}, {26.0, 14.6}},
        {{1.5, 8.0}, {9.5, 8.0}, {9.5, 8.8}, {1.5, 8.8}},
        {{2.0, 10.3}, {9.0, 10.3}, {9.0, 11.1}, {2.0, 11.1}},
        {{18.0, 6.0}, {24.0, 6.0}, {24.0, 12.0}, {18.0, 12.0}},
        {{16.0, 16.0}, {22.0, 16.0}, {22.0, 19.0}, {16.0, 19.0}},
        {{9.0, 14.0}, {14.0, 13.0}, {16.0, 16.5},
         {12.0, 18.5}, {7.0, 17.0}},
    };
    for (const auto& vertices : hole_vertices) {
        auto hole = makeRing(vertices);
        request.polygon.addRing(hole);
        request.holes.push_back(hole);
    }

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
    request.traversability_enabled = true;
    request.cspace_clearance_margin = 0.0;
    request.max_excluded_area_ratio = 0.05;
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
    EXPECT_FALSE(request.traversability_enabled);
    EXPECT_FALSE(request.physical_collision_check_enabled);
    EXPECT_DOUBLE_EQ(request.physical_footprint.body_width, 0.80);
    EXPECT_DOUBLE_EQ(request.physical_footprint.cleaner_diameter, 0.45);
    EXPECT_DOUBLE_EQ(request.cspace_clearance_margin, 0.0);
    EXPECT_DOUBLE_EQ(request.max_excluded_area_ratio, 0.05);
}

TEST(PlannerCore, MatchesValidatedNotchedRouteBaseline)
{
    yingshi::PlannerCore planner;

    const auto result = planner.plan(makeNotchedRequest());

    ASSERT_TRUE(result.success) << result.error_message;
    EXPECT_EQ(result.total_swaths, 52U);  // Phase 4A sweep alignment + adaptive headland
    EXPECT_NEAR(pathLength(result.path_points), 454.51, 2.0);
    EXPECT_FALSE(result.path_has_crossings);
}

TEST(PlannerCore, KeepsShortestConnectionsBetweenOrderedSwaths)
{
    yingshi::PlanningRequest request;
    request.polygon.addRing(makeRing({
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}));
    request.decomposition_enabled = false;
    request.swath_angle_optimization = false;
    request.path_simplify_enabled = false;
    request.swath_order_type = "boustrophedon";

    yingshi::PlannerCore planner;
    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.error_message;
    ASSERT_GT(result.total_swaths, 2U);
    // Cell-block 只固定遍历顺序，不能丢掉相邻 Swath 的最短安全连接。
    EXPECT_GE(result.total_connections, result.total_swaths);
}

TEST(PlannerCore, KeepsTraversableHoleRouteSafeAfterSimplification)
{
    yingshi::PlannerCore planner;
    auto request = makeNotchedRequest();
    request.polygon = f2c::types::Cell();
    request.polygon.addRing(makeRing({
        {0.0, 0.0}, {6.0, 0.0}, {6.0, 3.0}, {7.0, 3.0},
        {7.0, 0.0}, {12.0, 0.0}, {12.0, 8.0}, {7.0, 8.0},
        {7.0, 5.0}, {6.0, 5.0}, {6.0, 8.0}, {0.0, 8.0}}));
    const auto hole = makeRing({
        {8.0, 2.0}, {11.0, 2.0}, {11.0, 4.0}, {8.0, 4.0}});
    request.polygon.addRing(hole);
    request.holes = {hole};
    request.traversability_enabled = true;

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.error_message;
    EXPECT_FALSE(result.path_has_crossings);
}

TEST(PlannerCore, KeepsDenseHoleRoutesSafe)
{
    yingshi::PlannerCore planner;
    auto request = makeNotchedRequest();
    request.polygon = f2c::types::Cell();
    request.polygon.addRing(makeRing({
        {0.0, 0.0}, {17.5, 0.0}, {17.5, 18.0}, {0.0, 18.0}}));
    const std::vector<std::pair<double, double>> x_ranges {
        {3.0, 5.0}, {6.5, 8.5}, {10.0, 12.0}, {13.5, 15.5}};
    const std::vector<std::pair<double, double>> y_ranges {
        {3.0, 6.0}, {7.5, 10.5}, {12.0, 15.0}};
    std::vector<f2c::types::LinearRing> holes;
    for (const auto& [min_y, max_y] : y_ranges) {
        for (const auto& [min_x, max_x] : x_ranges) {
            holes.push_back(makeRing({
                {min_x, min_y}, {max_x, min_y},
                {max_x, max_y}, {min_x, max_y}}));
            request.polygon.addRing(holes.back());
        }
    }
    request.holes = holes;
    request.traversability_enabled = true;

    const auto result = planner.plan(request);
    if (!result.component_plans.empty()) {
        const auto& component = result.component_plans.front();
        std::vector<f2c::types::LinearRing> component_holes;
        for (std::size_t i = 0;
             i + 1 < component.planning_polygon.size(); ++i) {
            component_holes.push_back(
                component.planning_polygon.getInteriorRing(i));
        }
        for (std::size_t i = 0; i + 1 < component.path_points.size(); ++i) {
            if (yingshi::segmentCrossesHole(
                    component.path_points[i].getX(),
                    component.path_points[i].getY(),
                    component.path_points[i + 1].getX(),
                    component.path_points[i + 1].getY(),
                    component_holes,
                    50)) {
                ADD_FAILURE() << "dense crossing segment " << i << " ("
                              << component.path_points[i].getX() << ", "
                              << component.path_points[i].getY() << ") -> ("
                              << component.path_points[i + 1].getX() << ", "
                              << component.path_points[i + 1].getY() << ")";
                break;
            }
        }
    }
    ASSERT_TRUE(result.success) << result.error_message;
}

TEST(PlannerCore, KeepsFactoryWorkshopRouteInsideCspace)
{
    yingshi::PlannerCore planner;

    const auto result = planner.plan(makeFactoryWorkshopRequest());

    ASSERT_TRUE(result.success) << result.error_message;
}

TEST(PlannerCore, RejectsAPlanThatExceedsTheExclusionGate)
{
    yingshi::PlannerCore planner;
    auto request = makeSplitRoomRequest();
    request.max_excluded_area_ratio = 0.001;

    const auto result = planner.plan(request);

    EXPECT_FALSE(result.success);
    ASSERT_TRUE(result.traversability.analysis_valid)
        << result.traversability.error_message;
    EXPECT_TRUE(result.traversability.exclusion_limit_exceeded);
    EXPECT_TRUE(result.component_plans.empty());
}

TEST(PlannerCore, PreservesDisconnectedAreasAsSeparateSubpaths)
{
    yingshi::PlannerCore planner;

    const auto result = planner.plan(makeSplitRoomRequest());

    ASSERT_TRUE(result.success) << result.error_message;
    EXPECT_TRUE(result.traversability.requires_repositioning);
    EXPECT_EQ(result.traversability.component_count, 2U);
    ASSERT_EQ(result.component_plans.size(), 2U);
    EXPECT_TRUE(result.path_points.empty());
    for (const auto& component : result.component_plans) {
        EXPECT_TRUE(component.success) << component.error_message;
        EXPECT_FALSE(component.path_points.empty());
        EXPECT_GT(component.total_swaths, 0U);
        EXPECT_FALSE(component.path_leaves_planning_area);
        EXPECT_EQ(component.out_of_planning_area_segments, 0U);
        const auto& cell = component.planning_polygon;
        for (const auto& point : component.path_points) {
            bool inside_or_on_boundary =
                yingshi::pointInPolygon(
                    point.getX(), point.getY(), cell.getExteriorRing());
            // 浮点精度：路径点可能恰好在 cell 外环边界上
            if (!inside_or_on_boundary) {
                double bbox_min_x = 1e30, bbox_max_x = -1e30,
                       bbox_min_y = 1e30, bbox_max_y = -1e30;
                for (const auto& p : cell.getExteriorRing()) {
                    bbox_min_x = std::min(bbox_min_x, p.getX());
                    bbox_max_x = std::max(bbox_max_x, p.getX());
                    bbox_min_y = std::min(bbox_min_y, p.getY());
                    bbox_max_y = std::max(bbox_max_y, p.getY());
                }
                double dx = 0, dy = 0;
                if (point.getX() < bbox_min_x) dx = bbox_min_x - point.getX();
                else if (point.getX() > bbox_max_x) dx = point.getX() - bbox_max_x;
                if (point.getY() < bbox_min_y) dy = bbox_min_y - point.getY();
                else if (point.getY() > bbox_max_y) dy = point.getY() - bbox_max_y;
                inside_or_on_boundary = (std::sqrt(dx*dx + dy*dy) <= 1e-9);
            }
            EXPECT_TRUE(inside_or_on_boundary)
                << "point (" << point.getX() << ", " << point.getY()
                << ") outside cell exterior ring";
            for (std::size_t i = 0; i + 1 < cell.size(); ++i) {
                EXPECT_FALSE(yingshi::pointInPolygon(
                    point.getX(), point.getY(),
                    cell.getInteriorRing(i)));
            }
        }
    }
}

TEST(PlannerCore, RejectsAnEntirelyNonTraversableField)
{
    yingshi::PlannerCore planner;
    yingshi::PlanningRequest request;
    request.polygon.addRing(makeRing({
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 0.6}, {0.0, 0.6}}));
    request.robot_width = 1.0;
    request.max_excluded_area_ratio = 1.0;
    request.traversability_enabled = true;

    const auto result = planner.plan(request);

    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.traversability.analysis_valid);
    EXPECT_EQ(result.traversability.component_count, 0U);
    EXPECT_FALSE(result.traversability.exclusion_limit_exceeded);
    EXPECT_NE(result.error_message.find("no traversable component"),
              std::string::npos);
}

}  // namespace
