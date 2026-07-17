#include <gtest/gtest.h>

#include <cmath>

#include "yingshi_robot/boundary_filler.hpp"
#include "yingshi_robot/path_geometry.hpp"
#include "yingshi_robot/path_planner.hpp"

namespace {

f2c::types::Swath makeSwath(
    double x0, double y0, double x1, double y1, double width = 0.45)
{
    f2c::types::LineString line;
    line.addPoint(f2c::types::Point(x0, y0));
    line.addPoint(f2c::types::Point(x1, y1));
    return f2c::types::Swath(line, width);
}

f2c::types::Cell makeRectangle(
    double min_x, double min_y, double max_x, double max_y)
{
    return f2c::types::Cell(yingshi::makeClosedRing({
        f2c::types::Point(min_x, min_y),
        f2c::types::Point(max_x, min_y),
        f2c::types::Point(max_x, max_y),
        f2c::types::Point(min_x, max_y),
    }));
}

double swathMidY(const f2c::types::Swath& swath)
{
    return 0.5 * (
        swath.startPoint().getY() + swath.endPoint().getY());
}

TEST(BoundaryFill, InsertsAnEndSideFillWhenThereIsNoStartSideFill)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto top_cell = makeRectangle(0.0, 8.1, 10.0, 10.0);
    f2c::types::Swaths swaths;
    swaths.push_back(makeSwath(0.15, 8.5, 9.85, 8.5, 0.90));

    yingshi::fillBoundaryGaps(
        swaths, top_cell, full_polygon, 0.0, 0.90, 0.0);

    ASSERT_EQ(swaths.size(), 2U);
    EXPECT_NEAR(swathMidY(swaths.at(1)), 9.55, 1e-9);
}

TEST(BoundaryFill, FillsAnInternalCellSeamBeyondHalfCoverageWidth)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto middle_cell = makeRectangle(0.0, 4.0, 10.0, 6.0);
    f2c::types::Swaths swaths;
    swaths.push_back(makeSwath(0.15, 4.4, 9.85, 4.4, 0.90));

    yingshi::fillBoundaryGaps(
        swaths, middle_cell, full_polygon, 0.0, 0.90, 0.0);

    ASSERT_EQ(swaths.size(), 2U);
    EXPECT_NEAR(swathMidY(swaths.at(1)), 6.0, 1e-9);
}

TEST(BoundaryFill, SkipsOuterFillWhenExistingSwathAlreadyCoversBoundary)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto bottom_cell = makeRectangle(0.0, 0.0, 10.0, 1.2);
    f2c::types::Swaths swaths;
    swaths.push_back(makeSwath(0.15, 0.2, 9.85, 0.2, 0.90));
    swaths.push_back(makeSwath(0.15, 0.8, 9.85, 0.8, 0.90));

    yingshi::fillBoundaryGaps(
        swaths, bottom_cell, full_polygon, 0.0, 0.90, 0.0);

    EXPECT_EQ(swaths.size(), 2U);
}

TEST(BoundaryFill, KeepsOuterFillWhenSlantedSwathEndpointLeavesBoundaryGap)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto bottom_cell = makeRectangle(0.0, 0.0, 10.0, 1.2);
    f2c::types::Swaths swaths;
    swaths.push_back(makeSwath(0.15, 0.1, 9.85, 0.7, 0.90));

    yingshi::fillBoundaryGaps(
        swaths, bottom_cell, full_polygon, 0.0, 0.90, 0.0);

    bool found_bottom_boundary_fill = false;
    for (size_t i = 0; i < swaths.size(); ++i) {
        const auto& swath = swaths.at(i);
        if (std::abs(swathMidY(swath) - 0.45) < 1e-9 &&
            std::abs(
                swath.startPoint().getY() -
                swath.endPoint().getY()) < 1e-9) {
            found_bottom_boundary_fill = true;
            break;
        }
    }
    EXPECT_TRUE(found_bottom_boundary_fill);
}

TEST(BoundaryFill, RemovesSeamFillWhenTwoSidesAlreadyCoverTheGap)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    f2c::types::Cells cells;
    cells.addGeometry(makeRectangle(0.0, 0.0, 10.0, 4.0));
    cells.addGeometry(makeRectangle(0.0, 4.0, 10.0, 8.0));

    f2c::types::Swaths lower;
    lower.push_back(makeSwath(0.15, 3.55, 9.85, 3.55, 0.90));
    f2c::types::Swaths upper;
    upper.push_back(makeSwath(0.0, 4.0, 10.0, 4.0, 0.90));
    upper.push_back(makeSwath(0.15, 4.48, 9.85, 4.48, 0.90));
    f2c::types::SwathsByCells swaths_by_cells {lower, upper};

    EXPECT_EQ(yingshi::pruneRedundantCellSeamFills(
        swaths_by_cells, cells, full_polygon, 0.90), 1U);
    EXPECT_EQ(swaths_by_cells.sizeTotal(), 2U);
}

TEST(BoundaryFill, KeepsOneSeamFillForARealTwoSidedGap)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    f2c::types::Cells cells;
    cells.addGeometry(makeRectangle(0.0, 0.0, 10.0, 4.0));
    cells.addGeometry(makeRectangle(0.0, 4.0, 10.0, 8.0));

    f2c::types::Swaths lower;
    lower.push_back(makeSwath(0.15, 3.40, 9.85, 3.40, 0.90));
    f2c::types::Swaths upper;
    upper.push_back(makeSwath(0.0, 4.0, 10.0, 4.0, 0.90));
    upper.push_back(makeSwath(0.15, 4.60, 9.85, 4.60, 0.90));
    f2c::types::SwathsByCells swaths_by_cells {lower, upper};

    EXPECT_EQ(yingshi::pruneRedundantCellSeamFills(
        swaths_by_cells, cells, full_polygon, 0.90), 0U);
    EXPECT_EQ(swaths_by_cells.sizeTotal(), 3U);
}

TEST(BoundaryFill, KeepsOnlyOneCopyOfASharedSeamFill)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    f2c::types::Cells cells;
    cells.addGeometry(makeRectangle(0.0, 0.0, 10.0, 4.0));
    cells.addGeometry(makeRectangle(0.0, 4.0, 10.0, 8.0));

    f2c::types::Swaths lower;
    lower.push_back(makeSwath(0.15, 3.40, 9.85, 3.40, 0.90));
    lower.push_back(makeSwath(0.0, 4.0, 10.0, 4.0, 0.90));
    f2c::types::Swaths upper;
    upper.push_back(makeSwath(10.0, 4.0, 0.0, 4.0, 0.90));
    upper.push_back(makeSwath(0.15, 4.60, 9.85, 4.60, 0.90));
    f2c::types::SwathsByCells swaths_by_cells {lower, upper};

    EXPECT_EQ(yingshi::pruneRedundantCellSeamFills(
        swaths_by_cells, cells, full_polygon, 0.90), 1U);
    EXPECT_EQ(swaths_by_cells.sizeTotal(), 3U);
}

TEST(BoundaryFill, KeepsSeamFillWhenOppositeSwathsCoverDifferentIntervals)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    f2c::types::Cells cells;
    cells.addGeometry(makeRectangle(0.0, 0.0, 10.0, 4.0));
    cells.addGeometry(makeRectangle(0.0, 4.0, 10.0, 8.0));

    f2c::types::Swaths lower;
    lower.push_back(makeSwath(0.0, 3.55, 4.0, 3.55, 0.90));
    f2c::types::Swaths upper;
    upper.push_back(makeSwath(0.0, 4.0, 10.0, 4.0, 0.90));
    upper.push_back(makeSwath(6.0, 4.48, 10.0, 4.48, 0.90));
    f2c::types::SwathsByCells swaths_by_cells {lower, upper};

    EXPECT_EQ(yingshi::pruneRedundantCellSeamFills(
        swaths_by_cells, cells, full_polygon, 0.90), 0U);
    EXPECT_EQ(swaths_by_cells.sizeTotal(), 3U);
}

TEST(BoundaryFill, KeepsSeamFillWhenCommonCoverageMissesOneEndpoint)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    f2c::types::Cells cells;
    cells.addGeometry(makeRectangle(0.0, 0.0, 10.0, 4.0));
    cells.addGeometry(makeRectangle(0.0, 4.0, 10.0, 8.0));

    f2c::types::Swaths lower;
    lower.push_back(makeSwath(0.0, 3.55, 9.1, 3.55, 0.90));
    f2c::types::Swaths upper;
    upper.push_back(makeSwath(0.0, 4.0, 10.0, 4.0, 0.90));
    upper.push_back(makeSwath(0.0, 4.48, 9.1, 4.48, 0.90));
    f2c::types::SwathsByCells swaths_by_cells {lower, upper};

    EXPECT_EQ(yingshi::pruneRedundantCellSeamFills(
        swaths_by_cells, cells, full_polygon, 0.90), 0U);
    EXPECT_EQ(swaths_by_cells.sizeTotal(), 3U);
}

TEST(RouteInvariant, AppendedBoundarySwathStartsANewConnectedGroup)
{
    f2c::types::Route route;
    f2c::types::Swaths initial_group;
    initial_group.push_back(makeSwath(0.0, 0.0, 4.0, 0.0));
    route.addConnectedSwaths(f2c::types::MultiPoint(), initial_group);

    f2c::types::MultiPoint connection;
    connection.addPoint(f2c::types::Point(4.0, 0.0));
    connection.addPoint(f2c::types::Point(4.0, 1.0));
    yingshi::appendConnectedSwath(
        route, connection, makeSwath(4.0, 1.0, 0.0, 1.0));

    ASSERT_EQ(route.sizeVectorSwaths(), 2U);
    ASSERT_EQ(route.sizeConnections(), 2U);
    ASSERT_EQ(route.getSwaths(0).size(), 1U);
    ASSERT_EQ(route.getSwaths(1).size(), 1U);
    ASSERT_EQ(route.getConnection(1).size(), 2U);
    EXPECT_DOUBLE_EQ(route.getConnection(1).getGeometry(0).getX(), 4.0);
    EXPECT_DOUBLE_EQ(route.getSwaths(1).at(0).startPoint().getY(), 1.0);
}

TEST(RouteInvariant, SynchronizesConnectionAfterSwathEndpointAdjustment)
{
    f2c::types::Route route;
    f2c::types::Swaths first_group;
    first_group.push_back(makeSwath(0.0, 0.0, 10.0, 0.0));
    route.addConnectedSwaths(f2c::types::MultiPoint(), first_group);

    f2c::types::MultiPoint connection;
    connection.addPoint(f2c::types::Point(10.0, 0.0));
    connection.addPoint(f2c::types::Point(10.5, 0.5));
    connection.addPoint(f2c::types::Point(10.0, 1.0));
    f2c::types::Swaths second_group;
    second_group.push_back(makeSwath(10.0, 1.0, 0.0, 1.0));
    route.addConnectedSwaths(connection, second_group);

    f2c::types::Swaths adjusted_first;
    adjusted_first.push_back(makeSwath(0.1, 0.0, 9.9, 0.0));
    f2c::types::Swaths adjusted_second;
    adjusted_second.push_back(makeSwath(9.9, 1.0, 0.1, 1.0));
    route.setSwaths(0, adjusted_first);
    route.setSwaths(1, adjusted_second);

    EXPECT_EQ(yingshi::synchronizeRouteConnectionEndpoints(route, 0.2), 1U);
    const auto& synchronized = route.getConnection(1);
    ASSERT_EQ(synchronized.size(), 3U);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(0).getX(), 9.9);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(0).getY(), 0.0);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(1).getX(), 10.5);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(1).getY(), 0.5);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(2).getX(), 9.9);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(2).getY(), 1.0);
}

TEST(RouteInvariant, DropsSingleStaleEndpointAfterSwathAdjustment)
{
    f2c::types::Route route;
    f2c::types::Swaths first_group;
    first_group.push_back(makeSwath(0.0, 0.0, 9.94, 0.0));
    route.addConnectedSwaths(f2c::types::MultiPoint(), first_group);

    f2c::types::MultiPoint stale_endpoint;
    stale_endpoint.addPoint(f2c::types::Point(10.0, 0.0));
    f2c::types::Swaths second_group;
    second_group.push_back(makeSwath(9.85, 0.0, 9.85, 1.0));
    route.addConnectedSwaths(stale_endpoint, second_group);

    EXPECT_EQ(yingshi::synchronizeRouteConnectionEndpoints(route, 0.06), 1U);
    const auto& synchronized = route.getConnection(1);
    ASSERT_EQ(synchronized.size(), 2U);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(0).getX(), 9.94);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(1).getX(), 9.85);
}

TEST(RouteInvariant, PreservesSingleDistantDetourWaypoint)
{
    f2c::types::Route route;
    f2c::types::Swaths first_group;
    first_group.push_back(makeSwath(0.0, 0.0, 9.94, 0.0));
    route.addConnectedSwaths(f2c::types::MultiPoint(), first_group);

    f2c::types::MultiPoint detour;
    detour.addPoint(f2c::types::Point(11.0, 1.0));
    f2c::types::Swaths second_group;
    second_group.push_back(makeSwath(9.85, 0.0, 9.85, 1.0));
    route.addConnectedSwaths(detour, second_group);

    EXPECT_EQ(yingshi::synchronizeRouteConnectionEndpoints(route, 0.06), 1U);
    const auto& synchronized = route.getConnection(1);
    ASSERT_EQ(synchronized.size(), 3U);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(1).getX(), 11.0);
    EXPECT_DOUBLE_EQ(synchronized.getGeometry(1).getY(), 1.0);
}

TEST(DirectPath, PreservesEveryRouteConnectionWaypoint)
{
    f2c::types::Route route;
    f2c::types::Swaths first_group;
    first_group.push_back(makeSwath(0.0, 0.0, 1.0, 0.0));
    route.addConnectedSwaths(f2c::types::MultiPoint(), first_group);

    f2c::types::MultiPoint detour_connection;
    detour_connection.addPoint(f2c::types::Point(1.0, 0.0));
    detour_connection.addPoint(f2c::types::Point(1.0, 2.0));
    detour_connection.addPoint(f2c::types::Point(2.0, 2.0));
    detour_connection.addPoint(f2c::types::Point(2.0, 3.0));
    yingshi::appendConnectedSwath(
        route, detour_connection, makeSwath(2.0, 3.0, 3.0, 3.0));

    const auto path = yingshi::planDirectPath(route, 1.0);
    const auto points = yingshi::materializePath(path);

    ASSERT_EQ(path.size(), 5U);
    EXPECT_EQ(path[0].type, f2c::types::PathSectionType::SWATH);
    EXPECT_EQ(path[1].type, f2c::types::PathSectionType::TURN);
    EXPECT_EQ(path[2].type, f2c::types::PathSectionType::TURN);
    EXPECT_EQ(path[3].type, f2c::types::PathSectionType::TURN);
    EXPECT_EQ(path[4].type, f2c::types::PathSectionType::SWATH);
    ASSERT_EQ(points.size(), 6U);
    EXPECT_DOUBLE_EQ(points[0].getX(), 0.0);
    EXPECT_DOUBLE_EQ(points[1].getX(), 1.0);
    EXPECT_DOUBLE_EQ(points[2].getX(), 1.0);
    EXPECT_DOUBLE_EQ(points[2].getY(), 2.0);
    EXPECT_DOUBLE_EQ(points[3].getX(), 2.0);
    EXPECT_DOUBLE_EQ(points[4].getY(), 3.0);
    EXPECT_DOUBLE_EQ(points[5].getX(), 3.0);
    EXPECT_DOUBLE_EQ(yingshi::polylineLength(points), 6.0);
}

TEST(HoleAwareRoute, RepairsBorderFillConnectionAddedAfterRoutePlanning)
{
    const auto hole = yingshi::makeClosedRing({
        f2c::types::Point(9.0, 13.5),
        f2c::types::Point(19.0, 13.5),
        f2c::types::Point(19.0, 23.5),
        f2c::types::Point(9.0, 23.5),
    });
    const std::vector<f2c::types::LinearRing> holes {hole};

    f2c::types::Route route;
    f2c::types::Swaths last_planned_group;
    last_planned_group.push_back(
        makeSwath(0.21, 13.7865, 0.15, 13.7865, 0.90));
    route.addConnectedSwaths(f2c::types::MultiPoint(), last_planned_group);

    f2c::types::MultiPoint crossing_connection;
    crossing_connection.addPoint(f2c::types::Point(0.15, 13.7865));
    crossing_connection.addPoint(f2c::types::Point(25.0, 24.55));
    yingshi::appendConnectedSwath(
        route, crossing_connection,
        makeSwath(25.0, 24.55, 24.94, 24.55, 0.90));

    ASSERT_TRUE(yingshi::segmentCrossesHole(
        0.15, 13.7865, 25.0, 24.55, holes, 50));

    EXPECT_EQ(
        yingshi::repairRouteConnectionsAroundHoles(route, holes, 0.001), 1U);

    const auto& repaired = route.getConnection(1);
    ASSERT_GT(repaired.size(), 2U);
    for (size_t i = 0; i + 1 < repaired.size(); ++i) {
        EXPECT_FALSE(yingshi::segmentCrossesHole(
            repaired.getGeometry(i).getX(), repaired.getGeometry(i).getY(),
            repaired.getGeometry(i + 1).getX(),
            repaired.getGeometry(i + 1).getY(), holes, 50));
    }

    const auto path_points = yingshi::materializePath(
        yingshi::planDirectPath(route, 1.0));
    const auto clearance_ring = f2c::types::Cell::buffer(
        f2c::types::Cell(hole), 0.0005).getExteriorRing();
    const std::vector<f2c::types::LinearRing> clearance_area {
        clearance_ring};
    for (size_t i = 0; i + 1 < path_points.size(); ++i) {
        EXPECT_FALSE(yingshi::segmentCrossesHole(
            path_points[i].getX(), path_points[i].getY(),
            path_points[i + 1].getX(), path_points[i + 1].getY(), holes, 50));
        EXPECT_FALSE(yingshi::segmentCrossesHole(
            path_points[i].getX(), path_points[i].getY(),
            path_points[i + 1].getX(), path_points[i + 1].getY(),
            clearance_area, 50));
    }
}

TEST(HoleAwareRoute, RepairsImplicitEndpointsAndDropsControlPointInsideHole)
{
    const auto hole = yingshi::makeClosedRing({
        f2c::types::Point(9.0, 13.5),
        f2c::types::Point(19.0, 13.5),
        f2c::types::Point(19.0, 23.5),
        f2c::types::Point(9.0, 23.5),
    });
    const std::vector<f2c::types::LinearRing> holes {hole};

    f2c::types::Route route;
    f2c::types::Swaths first_group;
    first_group.push_back(makeSwath(0.21, 13.7865, 0.15, 13.7865, 0.90));
    route.addConnectedSwaths(f2c::types::MultiPoint(), first_group);

    f2c::types::MultiPoint stale_connection;
    stale_connection.addPoint(f2c::types::Point(12.0, 19.0));
    yingshi::appendConnectedSwath(
        route, stale_connection,
        makeSwath(25.0, 24.55, 24.94, 24.55, 0.90));

    EXPECT_EQ(
        yingshi::repairRouteConnectionsAroundHoles(route, holes, 0.001), 1U);

    const auto path_points = yingshi::materializePath(
        yingshi::planDirectPath(route, 1.0));
    ASSERT_GT(path_points.size(), 4U);
    for (size_t i = 0; i + 1 < path_points.size(); ++i) {
        EXPECT_FALSE(yingshi::segmentCrossesHole(
            path_points[i].getX(), path_points[i].getY(),
            path_points[i + 1].getX(), path_points[i + 1].getY(), holes, 50));
    }
}

TEST(HoleGeometry, BuildsAClosedRingFromUnclosedVertices)
{
    const std::vector<f2c::types::Point> vertices {
        f2c::types::Point(1.0, 1.0),
        f2c::types::Point(3.0, 1.0),
        f2c::types::Point(3.0, 3.0),
        f2c::types::Point(1.0, 3.0),
    };

    const auto ring = yingshi::makeClosedRing(vertices);

    ASSERT_EQ(ring.size(), 5U);
    EXPECT_DOUBLE_EQ(
        ring.getGeometry(0).getX(), ring.getGeometry(4).getX());
    EXPECT_DOUBLE_EQ(
        ring.getGeometry(0).getY(), ring.getGeometry(4).getY());
    EXPECT_TRUE(yingshi::pointInPolygon(2.0, 2.0, ring));
    EXPECT_FALSE(yingshi::pointInPolygon(0.0, 2.0, ring));
    EXPECT_FALSE(yingshi::pointInPolygon(4.0, 2.0, ring));
}

}  // namespace
