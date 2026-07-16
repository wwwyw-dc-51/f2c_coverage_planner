#include <gtest/gtest.h>

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

TEST(DirectPath, PreservesEveryRouteConnectionWaypoint)
{
    f2c::types::Route route;
    f2c::types::Swaths first_group;
    first_group.push_back(makeSwath(0.0, 0.0, 1.0, 0.0));
    route.addConnectedSwaths(f2c::types::MultiPoint(), first_group);

    f2c::types::MultiPoint l_connection;
    l_connection.addPoint(f2c::types::Point(1.0, 0.0));
    l_connection.addPoint(f2c::types::Point(1.0, 2.0));
    l_connection.addPoint(f2c::types::Point(2.0, 2.0));
    yingshi::appendConnectedSwath(
        route, l_connection, makeSwath(2.0, 2.0, 3.0, 2.0));

    const auto path = yingshi::planDirectPath(route, 1.0);
    const auto points = yingshi::materializePath(path);

    ASSERT_EQ(path.size(), 4U);
    EXPECT_EQ(path[0].type, f2c::types::PathSectionType::SWATH);
    EXPECT_EQ(path[1].type, f2c::types::PathSectionType::TURN);
    EXPECT_EQ(path[2].type, f2c::types::PathSectionType::TURN);
    EXPECT_EQ(path[3].type, f2c::types::PathSectionType::SWATH);
    ASSERT_EQ(points.size(), 5U);
    EXPECT_DOUBLE_EQ(points[0].getX(), 0.0);
    EXPECT_DOUBLE_EQ(points[1].getX(), 1.0);
    EXPECT_DOUBLE_EQ(points[2].getX(), 1.0);
    EXPECT_DOUBLE_EQ(points[2].getY(), 2.0);
    EXPECT_DOUBLE_EQ(points[3].getX(), 2.0);
    EXPECT_DOUBLE_EQ(points[4].getX(), 3.0);
    EXPECT_DOUBLE_EQ(yingshi::polylineLength(points), 5.0);
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
