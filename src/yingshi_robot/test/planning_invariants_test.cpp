#include <gtest/gtest.h>

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

}  // namespace
