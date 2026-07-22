#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "yingshi_robot/physical_footprint.hpp"
#include "yingshi_robot/path_planner.hpp"

namespace {

f2c::types::Cell makeRectangle(
    double min_x, double min_y, double max_x, double max_y)
{
    f2c::types::LinearRing ring;
    ring.addPoint(f2c::types::Point(min_x, min_y));
    ring.addPoint(f2c::types::Point(max_x, min_y));
    ring.addPoint(f2c::types::Point(max_x, max_y));
    ring.addPoint(f2c::types::Point(min_x, max_y));
    ring.addPoint(f2c::types::Point(min_x, min_y));
    return f2c::types::Cell(ring);
}

TEST(PhysicalFootprint, UsesCleanerOuterWidthForBoundaryCollision)
{
    const auto field = makeRectangle(0.0, 0.0, 10.0, 10.0);
    yingshi::PhysicalFootprintParams params;
    params.safety_margin = 0.03;

    const auto safe = yingshi::checkFootprintAtPose(
        f2c::types::Point(5.0, 1.0), 0.0, field, {}, params);
    const auto colliding = yingshi::checkFootprintAtPose(
        f2c::types::Point(5.0, 0.45), 0.0, field, {}, params);

    EXPECT_FALSE(safe.collision);
    EXPECT_TRUE(colliding.collision);
    EXPECT_TRUE(colliding.outside_boundary);
}

TEST(PhysicalFootprint, ClearanceRadiusIncludesFrontCleanerEnvelope)
{
    const yingshi::PhysicalFootprintParams params;
    const double body_radius = std::hypot(
        0.5 * params.body_length, 0.5 * params.body_width);
    const double cleaner_radius = std::hypot(
        params.cleaner_center_forward,
        std::abs(params.cleaner_center_lateral) +
            0.5 * params.cleaner_diameter);

    EXPECT_NEAR(
        yingshi::physicalFootprintClearanceRadius(params),
        std::max(body_radius, cleaner_radius) + params.safety_margin,
        1e-12);
}

TEST(PhysicalFootprint, RepairsCollidingSwathEndpointsInward)
{
    const auto field = makeRectangle(0.0, 0.0, 10.0, 10.0);
    yingshi::PhysicalFootprintParams params;

    f2c::types::LineString line;
    line.addPoint(f2c::types::Point(0.0, 5.0));
    line.addPoint(f2c::types::Point(10.0, 5.0));
    f2c::types::Route route;
    route.addConnectedSwaths(
        f2c::types::MultiPoint(),
        f2c::types::Swaths {
            f2c::types::Swath(line, 0.90, 0),
        });

    EXPECT_EQ(
        yingshi::repairRouteSwathEndpointsForCollision(
            route, field, {}, params), 1U);

    const auto& repaired = route.getSwaths(0).at(0);
    EXPECT_GT(repaired.startPoint().getX(), 0.0);
    EXPECT_LT(repaired.endPoint().getX(), 10.0);
    EXPECT_FALSE(yingshi::checkFootprintAtPose(
        repaired.startPoint(), 0.0, field, {}, params).collision);
    EXPECT_FALSE(yingshi::checkFootprintAtPose(
        repaired.endPoint(), 0.0, field, {}, params).collision);
}

TEST(PhysicalFootprint, DetectsFrontCleanerAtObstacleCorner)
{
    const auto field = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto obstacle = makeRectangle(4.0, 3.0, 6.0, 8.0);
    yingshi::PhysicalFootprintParams params;
    params.safety_margin = 0.03;

    const auto result = yingshi::checkFootprintAtPose(
        f2c::types::Point(3.5, 2.55), 0.0, field,
        {obstacle.getExteriorRing()}, params);

    EXPECT_TRUE(result.collision);
    EXPECT_TRUE(result.intersects_obstacle);
}

TEST(PhysicalFootprint, SamplesWholePathInsteadOfOnlyEndpoints)
{
    const auto field = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto obstacle = makeRectangle(4.0, 4.0, 6.0, 6.0);
    f2c::types::Path path;
    f2c::types::PathState state;
    state.point = f2c::types::Point(2.0, 5.0);
    state.angle = 0.0;
    state.len = 6.0;
    path.addState(state);

    const auto result = yingshi::checkPathFootprintCollision(
        path, field, {obstacle.getExteriorRing()});

    EXPECT_TRUE(result.collision);
    EXPECT_TRUE(result.intersects_obstacle);
    EXPECT_GT(result.checked_poses, 2U);
}

TEST(PhysicalFootprint, ChecksDiscontinuousConnectionBetweenStates)
{
    const auto field = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto obstacle = makeRectangle(4.0, 4.0, 6.0, 6.0);
    f2c::types::Path path;

    f2c::types::PathState first;
    first.point = f2c::types::Point(2.0, 5.0);
    first.angle = 0.0;
    first.len = 1.0;
    path.addState(first);

    f2c::types::PathState second;
    second.point = f2c::types::Point(8.0, 5.0);
    second.angle = 0.0;
    second.len = 0.0;
    path.addState(second);

    const auto result = yingshi::checkPathFootprintCollision(
        path, field, {obstacle.getExteriorRing()});

    EXPECT_TRUE(result.collision);
    EXPECT_TRUE(result.intersects_obstacle);
}

TEST(PhysicalFootprint, FollowsBackwardPathDirection)
{
    const auto field = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto obstacle = makeRectangle(1.0, 4.0, 3.0, 6.0);
    f2c::types::Path path;

    f2c::types::PathState state;
    state.point = f2c::types::Point(5.0, 5.0);
    state.angle = 0.0;
    state.len = 4.0;
    state.dir = f2c::types::PathDirection::BACKWARD;
    path.addState(state);

    const auto result = yingshi::checkPathFootprintCollision(
        path, field, {obstacle.getExteriorRing()});

    EXPECT_TRUE(result.collision);
    EXPECT_TRUE(result.intersects_obstacle);
}

}  // namespace
