#include <gtest/gtest.h>

#include <cmath>

#include "coverage_evaluator.hpp"
#include "yingshi_robot/path_geometry.hpp"

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

    f2c::types::Cell cell;
    cell.addRing(ring);
    return cell;
}

TEST(PathGeometry, MaterializesTheFinalPathStateEndpoint)
{
    f2c::types::Path path;
    path.addState(
        f2c::types::Point(0.0, 0.0), 0.0, 3.0,
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);
    path.addState(
        f2c::types::Point(3.0, 0.0), std::acos(-1.0) / 2.0, 4.0,
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);

    const auto points = yingshi::materializePath(path);
    const auto waypoints = yingshi::materializePathWaypoints(path);

    ASSERT_EQ(points.size(), 3U);
    EXPECT_EQ(waypoints.size(), 3U);
    EXPECT_DOUBLE_EQ(points.back().getX(), 3.0);
    EXPECT_DOUBLE_EQ(points.back().getY(), 4.0);
    EXPECT_DOUBLE_EQ(yingshi::polylineLength(points), 7.0);
}

TEST(PathGeometry, GridCoverageIncludesASingleFinalState)
{
    f2c::types::Path path;
    path.addState(
        f2c::types::Point(0.0, 0.0), 0.0, 10.0,
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);

    f2c::types::Cells target;
    target.addGeometry(makeRectangle(0.0, -0.2, 10.0, 0.2));

    EvalParams params;
    params.coverage_width = 0.5;
    params.grid_resolution = 0.1;
    params.use_grid_method = true;

    const auto result = evaluatePlan(path, {}, target, {}, 0.0, params);

    EXPECT_GT(result.coverage_rate, 0.99);
    EXPECT_NEAR(result.total_distance, 10.0, 1e-9);
}

TEST(PathGeometry, PreservesHeadingForBackwardPathWaypoints)
{
    f2c::types::Path path;
    path.addState(
        f2c::types::Point(5.0, 0.0), 0.0, 2.0,
        f2c::types::PathDirection::BACKWARD,
        f2c::types::PathSectionType::TURN);

    const auto waypoints = yingshi::materializePathWaypoints(path);

    ASSERT_EQ(waypoints.size(), 2U);
    EXPECT_DOUBLE_EQ(waypoints.back().point.getX(), 3.0);
    EXPECT_DOUBLE_EQ(waypoints.back().angle, 0.0);
}

TEST(PathGeometry, PreservesEveryStateEndpointAcrossDiscontinuities)
{
    f2c::types::Path path;
    path.addState(
        f2c::types::Point(0.0, 0.0), 0.0, 2.0,
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);
    path.addState(
        f2c::types::Point(10.0, 10.0), std::acos(-1.0) / 2.0, 1.0,
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);

    const auto points = yingshi::materializePath(path);

    ASSERT_EQ(points.size(), 4U);
    EXPECT_DOUBLE_EQ(points[1].getX(), 2.0);
    EXPECT_DOUBLE_EQ(points[1].getY(), 0.0);
    EXPECT_DOUBLE_EQ(points.back().getX(), 10.0);
    EXPECT_DOUBLE_EQ(points.back().getY(), 11.0);
}

// 不变量测试：materializePath 是所有路径消费者的唯一数据源。
// 发布路径、评估路径、渲染路径全部走此函数，确保三者一致。
TEST(PathGeometry, LengthMatchesSumOfStateLengths)
{
    f2c::types::Path path;
    path.addState(
        f2c::types::Point(0.0, 0.0), 0.0, 3.0,
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);
    path.addState(
        f2c::types::Point(3.0, 0.0), std::acos(-1.0) / 2.0, 4.0,
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);
    path.addState(
        f2c::types::Point(5.0, 5.0), 0.0, 2.0,  // discontinuous gap
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);

    const auto points = yingshi::materializePath(path);
    const double total_len = yingshi::polylineLength(points);

    // 期望: 3 + 4 + 2 = 9（不含 gap，gap 由 consumer 自行连接）
    EXPECT_DOUBLE_EQ(total_len, 9.0);
}

// 不变量测试：连续路径段之间无间隙，总长=各段长之和
TEST(PathGeometry, ContinuousPathLengthEqualsSumOfStates)
{
    f2c::types::Path path;
    path.addState(
        f2c::types::Point(0.0, 0.0), 0.0, 5.0,
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);
    path.addState(
        f2c::types::Point(5.0, 0.0), 0.0, 3.0,  // continuous — point == previous atEnd
        f2c::types::PathDirection::FORWARD,
        f2c::types::PathSectionType::SWATH);

    const auto points = yingshi::materializePath(path);
    const double total_len = yingshi::polylineLength(points);

    // 连续段: 第一段 (0→5) + 第二段 (5→8) = 8
    EXPECT_DOUBLE_EQ(total_len, 8.0);
}

}  // namespace
