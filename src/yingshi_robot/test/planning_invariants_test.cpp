#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "yingshi_robot/boundary_filler.hpp"
#include "yingshi_robot/path_geometry.hpp"
#include "yingshi_robot/path_planner.hpp"
#include "yingshi_robot/path_sanity_check.hpp"
#include "yingshi_robot/swath_generator.hpp"

namespace {

f2c::types::Swath makeSwath(
    double x0, double y0, double x1, double y1,
    double width = 0.45, int id = 0)
{
    f2c::types::LineString line;
    line.addPoint(f2c::types::Point(x0, y0));
    line.addPoint(f2c::types::Point(x1, y1));
    return f2c::types::Swath(line, width, id);
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

TEST(BoundaryFill, HonorsAnExplicitPreclearedBoundaryOffset)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto bottom_cell = makeRectangle(0.0, 0.0, 10.0, 1.2);
    f2c::types::Swaths swaths;
    swaths.push_back(makeSwath(0.15, 0.8, 9.85, 0.8, 0.90));

    yingshi::fillBoundaryGaps(
        swaths, bottom_cell, full_polygon, 0.0, 0.90, 0.0,
        0.0, 0.10);

    bool found_precleared_fill = false;
    for (std::size_t i = 0; i < swaths.size(); ++i) {
        if (std::abs(swathMidY(swaths.at(i)) - 0.10) < 1e-9) {
            found_precleared_fill = true;
            break;
        }
    }
    EXPECT_TRUE(found_precleared_fill);
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

TEST(BoundaryFill, CapsOuterFillAtTheCurrentCellReachableExtent)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto inset_bottom_cell = makeRectangle(0.15, 0.15, 9.85, 1.2);
    f2c::types::Swaths swaths;
    swaths.push_back(makeSwath(0.15, 0.8, 9.85, 0.8, 0.90));

    yingshi::fillBoundaryGaps(
        swaths, inset_bottom_cell, full_polygon, 0.0, 0.90, 0.0);

    bool found_bottom_boundary_fill = false;
    for (size_t i = 0; i < swaths.size(); ++i) {
        const auto& swath = swaths.at(i);
        if (std::abs(swathMidY(swath) - 0.45) > 1e-9 ||
            std::abs(
                swath.startPoint().getY() -
                swath.endPoint().getY()) > 1e-9) {
            continue;
        }
        found_bottom_boundary_fill = true;
        EXPECT_NEAR(std::min(
            swath.startPoint().getX(), swath.endPoint().getX()), 0.15, 1e-9);
        EXPECT_NEAR(std::max(
            swath.startPoint().getX(), swath.endPoint().getX()), 9.85, 1e-9);
    }
    EXPECT_TRUE(found_bottom_boundary_fill);
}

TEST(BoundaryFill, KeepsFillOutsideHeadlandCellInTheNormalDirection)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto inset_bottom_cell = makeRectangle(0.15, 0.60, 9.85, 1.2);
    f2c::types::Swaths swaths;
    swaths.push_back(makeSwath(0.15, 0.8, 9.85, 0.8, 0.90));

    yingshi::fillBoundaryGaps(
        swaths, inset_bottom_cell, full_polygon, 0.0, 0.90, 0.0);

    bool found_bottom_boundary_fill = false;
    for (size_t i = 0; i < swaths.size(); ++i) {
        const auto& swath = swaths.at(i);
        if (std::abs(swathMidY(swath) - 0.45) > 1e-9) continue;
        found_bottom_boundary_fill = true;
        EXPECT_NEAR(std::min(
            swath.startPoint().getX(), swath.endPoint().getX()), 0.15, 1e-9);
        EXPECT_NEAR(std::max(
            swath.startPoint().getX(), swath.endPoint().getX()), 9.85, 1e-9);
    }
    EXPECT_TRUE(found_bottom_boundary_fill);
}

TEST(BoundaryEndpointAdjustment, DoesNotLetHoleProximityAtMidpointShrinkSafeEndpoints)
{
    const auto outer = makeRectangle(0.0, 0.0, 25.0, 25.0);
    const std::vector<f2c::types::LinearRing> holes {
        yingshi::makeClosedRing({
            f2c::types::Point(9.0, 13.5),
            f2c::types::Point(19.0, 13.5),
            f2c::types::Point(19.0, 23.5),
            f2c::types::Point(9.0, 23.5),
        })
    };
    const auto swath = makeSwath(24.85, 24.0865, 0.15, 24.0865, 0.90);

    const auto adjusted = yingshi::adjustSwathEndpointsForBoundaryClearance(
        swath, outer.getExteriorRing(), holes, 0.90, 0.03);

    EXPECT_NEAR(adjusted.startPoint().getX(), 24.85, 1e-9);
    EXPECT_NEAR(adjusted.endPoint().getX(), 0.15, 1e-9);
}

TEST(BoundaryPolicy, ResolvesClosedOpenAndCustomMarginsConsistently)
{
    EXPECT_DOUBLE_EQ(
        yingshi::resolveBoundaryMargin("closed", 0.03, -0.2, -0.4),
        0.03);
    EXPECT_DOUBLE_EQ(
        yingshi::resolveBoundaryMargin("closed", 0.0, -0.2, -0.4),
        0.3);
    EXPECT_DOUBLE_EQ(
        yingshi::resolveBoundaryMargin("open", 0.03, 0.0, -0.4),
        -0.4);
    EXPECT_DOUBLE_EQ(
        yingshi::resolveBoundaryMargin("open", 0.03, -0.2, -0.4),
        -0.2);
    EXPECT_DOUBLE_EQ(
        yingshi::resolveBoundaryMargin("custom", 0.03, 0.12, -0.4),
        0.12);
}

TEST(BoundaryEndpointAdjustment, ShrinksOnlyTheEndpointThatIsNearAHole)
{
    const auto outer = makeRectangle(0.0, 0.0, 25.0, 25.0);
    const std::vector<f2c::types::LinearRing> holes {
        yingshi::makeClosedRing({
            f2c::types::Point(9.0, 13.5),
            f2c::types::Point(19.0, 13.5),
            f2c::types::Point(19.0, 23.5),
            f2c::types::Point(9.0, 23.5),
        })
    };
    const auto swath = makeSwath(19.15, 20.0, 24.85, 20.0, 0.90);

    const auto adjusted = yingshi::adjustSwathEndpointsForBoundaryClearance(
        swath, outer.getExteriorRing(), holes, 0.90, 0.03);

    EXPECT_NEAR(adjusted.startPoint().getX(), 19.21, 1e-9);
    EXPECT_NEAR(adjusted.endPoint().getX(), 24.85, 1e-9);
}

TEST(CellOrder, PreservesBoundaryFillAtExplicitCellEnd)
{
    f2c::types::Swaths cell;
    cell.push_back(makeSwath(18.38, 5.423, 24.79, 5.423, 0.90, 40));
    cell.push_back(makeSwath(24.79, 4.550, 0.21, 4.550, 0.90, 41));
    cell.push_back(makeSwath(0.15, 3.677, 24.85, 3.677, 0.90, 42));
    cell.push_back(makeSwath(24.85, 2.804, 0.15, 2.804, 0.90, 43));
    cell.push_back(makeSwath(0.15, 1.931, 24.85, 1.931, 0.90, 44));
    cell.push_back(makeSwath(24.85, 1.058, 0.15, 1.058, 0.90, 46));
    cell.push_back(makeSwath(0.21, 5.423, 11.62, 5.423, 0.90, 47));
    // 补线生成时已明确追加在末尾，但默认 ID=0 曾让 F2C 再排序时把它挪走。
    cell.push_back(makeSwath(0.15, 0.185, 24.85, 0.185, 0.90, 0));
    f2c::types::SwathsByCells cells {cell};
    std::vector<size_t> order;

    yingshi::greedyCellOrder(cells, order, {}, "boustrophedon");

    ASSERT_EQ(cells.size(), 1U);
    ASSERT_EQ(cells.at(0).size(), cell.size());
    EXPECT_NEAR(swathMidY(cells.at(0).back()), 0.185, 1e-9);
}

TEST(CellOrder, UsesPreviousCellExitToChooseNextCellEntryVariant)
{
    f2c::types::Swaths first_cell;
    first_cell.push_back(makeSwath(0.0, 0.0, 10.0, 0.0, 0.90, 0));

    f2c::types::Swaths second_cell;
    second_cell.push_back(makeSwath(0.0, 1.0, 10.0, 1.0, 0.90, 0));
    second_cell.push_back(makeSwath(0.0, 2.0, 10.0, 2.0, 0.90, 1));
    f2c::types::SwathsByCells cells {first_cell, second_cell};
    std::vector<size_t> order;

    yingshi::greedyCellOrder(cells, order, {}, "boustrophedon");

    ASSERT_EQ(cells.size(), 2U);
    ASSERT_EQ(cells.at(1).size(), 2U);
    EXPECT_NEAR(cells.at(1).at(0).startPoint().getX(), 10.0, 1e-9);
    EXPECT_NEAR(cells.at(1).at(0).startPoint().getY(), 1.0, 1e-9);
    EXPECT_NEAR(
        cells.at(0).back().endPoint().distance(
            cells.at(1).at(0).startPoint()),
        1.0, 1e-9);
}

TEST(CellOrder, OptimizesHoleOrderedEntryVariantsAcrossTheWholeChain)
{
    f2c::types::Swaths first_cell;
    first_cell.push_back(makeSwath(0.0, 0.0, 4.0, 0.0, 0.90, 0));

    f2c::types::Swaths second_cell;
    second_cell.push_back(makeSwath(0.0, 0.0, 10.0, 0.0, 0.90, 0));

    f2c::types::Swaths third_cell;
    third_cell.push_back(makeSwath(0.0, 1.0, 0.0, 2.0, 0.90, 0));

    f2c::types::SwathsByCells cells {
        first_cell, second_cell, third_cell};
    std::vector<size_t> order;
    const std::vector<f2c::types::LinearRing> holes {
        yingshi::makeClosedRing({
            f2c::types::Point(-1.1, -0.1),
            f2c::types::Point(-0.9, -0.1),
            f2c::types::Point(-0.9, 0.1),
            f2c::types::Point(-1.1, 0.1),
        })
    };

    yingshi::greedyCellOrder(cells, order, holes, "boustrophedon");

    ASSERT_EQ(cells.size(), 3U);
    ASSERT_EQ(order, (std::vector<size_t> {0U, 1U, 2U}));
    EXPECT_NEAR(cells.at(1).at(0).startPoint().getX(), 10.0, 1e-9);

    double connection_sum = 0.0;
    for (size_t i = 1; i < cells.size(); ++i) {
        connection_sum += cells.at(i - 1).back().endPoint().distance(
            cells.at(i).at(0).startPoint());
    }
    EXPECT_LT(connection_sum, 8.0);
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

TEST(BoundaryFill, UsesCoverageWidthRatherThanRowSpacingForBoundaryOffset)
{
    const auto full_polygon = makeRectangle(0.0, 0.0, 2.0, 20.0);
    f2c::types::Cells cells;
    cells.addGeometry(full_polygon);

    const auto generated = yingshi::generateSwathsForAllCells(
        cells,
        full_polygon,
        0.873,
        0.90,
        0.0,
        0.5,
        true,
        {0.5 * std::acos(-1.0)});

    // 左侧：首条主 swath 中心在 r_w/2=0.4365，半覆盖 0.45 已盖到 x=0，
    // fillBoundaryGaps 不会生成 x=0.45 补线 → 只断言确实没有旧偏移位置的补线。
    // 右侧：末条主 swath 在 1.3095，半覆盖仅到 1.7595，需要补线。
    //   正确实现 → 2.0 - 0.90/2 = 1.55
    //   旧错误实现 → 2.0 - 0.873/2 = 1.5635
    bool found_right_at_correct_offset = false;   // x ≈ 1.55
    bool found_right_at_old_wrong_offset = false; // x ≈ 1.5635
    for (size_t cell_index = 0; cell_index < generated.size(); ++cell_index) {
        const auto& swaths = generated.at(cell_index);
        for (size_t swath_index = 0; swath_index < swaths.size(); ++swath_index) {
            const auto& swath = swaths.at(swath_index);
            const double mid_x = 0.5 * (
                swath.startPoint().getX() + swath.endPoint().getX());
            found_right_at_correct_offset |= std::abs(mid_x - 1.55) < 1e-6;
            found_right_at_old_wrong_offset |=
                std::abs(mid_x - 1.5635) < 1e-6;
        }
    }

    EXPECT_TRUE(found_right_at_correct_offset)
        << "Right boundary fill should use coverage_width (center at 1.55)";
    EXPECT_FALSE(found_right_at_old_wrong_offset)
        << "Old row-spacing offset (1.5635) must not appear";
}

TEST(SwathGeneration, FailsClosedWhenAnyRetainedCellHasNoValidSwath)
{
    f2c::types::Cells cells;
    cells.addGeometry(makeRectangle(0.0, 0.0, 10.0, 10.0));
    cells.addGeometry(makeRectangle(20.0, 0.0, 20.1, 0.1));
    const auto full_polygon = makeRectangle(0.0, 0.0, 21.0, 10.0);

    const auto generated = yingshi::generateSwathsForAllCells(
        cells, full_polygon, 0.873, 0.90, 0.0, 0.5, true, {});

    EXPECT_EQ(generated.sizeTotal(), 0U);
    EXPECT_EQ(generated.size(), 0U);
}

TEST(BoundaryFill, RebalancesNearCoincidentSwathsInASingleNarrowRectangle)
{
    const auto corridor = makeRectangle(0.15, 0.15, 1.85, 19.85);
    f2c::types::Swaths swaths;
    swaths.push_back(makeSwath(1.55, 19.85, 1.55, 0.15, 0.90));
    swaths.push_back(makeSwath(1.46, 0.15, 1.46, 19.85, 0.90));
    swaths.push_back(makeSwath(0.587, 19.85, 0.587, 0.15, 0.90));
    swaths.push_back(makeSwath(0.45, 0.15, 0.45, 19.85, 0.90));

    EXPECT_EQ(
        yingshi::rebalanceNarrowCellSwaths(swaths, corridor, 0.90), 1U);

    ASSERT_EQ(swaths.size(), 3U);
    EXPECT_NEAR(swaths.at(0).startPoint().getX(), 1.55, 1e-9);
    EXPECT_NEAR(swaths.at(1).startPoint().getX(), 1.00, 1e-9);
    EXPECT_NEAR(swaths.at(2).startPoint().getX(), 0.45, 1e-9);
    EXPECT_GT(swaths.at(0).startPoint().getY(), swaths.at(0).endPoint().getY());
    EXPECT_LT(swaths.at(1).startPoint().getY(), swaths.at(1).endPoint().getY());
    EXPECT_GT(swaths.at(2).startPoint().getY(), swaths.at(2).endPoint().getY());
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

TEST(HoleGeometry, DetectsANarrowHoleWithoutDependingOnSampleSpacing)
{
    const auto narrow_hole = yingshi::makeClosedRing({
        f2c::types::Point(55.1, -0.1),
        f2c::types::Point(55.3, -0.1),
        f2c::types::Point(55.3, 0.1),
        f2c::types::Point(55.1, 0.1),
    });

    EXPECT_TRUE(yingshi::segmentCrossesHole(
        0.0, 0.0, 100.0, 0.0, {narrow_hole}, 10));
}

TEST(HoleGeometry, DoesNotTreatBoundaryContactAsEnteringAHole)
{
    const auto hole = yingshi::makeClosedRing({
        f2c::types::Point(5.0, 5.0),
        f2c::types::Point(6.0, 5.0),
        f2c::types::Point(6.0, 6.0),
        f2c::types::Point(5.0, 6.0),
    });
    const std::vector<f2c::types::LinearRing> holes {hole};

    EXPECT_FALSE(yingshi::pointInAnyHole(5.0, 5.5, holes));
    EXPECT_FALSE(yingshi::segmentCrossesHole(
        4.0, 5.0, 7.0, 5.0, holes, 10));
    EXPECT_FALSE(yingshi::segmentCrossesHole(
        4.0, 6.0, 6.0, 4.0, holes, 10));
    EXPECT_FALSE(yingshi::segmentCrossesHole(
        4.0, 5.5, 5.0, 5.5, holes, 10));
}

TEST(HoleGeometry, DetectsAShortSegmentThatCrossesAHoleBoundary)
{
    const auto hole = yingshi::makeClosedRing({
        f2c::types::Point(0.0, 0.0),
        f2c::types::Point(1.0, 0.0),
        f2c::types::Point(1.0, 1.0),
        f2c::types::Point(0.0, 1.0),
    });

    EXPECT_TRUE(yingshi::segmentCrossesHole(
        -1e-6, 0.5, 1e-6, 0.5, {hole}, 10));
}

TEST(PathSanity, TreatsHoleCrossingAsAPublishBlockingError)
{
    auto polygon = makeRectangle(0.0, 0.0, 10.0, 10.0);
    const auto hole = yingshi::makeClosedRing({
        f2c::types::Point(4.0, 4.0),
        f2c::types::Point(6.0, 4.0),
        f2c::types::Point(6.0, 6.0),
        f2c::types::Point(4.0, 6.0),
    });
    polygon.addRing(hole);
    const std::vector<f2c::types::Point> crossing_path {
        f2c::types::Point(1.0, 5.0),
        f2c::types::Point(9.0, 5.0),
    };

    const auto sanity = yingshi::checkPathSanity(
        f2c::types::Path(), crossing_path, polygon, {hole}, 1U);

    ASSERT_FALSE(sanity.passed);
    ASSERT_FALSE(sanity.issues.empty());
    EXPECT_TRUE(std::any_of(
        sanity.issues.begin(), sanity.issues.end(),
        [](const yingshi::SanityIssue& issue) {
            return issue.severity ==
                yingshi::SanityIssue::Severity::ERROR;
        }));
}

}  // namespace
