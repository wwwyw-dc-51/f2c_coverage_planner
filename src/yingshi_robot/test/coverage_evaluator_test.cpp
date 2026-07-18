#include <cmath>

#include <gtest/gtest.h>

#include "coverage_evaluator.hpp"

TEST(CoverageScoring, CoverageGateReachesFullAtPassThreshold)
{
    EXPECT_DOUBLE_EQ(coverage_scoring::coverageGate(0.90, 0.99), 0.0);
    EXPECT_NEAR(
        coverage_scoring::coverageGate(0.945, 0.99), 0.125, 1e-12);
    EXPECT_DOUBLE_EQ(coverage_scoring::coverageGate(0.99, 0.99), 1.0);
}

TEST(CoverageScoring, DistanceScoreHasRealFifteenPercentTolerance)
{
    EXPECT_DOUBLE_EQ(coverage_scoring::distanceScore(115.0, 100.0), 1.0);
    EXPECT_NEAR(
        coverage_scoring::distanceScore(215.0, 100.0),
        std::exp(-1.0), 1e-12);
}

TEST(CoverageScoring, OnlyTurnsAboveBoustrophedonBaselineArePenalized)
{
    EXPECT_DOUBLE_EQ(coverage_scoring::turnScore(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(coverage_scoring::turnScore(18, 19), 1.0);
    EXPECT_NEAR(
        coverage_scoring::turnScore(20, 19),
        std::exp(-3.0 * 2.0 / 18.0), 1e-12);
}

TEST(CoverageScoring, PlannedOverlapIsFreeAndPenaltyRemainsContinuous)
{
    const double planned_overlap = 0.03;
    const double planned_grid_rate = planned_overlap / (1.0 - planned_overlap);

    EXPECT_DOUBLE_EQ(
        coverage_scoring::overlapScore(
            planned_grid_rate, planned_overlap),
        1.0);
    EXPECT_NEAR(
        coverage_scoring::overlapScore(
            planned_grid_rate + 0.10, planned_overlap),
        std::exp(-0.20), 1e-12);
    EXPECT_GT(
        coverage_scoring::overlapScore(1.0, 0.0),
        coverage_scoring::overlapScore(1.01, 0.0));
}

TEST(CoverageScoring, WorkRatioPenalizesValuesAboveOneSymmetrically)
{
    EXPECT_DOUBLE_EQ(coverage_scoring::workRatioScore(0.8), 0.8);
    EXPECT_DOUBLE_EQ(coverage_scoring::workRatioScore(1.0), 1.0);
    EXPECT_DOUBLE_EQ(coverage_scoring::workRatioScore(1.25), 0.8);
}

TEST(CoverageScoring, TurnMergeDistanceTracksEffectiveSwathSpacing)
{
    EXPECT_DOUBLE_EQ(
        coverage_scoring::turnMergeDistance(0.4365, 0.75), 0.75);
    EXPECT_NEAR(
        coverage_scoring::turnMergeDistance(0.873, 0.75),
        0.91665, 1e-12);
}

TEST(CoverageScoring, ReportDescribesTheImplementedCoverageGate)
{
    EvalResult result;
    result.coverage_gate = 0.5;

    const std::string report = formatEvalReport(result, "scoring-test");

    EXPECT_NE(report.find("90%"), std::string::npos);
    EXPECT_EQ(report.find("^10"), std::string::npos);
}
