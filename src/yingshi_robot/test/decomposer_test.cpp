#include <gtest/gtest.h>

#include <algorithm>

#include "yingshi_robot/decomposer.hpp"

namespace {

f2c::types::LinearRing makeRectangleRing(
    double min_x, double min_y, double max_x, double max_y)
{
    f2c::types::LinearRing ring;
    ring.addPoint(f2c::types::Point(min_x, min_y));
    ring.addPoint(f2c::types::Point(max_x, min_y));
    ring.addPoint(f2c::types::Point(max_x, max_y));
    ring.addPoint(f2c::types::Point(min_x, max_y));
    ring.addPoint(f2c::types::Point(min_x, min_y));
    return ring;
}

TEST(Decomposer, SweepUsesHoleVerticesAsBothXAndYCuts)
{
    f2c::types::Cell work_area;
    work_area.addRing(makeRectangleRing(0.0, 0.0, 10.0, 10.0));
    work_area.addRing(makeRectangleRing(4.0, 4.0, 6.0, 6.0));

    yingshi::DecomposerParams params;
    params.use_sweep = true;

    const auto cells = yingshi::rectilinearDecompose(work_area, work_area, params);

    ASSERT_EQ(cells.size(), 8U);
    for (size_t ci = 0; ci < cells.size(); ++ci) {
        const auto& ring = cells.getGeometry(ci).getExteriorRing();
        double min_x = ring.getGeometry(0).getX();
        double max_x = min_x;
        for (size_t pi = 1; pi < ring.size(); ++pi) {
            min_x = std::min(min_x, ring.getGeometry(pi).getX());
            max_x = std::max(max_x, ring.getGeometry(pi).getX());
        }
        EXPECT_LE(max_x - min_x, 4.0 + 1e-6);
    }
}

}  // namespace
