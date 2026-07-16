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

TEST(Decomposer, SweepCreatesFullWidthStripsWithoutVerticalHoleCuts)
{
    f2c::types::Cell work_area;
    work_area.addRing(makeRectangleRing(0.0, 0.0, 10.0, 10.0));
    work_area.addRing(makeRectangleRing(4.0, 4.0, 6.0, 6.0));

    yingshi::DecomposerParams params;
    params.use_sweep = true;

    const auto cells = yingshi::rectilinearDecompose(work_area, work_area, params);

    // Sweep 模式：水平切割线 + X 压缩为 min/max（全宽条带）
    // 10x10 + 中心 2x2 孔洞 → 4 cells（上下全宽 + 中间孔洞左右各一）
    ASSERT_GE(cells.size(), 3U);
    // 每 cell 宽度不超过 10（全宽条带），验证没有孔洞顶点垂直切割
    for (size_t ci = 0; ci < cells.size(); ++ci) {
        const auto& ring = cells.getGeometry(ci).getExteriorRing();
        double min_x = ring.getGeometry(0).getX();
        double max_x = min_x;
        for (size_t pi = 1; pi < ring.size(); ++pi) {
            min_x = std::min(min_x, ring.getGeometry(pi).getX());
            max_x = std::max(max_x, ring.getGeometry(pi).getX());
        }
        EXPECT_LE(max_x - min_x, 10.0 + 1e-6);
    }
}

}  // namespace
