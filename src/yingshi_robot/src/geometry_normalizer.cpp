#include "yingshi_robot/geometry_normalizer.hpp"

#include <algorithm>
#include <cmath>

namespace yingshi {

namespace {

constexpr double kPointTolerance = 1e-12;

f2c::types::Point snapPoint(
    const f2c::types::Point& point,
    double snap_tolerance)
{
    if (snap_tolerance <= 0.0 || !std::isfinite(snap_tolerance)) {
        return point;
    }
    return f2c::types::Point(
        std::round(point.getX() / snap_tolerance) * snap_tolerance,
        std::round(point.getY() / snap_tolerance) * snap_tolerance);
}

double signedArea(const std::vector<f2c::types::Point>& points)
{
    if (points.size() < 3) return 0.0;

    double area = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& current = points[i];
        const auto& next = points[(i + 1) % points.size()];
        area += current.getX() * next.getY() -
            next.getX() * current.getY();
    }
    return 0.5 * area;
}

}  // namespace

f2c::types::LinearRing normalizeRing(
    const f2c::types::LinearRing& ring,
    bool counter_clockwise,
    double snap_tolerance)
{
    std::vector<f2c::types::Point> points;
    points.reserve(ring.size());
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const auto point = snapPoint(ring.getGeometry(i), snap_tolerance);
        if (points.empty() || points.back().distance(point) > kPointTolerance) {
            points.push_back(point);
        }
    }

    if (points.size() > 1 &&
        points.front().distance(points.back()) <= kPointTolerance) {
        points.pop_back();
    }

    if (points.size() >= 3) {
        const double area = signedArea(points);
        const bool should_reverse =
            (counter_clockwise && area < 0.0) ||
            (!counter_clockwise && area > 0.0);
        if (should_reverse) {
            // 不用 std::reverse — f2c::types::Point 的 move assignment 在某些
            // F2C 版本中存在 bug（swap 时访问空 this → segfault）。
            // 通过反向拷贝 + vector move 绕过 Point 的 operator=(&&)。
            std::vector<f2c::types::Point> reversed;
            reversed.reserve(points.size());
            for (auto it = points.rbegin(); it != points.rend(); ++it) {
                reversed.push_back(*it);
            }
            points = std::move(reversed);
        }
    }

    f2c::types::LinearRing normalized;
    for (const auto& point : points) normalized.addPoint(point);
    if (!points.empty()) normalized.addPoint(points.front());
    return normalized;
}

f2c::types::Cell normalizeCell(
    const f2c::types::Cell& cell,
    double snap_tolerance)
{
    f2c::types::Cell normalized;
    if (cell.size() == 0) return normalized;

    const auto exterior = normalizeRing(
        cell.getExteriorRing(), true, snap_tolerance);
    if (exterior.size() < 4) return normalized;
    normalized.addRing(exterior);

    for (std::size_t i = 0; i + 1 < cell.size(); ++i) {
        const auto interior = normalizeRing(
            cell.getInteriorRing(i), false, snap_tolerance);
        if (interior.size() >= 4) normalized.addRing(interior);
    }
    return normalized;
}

f2c::types::Cells normalizeCells(
    const f2c::types::Cells& cells,
    double snap_tolerance)
{
    f2c::types::Cells normalized;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const auto cell = normalizeCell(
            cells.getGeometry(i), snap_tolerance);
        if (cell.size() > 0) normalized.addGeometry(cell);
    }
    return normalized;
}

std::vector<f2c::types::LinearRing> normalizeRings(
    const std::vector<f2c::types::LinearRing>& rings,
    double snap_tolerance)
{
    std::vector<f2c::types::LinearRing> normalized;
    normalized.reserve(rings.size());
    for (const auto& ring : rings) {
        const auto normalized_ring = normalizeRing(
            ring, false, snap_tolerance);
        if (normalized_ring.size() >= 4) {
            normalized.push_back(normalized_ring);
        }
    }
    return normalized;
}

}  // namespace yingshi
