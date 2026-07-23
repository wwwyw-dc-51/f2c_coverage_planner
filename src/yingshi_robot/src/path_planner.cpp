/**
 * @file path_planner.cpp
 * @brief 路径规划模块实现 — RDP 简化、不可执行掉头检测、连接修复
 *
 * 从 polygon_planner_node.cpp 提取，属于模块化重构 Step 5。
 */

#include "yingshi_robot/path_planner.hpp"
#include "yingshi_robot/boundary_filler.hpp"  // segmentCrossesHole, pointInAnyHole
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace yingshi {

void appendConnectedSwath(
    f2c::types::Route& route,
    const f2c::types::MultiPoint& connection,
    const f2c::types::Swath& swath)
{
    f2c::types::Swaths group;
    group.push_back(swath);
    route.addConnectedSwaths(connection, group);
}

namespace {

void appendSegment(
    f2c::types::Path& path,
    const f2c::types::Point& start,
    const f2c::types::Point& end,
    f2c::types::PathSectionType type,
    double velocity)
{
    const double length = start.distance(end);
    if (length <= 1e-9) return;

    const double angle = std::atan2(
        end.getY() - start.getY(), end.getX() - start.getX());
    path.addState(
        start, angle, length,
        f2c::types::PathDirection::FORWARD, type, velocity);
}

void appendDistinct(
    std::vector<f2c::types::Point>& points,
    const f2c::types::Point& point)
{
    if (points.empty() || points.back().distance(point) > 1e-9) {
        points.push_back(point);
    }
}

void appendConnectionPolyline(
    f2c::types::Path& path,
    const std::vector<f2c::types::Point>& points,
    double velocity)
{
    for (size_t i = 1; i < points.size(); ++i) {
        appendSegment(
            path, points[i - 1], points[i],
            f2c::types::PathSectionType::TURN, velocity);
    }
}

struct HoleIntersection {
    double t;
    size_t hole_idx;
    size_t edge_idx;
    // 交点集合会排序/去重；只存标量，避免 F2C Point 的移动赋值在
    // std::sort/std::unique 搬移已移动对象时访问空 data_。
    double x;
    double y;
};

std::vector<HoleIntersection> findHoleIntersections(
    const f2c::types::Point& start,
    const f2c::types::Point& end,
    const std::vector<f2c::types::LinearRing>& hole_rings)
{
    std::vector<HoleIntersection> intersections;
    const double sx = start.getX();
    const double sy = start.getY();
    const double dx = end.getX() - sx;
    const double dy = end.getY() - sy;

    if (std::hypot(dx, dy) < 1e-12) return intersections;

    for (size_t hole_idx = 0; hole_idx < hole_rings.size(); ++hole_idx) {
        const auto& ring = hole_rings[hole_idx];
        for (size_t edge_idx = 0; edge_idx + 1 < ring.size(); ++edge_idx) {
            const double ax = ring.getGeometry(edge_idx).getX();
            const double ay = ring.getGeometry(edge_idx).getY();
            const double bx = ring.getGeometry(edge_idx + 1).getX();
            const double by = ring.getGeometry(edge_idx + 1).getY();
            const double edge_dx = bx - ax;
            const double edge_dy = by - ay;
            const double denominator = dx * edge_dy - dy * edge_dx;
            if (std::abs(denominator) < 1e-12) continue;

            const double t = ((ax - sx) * edge_dy - (ay - sy) * edge_dx) /
                denominator;
            const double u = ((ax - sx) * dy - (ay - sy) * dx) /
                denominator;
            if (t >= -1e-8 && t <= 1.0 + 1e-8 &&
                u >= -1e-8 && u <= 1.0 + 1e-8) {
                const double clamped_t = std::max(0.0, std::min(1.0, t));
                intersections.push_back({
                    clamped_t, hole_idx, edge_idx,
                    sx + clamped_t * dx, sy + clamped_t * dy});
            }
        }
    }

    std::sort(
        intersections.begin(), intersections.end(),
        [](const HoleIntersection& lhs, const HoleIntersection& rhs) {
            return lhs.t < rhs.t;
        });
    intersections.erase(
        std::unique(
            intersections.begin(), intersections.end(),
            [](const HoleIntersection& lhs, const HoleIntersection& rhs) {
                return lhs.hole_idx == rhs.hole_idx &&
                    std::abs(lhs.t - rhs.t) < 1e-8;
            }),
        intersections.end());
    return intersections;
}

double connectionPolylineLength(const std::vector<f2c::types::Point>& points)
{
    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += points[i - 1].distance(points[i]);
    }
    return length;
}

std::vector<f2c::types::Point> walkHoleBoundary(
    const HoleIntersection& entry,
    const HoleIntersection& exit,
    const std::vector<f2c::types::LinearRing>& hole_rings)
{
    if (entry.hole_idx != exit.hole_idx) return {};

    const auto& ring = hole_rings[entry.hole_idx];
    const size_t edge_count = ring.size() > 0 ? ring.size() - 1 : 0;
    if (edge_count == 0) return {};
    const f2c::types::Point entry_point(entry.x, entry.y);
    const f2c::types::Point exit_point(exit.x, exit.y);
    if (entry.edge_idx == exit.edge_idx) {
        return {entry_point, exit_point};
    }

    auto addVertex = [&](std::vector<f2c::types::Point>& path, size_t index) {
        path.emplace_back(
            ring.getGeometry(index).getX(), ring.getGeometry(index).getY());
    };

    std::vector<f2c::types::Point> forward {entry_point};
    size_t index = (entry.edge_idx + 1) % edge_count;
    while (true) {
        addVertex(forward, index);
        if (index == exit.edge_idx) break;
        index = (index + 1) % edge_count;
    }
    forward.push_back(exit_point);

    std::vector<f2c::types::Point> backward {entry_point};
    index = entry.edge_idx;
    while (true) {
        addVertex(backward, index);
        if (index == (exit.edge_idx + 1) % edge_count) break;
        index = (index + edge_count - 1) % edge_count;
    }
    backward.push_back(exit_point);

    return connectionPolylineLength(forward) <=
        connectionPolylineLength(backward) ? forward : backward;
}

std::vector<f2c::types::LinearRing> makeAvoidanceRings(
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double clearance)
{
    f2c::types::Cells avoidance_cells;
    for (const auto& ring : hole_rings) {
        try {
            const f2c::types::Cell hole_cell(ring);
            const auto avoidance_cell = clearance > 1e-9 ?
                f2c::types::Cell::buffer(hole_cell, clearance) : hole_cell;
            if (avoidance_cell.getExteriorRing().size() >= 4) {
                avoidance_cells.addGeometry(avoidance_cell);
            }
        } catch (...) {
            // 单孔洞 buffer 失败时保留原孔洞，避免整条连接修复链中断。
            try {
                const f2c::types::Cell hole_cell(ring);
                if (hole_cell.getExteriorRing().size() >= 4) {
                    avoidance_cells.addGeometry(hole_cell);
                }
            } catch (...) {
                // 无效孔洞交给后续路径碰撞门处理。
            }
        }
    }
    if (avoidance_cells.size() == 0) return hole_rings;

    f2c::types::Cells merged_cells;
    try {
        merged_cells = avoidance_cells.unionCascaded();
    } catch (...) {
        std::vector<f2c::types::LinearRing> fallback_rings;
        fallback_rings.reserve(avoidance_cells.size());
        for (size_t cell_idx = 0;
             cell_idx < avoidance_cells.size(); ++cell_idx) {
            const auto exterior =
                avoidance_cells.getGeometry(cell_idx).getExteriorRing();
            if (exterior.size() >= 4) {
                fallback_rings.push_back(exterior);
            }
        }
        return fallback_rings.empty() ? hole_rings : fallback_rings;
        /*
        // 障碍缓冲 union 失败时退回原始孔洞，交给后续碰撞门继续判定。
        return hole_rings;
    }
        */
    }
    std::vector<f2c::types::LinearRing> avoidance_rings;
    avoidance_rings.reserve(merged_cells.size());
    for (size_t cell_idx = 0; cell_idx < merged_cells.size(); ++cell_idx) {
        const auto exterior =
            merged_cells.getGeometry(cell_idx).getExteriorRing();
        if (exterior.size() >= 4) avoidance_rings.push_back(exterior);
    }
    return avoidance_rings.empty() ? hole_rings : avoidance_rings;
}

std::vector<f2c::types::Point> repairConnectionPolyline(
    const std::vector<f2c::types::Point>& points,
    const std::vector<f2c::types::LinearRing>& avoidance_rings,
    bool& repaired)
{
    repaired = false;
    if (points.size() < 2) return points;

    // genRoute 或旧修补可能留下位于禁入区内的中间控制点。
    // 去掉这些无效锚点后，由两侧安全端点重新生成绕行折线。
    std::vector<f2c::types::Point> valid_points;
    appendDistinct(valid_points, points.front());
    for (size_t point_idx = 1; point_idx + 1 < points.size(); ++point_idx) {
        const auto& point = points[point_idx];
        if (!pointInAnyHole(
                point.getX(), point.getY(), avoidance_rings)) {
            appendDistinct(valid_points, point);
        } else {
            repaired = true;
        }
    }
    appendDistinct(valid_points, points.back());

    std::vector<f2c::types::Point> result;
    appendDistinct(result, valid_points.front());
    for (size_t point_idx = 1; point_idx < valid_points.size(); ++point_idx) {
        const auto& start = valid_points[point_idx - 1];
        const auto& end = valid_points[point_idx];
        const auto intersections =
            findHoleIntersections(start, end, avoidance_rings);

        for (size_t hit_idx = 0; hit_idx + 1 < intersections.size();) {
            const auto& entry = intersections[hit_idx];
            const auto& exit = intersections[hit_idx + 1];
            if (entry.hole_idx != exit.hole_idx) {
                ++hit_idx;
                continue;
            }
            const auto detour =
                walkHoleBoundary(entry, exit, avoidance_rings);
            for (const auto& point : detour) {
                appendDistinct(result, point);
            }
            repaired = repaired || !detour.empty();
            hit_idx += 2;
        }
        appendDistinct(result, end);
    }
    return result;
}

}  // namespace

size_t repairRouteConnectionsAroundHoles(
    f2c::types::Route& route,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double clearance)
{
    if (hole_rings.empty() || route.sizeConnections() == 0) return 0;

    const auto avoidance_rings = makeAvoidanceRings(hole_rings, clearance);
    size_t repaired_connections = 0;

    for (size_t connection_idx = 0;
         connection_idx < route.sizeConnections(); ++connection_idx) {
        std::vector<f2c::types::Point> points;
        if (connection_idx > 0 &&
            connection_idx - 1 < route.sizeVectorSwaths()) {
            const auto& previous_swaths = route.getSwaths(connection_idx - 1);
            if (previous_swaths.size() > 0) {
                appendDistinct(points, previous_swaths.back().endPoint());
            }
        }

        const auto& connection = route.getConnection(connection_idx);
        for (const auto& point : connection) appendDistinct(points, point);

        if (connection_idx < route.sizeVectorSwaths()) {
            const auto& next_swaths = route.getSwaths(connection_idx);
            if (next_swaths.size() > 0) {
                appendDistinct(points, next_swaths.at(0).startPoint());
            }
        }

        bool repaired = false;
        const auto repaired_points =
            repairConnectionPolyline(points, avoidance_rings, repaired);
        if (!repaired) continue;

        f2c::types::MultiPoint repaired_connection;
        for (const auto& point : repaired_points) {
            repaired_connection.addPoint(point);
        }
        route.setConnection(connection_idx, repaired_connection);
        ++repaired_connections;
    }
    return repaired_connections;
}

namespace {

f2c::types::Point interpolatePoint(
    const f2c::types::Point& start,
    const f2c::types::Point& end,
    double t)
{
    return f2c::types::Point(
        start.getX() + t * (end.getX() - start.getX()),
        start.getY() + t * (end.getY() - start.getY()));
}

bool pointInsidePlanningCell(
    const f2c::types::Point& point,
    const f2c::types::Cell& cell)
{
    if (!pointInPolygon(
            point.getX(), point.getY(), cell.getExteriorRing())) {
        return false;
    }
    std::vector<f2c::types::LinearRing> holes;
    for (size_t i = 0; i + 1 < cell.size(); ++i) {
        holes.push_back(cell.getInteriorRing(i));
    }
    return !pointInAnyHole(point.getX(), point.getY(), holes);
}

std::vector<f2c::types::Point> repairOutsidePolyline(
    const std::vector<f2c::types::Point>& points,
    const f2c::types::Cell& planning_cell,
    bool& repaired)
{
    repaired = false;
    if (points.size() < 2 || planning_cell.getExteriorRing().size() < 4) {
        return points;
    }

    const std::vector<f2c::types::LinearRing> outer_rings {
        planning_cell.getExteriorRing()};
    std::vector<f2c::types::Point> result;
    appendDistinct(result, points.front());

    for (size_t point_idx = 1; point_idx < points.size(); ++point_idx) {
        const auto& start = points[point_idx - 1];
        const auto& end = points[point_idx];
        const auto intersections =
            findHoleIntersections(start, end, outer_rings);
        if (intersections.size() < 2) {
            appendDistinct(result, end);
            continue;
        }

        for (size_t interval_idx = 0;
             interval_idx <= intersections.size(); ++interval_idx) {
            const double start_t = interval_idx == 0
                ? 0.0 : intersections[interval_idx - 1].t;
            const double end_t = interval_idx == intersections.size()
                ? 1.0 : intersections[interval_idx].t;
            const auto interval_start =
                interpolatePoint(start, end, start_t);
            const auto interval_end = interpolatePoint(start, end, end_t);
            const auto midpoint = interpolatePoint(
                interval_start, interval_end, 0.5);
            if (pointInsidePlanningCell(midpoint, planning_cell) ||
                interval_idx == 0 ||
                interval_idx == intersections.size()) {
                appendDistinct(result, interval_end);
                continue;
            }

            const auto detour = walkHoleBoundary(
                intersections[interval_idx - 1],
                intersections[interval_idx], outer_rings);
            for (const auto& point : detour) {
                appendDistinct(result, point);
            }
            repaired = repaired || !detour.empty();
        }
    }
    return result;
}

}  // namespace

size_t repairRouteConnectionsOutsideCell(
    f2c::types::Route& route,
    const f2c::types::Cell& planning_cell)
{
    if (route.sizeConnections() == 0 ||
        planning_cell.getExteriorRing().size() < 4) {
        return 0;
    }

    size_t repaired_connections = 0;
    for (size_t connection_idx = 0;
         connection_idx < route.sizeConnections(); ++connection_idx) {
        std::vector<f2c::types::Point> points;
        if (connection_idx > 0 &&
            connection_idx - 1 < route.sizeVectorSwaths()) {
            const auto& previous_swaths = route.getSwaths(connection_idx - 1);
            if (previous_swaths.size() > 0) {
                appendDistinct(points, previous_swaths.back().endPoint());
            }
        }

        const auto& connection = route.getConnection(connection_idx);
        for (const auto& point : connection) appendDistinct(points, point);

        if (connection_idx < route.sizeVectorSwaths()) {
            const auto& next_swaths = route.getSwaths(connection_idx);
            if (next_swaths.size() > 0) {
                appendDistinct(points, next_swaths.at(0).startPoint());
            }
        }

        bool repaired = false;
        const auto repaired_points =
            repairOutsidePolyline(points, planning_cell, repaired);
        if (!repaired) continue;

        f2c::types::MultiPoint repaired_connection;
        for (const auto& point : repaired_points) {
            repaired_connection.addPoint(point);
        }
        route.setConnection(connection_idx, repaired_connection);
        ++repaired_connections;
    }
    return repaired_connections;
}

size_t synchronizeRouteConnectionEndpoints(
    f2c::types::Route& route,
    double max_endpoint_shift)
{
    constexpr double kEndpointTolerance = 1e-9;
    const double endpoint_shift_limit =
        std::max(0.0, max_endpoint_shift) + kEndpointTolerance;
    size_t synchronized = 0;
    const size_t group_count = route.sizeVectorSwaths();
    const size_t connection_count = route.sizeConnections();

    for (size_t group_idx = 1;
         group_idx < group_count && group_idx < connection_count;
         ++group_idx) {
        const auto& current_swaths = route.getSwaths(group_idx);
        const auto& connection = route.getConnection(group_idx);
        if (current_swaths.size() == 0 || connection.size() == 0) continue;

        size_t previous_idx = group_idx;
        while (previous_idx > 0) {
            --previous_idx;
            if (route.getSwaths(previous_idx).size() > 0) break;
        }
        const auto& previous_swaths = route.getSwaths(previous_idx);
        if (previous_swaths.size() == 0) continue;

        const auto previous_end = previous_swaths.back().endPoint();
        const auto current_start = current_swaths.at(0).startPoint();
        const auto& old_first = connection.getGeometry(0);
        const auto& old_last = connection.getGeometry(connection.size() - 1);
        const bool first_changed =
            old_first.distance(previous_end) > kEndpointTolerance;
        const bool last_changed =
            old_last.distance(current_start) > kEndpointTolerance;
        if (!first_changed && !last_changed) continue;

        f2c::types::MultiPoint synchronized_connection;
        synchronized_connection.addPoint(
            previous_end.getX(), previous_end.getY());

        if (connection.size() == 1) {
            // 单点若靠近新首尾点，就是缩进前遗留的旧端点；远离两端
            // 才按真实绕障控制点保留。
            const double distance_to_endpoint = std::min(
                old_first.distance(previous_end),
                old_first.distance(current_start));
            if (distance_to_endpoint > endpoint_shift_limit) {
                synchronized_connection.addPoint(
                    old_first.getX(), old_first.getY());
            }
        } else {
            for (size_t point_idx = 1;
                 point_idx + 1 < connection.size(); ++point_idx) {
                const auto& point = connection.getGeometry(point_idx);
                synchronized_connection.addPoint(point.getX(), point.getY());
            }
        }

        synchronized_connection.addPoint(
            current_start.getX(), current_start.getY());
        route.setConnection(group_idx, synchronized_connection);
        ++synchronized;
    }

    return synchronized;
}

f2c::types::Path planDirectPath(
    const f2c::types::Route& route,
    double cruise_velocity)
{
    f2c::types::Path path;
    const double velocity = cruise_velocity > 0.0 ? cruise_velocity : 1.0;
    bool has_endpoint = false;
    f2c::types::Point endpoint;

    for (size_t group_idx = 0;
         group_idx < route.sizeVectorSwaths(); ++group_idx) {
        const auto& swaths = route.getSwaths(group_idx);
        std::vector<f2c::types::Point> connection_points;

        if (has_endpoint) {
            appendDistinct(connection_points, endpoint);
        }
        if (group_idx < route.sizeConnections()) {
            const auto& connection = route.getConnection(group_idx);
            for (const auto& point : connection) {
                appendDistinct(connection_points, point);
            }
        }
        if (swaths.size() > 0) {
            appendDistinct(connection_points, swaths.at(0).startPoint());
        }
        appendConnectionPolyline(path, connection_points, velocity);
        if (!connection_points.empty()) {
            endpoint = connection_points.back();
            has_endpoint = true;
        }

        for (size_t swath_idx = 0; swath_idx < swaths.size(); ++swath_idx) {
            if (has_endpoint) {
                appendSegment(
                    path, endpoint,
                    swaths.at(swath_idx).startPoint(),
                    f2c::types::PathSectionType::TURN,
                    velocity);
            }
            path.appendSwath(swaths.at(swath_idx), velocity);
            endpoint = swaths.at(swath_idx).endPoint();
            has_endpoint = true;
        }
    }

    if (route.sizeConnections() > route.sizeVectorSwaths()) {
        std::vector<f2c::types::Point> trailing_points;
        if (has_endpoint) {
            appendDistinct(trailing_points, endpoint);
        }
        for (const auto& point : route.getLastConnection()) {
            appendDistinct(trailing_points, point);
        }
        appendConnectionPolyline(path, trailing_points, velocity);
    }

    return path;
}

// ========== RDP 路径简化（分段感知版）==========
// Ramer-Douglas-Peucker 算法，剔除共线或近似共线的冗余路径点
//
// 策略：先检测路径中的转弯点（方向突变），标记为必须保留的段边界，
// 然后只在相邻转弯点之间的"准直线段"内执行 RDP，保护曲线连接不被破坏。
std::vector<f2c::types::PathState> simplifyPathRDP(
    const f2c::types::Path& path,
    double epsilon,
    double turn_angle_threshold)
{
    if (path.size() < 3 || epsilon <= 0.0) {
        std::vector<f2c::types::PathState> result;
        for (size_t i = 0; i < path.size(); ++i) {
            result.push_back(path[i]);
        }
        return result;
    }

    // 将路径转为点列表用于 RDP
    struct Pt { double x, y; int idx; };
    std::vector<Pt> points;
    for (size_t i = 0; i < path.size(); ++i) {
        points.push_back({path[i].point.getX(), path[i].point.getY(), static_cast<int>(i)});
    }

    // ── 步骤1：检测转弯点（段边界）──
    std::vector<bool> keep(points.size(), false);
    keep[0] = true;
    keep[points.size() - 1] = true;

    std::vector<int> turn_points;
    turn_points.push_back(0);

    for (size_t i = 2; i < points.size(); ++i) {
        double dx1 = points[i-1].x - points[i-2].x;
        double dy1 = points[i-1].y - points[i-2].y;
        double dx2 = points[i].x - points[i-1].x;
        double dy2 = points[i].y - points[i-1].y;

        double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

        if (len1 > 1e-9 && len2 > 1e-9) {
            double cross = dx1 * dy2 - dy1 * dx2;
            double dot = dx1 * dx2 + dy1 * dy2;
            double angle = std::abs(std::atan2(cross, dot));

            if (angle > turn_angle_threshold) {
                keep[i - 1] = true;
                turn_points.push_back(static_cast<int>(i - 1));
            }
        }
    }
    turn_points.push_back(static_cast<int>(points.size()) - 1);

    // ── 步骤2：RDP 递归 ──
    std::function<void(int, int)> rdp = [&](int start, int end) {
        if (end - start <= 1) return;

        double max_dist = 0.0;
        int max_idx = start;

        double dx = points[end].x - points[start].x;
        double dy = points[end].y - points[start].y;
        double len = std::sqrt(dx * dx + dy * dy);

        for (int i = start + 1; i < end; ++i) {
            if (keep[i]) continue;

            double dist;
            if (len < 1e-12) {
                double pdx = points[i].x - points[start].x;
                double pdy = points[i].y - points[start].y;
                dist = std::sqrt(pdx * pdx + pdy * pdy);
            } else {
                double cross = std::abs(
                    (points[i].x - points[start].x) * dy -
                    (points[i].y - points[start].y) * dx
                );
                dist = cross / len;
            }

            if (dist > max_dist) {
                max_dist = dist;
                max_idx = i;
            }
        }

        if (max_dist > epsilon) {
            keep[max_idx] = true;
            rdp(start, max_idx);
            rdp(max_idx, end);
        }
    };

    // ── 步骤3：逐段执行 RDP ──
    for (size_t s = 0; s < turn_points.size() - 1; ++s) {
        rdp(turn_points[s], turn_points[s + 1]);
    }

    // 构建简化后的路径
    std::vector<f2c::types::PathState> simplified;
    for (size_t i = 0; i < points.size(); ++i) {
        if (keep[i]) {
            simplified.push_back(path[points[i].idx]);
        }
    }

    return simplified;
}

// ========== 简化路径（返回 Path 对象）==========
f2c::types::Path simplifyPath(
    const f2c::types::Path& path,
    double epsilon,
    double turn_angle_threshold)
{
    auto states = simplifyPathRDP(path, epsilon, turn_angle_threshold);
    f2c::types::Path result;
    for (const auto& s : states) {
        result.addState(s);
    }
    return result;
}
// v9.1 纯出口驱动贪心：Cell 顺序、入口方向、Cell 内部方向联合决定。
// 从 C0 的出口出发，每次比较所有未访问 Cell 的 normal/reverse 入口，
// 连接穿洞时增加惩罚；选中后立即更新当前出口并继续。
void greedyCellOrder(
    f2c::types::SwathsByCells& swaths_by_cells,
    std::vector<size_t>& cell_order,
    const std::vector<f2c::types::LinearRing>& hole_rings)
{
    const size_t n_cells = swaths_by_cells.size();

    // 初始化 cell_order 为恒等映射
    cell_order.resize(n_cells);
    for (size_t i = 0; i < n_cells; ++i) cell_order[i] = i;

    // 单 cell 无需排序
    if (n_cells <= 1) return;

    // 孔洞裁剪或接缝去重可能留下空 Cell；从第一个非空 Cell 启动，
    // 避免把空组当作出口解引用，同时保持原始索引映射。
    size_t start_ci = 0;
    while (start_ci < n_cells && swaths_by_cells[start_ci].size() == 0) {
        ++start_ci;
    }
    if (start_ci == n_cells) {
        swaths_by_cells = f2c::types::SwathsByCells();
        cell_order.clear();
        return;
    }

    std::vector<bool> visited(n_cells, false);
    f2c::types::SwathsByCells result;
    std::vector<size_t> new_order;

    // 从第一个非空 Cell 出发；正常场景下它就是 C0。
    visited[start_ci] = true;
    result.push_back(swaths_by_cells[start_ci]);
    new_order.push_back(start_ci);

    // 当前出口 = 起始 Cell 最后一条 swath 的终点
    double cur_x, cur_y;
    {
        const auto& c0 = swaths_by_cells[start_ci];
        const auto& last_sw = c0.at(c0.size() - 1);
        cur_x = last_sw.endPoint().getX();
        cur_y = last_sw.endPoint().getY();
    }

    // 贪心主循环
    while (new_order.size() < n_cells) {
        double best_dist = std::numeric_limits<double>::max();
        size_t best_ci = 0;
        bool best_reverse = false;

        for (size_t ci = 0; ci < n_cells; ++ci) {
            if (visited[ci]) continue;
            const auto& cell = swaths_by_cells[ci];
            if (cell.size() == 0) continue;

            const auto& first_sw = cell.at(0);
            const auto& last_sw = cell.at(cell.size() - 1);

            double fsx = first_sw.startPoint().getX();
            double fsy = first_sw.startPoint().getY();
            double lex = last_sw.endPoint().getX();
            double ley = last_sw.endPoint().getY();

            // 候选1: 正常方向 — cur → cell.first.start
            double dist_normal = std::hypot(cur_x - fsx, cur_y - fsy);
            if (!hole_rings.empty() &&
                yingshi::segmentCrossesHole(
                    cur_x, cur_y, fsx, fsy, hole_rings, 20)) {
                dist_normal += 1000.0;  // 穿洞惩罚
            }

            // 候选2: 反转方向 — cur → cell.last.end
            double dist_rev = std::hypot(cur_x - lex, cur_y - ley);
            if (!hole_rings.empty() &&
                yingshi::segmentCrossesHole(
                    cur_x, cur_y, lex, ley, hole_rings, 20)) {
                dist_rev += 1000.0;
            }

            if (dist_normal < best_dist) {
                best_dist = dist_normal;
                best_ci = ci;
                best_reverse = false;
            }
            if (dist_rev < best_dist) {
                best_dist = dist_rev;
                best_ci = ci;
                best_reverse = true;
            }
        }

        // 无有效候选（所有剩余 cell 均为空）→ 提前退出
        if (best_dist >= std::numeric_limits<double>::max() * 0.5) break;

        // 选中 best_ci，标记已访问
        visited[best_ci] = true;
        new_order.push_back(best_ci);

        // 先计算出口坐标（在 move 之前，避免 use-after-move）
        double next_x, next_y;
        {
            const auto& orig_cell = swaths_by_cells[best_ci];
            if (best_reverse) {
                // 出口 = 原 cell 第一个 swath 的起点（反转后最后一个 swath 的终点）
                next_x = orig_cell.at(0).startPoint().getX();
                next_y = orig_cell.at(0).startPoint().getY();
            } else {
                // 出口 = 最后一个 swath 的终点
                const auto& last_sw = orig_cell.at(orig_cell.size() - 1);
                next_x = last_sw.endPoint().getX();
                next_y = last_sw.endPoint().getY();
            }
        }

        auto selected_cell = std::move(swaths_by_cells[best_ci]);

        if (best_reverse) {
            // 反转整个 cell：倒序 swaths + 每条 swath 交换起终点
            f2c::types::Swaths reversed;
            for (size_t si = selected_cell.size(); si > 0; --si) {
                const auto& sw = selected_cell.at(si - 1);
                f2c::types::LineString rev_line;
                rev_line.addPoint(sw.endPoint());
                rev_line.addPoint(sw.startPoint());
                f2c::types::Swath rev_sw(rev_line, sw.getWidth(), sw.getId(), sw.getType());
                reversed.push_back(rev_sw);
            }
            selected_cell = reversed;
        }

        cur_x = next_x;
        cur_y = next_y;
        result.push_back(std::move(selected_cell));
    }

    swaths_by_cells = result;
    cell_order = new_order;
}

}  // namespace yingshi
