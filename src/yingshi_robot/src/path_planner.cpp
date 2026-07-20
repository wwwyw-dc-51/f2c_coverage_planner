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
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

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
            if (t > 1e-8 && t < 1.0 - 1e-8 &&
                u >= -1e-8 && u <= 1.0 + 1e-8) {
                intersections.push_back({
                    t, hole_idx, edge_idx,
                    sx + t * dx, sy + t * dy});
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

// ── 可见图路径规划器：在孔洞之间找最短安全路径 ──
// 用 avoidance ring 的顶点构建可见图，Dijkstra 搜索最短路径。
// 这是底层路径规划，不是后处理补丁——它找到的是真正的最短绕行路径，
// 而不是沿着孔洞边界"走一圈"。

bool segmentCrossesAnyRing(
    const f2c::types::Point& start,
    const f2c::types::Point& end,
    const std::vector<f2c::types::LinearRing>& rings)
{
    return !findHoleIntersections(start, end, rings).empty();
}

std::vector<f2c::types::Point> planHoleAvoidancePath(
    const f2c::types::Point& start,
    const f2c::types::Point& end,
    const std::vector<f2c::types::LinearRing>& avoidance_rings)
{
    // 直线不穿过任何 ring → 直接返回
    if (!segmentCrossesAnyRing(start, end, avoidance_rings)) {
        return {start, end};
    }

    // 收集候选路标点：start + all avoidance ring vertices + end
    struct Node { double x, y; };
    std::vector<Node> nodes;
    nodes.push_back({start.getX(), start.getY()});

    for (const auto& ring : avoidance_rings) {
        for (size_t i = 0; i + 1 < ring.size(); ++i) {
            nodes.push_back({
                ring.getGeometry(i).getX(),
                ring.getGeometry(i).getY()});
        }
    }
    nodes.push_back({end.getX(), end.getY()});

    const size_t n = nodes.size();
    const size_t start_id = 0;
    const size_t end_id = n - 1;
    if (n <= 2) return {start, end};

    // 构建邻接表：两点之间有边 ↔ 连线不穿过任何 enlarged ring
    std::vector<std::vector<std::pair<size_t, double>>> adj(n);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            f2c::types::Point pi(nodes[i].x, nodes[i].y);
            f2c::types::Point pj(nodes[j].x, nodes[j].y);
            if (!segmentCrossesAnyRing(pi, pj, avoidance_rings)) {
                double d = pi.distance(pj);
                adj[i].push_back({j, d});
                adj[j].push_back({i, d});
            }
        }
    }

    // Dijkstra
    std::vector<double> dist(n, std::numeric_limits<double>::max());
    std::vector<int> prev(n, -1);
    using PQItem = std::pair<double, size_t>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;

    dist[start_id] = 0.0;
    pq.push({0.0, start_id});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;
        if (u == end_id) break;
        for (const auto& [v, w] : adj[u]) {
            double nd = d + w;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = static_cast<int>(u);
                pq.push({nd, v});
            }
        }
    }

    // 重建路径（不可达时回退到直接连接）
    if (dist[end_id] >= std::numeric_limits<double>::max() * 0.5) {
        return {start, end};
    }

    std::vector<size_t> node_path;
    for (int u = static_cast<int>(end_id); u >= 0; u = prev[u]) {
        node_path.push_back(static_cast<size_t>(u));
    }
    std::reverse(node_path.begin(), node_path.end());

    std::vector<f2c::types::Point> result;
    result.reserve(node_path.size());
    for (size_t id : node_path) {
        result.emplace_back(nodes[id].x, nodes[id].y);
    }
    return result;
}

std::vector<f2c::types::LinearRing> makeAvoidanceRings(
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double clearance)
{
    f2c::types::Cells avoidance_cells;
    for (const auto& ring : hole_rings) {
        const f2c::types::Cell hole_cell(ring);
        const auto avoidance_cell = clearance > 1e-9 ?
            f2c::types::Cell::buffer(hole_cell, clearance) : hole_cell;
        if (avoidance_cell.getExteriorRing().size() >= 4) {
            avoidance_cells.addGeometry(avoidance_cell);
        }
    }
    if (avoidance_cells.size() == 0) return hole_rings;

    const auto merged_cells = avoidance_cells.unionCascaded();
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

        // ── 可见图路径规划：底层最短安全路径 ──
        // 替代 walkHoleBoundary 的"沿边界走"策略。
        // 在孔洞 avoidance ring 顶点上构建可见图 + Dijkstra，
        // 找到真正的最短绕行路径。
        const auto detour =
            planHoleAvoidancePath(start, end, avoidance_rings);
        for (size_t i = 1; i < detour.size(); ++i) {
            appendDistinct(result, detour[i]);
        }
        if (detour.size() > 2) {
            repaired = true;
        }
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

// ========== 贪心 Cell 排序（链式递推） ==========
// 保留 F2C 原生排序，只在四种合法入口变体中优化连接代价；有孔洞时
// 对固定极角顺序做全链动态规划，无孔洞时做 Cell/入口联合贪心。
void greedyCellOrder(
    f2c::types::SwathsByCells& swaths_by_cells,
    std::vector<size_t>& cell_order,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    const std::string& swath_order_type)
{
    const size_t n_cells = swaths_by_cells.size();

    // 初始化 cell_order 为恒等映射
    cell_order.resize(n_cells);
    for (size_t i = 0; i < n_cells; ++i) cell_order[i] = i;

    if (n_cells == 0) return;

    std::vector<bool> ids_normalized(n_cells, false);

    // F2C 依据 Swath ID 排序。补线在生成阶段已明确放在 Cell 的首/尾，
    // 因此先按当前顺序重新编号，不能让默认 ID=0 的补线跳回中间。
    const auto normalizeIds = [&](size_t ci) {
        if (ids_normalized[ci]) return;
        ids_normalized[ci] = true;
        auto& cell = swaths_by_cells[ci];
        for (size_t i = 0; i < cell.size(); ++i) {
            cell.at(i).setId(static_cast<int>(i));
        }
    };

    const auto sortedVariant = [&](size_t ci, uint32_t variant) {
        normalizeIds(ci);
        const auto& cell = swaths_by_cells[ci];
        if (swath_order_type == "snake") {
            f2c::rp::SnakeOrder sorter;
            return sorter.genSortedSwaths(cell, variant);
        }
        if (swath_order_type == "spiral") {
            f2c::rp::SpiralOrder sorter(6);
            return sorter.genSortedSwaths(cell, variant);
        }
        f2c::rp::BoustrophedonOrder sorter;
        return sorter.genSortedSwaths(cell, variant);
    };

    const auto connectionCost = [&](double from_x, double from_y,
                                    const f2c::types::Point& target) {
        double cost = std::hypot(
            from_x - target.getX(), from_y - target.getY());
        if (segmentCrossesHole(
                from_x, from_y, target.getX(), target.getY(),
                hole_rings, 20)) {
            cost += 1e9;  // 硬约束：视穿洞连接为不可行
        }
        return cost;
    };

    struct CellChoice {
        size_t cell_index {0};
        double cost {std::numeric_limits<double>::max()};
        f2c::types::Swaths swaths;
    };

    const auto chooseVariant = [&](size_t ci, double entry_x, double entry_y) {
        CellChoice best;
        best.cell_index = ci;
        normalizeIds(ci);
        if (swaths_by_cells[ci].size() == 0) return best;

        for (uint32_t variant = 0; variant < 4; ++variant) {
            auto candidate = sortedVariant(ci, variant);
            const double cost = connectionCost(
                entry_x, entry_y, candidate.at(0).startPoint());
            if (cost < best.cost) {
                best.cost = cost;
                best.swaths = std::move(candidate);
            }
        }
        return best;
    };

    if (n_cells == 1) {
        // C0 没有上游出口，固定使用 variant 0 作为稳定起点。
        swaths_by_cells[0] = sortedVariant(0, 0);
        return;
    }

    // ── 有孔洞时：圆形绕洞排序作为遍历序基础 ──
    // 贪心纯端点距离在孔洞场景下可能产生跨洞连接，
    // 圆形排序天然形成绕洞环，贪心在此基础上决定反转方向。
    std::vector<size_t> traversal_order;
    if (!hole_rings.empty()) {
        // 计算孔洞中心
        double h_cx = 0, h_cy = 0;
        size_t h_n = 0;
        for (const auto& hr : hole_rings) {
            for (size_t vi = 0; vi + 1 < hr.size(); ++vi) {
                h_cx += hr.getGeometry(vi).getX();
                h_cy += hr.getGeometry(vi).getY();
                ++h_n;
            }
        }
        if (h_n > 0) { h_cx /= h_n; h_cy /= h_n; }

        // 计算每个 cell 质心绕孔洞中心的极角
        std::vector<std::pair<double, size_t>> cell_angles;
        for (size_t ci = 0; ci < n_cells; ++ci) {
            double sx = 0, sy = 0;
            const auto& cell = swaths_by_cells[ci];
            size_t sn = 0;
            for (size_t j = 0; j < cell.size(); ++j) {
                sx += (cell.at(j).startPoint().getX() + cell.at(j).endPoint().getX()) * 0.5;
                sy += (cell.at(j).startPoint().getY() + cell.at(j).endPoint().getY()) * 0.5;
                ++sn;
            }
            double ang = (sn > 0) ? std::atan2(sy/sn - h_cy, sx/sn - h_cx) : 0.0;
            cell_angles.push_back({ang, ci});
        }
        std::sort(cell_angles.begin(), cell_angles.end());

        // 找到 C0 在排序后的位置，从 C0 开始
        size_t c0_pos = 0;
        for (size_t i = 0; i < cell_angles.size(); ++i) {
            if (cell_angles[i].second == 0) { c0_pos = i; break; }
        }
        for (size_t i = 0; i < n_cells; ++i) {
            traversal_order.push_back(cell_angles[(c0_pos + i) % n_cells].second);
        }
    } else {
        // 无孔洞：用恒等序（后续贪心会动态选择）
        for (size_t i = 0; i < n_cells; ++i) traversal_order.push_back(i);
    }

    if (!hole_rings.empty()) {
        // Cell 极角顺序固定时，逐 Cell 只看入口会把坏出口留给下一 Cell。
        // 对每个 Cell 的 4 种合法入口变体做动态规划，最小化整条链的
        // 跨 Cell 连接代价。C0 保持 variant 0，确保起点稳定。
        constexpr size_t kVariantCount = 4;
        const double infinity = std::numeric_limits<double>::max();
        std::vector<std::vector<f2c::types::Swaths>> variants(n_cells);
        for (size_t ci = 0; ci < n_cells; ++ci) {
            variants[ci].reserve(kVariantCount);
            for (uint32_t variant = 0; variant < kVariantCount; ++variant) {
                variants[ci].push_back(sortedVariant(ci, variant));
            }
        }

        std::vector<std::vector<double>> costs(
            n_cells, std::vector<double>(kVariantCount, infinity));
        std::vector<std::vector<int>> parents(
            n_cells, std::vector<int>(kVariantCount, -1));
        costs[0][0] = 0.0;

        for (size_t order_pos = 1; order_pos < n_cells; ++order_pos) {
            const size_t previous_ci = traversal_order[order_pos - 1];
            const size_t current_ci = traversal_order[order_pos];
            for (size_t current_variant = 0;
                 current_variant < kVariantCount; ++current_variant) {
                const auto& current = variants[current_ci][current_variant];
                if (current.size() == 0) continue;
                for (size_t previous_variant = 0;
                     previous_variant < kVariantCount; ++previous_variant) {
                    if (costs[order_pos - 1][previous_variant] >=
                        infinity * 0.5) {
                        continue;
                    }
                    const auto& previous =
                        variants[previous_ci][previous_variant];
                    if (previous.size() == 0) continue;
                    const auto& previous_exit =
                        previous.at(previous.size() - 1).endPoint();
                    const double candidate_cost =
                        costs[order_pos - 1][previous_variant] +
                        connectionCost(
                            previous_exit.getX(), previous_exit.getY(),
                            current.at(0).startPoint());
                    if (candidate_cost < costs[order_pos][current_variant]) {
                        costs[order_pos][current_variant] = candidate_cost;
                        parents[order_pos][current_variant] =
                            static_cast<int>(previous_variant);
                    }
                }
            }
        }

        size_t selected_variant = 0;
        for (size_t variant = 1; variant < kVariantCount; ++variant) {
            if (costs[n_cells - 1][variant] <
                costs[n_cells - 1][selected_variant]) {
                selected_variant = variant;
            }
        }
        if (costs[n_cells - 1][selected_variant] < infinity * 0.5) {
            std::vector<size_t> selected_variants(n_cells, 0);
            selected_variants[n_cells - 1] = selected_variant;
            for (size_t order_pos = n_cells - 1; order_pos > 0; --order_pos) {
                const int parent = parents[order_pos][selected_variant];
                if (parent < 0) break;
                selected_variant = static_cast<size_t>(parent);
                selected_variants[order_pos - 1] = selected_variant;
            }

            f2c::types::SwathsByCells optimized;
            for (size_t order_pos = 0; order_pos < n_cells; ++order_pos) {
                const size_t ci = traversal_order[order_pos];
                optimized.push_back(
                    variants[ci][selected_variants[order_pos]]);
            }
            swaths_by_cells = optimized;
            cell_order = traversal_order;
            return;
        }
    }

    // 无孔洞路径从固定的 C0 variant 0 开始，再联合选择 Cell 与入口变体。
    swaths_by_cells[0] = sortedVariant(0, 0);

    std::vector<bool> visited(n_cells, false);
    f2c::types::SwathsByCells result;
    std::vector<size_t> new_order;

    // 从 C0 出发
    visited[0] = true;
    result.push_back(swaths_by_cells[0]);
    new_order.push_back(0);

    // 当前出口 = C0 最后一条 swath 的终点
    double cur_x, cur_y;
    {
        const auto& c0 = swaths_by_cells[0];
        if (c0.size() == 0) {
            // 防御性：C0 无 swath，从第一个非空 cell 取起点
            cur_x = 0.0; cur_y = 0.0;
            for (size_t fi = 1; fi < swaths_by_cells.size(); ++fi) {
                if (swaths_by_cells[fi].size() > 0) {
                    cur_x = swaths_by_cells[fi].at(0).startPoint().getX();
                    cur_y = swaths_by_cells[fi].at(0).startPoint().getY();
                    break;
                }
            }
        } else {
            const auto& last_sw = c0.at(c0.size() - 1);
            cur_x = last_sw.endPoint().getX();
            cur_y = last_sw.endPoint().getY();
        }
    }

    // 主循环：有孔洞时保持极角 Cell 顺序；无孔洞时动态选择最近 Cell。
    size_t order_idx = 1;  // C0 已访问，从 traversal_order[1] 开始
    while (new_order.size() < n_cells) {
        CellChoice choice;

        if (!hole_rings.empty()) {
            const size_t next_ci = traversal_order[order_idx++];
            choice = chooseVariant(next_ci, cur_x, cur_y);
        } else {
            // Cell 次序与内部入口变体一起由上游真实出口决定。
            for (size_t ci = 0; ci < n_cells; ++ci) {
                if (visited[ci]) continue;
                auto candidate = chooseVariant(ci, cur_x, cur_y);
                if (candidate.cost < choice.cost) {
                    choice = std::move(candidate);
                }
            }
        }

        if (choice.cost >= std::numeric_limits<double>::max() * 0.5) {
            // 正常流程不会产生空 Cell；防御性退出避免解引用空 Swath。
            break;
        }

        const size_t selected_ci = choice.cell_index;
        visited[selected_ci] = true;
        new_order.push_back(selected_ci);
        swaths_by_cells[selected_ci] = std::move(choice.swaths);

        const auto& selected_cell = swaths_by_cells[selected_ci];
        if (selected_cell.size() > 0) {
            const auto& last_swath = selected_cell.at(selected_cell.size() - 1);
            cur_x = last_swath.endPoint().getX();
            cur_y = last_swath.endPoint().getY();
        }
        result.push_back(selected_cell);
    }

    swaths_by_cells = result;
    cell_order = new_order;
}

}  // namespace yingshi
