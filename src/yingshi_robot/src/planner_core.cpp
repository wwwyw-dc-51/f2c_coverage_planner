/**
 * @file planner_core.cpp
 * @brief PlannerCore 实现 — 纯算法流水线，不含 ROS/文件 I/O/评分
 */

#include "yingshi_robot/planner_core.hpp"
#include "yingshi_robot/decomposer.hpp"
#include "yingshi_robot/geometry_normalizer.hpp"
#include "yingshi_robot/swath_generator.hpp"
#include "yingshi_robot/boundary_filler.hpp"
#include "yingshi_robot/path_planner.hpp"
#include "yingshi_robot/path_geometry.hpp"
#include "yingshi_robot/planner_params.hpp"
#include <fields2cover/path_planning/dubins_curves.h>
#include <fields2cover/path_planning/dubins_curves_cc.h>
#include <fields2cover/path_planning/reeds_shepp_curves.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace yingshi {

namespace {

// ── 路径段穿洞计数 ──
size_t countCrossings(
    const std::vector<f2c::types::Point>& pts,
    const std::vector<f2c::types::LinearRing>& holes)
{
    if (holes.empty() || pts.size() < 2) return 0;
    size_t n = 0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        if (segmentCrossesHole(
                pts[i].getX(), pts[i].getY(),
                pts[i+1].getX(), pts[i+1].getY(), holes, 50))
            ++n;
    }
    return n;
}

bool pointOnRingBoundary(
    const f2c::types::Point& point,
    const f2c::types::LinearRing& ring,
    double tolerance)
{
    for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
        const auto& start = ring.getGeometry(i);
        const auto& end = ring.getGeometry(i + 1);
        const double dx = end.getX() - start.getX();
        const double dy = end.getY() - start.getY();
        const double length_squared = dx * dx + dy * dy;
        if (length_squared <= 1e-18) continue;
        const double t = std::clamp(
            ((point.getX() - start.getX()) * dx +
             (point.getY() - start.getY()) * dy) / length_squared,
            0.0, 1.0);
        const double closest_x = start.getX() + t * dx;
        const double closest_y = start.getY() + t * dy;
        if (std::hypot(point.getX() - closest_x,
                       point.getY() - closest_y) <= tolerance) {
            return true;
        }
    }
    return false;
}

bool pointInsideCellWithBoundaryTolerance(
    const f2c::types::Point& point,
    const f2c::types::Cell& cell)
{
    constexpr double kBoundaryTolerance = 1e-6;
    const auto& exterior = cell.getExteriorRing();
    const bool inside_exterior =
        pointInPolygon(point.getX(), point.getY(), exterior) ||
        pointOnRingBoundary(point, exterior, kBoundaryTolerance);
    if (!inside_exterior) return false;

    for (std::size_t i = 0; i + 1 < cell.size(); ++i) {
        const auto& hole = cell.getInteriorRing(i);
        if (pointInPolygon(point.getX(), point.getY(), hole) &&
            !pointOnRingBoundary(point, hole, kBoundaryTolerance)) {
            return false;
        }
    }
    return true;
}

size_t countSegmentsOutsideCell(
    const std::vector<f2c::types::Point>& points,
    const f2c::types::Cell& cell)
{
    if (points.size() < 2) return 0;

    std::vector<f2c::types::LinearRing> hole_rings;
    for (std::size_t i = 0; i + 1 < cell.size(); ++i) {
        hole_rings.push_back(cell.getInteriorRing(i));
    }

    constexpr double kSampleSpacing = 0.05;
    std::size_t violations = 0;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const auto& start = points[i];
        const auto& end = points[i + 1];
        const double segment_length = start.distance(end);
        if (segment_length <= 1e-9) continue;

        const std::size_t sample_count = std::max<std::size_t>(
            2, static_cast<std::size_t>(
                std::ceil(segment_length / kSampleSpacing)));
        bool outside = false;
        for (std::size_t sample = 0; sample <= sample_count; ++sample) {
            const double t = static_cast<double>(sample) /
                static_cast<double>(sample_count);
            const f2c::types::Point point(
                start.getX() + t * (end.getX() - start.getX()),
                start.getY() + t * (end.getY() - start.getY()));
            if (!pointInsideCellWithBoundaryTolerance(point, cell)) {
                outside = true;
                break;
            }
        }
        if (!outside && !hole_rings.empty() && segmentCrossesHole(
                start.getX(), start.getY(), end.getX(), end.getY(),
                hole_rings, 50)) {
            outside = true;
        }
        if (outside) ++violations;
    }
    return violations;
}

// ── 构建 F2C Robot ──
f2c::types::Robot makeRobot(const PlanningRequest& req)
{
    f2c::types::Robot robot(req.robot_width);
    robot.setMinTurningRadius(req.min_turning_radius);
    robot.setMaxDiffCurv(req.max_diff_curv);
    robot.setCovWidth(req.coverage_width);
    return robot;
}

double physicalHalfWidth(const PhysicalFootprintParams& params)
{
    return std::max(
        0.5 * params.body_width,
        std::abs(params.cleaner_center_lateral) +
            0.5 * params.cleaner_diameter) + params.safety_margin;
    /*
    // 清洁轮外缘决定横向最大占用宽度；车身宽度仍只用于碰撞检查。
    return std::max(
        0.5 * params.body_width,
        std::abs(params.cleaner_center_lateral) +
            0.5 * params.cleaner_diameter) + params.safety_margin;
    */
}

double holeRepairClearance(
    const PlanningRequest& req,
    bool cspace_constrained)
{
    if (cspace_constrained) return 0.0;

    const double coverage_half_width = 0.5 * req.coverage_width;
    if (!req.physical_collision_check_enabled) {
        return coverage_half_width;
    }

    return std::max(
        coverage_half_width,
        physicalFootprintClearanceRadius(req.physical_footprint));
}

// Snake 模式直连路由；其他模式保留 v9.12 的 genRoute。
f2c::types::Route buildSnakeRoute(
    const f2c::types::SwathsByCells& swaths_by_cells,
    const f2c::types::Cells& mid_hl)
{
    f2c::types::Route route;
    route.addConnection(f2c::types::MultiPoint());

    for (size_t ci = 0; ci < swaths_by_cells.size(); ++ci) {
        if (swaths_by_cells.at(ci).size() == 0) continue;
        route.addSwaths(swaths_by_cells.at(ci));

        f2c::types::MultiPoint conn;
        size_t next = ci + 1;
        while (next < swaths_by_cells.size() &&
               swaths_by_cells.at(next).size() == 0) ++next;
        if (next >= swaths_by_cells.size()) {
            route.addConnection(conn);
            continue;
        }

        const auto& last_sw = swaths_by_cells.at(ci).back();
        const auto& next_sw = swaths_by_cells.at(next).at(0);
        double lx = last_sw.endPoint().getX();
        double ly = last_sw.endPoint().getY();
        double nx = next_sw.startPoint().getX();
        double ny = next_sw.startPoint().getY();

        conn.addPoint(lx, ly);
        if (mid_hl.size() > 0) {
            const auto& hl = mid_hl.getGeometry(0).getExteriorRing();
            if (hl.size() > 2) {
                double hlx = 0, hly = 0;
                for (size_t hi = 0; hi + 1 < hl.size(); ++hi) {
                    hlx += hl.getGeometry(hi).getX();
                    hly += hl.getGeometry(hi).getY();
                }
                hlx /= (hl.size() - 1);
                hly /= (hl.size() - 1);
                double mx = (lx + nx) * 0.5;
                double my = (ly + ny) * 0.5;
                double dx = mx - hlx;
                double dy = my - hly;
                double d = std::hypot(dx, dy);
                if (d > 0.01)
                    conn.addPoint(hlx + dx * 0.3, hly + dy * 0.3);
            }
        }
        conn.addPoint(nx, ny);
        route.addConnection(conn);
    }
    return route;
}

}  // namespace

namespace {

PlanningComponentResult planSingleComponent(
    PlanningRequest req,
    std::size_t component_index,
    const f2c::types::Cells& coverage_target,
    bool cspace_constrained)
{
    req.polygon = normalizeCell(req.polygon);
    req.holes = normalizeRings(req.holes);

    PlanningComponentResult result;
    result.component_index = component_index;
    result.planning_polygon = req.polygon;
    result.coverage_target = coverage_target;
    auto t0 = std::chrono::steady_clock::now();

    try {
        // ── 1. 构建 Robot ──
        f2c::types::Robot robot = makeRobot(req);
        double r_w = req.coverage_width * (1.0 - req.swath_overlap_ratio);

        // ── 2. 生成 headlands（含自适应 + Phase 4A 对齐旋转 + 逐 cell 自适应）──
        f2c::hg::ConstHL hl_gen;
        f2c::types::Cells field;
        field.addGeometry(req.polygon);

        // 自适应 headland：检测多边形最小通道宽度
        double effective_mid_hl_ratio = req.mid_hl_width_ratio;
        double effective_no_hl_ratio = req.no_hl_width_ratio;
        {
            const auto& ring = req.polygon.getExteriorRing();
            double min_passage = std::numeric_limits<double>::max();
            size_t n = ring.size();
            for (size_t i = 0; i < n; ++i) {
                size_t i_next = (i + 1) % n;
                for (size_t j = i + 2; j < n; ++j) {
                    size_t j_next = (j + 1) % n;
                    if (j == i || j == i_next || j_next == i) continue;
                    double d1 = std::hypot(ring.getGeometry(i).getX() - ring.getGeometry(j).getX(), ring.getGeometry(i).getY() - ring.getGeometry(j).getY());
                    double d2 = std::hypot(ring.getGeometry(i).getX() - ring.getGeometry(j_next).getX(), ring.getGeometry(i).getY() - ring.getGeometry(j_next).getY());
                    double d3 = std::hypot(ring.getGeometry(i_next).getX() - ring.getGeometry(j).getX(), ring.getGeometry(i_next).getY() - ring.getGeometry(j).getY());
                    double d4 = std::hypot(ring.getGeometry(i_next).getX() - ring.getGeometry(j_next).getX(), ring.getGeometry(i_next).getY() - ring.getGeometry(j_next).getY());
                    min_passage = std::min({min_passage, d1, d2, d3, d4});
                }
            }
            if (min_passage < 10.0 && min_passage > 0.01) {
                double total_hl_erosion = (req.mid_hl_width_ratio + req.no_hl_width_ratio) * req.robot_width;
                double eroded_passage = min_passage - 2.0 * total_hl_erosion;
                if (eroded_passage < r_w) {
                    double max_erosion = std::max(0.0, (min_passage - r_w) / 2.0);
                    double max_ratio = max_erosion / req.robot_width;
                    double total_ratio = req.mid_hl_width_ratio + req.no_hl_width_ratio;
                    if (total_ratio > 1e-9) {
                        double scale = max_ratio / total_ratio;
                        effective_mid_hl_ratio = req.mid_hl_width_ratio * scale;
                        effective_no_hl_ratio = req.no_hl_width_ratio * scale;
                    }
                }
            }
        }

        double mid_hl_w = effective_mid_hl_ratio * req.robot_width;
        f2c::types::Cells mid_hl;
        if (mid_hl_w > 1e-6) {
            mid_hl = hl_gen.generateHeadlands(field, mid_hl_w);
            if (mid_hl.size() > 0) mid_hl = simplifyCells(mid_hl, 5.0);
        }

        // Phase 4A: Sweep 对齐旋转 — 检测最长边方向，旋转后分解再转回
        double sweep_align_angle = 0.0;
        if (req.use_sweep_decomp && req.polygon.size() > 0) {
            const auto& poly_ext = req.polygon.getExteriorRing();
            double longest_edge_len = 0.0;
            for (size_t pi = 0; pi + 1 < poly_ext.size(); ++pi) {
                double dx = poly_ext.getGeometry(pi+1).getX() - poly_ext.getGeometry(pi).getX();
                double dy = poly_ext.getGeometry(pi+1).getY() - poly_ext.getGeometry(pi).getY();
                double el = std::hypot(dx, dy);
                if (el > longest_edge_len) { longest_edge_len = el; sweep_align_angle = std::atan2(dy, dx); }
            }
            while (sweep_align_angle > M_PI/2) sweep_align_angle -= M_PI;
            while (sweep_align_angle <= -M_PI/2) sweep_align_angle += M_PI;
            if (std::abs(sweep_align_angle) < 10.0 * M_PI / 180.0) sweep_align_angle = 0.0;
        }

        f2c::types::Cells no_hl;
        if (req.decomposition_enabled && mid_hl.size() > 0) {
            for (size_t ci = 0; ci < mid_hl.size(); ++ci) {
                f2c::types::Cell work_cell = mid_hl.getGeometry(ci);
                f2c::types::Cell grid_cell = (ci < field.size()) ? field.getGeometry(ci) : work_cell;
                DecomposerParams dparams;
                dparams.use_sweep = req.use_sweep_decomp;

                f2c::types::Cells decomposed;
                if (std::abs(sweep_align_angle) > 0.001) {
                    auto rotated = rotateCell(work_cell, -sweep_align_angle);
                    auto rotated_grid = rotateCell(grid_cell, -sweep_align_angle);
                    dparams.use_sweep = true;
                    auto sub = rectilinearDecompose(rotated, rotated_grid, dparams);
                    for (size_t si = 0; si < sub.size(); ++si)
                        decomposed.addGeometry(rotateCell(sub.getGeometry(si), sweep_align_angle));
                } else {
                    decomposed = rectilinearDecompose(work_cell, grid_cell, dparams);
                }
                decomposed = normalizeCells(decomposed);

                // 逐 cell 自适应 no_hl
                double cell_no_hl_ratio = effective_no_hl_ratio;
                {
                    double cell_perimeter = work_cell.getExteriorRing().length();
                    double cell_area = work_cell.area();
                    double est_width = (cell_perimeter > 1e-9) ? (2.0 * cell_area / cell_perimeter) : 0.0;
                    double required = r_w + 0.05;
                    double eroded_width = est_width - 2.0 * cell_no_hl_ratio * req.robot_width;
                    if (eroded_width < required && est_width > 0.01) {
                        double max_erosion = std::max(0.0, (est_width - required) / 2.0);
                        cell_no_hl_ratio = std::max(0.0, max_erosion / req.robot_width);
                    }
                }

                double no_hl_w = cell_no_hl_ratio * req.robot_width;
                if (no_hl_w > 1e-6) {
                    for (size_t di = 0; di < decomposed.size(); ++di) {
                        f2c::types::Cells single;
                        single.addGeometry(decomposed.getGeometry(di));
                        auto eroded = hl_gen.generateHeadlands(single, no_hl_w);
                        for (size_t ei = 0; ei < eroded.size(); ++ei) no_hl.addGeometry(eroded.getGeometry(ei));
                    }
                } else {
                    for (size_t di = 0; di < decomposed.size(); ++di) no_hl.addGeometry(decomposed.getGeometry(di));
                }
            }
        } else {
            double no_hl_w = effective_no_hl_ratio * req.robot_width;
            if (no_hl_w > 1e-6) { no_hl = hl_gen.generateHeadlands(field, no_hl_w); }
            else { no_hl.addGeometry(req.polygon); }
        }

        no_hl = normalizeCells(no_hl);

        if (no_hl.size() == 0) {
            result.error_message = "Decomposition produced zero cells";
            return result;
        }

        // 第一轮：v9.11 同向合并（顶点邻近 + 质心连线孔洞保护 + interior ring）
        if (no_hl.size() > 1) {
            const double merge_angle_threshold = req.use_sweep_decomp
                ? 60.0 : req.merge_angle_threshold;
            no_hl = mergeCellsWithSimilarDirection(
                no_hl, req.polygon, req.coverage_width,
                merge_angle_threshold, req.use_sweep_decomp).cells;
            no_hl = normalizeCells(no_hl);
        }

        // 第二轮：合并被孔洞顶点误切的同 x-span 矩形条带（interior ring 把关）
        if (req.use_sweep_decomp && no_hl.size() > 1) {
            no_hl = mergeAdjacentSweepStrips(no_hl, req.coverage_width);
            no_hl = normalizeCells(no_hl);
        }

        // 方案 1：保守垂直兜底合并
        // 方案 2 的有洞条带子矩形拆分可能产生新的同 X 跨度相邻 cell，
        // 再跑一次垂直合并作为安全收尾
        if (req.use_sweep_decomp && no_hl.size() > 1) {
            no_hl = mergeAdjacentSweepStrips(no_hl, req.coverage_width);
            no_hl = normalizeCells(no_hl);
        }

        if (req.filter_tiny_cells) {
            const double min_cell_area =
                req.min_cell_area_ratio * req.coverage_width * req.robot_width;
            no_hl = filterTinyCells(no_hl, min_cell_area);
        }
        if (no_hl.size() == 0) {
            result.error_message = "Cell filtering produced zero cells";
            return result;
        }

        // ── 3. 生成 swaths（全局角度优化 + 边界填补）──
        const double physical_half_width =
            !cspace_constrained && req.physical_collision_check_enabled
            ? physicalHalfWidth(req.physical_footprint)
            : 0.0;
        f2c::types::SwathsByCells swaths_by_cells =
            generateSwathsForAllCells(
                no_hl, req.polygon, r_w, req.coverage_width,
                req.swath_endpoint_shrink_distance, req.min_swath_length,
                req.swath_angle_optimization, req.swath_angle_candidates,
                cspace_constrained ? 1e-4 : -1.0,
                req.use_sweep_decomp,
                physical_half_width);

        if (swaths_by_cells.sizeTotal() == 0) {
            result.error_message = "No swaths generated";
            return result;
        }

        // ── 4. 去重接缝补线 ──
        pruneRedundantCellSeamFills(
            swaths_by_cells, no_hl, req.polygon, req.coverage_width);

        if (!req.holes.empty()) {
            clipSwathsCrossingHoles(
                swaths_by_cells, req.holes, req.min_swath_length);
        }

        if (swaths_by_cells.sizeTotal() == 0) {
            result.error_message = "No swaths remain after hole clipping";
            return result;
        }

        // ── 5. 孔洞裁剪 ──
        // 先把穿过孔洞的单条 swath 切成安全片段，避免后续 Route
        // 连接器把本应断开的两侧重新视为同一条工作线。

        // ── 6. Swath + Cell 排序 ──
        std::vector<size_t> cell_order;
        for (size_t i = 0; i < swaths_by_cells.size(); ++i)
            cell_order.push_back(i);

        if (req.swath_order_type != "none") {
            greedyCellOrder(swaths_by_cells, cell_order, req.holes,
                            req.swath_order_type);
        }

        // ── 7. 构建 Route：恢复 v9.12 的 Snake 特例与 genRoute 基线 ──
        f2c::types::Route route;
        if (req.swath_order_type == "snake") {
            route = buildSnakeRoute(swaths_by_cells, mid_hl);
        } else {
            f2c::rp::RoutePlannerBase route_planner;
            route = route_planner.genRoute(mid_hl, swaths_by_cells);
        }

        // ── 8. 孔洞感知修复（genRoute 后立即修）──
        if (!req.holes.empty()) {
            const double connection_clearance = holeRepairClearance(
                req, cspace_constrained);
            repairRouteConnectionsAroundHoles(
                route, req.holes, connection_clearance);
            synchronizeRouteConnectionEndpoints(
                route, connection_clearance);
        }
        repairRouteConnectionsOutsideCell(route, req.polygon);

        // ── 9. 边界策略：闭合边界收缩端点 / 开放边界延伸 ──
        {
            const double margin = resolveBoundaryMargin(
                req.boundary_type,
                req.swath_endpoint_shrink_distance,
                req.boundary_coverage_margin,
                req.boundary_open_default_margin);
            if (std::abs(margin) > 1e-9 &&
                !(cspace_constrained && !req.holes.empty())) {
                const auto& outer = req.polygon.getExteriorRing();
                std::vector<f2c::types::LinearRing> hr;
                for (size_t ri = 0; ri + 1 < req.polygon.size(); ++ri)
                    hr.push_back(req.polygon.getInteriorRing(ri));
                applyBoundaryMarginToRoute(
                    route, outer, hr, req.coverage_width, margin);
                synchronizeRouteConnectionEndpoints(
                    route, 2.0 * std::abs(margin));
            }
        }

        // ── 9.5. 边界调整后最终孔洞修复 ──
        if (!req.holes.empty()) {
            repairRouteConnectionsAroundHoles(
                route, req.holes,
                holeRepairClearance(req, cspace_constrained));
        }
        repairRouteConnectionsOutsideCell(route, req.polygon);

        // 仅在 C-space 的 direct + 非 none 基线中尝试短连接直连；
        // TSP 对照和未约束物理 footprint 管线保持原样。
        if (cspace_constrained && req.turn_planner_type == "direct" &&
            req.swath_order_type != "none") {
            removeSafeMicroDetours(
                route, req.polygon, req.holes, 2e-4);
            shortenSafeRouteConnections(
                route, req.polygon, req.holes, 2.0, 0.05);
        }

        if (req.physical_collision_check_enabled && !cspace_constrained) {
            const auto endpoint_repairs =
                repairRouteSwathEndpointsForCollision(
                    route, req.polygon, req.holes,
                    req.physical_footprint);
            if (endpoint_repairs > 0) {
                synchronizeRouteConnectionEndpoints(
                    route,
                    2.0 * physicalFootprintClearanceRadius(
                        req.physical_footprint));
                if (!req.holes.empty()) {
                    repairRouteConnectionsAroundHoles(
                        route, req.holes,
                        holeRepairClearance(req, cspace_constrained));
                }
            }
        }

        // ── 9. 路径规划 ──
        f2c::types::Path path;
        if (req.turn_planner_type == "dubins_cc") {
            f2c::pp::PathPlanning pp;
            f2c::pp::DubinsCurvesCC dcc;
            path = pp.planPath(robot, route, dcc);
        } else if (req.turn_planner_type == "reeds_shepp") {
            f2c::pp::PathPlanning pp;
            f2c::pp::ReedsSheppCurves rs;
            path = pp.planPath(robot, route, rs);
        } else if (req.turn_planner_type == "dubins") {
            f2c::pp::PathPlanning pp;
            f2c::pp::DubinsCurves dc;
            path = pp.planPath(robot, route, dc);
        } else {
            // direct：零半径欧氏直连（默认）
            path = planDirectPath(route, 1.0);
        }

        // ── 10. 路径后处理 ──
        const auto unsimplified_path = path;
        if (req.path_simplify_enabled && path.size() > 0) {
            auto states = simplifyPathRDP(
                path, req.path_simplify_tolerance,
                req.path_simplify_turn_threshold);
            path = f2c::types::Path();
            for (auto& s : states) path.addState(s);
        }

        if (cspace_constrained && req.path_simplify_enabled) {
            const auto simplified_points = materializePath(path);
            const bool simplification_is_unsafe =
                countCrossings(simplified_points, req.holes) > 0 ||
                countSegmentsOutsideCell(
                    simplified_points, req.polygon) > 0;
            if (simplification_is_unsafe) {
                // RDP 可能把绕孔洞的折线压成一条穿洞弦线；C-space
                // 路径宁可保留原始折点，也不能牺牲碰撞安全性。
                path = unsimplified_path;
            }
        }

        if (req.physical_collision_check_enabled && !cspace_constrained &&
            req.path_simplify_enabled) {
            const auto simplified_collision = checkPathFootprintCollision(
                path, req.polygon, req.holes, req.physical_footprint);
            if (simplified_collision.collision) {
                const auto original_collision = checkPathFootprintCollision(
                    unsimplified_path, req.polygon, req.holes,
                    req.physical_footprint);
                if (!original_collision.collision) {
                    path = unsimplified_path;
                }
            }
        }

        // ── 11. 展平路径 + 诊断 ──
        result.path = path;
        result.path_points = materializePath(path);
        auto waypoints = materializePathWaypoints(path);
        for (auto& w : waypoints)
            result.path_waypoints.push_back(w.point);

        result.hole_crossing_segments =
            countCrossings(result.path_points, req.holes);
        result.path_has_crossings =
            (result.hole_crossing_segments > 0);
        if (cspace_constrained) {
            result.out_of_planning_area_segments =
                countSegmentsOutsideCell(
                    result.path_points, req.polygon);
            result.path_leaves_planning_area =
                result.out_of_planning_area_segments > 0;
        }
        if (req.physical_collision_check_enabled && !cspace_constrained) {
            // 碰撞检查前将 polygon 向外 buffer，避免 swath 端点贴边时
            // footprint 被误判为"超出边界"。
            const double footprint_buffer =
                physicalFootprintClearanceRadius(req.physical_footprint);
            const auto buffered_polygon =
                f2c::types::Cell::buffer(req.polygon, footprint_buffer);
            // 孔洞也向内缩一个 buffer，避免贴洞边时被误判为"intersects obstacle"
            std::vector<f2c::types::LinearRing> buffered_holes;
            buffered_holes.reserve(req.holes.size());
            for (const auto& hole : req.holes) {
                if (hole.size() < 4) continue;
                auto shrunk = f2c::types::Cell::buffer(
                    f2c::types::Cell(hole), -footprint_buffer);
                if (shrunk.size() > 0) {
                    buffered_holes.push_back(shrunk.getExteriorRing());
                }
            }
            result.physical_collision = checkPathFootprintCollision(
                path, buffered_polygon, buffered_holes, req.physical_footprint);
        }
        if (result.path_has_crossings) {
            result.error_message =
                "Planned path crosses obstacle holes";
        } else if (result.path_leaves_planning_area) {
            result.error_message =
                "Planned path leaves the C-space planning area";
        } else if (result.physical_collision.collision) {
            result.error_message =
                "Physical footprint collision: " +
                result.physical_collision.message;
        }

        // ── 12. 组装结果 ──
        result.cells_with_swaths = swaths_by_cells;
        result.route = route;
        result.cell_order = cell_order;
        result.decomposition_cells = no_hl;
        result.total_swaths = swaths_by_cells.sizeTotal();
        result.total_connections = route.sizeConnections();
        result.success = !result.path_has_crossings &&
            !result.path_leaves_planning_area &&
            !result.physical_collision.collision;

    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = e.what();
    } catch (...) {
        result.success = false;
        result.error_message = "PlannerCore::plan(): non-C++ crash";
    }

    auto t1 = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

std::vector<f2c::types::LinearRing> interiorRings(
    const f2c::types::Cell& cell)
{
    std::vector<f2c::types::LinearRing> rings;
    if (cell.size() > 1) {
        rings.reserve(cell.size() - 1);
    }
    for (std::size_t i = 0; i + 1 < cell.size(); ++i) {
        rings.push_back(cell.getInteriorRing(i));
    }
    return rings;
}

void exposeSingleComponent(
    const PlanningComponentResult& component,
    PlanningResult& result)
{
    result.path = component.path;
    result.path_points = component.path_points;
    result.path_waypoints = component.path_waypoints;
    result.cells_with_swaths = component.cells_with_swaths;
    result.route = component.route;
    result.cell_order = component.cell_order;
    result.total_swaths = component.total_swaths;
    result.total_connections = component.total_connections;
    result.hole_crossing_segments = component.hole_crossing_segments;
    result.out_of_planning_area_segments =
        component.out_of_planning_area_segments;
    result.path_has_crossings = component.path_has_crossings;
    result.path_leaves_planning_area =
        component.path_leaves_planning_area;
    result.physical_collision = component.physical_collision;
}

}  // namespace

PlanningResult PlannerCore::plan(const PlanningRequest& req)
{
    PlanningResult result;
    const auto planning_start = std::chrono::steady_clock::now();

    if (!req.traversability_enabled) {
        f2c::types::Cells coverage_target;
        coverage_target.addGeometry(req.polygon);
        auto component = planSingleComponent(
            req, 0, coverage_target, false);
        result.component_plans.push_back(component);
        exposeSingleComponent(component, result);
        result.success = component.success;
        result.error_message = component.error_message;
    } else {
        TraversabilityParams params;
        params.robot_width = req.robot_width;
        params.clearance_margin = req.cspace_clearance_margin;
        params.max_excluded_area_ratio = req.max_excluded_area_ratio;
        result.traversability = analyzeTraversability(req.polygon, params);

        if (!result.traversability.analysis_valid) {
            result.error_message = "C-space analysis failed: " +
                result.traversability.error_message;
        } else if (result.traversability.component_count == 0) {
            result.error_message = "C-space has no traversable component";
        } else if (result.traversability.exclusion_limit_exceeded) {
            result.error_message =
                "C-space exclusion gate exceeded: excluded ratio " +
                std::to_string(
                    result.traversability.excluded_area_ratio) +
                " is greater than configured maximum " +
                std::to_string(req.max_excluded_area_ratio);
        } else {
            bool all_components_succeeded = true;
            for (std::size_t i = 0;
                 i < result.traversability.coverage_components.size(); ++i) {
                const auto& coverage_component =
                    result.traversability.coverage_components[i];
                if (coverage_component.size() != 1) {
                    result.error_message =
                        "C-space coverage recovery produced an invalid "
                        "component at index " + std::to_string(i);
                    all_components_succeeded = false;
                    break;
                }

                PlanningRequest component_request = req;
                component_request.polygon =
                    result.traversability.center_space.getGeometry(i);
                component_request.holes =
                    interiorRings(component_request.polygon);
                component_request.traversability_enabled = false;

                auto component = planSingleComponent(
                    component_request, i, coverage_component, true);
                result.total_swaths += component.total_swaths;
                result.total_connections += component.total_connections;
                result.hole_crossing_segments +=
                    component.hole_crossing_segments;
                result.out_of_planning_area_segments +=
                    component.out_of_planning_area_segments;
                result.path_has_crossings =
                    result.path_has_crossings ||
                    component.path_has_crossings;
                result.path_leaves_planning_area =
                    result.path_leaves_planning_area ||
                    component.path_leaves_planning_area;
                if (component.physical_collision.collision) {
                    result.physical_collision = component.physical_collision;
                }
                if (!component.success) {
                    result.error_message =
                        "C-space component " + std::to_string(i) +
                        " failed: " + component.error_message;
                    all_components_succeeded = false;
                }
                result.component_plans.push_back(std::move(component));
                if (!all_components_succeeded) {
                    break;
                }
            }

            if (all_components_succeeded &&
                result.component_plans.size() ==
                    result.traversability.component_count) {
                result.success = true;
                if (result.component_plans.size() == 1) {
                    exposeSingleComponent(
                        result.component_plans.front(), result);
                }
            } else if (all_components_succeeded) {
                result.error_message =
                    "C-space component count changed during planning";
            }
        }
    }

    const auto planning_end = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(
            planning_end - planning_start).count();
    return result;
}

}  // namespace yingshi
