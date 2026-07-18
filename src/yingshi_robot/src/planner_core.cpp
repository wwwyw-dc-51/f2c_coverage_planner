/**
 * @file planner_core.cpp
 * @brief PlannerCore 实现 — 纯算法流水线，不含 ROS/文件 I/O/评分
 */

#include "yingshi_robot/planner_core.hpp"
#include "yingshi_robot/decomposer.hpp"
#include "yingshi_robot/swath_generator.hpp"
#include "yingshi_robot/boundary_filler.hpp"
#include "yingshi_robot/path_planner.hpp"
#include "yingshi_robot/path_geometry.hpp"
#include "yingshi_robot/planner_params.hpp"
#include <fields2cover/path_planning/dubins_curves.h>
#include <fields2cover/path_planning/dubins_curves_cc.h>
#include <fields2cover/path_planning/reeds_shepp_curves.h>
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

size_t countSegmentsOutsideCell(
    const std::vector<f2c::types::Point>& points,
    const f2c::types::Cell& cell)
{
    if (points.size() < 2) return 0;
    f2c::types::Cells allowed;
    allowed.addGeometry(cell);
    std::size_t violations = 0;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        f2c::types::LineString segment;
        segment.addPoint(points[i]);
        segment.addPoint(points[i + 1]);
        const double segment_length = segment.length();
        if (segment_length <= 1e-9) continue;
        const auto inside_parts = allowed.getLinesInside(segment);
        double inside_length = 0.0;
        for (std::size_t j = 0; j < inside_parts.size(); ++j) {
            inside_length += inside_parts.getGeometry(j).length();
        }
        if (inside_length + 1e-5 < segment_length) {
            ++violations;
        }
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

// ── Snake 模式直连路由 ──
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
                double mx = (lx+nx)*0.5, my = (ly+ny)*0.5;
                double dx = mx-hlx, dy = my-hly;
                double d = std::hypot(dx, dy);
                if (d > 0.01)
                    conn.addPoint(hlx+dx*0.3, hly+dy*0.3);
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
    const PlanningRequest& req,
    std::size_t component_index,
    const f2c::types::Cells& coverage_target,
    bool cspace_constrained)
{
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

        if (no_hl.size() == 0) {
            result.error_message = "Decomposition produced zero cells";
            return result;
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

        if (no_hl.size() > 1) {
            const double merge_angle_threshold = req.use_sweep_decomp
                ? 60.0 : req.merge_angle_threshold;
            no_hl = mergeCellsWithSimilarDirection(
                no_hl, req.polygon, req.coverage_width,
                merge_angle_threshold, req.use_sweep_decomp).cells;
        }

        // ── 3. 生成 swaths（全局角度优化 + 边界填补）──
        f2c::types::SwathsByCells swaths_by_cells =
            generateSwathsForAllCells(
                no_hl, req.polygon, r_w, req.coverage_width,
                req.swath_endpoint_shrink_distance, req.min_swath_length,
                req.swath_angle_optimization, req.swath_angle_candidates,
                cspace_constrained ? 1e-4 : -1.0,
                req.use_sweep_decomp);

        if (swaths_by_cells.sizeTotal() == 0) {
            result.error_message = "No swaths generated";
            return result;
        }

        // ── 4. 去重接缝补线 ──
        pruneRedundantCellSeamFills(
            swaths_by_cells, no_hl, req.polygon, req.coverage_width);

        // ── 5. 孔洞裁剪 ──
        // （当前未实现 standalone splitSwathsCrossingHoles；
        //  孔洞处理由下游 repairRouteConnectionsAroundHoles 覆盖）

        // ── 6. Swath + Cell 排序 ──
        std::vector<size_t> cell_order;
        for (size_t i = 0; i < swaths_by_cells.size(); ++i)
            cell_order.push_back(i);

        if (req.swath_order_type != "none") {
            greedyCellOrder(swaths_by_cells, cell_order, req.holes,
                            req.swath_order_type);
        }

        // ── 7. 构建 Route ──
        f2c::types::Route route;
        if (req.swath_order_type == "snake") {
            route = buildSnakeRoute(swaths_by_cells, mid_hl);
        } else {
            f2c::rp::RoutePlannerBase route_planner;
            route = route_planner.genRoute(mid_hl, swaths_by_cells);
        }

        // ── 8. 孔洞感知修复（genRoute 后立即修）──
        if (!req.holes.empty()) {
            const double connection_clearance = cspace_constrained
                ? 1e-4
                : req.coverage_width * 0.5;
            repairRouteConnectionsAroundHoles(
                route, req.holes, connection_clearance);
            synchronizeRouteConnectionEndpoints(
                route, connection_clearance);
        }

        // ── 9. 边界策略：闭合边界收缩端点 / 开放边界延伸 ──
        {
            const double margin = resolveBoundaryMargin(
                req.boundary_type,
                req.swath_endpoint_shrink_distance,
                req.boundary_coverage_margin,
                req.boundary_open_default_margin);
            if (std::abs(margin) > 1e-9) {
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
                route, req.holes, 0.001);
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
        if (req.path_simplify_enabled && path.size() > 0) {
            auto states = simplifyPathRDP(
                path, req.path_simplify_tolerance,
                req.path_simplify_turn_threshold);
            path = f2c::types::Path();
            for (auto& s : states) path.addState(s);
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
        if (result.path_has_crossings) {
            result.error_message =
                "Planned path crosses obstacle holes";
        } else if (result.path_leaves_planning_area) {
            result.error_message =
                "Planned path leaves the C-space planning area";
        }

        // ── 12. 组装结果 ──
        result.cells_with_swaths = swaths_by_cells;
        result.route = route;
        result.cell_order = cell_order;
        result.decomposition_cells = no_hl;
        result.total_swaths = swaths_by_cells.sizeTotal();
        result.total_connections = route.sizeConnections();
        result.success = !result.path_has_crossings &&
            !result.path_leaves_planning_area;

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
