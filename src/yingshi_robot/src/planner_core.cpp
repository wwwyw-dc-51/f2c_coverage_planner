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

PlanningResult PlannerCore::plan(const PlanningRequest& req)
{
    PlanningResult result;
    auto t0 = std::chrono::steady_clock::now();

    try {
        // ── 1. 构建 Robot ──
        f2c::types::Robot robot = makeRobot(req);
        double r_w = req.coverage_width * (1.0 - req.swath_overlap_ratio);

        // ── 2. 生成 headlands ──
        f2c::hg::ConstHL hl_gen;
        f2c::types::Cells field;
        field.addGeometry(req.polygon);

        double mid_hl_w = req.mid_hl_width_ratio * req.robot_width;
        f2c::types::Cells mid_hl;
        if (mid_hl_w > 1e-6 && req.decomposition_enabled) {
            mid_hl = hl_gen.generateHeadlands(field, mid_hl_w);
            if (mid_hl.size() > 0) mid_hl = simplifyCells(mid_hl, 5.0);
        }

        f2c::types::Cells no_hl;
        if (req.decomposition_enabled && mid_hl.size() > 0) {
            for (size_t ci = 0; ci < mid_hl.size(); ++ci) {
                f2c::types::Cell work = mid_hl.getGeometry(ci);
                f2c::types::Cell grid = (ci < field.size())
                    ? field.getGeometry(ci) : work;
                DecomposerParams dparams;
                dparams.use_sweep = req.use_sweep_decomp;
                auto decomposed = rectilinearDecompose(work, grid, dparams);
                // 对每个分解后的 cell 施加 secondary headland 侵蚀
                double no_hl_w = req.no_hl_width_ratio * req.robot_width;
                if (no_hl_w > 1e-6) {
                    for (size_t di = 0; di < decomposed.size(); ++di) {
                        f2c::types::Cells single;
                        single.addGeometry(decomposed.getGeometry(di));
                        auto eroded = hl_gen.generateHeadlands(single, no_hl_w);
                        for (size_t ei = 0; ei < eroded.size(); ++ei)
                            no_hl.addGeometry(eroded.getGeometry(ei));
                    }
                } else {
                    for (size_t di = 0; di < decomposed.size(); ++di)
                        no_hl.addGeometry(decomposed.getGeometry(di));
                }
            }
        } else {
            // 不分解：对 polygon 整体施加 no_hl 侵蚀
            double no_hl_w = req.no_hl_width_ratio * req.robot_width;
            if (no_hl_w > 1e-6) {
                no_hl = hl_gen.generateHeadlands(field, no_hl_w);
            } else {
                no_hl.addGeometry(req.polygon);
            }
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
                req.swath_angle_optimization, req.swath_angle_candidates);

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
            repairRouteConnectionsAroundHoles(
                route, req.holes, req.coverage_width * 0.5);
            synchronizeRouteConnectionEndpoints(
                route, req.coverage_width * 0.5);
        }

        // ── 9. 边界策略：闭合边界收缩端点 / 开放边界延伸 ──
        {
            double margin = (req.boundary_type == "closed")
                ? req.swath_endpoint_shrink_distance
                : req.boundary_coverage_margin;
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
        if (result.path_has_crossings) {
            result.error_message =
                "Planned path crosses obstacle holes";
        }

        // ── 12. 组装结果 ──
        result.cells_with_swaths = swaths_by_cells;
        result.route = route;
        result.cell_order = cell_order;
        result.total_swaths = swaths_by_cells.sizeTotal();
        result.total_connections = route.sizeConnections();
        result.success = !result.path_has_crossings;

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

}  // namespace yingshi
