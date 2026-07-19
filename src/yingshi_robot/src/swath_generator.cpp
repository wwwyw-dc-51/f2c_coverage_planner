/**
 * @file swath_generator.cpp
 * @brief Swath 生成模块实现 — 角度优化、端点调整、斜边检测、几何变换
 *
 * 从 polygon_planner_node.cpp 提取，属于模块化重构 Step 3。
 */

#include "yingshi_robot/swath_generator.hpp"
#include "yingshi_robot/decomposer.hpp"
#include "yingshi_robot/boundary_filler.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace yingshi {

// ========== Swath 长度计算 ==========
double swathLength(const f2c::types::Swath& swath)
{
    const auto start_point = swath.startPoint();
    const auto end_point = swath.endPoint();
    const double dx = end_point.getX() - start_point.getX();
    const double dy = end_point.getY() - start_point.getY();
    return std::sqrt(dx * dx + dy * dy);
}

// ========== 过滤短 Swaths ==========
f2c::types::Swaths filterShortSwaths(
    const f2c::types::Swaths& swaths,
    double min_length,
    size_t& removed_count)
{
    removed_count = 0;
    if (min_length <= 0.0) {
        return swaths;
    }

    f2c::types::Swaths filtered_swaths;
    for (size_t i = 0; i < swaths.size(); ++i) {
        const auto& swath = swaths.at(i);
        const double length = swathLength(swath);
        if (length < min_length) {
            ++removed_count;
            continue;
        }
        filtered_swaths.push_back(swath);
    }

    return filtered_swaths;
}

// ========== 双向 Swath 端点调整 ==========
// distance > 0：端点向中心收缩（闭合边界安全模式，留转向空间）
// distance < 0：端点向外延伸（开放边界覆盖模式，牺牲重叠换覆盖率）
// distance = 0：不调整，保持原样
f2c::types::Swath adjustSwathEndpoints(
    const f2c::types::Swath& swath,
    double distance)
{
    if (distance == 0.0) {
        return swath;
    }

    f2c::types::Point start_point = swath.startPoint();
    f2c::types::Point end_point = swath.endPoint();

    double dx = end_point.getX() - start_point.getX();
    double dy = end_point.getY() - start_point.getY();
    double length = std::sqrt(dx * dx + dy * dy);

    double abs_distance = std::abs(distance);
    if (length <= 2.0 * abs_distance) {
        return swath;  // swath 太短，不做调整
    }

    double unit_dx = dx / length;
    double unit_dy = dy / length;

    f2c::types::Point new_start(
        start_point.getX() + distance * unit_dx,
        start_point.getY() + distance * unit_dy
    );
    f2c::types::Point new_end(
        end_point.getX() - distance * unit_dx,
        end_point.getY() - distance * unit_dy
    );

    f2c::types::LineString new_path(new_start, new_end);
    f2c::types::Swath adjusted_swath(new_path, swath.getWidth(), swath.getId(), swath.getType());

    return adjusted_swath;
}

// ========== 批量 Swath 端点调整 ==========
f2c::types::SwathsByCells adjustSwathsEndpoints(
    const f2c::types::SwathsByCells& swaths_by_cells,
    double distance)
{
    if (distance == 0.0) {
        return swaths_by_cells;
    }

    f2c::types::SwathsByCells adjusted_swaths_by_cells;

    for (size_t cell_idx = 0; cell_idx < swaths_by_cells.size(); ++cell_idx) {
        const auto& cell_swaths = swaths_by_cells.at(cell_idx);
        f2c::types::Swaths adjusted_cell_swaths;

        for (size_t swath_idx = 0; swath_idx < cell_swaths.size(); ++swath_idx) {
            const auto& swath = cell_swaths.at(swath_idx);
            f2c::types::Swath adjusted_swath = adjustSwathEndpoints(swath, distance);
            adjusted_cell_swaths.push_back(adjusted_swath);
        }

        adjusted_swaths_by_cells.push_back(adjusted_cell_swaths);
    }

    return adjusted_swaths_by_cells;
}

// ========== 计算多边形主方向 ==========
// 委托给 decomposer 模块的 computeCellMainDirection（逻辑完全一致）
double computePolygonMainDirection(const f2c::types::Cell& cell)
{
    return yingshi::computeCellMainDirection(cell);
}

// ========== 旋转 Cell（绕原点）==========
f2c::types::Cell rotateCell(const f2c::types::Cell& cell, double angle)
{
    double c = std::cos(angle), s = std::sin(angle);
    f2c::types::Cell result;

    // 旋转外环
    f2c::types::LinearRing rotated_ext;
    const auto& ext = cell.getExteriorRing();
    for (size_t i = 0; i < ext.size(); ++i) {
        double x = ext.getGeometry(i).getX();
        double y = ext.getGeometry(i).getY();
        rotated_ext.addPoint(f2c::types::Point(c*x - s*y, s*x + c*y));
    }
    result.addRing(rotated_ext);

    // 旋转内环（孔洞）
    for (size_t hi = 0; hi + 1 < cell.size(); ++hi) {
        f2c::types::LinearRing rotated_int;
        const auto& ir = cell.getInteriorRing(hi);
        for (size_t i = 0; i < ir.size(); ++i) {
            double x = ir.getGeometry(i).getX();
            double y = ir.getGeometry(i).getY();
            rotated_int.addPoint(f2c::types::Point(c*x - s*y, s*x + c*y));
        }
        result.addRing(rotated_int);
    }
    return result;
}

// ========== 旋转 Swath（绕原点）==========
f2c::types::Swath rotateSwath(const f2c::types::Swath& sw, double angle)
{
    double c = std::cos(angle), s = std::sin(angle);
    double sx = sw.startPoint().getX(), sy = sw.startPoint().getY();
    double ex = sw.endPoint().getX(), ey = sw.endPoint().getY();

    f2c::types::Swath rotated(sw);
    f2c::types::LineString new_path;
    new_path.addPoint(f2c::types::Point(c*sx - s*sy, s*sx + c*sy));
    new_path.addPoint(f2c::types::Point(c*ex - s*ey, s*ex + c*ey));
    rotated.setPath(new_path);
    return rotated;
}

// ========== 检测斜边并返回最佳 swath 角度 ==========
// 场景：sweep 分解产生水平条带，若 cell 贴着斜边界（如 notched L 形斜边），
// 水平 swath 会在斜边末端形成三角缝隙。此处检测 cell 是否贴近斜边界，
// 若是则返回斜边方向作为 swath 角度，使 swath 平行于斜边以减少末端三角。
double detectSlantedBoundaryAngle(
    const f2c::types::Cell& cell,
    const f2c::types::Cell& full_polygon,
    double default_angle,
    double cov_width)
{
    const auto& cell_ring = cell.getExteriorRing();
    const auto& poly_ring = full_polygon.getExteriorRing();
    if (cell_ring.size() < 3 || poly_ring.size() < 4) return default_angle;

    // ── 收集 cell 顶点，计算 bbox ──
    std::vector<std::pair<double,double>> cpts;
    double c_min_x=1e9, c_max_x=-1e9, c_min_y=1e9, c_max_y=-1e9;
    for (size_t i = 0; i + 1 < cell_ring.size(); ++i) {
        double cx = cell_ring.getGeometry(i).getX();
        double cy = cell_ring.getGeometry(i).getY();
        cpts.push_back({cx, cy});
        if (cx < c_min_x) c_min_x = cx;
        if (cx > c_max_x) c_max_x = cx;
        if (cy < c_min_y) c_min_y = cy;
        if (cy > c_max_y) c_max_y = cy;
    }

    // ── 扫描多边形外环，找到 cell 贴近的斜边 ──
    double best_len = 0.0;
    double best_angle = default_angle;
    const double slant_threshold = 0.9659;  // cos(15°)，边与默认方向夹角>15°视为斜边

    for (size_t pi = 0; pi + 1 < poly_ring.size(); ++pi) {
        double px1 = poly_ring.getGeometry(pi).getX();
        double py1 = poly_ring.getGeometry(pi).getY();
        double px2 = poly_ring.getGeometry(pi+1).getX();
        double py2 = poly_ring.getGeometry(pi+1).getY();

        double edx = px2 - px1, edy = py2 - py1;
        double elen = std::hypot(edx, edy);
        if (elen < 0.05) continue;

        // 检查是否为斜边（15° < 与默认方向夹角 < 75°）
        double cos_slant = std::abs(edx * std::cos(default_angle) + edy * std::sin(default_angle)) / elen;
        if (cos_slant > slant_threshold) continue;  // 夹角<15°，近似平行，跳过
        if (cos_slant < 0.2588) continue;           // 夹角>75°，近似垂直，跳过

        // 边 bbox 与 cell bbox 相交？
        double e_min_x = std::min(px1, px2), e_max_x = std::max(px1, px2);
        double e_min_y = std::min(py1, py2), e_max_y = std::max(py1, py2);
        if (e_max_x < c_min_x - cov_width || e_min_x > c_max_x + cov_width ||
            e_max_y < c_min_y - cov_width || e_min_y > c_max_y + cov_width) continue;

        // cell 顶点到边的最短距离
        double min_dist = 1e9;
        for (const auto& cp : cpts) {
            double t = ((cp.first - px1)*edx + (cp.second - py1)*edy) / (elen*elen);
            t = std::max(0.0, std::min(1.0, t));
            double nx = px1 + t*edx - cp.first;
            double ny = py1 + t*edy - cp.second;
            double d = std::hypot(nx, ny);
            if (d < min_dist) min_dist = d;
        }

        if (min_dist < cov_width * 1.5 && elen > best_len) {
            best_len = elen;
            best_angle = std::atan2(edy, edx);
        }
    }

    return best_angle;
}

// ========== Swath 多角度选择 ==========
// 对单个 Cell 尝试多个候选角度，返回 swaths 数量最少的方案
f2c::types::Swaths optimizeSwathAngle(
    const f2c::types::Cell& cell,
    f2c::sg::BruteForce& swath_generator,
    double cov_width,
    const std::vector<double>& angle_candidates)
{
    if (angle_candidates.empty()) {
        double default_angle = yingshi::computeCellMainDirection(cell);
        return swath_generator.generateSwaths(default_angle, cov_width, cell);
    }

    f2c::types::Swaths best_swaths;
    size_t best_count = std::numeric_limits<size_t>::max();

    for (double angle : angle_candidates) {
        auto candidate_swaths = swath_generator.generateSwaths(angle, cov_width, cell);
        size_t count = candidate_swaths.size();

        if (count > 0 && count < best_count) {
            best_count = count;
            best_swaths = candidate_swaths;
        }
    }

    return best_swaths;
}

// ========== 全 Cell Swath 生成 ==========
// 从 ROS 节点 planCoveragePath 提取的纯算法函数。
// 全局角度优化：测试所有候选角度，选总 swath 数最少的，所有 cell 用同一角度。
// 非全局时：逐 cell 独立 computeCellMainDirection + optimizeSwathAngle。
f2c::types::SwathsByCells generateSwathsForAllCells(
    const f2c::types::Cells& no_hl,
    const f2c::types::Cell& full_polygon,
    double r_w,
    double coverage_width,
    double swath_endpoint_shrink_distance,
    double min_swath_length,
    bool swath_angle_optimization,
    const std::vector<double>& swath_angle_candidates,
    double boundary_fill_offset,
    bool use_sweep_decomp)
{
    f2c::sg::BruteForce swath_gen;
    swath_gen.setAllowOverlap(true);
    f2c::types::SwathsByCells swaths_by_cells;

    if (swath_angle_optimization) {
        // ★ 全局优化：对所有 cell 测试同一角度，选总 swath 数最少的
        std::vector<double> cands = swath_angle_candidates;
        // 收集所有 cell 的边缘角度（全局候选）
        for (size_t ci = 0; ci < no_hl.size(); ++ci) {
            auto edge_cands = extractEdgeAngles(no_hl.getGeometry(ci), 2.0);
            cands.insert(cands.end(), edge_cands.begin(), edge_cands.end());
        }
        // 去重
        std::sort(cands.begin(), cands.end());
        cands.erase(std::unique(cands.begin(), cands.end(),
            [](double a, double b) { return std::abs(a-b) < 1e-4; }),
            cands.end());

        double best_ang = 0.0;  // 提升作用域，用于离群 veto
        if (!cands.empty()) {
            size_t best_total = std::numeric_limits<size_t>::max();
            std::vector<f2c::types::Swaths> best_cell_swaths;
            best_cell_swaths.reserve(no_hl.size());

            for (double ang : cands) {
                size_t total = 0;
                bool covers_all_cells = true;
                std::vector<f2c::types::Swaths> cell_swaths;
                cell_swaths.reserve(no_hl.size());
                for (size_t ci = 0; ci < no_hl.size(); ++ci) {
                    auto cs = swath_gen.generateSwaths(ang, r_w,
                        no_hl.getGeometry(ci));
                    size_t removed_count = 0;
                    cs = filterShortSwaths(
                        cs, min_swath_length, removed_count);
                    if (cs.size() == 0) {
                        covers_all_cells = false;
                        break;
                    }
                    total += cs.size();
                    cell_swaths.push_back(std::move(cs));
                }
                if (covers_all_cells && total < best_total) {
                    best_total = total;
                    best_ang = ang;
                    best_cell_swaths = std::move(cell_swaths);
                }
            }

            // 用最优全局角度生成 swaths + 边界间隙补填
            for (size_t ci = 0; ci < best_cell_swaths.size(); ++ci) {
                f2c::types::Swaths cs = best_cell_swaths[ci];
                fillBoundaryGaps(
                    cs, no_hl.getGeometry(ci), full_polygon,
                    best_ang, coverage_width,
                    swath_endpoint_shrink_distance, 0.0,
                    boundary_fill_offset);
                swaths_by_cells.push_back(cs);
            }

            // ★ 方向 B：全局收边 + per-cell 离群 veto
            // 全局角度选定后逐 cell 检查该角度是否对该 cell 离群
            if (swaths_by_cells.size() > 0) {
                for (size_t ci = 0; ci < no_hl.size(); ++ci) {
                    const auto& cell = no_hl.getGeometry(ci);
                    size_t global_cnt = swaths_by_cells.at(ci).size();
                    if (global_cnt == 0) continue;

                    // 1. 计算该 cell 的本地最优角度
                    double local_ang = computeCellMainDirection(cell);
                    if (use_sweep_decomp && full_polygon.size() > 0) {
                        double slant_ang = detectSlantedBoundaryAngle(
                            cell, full_polygon, local_ang, coverage_width);
                        if (std::abs(slant_ang - local_ang) > 0.05) {
                            local_ang = slant_ang;
                        }
                    }

                    // 2. 在本地候选角度中选最少 swath 数
                    std::vector<double> local_cands;
                    auto edge_angs = extractEdgeAngles(cell, 2.0);
                    local_cands.insert(local_cands.end(),
                        edge_angs.begin(), edge_angs.end());
                    for (double a : swath_angle_candidates) {
                        local_cands.push_back(a);
                    }
                    local_cands.push_back(local_ang);

                    auto local_swaths = optimizeSwathAngle(
                        cell, swath_gen, r_w, local_cands);
                    size_t local_best = local_swaths.size();

                    // 3. 角度偏差 guard：本地最优角偏离全局 > 30° 时不 override
                    //    防止 cell 间方向不一致导致连接段混乱
                    double ang_diff = std::abs(local_ang - best_ang);
                    if (ang_diff > M_PI / 2.0) ang_diff = M_PI - ang_diff;
                    if (ang_diff > M_PI / 6.0) continue;  // > 30°

                    // 4. 离群判断：全局角度 swath 数比本地最优多 100%+ 且多至少 3 条
                    if (local_best > 0 && global_cnt > local_best * 2.0
                        && global_cnt - local_best >= 3) {
                        auto cs = local_swaths;
                        fillBoundaryGaps(cs, cell, full_polygon,
                            local_ang, coverage_width,
                            swath_endpoint_shrink_distance, 0.0,
                            boundary_fill_offset);
                        size_t rm = 0;
                        cs = filterShortSwaths(cs, min_swath_length, rm);
                        if (cs.size() > 0) {
                            swaths_by_cells.at(ci) = cs;
                        }
                    }
                }
            }
        }
    }

    // 全局优化未生效时，走逐 cell 独立优化路径
    if (swaths_by_cells.sizeTotal() == 0) {
        f2c::types::SwathsByCells per_cell_swaths;
        for (size_t ci = 0; ci < no_hl.size(); ++ci) {
            const auto& sub = no_hl.getGeometry(ci);
            double ang = computeCellMainDirection(sub);

            // 斜边感知：sweep 分解产生的 cell 贴着斜边界时调整 swath 角度
            if (use_sweep_decomp && full_polygon.size() > 0) {
                double slant_ang = detectSlantedBoundaryAngle(sub, full_polygon, ang, coverage_width);
                if (std::abs(slant_ang - ang) > 0.05) ang = slant_ang;
            }

            f2c::types::Swaths cs;
            if (swath_angle_optimization) {
                auto cands = extractEdgeAngles(sub, 2.0);
                for (double a : swath_angle_candidates) cands.push_back(a);
                cands.push_back(ang);
                cs = optimizeSwathAngle(sub, swath_gen, r_w, cands);
            } else {
                cs = swath_gen.generateSwaths(ang, r_w, sub);
            }
            size_t rm = 0;
            cs = filterShortSwaths(cs, min_swath_length, rm);
            fillBoundaryGaps(
                cs, sub, full_polygon, ang, coverage_width,
                swath_endpoint_shrink_distance, 0.0,
                boundary_fill_offset);
            if (cs.size() == 0) {
                return f2c::types::SwathsByCells();
            }
            per_cell_swaths.push_back(cs);
        }
        swaths_by_cells = per_cell_swaths;
    }

    if (no_hl.size() == 1 && swaths_by_cells.size() == 1) {
        rebalanceNarrowCellSwaths(
            swaths_by_cells.at(0), no_hl.getGeometry(0), coverage_width);
    }

    // ── 孔洞端点安全收缩 ──
    // swath 端点贴孔洞边界会导致下游连接修复沿孔洞走，
    // 内缩端点留出转向空间，让 connection repair 有操作余地。
    if (full_polygon.size() > 1) {
        shrinkSwathEndpointsFromHoles(
            swaths_by_cells, full_polygon, r_w * 0.25);
    }

    return swaths_by_cells;
}

// ========== 孔洞端点安全收缩 ==========
// 对每个 swath 端点检查到孔洞边缘的最小距离，
// 若小于 hole_clearance 则沿 swath 方向向内收缩至安全距离。
void shrinkSwathEndpointsFromHoles(
    f2c::types::SwathsByCells& swaths_by_cells,
    const f2c::types::Cell& full_polygon,
    double hole_clearance)
{
    if (hole_clearance <= 0.0) return;
    if (full_polygon.size() <= 1) return;  // 无孔洞

    // 收集所有孔洞环
    std::vector<const f2c::types::LinearRing*> hole_rings;
    for (size_t hi = 0; hi + 1 < full_polygon.size(); ++hi) {
        hole_rings.push_back(&full_polygon.getInteriorRing(hi));
    }
    if (hole_rings.empty()) return;

    for (size_t ci = 0; ci < swaths_by_cells.size(); ++ci) {
        auto& cell_swaths = swaths_by_cells.at(ci);
        for (size_t si = 0; si < cell_swaths.size(); ++si) {
            auto& swath = cell_swaths.at(si);
            double sx = swath.startPoint().getX();
            double sy = swath.startPoint().getY();
            double ex = swath.endPoint().getX();
            double ey = swath.endPoint().getY();
            double swath_len = std::hypot(ex - sx, ey - sy);
            if (swath_len < 1e-9) continue;

            double unit_dx = (ex - sx) / swath_len;
            double unit_dy = (ey - sy) / swath_len;

            // 对起点和终点分别检查
            for (int ep = 0; ep < 2; ++ep) {
                double px = (ep == 0) ? sx : ex;
                double py = (ep == 0) ? sy : ey;

                // 计算端点到最近孔洞边缘的距离
                double min_dist = std::numeric_limits<double>::max();
                for (const auto* ring : hole_rings) {
                    for (size_t vi = 0; vi + 1 < ring->size(); ++vi) {
                        double ax = ring->getGeometry(vi).getX();
                        double ay = ring->getGeometry(vi).getY();
                        double bx = ring->getGeometry(vi + 1).getX();
                        double by = ring->getGeometry(vi + 1).getY();

                        double edx = bx - ax, edy = by - ay;
                        double ed_len2 = edx * edx + edy * edy;
                        if (ed_len2 < 1e-12) {
                            min_dist = std::min(min_dist,
                                std::hypot(px - ax, py - ay));
                            continue;
                        }
                        double t = ((px - ax) * edx + (py - ay) * edy) / ed_len2;
                        t = std::max(0.0, std::min(1.0, t));
                        double nx = ax + t * edx;
                        double ny = ay + t * edy;
                        min_dist = std::min(min_dist, std::hypot(px - nx, py - ny));
                    }
                }

                // 若距离不足，向内收缩
                if (min_dist < hole_clearance) {
                    double shrink = hole_clearance - min_dist;
                    // 起点向内（+方向），终点向内（-方向）
                    double sign = (ep == 0) ? 1.0 : -1.0;
                    double new_px = px + sign * shrink * unit_dx;
                    double new_py = py + sign * shrink * unit_dy;

                    // 确保收缩后 swath 仍然有效（两端距离 > 0）
                    double other_px = (ep == 0) ? ex : sx;
                    double other_py = (ep == 0) ? ey : sy;
                    double new_len = std::hypot(new_px - other_px, new_py - other_py);
                    if (new_len < 0.10) continue;  // 太短，跳过

                    if (ep == 0) {
                        sx = new_px; sy = new_py;
                    } else {
                        ex = new_px; ey = new_py;
                    }
                }
            }

            // 写回修改后的 swath
            if (std::abs(sx - swath.startPoint().getX()) > 1e-9 ||
                std::abs(sy - swath.startPoint().getY()) > 1e-9 ||
                std::abs(ex - swath.endPoint().getX()) > 1e-9 ||
                std::abs(ey - swath.endPoint().getY()) > 1e-9) {
                f2c::types::LineString new_path;
                new_path.addPoint(f2c::types::Point(sx, sy));
                new_path.addPoint(f2c::types::Point(ex, ey));
                f2c::types::Swath adjusted(new_path, swath.getWidth(),
                    swath.getId(), swath.getType());
                cell_swaths.at(si) = adjusted;
            }
        }
    }
}

}  // namespace yingshi
