/**
 * @file decomposer.cpp
 * @brief 区域分解模块实现 — Sweep 扫描线分解 + 几何工具函数
 *
 * 从 polygon_planner_node.cpp 提取，属于模块化重构 Step 2。
 * 所有函数均为纯几何计算，不依赖 ROS2。
 */

#include "yingshi_robot/decomposer.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>

namespace yingshi {

// ========== 主分解函数 ==========
// Sweep 扫描线分解：以孔洞顶点 y 坐标做水平切分线，生成全宽条带 Cells
// 相比 Boustrophedon 网格分解，cell 数大幅减少（S3: 42→7）
f2c::types::Cells rectilinearDecompose(
    const f2c::types::Cell& work_area,
    const f2c::types::Cell& grid_src,
    const DecomposerParams& params)
{
    (void)grid_src;  // 不再使用原始边界，以生成边界为准

    // 1. 收集坐标
    std::vector<double> xs, ys;
    auto collect_xy = [&](const f2c::types::LinearRing& ring) {
        for (size_t i = 0; i + 1 < ring.size(); ++i) {
            xs.push_back(ring.getGeometry(i).getX());
            ys.push_back(ring.getGeometry(i).getY());
        }
    };
    auto collect_y = [&](const f2c::types::LinearRing& ring) {
        for (size_t i = 0; i + 1 < ring.size(); ++i)
            ys.push_back(ring.getGeometry(i).getY());
    };

    if (params.use_sweep) {
        // Sweep 扫描线分解：以孔洞顶点 Y 做水平切割线，孔洞 X 边界做垂直切割线
        // 形成对齐孔洞包围盒的网格，避免全宽条带被 difference 碎片化为孤立小块
        collect_y(work_area.getExteriorRing());
        for (size_t ri = 0; ri + 1 < work_area.size(); ++ri) {
            collect_y(work_area.getInteriorRing(ri));
        }
        // X 保留外边界 min/max
        for (size_t i = 0; i + 1 < work_area.getExteriorRing().size(); ++i) {
            xs.push_back(work_area.getExteriorRing().getGeometry(i).getX());
        }
        auto [xmin_it, xmax_it] = std::minmax_element(xs.begin(), xs.end());
        double x_min = *xmin_it, x_max = *xmax_it;
        xs.clear();
        xs.push_back(x_min);
        xs.push_back(x_max);
        // 收集孔洞 bbox，判定是否启用 X 边界切割
        // 条件：存在 Y 重叠但 X 不重叠的孔洞对 → 同一"货架行"多列
        // 仅在密集列孔洞场景下加 X 切割，避免稀疏/不规则孔洞场景退化
        if (work_area.size() > 1) {
            struct HoleBBox { double xmin, xmax, ymin, ymax; };
            std::vector<HoleBBox> hole_bboxes;
            for (size_t ri = 0; ri + 1 < work_area.size(); ++ri) {
                const auto& hr = work_area.getInteriorRing(ri);
                double hx_min = 1e9, hx_max = -1e9, hy_min = 1e9, hy_max = -1e9;
                for (size_t vi = 0; vi + 1 < hr.size(); ++vi) {
                    double hx = hr.getGeometry(vi).getX();
                    double hy = hr.getGeometry(vi).getY();
                    if (hx < hx_min) hx_min = hx;
                    if (hx > hx_max) hx_max = hx;
                    if (hy < hy_min) hy_min = hy;
                    if (hy > hy_max) hy_max = hy;
                }
                if (hx_min < hx_max && hy_min < hy_max)
                    hole_bboxes.push_back({hx_min, hx_max, hy_min, hy_max});
            }
            // 按 Y 范围聚类孔洞，计数每行的列数
            // 条件：≥2 行 × ≥2 列，或单行 ≥3 列 → 密集货架网格
            std::vector<bool> visited(hole_bboxes.size(), false);
            int multi_col_rows = 0;
            int max_cols_in_row = 0;
            for (size_t i = 0; i < hole_bboxes.size(); ++i) {
                if (visited[i]) continue;
                double row_ymin = hole_bboxes[i].ymin;
                double row_ymax = hole_bboxes[i].ymax;
                std::vector<size_t> row_holes;
                for (size_t j = i; j < hole_bboxes.size(); ++j) {
                    if (visited[j]) continue;
                    if (hole_bboxes[j].ymax > row_ymin &&
                        row_ymax > hole_bboxes[j].ymin) {
                        row_holes.push_back(j);
                        visited[j] = true;
                        row_ymin = std::min(row_ymin, hole_bboxes[j].ymin);
                        row_ymax = std::max(row_ymax, hole_bboxes[j].ymax);
                    }
                }
                int cols = 0;
                for (size_t a = 0; a < row_holes.size(); ++a) {
                    bool is_new_col = true;
                    for (size_t b = 0; b < a && is_new_col; ++b) {
                        size_t ia = row_holes[a], ib = row_holes[b];
                        if (!(hole_bboxes[ia].xmax < hole_bboxes[ib].xmin ||
                              hole_bboxes[ib].xmax < hole_bboxes[ia].xmin))
                            is_new_col = false;
                    }
                    if (is_new_col) ++cols;
                }
                max_cols_in_row = std::max(max_cols_in_row, cols);
                if (cols >= 2) ++multi_col_rows;
            }
            bool has_dense_columns = (multi_col_rows >= 2) || (max_cols_in_row >= 3);
            if (has_dense_columns) {
                for (const auto& hb : hole_bboxes) {
                    xs.push_back(hb.xmin);
                    xs.push_back(hb.xmax);
                }
            }
        }
    } else {
        collect_xy(work_area.getExteriorRing());
        for (size_t ri = 0; ri + 1 < work_area.size(); ++ri)
            collect_xy(work_area.getInteriorRing(ri));
    }

    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    auto dedup = [](std::vector<double>& v) {
        v.erase(std::unique(v.begin(), v.end(),
            [](double a, double b) { return std::abs(a-b) < 1e-6; }), v.end());
    };
    dedup(xs);
    dedup(ys);

    if (ys.size() < 2) {
        f2c::types::Cells fallback;
        fallback.addGeometry(work_area);
        return fallback;
    }

    // Sweep 模式：X=外边界+孔洞边界, Y=外环顶点+孔洞顶点，形成对齐孔洞的网格
    // 非 sweep 模式（else 分支）：保留所有顶点 X/Y 做全量双向切割

    if (xs.size() < 2) {
        f2c::types::Cells fallback;
        fallback.addGeometry(work_area);
        return fallback;
    }

    // 2. 创建网格/条带矩形，与 work_area 求交
    f2c::types::Cells result;
    for (size_t i = 0; i + 1 < xs.size(); ++i) {
        for (size_t j = 0; j + 1 < ys.size(); ++j) {
            double x0 = xs[i], x1 = xs[i+1];
            double y0 = ys[j], y1 = ys[j+1];
            double cw = x1 - x0, ch = y1 - y0;
            if (cw < 0.01 || ch < 0.01) continue;

            f2c::types::LinearRing ring;
            ring.addPoint(f2c::types::Point(x0, y0));
            ring.addPoint(f2c::types::Point(x1, y0));
            ring.addPoint(f2c::types::Point(x1, y1));
            ring.addPoint(f2c::types::Point(x0, y1));
            ring.addPoint(f2c::types::Point(x0, y0));

            f2c::types::Cell grid_cell;
            grid_cell.addRing(ring);

            auto intersected = f2c::types::Cells::intersection(grid_cell, work_area);
            for (size_t ci = 0; ci < intersected.size(); ++ci) {
                f2c::types::Cell cell = intersected.getGeometry(ci);
                if (cell.size() > 1) {
                    f2c::types::Cells single;
                    single.addGeometry(cell);
                    for (size_t hi = 0; hi + 1 < cell.size(); ++hi) {
                        f2c::types::Cell hole_cell;
                        hole_cell.addRing(cell.getInteriorRing(hi));
                        f2c::types::Cells hole_cells;
                        hole_cells.addGeometry(hole_cell);
                        hole_cells = hole_cells.buffer(0.001);
                        single = single.difference(hole_cells);
                    }
                    for (size_t si = 0; si < single.size(); ++si)
                        result.addGeometry(single.getGeometry(si));
                } else {
                    result.addGeometry(cell);
                }
            }
        }
    }
    if (result.size() == 0) result.addGeometry(work_area);

    // Sweep模式后处理：对贴近孔洞的cell做显式减法（防止intersection遗漏小孔洞）
    if (params.use_sweep && work_area.size() > 1) {
        for (size_t hi = 0; hi + 1 < work_area.size(); ++hi) {
            const auto& hr = work_area.getInteriorRing(hi);
            double h_min_x=1e9, h_max_x=-1e9, h_min_y=1e9, h_max_y=-1e9;
            for (size_t vi = 0; vi + 1 < hr.size(); ++vi) {
                double hx=hr.getGeometry(vi).getX(), hy=hr.getGeometry(vi).getY();
                if(hx<h_min_x)h_min_x=hx;
                if(hx>h_max_x)h_max_x=hx;
                if(hy<h_min_y)h_min_y=hy;
                if(hy>h_max_y)h_max_y=hy;
            }
            f2c::types::Cell hole_cell; hole_cell.addRing(hr);
            f2c::types::Cells hole_cells; hole_cells.addGeometry(hole_cell);
            hole_cells = hole_cells.buffer(0.001);

            f2c::types::Cells cleaned;
            for (size_t ci = 0; ci < result.size(); ++ci) {
                auto mc = result.getGeometry(ci);
                const auto& mr = mc.getExteriorRing();
                double c_min_x=1e9,c_max_x=-1e9,c_min_y=1e9,c_max_y=-1e9;
                for(size_t vi=0;vi+1<mr.size();++vi){
                    double cx=mr.getGeometry(vi).getX(),cy=mr.getGeometry(vi).getY();
                    if(cx<c_min_x)c_min_x=cx;
                    if(cx>c_max_x)c_max_x=cx;
                    if(cy<c_min_y)c_min_y=cy;
                    if(cy>c_max_y)c_max_y=cy;
                }
                // 只对与孔洞bbox重叠的cell做减法
                bool overlaps = (c_min_x < h_max_x && c_max_x > h_min_x &&
                                c_min_y < h_max_y && c_max_y > h_min_y);
                if (overlaps) {
                    f2c::types::Cells tmp; tmp.addGeometry(mc);
                    auto diff = tmp.difference(hole_cells);
                    for (size_t di = 0; di < diff.size(); ++di)
                        cleaned.addGeometry(diff.getGeometry(di));
                } else {
                    cleaned.addGeometry(mc);
                }
            }
            if (cleaned.size() > 0) result = cleaned;
        }
    }

    return result;
}

// ========== 简化环：移除共线冗余顶点 ==========
// OGR buffer 操作会在直线段上插入多余顶点 → swath 角度微偏 + 连接路径蜿蜒
// 移除环上夹角接近 180° 的共线冗余顶点，恢复规整多边形
f2c::types::LinearRing simplifyRing(
    const f2c::types::LinearRing& ring,
    double angle_tol_deg)
{
    if (ring.size() < 4) return ring;
    const double tol_rad = angle_tol_deg * M_PI / 180.0;

    // 检测是否首尾闭合（OGR LinearRing 要求闭合）
    bool closed = (std::abs(ring.getGeometry(0).getX() -
                            ring.getGeometry(ring.size()-1).getX()) < 1e-9 &&
                   std::abs(ring.getGeometry(0).getY() -
                            ring.getGeometry(ring.size()-1).getY()) < 1e-9);
    size_t n_unique = closed ? ring.size() - 1 : ring.size();

    f2c::types::LinearRing out;
    for (size_t i = 0; i < n_unique; ++i) {
        size_t prev_i = (i == 0) ? n_unique - 1 : i - 1;
        size_t next_i = (i == n_unique - 1) ? 0 : i + 1;
        double dx1 = ring.getGeometry(i).getX() - ring.getGeometry(prev_i).getX();
        double dy1 = ring.getGeometry(i).getY() - ring.getGeometry(prev_i).getY();
        double dx2 = ring.getGeometry(next_i).getX() - ring.getGeometry(i).getX();
        double dy2 = ring.getGeometry(next_i).getY() - ring.getGeometry(i).getY();
        double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
        double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
        if (len1 < 1e-9 || len2 < 1e-9) continue;
        double cross = dx1*dy2 - dy1*dx2;
        double sin_angle = std::abs(cross) / (len1 * len2);
        if (sin_angle > std::sin(tol_rad)) {
            out.addPoint(ring.getGeometry(i));
        }
    }
    if (out.size() > 0) {
        out.addPoint(out.getGeometry(0));  // 闭合环
    }
    return (out.size() >= 4) ? out : ring;
}

// ========== 批量简化 Cells ==========
f2c::types::Cells simplifyCells(
    const f2c::types::Cells& cells,
    double angle_tol_deg)
{
    f2c::types::Cells out;
    for (size_t ci = 0; ci < cells.size(); ++ci) {
        f2c::types::Cell c = cells.getGeometry(ci);
        f2c::types::Cell clean;
        clean.addRing(simplifyRing(c.getExteriorRing(), angle_tol_deg));
        // 保留内部环（孔洞），用相同阈值
        for (size_t ri = 0; ri < c.size() - 1; ++ri) {
            clean.addRing(simplifyRing(c.getInteriorRing(ri), angle_tol_deg));
        }
        out.addGeometry(clean);
    }
    return out;
}

// ========== 计算 Cell 主方向（最长边缘方向）==========
double computeCellMainDirection(const f2c::types::Cell& cell)
{
    const auto& exterior_ring = cell.getExteriorRing();

    if (exterior_ring.size() < 2) {
        return 0.0;
    }

    double max_length = 0.0;
    double main_angle = 0.0;

    // 遍历外环的所有边
    for (size_t i = 0; i < exterior_ring.size(); ++i) {
        size_t next_i = (i + 1) % exterior_ring.size();

        const auto& p1 = exterior_ring.getGeometry(i);
        const auto& p2 = exterior_ring.getGeometry(next_i);

        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();
        double length = std::sqrt(dx * dx + dy * dy);

        if (length > max_length) {
            max_length = length;
            main_angle = std::atan2(dy, dx);
        }
    }

    return main_angle;
}

// ========== 过滤面积过小的 cell ==========
f2c::types::Cells filterTinyCells(
    const f2c::types::Cells& cells,
    double min_area)
{
    if (min_area <= 0.0) return cells;

    f2c::types::Cells filtered;
    for (size_t i = 0; i < cells.size(); ++i) {
        const auto& cell = cells.getGeometry(i);
        double area = cell.area();
        if (area >= min_area) {
            filtered.addGeometry(cell);
        }
    }
    return filtered;
}

CellMergeResult mergeCellsWithSimilarDirection(
    const f2c::types::Cells& cells,
    const f2c::types::Cell& full_polygon,
    double coverage_width,
    double angle_threshold_deg,
    bool protect_hole_separation)
{
    CellMergeResult merge_result;
    merge_result.cells = cells;
    if (cells.size() <= 1 || coverage_width <= 0.0) {
        return merge_result;
    }

    const size_t cell_count = cells.size();
    std::vector<double> cell_directions(cell_count);
    for (size_t i = 0; i < cell_count; ++i) {
        cell_directions[i] = computeCellMainDirection(cells.getGeometry(i));
    }

    std::vector<std::vector<bool>> adjacent(
        cell_count, std::vector<bool>(cell_count, false));
    for (size_t i = 0; i < cell_count; ++i) {
        const auto& ring_i = cells.getGeometry(i).getExteriorRing();
        for (size_t j = i + 1; j < cell_count; ++j) {
            const auto& ring_j = cells.getGeometry(j).getExteriorRing();
            double min_distance = std::numeric_limits<double>::max();
            for (size_t pi = 0; pi < ring_i.size(); ++pi) {
                const double ix = ring_i.getGeometry(pi).getX();
                const double iy = ring_i.getGeometry(pi).getY();
                for (size_t pj = 0; pj < ring_j.size(); ++pj) {
                    min_distance = std::min(
                        min_distance,
                        std::hypot(
                            ix - ring_j.getGeometry(pj).getX(),
                            iy - ring_j.getGeometry(pj).getY()));
                }
            }
            adjacent[i][j] = adjacent[j][i] =
                min_distance < 2.0 * coverage_width;
        }
    }

    const double angle_threshold =
        std::clamp(angle_threshold_deg, 0.0, 90.0) * M_PI / 180.0;
    std::vector<int> group(cell_count, -1);
    int next_group = 0;
    for (size_t i = 0; i < cell_count; ++i) {
        if (group[i] >= 0) continue;
        group[i] = next_group++;

        for (size_t j = i + 1; j < cell_count; ++j) {
            if (group[j] >= 0 || !adjacent[i][j]) continue;

            double angle_i = cell_directions[i];
            while (angle_i < 0.0) angle_i += M_PI;
            while (angle_i >= M_PI) angle_i -= M_PI;
            double angle_j = cell_directions[j];
            while (angle_j < 0.0) angle_j += M_PI;
            while (angle_j >= M_PI) angle_j -= M_PI;
            double angle_diff = std::abs(angle_i - angle_j);
            if (angle_diff > M_PI / 2.0) angle_diff = M_PI - angle_diff;
            if (angle_diff >= angle_threshold) continue;

            if (protect_hole_separation && full_polygon.size() > 1) {
                const auto& ring_i =
                    cells.getGeometry(i).getExteriorRing();
                const auto& ring_j =
                    cells.getGeometry(j).getExteriorRing();

                auto ringCentroid = [](const f2c::types::LinearRing& ring) {
                    if (ring.size() <= 1) {
                        return f2c::types::Point(0.0, 0.0);
                    }
                    const size_t count = ring.size() - 1;
                    double x = 0.0;
                    double y = 0.0;
                    for (size_t point_idx = 0; point_idx < count;
                         ++point_idx) {
                        x += ring.getGeometry(point_idx).getX();
                        y += ring.getGeometry(point_idx).getY();
                    }
                    return f2c::types::Point(x / count, y / count);
                };

                const auto centroid_i = ringCentroid(ring_i);
                const auto centroid_j = ringCentroid(ring_j);
                bool crosses_hole = false;
                for (size_t hole_idx = 0;
                     hole_idx + 1 < full_polygon.size() && !crosses_hole;
                     ++hole_idx) {
                    const auto& hole =
                        full_polygon.getInteriorRing(hole_idx);
                    for (size_t edge_idx = 0;
                         edge_idx + 1 < hole.size() && !crosses_hole;
                         ++edge_idx) {
                        const double ax = hole.getGeometry(edge_idx).getX();
                        const double ay = hole.getGeometry(edge_idx).getY();
                        const double bx =
                            hole.getGeometry(edge_idx + 1).getX();
                        const double by =
                            hole.getGeometry(edge_idx + 1).getY();
                        const double dx =
                            centroid_j.getX() - centroid_i.getX();
                        const double dy =
                            centroid_j.getY() - centroid_i.getY();
                        const double denominator =
                            dx * (by - ay) - dy * (bx - ax);
                        if (std::abs(denominator) < 1e-18) continue;
                        const double t =
                            ((ax - centroid_i.getX()) * (by - ay) -
                             (ay - centroid_i.getY()) * (bx - ax)) /
                            denominator;
                        const double u =
                            ((ax - centroid_i.getX()) * dy -
                             (ay - centroid_i.getY()) * dx) /
                            denominator;
                        if (t > 0.001 && t < 0.999 &&
                            u > 0.0 && u < 1.0) {
                            crosses_hole = true;
                        }
                    }
                }

                if (crosses_hole) {
                    double touch_distance =
                        std::numeric_limits<double>::max();
                    for (size_t pi = 0; pi + 1 < ring_i.size(); ++pi) {
                        const double ix = ring_i.getGeometry(pi).getX();
                        const double iy = ring_i.getGeometry(pi).getY();
                        for (size_t pj = 0; pj + 1 < ring_j.size(); ++pj) {
                            touch_distance = std::min(
                                touch_distance,
                                std::hypot(
                                    ix - ring_j.getGeometry(pj).getX(),
                                    iy - ring_j.getGeometry(pj).getY()));
                        }
                    }
                    if (touch_distance >= coverage_width * 0.25) {
                        continue;
                    }
                }
            }

            group[j] = group[i];
        }
    }

    if (next_group >= static_cast<int>(cell_count)) {
        return merge_result;
    }

    f2c::types::Cells merged_cells;
    for (int group_id = 0; group_id < next_group; ++group_id) {
        f2c::types::Cell merged_cell;
        bool first = true;
        size_t successful_merges = 0;
        for (size_t i = 0; i < cell_count; ++i) {
            if (group[i] != group_id) continue;
            if (first) {
                merged_cell = cells.getGeometry(i);
                first = false;
                continue;
            }

            f2c::types::Cells temporary(merged_cell);
            auto union_result = temporary.unionOp(cells.getGeometry(i));
            if (union_result.size() == 0) {
                // 拓扑运算失败时保留当前单元，不能静默缩小作业区。
                merged_cells.addGeometry(cells.getGeometry(i));
                continue;
            }

            // 检查合并后是否产生 interior ring（孔洞被包入 cell 内部）
            // 若产生孔洞则拒绝本次合并，保留原始 cell（与 mergeAdjacentSweepStrips 一致）
            bool has_interior = false;
            for (size_t ri = 0; ri < union_result.size() && !has_interior; ++ri) {
                if (union_result.getGeometry(ri).size() > 1)
                    has_interior = true;
            }
            if (has_interior) {
                merged_cells.addGeometry(cells.getGeometry(i));
                continue;
            }

            ++successful_merges;
            merged_cell = union_result.getGeometry(0);
            // 保持 legacy 的 MultiPolygon 顺序：首块继续参与后续合并，
            // 其余分量先作为独立 Cell 输出，确保路线基线不漂移。
            for (size_t result_idx = 1;
                 result_idx < union_result.size(); ++result_idx) {
                merged_cells.addGeometry(
                    union_result.getGeometry(result_idx));
            }
        }
        merge_result.merged_count += successful_merges;
        merged_cells.addGeometry(merged_cell);
    }

    merge_result.cells = merged_cells;
    return merge_result;
}

// ========== Sweep 条带合并（同 x-span + 垂直相邻）==========
// 专攻被远端孔洞顶点误切的矩形条带：同 x-span、垂直紧邻的 cell 直接合并，
// unionCascaded 后若产生 interior ring 则回退保留原始 cell。
f2c::types::Cells mergeAdjacentSweepStrips(
    const f2c::types::Cells& cells,
    double /*coverage_width*/)
{
    if (cells.size() <= 1) return cells;

    const size_t n = cells.size();
    constexpr double kXSpanTol = 0.05;
    constexpr double kVGapTol = 0.01;

    struct Span { double xmin, xmax, ymin, ymax; };
    std::vector<Span> spans(n);
    for (size_t i = 0; i < n; ++i) {
        const auto& ring = cells.getGeometry(i).getExteriorRing();
        double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
        for (size_t j = 0; j + 1 < ring.size(); ++j) {
            double x = ring.getGeometry(j).getX();
            double y = ring.getGeometry(j).getY();
            xmin = std::min(xmin, x); xmax = std::max(xmax, x);
            ymin = std::min(ymin, y); ymax = std::max(ymax, y);
        }
        spans[i] = {xmin, xmax, ymin, ymax};
    }

    std::vector<size_t> parent(n);
    for (size_t i = 0; i < n; ++i) parent[i] = i;
    auto find = [&](size_t x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto unite = [&](size_t a, size_t b) {
        a = find(a); b = find(b);
        if (a != b) parent[b] = a;
    };

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (std::abs(spans[i].xmin - spans[j].xmin) > kXSpanTol) continue;
            if (std::abs(spans[i].xmax - spans[j].xmax) > kXSpanTol) continue;
            double vgap = std::max(spans[i].ymin - spans[j].ymax,
                                   spans[j].ymin - spans[i].ymax);
            if (vgap > kVGapTol) continue;
            unite(i, j);
        }
    }

    std::vector<size_t> group_id(n, SIZE_MAX);
    size_t group_cnt = 0;
    for (size_t i = 0; i < n; ++i) {
        size_t root = find(i);
        if (group_id[root] == SIZE_MAX) group_id[root] = group_cnt++;
    }

    f2c::types::Cells result;
    for (size_t g = 0; g < group_cnt; ++g) {
        f2c::types::Cells group_cells;
        for (size_t i = 0; i < n; ++i) {
            if (group_id[find(i)] == g)
                group_cells.addGeometry(cells.getGeometry(i));
        }
        if (group_cells.size() == 1) {
            result.addGeometry(group_cells.getGeometry(0));
        } else {
            auto merged = group_cells.unionCascaded();
            bool has_interior = false;
            for (size_t mi = 0; mi < merged.size() && !has_interior; ++mi) {
                if (merged.getGeometry(mi).size() > 1)
                    has_interior = true;
            }
            if (has_interior) {
                for (size_t i = 0; i < n; ++i) {
                    if (group_id[find(i)] == g)
                        result.addGeometry(cells.getGeometry(i));
                }
            } else {
                for (size_t mi = 0; mi < merged.size(); ++mi)
                    result.addGeometry(merged.getGeometry(mi));
            }
        }
    }
    return result;
}

// ========== 从多边形边缘提取角度候选（边缘方向去重）==========
// 依据 Rotating Calipers 定理：最优 swath 方向一定平行于多边形某条边
std::vector<double> extractEdgeAngles(
    const f2c::types::Cell& cell,
    double dedup_tolerance_deg)
{
    const auto& ring = cell.getExteriorRing();
    std::vector<double> raw_angles;

    for (size_t i = 0; i < ring.size(); ++i) {
        size_t next = (i + 1) % ring.size();
        double dx = ring.getGeometry(next).getX() - ring.getGeometry(i).getX();
        double dy = ring.getGeometry(next).getY() - ring.getGeometry(i).getY();
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-9) continue;  // 忽略零长度边
        raw_angles.push_back(std::atan2(dy, dx));
    }

    // 按角度排序后去重（相近角度合并）
    std::sort(raw_angles.begin(), raw_angles.end());
    double tol_rad = dedup_tolerance_deg * M_PI / 180.0;
    std::vector<double> deduped;
    for (double a : raw_angles) {
        if (deduped.empty() || std::abs(a - deduped.back()) > tol_rad) {
            deduped.push_back(a);
        }
    }
    // 检查首尾相邻（角度环接近 2π 的情况）
    if (deduped.size() > 1) {
        double wrap_dist = deduped.front() + 2.0 * M_PI - deduped.back();
        if (wrap_dist < tol_rad) {
            deduped.erase(deduped.begin());
        }
    }

    return deduped;
}

// ========== 提取分解角度候选（边缘垂直方向）==========
// 定理（Huang 2001）：最优 Boustrophedon 分解方向一定垂直于多边形某条边
// 即分解 sweep line 方向 = 边缘方向 + 90°
std::vector<double> extractDecompositionAngles(
    const f2c::types::Cell& polygon,
    double merge_angle_threshold_deg)
{
    auto edge_angles = extractEdgeAngles(polygon, merge_angle_threshold_deg);
    std::vector<double> decomp_angles;
    for (double a : edge_angles) {
        // 垂直方向即 +90°，归一化到 (-π, π]
        double da = a + M_PI / 2.0;
        if (da > M_PI) da -= 2.0 * M_PI;
        decomp_angles.push_back(da);
    }
    return decomp_angles;
}

}  // namespace yingshi
