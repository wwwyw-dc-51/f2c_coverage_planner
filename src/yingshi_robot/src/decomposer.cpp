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
#include <cstdio>
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
        // Sweep 扫描线分解：仅用孔洞顶点 Y 做水平切割线
        // X 压缩为 min/max（全宽条带），不做垂直切割
        collect_y(work_area.getExteriorRing());
        for (size_t ri = 0; ri + 1 < work_area.size(); ++ri) {
            collect_y(work_area.getInteriorRing(ri));
        }
        // X 只保留外边界 min/max，不收集孔洞顶点（回退双向切割）
        for (size_t i = 0; i + 1 < work_area.getExteriorRing().size(); ++i) {
            xs.push_back(work_area.getExteriorRing().getGeometry(i).getX());
        }
        // 压缩为 min/max（全宽条带）
        auto [xmin_it, xmax_it] = std::minmax_element(xs.begin(), xs.end());
        double x_min = *xmin_it, x_max = *xmax_it;
        xs.clear();
        xs.push_back(x_min);
        xs.push_back(x_max);
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

    // Sweep 模式：X 压缩为 min/max 全宽条带，Y 用孔洞顶点做水平切割线
    // 非 sweep 模式（else 分支）：保留所有顶点 X/Y 做双向切割

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

// ========== 共享边检测 ==========
// 检测两个外环是否有共线且重叠的边段，返回总重叠长度
double sharedEdgeLength(
    const f2c::types::LinearRing& ring_a,
    const f2c::types::LinearRing& ring_b,
    double collinear_angle_tol_rad,
    double collinear_dist_tol)
{
    double total_overlap = 0.0;

    // 角度容差 → dot product 容差：||dot| - 1| < 1 - cos(angle_tol)
    const double dot_tol = 1.0 - std::cos(collinear_angle_tol_rad);

    // 遍历 ring_a 的每条边 (pa → pa_next)
    for (size_t ia = 0; ia + 1 < ring_a.size(); ++ia) {
        const double ax1 = ring_a.getGeometry(ia).getX();
        const double ay1 = ring_a.getGeometry(ia).getY();
        const double ax2 = ring_a.getGeometry(ia + 1).getX();
        const double ay2 = ring_a.getGeometry(ia + 1).getY();

        const double adx = ax2 - ax1;
        const double ady = ay2 - ay1;
        const double alen = std::hypot(adx, ady);
        if (alen < 1e-9) continue;

        // ring_a 边的方向单位向量
        const double aux = adx / alen;
        const double auy = ady / alen;

        // 遍历 ring_b 的每条边
        for (size_t ib = 0; ib + 1 < ring_b.size(); ++ib) {
            const double bx1 = ring_b.getGeometry(ib).getX();
            const double by1 = ring_b.getGeometry(ib).getY();
            const double bx2 = ring_b.getGeometry(ib + 1).getX();
            const double by2 = ring_b.getGeometry(ib + 1).getY();

            const double bdx = bx2 - bx1;
            const double bdy = by2 - by1;
            const double blen = std::hypot(bdx, bdy);
            if (blen < 1e-9) continue;

            // 方向夹角检测：||dot| - 1| < 1 - cos(angle_tol)
            const double bux = bdx / blen;
            const double buy = bdy / blen;
            const double dot = aux * bux + auy * buy;

            if (std::abs(std::abs(dot) - 1.0) > dot_tol) continue;

            // 线段间最短距离检测（共线判定）
            // 从 b 线段起点到 a 线段的垂距
            const double cross_dist =
                std::abs((bx1 - ax1) * ady - (by1 - ay1) * adx) / alen;
            if (cross_dist > collinear_dist_tol) continue;

            // 投影参数化：将 b 线段投影到 a 线段方向上
            const double proj_b1 = (bx1 - ax1) * aux + (by1 - ay1) * auy;
            const double proj_b2 = (bx2 - ax1) * aux + (by2 - ay1) * auy;

            const double b_min = std::min(proj_b1, proj_b2);
            const double b_max = std::max(proj_b1, proj_b2);

            // 与 a 线段的 [0, alen] 区间取交集
            const double overlap_start = std::max(0.0, b_min);
            const double overlap_end = std::min(alen, b_max);

            if (overlap_end > overlap_start) {
                total_overlap += overlap_end - overlap_start;
            }
        }
    }

    return total_overlap;
}

CellMergeResult mergeCellsWithSimilarDirection(
    const f2c::types::Cells& cells,
    const f2c::types::Cell& full_polygon,
    double coverage_width,
    double angle_threshold_deg,
    double min_shared_edge_len)
{
    CellMergeResult merge_result;
    merge_result.cells = cells;
    if (cells.size() <= 1 || coverage_width <= 0.0) {
        return merge_result;
    }

    const size_t cell_count = cells.size();

    // 1. 计算每个 cell 的主方向
    std::vector<double> cell_directions(cell_count);
    for (size_t i = 0; i < cell_count; ++i) {
        cell_directions[i] = computeCellMainDirection(cells.getGeometry(i));
    }

    // 2. 共享边检测（替代顶点距离邻近判断）
    //    只合并真正共享边界的 cell，而非"距离近"的 cell
    std::vector<std::vector<bool>> adjacent(
        cell_count, std::vector<bool>(cell_count, false));
    size_t edge_pairs_found = 0;
    double max_edge_found = 0.0;
    for (size_t i = 0; i < cell_count; ++i) {
        const auto& ring_i = cells.getGeometry(i).getExteriorRing();
        for (size_t j = i + 1; j < cell_count; ++j) {
            const auto& ring_j = cells.getGeometry(j).getExteriorRing();
            double shared = sharedEdgeLength(ring_i, ring_j);
            if (shared > max_edge_found) max_edge_found = shared;
            if (shared >= min_shared_edge_len) {
                adjacent[i][j] = adjacent[j][i] = true;
                ++edge_pairs_found;
            }
        }
    }
    fprintf(stderr, "[merge] %zu cells, shared-edge pairs found: %zu, max_shared=%.3f, min_edge=%.3f\n",
        cell_count, edge_pairs_found, max_edge_found, min_shared_edge_len);

    // 3. 方向一致性过滤 + union-find 分组
    const double angle_threshold =
        std::clamp(angle_threshold_deg, 0.0, 90.0) * M_PI / 180.0;

    // Union-find: 每个 cell 初始指向自己
    std::vector<size_t> parent(cell_count);
    for (size_t i = 0; i < cell_count; ++i) parent[i] = i;

    std::function<size_t(size_t)> find = [&](size_t x) -> size_t {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    };
    auto unite = [&](size_t a, size_t b) {
        size_t ra = find(a), rb = find(b);
        if (ra != rb) parent[ra] = rb;
    };

    for (size_t i = 0; i < cell_count; ++i) {
        for (size_t j = i + 1; j < cell_count; ++j) {
            if (!adjacent[i][j]) continue;

            // 方向一致性
            double angle_i = cell_directions[i];
            while (angle_i < 0.0) angle_i += M_PI;
            while (angle_i >= M_PI) angle_i -= M_PI;
            double angle_j = cell_directions[j];
            while (angle_j < 0.0) angle_j += M_PI;
            while (angle_j >= M_PI) angle_j -= M_PI;
            double angle_diff = std::abs(angle_i - angle_j);
            if (angle_diff > M_PI / 2.0) angle_diff = M_PI - angle_diff;
            if (angle_diff >= angle_threshold) continue;

            // 通过共享边 + 方向检测 → 候选合并
            unite(i, j);
        }
    }

    // 4. 按 union-find 根分组
    std::vector<std::vector<size_t>> groups(cell_count);
    for (size_t i = 0; i < cell_count; ++i) {
        groups[find(i)].push_back(i);
    }

    // 5. 逐组合并 + 几何验证门
    f2c::types::Cells merged_cells;
    for (const auto& group : groups) {
        if (group.empty()) continue;

        if (group.size() == 1) {
            // 孤立 cell，不需要合并
            merged_cells.addGeometry(cells.getGeometry(group[0]));
            continue;
        }

        // 组内按面积降序排列，大 cell 优先作为合并基
        std::vector<size_t> sorted_group = group;
        std::sort(sorted_group.begin(), sorted_group.end(),
            [&](size_t a, size_t b) {
                return cells.getGeometry(a).area() >
                       cells.getGeometry(b).area();
            });

        f2c::types::Cell merged_cell = cells.getGeometry(sorted_group[0]);
        size_t successful_merges = 0;
        bool group_valid = true;

        for (size_t k = 1; k < sorted_group.size() && group_valid; ++k) {
            const auto& other = cells.getGeometry(sorted_group[k]);
            f2c::types::Cells temporary(merged_cell);
            auto union_result = temporary.unionOp(other);

            if (union_result.size() == 0) {
                // 拓扑运算失败 → 保留原始 cell 为独立单元
                merged_cells.addGeometry(other);
                continue;
            }

            // ★ 几何验证门：检查合并后的 cell 是否包进了孔洞
            //    Cell::size() 返回 ring 数量，> 1 表示有 interior ring
            const auto& result_cell = union_result.getGeometry(0);
            if (result_cell.size() > 1) {
                // 合并产生了 interior ring → 包进了孔洞 → 拒绝！
                merged_cells.addGeometry(other);
                continue;
            }

            ++successful_merges;
            merged_cell = result_cell;

            // 其余独立分量作为独立 Cell 输出
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
    fprintf(stderr, "[merge] %zu cells → %zu cells (%zu merged, min_edge=%.3f)\n",
        cell_count, merged_cells.size(), merge_result.merged_count,
        min_shared_edge_len);
    return merge_result;
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
