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
        // 双向切割：从孔洞顶点同时收集 X 和 Y 坐标
        // Y: 外边界 + 孔洞顶点（水平切割线）
        // X: 外边界 + 孔洞顶点（垂直切割线，新增）
        // 不压缩 X 为全宽 → 孔洞附近 cell 更矩形，覆盖更完整
        collect_y(work_area.getExteriorRing());
        for (size_t ri = 0; ri + 1 < work_area.size(); ++ri) {
            collect_y(work_area.getInteriorRing(ri));
            // 同时收集孔洞顶点 X 坐标（双向切割核心）
            const auto& hr = work_area.getInteriorRing(ri);
            for (size_t vi = 0; vi + 1 < hr.size(); ++vi) {
                xs.push_back(hr.getGeometry(vi).getX());
            }
        }
        // 外边界 X 坐标
        for (size_t i = 0; i + 1 < work_area.getExteriorRing().size(); ++i) {
            xs.push_back(work_area.getExteriorRing().getGeometry(i).getX());
        }
        // 注意：不再压缩 X 为 min/max，保留孔洞顶点 X 切割线
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

    // 双向切割：保留所有顶点 X/Y 坐标作为切割线
    // 不再压缩 X 为全宽 → 孔洞附近生成垂直切割线，cell 更矩形

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
