/**
 * @file boundary_filler.cpp
 * @brief 边界补刀模块实现 — 外环/孔洞边界间隙补填（完整版，含 getLinesInside 裁剪）
 *
 * 从 polygon_planner_node.cpp v8/v9 成熟实现完整提取，属于模块化重构 Step 4+。
 * 包含：fillBoundaryGaps（外环+孔洞补刀）、点-in-多边形检测（射线法）、线段穿洞检测
 *
 * 算法要点（Phase 2B）：
 *   - 平行容差 cos(20°) = 0.9397，覆盖更多斜边缝隙
 *   - 法向指向 cell 质心（外环）/ 远离孔洞中心（孔洞）
 *   - getLinesInside 裁剪确保边界 swath 不超出多边形
 *   - 出发侧填充插入最前，到达侧填充追加末尾
 */

#include "yingshi_robot/boundary_filler.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace yingshi {

// ========== 射线法：点是否在多边形内部 ==========
bool pointInPolygon(double px, double py, const f2c::types::LinearRing& ring)
{
    int crossings = 0;
    size_t n = ring.size();
    for (size_t i = 0; i + 1 < n; ++i) {
        double x1 = ring.getGeometry(i).getX();
        double y1 = ring.getGeometry(i).getY();
        double x2 = ring.getGeometry(i + 1).getX();
        double y2 = ring.getGeometry(i + 1).getY();

        if ((y1 > py) != (y2 > py)) {
            double x_intersect = x1 + (py - y1) * (x2 - x1) / (y2 - y1);
            if (px < x_intersect) {
                ++crossings;
            }
        }
    }
    return (crossings % 2) == 1;
}

// ========== 点是否在任意孔洞内 ==========
bool pointInAnyHole(double px, double py,
                    const std::vector<f2c::types::LinearRing>& hole_rings)
{
    for (const auto& hr : hole_rings) {
        if (hr.size() < 3) continue;
        if (pointInPolygon(px, py, hr)) {
            return true;
        }
    }
    return false;
}

// ========== 线段是否穿过孔洞（采样检测）==========
bool segmentCrossesHole(double x0, double y0, double x1, double y1,
                        const std::vector<f2c::types::LinearRing>& hole_rings,
                        int num_samples)
{
    if (hole_rings.empty() || num_samples <= 0) return false;

    for (int k = 0; k <= num_samples; ++k) {
        double t = static_cast<double>(k) / num_samples;
        double sx = x0 + t * (x1 - x0);
        double sy = y0 + t * (y1 - y0);
        if (pointInAnyHole(sx, sy, hole_rings)) {
            return true;
        }
    }
    return false;
}

// ========== 边界间隙补填（完整版，v8/v9 成熟实现）==========
void fillBoundaryGaps(
    f2c::types::Swaths& cell_swaths,
    const f2c::types::Cell& cell,
    const f2c::types::Cell& full_polygon,
    double swath_angle,
    double cov_width,
    double shrink_dist,
    double robot_half_width)
{
    if (cell_swaths.size() == 0) return;
    const auto& cell_ring = cell.getExteriorRing();
    if (cell_ring.size() < 3) return;
    const auto& poly_ring = full_polygon.getExteriorRing();

    // 碰撞安全: 补线偏移用 robot_half_width，确保机器人不越界
    // 未提供时回退到 cov_width/2（兼容旧调用）
    double boundary_offset = (robot_half_width > 0.0) ? robot_half_width : (cov_width * 0.5);
    (void)shrink_dist;  // 端点缩进由 adjustSwathEndpoints 单独处理

    // ── cell bbox ──
    double c_min_x = 1e9, c_max_x = -1e9, c_min_y = 1e9, c_max_y = -1e9;
    for (size_t ci = 0; ci + 1 < cell_ring.size(); ++ci) {
        double cx = cell_ring.getGeometry(ci).getX();
        double cy = cell_ring.getGeometry(ci).getY();
        if (cx < c_min_x) c_min_x = cx;
        if (cx > c_max_x) c_max_x = cx;
        if (cy < c_min_y) c_min_y = cy;
        if (cy > c_max_y) c_max_y = cy;
    }

    // ── swath 方向向量（外环+孔洞共用）──
    double s_dx = std::cos(swath_angle);
    double s_dy = std::sin(swath_angle);
    // Phase 2B: 放宽容差 5°→20°，覆盖更多斜边缝隙
    const double angle_tol = 0.9397;  // cos(20°)

    // cell 质心（判断多边形内侧/孔洞外侧方向用）
    double cell_cx = 0, cell_cy = 0;
    for (size_t ci = 0; ci + 1 < cell_ring.size(); ++ci) {
        cell_cx += cell_ring.getGeometry(ci).getX();
        cell_cy += cell_ring.getGeometry(ci).getY();
    }
    cell_cx /= (cell_ring.size() - 1);
    cell_cy /= (cell_ring.size() - 1);

    // ── 已有 swath 沿法向投影范围 ──
    double proj_min = 1e9, proj_max = -1e9;
    for (size_t si = 0; si < cell_swaths.size(); ++si) {
        double cx = (cell_swaths.at(si).startPoint().getX()
                   + cell_swaths.at(si).endPoint().getX()) * 0.5;
        double cy = (cell_swaths.at(si).startPoint().getY()
                   + cell_swaths.at(si).endPoint().getY()) * 0.5;
        double proj = cx * (-s_dy) + cy * s_dx;
        if (proj < proj_min) proj_min = proj;
        if (proj > proj_max) proj_max = proj;
    }

    std::vector<f2c::types::Swath> start_fills;
    std::vector<f2c::types::Swath> end_fills;

    // ═══════════════════════════════════════════════════
    // 外环边界补刀
    // ═══════════════════════════════════════════════════
    // 移除 touches_outer 整体门控，改为逐边独立判断。
    // 原因：cell 经 GDAL 操作后顶点可能与 polygon 顶点略有偏差，
    // vertex-to-vertex 距离门控会误杀实际贴边的 cell。
    {
        for (size_t pi = 0; pi + 1 < poly_ring.size(); ++pi)
        {
            double px1 = poly_ring.getGeometry(pi).getX();
            double py1 = poly_ring.getGeometry(pi).getY();
            double px2 = poly_ring.getGeometry(pi + 1).getX();
            double py2 = poly_ring.getGeometry(pi + 1).getY();

            double edx = px2 - px1, edy = py2 - py1;
            double elen = std::hypot(edx, edy);
            if (elen < 0.01) continue;

            // 边平行于 swath 方向
            double cos_angle = std::abs(edx * s_dx + edy * s_dy) / elen;
            if (cos_angle < angle_tol) continue;

            // 边是否贴近 cell (bbox 距离)
            double e_min_x = std::min(px1, px2), e_max_x = std::max(px1, px2);
            double e_min_y = std::min(py1, py2), e_max_y = std::max(py1, py2);
            double bbox_dx = std::max(0.0, std::max(c_min_x - e_max_x, e_min_x - c_max_x));
            double bbox_dy = std::max(0.0, std::max(c_min_y - e_max_y, e_min_y - c_max_y));
            double bbox_dist = std::hypot(bbox_dx, bbox_dy);
            if (bbox_dist > cov_width * 1.5) continue;

            // 内侧方向（法向指向 cell 质心）
            double n_x = -edy / elen;
            double n_y =  edx / elen;
            double mx = (px1 + px2) * 0.5, my = (py1 + py2) * 0.5;
            if (n_x * (cell_cx - mx) + n_y * (cell_cy - my) < 0) {
                n_x = -n_x; n_y = -n_y;
            }

            // 边界 swath：多边形边向内偏移 boundary_offset（robot_half_width）
            f2c::types::LineString bline;
            bline.addPoint(f2c::types::Point(px1 + boundary_offset * n_x, py1 + boundary_offset * n_y));
            bline.addPoint(f2c::types::Point(px2 + boundary_offset * n_x, py2 + boundary_offset * n_y));

            // 裁剪到多边形内 + 排除孔洞
            f2c::types::Cells poly_tmp;
            poly_tmp.addGeometry(full_polygon);
            auto inside = poly_tmp.getLinesInside(bline);

            for (size_t li = 0; li < inside.size(); ++li) {
                auto seg = inside.getGeometry(li);
                double seg_len = std::hypot(
                    seg.getGeometry(1).getX() - seg.getGeometry(0).getX(),
                    seg.getGeometry(1).getY() - seg.getGeometry(0).getY());
                if (seg_len < cov_width * 0.5) continue;

                f2c::types::Swath fill_sw(seg, cov_width);

                // 判断填充在出发侧还是到达侧
                double fcx = (seg.getGeometry(0).getX() + seg.getGeometry(1).getX()) * 0.5;
                double fcy = (seg.getGeometry(0).getY() + seg.getGeometry(1).getY()) * 0.5;
                double fproj = fcx * (-s_dy) + fcy * s_dx;
                double dist_to_first = std::abs(fproj - proj_min);
                double dist_to_last  = std::abs(fproj - proj_max);
                if (cell_swaths.size() > 0 && dist_to_first < dist_to_last) {
                    start_fills.push_back(fill_sw);
                } else {
                    end_fills.push_back(fill_sw);
                }
            }
        }

        // 出发侧插入最前，到达侧追加末尾
        if (!start_fills.empty()) {
            f2c::types::Swaths reordered;
            for (auto& sf : start_fills) reordered.push_back(sf);
            for (size_t si = 0; si < cell_swaths.size(); ++si)
                reordered.push_back(cell_swaths.at(si));
            for (auto& ef : end_fills) reordered.push_back(ef);
            cell_swaths = reordered;
        }
    }

    // ═══════════════════════════════════════════════════
    // 孔洞边界补刀
    // ═══════════════════════════════════════════════════
    for (size_t hi = 0; hi + 1 < full_polygon.size(); ++hi) {
        const auto& hole_ring = full_polygon.getInteriorRing(hi);
        if (hole_ring.size() < 4) continue;

        // 判断 cell 是否贴近该孔洞
        bool touches_hole = false;
        {
            double min_d = std::numeric_limits<double>::max();
            for (size_t ci = 0; ci + 1 < cell_ring.size(); ++ci) {
                double cx = cell_ring.getGeometry(ci).getX();
                double cy = cell_ring.getGeometry(ci).getY();
                for (size_t pi = 0; pi + 1 < hole_ring.size(); ++pi) {
                    double d = std::hypot(cx - hole_ring.getGeometry(pi).getX(),
                                          cy - hole_ring.getGeometry(pi).getY());
                    if (d < min_d) min_d = d;
                }
            }
            touches_hole = (min_d < cov_width * 1.5);
        }

        if (!touches_hole) continue;

        // 孔洞质心
        double hole_cx = 0, hole_cy = 0;
        for (size_t pi = 0; pi + 1 < hole_ring.size(); ++pi) {
            hole_cx += hole_ring.getGeometry(pi).getX();
            hole_cy += hole_ring.getGeometry(pi).getY();
        }
        hole_cx /= (hole_ring.size() - 1);
        hole_cy /= (hole_ring.size() - 1);

        for (size_t pi = 0; pi + 1 < hole_ring.size(); ++pi) {
            double px1 = hole_ring.getGeometry(pi).getX();
            double py1 = hole_ring.getGeometry(pi).getY();
            double px2 = hole_ring.getGeometry(pi + 1).getX();
            double py2 = hole_ring.getGeometry(pi + 1).getY();

            double edx = px2 - px1, edy = py2 - py1;
            double elen = std::hypot(edx, edy);
            if (elen < 0.01) continue;

            double cos_angle = std::abs(edx * s_dx + edy * s_dy) / elen;
            if (cos_angle < angle_tol) continue;

            double e_min_x = std::min(px1, px2), e_max_x = std::max(px1, px2);
            double e_min_y = std::min(py1, py2), e_max_y = std::max(py1, py2);
            double bbox_dx = std::max(0.0, std::max(c_min_x - e_max_x, e_min_x - c_max_x));
            double bbox_dy = std::max(0.0, std::max(c_min_y - e_max_y, e_min_y - c_max_y));
            double bbox_dist = std::hypot(bbox_dx, bbox_dy);
            if (bbox_dist > cov_width * 1.5) continue;

            // 法向指向外侧（远离孔洞质心）
            double n_x = -edy / elen;
            double n_y =  edx / elen;
            double mx = (px1 + px2) * 0.5, my = (py1 + py2) * 0.5;
            if (n_x * (mx - hole_cx) + n_y * (my - hole_cy) < 0) {
                n_x = -n_x; n_y = -n_y;
            }

            // 孔洞边界 swath：向外偏移 half_w
            f2c::types::LineString bline;
            bline.addPoint(f2c::types::Point(px1 + boundary_offset * n_x, py1 + boundary_offset * n_y));
            bline.addPoint(f2c::types::Point(px2 + boundary_offset * n_x, py2 + boundary_offset * n_y));

            // 裁剪到多边形内
            f2c::types::Cells poly_tmp;
            poly_tmp.addGeometry(full_polygon);
            auto inside = poly_tmp.getLinesInside(bline);

            for (size_t li = 0; li < inside.size(); ++li) {
                auto seg = inside.getGeometry(li);
                double seg_len = std::hypot(
                    seg.getGeometry(1).getX() - seg.getGeometry(0).getX(),
                    seg.getGeometry(1).getY() - seg.getGeometry(0).getY());
                if (seg_len < cov_width * 0.5) continue;

                f2c::types::Swath fill_sw(seg, cov_width);

                double fcx = (seg.getGeometry(0).getX() + seg.getGeometry(1).getX()) * 0.5;
                double fcy = (seg.getGeometry(0).getY() + seg.getGeometry(1).getY()) * 0.5;
                double fproj = fcx * (-s_dy) + fcy * s_dx;
                double dist_to_first = std::abs(fproj - proj_min);
                double dist_to_last  = std::abs(fproj - proj_max);
                if (cell_swaths.size() > 0 && dist_to_first < dist_to_last) {
                    start_fills.push_back(fill_sw);
                } else {
                    end_fills.push_back(fill_sw);
                }
            }
        }
    }
}

}  // namespace yingshi
