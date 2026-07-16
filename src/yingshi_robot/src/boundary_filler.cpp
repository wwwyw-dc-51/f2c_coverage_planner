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

f2c::types::LinearRing makeClosedRing(
    const std::vector<f2c::types::Point>& points)
{
    f2c::types::LinearRing ring;
    for (const auto& point : points) {
        ring.addPoint(point);
    }
    if (ring.size() >= 3 &&
        ring.getGeometry(0).distance(
            ring.getGeometry(ring.size() - 1)) > 0.0) {
        const auto first = ring.getGeometry(0);
        ring.addPoint(first);
    }
    return ring;
}

// ========== 射线法：点是否在多边形内部 ==========
bool pointInPolygon(double px, double py, const f2c::types::LinearRing& ring)
{
    bool inside = false;
    const size_t n = ring.size();
    if (n < 3) return false;

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = ring.getGeometry(i).getX();
        const double yi = ring.getGeometry(i).getY();
        const double xj = ring.getGeometry(j).getX();
        const double yj = ring.getGeometry(j).getY();
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
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

    // 此偏移只保证覆盖工具贴边；机器人外形净空需要单独校验。
    double boundary_offset = cov_width * 0.5;
    (void)robot_half_width;  // 保留接口兼容性，当前不参与边界补线偏移。
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

            // 边界 swath：按覆盖宽度的一半向多边形内部偏移。
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
        if (!start_fills.empty() || !end_fills.empty()) {
            f2c::types::Swaths reordered;
            for (auto& sf : start_fills) reordered.push_back(sf);
            for (size_t si = 0; si < cell_swaths.size(); ++si)
                reordered.push_back(cell_swaths.at(si));
            for (auto& ef : end_fills) reordered.push_back(ef);
            cell_swaths = reordered;
        }
    }

    // ═══════════════════════════════════════════════════
    // Cell 内部接缝补刀
    // ═══════════════════════════════════════════════════
    // 分解后的相邻 cell 会独立生成 swath，法向相位可能不一致。
    // 这里只处理仍位于目标区域内部的严格平行接缝；物理外边界和孔洞
    // 边界继续由上面的专用逻辑负责。
    struct SeamInterval {
        double projection;
        double along_min;
        double along_max;
    };
    std::vector<SeamInterval> seam_intervals;

    auto pointInTargetArea = [&](double x, double y) {
        if (!pointInPolygon(x, y, poly_ring)) return false;
        for (size_t hi = 0; hi + 1 < full_polygon.size(); ++hi) {
            if (pointInPolygon(
                    x, y, full_polygon.getInteriorRing(hi))) {
                return false;
            }
        }
        return true;
    };

    double current_proj_min = std::numeric_limits<double>::max();
    double current_proj_max = -std::numeric_limits<double>::max();
    for (size_t si = 0; si < cell_swaths.size(); ++si) {
        const double cx = 0.5 * (
            cell_swaths.at(si).startPoint().getX() +
            cell_swaths.at(si).endPoint().getX());
        const double cy = 0.5 * (
            cell_swaths.at(si).startPoint().getY() +
            cell_swaths.at(si).endPoint().getY());
        const double projection = cx * (-s_dy) + cy * s_dx;
        current_proj_min = std::min(current_proj_min, projection);
        current_proj_max = std::max(current_proj_max, projection);
    }

    const double sample_step = std::min(0.05, cov_width * 0.1);
    for (size_t ci = 0; ci + 1 < cell_ring.size(); ++ci) {
        const double x1 = cell_ring.getGeometry(ci).getX();
        const double y1 = cell_ring.getGeometry(ci).getY();
        const double x2 = cell_ring.getGeometry(ci + 1).getX();
        const double y2 = cell_ring.getGeometry(ci + 1).getY();
        const double edge_dx = x2 - x1;
        const double edge_dy = y2 - y1;
        const double edge_len = std::hypot(edge_dx, edge_dy);
        if (edge_len < cov_width * 0.5) continue;

        // 接缝补线不能改变斜边几何，只接受近乎严格平行的分解边。
        const double parallel =
            std::abs(edge_dx * s_dx + edge_dy * s_dy) / edge_len;
        if (parallel < 0.999) continue;

        double inward_x = -edge_dy / edge_len;
        double inward_y = edge_dx / edge_len;
        const double mid_x = 0.5 * (x1 + x2);
        const double mid_y = 0.5 * (y1 + y2);
        if (inward_x * (cell_cx - mid_x) +
            inward_y * (cell_cy - mid_y) < 0.0) {
            inward_x = -inward_x;
            inward_y = -inward_y;
        }

        // 跨过该边仍在目标区域内，才是分解接缝。
        if (!pointInTargetArea(
                mid_x - inward_x * sample_step,
                mid_y - inward_y * sample_step)) {
            continue;
        }

        const double projection = mid_x * (-s_dy) + mid_y * s_dx;
        double nearest = std::numeric_limits<double>::max();
        for (size_t si = 0; si < cell_swaths.size(); ++si) {
            const double sx = 0.5 * (
                cell_swaths.at(si).startPoint().getX() +
                cell_swaths.at(si).endPoint().getX());
            const double sy = 0.5 * (
                cell_swaths.at(si).startPoint().getY() +
                cell_swaths.at(si).endPoint().getY());
            nearest = std::min(
                nearest,
                std::abs(projection - (sx * (-s_dy) + sy * s_dx)));
        }
        if (nearest <= boundary_offset + 1e-6) continue;

        const double along_1 = x1 * s_dx + y1 * s_dy;
        const double along_2 = x2 * s_dx + y2 * s_dy;
        const double along_min = std::min(along_1, along_2);
        const double along_max = std::max(along_1, along_2);

        bool merged = false;
        for (auto& interval : seam_intervals) {
            if (std::abs(interval.projection - projection) <= 1e-6 &&
                along_min <= interval.along_max + 1e-6 &&
                along_max >= interval.along_min - 1e-6) {
                interval.along_min = std::min(interval.along_min, along_min);
                interval.along_max = std::max(interval.along_max, along_max);
                merged = true;
                break;
            }
        }
        if (!merged) {
            seam_intervals.push_back({projection, along_min, along_max});
        }
    }

    if (!seam_intervals.empty()) {
        f2c::types::Swaths seam_start_fills;
        f2c::types::Swaths seam_end_fills;
        for (const auto& interval : seam_intervals) {
            f2c::types::LineString line;
            line.addPoint(f2c::types::Point(
                s_dx * interval.along_min - s_dy * interval.projection,
                s_dy * interval.along_min + s_dx * interval.projection));
            line.addPoint(f2c::types::Point(
                s_dx * interval.along_max - s_dy * interval.projection,
                s_dy * interval.along_max + s_dx * interval.projection));
            f2c::types::Swath fill(line, cov_width);

            const double distance_to_start =
                std::abs(interval.projection - current_proj_min);
            const double distance_to_end =
                std::abs(interval.projection - current_proj_max);
            if (distance_to_start < distance_to_end) {
                seam_start_fills.push_back(fill);
            } else {
                seam_end_fills.push_back(fill);
            }
        }

        f2c::types::Swaths reordered;
        for (size_t i = 0; i < seam_start_fills.size(); ++i) {
            reordered.push_back(seam_start_fills.at(i));
        }
        for (size_t i = 0; i < cell_swaths.size(); ++i) {
            reordered.push_back(cell_swaths.at(i));
        }
        for (size_t i = 0; i < seam_end_fills.size(); ++i) {
            reordered.push_back(seam_end_fills.at(i));
        }
        cell_swaths = reordered;
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
