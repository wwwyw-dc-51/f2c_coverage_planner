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

                // 已有 swath 若能覆盖这段物理边界，就不再额外铺一条贴边线。
                // 判断针对真实边界而非候选补线中心，避免 S3 底边这类
                // 已覆盖区域出现局部加密轨迹。
                bool boundary_already_covered = false;
                const double edge_ux = edx / elen;
                const double edge_uy = edy / elen;
                const double seg_along_1 =
                    (seg.getGeometry(0).getX() - px1) * edge_ux +
                    (seg.getGeometry(0).getY() - py1) * edge_uy;
                const double seg_along_2 =
                    (seg.getGeometry(1).getX() - px1) * edge_ux +
                    (seg.getGeometry(1).getY() - py1) * edge_uy;
                const double seg_along_min = std::min(seg_along_1, seg_along_2);
                const double seg_along_max = std::max(seg_along_1, seg_along_2);
                for (size_t si = 0; si < cell_swaths.size(); ++si) {
                    const auto& swath = cell_swaths.at(si);
                    const double sx1 = swath.startPoint().getX();
                    const double sy1 = swath.startPoint().getY();
                    const double sx2 = swath.endPoint().getX();
                    const double sy2 = swath.endPoint().getY();
                    const double swath_len = std::hypot(sx2 - sx1, sy2 - sy1);
                    if (swath_len < 1e-9) continue;
                    const double parallel = std::abs(
                        (sx2 - sx1) * edge_ux +
                        (sy2 - sy1) * edge_uy) / swath_len;
                    if (parallel < angle_tol) continue;

                    const auto normalDistance = [&](double x, double y) {
                        return std::abs(
                            (x - px1) * (-edge_uy) +
                            (y - py1) * edge_ux);
                    };
                    const double max_normal_distance = std::max({
                        normalDistance(sx1, sy1),
                        normalDistance(sx2, sy2),
                        normalDistance(
                            0.5 * (sx1 + sx2),
                            0.5 * (sy1 + sy2))});
                    if (max_normal_distance > boundary_offset + 1e-6) continue;

                    const double swath_along_1 =
                        (sx1 - px1) * edge_ux + (sy1 - py1) * edge_uy;
                    const double swath_along_2 =
                        (sx2 - px1) * edge_ux + (sy2 - py1) * edge_uy;
                    const double overlap = std::min(
                        seg_along_max, std::max(swath_along_1, swath_along_2)) -
                        std::max(
                            seg_along_min,
                            std::min(swath_along_1, swath_along_2));
                    if (overlap >= seg_len - cov_width - 1e-6) {
                        boundary_already_covered = true;
                        break;
                    }
                }
                if (boundary_already_covered) continue;

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

size_t pruneRedundantCellSeamFills(
    f2c::types::SwathsByCells& swaths_by_cells,
    const f2c::types::Cells& cells,
    const f2c::types::Cell& full_polygon,
    double cov_width,
    double gap_tolerance_ratio)
{
    if (swaths_by_cells.sizeTotal() == 0 || cells.size() == 0 ||
        cov_width <= 0.0) {
        return 0;
    }

    struct SwathGeometry {
        size_t group_idx;
        size_t swath_idx;
        double x1;
        double y1;
        double x2;
        double y2;
        double length;
        bool seam_candidate {false};
    };

    std::vector<SwathGeometry> geometries;
    for (size_t group_idx = 0; group_idx < swaths_by_cells.size(); ++group_idx) {
        const auto& group = swaths_by_cells.at(group_idx);
        for (size_t swath_idx = 0; swath_idx < group.size(); ++swath_idx) {
            const auto& swath = group.at(swath_idx);
            const double x1 = swath.startPoint().getX();
            const double y1 = swath.startPoint().getY();
            const double x2 = swath.endPoint().getX();
            const double y2 = swath.endPoint().getY();
            geometries.push_back({
                group_idx, swath_idx, x1, y1, x2, y2,
                std::hypot(x2 - x1, y2 - y1), false});
        }
    }

    const auto& outer_ring = full_polygon.getExteriorRing();
    auto pointInTargetArea = [&](double x, double y) {
        if (!pointInPolygon(x, y, outer_ring)) return false;
        for (size_t hi = 0; hi + 1 < full_polygon.size(); ++hi) {
            if (pointInPolygon(x, y, full_polygon.getInteriorRing(hi))) {
                return false;
            }
        }
        return true;
    };

    constexpr double kGeometryTolerance = 1e-5;
    const double sample_step = std::min(0.05, cov_width * 0.1);
    for (auto& geometry : geometries) {
        if (geometry.length < cov_width * 0.5) continue;
        const double swath_dx = (geometry.x2 - geometry.x1) / geometry.length;
        const double swath_dy = (geometry.y2 - geometry.y1) / geometry.length;
        const double swath_mid_x = 0.5 * (geometry.x1 + geometry.x2);
        const double swath_mid_y = 0.5 * (geometry.y1 + geometry.y2);

        for (size_t cell_idx = 0;
             cell_idx < cells.size() && !geometry.seam_candidate; ++cell_idx) {
            const auto& ring = cells.getGeometry(cell_idx).getExteriorRing();
            for (size_t edge_idx = 0; edge_idx + 1 < ring.size(); ++edge_idx) {
                const double ex1 = ring.getGeometry(edge_idx).getX();
                const double ey1 = ring.getGeometry(edge_idx).getY();
                const double ex2 = ring.getGeometry(edge_idx + 1).getX();
                const double ey2 = ring.getGeometry(edge_idx + 1).getY();
                const double edge_len = std::hypot(ex2 - ex1, ey2 - ey1);
                if (edge_len < cov_width * 0.5) continue;
                const double edge_dx = (ex2 - ex1) / edge_len;
                const double edge_dy = (ey2 - ey1) / edge_len;
                if (std::abs(swath_dx * edge_dx + swath_dy * edge_dy) < 0.999) {
                    continue;
                }

                const double line_distance = std::abs(
                    (swath_mid_x - ex1) * (-edge_dy) +
                    (swath_mid_y - ey1) * edge_dx);
                if (line_distance > kGeometryTolerance) continue;

                const double swath_along_1 =
                    (geometry.x1 - ex1) * edge_dx +
                    (geometry.y1 - ey1) * edge_dy;
                const double swath_along_2 =
                    (geometry.x2 - ex1) * edge_dx +
                    (geometry.y2 - ey1) * edge_dy;
                const double overlap = std::min(
                    edge_len, std::max(swath_along_1, swath_along_2)) -
                    std::max(0.0, std::min(swath_along_1, swath_along_2));
                if (overlap < cov_width * 0.5) continue;

                const double edge_mid_x = 0.5 * (ex1 + ex2);
                const double edge_mid_y = 0.5 * (ey1 + ey2);
                const double normal_x = -edge_dy;
                const double normal_y = edge_dx;
                if (pointInTargetArea(
                        edge_mid_x + normal_x * sample_step,
                        edge_mid_y + normal_y * sample_step) &&
                    pointInTargetArea(
                        edge_mid_x - normal_x * sample_step,
                        edge_mid_y - normal_y * sample_step)) {
                    geometry.seam_candidate = true;
                    break;
                }
            }
        }
    }

    std::vector<std::vector<bool>> remove(swaths_by_cells.size());
    for (size_t group_idx = 0; group_idx < swaths_by_cells.size(); ++group_idx) {
        remove[group_idx].assign(swaths_by_cells.at(group_idx).size(), false);
    }

    const double allowed_gap =
        cov_width * (1.0 + std::max(0.0, gap_tolerance_ratio));
    for (size_t candidate_idx = 0;
         candidate_idx < geometries.size(); ++candidate_idx) {
        const auto& candidate = geometries[candidate_idx];
        if (!candidate.seam_candidate) continue;

        // 完全相同的共享接缝只保留第一次出现的一条。
        bool duplicate = false;
        for (size_t previous_idx = 0;
             previous_idx < candidate_idx; ++previous_idx) {
            const auto& previous = geometries[previous_idx];
            if (!previous.seam_candidate) continue;
            const double direct =
                std::hypot(candidate.x1 - previous.x1,
                           candidate.y1 - previous.y1) +
                std::hypot(candidate.x2 - previous.x2,
                           candidate.y2 - previous.y2);
            const double reversed =
                std::hypot(candidate.x1 - previous.x2,
                           candidate.y1 - previous.y2) +
                std::hypot(candidate.x2 - previous.x1,
                           candidate.y2 - previous.y1);
            if (std::min(direct, reversed) <= 2.0 * kGeometryTolerance) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            remove[candidate.group_idx][candidate.swath_idx] = true;
            continue;
        }

        const double dir_x = (candidate.x2 - candidate.x1) / candidate.length;
        const double dir_y = (candidate.y2 - candidate.y1) / candidate.length;
        const double normal_x = -dir_y;
        const double normal_y = dir_x;
        const double origin_x = 0.5 * (candidate.x1 + candidate.x2);
        const double origin_y = 0.5 * (candidate.y1 + candidate.y2);
        struct NeighborInterval {
            double along_min;
            double along_max;
            double distance_at_min;
            double distance_at_max;
        };
        std::vector<NeighborInterval> negative_neighbors;
        std::vector<NeighborInterval> positive_neighbors;

        for (const auto& other : geometries) {
            if (other.seam_candidate || other.length < 1e-9) continue;
            const double other_dir_x = (other.x2 - other.x1) / other.length;
            const double other_dir_y = (other.y2 - other.y1) / other.length;
            if (std::abs(dir_x * other_dir_x + dir_y * other_dir_y) <
                0.999999) {
                continue;
            }

            const double other_along_1 =
                (other.x1 - origin_x) * dir_x +
                (other.y1 - origin_y) * dir_y;
            const double other_along_2 =
                (other.x2 - origin_x) * dir_x +
                (other.y2 - origin_y) * dir_y;
            const double other_distance_1 =
                (other.x1 - origin_x) * normal_x +
                (other.y1 - origin_y) * normal_y;
            const double other_distance_2 =
                (other.x2 - origin_x) * normal_x +
                (other.y2 - origin_y) * normal_y;
            const bool forward_along = other_along_1 <= other_along_2;
            const double raw_along_min =
                std::min(other_along_1, other_along_2);
            const double raw_along_max =
                std::max(other_along_1, other_along_2);
            const double raw_distance_at_min =
                forward_along ? other_distance_1 : other_distance_2;
            const double raw_distance_at_max =
                forward_along ? other_distance_2 : other_distance_1;
            const double clipped_along_min = std::max(
                -candidate.length * 0.5,
                raw_along_min);
            const double clipped_along_max = std::min(
                candidate.length * 0.5,
                raw_along_max);
            const double overlap = clipped_along_max - clipped_along_min;
            if (overlap < std::min(cov_width * 0.5, candidate.length * 0.25)) {
                continue;
            }

            const auto distanceAt = [&](double along) {
                if (raw_along_max - raw_along_min < 1e-12) {
                    return 0.5 * (
                        raw_distance_at_min + raw_distance_at_max);
                }
                const double ratio =
                    (along - raw_along_min) /
                    (raw_along_max - raw_along_min);
                return raw_distance_at_min + ratio *
                    (raw_distance_at_max - raw_distance_at_min);
            };
            const double distance_at_min = distanceAt(clipped_along_min);
            const double distance_at_max = distanceAt(clipped_along_max);
            if (std::max(distance_at_min, distance_at_max) <
                -kGeometryTolerance) {
                negative_neighbors.push_back({
                    clipped_along_min, clipped_along_max,
                    distance_at_min, distance_at_max});
            } else if (std::min(distance_at_min, distance_at_max) >
                       kGeometryTolerance) {
                positive_neighbors.push_back({
                    clipped_along_min, clipped_along_max,
                    distance_at_min, distance_at_max});
            }
        }

        const auto interpolateDistance = [](
            const NeighborInterval& neighbor, double along) {
            if (neighbor.along_max - neighbor.along_min < 1e-12) {
                return 0.5 * (
                    neighbor.distance_at_min + neighbor.distance_at_max);
            }
            const double ratio =
                (along - neighbor.along_min) /
                (neighbor.along_max - neighbor.along_min);
            return neighbor.distance_at_min + ratio *
                (neighbor.distance_at_max - neighbor.distance_at_min);
        };
        const double endpoint_margin =
            std::min(cov_width * 0.5, candidate.length * 0.5);
        const double required_min =
            -candidate.length * 0.5 + endpoint_margin;
        const double required_max =
            candidate.length * 0.5 - endpoint_margin;
        for (const auto& negative : negative_neighbors) {
            for (const auto& positive : positive_neighbors) {
                const double common_min =
                    std::max(negative.along_min, positive.along_min);
                const double common_max =
                    std::min(negative.along_max, positive.along_max);
                if (common_min > required_min + 1e-6 ||
                    common_max < required_max - 1e-6) {
                    continue;
                }
                const double gap_at_min =
                    interpolateDistance(positive, required_min) -
                    interpolateDistance(negative, required_min);
                const double gap_at_max =
                    interpolateDistance(positive, required_max) -
                    interpolateDistance(negative, required_max);
                if (std::max(gap_at_min, gap_at_max) <=
                    allowed_gap + 1e-6) {
                    remove[candidate.group_idx][candidate.swath_idx] = true;
                    break;
                }
            }
            if (remove[candidate.group_idx][candidate.swath_idx]) break;
        }
    }

    size_t removed_count = 0;
    for (size_t group_idx = 0; group_idx < swaths_by_cells.size(); ++group_idx) {
        f2c::types::Swaths filtered;
        const auto& group = swaths_by_cells.at(group_idx);
        for (size_t swath_idx = 0; swath_idx < group.size(); ++swath_idx) {
            if (remove[group_idx][swath_idx]) {
                ++removed_count;
            } else {
                filtered.push_back(group.at(swath_idx));
            }
        }
        swaths_by_cells.at(group_idx) = filtered;
    }
    return removed_count;
}

}  // namespace yingshi
