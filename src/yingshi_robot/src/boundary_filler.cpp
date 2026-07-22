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

namespace {

bool pointStrictlyInsideRing(
    double px, double py, const f2c::types::LinearRing& ring);

}  // namespace

// ========== 点是否在任意孔洞内 ==========
bool pointInAnyHole(double px, double py,
                    const std::vector<f2c::types::LinearRing>& hole_rings)
{
    for (const auto& hr : hole_rings) {
        if (hr.size() < 3) continue;
        if (pointStrictlyInsideRing(px, py, hr)) {
            return true;
        }
    }
    return false;
}

namespace {

constexpr double kIntersectionTolerance = 1e-10;

size_t uniqueRingVertexCount(const f2c::types::LinearRing& ring)
{
    if (ring.size() < 2) return ring.size();
    return ring.getGeometry(0).distance(
        ring.getGeometry(ring.size() - 1)) <= kIntersectionTolerance
        ? ring.size() - 1 : ring.size();
}

bool pointOnRingBoundary(
    double px, double py, const f2c::types::LinearRing& ring)
{
    const size_t vertex_count = uniqueRingVertexCount(ring);
    for (size_t i = 0; i < vertex_count; ++i) {
        const size_t next = (i + 1) % vertex_count;
        const double ax = ring.getGeometry(i).getX();
        const double ay = ring.getGeometry(i).getY();
        const double bx = ring.getGeometry(next).getX();
        const double by = ring.getGeometry(next).getY();
        const double dx = bx - ax;
        const double dy = by - ay;
        const double length_squared = dx * dx + dy * dy;
        if (length_squared <=
            kIntersectionTolerance * kIntersectionTolerance) {
            continue;
        }
        const double projection =
            ((px - ax) * dx + (py - ay) * dy) / length_squared;
        if (projection < -kIntersectionTolerance ||
            projection > 1.0 + kIntersectionTolerance) {
            continue;
        }
        const double cross = (px - ax) * dy - (py - ay) * dx;
        if (std::abs(cross) <=
            kIntersectionTolerance * std::sqrt(length_squared)) {
            return true;
        }
    }
    return false;
}

bool pointStrictlyInsideRing(
    double px, double py, const f2c::types::LinearRing& ring)
{
    return !pointOnRingBoundary(px, py, ring) &&
           pointInPolygon(px, py, ring);
}

void appendIntersectionParameters(
    double x0, double y0, double x1, double y1,
    const f2c::types::LinearRing& ring,
    std::vector<double>& parameters)
{
    const double segment_dx = x1 - x0;
    const double segment_dy = y1 - y0;
    const double segment_length_squared =
        segment_dx * segment_dx + segment_dy * segment_dy;
    if (segment_length_squared <=
        kIntersectionTolerance * kIntersectionTolerance) {
        return;
    }

    const size_t vertex_count = uniqueRingVertexCount(ring);
    for (size_t i = 0; i < vertex_count; ++i) {
        const size_t next = (i + 1) % vertex_count;
        const double ax = ring.getGeometry(i).getX();
        const double ay = ring.getGeometry(i).getY();
        const double edge_dx = ring.getGeometry(next).getX() - ax;
        const double edge_dy = ring.getGeometry(next).getY() - ay;
        const double offset_x = ax - x0;
        const double offset_y = ay - y0;
        const double denominator =
            segment_dx * edge_dy - segment_dy * edge_dx;

        if (std::abs(denominator) > kIntersectionTolerance) {
            const double t =
                (offset_x * edge_dy - offset_y * edge_dx) / denominator;
            const double u =
                (offset_x * segment_dy - offset_y * segment_dx) /
                denominator;
            if (t >= -kIntersectionTolerance &&
                t <= 1.0 + kIntersectionTolerance &&
                u >= -kIntersectionTolerance &&
                u <= 1.0 + kIntersectionTolerance) {
                parameters.push_back(std::clamp(t, 0.0, 1.0));
            }
            continue;
        }

        const double collinear_cross =
            offset_x * segment_dy - offset_y * segment_dx;
        if (std::abs(collinear_cross) >
            kIntersectionTolerance * std::sqrt(segment_length_squared)) {
            continue;
        }
        const double edge_end_offset_x =
            ring.getGeometry(next).getX() - x0;
        const double edge_end_offset_y =
            ring.getGeometry(next).getY() - y0;
        const double t0 =
            (offset_x * segment_dx + offset_y * segment_dy) /
            segment_length_squared;
        const double t1 =
            (edge_end_offset_x * segment_dx +
             edge_end_offset_y * segment_dy) /
            segment_length_squared;
        const double overlap_start =
            std::max(0.0, std::min(t0, t1));
        const double overlap_end =
            std::min(1.0, std::max(t0, t1));
        if (overlap_start <= overlap_end + kIntersectionTolerance) {
            parameters.push_back(overlap_start);
            parameters.push_back(overlap_end);
        }
    }
}

}  // namespace

// ========== 线段是否穿过孔洞内部（精确边界求交）==========
bool segmentCrossesHole(double x0, double y0, double x1, double y1,
                        const std::vector<f2c::types::LinearRing>& hole_rings,
                        int num_samples)
{
    static_cast<void>(num_samples);  // 兼容旧调用；判定不再依赖采样密度。
    for (const auto& hole : hole_rings) {
        if (uniqueRingVertexCount(hole) < 3) continue;
        std::vector<double> parameters {0.0, 1.0};
        appendIntersectionParameters(
            x0, y0, x1, y1, hole, parameters);
        std::sort(parameters.begin(), parameters.end());
        parameters.erase(std::unique(
            parameters.begin(), parameters.end(),
            [](double lhs, double rhs) {
                return std::abs(lhs - rhs) <= kIntersectionTolerance;
            }), parameters.end());

        for (size_t i = 0; i + 1 < parameters.size(); ++i) {
            if (parameters[i + 1] - parameters[i] <=
                kIntersectionTolerance) {
                continue;
            }
            const double midpoint =
                0.5 * (parameters[i] + parameters[i + 1]);
            const double x = x0 + midpoint * (x1 - x0);
            const double y = y0 + midpoint * (y1 - y0);
            if (pointStrictlyInsideRing(x, y, hole)) return true;
        }
    }
    return false;
}

namespace {

double pointToRingDistance(
    double px, double py, const f2c::types::LinearRing& ring)
{
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < ring.size(); ++i) {
        const double ax = ring.getGeometry(i).getX();
        const double ay = ring.getGeometry(i).getY();
        const double bx = ring.getGeometry(i + 1).getX();
        const double by = ring.getGeometry(i + 1).getY();
        const double dx = bx - ax;
        const double dy = by - ay;
        const double length_squared = dx * dx + dy * dy;
        if (length_squared < 1e-18) {
            min_distance = std::min(
                min_distance, std::hypot(px - ax, py - ay));
            continue;
        }
        const double projection = std::clamp(
            ((px - ax) * dx + (py - ay) * dy) / length_squared,
            0.0, 1.0);
        const double closest_x = ax + projection * dx;
        const double closest_y = ay + projection * dy;
        min_distance = std::min(
            min_distance, std::hypot(px - closest_x, py - closest_y));
    }
    return min_distance;
}

}  // namespace

f2c::types::Swath adjustSwathEndpointsForBoundaryClearance(
    const f2c::types::Swath& swath,
    const f2c::types::LinearRing& outer_ring,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double coverage_width,
    double default_margin)
{
    if (default_margin == 0.0 || coverage_width <= 0.0) return swath;

    const auto start = swath.startPoint();
    const auto end = swath.endPoint();
    const double dx = end.getX() - start.getX();
    const double dy = end.getY() - start.getY();
    const double length = std::hypot(dx, dy);
    if (length < 1e-9) return swath;

    const double proximity_threshold = 2.0 * coverage_width;
    const auto endpointMargin = [&](const f2c::types::Point& endpoint) {
        double hole_distance = std::numeric_limits<double>::max();
        for (const auto& hole : hole_rings) {
            hole_distance = std::min(
                hole_distance,
                pointToRingDistance(
                    endpoint.getX(), endpoint.getY(), hole));
        }
        if (hole_distance < proximity_threshold) {
            // 开放边界也不能把端点向孔洞延伸，孔洞侧始终使用正净空。
            return 2.0 * std::abs(default_margin);
        }

        const double outer_distance = pointToRingDistance(
            endpoint.getX(), endpoint.getY(), outer_ring);
        if (outer_distance < proximity_threshold) return 0.0;
        return default_margin;
    };

    const double start_margin = endpointMargin(start);
    const double end_margin = endpointMargin(end);
    if (length - start_margin - end_margin <= 1e-9) return swath;

    const double unit_x = dx / length;
    const double unit_y = dy / length;
    f2c::types::LineString adjusted_path;
    adjusted_path.addPoint(f2c::types::Point(
        start.getX() + start_margin * unit_x,
        start.getY() + start_margin * unit_y));
    adjusted_path.addPoint(f2c::types::Point(
        end.getX() - end_margin * unit_x,
        end.getY() - end_margin * unit_y));
    f2c::types::Swath adjusted(
        adjusted_path, swath.getWidth(), swath.getId(), swath.getType());
    adjusted.setCreationDir(swath.getCreationDir());
    return adjusted;
}

// ========== 边界间隙补填（完整版，v8/v9 成熟实现）==========
void fillBoundaryGaps(
    f2c::types::Swaths& cell_swaths,
    const f2c::types::Cell& cell,
    const f2c::types::Cell& full_polygon,
    double swath_angle,
    double cov_width,
    double shrink_dist,
    double robot_half_width,
    double boundary_offset_override)
{
    if (cell_swaths.size() == 0) return;
    const auto& cell_ring = cell.getExteriorRing();
    if (cell_ring.size() < 3) return;
    const auto& poly_ring = full_polygon.getExteriorRing();

    const double boundary_offset = boundary_offset_override >= 0.0
        ? boundary_offset_override
        : std::max(cov_width * 0.5, robot_half_width);
    /*

    // 原始几何下，边界补线至少要为实体外形保留横向净空。
    const double boundary_offset = boundary_offset_override >= 0.0
        ? boundary_offset_override
        : std::max(cov_width * 0.5, robot_half_width);
    (void)shrink_dist;  // 端点缩进由 adjustSwathEndpoints 单独处理

    // ── cell bbox ──
    */
    (void)shrink_dist;
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

    double cell_along_min = std::numeric_limits<double>::max();
    double cell_along_max = -std::numeric_limits<double>::max();
    for (size_t ci = 0; ci + 1 < cell_ring.size(); ++ci) {
        const double along =
            cell_ring.getGeometry(ci).getX() * s_dx +
            cell_ring.getGeometry(ci).getY() * s_dy;
        cell_along_min = std::min(cell_along_min, along);
        cell_along_max = std::max(cell_along_max, along);
    }

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

            // 先按完整作业区裁剪，保留位于 no_hl Cell 法向范围外、但仍负责
            // 真实边界覆盖的补线（S6 门洞/柜边）。随后只沿 Swath 行进方向
            // 限制到当前 Cell 的可达投影范围，避免端点多伸出形成毛刺。
            f2c::types::Cells poly_tmp;
            poly_tmp.addGeometry(full_polygon);
            auto inside = poly_tmp.getLinesInside(bline);

            for (size_t li = 0; li < inside.size(); ++li) {
                auto seg = inside.getGeometry(li);
                const double x0 = seg.getGeometry(0).getX();
                const double y0 = seg.getGeometry(0).getY();
                const double x1 = seg.getGeometry(1).getX();
                const double y1 = seg.getGeometry(1).getY();
                const double along0 = x0 * s_dx + y0 * s_dy;
                const double along1 = x1 * s_dx + y1 * s_dy;
                const double along_delta = along1 - along0;
                if (std::abs(along_delta) < 1e-9) continue;

                const double range_t0 =
                    (cell_along_min - along0) / along_delta;
                const double range_t1 =
                    (cell_along_max - along0) / along_delta;
                const double clipped_t0 = std::max(
                    0.0, std::min(range_t0, range_t1));
                const double clipped_t1 = std::min(
                    1.0, std::max(range_t0, range_t1));
                if (clipped_t1 - clipped_t0 < 1e-9) continue;

                f2c::types::LineString capped_seg;
                capped_seg.addPoint(f2c::types::Point(
                    x0 + clipped_t0 * (x1 - x0),
                    y0 + clipped_t0 * (y1 - y0)));
                capped_seg.addPoint(f2c::types::Point(
                    x0 + clipped_t1 * (x1 - x0),
                    y0 + clipped_t1 * (y1 - y0)));
                seg = capped_seg;
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
            start_fills.clear();
            end_fills.clear();
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

    // 孔洞边界补线合并回 cell_swaths（与外环补线对称）
    if (!start_fills.empty() || !end_fills.empty()) {
        f2c::types::Swaths reordered;
        for (size_t i = 0; i < start_fills.size(); ++i)
            reordered.push_back(start_fills.at(i));
        for (size_t i = 0; i < cell_swaths.size(); ++i)
            reordered.push_back(cell_swaths.at(i));
        for (size_t i = 0; i < end_fills.size(); ++i)
            reordered.push_back(end_fills.at(i));
        cell_swaths = reordered;
    }
}

size_t rebalanceNarrowCellSwaths(
    f2c::types::Swaths& swaths,
    const f2c::types::Cell& cell,
    double coverage_width)
{
    if (swaths.size() < 4 || coverage_width <= 0.0 || cell.size() != 1) {
        return 0;
    }

    const auto& ring = cell.getExteriorRing();
    if (ring.size() != 5) return 0;

    const auto& first = swaths.at(0);
    const double first_dx = first.endPoint().getX() - first.startPoint().getX();
    const double first_dy = first.endPoint().getY() - first.startPoint().getY();
    const double first_length = std::hypot(first_dx, first_dy);
    if (first_length < coverage_width) return 0;

    const double dir_x = first_dx / first_length;
    const double dir_y = first_dy / first_length;
    const double normal_x = -dir_y;
    const double normal_y = dir_x;

    size_t parallel_edges = 0;
    size_t normal_edges = 0;
    for (size_t edge_index = 0; edge_index + 1 < ring.size(); ++edge_index) {
        const double edge_dx =
            ring.getGeometry(edge_index + 1).getX() -
            ring.getGeometry(edge_index).getX();
        const double edge_dy =
            ring.getGeometry(edge_index + 1).getY() -
            ring.getGeometry(edge_index).getY();
        const double edge_length = std::hypot(edge_dx, edge_dy);
        if (edge_length < 1e-9) return 0;
        const double alignment = std::abs(
            (edge_dx * dir_x + edge_dy * dir_y) / edge_length);
        if (alignment >= 0.999) {
            ++parallel_edges;
        } else if (alignment <= 0.045) {
            ++normal_edges;
        } else {
            return 0;
        }
    }
    if (parallel_edges != 2 || normal_edges != 2) return 0;

    std::vector<double> projections;
    projections.reserve(swaths.size());
    double min_length = std::numeric_limits<double>::max();
    double max_length = 0.0;
    for (size_t swath_index = 0; swath_index < swaths.size(); ++swath_index) {
        const auto& swath = swaths.at(swath_index);
        const double dx = swath.endPoint().getX() - swath.startPoint().getX();
        const double dy = swath.endPoint().getY() - swath.startPoint().getY();
        const double length = std::hypot(dx, dy);
        if (length < coverage_width) return 0;
        const double alignment = std::abs(
            (dx * dir_x + dy * dir_y) / length);
        if (alignment < 0.999) return 0;

        min_length = std::min(min_length, length);
        max_length = std::max(max_length, length);
        const double mid_x = 0.5 * (
            swath.startPoint().getX() + swath.endPoint().getX());
        const double mid_y = 0.5 * (
            swath.startPoint().getY() + swath.endPoint().getY());
        projections.push_back(mid_x * normal_x + mid_y * normal_y);
    }
    if (max_length - min_length > coverage_width * 0.25) return 0;

    const auto [min_projection_it, max_projection_it] =
        std::minmax_element(projections.begin(), projections.end());
    const double min_projection = *min_projection_it;
    const double max_projection = *max_projection_it;
    const double projection_span = max_projection - min_projection;
    if (projection_span <= coverage_width ||
        projection_span > 3.0 * coverage_width) {
        return 0;
    }

    double cell_projection_min = std::numeric_limits<double>::max();
    double cell_projection_max = -std::numeric_limits<double>::max();
    for (size_t point_index = 0; point_index + 1 < ring.size(); ++point_index) {
        const auto point = ring.getGeometry(point_index);
        const double projection =
            point.getX() * normal_x + point.getY() * normal_y;
        cell_projection_min = std::min(cell_projection_min, projection);
        cell_projection_max = std::max(cell_projection_max, projection);
    }
    const double half_coverage = 0.5 * coverage_width;
    if (min_projection - cell_projection_min > half_coverage + 1e-6 ||
        cell_projection_max - max_projection > half_coverage + 1e-6) {
        return 0;
    }

    std::vector<double> sorted_projections = projections;
    std::sort(sorted_projections.begin(), sorted_projections.end());
    double minimum_gap = std::numeric_limits<double>::max();
    for (size_t index = 1; index < sorted_projections.size(); ++index) {
        minimum_gap = std::min(
            minimum_gap,
            sorted_projections[index] - sorted_projections[index - 1]);
    }
    if (minimum_gap >= coverage_width * 0.25) return 0;

    const size_t target_count = static_cast<size_t>(
        std::ceil(projection_span / coverage_width)) + 1;
    if (target_count >= swaths.size() || target_count < 2) return 0;

    double along_min = std::numeric_limits<double>::max();
    double along_max = -std::numeric_limits<double>::max();
    for (size_t point_index = 0; point_index + 1 < ring.size(); ++point_index) {
        const auto& point = ring.getGeometry(point_index);
        const double along = point.getX() * dir_x + point.getY() * dir_y;
        along_min = std::min(along_min, along);
        along_max = std::max(along_max, along);
    }

    const bool starts_at_max =
        std::abs(projections.front() - max_projection) <
        std::abs(projections.front() - min_projection);
    const double projection_step =
        projection_span / static_cast<double>(target_count - 1);
    f2c::types::Swaths rebalanced;
    for (size_t index = 0; index < target_count; ++index) {
        const double projection = starts_at_max
            ? max_projection - projection_step * index
            : min_projection + projection_step * index;
        const f2c::types::Point low(
            dir_x * along_min + normal_x * projection,
            dir_y * along_min + normal_y * projection);
        const f2c::types::Point high(
            dir_x * along_max + normal_x * projection,
            dir_y * along_max + normal_y * projection);

        f2c::types::LineString line;
        if (index % 2 == 0) {
            line.addPoint(low);
            line.addPoint(high);
        } else {
            line.addPoint(high);
            line.addPoint(low);
        }
        const auto& source = swaths.at(std::min(index, swaths.size() - 1));
        f2c::types::Swath rebalanced_swath(
            line, coverage_width, source.getId(), source.getType());
        rebalanced_swath.setCreationDir(source.getCreationDir());
        rebalanced.push_back(rebalanced_swath);
    }

    const size_t removed_count = swaths.size() - rebalanced.size();
    swaths = rebalanced;
    return removed_count;
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

// ========== 边界策略：对 Route 中全部 swath 应用边界缩进/延伸 ==========
// 从 ROS 节点 planCoveragePath 提取。对每个 swath 端点独立判断
// 外环/孔洞净空后调整，确保边界策略一致。
double resolveBoundaryMargin(
    const std::string& boundary_type,
    double endpoint_shrink,
    double boundary_coverage_margin,
    double open_default_margin)
{
    if (boundary_type == "closed") {
        return endpoint_shrink > 0.0 ? endpoint_shrink : 0.3;
    }
    if (boundary_type == "open") {
        return boundary_coverage_margin < 0.0
            ? boundary_coverage_margin
            : open_default_margin;
    }
    return boundary_coverage_margin;
}

size_t applyBoundaryMarginToRoute(
    f2c::types::Route& route,
    const f2c::types::LinearRing& outer_ring,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double coverage_width,
    double margin)
{
    if (std::abs(margin) < 1e-9) return 0;

    size_t adjusted = 0;
    for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
        f2c::types::Swaths& route_swaths = route.getSwaths(i);
        f2c::types::Swaths adjusted_swaths;
        for (size_t j = 0; j < route_swaths.size(); ++j) {
            adjusted_swaths.push_back(
                adjustSwathEndpointsForBoundaryClearance(
                    route_swaths.at(j), outer_ring, hole_rings,
                    coverage_width, margin));
        }
        route.setSwaths(i, adjusted_swaths);
        ++adjusted;
    }
    return adjusted;
}

}  // namespace yingshi
