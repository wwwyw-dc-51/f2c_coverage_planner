/**
 * @file swath_generator.cpp
 * @brief Swath 生成模块实现 — 角度优化、端点调整、斜边检测、几何变换
 *
 * 从 polygon_planner_node.cpp 提取，属于模块化重构 Step 3。
 */

#include "yingshi_robot/swath_generator.hpp"
#include "yingshi_robot/decomposer.hpp"
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

}  // namespace yingshi
