/**
 * @file coverage_evaluator.hpp
 * @brief F2C 全覆盖路径规划 — 评估模块（纯头文件库）
 *
 * 提供覆盖率、路径效率、转弯、曲率等指标的量化计算和格式化报告输出。
 * 无 ROS 依赖，仅依赖 Fields2Cover 和 C++ 标准库。
 *
 * 使用方式：
 *   #include "coverage_evaluator.hpp"
 *   EvalResult r = evaluatePlan(path, swaths, target_cells, planning_time_ms, params);
 *   std::string report = formatEvalReport(r);
 *   RCLCPP_INFO(logger, "\n%s", report.c_str());
 */

#pragma once

#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <utility>
#include <fstream>

#include "fields2cover.h"
#include "yingshi_robot/path_geometry.hpp"

// ============================================================================
// 1. 数据结构
// ============================================================================

/// 评估参数配置
struct EvalParams {
    double max_diff_curv = 0.3;       ///< 机器人最大允许曲率
    double coverage_width = 0.45;     ///< 覆盖宽度 (m)
    double swath_overlap_ratio = 0.0; ///< 预期重叠率 (0~1)，故意设的重叠不应扣分
    double grid_resolution = 0.1;     ///< 网格法分辨率 (m)
    double coverage_threshold = 0.99; ///< 覆盖率达标阈值
    double turn_angle_threshold = 30.0; ///< 转弯判定角度阈值（度）
    double min_turn_merge_distance = 0.75; ///< 相邻方向变化归并距离下限（米），实际值随有效行距增大
    bool use_grid_method = false;     ///< true=网格法 false=几何法
    const char* turn_planner_type = "direct"; ///< 掉头模式，决定评分方案
    double full_net_area_override = -1.0; ///< 覆盖目标面积覆盖（>0 时覆盖 cell 面积计算），用于纠正headland侵蚀偏差
};

/// 评估结果
struct EvalResult {
    // ── 元信息 ──
    int swath_count = 0;
    int cell_count = 0;
    int path_point_count = 0;
    const char* coverage_method = "geometric";

    // ── 第一层：覆盖完整性 ──
    double net_area = 0.0;           ///< 目标区域净面积 (m²)
    double covered_area = 0.0;       ///< 已覆盖面积 (m²)
    double coverage_rate = 0.0;      ///< 覆盖率 (0~1)
    bool coverage_pass = false;      ///< 覆盖率 ≥ 阈值？

    // ── 网格法渲染数据（避免 Python 重算） ──
    double grid_resolution = 0.0;    ///< 网格分辨率
    std::vector<std::pair<double,double>> covered_grid;   ///< 覆盖网格点
    std::vector<std::pair<double,double>> uncovered_grid; ///< 未覆盖网格点

    // ── 第二层：效率指标 ──
    double total_distance = 0.0;     ///< 路径总长 (m)
    double swath_total_dist = 0.0;   ///< Swath 线段总长 (m)
    double work_ratio = 0.0;         ///< 有效工作比，>1.0 表示重叠过多
    int turn_count = 0;              ///< 转弯次数
    double overlap_rate = 0.0;       ///< 重叠率 (0~1)
    double planning_time_ms = 0.0;   ///< 规划耗时 (ms)
    double avg_curvature = 0.0;      ///< 平均曲率
    double max_curvature = 0.0;      ///< 最大曲率

    // ── 掉头可行性 ──
    int turn_total_count = 0;         ///< 掉头段总数
    int turn_infeasible_count = 0;    ///< 不可执行的掉头数
    bool turn_feasibility_pass = true; ///< 是否所有掉头都可执行
    double turn_worst_curvature = 0.0; ///< 最差掉头曲率
    const char* turn_planner_used = ""; ///< 实际使用的掉头规划器

    // ── 综合评分 ──
    double single_score = 0.0;       ///< 单场景综合得分 (0~100)
    double coverage_gate = 0.0;      ///< 90% 到达标阈值之间归一化后的三次门控
    double efficiency_score = 0.0;   ///< 6 项加权效率分

    // ── 评分分解 ──
    double score_work_ratio = 0.0;
    double score_distance = 0.0;
    double score_turns = 0.0;
    double score_overlap = 0.0;
    double score_time = 0.0;
    double score_smoothness = 0.0;
};

namespace coverage_scoring {

constexpr double kCoverageGateLower = 0.90;
constexpr double kDistanceFreeStretch = 1.15;
constexpr double kExtraTurnPenalty = 3.0;
constexpr double kOverlapPenalty = 2.0;
constexpr double kTurnMergeSpacingFactor = 1.05;

inline double coverageGate(
    double coverage_rate,
    double coverage_threshold)
{
    if (coverage_rate <= kCoverageGateLower) return 0.0;
    if (coverage_threshold <= kCoverageGateLower ||
        coverage_rate >= coverage_threshold) {
        return 1.0;
    }

    const double normalized =
        (coverage_rate - kCoverageGateLower) /
        (coverage_threshold - kCoverageGateLower);
    return normalized * normalized * normalized;
}

inline double distanceScore(double total_distance, double ideal_distance)
{
    if (ideal_distance <= 1e-9) {
        return total_distance <= 1e-9 ? 1.0 : 0.0;
    }
    const double stretch = std::max(0.0, total_distance) / ideal_distance;
    return std::exp(-std::max(0.0, stretch - kDistanceFreeStretch));
}

inline double workRatioScore(double work_ratio)
{
    if (work_ratio <= 0.0) return 0.0;
    return work_ratio <= 1.0 ? work_ratio : 1.0 / work_ratio;
}

inline double turnScore(int turn_count, int swath_count)
{
    const int baseline_turns = std::max(0, swath_count - 1);
    const int extra_turns = std::max(0, turn_count - baseline_turns);
    const double baseline = std::max(1, baseline_turns);
    return std::exp(-kExtraTurnPenalty * extra_turns / baseline);
}

inline double overlapScore(
    double measured_overlap,
    double planned_overlap)
{
    const double bounded_planned = std::clamp(planned_overlap, 0.0, 0.999999);
    const double planned_grid_overlap =
        bounded_planned / (1.0 - bounded_planned);
    const double avoidable_overlap =
        std::max(0.0, measured_overlap - planned_grid_overlap);
    return std::exp(-kOverlapPenalty * avoidable_overlap);
}

inline double turnMergeDistance(
    double effective_width,
    double configured_minimum)
{
    return std::max(
        std::max(0.0, configured_minimum),
        kTurnMergeSpacingFactor * std::max(0.0, effective_width));
}

}  // namespace coverage_scoring

// ============================================================================
// 2. 几何工具函数
// ============================================================================

/// 两点间欧氏距离
inline double pointDistance(const f2c::types::Point& a, const f2c::types::Point& b) {
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    return std::sqrt(dx * dx + dy * dy);
}

/// 点到线段的最短距离（含投影判断）
inline double pointToSegmentDist(
    const f2c::types::Point& p,
    const f2c::types::Point& seg_a,
    const f2c::types::Point& seg_b)
{
    double dx = seg_b.getX() - seg_a.getX();
    double dy = seg_b.getY() - seg_a.getY();
    double len_sq = dx * dx + dy * dy;

    if (len_sq < 1e-12) {
        // 线段退化为点
        return pointDistance(p, seg_a);
    }

    // 投影参数 t，clamp 到 [0, 1]
    double t = ((p.getX() - seg_a.getX()) * dx + (p.getY() - seg_a.getY()) * dy) / len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double proj_x = seg_a.getX() + t * dx;
    double proj_y = seg_a.getY() + t * dy;
    double pdx = p.getX() - proj_x;
    double pdy = p.getY() - proj_y;
    return std::sqrt(pdx * pdx + pdy * pdy);
}

/// 射线法：点是否在多边形内部
inline bool pointInPolygon(double px, double py,
                           const f2c::types::LinearRing& ring)
{
    bool inside = false;
    size_t n = ring.size();
    if (n < 3) return false;

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double xi = ring.getGeometry(i).getX();
        double yi = ring.getGeometry(i).getY();
        double xj = ring.getGeometry(j).getX();
        double yj = ring.getGeometry(j).getY();

        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

/// 计算线性环的面积（鞋带公式）
inline double ringArea(const f2c::types::LinearRing& ring) {
    double area = 0.0;
    size_t n = ring.size();
    if (n < 3) return 0.0;
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += ring.getGeometry(i).getX() * ring.getGeometry(j).getY();
        area -= ring.getGeometry(j).getX() * ring.getGeometry(i).getY();
    }
    return std::abs(area) / 2.0;
}

// ============================================================================
// 3. 指标计算函数
// ============================================================================

/// 计算路径总长（所有相邻点欧氏距离累加）
inline double computeTotalDistance(const f2c::types::Path& path) {
    return yingshi::polylineLength(yingshi::materializePath(path));
}

/// 计算 Swath 线段总长
inline double computeSwathTotalDist(const f2c::types::Swaths& swaths) {
    double total = 0.0;
    for (size_t i = 0; i < swaths.size(); ++i) {
        const auto& s = swaths.at(i);
        total += pointDistance(s.startPoint(), s.endPoint());
    }
    return total;
}

/// 计算从 SwathsByCells 中提取所有 swaths 后的总长
inline double computeSwathTotalDist(const f2c::types::SwathsByCells& swaths_by_cells) {
    double total = 0.0;
    for (size_t ci = 0; ci < swaths_by_cells.size(); ++ci) {
        const auto& cell_swaths = swaths_by_cells.at(ci);
        for (size_t si = 0; si < cell_swaths.size(); ++si) {
            const auto& s = cell_swaths.at(si);
            total += pointDistance(s.startPoint(), s.endPoint());
        }
    }
    return total;
}

/// 统计最终发布折线中的转弯动作；一个掉头内的连续方向变化只计一次。
inline int countTurns(
    const f2c::types::Path& path,
    double angle_threshold_deg,
    double merge_distance = 0.75)
{
    const auto points = yingshi::materializePath(path);
    int turns = 0;
    if (points.size() < 3) return turns;

    const double threshold_rad = angle_threshold_deg * M_PI / 180.0;
    double arc_distance = 0.0;
    double last_turn_arc = -std::numeric_limits<double>::infinity();

    for (size_t i = 1; i + 1 < points.size(); ++i) {
        arc_distance += points[i - 1].distance(points[i]);
        const double dx_prev = points[i].getX() - points[i - 1].getX();
        const double dy_prev = points[i].getY() - points[i - 1].getY();
        const double angle_prev = std::atan2(dy_prev, dx_prev);

        const double dx_next = points[i + 1].getX() - points[i].getX();
        const double dy_next = points[i + 1].getY() - points[i].getY();
        const double angle_next = std::atan2(dy_next, dx_next);

        double delta = std::abs(angle_next - angle_prev);
        delta = std::min(delta, 2.0 * M_PI - delta);

        if (delta > threshold_rad) {
            if (arc_distance - last_turn_arc > std::max(0.0, merge_distance)) {
                ++turns;
            }
            // 用最近一次方向变化更新锚点，避免一个平滑掉头被拆成多个动作。
            last_turn_arc = arc_distance;
        }
    }
    return turns;
}

/// 计算路径曲率（平均和最大）
inline std::pair<double, double> computeCurvature(const f2c::types::Path& path) {
    double sum_curv = 0.0;
    double max_curv = 0.0;
    int count = 0;

    for (size_t i = 1; i < path.size() - 1; ++i) {
        // 跳过 TURN 段（direct 模式原地转、dubins 曲线均由掉头可行性检查单独处理）
        if (path[i].type == f2c::types::PathSectionType::TURN) continue;

        double dx = path[i + 1].point.getX() - path[i].point.getX();
        double dy = path[i + 1].point.getY() - path[i].point.getY();
        double ds = std::sqrt(dx * dx + dy * dy);

        if (ds < 1e-9) continue;

        double d_angle = std::abs(path[i + 1].angle - path[i].angle);
        d_angle = std::min(d_angle, 2.0 * M_PI - d_angle);

        double curv = d_angle / ds;
        sum_curv += curv;
        if (curv > max_curv) max_curv = curv;
        ++count;
    }

    double avg = (count > 0) ? (sum_curv / count) : 0.0;
    return {avg, max_curv};
}

// ============================================================================
// 4. 覆盖率计算
// ============================================================================

/// 网格法：精确计算覆盖率和重叠率
/// @param path 规划路径
/// @param target_cells 目标区域（含孔洞的外环信息）
/// @param hole_rings 孔洞环列表（用于点包含测试）
/// @param params 评估参数
/// @param out_overlap_rate [out] 基于路径扫掠的真实重叠率（可选）
/// @return {covered_area, coverage_rate}
/// @note 包含 Y 轴预过滤优化：每行网格点只检查 y-range 覆盖该行的路径段，
///       将 O(G×P) 降为 O(G×P_filtered)，对大面积+长路径场景加速 20-50x
///       逐格元访问计数用于计算路径级重叠率（非 swath 几何估算）
inline std::pair<double, double> computeCoverageGrid(
    const f2c::types::Path& path,
    const f2c::types::Cells& target_cells,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    const EvalParams& params,
    std::vector<std::pair<double,double>>* out_covered = nullptr,
    std::vector<std::pair<double,double>>* out_uncovered = nullptr,
    double* out_overlap_rate = nullptr)
{
    double res = params.grid_resolution;
    double half_cov = params.coverage_width / 2.0;

    // 1. 计算所有 cells 的包围盒
    if (target_cells.size() == 0) return {0.0, 0.0};

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x;

    for (size_t ci = 0; ci < target_cells.size(); ++ci) {
        const auto& ring = target_cells.getGeometry(ci).getExteriorRing();
        for (size_t pi = 0; pi < ring.size(); ++pi) {
            double x = ring.getGeometry(pi).getX();
            double y = ring.getGeometry(pi).getY();
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
        }
    }

    // 2. 预计算每个路径段的 y 范围（Y 轴预过滤优化的关键）
    const auto path_points = yingshi::materializePath(path);
    size_t path_seg_count = (path_points.size() > 1) ? (path_points.size() - 1) : 0;
    std::vector<double> seg_min_y(path_seg_count), seg_max_y(path_seg_count);
    for (size_t pi = 0; pi < path_seg_count; ++pi) {
        double y1 = path_points[pi].getY();
        double y2 = path_points[pi + 1].getY();
        seg_min_y[pi] = std::min(y1, y2) - half_cov;
        seg_max_y[pi] = std::max(y1, y2) + half_cov;
    }

    // 3. 栅格化遍历（带 Y 轴预过滤 + 连续段聚合成 pass）
    // 不能直接统计命中网格的 segment 数：相邻碎段会把同一趟经过重复计数。
    // 正确做法：将连续覆盖段聚合为一次 pass，同一次 pass 内的多个段只算一次。
    int target_count = 0;
    int covered_count = 0;
    int total_passes = 0;  // 所有格元的 pass 次数之和，用于重叠率

    double gy_start = min_y + res / 2.0;
    int total_x_steps = static_cast<int>((max_x - min_x) / res) + 1;

    for (double gx = min_x + res / 2.0; gx <= max_x; gx += res) {
        for (double gy = gy_start; gy <= max_y; gy += res) {

            // a) 检查是否在目标区域内
            bool in_target = false;
            for (size_t ci = 0; ci < target_cells.size() && !in_target; ++ci) {
                const auto& ring = target_cells.getGeometry(ci).getExteriorRing();
                if (pointInPolygon(gx, gy, ring)) {
                    // 检查是否在孔洞内
                    bool in_hole = false;
                    for (const auto& hole : hole_rings) {
                        if (pointInPolygon(gx, gy, hole)) {
                            in_hole = true;
                            break;
                        }
                    }
                    if (!in_hole) in_target = true;
                }
            }

            if (!in_target) continue;
            ++target_count;

            // b) 统计该格元被路径扫过的次数（连续段聚合，避免碎段重复计数）
            int passes = 0;
            bool in_pass = false;
            for (size_t pi = 0; pi < path_seg_count; ++pi) {
                if (gy < seg_min_y[pi] || gy > seg_max_y[pi]) {
                    in_pass = false;  // Y 轴预过滤：段完全覆盖不到该行，断开 pass
                    continue;
                }
                double dist = pointToSegmentDist(
                    f2c::types::Point(gx, gy),
                    path_points[pi],
                    path_points[pi + 1]
                );
                if (dist <= half_cov) {
                    if (!in_pass) {
                        ++passes;
                        in_pass = true;
                    }
                } else {
                    in_pass = false;
                }
            }
            if (passes > 0) {
                ++covered_count;
                total_passes += passes;
                if (out_covered) out_covered->push_back({gx, gy});
            } else {
                if (out_uncovered) out_uncovered->push_back({gx, gy});
            }
        }

        // 进度日志（每 10 列输出一次，避免刷屏）
        int col_idx = static_cast<int>((gx - min_x) / res);
        if (col_idx % 10 == 0) {
            double pct = (total_x_steps > 0) ? (100.0 * col_idx / total_x_steps) : 100.0;
            fprintf(stderr, "\r  [GridEval] %.0f%% (col %d/%d), target_cells=%d, covered=%d",
                    pct, col_idx, total_x_steps, target_count, covered_count);
        }
    }
    fprintf(stderr, "\n");  // 换行结束进度

    if (target_count == 0) return {0.0, 0.0};

    double covered_area = covered_count * res * res;
    double rate = static_cast<double>(covered_count) / target_count;

    // 基于最终路径扫掠的重叠率：(总 pass 次数 - 被覆盖格元数) / 目标格元总数
    if (out_overlap_rate) {
        *out_overlap_rate = (target_count > 0)
            ? std::max(0.0, static_cast<double>(total_passes - covered_count) / target_count)
            : 0.0;
    }

    return {covered_area, rate};
}

/// 几何估算法：快速估算覆盖率
inline std::pair<double, double> computeCoverageGeometric(
    const f2c::types::Swaths& swaths,
    double net_area,
    double coverage_width)
{
    if (net_area <= 0.0) return {0.0, 0.0};

    double swath_total = 0.0;
    for (size_t i = 0; i < swaths.size(); ++i) {
        swath_total += pointDistance(swaths.at(i).startPoint(), swaths.at(i).endPoint());
    }

    // 简单估算：swath 总长 × 覆盖宽度
    double covered = swath_total * coverage_width;
    double rate = std::min(1.0, covered / net_area);
    return {covered, rate};
}

// ============================================================================
// 5. 综合评估函数
// ============================================================================

/// 一站式评估：计算所有指标并返回综合评分
inline EvalResult evaluatePlan(
    const f2c::types::Path& path,
    const f2c::types::Swaths& swaths,
    const f2c::types::Cells& target_cells,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double planning_time_ms,
    const EvalParams& params)
{
    EvalResult r;

    // ── 元信息 ──
    r.path_point_count = static_cast<int>(yingshi::materializePath(path).size());
    r.swath_count = static_cast<int>(swaths.size());
    r.cell_count = static_cast<int>(target_cells.size());
    r.planning_time_ms = planning_time_ms;

    // ── 目标区域净面积（F2C 自带面积，已处理孔洞；可用 full_net_area_override 修正 headland 侵蚀偏差） ──
    r.net_area = 0.0;
    if (params.full_net_area_override > 0.0) {
        r.net_area = params.full_net_area_override;
    } else {
        for (size_t ci = 0; ci < target_cells.size(); ++ci) {
            r.net_area += target_cells.getGeometry(ci).area();
        }
    }

    // ── 覆盖率 ──
    bool grid_overlap_ready = false;  // 网格模式已在 computeCoverageGrid 中算好重叠率
    if (params.use_grid_method) {
        r.coverage_method = "grid";
        r.grid_resolution = params.grid_resolution;
        auto [covered_area, rate] = computeCoverageGrid(path, target_cells, hole_rings, params,
            &r.covered_grid, &r.uncovered_grid, &r.overlap_rate);
        r.covered_area = covered_area;
        r.coverage_rate = rate;
        grid_overlap_ready = true;
    } else {
        r.coverage_method = "geometric";
        double effective_width = params.coverage_width * (1.0 - params.swath_overlap_ratio);
        auto [covered_area, rate] = computeCoverageGeometric(swaths, r.net_area, effective_width);
        r.covered_area = covered_area;
        r.coverage_rate = rate;
    }
    r.coverage_pass = (r.coverage_rate >= params.coverage_threshold);

    // ── 路径总长 ──
    r.total_distance = computeTotalDistance(path);

    // ── Swath 总长 ──
    r.swath_total_dist = computeSwathTotalDist(swaths);

    // ── 有效工作比（原始值，可 >1.0 表示存在大量重叠） ──
    r.work_ratio = (r.total_distance > 0.0)
                   ? (r.swath_total_dist / r.total_distance)
                   : 0.0;

    const double effective_width =
        params.coverage_width * (1.0 - params.swath_overlap_ratio);

    // ── 转弯次数 ──
    r.turn_count = countTurns(
        path, params.turn_angle_threshold,
        coverage_scoring::turnMergeDistance(
            effective_width, params.min_turn_merge_distance));

    // ── 重叠率 ──
    // 网格法：基于最终路径扫掠的逐格元访问计数（真实重叠，无估算）
    // 几何法：swath 总长 × 有效间距 vs 净面积（近似，不读最终路径）
    if (!grid_overlap_ready) {
        if (r.net_area > 0.0) {
            double expected_coverage = r.swath_total_dist * effective_width;
            r.overlap_rate = std::max(0.0, (expected_coverage - r.net_area) / r.net_area);
        }
    }
    // ── 曲率 ──
    auto [avg_curv, max_curv] = computeCurvature(path);
    r.avg_curvature = avg_curv;
    r.max_curvature = max_curv;

    // ── 综合评分 ──
    // CoverageGate：90% 为零，达到 coverage_threshold 时才满分。
    r.coverage_gate = coverage_scoring::coverageGate(
        r.coverage_rate, params.coverage_threshold);

    // 路径长度得分：理想路径 = 面积 / 有效间距（全覆盖所需最短swath总长）
    double ideal_dist = r.net_area / std::max(0.01, effective_width);
    r.score_distance = coverage_scoring::distanceScore(
        r.total_distance, ideal_dist);

    // 有效工作比以 1.0 为最佳；偏低和超过 1.0 都会扣分。
    r.score_work_ratio = coverage_scoring::workRatioScore(r.work_ratio);

    // 往复式覆盖固有的 N-1 次掉头不扣分，只惩罚额外转弯。
    r.score_turns = coverage_scoring::turnScore(
        r.turn_count, r.swath_count);

    // 网格法会测到主动设置的相邻 swath 重叠，几何法已用有效宽度扣除。
    r.score_overlap = coverage_scoring::overlapScore(
        r.overlap_rate,
        grid_overlap_ready ? params.swath_overlap_ratio : 0.0);

    // 耗时得分（5 秒为上限）
    r.score_time = std::max(0.0, 1.0 - r.planning_time_ms / 5000.0);

    // 平滑度得分
    r.score_smoothness = (params.max_diff_curv > 0.0)
                         ? std::max(0.0, 1.0 - r.avg_curvature / params.max_diff_curv)
                         : 1.0;

    // 效率分 = 根据掉头模式选择权重
    // direct 模式：零半径原地转，曲率/平滑度无意义，权重分配到其他项
    bool is_direct = (params.turn_planner_type &&
                      std::string(params.turn_planner_type) == "direct");
    if (is_direct) {
        // 5 项加权（去掉平滑度 5%，分配到路径长度和有效工作比）
        r.efficiency_score = 100.0 * (
            0.33 * r.score_work_ratio +
            0.27 * r.score_distance +
            0.15 * r.score_turns +
            0.15 * r.score_overlap +
            0.10 * r.score_time
        );
        r.score_smoothness = 0.0;  // direct 模式不参与评分
    } else {
        // 6 项加权（含平滑度，用于 dubins/reeds_shepp）
        r.efficiency_score = 100.0 * (
            0.30 * r.score_work_ratio +
            0.25 * r.score_distance +
            0.15 * r.score_turns +
            0.15 * r.score_overlap +
            0.10 * r.score_time +
            0.05 * r.score_smoothness
        );
    }

    // 最终得分
    r.single_score = r.coverage_gate * r.efficiency_score;

    return r;
}

// ============================================================================
// 6. 格式化报告输出
// ============================================================================

/// 生成评估报告字符串
inline std::string formatEvalReport(const EvalResult& r, const char* scenario_name) {
    bool is_direct = (r.turn_planner_used && std::string(r.turn_planner_used).find("Direct") == 0);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "\n========== F2C \u8986\u76d6\u8def\u5f84\u89c4\u5212 - \u7b97\u6cd5\u8bc4\u4f30\u62a5\u544a ==========\n";
    if (scenario_name) oss << "\u573a\u666f: " << scenario_name << "\n";
    oss << "\u8986\u76d6\u9762\u79ef: " << r.net_area << " m\u00b2\n";
    oss << "\u8986\u76d6\u7387\u65b9\u6cd5: " << r.coverage_method << "\n";
    oss << "\n--- \u7b2c\u4e00\u5c42: \u8986\u76d6\u5b8c\u6574\u6027 ---\n";
    oss << "\u76ee\u6807\u51c0\u9762\u79ef: " << r.net_area << " m\u00b2\n";
    oss << "\u5df2\u8986\u76d6\u9762\u79ef: " << r.covered_area << " m\u00b2\n";
    oss << std::setprecision(2) << "\u8986\u76d6\u7387: " << (r.coverage_rate*100) << "%  "
        << (r.coverage_pass ? "[\u901a\u8fc7]" : "[\u4e0d\u5408\u683c]") << "\n";
    oss << std::setprecision(3) << "\u672a\u8986\u76d6\u9762\u79ef: "
        << (r.net_area*(1-r.coverage_rate)) << " m\u00b2\n";
    oss << "\n--- \u7b2c\u4e8c\u5c42: \u6548\u7387\u6307\u6807 ---\n";
    oss << "\u8def\u5f84\u603b\u957f: " << r.total_distance << " m\n";
    oss << "Swath \u603b\u957f: " << r.swath_total_dist << " m\n";
    oss << std::setprecision(1) << "\u6709\u6548\u5de5\u4f5c\u6bd4: " << (r.work_ratio*100) << "%"
        << (r.work_ratio > 1.0 ? " \u26a0\ufe0f \u91cd\u53e0\u8fc7\u591a\uff0cSwath\u603b\u957f>\u8def\u5f84\u603b\u957f" : "") << "\n";
    oss << "\u8f6c\u5f2f\u6b21\u6570: " << r.turn_count << "\n";
    oss << "\u91cd\u53e0\u7387: " << (r.overlap_rate*100) << "%\n";
    oss << std::setprecision(0) << "\u89c4\u5212\u8017\u65f6: " << r.planning_time_ms << " ms\n";
    if (!is_direct) {
        oss << std::setprecision(3) << "\u5e73\u5747\u66f2\u7387: " << r.avg_curvature << "\n";
        oss << "\u6700\u5927\u66f2\u7387: " << r.max_curvature << "\n";
    }
    if (r.turn_planner_used && r.turn_planner_used[0])
        oss << "\u6389\u5934\u89c4\u5212\u5668: " << r.turn_planner_used << "\n";
    oss << "\u6389\u5934\u53ef\u884c\u6027: "
        << (r.turn_total_count>0 ? (r.turn_feasibility_pass?"[\u53ef\u884c]":"[\u4e0d\u53ef\u884c]") : "N/A") << "\n";
    oss << "\n--- \u7efc\u5408\u8bc4\u5206 ---\n";
    double w_work = is_direct ? 33.0 : 30.0;
    double w_dist = is_direct ? 27.0 : 25.0;
    oss << std::setprecision(3) << "\u8986\u76d6\u7387\u7cfb\u6570: " << r.coverage_gate
        << " (90%\u81f3\u8fbe\u6807\u9608\u503c\u7684\u4e09\u6b21\u95e8\u63a7)\n";
    oss << "\u6709\u6548\u5de5\u4f5c\u6bd4: " << (r.score_work_ratio*w_work) << "/" << (int)w_work << "\n";
    oss << "\u8def\u5f84\u957f\u5ea6: " << (r.score_distance*w_dist) << "/" << (int)w_dist << "\n";
    oss << "\u8f6c\u5f2f\u6b21\u6570: " << (r.score_turns*15) << "/15\n";
    oss << "\u91cd\u53e0\u7387: " << (r.score_overlap*15) << "/15\n";
    oss << "\u89c4\u5212\u8017\u65f6: " << (r.score_time*10) << "/10\n";
    if (!is_direct) oss << "\u8def\u5f84\u5e73\u6ed1\u5ea6: " << (r.score_smoothness*5) << "/5\n";
    oss << "\u6548\u7387\u5206\u5408\u8ba1: " << r.efficiency_score << "\n";
    oss << std::setprecision(1) << "\u7efc\u5408\u5f97\u5206: " << r.single_score << " / 100\n";
    oss << "========================================================\n";
    return oss.str();
}

/// 生成 A/B 对比报告字符串
inline std::string formatComparisonReport(
    const EvalResult& baseline,
    const EvalResult& optimized)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    auto pct_change = [](double old_val, double new_val) -> std::string {
        if (std::abs(old_val) < 1e-9) return "  N/A  ";
        double change = (new_val - old_val) / old_val * 100.0;
        std::ostringstream s;
        s << std::setw(6) << std::setprecision(1) << change << "%";
        return s.str();
    };

    const char* B = "║";

    oss << "\n";
    oss << "╔══════════════════════════════════════════════╗\n";
    oss << B << "  方案对比                                     " << B << "\n";
    oss << "╠══════════════════════════════════════════════╣\n";
    oss << B << "  指标          │ 基线(A)  │ 优化(B)  │ 变化      " << B << "\n";
    oss << B << " ────────────────────────────────────────────  " << B << "\n";

    auto row = [&](const char* name, double a, double b, const char* /*unit*/, bool pct_fmt) {
        oss << B << "  " << std::setw(13) << std::left << name << "│ ";
        if (pct_fmt) {
            oss << std::setw(7) << std::setprecision(1) << (a * 100.0) << "%  │ "
                << std::setw(7) << std::setprecision(1) << (b * 100.0) << "%  │ ";
        } else {
            oss << std::setw(7) << std::setprecision(1) << a << "  │ "
                << std::setw(7) << std::setprecision(1) << b << "  │ ";
        }
        oss << pct_change(a, b) << "    " << B << "\n";
    };

    auto row_pct = [&](const char* name, double a, double b) {
        row(name, a, b, "%", true);
    };

    row_pct("覆盖率", baseline.coverage_rate, optimized.coverage_rate);
    row("路径总长", baseline.total_distance, optimized.total_distance, "m", false);
    row_pct("有效工作比", baseline.work_ratio, optimized.work_ratio);
    row("转弯次数", baseline.turn_count, optimized.turn_count, "", false);
    row_pct("重叠率", baseline.overlap_rate, optimized.overlap_rate);
    row("规划耗时", baseline.planning_time_ms, optimized.planning_time_ms, "ms", false);

    oss << B << " ────────────────────────────────────────────  " << B << "\n";

    oss << std::setprecision(1);
    oss << B << "  综合得分      │ " << std::setw(7) << baseline.single_score
        << "  │ " << std::setw(7) << optimized.single_score << "  │ "
        << std::setw(6) << (optimized.single_score - baseline.single_score)
        << "     " << B << "\n";

    const char* conclusion;
    if (optimized.single_score >= baseline.single_score + 1.0) {
        conclusion = "✅ 优化有效";
    } else if (optimized.single_score >= baseline.single_score - 1.0) {
        conclusion = "➡️ 无明显差异";
    } else {
        conclusion = "❌ 优化退化";
    }
    oss << B << "  结论          │ " << std::setw(33) << std::left << conclusion << B << "\n";
    oss << "╚══════════════════════════════════════════════╝\n";

    return oss.str();
}

/// 将网格覆盖数据写入 JSON 文件（供 Python 渲染直接读取，避免重算）
inline void writeGridJson(const EvalResult& r, const std::string& filepath) {
    std::ofstream f(filepath);
    if (!f.is_open()) {
        fprintf(stderr, "[GridJson] ERROR: cannot write %s\n", filepath.c_str());
        return;
    }
    f << std::fixed << std::setprecision(3);
    f << "{\n";
    f << "  \"grid_resolution\": " << r.grid_resolution << ",\n";
    f << "  \"coverage_rate\": " << r.coverage_rate << ",\n";

    auto write_points = [&f](const char* key, const auto& pts) {
        f << "  \"" << key << "\": [";
        for (size_t i = 0; i < pts.size(); ++i) {
            if (i > 0) f << ",";
            if (i % 20 == 0) f << "\n    ";
            f << "[" << pts[i].first << "," << pts[i].second << "]";
        }
        f << "\n  ]";
    };

    write_points("covered", r.covered_grid);
    f << ",\n";
    write_points("uncovered", r.uncovered_grid);
    f << "\n}\n";
    f.close();
    fprintf(stderr, "[GridJson] wrote %s (%zu covered + %zu uncovered points)\n",
            filepath.c_str(), r.covered_grid.size(), r.uncovered_grid.size());
}
