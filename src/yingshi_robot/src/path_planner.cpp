/**
 * @file path_planner.cpp
 * @brief 路径规划模块实现 — RDP 简化、不可执行掉头检测、连接修复
 *
 * 从 polygon_planner_node.cpp 提取，属于模块化重构 Step 5。
 */

#include "yingshi_robot/path_planner.hpp"
#include "yingshi_robot/boundary_filler.hpp"  // segmentCrossesHole, pointInAnyHole
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace yingshi {

// ========== RDP 路径简化（分段感知版）==========
// Ramer-Douglas-Peucker 算法，剔除共线或近似共线的冗余路径点
//
// 策略：先检测路径中的转弯点（方向突变），标记为必须保留的段边界，
// 然后只在相邻转弯点之间的"准直线段"内执行 RDP，保护曲线连接不被破坏。
std::vector<f2c::types::PathState> simplifyPathRDP(
    const f2c::types::Path& path,
    double epsilon,
    double turn_angle_threshold)
{
    if (path.size() < 3 || epsilon <= 0.0) {
        std::vector<f2c::types::PathState> result;
        for (size_t i = 0; i < path.size(); ++i) {
            result.push_back(path[i]);
        }
        return result;
    }

    // 将路径转为点列表用于 RDP
    struct Pt { double x, y; int idx; };
    std::vector<Pt> points;
    for (size_t i = 0; i < path.size(); ++i) {
        points.push_back({path[i].point.getX(), path[i].point.getY(), static_cast<int>(i)});
    }

    // ── 步骤1：检测转弯点（段边界）──
    std::vector<bool> keep(points.size(), false);
    keep[0] = true;
    keep[points.size() - 1] = true;

    std::vector<int> turn_points;
    turn_points.push_back(0);

    for (size_t i = 2; i < points.size(); ++i) {
        double dx1 = points[i-1].x - points[i-2].x;
        double dy1 = points[i-1].y - points[i-2].y;
        double dx2 = points[i].x - points[i-1].x;
        double dy2 = points[i].y - points[i-1].y;

        double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

        if (len1 > 1e-9 && len2 > 1e-9) {
            double cross = dx1 * dy2 - dy1 * dx2;
            double dot = dx1 * dx2 + dy1 * dy2;
            double angle = std::abs(std::atan2(cross, dot));

            if (angle > turn_angle_threshold) {
                keep[i - 1] = true;
                turn_points.push_back(static_cast<int>(i - 1));
            }
        }
    }
    turn_points.push_back(static_cast<int>(points.size()) - 1);

    // ── 步骤2：RDP 递归 ──
    std::function<void(int, int)> rdp = [&](int start, int end) {
        if (end - start <= 1) return;

        double max_dist = 0.0;
        int max_idx = start;

        double dx = points[end].x - points[start].x;
        double dy = points[end].y - points[start].y;
        double len = std::sqrt(dx * dx + dy * dy);

        for (int i = start + 1; i < end; ++i) {
            if (keep[i]) continue;

            double dist;
            if (len < 1e-12) {
                double pdx = points[i].x - points[start].x;
                double pdy = points[i].y - points[start].y;
                dist = std::sqrt(pdx * pdx + pdy * pdy);
            } else {
                double cross = std::abs(
                    (points[i].x - points[start].x) * dy -
                    (points[i].y - points[start].y) * dx
                );
                dist = cross / len;
            }

            if (dist > max_dist) {
                max_dist = dist;
                max_idx = i;
            }
        }

        if (max_dist > epsilon) {
            keep[max_idx] = true;
            rdp(start, max_idx);
            rdp(max_idx, end);
        }
    };

    // ── 步骤3：逐段执行 RDP ──
    for (size_t s = 0; s < turn_points.size() - 1; ++s) {
        rdp(turn_points[s], turn_points[s + 1]);
    }

    // 构建简化后的路径
    std::vector<f2c::types::PathState> simplified;
    for (size_t i = 0; i < points.size(); ++i) {
        if (keep[i]) {
            simplified.push_back(path[points[i].idx]);
        }
    }

    return simplified;
}

// ========== 简化路径（返回 Path 对象）==========
f2c::types::Path simplifyPath(
    const f2c::types::Path& path,
    double epsilon,
    double turn_angle_threshold)
{
    auto states = simplifyPathRDP(path, epsilon, turn_angle_threshold);
    f2c::types::Path result;
    for (const auto& s : states) {
        result.addState(s);
    }
    return result;
}

// ========== 不可执行掉头检测 ==========
// 遍历路径中所有连续段，检查曲率是否超过限制
size_t checkInfeasibleTurns(
    const f2c::types::Path& path,
    double max_diff_curv,
    double min_turning_radius)
{
    size_t infeasible_count = 0;
    (void)min_turning_radius;  // 保留接口兼容性，曲率检测仅用 max_diff_curv

    for (size_t i = 0; i + 2 < path.size(); ++i) {
        double dx1 = path[i+1].point.getX() - path[i].point.getX();
        double dy1 = path[i+1].point.getY() - path[i].point.getY();
        double dx2 = path[i+2].point.getX() - path[i+1].point.getX();
        double dy2 = path[i+2].point.getY() - path[i+1].point.getY();

        double len1 = std::hypot(dx1, dy1);
        double len2 = std::hypot(dx2, dy2);

        if (len1 < 1e-9 || len2 < 1e-9) continue;

        double cross = dx1 * dy2 - dy1 * dx2;
        double curv = std::abs(cross) / (len1 * len2);

        if (curv > max_diff_curv) {
            ++infeasible_count;
        }
    }

    return infeasible_count;
}

// ========== 双向连接修复 ==========
// 在两段路径之间插入中间过渡点，防止直接连接时 getInAngle 崩溃
f2c::types::Path repairConnection(
    const f2c::types::Point& p1,
    const f2c::types::Point& p2,
    double min_dist)
{
    f2c::types::Path path;
    (void)min_dist;  // 保留参数以备后续需要插入中间过渡点

    // 始终保留两个端点，确保路径至少有两个点（避免下游越界）
    f2c::types::PathState s1, s2;
    s1.point = p1;
    s2.point = p2;
    path.addState(s1);
    path.addState(s2);

    return path;
}

// ========== 贪心 Cell 排序 ==========
// 替代死板的圆形排序 (circular sort)，根据 Boustrophedon 排序后
// swath 的实际端点位置，动态决定 cell 遍历顺序。
//
// 算法：
//   1. 从 C0 出发，cur = C0.last_swath.endPoint()
//   2. 每次在未访问 cell 中选择端点距离 cur 最近的
//   3. 两个候选方向：正常 (first.start) / 反转 (last.end)
//   4. 穿洞检测 → 穿洞则距离 +1000（最后手段才选）
//   5. 选 reverse 则翻转该 cell 的 swaths（顺序 + 每条起终点）
//   6. 更新 cur 为选中 cell 的出口，继续直到所有 cell 访问完毕
void greedyCellOrder(
    f2c::types::SwathsByCells& swaths_by_cells,
    std::vector<size_t>& cell_order,
    const std::vector<f2c::types::LinearRing>& hole_rings)
{
    const size_t n_cells = swaths_by_cells.size();

    // 初始化 cell_order 为恒等映射
    cell_order.resize(n_cells);
    for (size_t i = 0; i < n_cells; ++i) cell_order[i] = i;

    // 单 cell 无需排序
    if (n_cells <= 1) return;

    std::vector<bool> visited(n_cells, false);
    f2c::types::SwathsByCells result;
    std::vector<size_t> new_order;

    // 从 C0 出发
    visited[0] = true;
    result.push_back(swaths_by_cells[0]);
    new_order.push_back(0);

    // 当前出口 = C0 最后一条 swath 的终点
    double cur_x, cur_y;
    {
        const auto& c0 = swaths_by_cells[0];
        const auto& last_sw = c0.at(c0.size() - 1);
        cur_x = last_sw.endPoint().getX();
        cur_y = last_sw.endPoint().getY();
    }

    // 贪心主循环
    while (new_order.size() < n_cells) {
        double best_dist = std::numeric_limits<double>::max();
        size_t best_ci = 0;
        bool best_reverse = false;

        for (size_t ci = 0; ci < n_cells; ++ci) {
            if (visited[ci]) continue;
            const auto& cell = swaths_by_cells[ci];
            if (cell.size() == 0) continue;

            const auto& first_sw = cell.at(0);
            const auto& last_sw = cell.at(cell.size() - 1);

            double fsx = first_sw.startPoint().getX();
            double fsy = first_sw.startPoint().getY();
            double lex = last_sw.endPoint().getX();
            double ley = last_sw.endPoint().getY();

            // 候选1: 正常方向 — cur → cell.first.start
            double dist_normal = std::hypot(cur_x - fsx, cur_y - fsy);
            if (!hole_rings.empty() &&
                yingshi::segmentCrossesHole(cur_x, cur_y, fsx, fsy, hole_rings, 20)) {
                dist_normal += 1000.0;  // 穿洞惩罚
            }

            // 候选2: 反转方向 — cur → cell.last.end
            double dist_rev = std::hypot(cur_x - lex, cur_y - ley);
            if (!hole_rings.empty() &&
                yingshi::segmentCrossesHole(cur_x, cur_y, lex, ley, hole_rings, 20)) {
                dist_rev += 1000.0;
            }

            if (dist_normal < best_dist) {
                best_dist = dist_normal;
                best_ci = ci;
                best_reverse = false;
            }
            if (dist_rev < best_dist) {
                best_dist = dist_rev;
                best_ci = ci;
                best_reverse = true;
            }
        }

        // 无有效候选（所有剩余 cell 均为空）→ 提前退出
        if (best_dist >= std::numeric_limits<double>::max() * 0.5) break;

        // 选中 best_ci，标记已访问
        visited[best_ci] = true;
        new_order.push_back(best_ci);

        // 先计算出口坐标（在 move 之前，避免 use-after-move）
        double next_x, next_y;
        {
            const auto& orig_cell = swaths_by_cells[best_ci];
            if (best_reverse) {
                // 出口 = 原 cell 第一个 swath 的起点（反转后最后一个 swath 的终点）
                next_x = orig_cell.at(0).startPoint().getX();
                next_y = orig_cell.at(0).startPoint().getY();
            } else {
                // 出口 = 最后一个 swath 的终点
                const auto& last_sw = orig_cell.at(orig_cell.size() - 1);
                next_x = last_sw.endPoint().getX();
                next_y = last_sw.endPoint().getY();
            }
        }

        auto selected_cell = std::move(swaths_by_cells[best_ci]);

        if (best_reverse) {
            // 反转整个 cell：倒序 swaths + 每条 swath 交换起终点
            f2c::types::Swaths reversed;
            for (int si = static_cast<int>(selected_cell.size()) - 1; si >= 0; --si) {
                const auto& sw = selected_cell[si];
                f2c::types::LineString rev_line;
                rev_line.addPoint(sw.endPoint());
                rev_line.addPoint(sw.startPoint());
                f2c::types::Swath rev_sw(rev_line, sw.getWidth(), sw.getId(), sw.getType());
                reversed.push_back(rev_sw);
            }
            selected_cell = reversed;
        }

        cur_x = next_x;
        cur_y = next_y;
        result.push_back(std::move(selected_cell));
    }

    swaths_by_cells = result;
    cell_order = new_order;
}

}  // namespace yingshi
