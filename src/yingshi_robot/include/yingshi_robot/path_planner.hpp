#pragma once
#include <fields2cover.h>
#include <vector>
#include <string>
#include "yingshi_robot/planner_params.hpp"

namespace yingshi {

// ========== 路径规划模块 ==========
// 负责：RDP 路径简化、TSP 路由规划、路径后处理

// RDP (Ramer-Douglas-Peucker) 路径简化（分段感知版）
// 先检测转弯点（方向突变标记为段边界），再逐段执行 RDP
// epsilon: 简化容差 (m)
// turn_angle_threshold: 转弯检测角度阈值 (rad)
std::vector<f2c::types::PathState> simplifyPathRDP(
    const f2c::types::Path& path,
    double epsilon,
    double turn_angle_threshold);

// 简化路径并返回简化后的 Path 对象（不保留 PathState 类型信息）
f2c::types::Path simplifyPath(
    const f2c::types::Path& path,
    double epsilon,
    double turn_angle_threshold);

// 检查路径中是否存在不可执行的掉头（曲率超过限制）
// 返回不可执行段的数量
size_t checkInfeasibleTurns(
    const f2c::types::Path& path,
    double max_diff_curv,
    double min_turning_radius);

// 双向连接修复：在两段路径之间插入中间点防止 getInAngle 崩溃
// 当两点距离 < min_dist 时插入中间过渡点
f2c::types::Path repairConnection(
    const f2c::types::Point& p1,
    const f2c::types::Point& p2,
    double min_dist = 0.01);

// 贪心 Cell 排序：根据 swath 端点实际位置动态决定遍历顺序
// 从 C0 出发，每次选择端点最近的未访问 cell，支持自动翻转 swath 方向
// hole_rings: 孔洞环（用于穿洞检测，空则跳过）
// cell_order: [out] 遍历顺序 → 原始 no_hl 索引
void greedyCellOrder(
    f2c::types::SwathsByCells& swaths_by_cells,
    std::vector<size_t>& cell_order,
    const std::vector<f2c::types::LinearRing>& hole_rings);

}  // namespace yingshi
