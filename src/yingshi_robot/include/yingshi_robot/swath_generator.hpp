#pragma once
#include <fields2cover.h>
#include <vector>
#include "yingshi_robot/planner_params.hpp"

namespace yingshi {

// ========== Swath 生成模块 ==========
// 负责：swath 角度优化、端点调整、斜边检测、几何变换

// 计算 Swath 长度（欧氏距离）
double swathLength(const f2c::types::Swath& swath);

// 过滤长度不足的短 swaths，返回 removed_count
f2c::types::Swaths filterShortSwaths(
    const f2c::types::Swaths& swaths,
    double min_length,
    size_t& removed_count);

// 双向 Swath 端点调整（统一收敛/延伸）
// distance > 0：端点向中心收缩（闭合边界安全模式）
// distance < 0：端点向外延伸（开放边界覆盖模式）
// distance = 0：不调整
f2c::types::Swath adjustSwathEndpoints(
    const f2c::types::Swath& swath,
    double distance);

// 批量调整 SwathsByCells 中所有 swaths 的端点
f2c::types::SwathsByCells adjustSwathsEndpoints(
    const f2c::types::SwathsByCells& swaths_by_cells,
    double distance);

// 计算多边形最长边的方向角（委托给 decomposer 模块的 computeCellMainDirection）
double computePolygonMainDirection(const f2c::types::Cell& cell);

// 旋转 Cell 的所有顶点（绕原点，逆时针为正）
f2c::types::Cell rotateCell(const f2c::types::Cell& cell, double angle);

// 旋转 Swath（绕原点）
f2c::types::Swath rotateSwath(const f2c::types::Swath& sw, double angle);

// 检测斜边并返回最佳 swath 角度（用于斜边 cell 的自适应方向选择）
// 若 cell 贴近多边形斜边界（与 sweep 方向夹角 15°~75°），则返回斜边方向
double detectSlantedBoundaryAngle(
    const f2c::types::Cell& cell,
    const f2c::types::Cell& full_polygon,
    double default_angle,
    double cov_width);

// Swath 多角度选择：尝试多个候选角度，返回 swaths 数量最少的方案
f2c::types::Swaths optimizeSwathAngle(
    const f2c::types::Cell& cell,
    f2c::sg::BruteForce& swath_generator,
    double cov_width,
    const std::vector<double>& angle_candidates);

// ========== 全 Cell Swath 生成（全局角度优化 + 边界填补）==========
// 这是从 ROS 节点提取的纯算法函数，无 ROS 依赖。
// 参数 swath_angle_optimization 为 true 时做全局扫描：测试所有候选角度，
// 选总 swath 数最少的统一角度；否则逐 cell 独立优化。
// 返回按 cell 分组的 swaths（已 fillBoundaryGaps + filterShortSwaths）。
f2c::types::SwathsByCells generateSwathsForAllCells(
    const f2c::types::Cells& no_hl,
    const f2c::types::Cell& full_polygon,
    double r_w,
    double coverage_width,
    double swath_endpoint_shrink_distance,
    double min_swath_length,
    bool swath_angle_optimization,
    const std::vector<double>& swath_angle_candidates);

}  // namespace yingshi
