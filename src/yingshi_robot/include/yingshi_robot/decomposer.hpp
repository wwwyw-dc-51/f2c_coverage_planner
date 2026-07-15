#pragma once
#include <fields2cover.h>
#include <vector>
#include "yingshi_robot/planner_params.hpp"

namespace yingshi {

// ========== 区域分解模块 ==========
// Sweep 扫描线分解：用孔洞顶点 y 坐标做水平切分线，生成全宽条带 Cells
// 相比 Boustrophedon 网格分解，cell 数大幅减少（S3: 42→7）

// 主分解函数：将 work_area 分解为矩形条带
f2c::types::Cells rectilinearDecompose(
    const f2c::types::Cell& work_area,
    const f2c::types::Cell& grid_src,
    const DecomposerParams& params);

// 移除环上共线冗余顶点（OGR buffer 会产生多余顶点）
f2c::types::LinearRing simplifyRing(
    const f2c::types::LinearRing& ring,
    double angle_tol_deg = 0.5);

// 批量简化 Cells
f2c::types::Cells simplifyCells(
    const f2c::types::Cells& cells,
    double angle_tol_deg = 0.5);

// 计算 cell 的主方向（最长边缘方向）
double computeCellMainDirection(const f2c::types::Cell& cell);

// 过滤面积过小的 cell
f2c::types::Cells filterTinyCells(
    const f2c::types::Cells& cells,
    double min_area);

// 从多边形边缘提取所有唯一边缘方向角（去重排序）
std::vector<double> extractEdgeAngles(
    const f2c::types::Cell& cell,
    double dedup_tolerance_deg = 2.0);

// 从多边形边缘提取分解角度候选（边缘垂直方向，Huang 2001定理）
std::vector<double> extractDecompositionAngles(
    const f2c::types::Cell& polygon,
    double merge_angle_threshold_deg = 60.0);

}  // namespace yingshi
