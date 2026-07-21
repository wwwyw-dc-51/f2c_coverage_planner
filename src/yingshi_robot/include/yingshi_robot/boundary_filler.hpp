#pragma once
#include <fields2cover.h>
#include <vector>
#include "yingshi_robot/planner_params.hpp"

namespace yingshi {

// ========== 边界补刀模块 ==========
// 负责：边界间隙检测与填充（外环+孔洞）、孔洞检测工具函数

// 边界间隙补填（完整版，v8/v9 成熟实现）
// 在 swath 排完后，检测并填充最外层到 cell 边界的剩余间隙。
// full_polygon: 原始完整多边形（外环 + 孔洞），用于确定真实边界。
// cell 是 no_hl 子区域，其边界已被 headland 侵蚀。
// 算法要点：
//   - 平行容差 cos(20°) = 0.9397（Phase 2B，覆盖斜边缝隙）
//   - 法向指向 cell 质心（外环）/ 远离孔洞中心（孔洞）
//   - getLinesInside 裁剪确保边界 swath 不超出多边形
//   - 出发侧填充插入最前，到达侧填充追加末尾
void fillBoundaryGaps(
    f2c::types::Swaths& cell_swaths,
    const f2c::types::Cell& cell,
    const f2c::types::Cell& full_polygon,
    double swath_angle,
    double cov_width,
    double shrink_dist,
    double robot_half_width = 0.0,
    double boundary_offset_override = -1.0);

// 单 Cell 窄矩形中，边界补线可能与 F2C 主 swath 近重合。
// 将这类轨迹重排为边界锚定的等距 swath，在不留下覆盖间隙的前提下
// 删除冗余轨迹。非矩形、宽区域或没有近重合线时不做修改。
size_t rebalanceNarrowCellSwaths(
    f2c::types::Swaths& swaths,
    const f2c::types::Cell& cell,
    double coverage_width);

// 按端点分别计算外边界/孔洞净空并调整 swath，避免中点靠近孔洞时
// 错误地同时缩短两个远离孔洞的安全端点。
f2c::types::Swath adjustSwathEndpointsForBoundaryClearance(
    const f2c::types::Swath& swath,
    const f2c::types::LinearRing& outer_ring,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double coverage_width,
    double default_margin);

// 自适应端点缩进：用 robot 四角 point-in-polygon 检测，只缩进会越界的端点
// robot_half_length: 沿 swath 方向的 robot 半长（行进方向）
// robot_half_width:  垂直 swath 方向的 robot 半宽
f2c::types::Swath adjustSwathEndpointsAdaptive(
    const f2c::types::Swath& swath,
    const f2c::types::LinearRing& outer_ring,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double robot_half_length,
    double robot_half_width,
    double shrink_distance);

// 从全部 Cell 的 swath 中删除功能性重复的内部接缝补线。
// 仅当接缝两侧原有 swath 的总间距不超过覆盖宽度加容差时删除；
// 真正超过覆盖能力的缺口仍保留一条补线。返回删除数量。
size_t pruneRedundantCellSeamFills(
    f2c::types::SwathsByCells& swaths_by_cells,
    const f2c::types::Cells& cells,
    const f2c::types::Cell& full_polygon,
    double cov_width,
    double gap_tolerance_ratio = 0.05);

// ========== 孔洞检测工具 ==========

// 从顶点构造闭合环；若输入未重复首点，则自动补上。
f2c::types::LinearRing makeClosedRing(
    const std::vector<f2c::types::Point>& points);

// 点-in-多边形检测（射线法）
bool pointInPolygon(double px, double py, const f2c::types::LinearRing& ring);

// 点是否在任意孔洞内
bool pointInAnyHole(double px, double py,
                    const std::vector<f2c::types::LinearRing>& hole_rings);

// 线段是否穿过孔洞内部（精确边界求交）；末参数仅为旧调用兼容。
bool segmentCrossesHole(double x0, double y0, double x1, double y1,
                        const std::vector<f2c::types::LinearRing>& hole_rings,
                        int num_samples = 10);

// ========== 边界策略：对 Route 中全部 swath 应用边界缩进/延伸 ==========
// 将边界预设解析为实际端点调整量，供所有规划管线共享。
double resolveBoundaryMargin(
    const std::string& boundary_type,
    double endpoint_shrink,
    double boundary_coverage_margin,
    double open_default_margin);

// 根据 boundary_type 决定 margin：
//   "closed" → 使用 shrink_margin（正值=向内收缩）
//   "open"   → 使用 boundary_coverage_margin（负值=向外延伸）
//   "custom" → 直接使用 boundary_coverage_margin
// 对每个 swath 端点独立判断外环/孔洞净空后调整，
// 并同步 route connection 端点。返回调整的 swath 组数。
size_t applyBoundaryMarginToRoute(
    f2c::types::Route& route,
    const f2c::types::LinearRing& outer_ring,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double coverage_width,
    double margin);

}  // namespace yingshi
