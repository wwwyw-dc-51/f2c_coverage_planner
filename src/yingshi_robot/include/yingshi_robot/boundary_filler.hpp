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
    double shrink_dist);

// ========== 孔洞检测工具 ==========

// 点-in-多边形检测（射线法）
bool pointInPolygon(double px, double py, const f2c::types::LinearRing& ring);

// 点是否在任意孔洞内
bool pointInAnyHole(double px, double py,
                    const std::vector<f2c::types::LinearRing>& hole_rings);

// 线段是否穿过孔洞（采样检测）
bool segmentCrossesHole(double x0, double y0, double x1, double y1,
                        const std::vector<f2c::types::LinearRing>& hole_rings,
                        int num_samples = 10);

}  // namespace yingshi
