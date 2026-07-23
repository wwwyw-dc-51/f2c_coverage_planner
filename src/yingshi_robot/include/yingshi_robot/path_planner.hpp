#pragma once
#include <fields2cover.h>
#include <string>
#include <vector>
#include "yingshi_robot/planner_params.hpp"

namespace yingshi {

// ========== 路径规划模块 ==========
// 负责：RDP 路径简化、TSP 路由规划、路径后处理

// 将一条 swath 作为独立 group 追加，并把 connection 放在该 group 之前。
// 禁止用 addSwath() 后补 addConnection()，否则 F2C 会把 swath 合并进旧 group。
void appendConnectedSwath(
    f2c::types::Route& route,
    const f2c::types::MultiPoint& connection,
    const f2c::types::Swath& swath);

// 按 Route 中的原始折线逐段生成 direct 路径，不调用 F2C simplifyConnection。
// connection 的全部控制点都会成为 TURN 段端点，swath 保持 SWATH 类型。
f2c::types::Path planDirectPath(
    const f2c::types::Route& route,
    double cruise_velocity);

// 在 Route 全部生成完成后，修复穿越孔洞的连接折线。
// 前提：swath 端点位于指定 clearance 之外；返回实际修改的 connection 数量。
size_t repairRouteConnectionsAroundHoles(
    f2c::types::Route& route,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double clearance);

// 修复穿过外环凹口的连接段，保证连接折线留在规划 Cell 内。
size_t repairRouteConnectionsOutsideCell(
    f2c::types::Route& route,
    const f2c::types::Cell& planning_cell);

// 对很短的连接尝试首尾直连。
// 仅当直线完全位于 planning_cell 自由空间、没有穿过孔洞，且确实
// 比原折线更短时替换；不改变连接两端。最终 footprint 安全检查仍由
// PlannerCore 的现有安全门负责。
size_t shortenSafeRouteConnections(
    f2c::types::Route& route,
    const f2c::types::Cell& planning_cell,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double max_direct_length = 2.0,
    double min_saved_length = 0.05);

// 删除连接中由边界交点数值误差产生的微型折返段，返回删除的中间点数。
// 只尝试移除相邻短段涉及的中间点；替代线段必须保持在自由空间内，
// 且首尾端点不变。失败时保留原连接。
size_t removeSafeMicroDetours(
    f2c::types::Route& route,
    const f2c::types::Cell& planning_cell,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    double max_micro_segment = 2e-4);

// swath 端点缩进后，同步每条跨 group connection 的首尾点。
// 距新端点不超过 max_endpoint_shift 的单控制点视为旧端点并删除；
// 更远的单控制点和多点 connection 的中间绕障点保持不变。
// 返回实际修改的 connection 数量。
size_t synchronizeRouteConnectionEndpoints(
    f2c::types::Route& route,
    double max_endpoint_shift);

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

// v9.12 Cell 排序：保留 F2C 原生排序，在四种合法入口变体中优化
// 连接代价；有孔洞时以极角 Cell 顺序为安全骨架并做全链动态规划，
// 无孔洞时联合选择最近 Cell 与入口变体。
// hole_rings: 孔洞环（用于连接穿洞检测，空则跳过）
// swath_order_type: "boustrophedon"（默认）| "snake" | "spiral"
// cell_order: [out] 遍历顺序 → 原始 no_hl 索引
void greedyCellOrder(
    f2c::types::SwathsByCells& swaths_by_cells,
    std::vector<size_t>& cell_order,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    const std::string& swath_order_type = "boustrophedon");

}  // namespace yingshi
