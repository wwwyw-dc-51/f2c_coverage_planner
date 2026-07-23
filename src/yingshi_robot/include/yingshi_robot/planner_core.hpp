#pragma once
// ========== PlannerCore 门面：纯 C++ 规划流水线，不含 ROS/文件/评分 ==========
//
// 依赖方向：
//   PlannerCore → F2C types (Point, Cell, Swath, Route, Path)
//   PlannerCore → coverage_planner_core 算法模块
//   ROS 节点 → PlannerCore（单方向消费 PlanningResult）
//
// PlannerCore 不知道 rclcpp、ROS 消息、ofstream、/tmp、评分公式。

#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <fields2cover.h>

#include "yingshi_robot/physical_footprint.hpp"
#include "yingshi_robot/traversability.hpp"

namespace yingshi {

// ========== 规划请求（不可变配置 + 几何输入）==========
struct PlanningRequest {
    // ── 几何输入 ──
    f2c::types::Cell polygon;                      // 外环 + 孔洞
    std::vector<f2c::types::LinearRing> holes;     // 孔洞内环（用于孔洞感知）

    // ── 机器人参数 ──
    double robot_width = 0.75;
    double coverage_width = 0.90;
    double min_turning_radius = 0.01;
    double max_diff_curv = 0.3;

    // ── Headland 参数 ──
    double mid_hl_width_ratio = 0.20;
    double no_hl_width_ratio = 0.00;

    // ── Swath 参数 ──
    double swath_overlap_ratio = 0.03;
    double swath_endpoint_shrink_distance = 0.03;
    double min_swath_length = 0.5;

    // ── 分解参数 ──
    bool decomposition_enabled = true;
    bool use_sweep_decomp = true;
    double merge_angle_threshold = 60.0;

    // ── Swath 优化 ──
    bool swath_angle_optimization = true;
    std::vector<double> swath_angle_candidates;

    // ── 排序参数 ──
    std::string swath_order_type = "boustrophedon";   // v9.12 排序；none：F2C TSP 基线
    std::string turn_planner_type = "direct";          // direct | dubins | reeds_shepp

    // ── 路径后处理 ──
    bool path_simplify_enabled = true;
    double path_simplify_tolerance = 0.05;
    double path_simplify_turn_threshold = 0.15;

    // ── 过滤 ──
    bool filter_tiny_cells = true;
    double min_cell_area_ratio = 2.0;

    // ── 边界策略 ──
    std::string boundary_type = "closed";
    double boundary_coverage_margin = -0.3;
    double boundary_open_default_margin = -0.3;

    // ── C-space 可通行性前处理 ──
    bool traversability_enabled = false;
    double cspace_clearance_margin = 0.0;
    double max_excluded_area_ratio = 0.05;

    // ── 独立实体碰撞检查 ──
    // Collision-only physical gate, separate from robot_width semantics.
    // 默认关闭以保持纯 C++ 历史基线；ROS 节点构造请求时显式开启。
    bool physical_collision_check_enabled = false;
    PhysicalFootprintParams physical_footprint;
};

// 单个中心可达分量的独立规划结果。分量之间没有隐式连线；
// 上层必须显式安排重定位，不能把多个 Path 展平成一条路径。
struct PlanningComponentResult {
    std::size_t component_index = 0;
    f2c::types::Cell planning_polygon;
    f2c::types::Cells coverage_target;

    f2c::types::Path path;
    std::vector<f2c::types::Point> path_points;
    std::vector<f2c::types::Point> path_waypoints;
    f2c::types::SwathsByCells cells_with_swaths;
    f2c::types::Route route;
    std::vector<size_t> cell_order;
    f2c::types::Cells decomposition_cells;  // no_hl 分解后的 cell 几何（供可视化）
    size_t total_swaths = 0;
    size_t total_connections = 0;

    double planning_time_ms = 0.0;
    size_t hole_crossing_segments = 0;
    size_t out_of_planning_area_segments = 0;
    bool path_has_crossings = false;
    bool path_leaves_planning_area = false;
    FootprintCollisionResult physical_collision;
    bool success = false;
    std::string error_message;
};

// ========== 规划结果（不可变输出）==========
struct PlanningResult {
    // ── 最终路径 ──
    f2c::types::Path path;                           // 含 PathState 的完整路径
    std::vector<f2c::types::Point> path_points;      // 展平后的路径点列表
    std::vector<f2c::types::Point> path_waypoints;   // 仅 TURN/SWATH 段端点

    // ── 中间产物（调试/可视化的只读快照）──
    f2c::types::SwathsByCells cells_with_swaths;     // 排序后的 cell → swaths 映射
    f2c::types::Route route;                         // 最终 Route
    std::vector<size_t> cell_order;                  // 遍历顺序 → 原始索引
    size_t total_swaths = 0;
    size_t total_connections = 0;

    // ── 诊断信息 ──
    double planning_time_ms = 0.0;
    size_t hole_crossing_segments = 0;               // 穿洞段数（0 = 安全）
    size_t out_of_planning_area_segments = 0;
    bool path_has_crossings = false;
    bool path_leaves_planning_area = false;
    FootprintCollisionResult physical_collision;

    // ── C-space 诊断与分量结果 ──
    TraversabilityResult traversability;
    std::vector<PlanningComponentResult> component_plans;

    // ── 状态 ──
    bool success = false;
    std::string error_message;
};

// ========== 规划核心门面 ==========
//
// 当前状态（2026-07-18）：与 legacy 的 notched 基线对齐并作为默认管线。
//
// 已实现：
//   - headland 生成（ConstHL + mid_hl / no_hl 双层侵蚀）
//   - rectilinear 分解 + no_hl secondary erosion
//   - swath 生成 + fillBoundaryGaps + filterShortSwaths
//   - swath_angle_optimization（多角度候选，选 swath 数最少）
//   - tiny-cell 过滤 + 相邻同向 Cell 合并（与 legacy 共用实现）
//   - pruneRedundantCellSeamFills
//   - greedyCellOrder（v9.12 极角/四变体 + 无洞联合贪心）
//   - genRoute（boustrophedon/spiral 等模式）+ Snake 直连特例
//   - repairRouteConnectionsAroundHoles + synchronizeRouteConnectionEndpoints
//   - turn_planner_type：dubins / dubins_cc / reeds_shepp / direct
//   - RDP 路径简化 + materializePath
//   - 孔洞作为 polygon 内环传入（buildPlanningRequest 含 addRing）
//   - use_planner_core_:=true 切换至 planWithCore()（含评估输出）
//
// 2026-07-18 notched 双管线验证：coverage 均 99.99%，Core 81.8，
// legacy 81.7；路径长度、转弯数和重叠率持平。
class PlannerCore {
public:
    PlannerCore() = default;
    ~PlannerCore() = default;

    // 不可复制，可移动。
    PlannerCore(const PlannerCore&) = delete;
    PlannerCore& operator=(const PlannerCore&) = delete;
    PlannerCore(PlannerCore&&) = default;
    PlannerCore& operator=(PlannerCore&&) = default;

    // 执行全覆盖路径规划。
    // 请求中的配置在本次调用期间视为不可变快照；
    // 多次调用之间互相独立，无共享可变状态。
    PlanningResult plan(const PlanningRequest& request);
};

}  // namespace yingshi
