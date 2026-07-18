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
    double max_diff_curv = 0.1;

    // ── Headland 参数 ──
    double mid_hl_width_ratio = 0.20;
    double no_hl_width_ratio = 0.00;

    // ── Swath 参数 ──
    double swath_overlap_ratio = 0.03;
    double swath_endpoint_shrink_distance = 0.03;
    double min_swath_length = 0.2;

    // ── 分解参数 ──
    bool decomposition_enabled = true;
    bool use_sweep_decomp = false;
    bool decomposition_angle_optimization = false;
    double merge_angle_threshold = 45.0;

    // ── Swath 优化 ──
    bool swath_angle_optimization = false;
    std::vector<double> swath_angle_candidates;

    // ── 排序参数 ──
    std::string swath_order_type = "boustrophedon";   // boustrophedon | snake | spiral | none
    std::string turn_planner_type = "direct";          // direct | dubins | reeds_shepp

    // ── 路径后处理 ──
    bool path_simplify_enabled = false;
    double path_simplify_tolerance = 0.05;
    double path_simplify_turn_threshold = 0.15;

    // ── 过滤 ──
    bool filter_tiny_cells = false;
    double min_cell_area_ratio = 2.0;
    double min_hole_area = 0.1;

    // ── 边界策略 ──
    std::string boundary_type = "closed";
    double boundary_coverage_margin = -0.3;
    double boundary_open_default_margin = -0.3;
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
    bool path_has_crossings = false;

    // ── 状态 ──
    bool success = false;
    std::string error_message;
};

// ========== 规划核心门面 ==========
//
// 当前状态（2026-07-17）：接口完整，核心功能已实现，ROS 节点已接入。
//
// 已实现：
//   - headland 生成（ConstHL + mid_hl / no_hl 双层侵蚀）
//   - rectilinear 分解 + no_hl secondary erosion
//   - swath 生成 + fillBoundaryGaps + filterShortSwaths
//   - swath_angle_optimization（多角度候选，选 swath 数最少）
//   - tiny-cell 过滤 + 相邻同向 Cell 合并（与 legacy 共用实现）
//   - pruneRedundantCellSeamFills
//   - greedyCellOrder / Snake 路由
//   - genRoute（Boustrophedon TSP）+ Snake 直连
//   - repairRouteConnectionsAroundHoles + synchronizeRouteConnectionEndpoints
//   - turn_planner_type：dubins / dubins_cc / reeds_shepp / direct
//   - RDP 路径简化 + materializePath
//   - 孔洞作为 polygon 内环传入（buildPlanningRequest 含 addRing）
//   - use_planner_core_:=true 切换至 planWithCore()（含评估输出）
//
// 待补齐（按优先级）：
//   1. 全场景回归确认与 legacy 对齐（2026-07-18 修复前 notched: 77.8 vs 81.7）
//   2. sweep_align_angle + 自适应 headland ratio
//   3. PathSanityCheck 集成到 publish seam 前
//
// use_planner_core_ 默认 false；结果对齐并通过独立集成测试后切为 true。
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
