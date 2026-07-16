#pragma once
#include <string>

namespace yingshi {

// ========== 分解参数 ==========
struct DecomposerParams {
    bool   use_sweep             = true;   // 扫描线分解（孔洞顶点切分）
    double merge_angle_threshold = 60.0;   // cell 合并角度阈值（度）
    bool   filter_tiny           = true;   // 过滤微小子区域
    double min_cell_area_ratio   = 2.0;    // 最小面积 = ratio × cov_width × robot_width
};

// ========== Swath 生成参数 ==========
struct SwathParams {
    double coverage_width         = 0.45;
    double swath_overlap_ratio    = 0.03;  // 0~1，3% 重叠防漏缝
    bool   angle_optimization     = true;  // 多角度选择
    double min_swath_length       = 0.5;   // 短于此时丢弃
    double endpoint_shrink        = 0.03;  // 闭合边界向内收缩
    std::string angle_candidates;          // 补充候选角度，留空=自动提取
    bool   decomposition_angle_opt = false;
};

// ========== 边界补刀参数 ==========
struct FillParams {
    double coverage_width  = 0.45;
    std::string boundary_type = "closed";  // "closed"/"open"/"custom"
    double boundary_margin = -0.3;         // 正=收缩，负=延伸
    double open_default_margin = -0.3;     // open 模式默认延伸量
};

// ========== 路径规划参数 ==========
struct PathParams {
    std::string turn_planner_type = "direct";   // "direct"/"dubins"/"reeds_shepp"
    std::string swath_order_type  = "boustrophedon";
    bool   simplify_enabled       = true;
    double simplify_tolerance     = 0.05;        // RDP 容差 (m)
    double simplify_turn_threshold = 0.15;       // 转弯角度阈值 (rad)
    bool   ortools_exact_solve    = false;       // OR-Tools 详细日志
    double max_diff_curv          = 0.3;
    double min_turning_radius     = 0.01;
    double robot_width            = 0.95;
    double coverage_width         = 0.45;
    double path_resolution        = 0.1;
    double swath_overlap_ratio    = 0.03;
    // 边界策略（传递给 swath 端点调整）
    std::string boundary_type     = "closed";
    double endpoint_shrink        = 0.03;
    double boundary_margin        = -0.3;
};

// ========== 通用几何工具常量 ==========
namespace GeomConst {
    constexpr double kHalfWidthRatio   = 0.5;     // half_w = cov_width * 0.5
    constexpr double kAngleTolCos      = 0.9397;  // cos(20°)，平行边判断
    constexpr double kSwathAlignCos    = 0.8660;  // cos(30°)，段方向对齐
    constexpr double kDefaultBboxDist  = 1.5;     // bbox 距离系数（× cov_width）
    constexpr double kMinSegLength     = 0.01;    // 退化边阈值 (m)
    constexpr double kSpanThreshold    = 0.5;     // v7 覆盖判定：50% 跨度
    constexpr double kRevDistRatio     = 0.7;     // Cell反转至少缩短30%
    constexpr double kHoleSepSentinel  = 1e10;    // ROS2 多孔洞分隔符
}

}  // namespace yingshi
