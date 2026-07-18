#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"  // 添加PoseArray消息类型
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "fields2cover.h"  // Fields2Cover头文件
#if YINGSHI_EVAL_ENABLED
#include "coverage_evaluator.hpp"  // 评估模块 — 提供覆盖率/效率/曲率量化评分
#endif

// ── 模块化重构：独立算法模块 ──
#include "yingshi_robot/planner_params.hpp"
#include "yingshi_robot/decomposer.hpp"
#include "yingshi_robot/swath_generator.hpp"
#include "yingshi_robot/boundary_filler.hpp"
#include "yingshi_robot/path_planner.hpp"
#include "yingshi_robot/path_geometry.hpp"
#include "yingshi_robot/path_sanity_check.hpp"
#include "yingshi_robot/planner_core.hpp"
#include "yingshi_robot/artifact_sink.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>
#include <chrono>
#include <functional>

namespace {

f2c::types::LinearRing makeClosedF2CRing(
    const geometry_msgs::msg::Polygon& polygon)
{
    std::vector<f2c::types::Point> points;
    points.reserve(polygon.points.size());
    for (const auto& point : polygon.points) {
        points.emplace_back(point.x, point.y);
    }
    return yingshi::makeClosedRing(points);
}

}  // namespace

class PolygonPlannerNode : public rclcpp::Node
{
private:
    // 添加成员变量存储参数
    double robot_width_;
    double min_turning_radius_;
    double max_diff_curv_;
    double coverage_width_;
    double path_resolution_;  // 添加路径分辨率参数
    double decomposition_angle_;  // Boustrophedon 分解角度
    double mid_hl_width_ratio_;  // mid_hl 宽度比例（相对于覆盖宽度）
    double no_hl_width_ratio_;   // no_hl 宽度比例（相对于覆盖宽度）
    double min_hole_area_;       // 最小 holes 面积阈值（平方米），小于此面积的 holes 将被剔除
    double swath_endpoint_shrink_distance_;  // 条带端点向中心收缩的距离（米），用于留出机器人转向空间

    double min_swath_length_;

    // ── 评估参数 ──
    bool eval_enable_report_;
    double eval_grid_resolution_;
    bool eval_use_grid_method_;
    double eval_coverage_threshold_;
    std::string output_dir_;
    bool use_planner_core_;            // 是否使用 PlannerCore（P2 迁移开关）

    // ── 优化开关参数 ──
    bool use_optimized_planner_;       // ★ 总开关：true=优化版 false=原版
    bool swath_angle_optimization_;    // Swath 多角度选择（使用边缘方向剪枝）
    std::string swath_angle_candidates_; // 补充候选角度，留空则自动从多边形边缘提取
    std::vector<double> swath_angle_list_; // 解析后的角度列表
    bool filter_tiny_cells_;           // 过滤微小子区域
    double min_cell_area_ratio_;       // 最小面积 = ratio × cov_width × robot_width
    bool path_simplify_enabled_;       // RDP 路径简化
    double path_simplify_tolerance_;   // RDP 容差 (m)
    double path_simplify_turn_threshold_; // RDP 转弯检测角度阈值 (rad)
    double swath_overlap_ratio_;       // Swath 重叠率 (0~1)，0=刚好接上，0.03=3%重叠防漏缝
    bool ortools_exact_solve_;         // OR-Tools 求解器详细日志（genRoute show_log=true）
    bool decomposition_angle_optimization_; // 分解角度多角度选择
    bool decomposition_enabled_;       // 是否启用 Boustrophedon 分解（关=W1不分解）
    bool use_sweep_decomp_;           // ★ 扫描线分解：仅水平切分，孔洞→条带(7cells)而非网格(42cells)
    double merge_angle_threshold_;    // cell 合并角度阈值（度），默认45°
    std::string swath_order_type_;    // swath 排序算法：boustrophedon | snake | spiral | none
    std::string turn_planner_type_;   // "direct"/"dubins"/"dubins_cc"/"reeds_shepp"

    // ── 边界覆盖策略（统一闭合/开放边界） ──
    std::string boundary_type_;        // "closed"/"open"/"custom" — 边界语义预设
    double boundary_coverage_margin_;  // Swath 端点调整量：正=收缩(闭合), 负=延伸(开放), 0=原位
    double boundary_open_default_margin_; // open 边界默认延伸量 (m)，boundary_coverage_margin=0 时使用
public:
    PolygonPlannerNode()
    : Node("polygon_planner_node")
    {
        // 声明参数，设置默认值
        this->declare_parameter("robot_width", 1.0);
        this->declare_parameter("min_turning_radius", 0.01);  // 差速驱动近似原地转
        this->declare_parameter("max_diff_curv", 0.1);
        this->declare_parameter("coverage_width", 1.0);
        this->declare_parameter("path_resolution", 0.1);  // 添加路径分辨率参数
        this->declare_parameter("decomposition_angle", 0.0);  // Boustrophedon 分解角度（弧度），默认0度（沿X轴方向）
        this->declare_parameter("mid_hl_width_ratio", 0.1);  // mid_hl 宽度比例（相对于覆盖宽度）
        this->declare_parameter("no_hl_width_ratio", 0.5);   // no_hl 宽度比例（相对于覆盖宽度）
        this->declare_parameter("min_hole_area", 0.1);       // 最小 holes 面积阈值（平方米），小于此面积的 holes 将被剔除
        this->declare_parameter("swath_endpoint_shrink_distance", 0.03);  // 条带端点向中心收缩的距离（米），batch 实测最优值

        // 获取参数值并存储
        this->declare_parameter("min_swath_length", 0.2);

        // ── 评估参数 ──
        this->declare_parameter("eval_enable_report", true);
        this->declare_parameter("eval_grid_resolution", 0.1);
        this->declare_parameter("eval_use_grid_method", false);
        this->declare_parameter("eval_coverage_threshold", 0.99);
        this->declare_parameter(
            "output_dir", yingshi::defaultArtifactDirectory());
        this->declare_parameter("use_planner_core", false);  // P2 迁移开关：true=PlannerCore管线

        // ── 优化参数 ──
        this->declare_parameter("use_optimized_planner", false);
        this->declare_parameter("swath_angle_optimization", false);
        this->declare_parameter("swath_angle_candidates", "");  // 留空=自动提取多边形边缘角度
        this->declare_parameter("filter_tiny_cells", false);
        this->declare_parameter("min_cell_area_ratio", 2.0);
        this->declare_parameter("path_simplify_enabled", false);
        this->declare_parameter("path_simplify_tolerance", 0.05);
        this->declare_parameter("path_simplify_turn_threshold", 0.15);  // 转弯检测角度阈值 (rad)，~8.6°
        this->declare_parameter("swath_overlap_ratio", 0.0);  // Swath 重叠率 (0~1)，0.03=3%重叠
        this->declare_parameter("ortools_exact_solve", false);  // 启用 OR-Tools 求解器详细日志输出
        this->declare_parameter("decomposition_angle_optimization", false);
        this->declare_parameter("decomposition_enabled", true);  // Boustrophedon 分解开关
        this->declare_parameter("use_sweep_decomp", false);     // ★ 扫描线分解：仅水平切孔洞顶点→全宽条带
        this->declare_parameter("merge_angle_threshold", 45.0); // cell 合并角度阈值（度），无sweep默认45°
        this->declare_parameter("swath_order_type", "none");     // swath 排序: boustrophedon | snake | spiral | none
        this->declare_parameter("turn_planner_type", "direct");  // "direct"/"dubins"/"reeds_shepp"，差速驱动默认零半径欧氏连接

        // ── 边界覆盖策略参数 ──
        // boundary_type: "closed"=闭合硬边界（swath端点内缩留安全距离）
        //                "open"=开放软边界（swath端点外伸换覆盖率）
        //                "custom"=手动指定 boundary_coverage_margin
        this->declare_parameter("boundary_type", "closed");
        // boundary_coverage_margin: swath端点调整量（米）
        //   正值 = 向外延伸（开放边界，牺牲重叠换覆盖率）
        //   负值 = 向内收缩（闭合边界，留安全距离防撞墙）
        //   零值 = 不做调整（swath端点即为cell边界）
        this->declare_parameter("boundary_coverage_margin", -0.3);
        this->declare_parameter("boundary_open_default_margin", -0.3);  // open 边界默认延伸量 (m)，负值=向外延伸

        updateParameters();

        // 添加参数回调
        auto param_callback = 
            [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult {
                auto result = rcl_interfaces::msg::SetParametersResult();
                result.successful = true;
                
                for (const auto &param : parameters) {
                    RCLCPP_INFO(this->get_logger(), 
                        "Parameter '%s' changed to: %s", 
                        param.get_name().c_str(), 
                        param.value_to_string().c_str());
                }
                
                updateParameters(&parameters);
                return result;
            };

        param_callback_handle_ = this->add_on_set_parameters_callback(param_callback);

        // 打印初始参数值
        RCLCPP_INFO(this->get_logger(), "Initial parameters:");
        RCLCPP_INFO(this->get_logger(), "robot_width: %.2f", robot_width_);
        RCLCPP_INFO(this->get_logger(), "min_turning_radius: %.2f", min_turning_radius_);
        RCLCPP_INFO(this->get_logger(), "max_diff_curv: %.2f", max_diff_curv_);
        RCLCPP_INFO(this->get_logger(), "coverage_width: %.2f", coverage_width_);
        RCLCPP_INFO(this->get_logger(), "path_resolution: %.2f", path_resolution_);
        RCLCPP_INFO(this->get_logger(), "decomposition_angle: %.2f deg (%.2f rad)", 
                    decomposition_angle_ * 180.0 / M_PI, decomposition_angle_);
        RCLCPP_INFO(this->get_logger(), "mid_hl_width_ratio: %.2f", mid_hl_width_ratio_);
        RCLCPP_INFO(this->get_logger(), "no_hl_width_ratio: %.2f", no_hl_width_ratio_);
        RCLCPP_INFO(this->get_logger(), "min_hole_area: %.3f m²", min_hole_area_);
        RCLCPP_INFO(this->get_logger(), "swath_endpoint_shrink_distance: %.3f m", swath_endpoint_shrink_distance_);
        RCLCPP_INFO(this->get_logger(), "min_swath_length: %.3f m", min_swath_length_);

        // 打印评估和优化参数
        RCLCPP_INFO(this->get_logger(), "--- Evaluation & Optimization ---");
        RCLCPP_INFO(this->get_logger(), "eval_enable_report: %s", eval_enable_report_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "eval_grid_resolution: %.2f m", eval_grid_resolution_);
        RCLCPP_INFO(this->get_logger(), "eval_use_grid_method: %s", eval_use_grid_method_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "use_optimized_planner: %s", use_optimized_planner_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "swath_angle_optimization: %s", swath_angle_optimization_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "swath_angle_candidates: %s", swath_angle_candidates_.c_str());
        RCLCPP_INFO(this->get_logger(), "filter_tiny_cells: %s", filter_tiny_cells_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "path_simplify_enabled: %s (tolerance=%.2f m, turn_threshold=%.3f rad)",
                    path_simplify_enabled_ ? "true" : "false", path_simplify_tolerance_,
                    path_simplify_turn_threshold_);
        RCLCPP_INFO(this->get_logger(), "swath_overlap_ratio: %.1f%%",
                    swath_overlap_ratio_ * 100.0);
        RCLCPP_INFO(this->get_logger(), "ortools_exact_solve (verbose log): %s", ortools_exact_solve_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "decomposition_angle_optimization: %s", decomposition_angle_optimization_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "decomposition_enabled: %s", decomposition_enabled_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "use_sweep_decomp: %s", use_sweep_decomp_ ? "true (horizontal strips)" : "false (grid)");
        RCLCPP_INFO(this->get_logger(), "turn_planner_type: %s", turn_planner_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "--- Boundary Coverage Strategy ---");
        RCLCPP_INFO(this->get_logger(), "boundary_type: %s", boundary_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "boundary_coverage_margin: %.3f m (%s)",
                    boundary_coverage_margin_,
                    boundary_coverage_margin_ > 0.0 ? "extend outward (open)" :
                    boundary_coverage_margin_ < 0.0 ? "shrink inward (closed)" : "no adjustment");
        RCLCPP_INFO(this->get_logger(), "boundary_open_default_margin: %.3f m", boundary_open_default_margin_);
        RCLCPP_INFO(this->get_logger(), "---------------------------------");

        // 创建多个多边形输入订阅器（最多4个）
        for (int i = 1; i <= 4; ++i) {
            auto polygon_sub = this->create_subscription<geometry_msgs::msg::Polygon>(
                "input_polygon_" + std::to_string(i), 10,
                [this, i](const geometry_msgs::msg::Polygon::SharedPtr msg) {
                    this->polygonCallback(msg, i);
                }
            );
            polygon_subs_.push_back(polygon_sub);
            
            // 创建holes订阅器
            auto holes_sub = this->create_subscription<geometry_msgs::msg::Polygon>(
                "input_polygon_" + std::to_string(i) + "_holes", 10,
                [this, i](const geometry_msgs::msg::Polygon::SharedPtr msg) {
                    this->holesCallback(msg, i);
                }
            );
            holes_subs_.push_back(holes_sub);
        }
        
        // 创建多个输出发布器（每个多边形对应一组）
        for (int i = 1; i <= 4; ++i) {
            // 路径发布器
            auto path_pub = this->create_publisher<nav_msgs::msg::Path>(
                "planned2_path_" + std::to_string(i),
                rclcpp::QoS(10).transient_local()
            );
            path_pubs_.push_back(path_pub);

            // 航向线端点发布器
            auto swath_points_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
                "swath_points_" + std::to_string(i),
                rclcpp::QoS(10).transient_local()
            );
            swath_points_pubs_.push_back(swath_points_pub);

            // 采样路径点发布器
            auto sampled_path_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
                "sampled_path_points_" + std::to_string(i),
                rclcpp::QoS(10).transient_local()
            );
            sampled_path_pubs_.push_back(sampled_path_pub);

            // 多边形轮廓发布器
            auto polygon_viz_pub = this->create_publisher<nav_msgs::msg::Path>(
                "polygon_outline_" + std::to_string(i),
                rclcpp::QoS(10).transient_local()
            );
            polygon_viz_pubs_.push_back(polygon_viz_pub);

            // Holes轮廓发布器
            auto holes_outline_pub = this->create_publisher<nav_msgs::msg::Path>(
                "holes_outline_" + std::to_string(i),
                rclcpp::QoS(10).transient_local()
            );
            holes_outline_pubs_.push_back(holes_outline_pub);

            // 区域分解轮廓发布器
            auto decomposition_outline_pub = this->create_publisher<nav_msgs::msg::Path>(
                "decomposition_outline_" + std::to_string(i),
                rclcpp::QoS(10).transient_local()
            );
            decomposition_outline_pubs_.push_back(decomposition_outline_pub);

            // 不可执行掉头段标记发布器
            auto infeasible_turn_pub = this->create_publisher<visualization_msgs::msg::Marker>(
                "infeasible_turns_" + std::to_string(i),
                rclcpp::QoS(10).transient_local()
            );
            infeasible_turn_pubs_.push_back(infeasible_turn_pub);
        }

        // 添加定时器，每秒重新发布一次路径和多边形
        timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&PolygonPlannerNode::publishLastPath, this));

        clear_cache_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/clear_coverage_planner_cache",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                clearAllPlanningCaches(true);
                response->success = true;
                response->message = "Coverage planner cache cleared";
                RCLCPP_INFO(this->get_logger(), "Coverage planner cache cleared by service");
            });

        // 初始化数组
        polygon_received_.fill(false);
        holes_received_.fill(false);
        
        RCLCPP_INFO(this->get_logger(), "PolygonPlannerNode started with support for 4 polygons");
    }

private:
    // 存储多个多边形的数据（使用数组索引0-3对应多边形1-4）
    std::array<nav_msgs::msg::Path, 4> last_paths_;           // 存储4个多边形的路径
    std::array<geometry_msgs::msg::PoseArray, 4> last_swath_points_;  // 存储4个多边形的航向线端点
    std::array<geometry_msgs::msg::PoseArray, 4> last_sampled_points_;  // 存储4个多边形的采样点
    std::array<geometry_msgs::msg::Polygon, 4> last_polygons_;  // 存储4个多边形
    std::array<std::vector<geometry_msgs::msg::Polygon>, 4> last_holes_;  // 存储4个多边形的holes列表
    std::array<nav_msgs::msg::Path, 4> last_holes_outlines_;  // 存储4个多边形的holes轮廓路径
    std::array<nav_msgs::msg::Path, 4> last_decomposition_outlines_;  // 存储4个多边形的区域分解轮廓路径
    std::array<visualization_msgs::msg::Marker, 4> last_infeasible_turn_markers_; // 不可执行掉头标记
    std::array<bool, 4> polygon_received_;  // 标记哪些多边形已接收
    std::array<bool, 4> holes_received_;  // 标记holes消息是否已接收（即使为空，也表示已明确知道没有holes）
    size_t vis_json_cell_count_ = 0;  // Vis JSON 多 cell 追加计数器（per polygon处理）

    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_cache_service_;
    
    // 多个发布器和订阅器
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr> polygon_subs_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr> holes_subs_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> swath_points_pubs_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> sampled_path_pubs_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> polygon_viz_pubs_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> holes_outline_pubs_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> decomposition_outline_pubs_;
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> infeasible_turn_pubs_;

    void updateParameters(const std::vector<rclcpp::Parameter>* new_params = nullptr)
    {
        // 辅助 lambda：从回调参数中查找最新值，找不到则用 get_parameter（初始加载路径）
        auto get_double = [&](const std::string& name) -> double {
            if (new_params) for (auto& p : *new_params) if (p.get_name() == name) return p.as_double();
            return this->get_parameter(name).as_double();
        };
        auto get_bool = [&](const std::string& name) -> bool {
            if (new_params) for (auto& p : *new_params) if (p.get_name() == name) return p.as_bool();
            return this->get_parameter(name).as_bool();
        };
        auto get_string = [&](const std::string& name) -> std::string {
            if (new_params) for (auto& p : *new_params) if (p.get_name() == name) return p.as_string();
            return this->get_parameter(name).as_string();
        };
        // 更新参数值（优先用回调中的新值，避免 ROS2 参数回调时序问题）
        robot_width_ = get_double("robot_width");
        min_turning_radius_ = get_double("min_turning_radius");
        max_diff_curv_ = get_double("max_diff_curv");
        coverage_width_ = get_double("coverage_width");
        path_resolution_ = get_double("path_resolution");
        decomposition_angle_ = get_double("decomposition_angle");
        mid_hl_width_ratio_ = get_double("mid_hl_width_ratio");
        no_hl_width_ratio_ = get_double("no_hl_width_ratio");
        min_hole_area_ = get_double("min_hole_area");
        swath_endpoint_shrink_distance_ = get_double("swath_endpoint_shrink_distance");
        min_swath_length_ = get_double("min_swath_length");

        // ── 评估参数 ──
        eval_enable_report_ = get_bool("eval_enable_report");
        eval_grid_resolution_ = get_double("eval_grid_resolution");
        eval_use_grid_method_ = get_bool("eval_use_grid_method");
        eval_coverage_threshold_ = get_double("eval_coverage_threshold");
        output_dir_ = get_string("output_dir");
        use_planner_core_ = get_bool("use_planner_core");

        // ── 优化参数 ──
        use_optimized_planner_ = get_bool("use_optimized_planner");
        swath_angle_optimization_ = get_bool("swath_angle_optimization");
        swath_angle_candidates_ = get_string("swath_angle_candidates");
        filter_tiny_cells_ = get_bool("filter_tiny_cells");
        min_cell_area_ratio_ = get_double("min_cell_area_ratio");
        path_simplify_enabled_ = get_bool("path_simplify_enabled");
        path_simplify_tolerance_ = get_double("path_simplify_tolerance");
        path_simplify_turn_threshold_ = get_double("path_simplify_turn_threshold");
        swath_overlap_ratio_ = get_double("swath_overlap_ratio");
        ortools_exact_solve_ = get_bool("ortools_exact_solve");
        decomposition_angle_optimization_ = get_bool("decomposition_angle_optimization");
        decomposition_enabled_ = get_bool("decomposition_enabled");
        use_sweep_decomp_ = get_bool("use_sweep_decomp");
        merge_angle_threshold_ = get_double("merge_angle_threshold");
        swath_order_type_ = get_string("swath_order_type");
        turn_planner_type_ = get_string("turn_planner_type");

        // ── 边界覆盖策略 ──
        boundary_type_ = get_string("boundary_type");
        boundary_coverage_margin_ = get_double("boundary_coverage_margin");
        boundary_open_default_margin_ = get_double("boundary_open_default_margin");

        // 解析角度候选列表
        swath_angle_list_.clear();
        std::istringstream angle_stream(swath_angle_candidates_);
        std::string token;
        while (std::getline(angle_stream, token, ',')) {
            try {
                double deg = std::stod(token);
                swath_angle_list_.push_back(deg * M_PI / 180.0);  // 转为弧度
            } catch (...) {
                RCLCPP_WARN(this->get_logger(),
                           "Invalid swath angle candidate: '%s', skipping", token.c_str());
            }
        }

        // 参数合法性校验 — 拒绝零/负分辨率、非法枚举值、越界比例
        if (eval_grid_resolution_ <= 0.0) eval_grid_resolution_ = 0.1;
        if (coverage_width_ <= 0.0) coverage_width_ = 1.0;
        if (robot_width_ <= 0.0) robot_width_ = 1.0;
        if (swath_overlap_ratio_ < 0.0) swath_overlap_ratio_ = 0.0;
        if (swath_overlap_ratio_ > 0.5) swath_overlap_ratio_ = 0.5;
        if (merge_angle_threshold_ < 0.0) merge_angle_threshold_ = 45.0;
        if (merge_angle_threshold_ > 90.0) merge_angle_threshold_ = 90.0;
        const std::vector<std::string> valid_boundary = {"closed","open","custom"};
        if (std::find(valid_boundary.begin(), valid_boundary.end(), boundary_type_) == valid_boundary.end())
            boundary_type_ = "closed";
        const std::vector<std::string> valid_order = {"boustrophedon","snake","spiral","none"};
        if (std::find(valid_order.begin(), valid_order.end(), swath_order_type_) == valid_order.end())
            swath_order_type_ = "boustrophedon";
        const std::vector<std::string> valid_turn = {"direct","dubins","dubins_cc","reeds_shepp"};
        if (std::find(valid_turn.begin(), valid_turn.end(), turn_planner_type_) == valid_turn.end())
            turn_planner_type_ = "direct";
    }

    void publishEmptyPlanningOutputs(int index)
    {
        if (index < 0 || index >= 4) {
            return;
        }

        nav_msgs::msg::Path empty_path;
        empty_path.header.frame_id = "map";
        empty_path.header.stamp = this->now();

        geometry_msgs::msg::PoseArray empty_pose_array;
        empty_pose_array.header.frame_id = "map";
        empty_pose_array.header.stamp = this->now();

        path_pubs_[index]->publish(empty_path);
        swath_points_pubs_[index]->publish(empty_pose_array);
        sampled_path_pubs_[index]->publish(empty_pose_array);
        polygon_viz_pubs_[index]->publish(empty_path);
        holes_outline_pubs_[index]->publish(empty_path);
        decomposition_outline_pubs_[index]->publish(empty_path);
    }

    void clearPlanningCacheForPolygon(int index, bool publish_empty_outputs)
    {
        if (index < 0 || index >= 4) {
            return;
        }

        last_paths_[index] = nav_msgs::msg::Path();
        last_swath_points_[index] = geometry_msgs::msg::PoseArray();
        last_sampled_points_[index] = geometry_msgs::msg::PoseArray();
        last_polygons_[index] = geometry_msgs::msg::Polygon();
        last_holes_[index].clear();
        last_holes_outlines_[index] = nav_msgs::msg::Path();
        last_decomposition_outlines_[index] = nav_msgs::msg::Path();
        last_infeasible_turn_markers_[index] = visualization_msgs::msg::Marker();
        polygon_received_[index] = false;
        holes_received_[index] = false;

        if (publish_empty_outputs) {
            publishEmptyPlanningOutputs(index);
        }

        // 清除 RViz 中的旧 marker（掉头标记等）
        {
            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = "map";
            clear.header.stamp = this->now();
            clear.ns = "infeasible_turns";
            clear.id = 0;
            clear.action = visualization_msgs::msg::Marker::DELETE;
            infeasible_turn_pubs_[index]->publish(clear);
        }

        RCLCPP_INFO(this->get_logger(),
                    "Cleared cached planning outputs for polygon_%d", index + 1);
    }

    void clearAllPlanningCaches(bool publish_empty_outputs)
    {
        for (int i = 0; i < 4; ++i) {
            clearPlanningCacheForPolygon(i, publish_empty_outputs);
        }
    }
    
    // Calculate one generated swath length. (→ yingshi::swathLength)
    double swathLength(const f2c::types::Swath& swath) const
    {
        return yingshi::swathLength(swath);
    }

    f2c::types::Swaths filterShortSwaths(
        const f2c::types::Swaths& swaths,
        double min_length,
        size_t& removed_count) const
    {
        removed_count = 0;
        if (min_length <= 0.0) {
            return swaths;
        }

        f2c::types::Swaths filtered_swaths;
        for (size_t i = 0; i < swaths.size(); ++i) {
            const auto& swath = swaths.at(i);
            const double length = swathLength(swath);
            if (length < min_length) {
                ++removed_count;
                RCLCPP_INFO(this->get_logger(),
                            "Filtered short swath %zu: length=%.3f m < %.3f m",
                            i, length, min_length);
                continue;
            }
            filtered_swaths.push_back(swath);
        }

        return filtered_swaths;
    }

    // Calculate the polygon main direction from its longest edge.
    double computePolygonMainDirection(const geometry_msgs::msg::Polygon& polygon) const
    {
        if (polygon.points.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Polygon has less than 2 points, using default angle 0");
            return 0.0;
        }
        
        double max_length = 0.0;
        double main_angle = 0.0;
        
        // 遍历多边形的所有边
        for (size_t i = 0; i < polygon.points.size(); ++i) {
            size_t next_i = (i + 1) % polygon.points.size();
            
            const auto& p1 = polygon.points[i];
            const auto& p2 = polygon.points[next_i];
            
            // 计算边的长度
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double length = std::sqrt(dx * dx + dy * dy);
            
            // 如果这是目前最长的边，记录其方向角
            if (length > max_length) {
                max_length = length;
                // 计算方向角（atan2返回-π到π之间的角度）
                main_angle = std::atan2(dy, dx);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Polygon main direction: longest edge length=%.2f, angle=%.2f deg (%.2f rad)", 
                   max_length, main_angle * 180.0 / M_PI, main_angle);
        
        return main_angle;
    }

    // 双向 Swath 端点调整（统一收敛/延伸）
    // distance > 0：端点向中心收缩（闭合边界安全模式，留转向空间）
    // distance < 0：端点向外延伸（开放边界覆盖模式，牺牲重叠换覆盖率）
    // distance = 0：不调整，保持原样
    f2c::types::Swath adjustSwathEndpoints(const f2c::types::Swath& swath, double distance) const
    {
        if (distance == 0.0) {
            return swath;  // 不做调整
        }

        // 获取原始起点和终点
        f2c::types::Point start_point = swath.startPoint();
        f2c::types::Point end_point = swath.endPoint();

        // 计算 swath 的方向向量和长度
        double dx = end_point.getX() - start_point.getX();
        double dy = end_point.getY() - start_point.getY();
        double length = std::sqrt(dx * dx + dy * dy);

        // 如果 swath 太短，不做调整（避免端点翻转）
        double abs_distance = std::abs(distance);
        if (length <= 2.0 * abs_distance) {
            RCLCPP_WARN(this->get_logger(),
                       "Swath too short (%.3f m) to adjust by %.3f m, keeping original",
                       length, distance);
            return swath;
        }

        // 归一化方向向量
        double unit_dx = dx / length;
        double unit_dy = dy / length;

        // 双向调整逻辑：
        // - distance > 0（收缩）：start 向 end 移动，end 向 start 移动
        // - distance < 0（延伸）：start 背向 end 移动，end 背向 start 移动
        f2c::types::Point new_start(
            start_point.getX() + distance * unit_dx,
            start_point.getY() + distance * unit_dy
        );
        f2c::types::Point new_end(
            end_point.getX() - distance * unit_dx,
            end_point.getY() - distance * unit_dy
        );

        // 创建新的 LineString（从新的起点到新的终点）
        f2c::types::LineString new_path(new_start, new_end);

        // 创建新的 swath（保持原有的宽度、ID 和类型）
        f2c::types::Swath adjusted_swath(new_path, swath.getWidth(), swath.getId(), swath.getType());

        return adjusted_swath;
    }
    
    // 对 SwathsByCells 中的所有 swaths 应用端点调整
    f2c::types::SwathsByCells adjustSwathsEndpoints(
        const f2c::types::SwathsByCells& swaths_by_cells,
        double distance) const
    {
        if (distance == 0.0) {
            return swaths_by_cells;  // 不做调整，直接返回
        }

        f2c::types::SwathsByCells adjusted_swaths_by_cells;

        // 遍历每个单元格的 swaths
        for (size_t cell_idx = 0; cell_idx < swaths_by_cells.size(); ++cell_idx) {
            const auto& cell_swaths = swaths_by_cells.at(cell_idx);
            f2c::types::Swaths adjusted_cell_swaths;

            // 对每个 swath 进行端点调整
            for (size_t swath_idx = 0; swath_idx < cell_swaths.size(); ++swath_idx) {
                const auto& swath = cell_swaths.at(swath_idx);
                f2c::types::Swath adjusted_swath = adjustSwathEndpoints(swath, distance);
                adjusted_cell_swaths.push_back(adjusted_swath);
            }

            adjusted_swaths_by_cells.push_back(adjusted_cell_swaths);
        }

        return adjusted_swaths_by_cells;
    }

    // OGR buffer 操作会在直线段上插入多余顶点，移除环上共线冗余顶点 (→ yingshi::simplifyRing)
    f2c::types::LinearRing simplifyRing(const f2c::types::LinearRing& ring,
                                         double angle_tol_deg = 0.5) const
    {
        return yingshi::simplifyRing(ring, angle_tol_deg);
    }

    f2c::types::Cells simplifyCells(const f2c::types::Cells& cells,
                                     double angle_tol_deg = 0.5,
                                     double hole_angle_tol_deg = 0.5) const
    {
        // 外环和内环可独立控制容差（孔洞用更保守的阈值避免变形）
        f2c::types::Cells out;
        for (size_t ci = 0; ci < cells.size(); ++ci) {
            f2c::types::Cell c = cells.getGeometry(ci);
            f2c::types::Cell clean;
            clean.addRing(yingshi::simplifyRing(c.getExteriorRing(), angle_tol_deg));
            for (size_t ri = 0; ri < c.size() - 1; ++ri) {
                clean.addRing(yingshi::simplifyRing(c.getInteriorRing(ri), hole_angle_tol_deg));
            }
            out.addGeometry(clean);
        }
        return out;
    }

    // 计算Cell（子区域）最长边的方向角 (→ yingshi::computeCellMainDirection)
    double computeCellMainDirection(const f2c::types::Cell& cell) const
    {
        return yingshi::computeCellMainDirection(cell);
    }

    // ========== 优化函数：检测斜边并返回最佳 swath 角度 ==========
    // 场景：sweep 分解产生水平条带，若 cell 贴着斜边界（如 notched L 形斜边），
    // 水平 swath 会在斜边末端形成三角缝隙。此处检测 cell 是否贴近斜边界，
    // 若是则返回斜边方向作为 swath 角度，使 swath 平行于斜边以减少末端三角。
    //
    // 参数：
    //   cell         - 待检查的子区域
    //   full_polygon - 原始多边形（含外环+孔洞）
    //   default_angle- sweep 默认方向（通常≈0，即水平）
    //   cov_width    - 覆盖宽度（用于判定"贴近"阈值）
    //
    // 返回：斜边的方向角（弧度）；若无合适斜边则返回 default_angle
    double detectSlantedBoundaryAngle(
        const f2c::types::Cell& cell,
        const f2c::types::Cell& full_polygon,
        double default_angle,
        double cov_width) const
    {
        const auto& cell_ring = cell.getExteriorRing();
        const auto& poly_ring = full_polygon.getExteriorRing();
        if (cell_ring.size() < 3 || poly_ring.size() < 4) return default_angle;

        // ── 收集 cell 顶点，计算 bbox ──
        std::vector<std::pair<double,double>> cpts;
        double c_min_x=1e9, c_max_x=-1e9, c_min_y=1e9, c_max_y=-1e9;
        for (size_t i = 0; i + 1 < cell_ring.size(); ++i) {
            double cx = cell_ring.getGeometry(i).getX();
            double cy = cell_ring.getGeometry(i).getY();
            cpts.push_back({cx, cy});
            if (cx < c_min_x) c_min_x = cx;
            if (cx > c_max_x) c_max_x = cx;
            if (cy < c_min_y) c_min_y = cy;
            if (cy > c_max_y) c_max_y = cy;
        }

        // ── 扫描多边形外环，找到 cell 贴近的斜边 ──
        // 贴近判定：cell 顶点到边距离 < cov_width
        double best_len = 0.0;
        double best_angle = default_angle;
        const double slant_threshold = 0.9659;  // cos(15°)，边与默认方向夹角>15°视为斜边

        for (size_t pi = 0; pi + 1 < poly_ring.size(); ++pi) {
            double px1 = poly_ring.getGeometry(pi).getX();
            double py1 = poly_ring.getGeometry(pi).getY();
            double px2 = poly_ring.getGeometry(pi+1).getX();
            double py2 = poly_ring.getGeometry(pi+1).getY();

            double edx = px2 - px1, edy = py2 - py1;
            double elen = std::hypot(edx, edy);
            if (elen < 0.05) continue;

            // ── 检查是否为斜边（15° < 与默认方向夹角 < 75°）──
            // 夹角太小 → 平行于 sweep，无意义
            // 夹角太大 → 垂直于 sweep，水平条带中产生过多短 swath
            double cos_slant = std::abs(edx * std::cos(default_angle) + edy * std::sin(default_angle)) / elen;
            if (cos_slant > slant_threshold) continue;  // 夹角<15°，近似平行，跳过
            if (cos_slant < 0.2588) continue;           // 夹角>75°，近似垂直，跳过（避免过多短swath）

            // ── 边 bbox 与 cell bbox 相交？──
            double e_min_x = std::min(px1, px2), e_max_x = std::max(px1, px2);
            double e_min_y = std::min(py1, py2), e_max_y = std::max(py1, py2);
            if (e_max_x < c_min_x - cov_width || e_min_x > c_max_x + cov_width ||
                e_max_y < c_min_y - cov_width || e_min_y > c_max_y + cov_width) continue;

            // ── cell 顶点到边的最短距离 ──
            double min_dist = 1e9;
            for (const auto& cp : cpts) {
                double t = ((cp.first - px1)*edx + (cp.second - py1)*edy) / (elen*elen);
                t = std::max(0.0, std::min(1.0, t));
                double nx = px1 + t*edx - cp.first;
                double ny = py1 + t*edy - cp.second;
                double d = std::hypot(nx, ny);
                if (d < min_dist) min_dist = d;
            }

            if (min_dist < cov_width * 1.5 && elen > best_len) {
                best_len = elen;
                best_angle = std::atan2(edy, edx);
            }
        }

        if (best_len > 0.0 && best_angle != default_angle) {
            RCLCPP_INFO(this->get_logger(),
                "  Slanted boundary detected: cell touches edge at %.1f° (default=%.1f°), len=%.2f m",
                best_angle * 180.0 / M_PI, default_angle * 180.0 / M_PI, best_len);
        }
        return best_angle;
    }

    // 旋转 Cell（用于倾斜 sweep）(→ yingshi::rotateCell)
    f2c::types::Cell rotateCell(const f2c::types::Cell& cell, double angle) const
    {
        return yingshi::rotateCell(cell, angle);
    }

    // 旋转 Swath（绕原点）(→ yingshi::rotateSwath)
    f2c::types::Swath rotateSwath(const f2c::types::Swath& sw, double angle) const
    {
        return yingshi::rotateSwath(sw, angle);
    }

    // ========== 优化函数：从多边形边缘自动提取候选角度 ==========
    // 依据 Rotating Calipers 定理：最优 swath 方向一定平行于多边形某条边 (→ yingshi::extractEdgeAngles)
    std::vector<double> extractAllEdgeAngles(
        const f2c::types::Cell& cell,
        double dedup_tolerance_deg = 2.0) const
    {
        return yingshi::extractEdgeAngles(cell, dedup_tolerance_deg);
    }

    // 提取分解角度候选（边缘垂直方向，Huang 2001定理）(→ yingshi::extractDecompositionAngles)
    std::vector<double> extractDecompositionAngles(
        const f2c::types::Cell& cell) const
    {
        return yingshi::extractDecompositionAngles(cell);
    }

    void publishLastPath()
    {
        // 为每个已接收的多边形重新发布路径
        for (int i = 0; i < 4; ++i) {
            if (!polygon_received_[i]) continue;
            
            if (!last_paths_[i].poses.empty()) {
                // 保留原始规划时间戳，不刷新——旧计划不得伪装成新计划。
                path_pubs_[i]->publish(last_paths_[i]);
            }
            if (!last_swath_points_[i].poses.empty()) {
                swath_points_pubs_[i]->publish(last_swath_points_[i]);
            }
            if (!last_sampled_points_[i].poses.empty()) {
                sampled_path_pubs_[i]->publish(last_sampled_points_[i]);
            }
            if (last_polygons_[i].points.size() > 0) {
                auto outline = createPolygonPath(last_polygons_[i]);
                polygon_viz_pubs_[i]->publish(outline);
            }
            if (!last_holes_outlines_[i].poses.empty()) {
                holes_outline_pubs_[i]->publish(last_holes_outlines_[i]);
            }
            if (!last_decomposition_outlines_[i].poses.empty()) {
                decomposition_outline_pubs_[i]->publish(last_decomposition_outlines_[i]);
            }
            if (!last_infeasible_turn_markers_[i].points.empty()) {
                infeasible_turn_pubs_[i]->publish(last_infeasible_turn_markers_[i]);
            }
        }
    }

    nav_msgs::msg::Path createPolygonPath(const geometry_msgs::msg::Polygon& polygon)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();

        // 添加多边形的顶点
        for (const auto& point : polygon.points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0;
            
            // 设置一个默认的朝向（这里不太重要，因为我们只关心位置）
            pose.pose.orientation.w = 1.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            
            path.poses.push_back(pose);
        }

        // 如果多边形未闭合，添加闭合点
        if (!polygon.points.empty() && 
            (polygon.points.front().x != polygon.points.back().x ||
             polygon.points.front().y != polygon.points.back().y)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = polygon.points.front().x;
            pose.pose.position.y = polygon.points.front().y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        return path;
    }

    // 计算 Polygon 的面积（使用鞋带公式）
    double calculatePolygonArea(const geometry_msgs::msg::Polygon& polygon) const
    {
        if (polygon.points.size() < 3) {
            return 0.0;
        }
        
        double area = 0.0;
        size_t n = polygon.points.size();
        
        // 使用鞋带公式（Shoelace formula）计算多边形面积
        for (size_t i = 0; i < n; ++i) {
            size_t next_i = (i + 1) % n;
            area += polygon.points[i].x * polygon.points[next_i].y;
            area -= polygon.points[next_i].x * polygon.points[i].y;
        }
        
        // 返回绝对值的一半
        return std::abs(area) / 2.0;
    }

    // 过滤 holes，剔除面积小于阈值的
    std::vector<geometry_msgs::msg::Polygon> filterHolesByArea(
        const std::vector<geometry_msgs::msg::Polygon>& holes, 
        double min_area) const
    {
        std::vector<geometry_msgs::msg::Polygon> filtered_holes;
        
        for (size_t i = 0; i < holes.size(); ++i) {
            const auto& hole = holes[i];
            double area = calculatePolygonArea(hole);
            
            if (area >= min_area) {
                filtered_holes.push_back(hole);
                RCLCPP_DEBUG(this->get_logger(), 
                           "Hole %zu: area=%.3f m² (>= %.3f m²), kept", 
                           i, area, min_area);
            } else {
                RCLCPP_INFO(this->get_logger(), 
                           "Hole %zu: area=%.3f m² (< %.3f m²), filtered out", 
                           i, area, min_area);
            }
        }
        
        return filtered_holes;
    }

    nav_msgs::msg::Path createHolesOutlinePath(const std::vector<geometry_msgs::msg::Polygon>& holes)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();

        // 为每个hole创建轮廓路径
        for (size_t hole_idx = 0; hole_idx < holes.size(); ++hole_idx) {
            const auto& hole = holes[hole_idx];
            
            if (hole.points.size() < 3) {
                continue;
            }

            // 添加当前hole的所有点
            for (const auto& point : hole.points) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = 0.0;
                
                // 设置朝向
                pose.pose.orientation.w = 1.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                
                path.poses.push_back(pose);
            }

            // 确保hole闭合（如果未闭合，添加第一个点）
            if (!hole.points.empty() && 
                (hole.points.front().x != hole.points.back().x ||
                 hole.points.front().y != hole.points.back().y)) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = hole.points.front().x;
                pose.pose.position.y = hole.points.front().y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
        }

        return path;
    }

    nav_msgs::msg::Path createDecompositionOutlinePath(const f2c::types::Cells& cells)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();

        // 每个 cell 外环后插入一个"抬笔点"来切断 cell 间连线
        for (size_t cell_idx = 0; cell_idx < cells.size(); ++cell_idx) {
            f2c::types::Cell f2c_cell = cells.getGeometry(cell_idx);

            const auto& exterior_ring = f2c_cell.getExteriorRing();
            if (exterior_ring.size() >= 3) {
                for (size_t pi = 0; pi < exterior_ring.size(); ++pi) {
                    const auto& pt = exterior_ring.getGeometry(pi);
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path.header;
                    pose.pose.position.x = pt.getX();
                    pose.pose.position.y = pt.getY();
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    path.poses.push_back(pose);
                }
                // 闭合
                const auto& fp = exterior_ring.getGeometry(0);
                const auto& lp = exterior_ring.getGeometry(exterior_ring.size() - 1);
                if (std::abs(fp.getX() - lp.getX()) > 1e-6 ||
                    std::abs(fp.getY() - lp.getY()) > 1e-6) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path.header;
                    pose.pose.position.x = fp.getX();
                    pose.pose.position.y = fp.getY();
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    path.poses.push_back(pose);
                }
            }

            // 内环（孔洞边界）
            for (size_t ri = 0; ri + 1 < f2c_cell.size(); ++ri) {
                const auto& ir = f2c_cell.getInteriorRing(ri);
                if (ir.size() < 3) continue;
                for (size_t pi = 0; pi < ir.size(); ++pi) {
                    const auto& pt = ir.getGeometry(pi);
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path.header;
                    pose.pose.position.x = pt.getX();
                    pose.pose.position.y = pt.getY();
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    path.poses.push_back(pose);
                }
                const auto& fp = ir.getGeometry(0);
                const auto& lp = ir.getGeometry(ir.size() - 1);
                if (std::abs(fp.getX() - lp.getX()) > 1e-6 ||
                    std::abs(fp.getY() - lp.getY()) > 1e-6) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path.header;
                    pose.pose.position.x = fp.getX();
                    pose.pose.position.y = fp.getY();
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    path.poses.push_back(pose);
                }
            }
        }

        return path;
    }

    void holesCallback(const geometry_msgs::msg::Polygon::SharedPtr msg, int polygon_id)
    {
        int index = polygon_id - 1;  // 转换为数组索引 (1-4 -> 0-3)
        
        // 解析holes消息（使用特殊标记x=1e10分隔不同的hole）
        std::vector<geometry_msgs::msg::Polygon> holes;
        geometry_msgs::msg::Polygon current_hole;
        const double HOLE_SEPARATOR = 1e10;  // 特殊标记值
        
        for (const auto& point : msg->points) {
            if (std::abs(point.x - HOLE_SEPARATOR) < 1.0) {
                // 遇到分隔符，保存当前hole并开始新的hole
                if (current_hole.points.size() >= 3) {
                    holes.push_back(current_hole);
                }
                current_hole.points.clear();
            } else {
                current_hole.points.push_back(point);
            }
        }
        
        // 添加最后一个hole
        if (current_hole.points.size() >= 3) {
            holes.push_back(current_hole);
        }
        
        // 计算每个hole的面积并打印
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Received %zu holes for polygon_%d (before filtering):", 
                    holes.size(), polygon_id);
        double total_area = 0.0;
        for (size_t i = 0; i < holes.size(); ++i) {
            double area = calculatePolygonArea(holes[i]);
            total_area += area;
            RCLCPP_INFO(this->get_logger(), 
                       "  Hole %zu: %zu points, area=%.3f m²", 
                       i, holes[i].points.size(), area);
        }
        RCLCPP_INFO(this->get_logger(), 
                   "  Total area of all holes: %.3f m²", total_area);
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        // 过滤holes：剔除面积小于阈值的
        std::vector<geometry_msgs::msg::Polygon> filtered_holes = filterHolesByArea(holes, min_hole_area_);
        
        RCLCPP_INFO(this->get_logger(), 
                   "After filtering (min_area=%.3f m²): %zu holes kept, %zu holes filtered out", 
                   min_hole_area_, filtered_holes.size(), 
                   holes.size() - filtered_holes.size());
        
        // 保存过滤后的holes
        last_holes_[index] = filtered_holes;
        holes_received_[index] = true;  // 标记holes消息已接收（即使为空）

        // 创建并发布holes轮廓路径（使用过滤后的holes）
        nav_msgs::msg::Path holes_outline = createHolesOutlinePath(filtered_holes);
        last_holes_outlines_[index] = holes_outline;
        holes_outline_pubs_[index]->publish(holes_outline);
        RCLCPP_INFO(this->get_logger(), 
                   "Published holes outline to topic: /holes_outline_%d", polygon_id);
        RCLCPP_INFO(this->get_logger(), 
                   "  Total holes (filtered): %zu, Total points in path: %zu", 
                   filtered_holes.size(), holes_outline.poses.size());

        // 如果多边形已接收，规划路径（包含holes）
        if (polygon_received_[index]) {
            RCLCPP_INFO(this->get_logger(), 
                       "Polygon and holes both received, starting path planning with %zu holes (after filtering)", 
                       filtered_holes.size());
            planCoveragePath(last_polygons_[index], polygon_id);
        } else {
            RCLCPP_INFO(this->get_logger(), 
                       "Holes received, waiting for polygon before path planning...");
        }
    }

    void polygonCallback(const geometry_msgs::msg::Polygon::SharedPtr msg, int polygon_id)
    {
        int index = polygon_id - 1;  // 转换为数组索引 (1-4 -> 0-3)

        RCLCPP_INFO(this->get_logger(), "Received polygon_%d with %zu points:",
                    polygon_id, msg->points.size());
        for (size_t i = 0; i < msg->points.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Point %zu: (%.2f, %.2f, %.2f)", 
                        i, msg->points[i].x, msg->points[i].y, msg->points[i].z);
        }

        // 保存多边形并发布可视化路径
        last_polygons_[index] = *msg;
        polygon_received_[index] = true;
        
        auto outline = createPolygonPath(last_polygons_[index]);
        polygon_viz_pubs_[index]->publish(outline);
        RCLCPP_INFO(this->get_logger(), "Published polygon_%d outline with %zu points", 
                    polygon_id, outline.poses.size());

        // 检查holes消息是否已接收
        // 如果holes消息已接收（即使为空），说明holes信息完整，可以规划
        // 如果holes消息未接收，等待holes消息到达
        // 这样可以避免先规划一次没有holes的路径，然后再规划一次有holes的路径
        if (holes_received_[index]) {
            RCLCPP_INFO(this->get_logger(), 
                       "Polygon and holes both received, starting path planning with %zu holes", 
                       last_holes_[index].size());
            planCoveragePath(*msg, polygon_id);
        } else {
            RCLCPP_INFO(this->get_logger(), 
                       "Polygon received, waiting for holes message before path planning...");
            RCLCPP_INFO(this->get_logger(), 
                       "  (Note: Even if there are no holes, an empty holes message will be sent)");
        }
    }

    // ========== 优化函数：Swath 多角度选择 ==========
    // 对单个 Cell 尝试多个候选角度，返回 swaths 数量最少的方案
    f2c::types::Swaths optimizeSwathAngle(
        const f2c::types::Cell& cell,
        f2c::sg::BruteForce& swath_generator,
        double cov_width,
        const std::vector<double>& angle_candidates) const
    {
        if (angle_candidates.empty()) {
            // 回退：使用最长边方向
            double default_angle = computeCellMainDirection(cell);
            return swath_generator.generateSwaths(default_angle, cov_width, cell);
        }

        f2c::types::Swaths best_swaths;
        size_t best_count = std::numeric_limits<size_t>::max();
        double best_angle = 0.0;

        for (double angle : angle_candidates) {
            auto candidate_swaths = swath_generator.generateSwaths(angle, cov_width, cell);
            size_t count = candidate_swaths.size();

            RCLCPP_INFO(this->get_logger(),
                       "  Angle candidate %.1f deg: %zu swaths",
                       angle * 180.0 / M_PI, count);

            if (count > 0 && count < best_count) {
                best_count = count;
                best_swaths = candidate_swaths;
                best_angle = angle;
            }
        }

        if (best_count < std::numeric_limits<size_t>::max()) {
            RCLCPP_INFO(this->get_logger(),
                       "  Best angle: %.1f deg with %zu swaths",
                       best_angle * 180.0 / M_PI, best_count);
        }

        return best_swaths;
    }

    // ========== 优化函数：边界间隙补填 ==========
    //   swath 排完后，最外层到 cell 边界的剩余距离不够一条 swath 也要补
    //   宁愿重叠率上去，也要覆盖率
    // full_polygon: 原始完整多边形（外环 + 孔洞），用于确定真实边界
    //               cell 是 no_hl 子区域，其边界已被 headland 侵蚀
    //               当 cell 边界贴近 full_polygon 外环时，用外环作为间隙检测参考
    void fillBoundaryGaps(
        f2c::types::Swaths& cell_swaths,
        const f2c::types::Cell& cell,
        const f2c::types::Cell& full_polygon,
        double swath_angle,
        double cov_width,
        double shrink_dist) const
    {
        size_t before = cell_swaths.size();
        yingshi::fillBoundaryGaps(cell_swaths, cell, full_polygon,
                                  swath_angle, cov_width, shrink_dist);
        size_t after = cell_swaths.size();
        if (after > before) {
            RCLCPP_INFO(this->get_logger(),
                "  fillBoundaryGaps: %zu swaths → %zu swaths (+%zu boundary fills)",
                before, after, after - before);
        }
    }

    // ========== 优化函数：过滤微小子区域 ==========
    // 过滤面积过小的 Cell，避免生成只有 0~1 条 swath 的无效子区域
    f2c::types::Cells filterTinyCells(
        const f2c::types::Cells& cells,
        double min_area) const
    {
        if (min_area <= 0.0) return cells;

        f2c::types::Cells filtered;
        size_t removed = 0;

        for (size_t i = 0; i < cells.size(); ++i) {
            const auto& cell = cells.getGeometry(i);
            double area = cell.area();

            if (area < min_area) {
                ++removed;
                RCLCPP_INFO(this->get_logger(),
                           "Filtered tiny cell %zu: area=%.3f m² < %.3f m²",
                           i, area, min_area);
                continue;
            }
            filtered.addGeometry(cell);
        }

        if (removed > 0) {
            RCLCPP_INFO(this->get_logger(),
                       "Tiny cell filter: removed %zu/%zu cells (min_area=%.3f m²)",
                       removed, cells.size(), min_area);
        }

        return filtered;
    }

    // ========== 优化函数：RDP 路径简化（分段感知版） ==========
    // Ramer-Douglas-Peucker 算法，剔除共线或近似共线的冗余路径点
    //
    // 【修复说明】原版 RDP 对整条路径（swath 直线段 + Dubins 曲线连接段）一次性简化，
    // Dubins 曲线的密集点会被整段删除，导致 swath 之间退化为直线连接，
    // 在 RViz 中表现为路径"不闭合"。
    //
    // 修复方案：先检测路径中的转弯点（方向突变），标记为必须保留的段边界，
    // 然后只在相邻转弯点之间的"准直线段"内执行 RDP，保护曲线连接不被破坏。
    std::vector<f2c::types::PathState> simplifyPathRDP(
        const f2c::types::Path& path,
        double epsilon) const
    {
        if (path.size() < 3 || epsilon <= 0.0) {
            // 不简化，直接拷贝
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

        // ── 步骤1：检测转弯点（段边界） ──
        // 计算相邻三点的方向变化，方向突变 > 阈值标记为"转弯点"
        // 转弯点是 swath 与 Dubins 曲线的交界，必须保留
        std::vector<bool> keep(points.size(), false);
        keep[0] = true;                          // 起点必须保留
        keep[points.size() - 1] = true;          // 终点必须保留

        const double turn_angle_threshold = path_simplify_turn_threshold_;  // ROS 参数，区分直线行驶和转弯
        std::vector<int> turn_points;              // 所有段边界索引（含首尾）
        turn_points.push_back(0);

        for (size_t i = 2; i < points.size(); ++i) {
            double dx1 = points[i-1].x - points[i-2].x;
            double dy1 = points[i-1].y - points[i-2].y;
            double dx2 = points[i].x - points[i-1].x;
            double dy2 = points[i].y - points[i-1].y;

            double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

            if (len1 > 1e-9 && len2 > 1e-9) {
                // 使用叉积/点积计算转角（更精确）
                double cross = dx1 * dy2 - dy1 * dx2;
                double dot = dx1 * dx2 + dy1 * dy2;
                double angle = std::abs(std::atan2(cross, dot));

                if (angle > turn_angle_threshold) {
                    keep[i - 1] = true;  // 标记转弯点必须保留
                    turn_points.push_back(static_cast<int>(i - 1));
                }
            }
        }
        turn_points.push_back(static_cast<int>(points.size()) - 1);

        // ── 步骤2：RDP 递归（保持不变） ──
        std::function<void(int, int)> rdp = [&](int start, int end) {
            if (end - start <= 1) return;

            double max_dist = 0.0;
            int max_idx = start;

            double dx = points[end].x - points[start].x;
            double dy = points[end].y - points[start].y;
            double len = std::sqrt(dx * dx + dy * dy);

            for (int i = start + 1; i < end; ++i) {
                // 跳过已标记为必须保留的点（转弯点）
                if (keep[i]) continue;

                double dist;
                if (len < 1e-12) {
                    double pdx = points[i].x - points[start].x;
                    double pdy = points[i].y - points[start].y;
                    dist = std::sqrt(pdx * pdx + pdy * pdy);
                } else {
                    // 点到线段距离（叉积法）
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

        // ── 步骤3：逐段执行 RDP（每个转弯点之间的独立段） ──
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

        RCLCPP_INFO(this->get_logger(),
                   "RDP simplification: %zu -> %zu points (epsilon=%.3f m, removed %.1f%%, %zu segments)",
                   path.size(), simplified.size(), epsilon,
                   (1.0 - static_cast<double>(simplified.size()) / path.size()) * 100.0,
                   turn_points.size() - 1);

        return simplified;
    }

    // ========== PlannerCore 桥接（P2 迁移）==========

    // 从当前 ROS 参数和消息构建纯算法请求
    yingshi::PlanningRequest buildPlanningRequest(
        const geometry_msgs::msg::Polygon& polygon_msg,
        int polygon_id) const
    {
        yingshi::PlanningRequest req;
        int idx = polygon_id - 1;

        // 几何输入
        f2c::types::LinearRing ring;
        for (const auto& p : polygon_msg.points)
            ring.addPoint(f2c::types::Point(p.x, p.y));
        if (polygon_msg.points.size() > 0 &&
            (polygon_msg.points.front().x != polygon_msg.points.back().x ||
             polygon_msg.points.front().y != polygon_msg.points.back().y))
            ring.addPoint(f2c::types::Point(
                polygon_msg.points.front().x,
                polygon_msg.points.front().y));

        req.polygon.setGeometry(0, ring);
        for (size_t hi = 0; hi < last_holes_[idx].size(); ++hi) {
            const auto& h = last_holes_[idx][hi];
            if (h.points.size() < 3) continue;
            f2c::types::LinearRing hr;
            for (const auto& hp : h.points)
                hr.addPoint(f2c::types::Point(hp.x, hp.y));
            // 闭合 ring
            if (hr.size() >= 3) {
                const auto& first = h.points.front();
                const auto& last = h.points.back();
                if (first.x != last.x || first.y != last.y)
                    hr.addPoint(f2c::types::Point(first.x, first.y));
            }
            if (hr.size() >= 4) {
                req.holes.push_back(hr);
                req.polygon.addRing(hr);  // 作为内环加入 polygon
            }
        }

        // 机器人参数
        req.robot_width = robot_width_;
        req.coverage_width = coverage_width_;
        req.min_turning_radius = min_turning_radius_;
        req.max_diff_curv = max_diff_curv_;

        // Headland 参数
        req.mid_hl_width_ratio = mid_hl_width_ratio_;
        req.no_hl_width_ratio = no_hl_width_ratio_;

        // Swath 参数
        req.swath_overlap_ratio = swath_overlap_ratio_;
        req.swath_endpoint_shrink_distance = swath_endpoint_shrink_distance_;
        req.min_swath_length = min_swath_length_;

        // 分解参数
        req.decomposition_enabled = decomposition_enabled_;
        req.use_sweep_decomp = use_sweep_decomp_;
        req.decomposition_angle_optimization = decomposition_angle_optimization_;
        req.merge_angle_threshold = merge_angle_threshold_;

        // Swath 优化
        req.swath_angle_optimization = swath_angle_optimization_;
        req.swath_angle_candidates = swath_angle_list_;

        // 排序 + 路径
        req.swath_order_type = swath_order_type_;
        req.turn_planner_type = turn_planner_type_;
        req.path_simplify_enabled = path_simplify_enabled_;
        req.path_simplify_tolerance = path_simplify_tolerance_;
        req.path_simplify_turn_threshold = path_simplify_turn_threshold_;

        // 过滤
        req.filter_tiny_cells = filter_tiny_cells_;
        req.min_cell_area_ratio = min_cell_area_ratio_;
        req.min_hole_area = min_hole_area_;

        // 边界策略
        req.boundary_type = boundary_type_;
        req.boundary_coverage_margin = boundary_coverage_margin_;
        req.boundary_open_default_margin = boundary_open_default_margin_;

        return req;
    }

    yingshi::PlannerCore core_;  // P2: 纯算法规划核心

    // ========== PlannerCore 规划路径（P2 迁移）==========
    void planWithCore(const geometry_msgs::msg::Polygon& polygon, int polygon_id)
    {
        int index = polygon_id - 1;
        auto planning_start = std::chrono::steady_clock::now();

        // 1. 构建纯算法请求
        auto req = buildPlanningRequest(polygon, polygon_id);

        // 2. 执行规划
        auto result = core_.plan(req);

        if (!result.success) {
            RCLCPP_ERROR(this->get_logger(),
                "PlannerCore plan() failed: %s", result.error_message.c_str());
            clearPlanningCacheForPolygon(index, true);
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "PlannerCore: %zu swaths, %zu connections, %.1f ms",
            result.total_swaths, result.total_connections,
            result.planning_time_ms);

        if (result.path_has_crossings) {
            RCLCPP_WARN(this->get_logger(),
                "⚠ PlannerCore: %zu hole-crossing segments detected",
                result.hole_crossing_segments);
        }

        // 3. 发布 nav_msgs::Path（从 F2C Path 完整还原，含角度）
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        const auto path_waypoints = yingshi::materializePathWaypoints(result.path);
        for (const auto& wp : path_waypoints) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = wp.point.getX();
            ps.pose.position.y = wp.point.getY();
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = std::cos(wp.angle / 2.0);
            ps.pose.orientation.z = std::sin(wp.angle / 2.0);
            path_msg.poses.push_back(ps);
        }

        // 发布前自洽性检查
        {
            std::vector<f2c::types::LinearRing> sanity_holes;
            for (const auto& hole : last_holes_[index]) {
                auto hr = makeClosedF2CRing(hole);
                if (hr.size() >= 4) sanity_holes.push_back(hr);
            }
            auto sanity = yingshi::checkPathSanity(
                result.path, result.path_points, req.polygon,
                sanity_holes, result.total_swaths);
            if (!sanity.passed) {
                for (const auto& iss : sanity.issues) {
                    if (iss.severity == yingshi::SanityIssue::Severity::ERROR)
                        RCLCPP_ERROR(this->get_logger(), "Core Sanity: %s", iss.message.c_str());
                    else
                        RCLCPP_WARN(this->get_logger(), "Core Sanity: %s", iss.message.c_str());
                }
            }
        }

        last_paths_[index] = path_msg;
        path_pubs_[index]->publish(path_msg);

        // 4. 发布 swath 端点（可视化）
        geometry_msgs::msg::PoseArray swath_pts;
        swath_pts.header.frame_id = "map";
        swath_pts.header.stamp = this->now();

        // 从 Route 提取 swaths
        f2c::types::Swaths all_swaths;
        for (size_t i = 0; i < result.route.sizeVectorSwaths(); ++i) {
            const auto& rs = result.route.getSwaths(i);
            for (size_t j = 0; j < rs.size(); ++j)
                all_swaths.push_back(rs.at(j));
        }

        for (const auto& sw : all_swaths) {
            geometry_msgs::msg::Pose sp, ep;
            sp.position.x = sw.startPoint().getX();
            sp.position.y = sw.startPoint().getY();
            sp.position.z = 0.0;
            ep.position.x = sw.endPoint().getX();
            ep.position.y = sw.endPoint().getY();
            ep.position.z = 0.0;
            double dx = ep.position.x - sp.position.x;
            double dy = ep.position.y - sp.position.y;
            double yaw = std::atan2(dy, dx);
            sp.orientation.w = std::cos(yaw / 2.0);
            sp.orientation.z = std::sin(yaw / 2.0);
            ep.orientation = sp.orientation;
            swath_pts.poses.push_back(sp);
            swath_pts.poses.push_back(ep);
        }

        last_swath_points_[index] = swath_pts;
        swath_points_pubs_[index]->publish(swath_pts);

        // 5. 发布采样路径点（在 swath 线上均匀采样）
        geometry_msgs::msg::PoseArray sampled_pts;
        sampled_pts.header.frame_id = "map";
        sampled_pts.header.stamp = this->now();

        for (const auto& sw : all_swaths) {
            double dx = sw.endPoint().getX() - sw.startPoint().getX();
            double dy = sw.endPoint().getY() - sw.startPoint().getY();
            double len = std::hypot(dx, dy);
            double yaw = std::atan2(dy, dx);
            int n = std::max(2, static_cast<int>(len / path_resolution_));
            for (int i = 0; i < n; ++i) {
                double r = static_cast<double>(i) / (n - 1);
                geometry_msgs::msg::Pose p;
                p.position.x = sw.startPoint().getX() + r * dx;
                p.position.y = sw.startPoint().getY() + r * dy;
                p.position.z = 0.0;
                p.orientation.w = std::cos(yaw / 2.0);
                p.orientation.z = std::sin(yaw / 2.0);
                sampled_pts.poses.push_back(p);
            }
        }

        last_sampled_points_[index] = sampled_pts;
        sampled_path_pubs_[index]->publish(sampled_pts);

        // 6. 清除可能残留的掉头不可执行标记（direct 模式无转弯曲线）
        {
            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = "map";
            clear.header.stamp = this->now();
            clear.ns = "infeasible_turns";
            clear.id = 0;
            clear.action = visualization_msgs::msg::Marker::DELETE;
            infeasible_turn_pubs_[index]->publish(clear);
            last_infeasible_turn_markers_[index] = visualization_msgs::msg::Marker();
        }

        // 7. 孔洞交叉诊断
        if (!last_holes_[index].empty()) {
            std::vector<f2c::types::LinearRing> hole_rings;
            for (const auto& hole : last_holes_[index]) {
                auto hr = makeClosedF2CRing(hole);
                if (hr.size() >= 4) hole_rings.push_back(hr);
            }
            if (!hole_rings.empty()) {
                size_t pts_in_hole = 0, segs_crossing = 0;
                for (const auto& pt : result.path_points) {
                    if (yingshi::pointInAnyHole(pt.getX(), pt.getY(), hole_rings))
                        ++pts_in_hole;
                }
                for (size_t si = 0; si + 1 < result.path_points.size(); ++si) {
                    if (yingshi::segmentCrossesHole(
                            result.path_points[si].getX(), result.path_points[si].getY(),
                            result.path_points[si+1].getX(), result.path_points[si+1].getY(),
                            hole_rings, 50))
                        ++segs_crossing;
                }
                RCLCPP_INFO(this->get_logger(),
                    "  Core path (%zu pts): %zu pts in hole, %zu segs crossing",
                    result.path_points.size(), pts_in_hole, segs_crossing);
                if (segs_crossing > 0) {
                    RCLCPP_WARN(this->get_logger(),
                        "⚠ CORE PATH CROSSES HOLE! (%zu segments)", segs_crossing);
                }
            }
        }

        // 8. 评估（复用 PlannerCore 结果，重建需要的中间变量）
#if YINGSHI_EVAL_ENABLED
        if (eval_enable_report_) {
            // 构建 full_polygon Cells（含孔洞内环）
            f2c::types::Cells full_polygon_cells;
            full_polygon_cells.addGeometry(req.polygon);

            // 构建孔洞环
            std::vector<f2c::types::LinearRing> eval_hole_rings;
            for (const auto& hole : last_holes_[index]) {
                auto hr = makeClosedF2CRing(hole);
                if (hr.size() >= 4) eval_hole_rings.push_back(hr);
            }

            // 从 Route 提取所有 swaths
            f2c::types::Swaths eval_swaths;
            for (size_t i = 0; i < result.route.sizeVectorSwaths(); ++i) {
                const auto& rs = result.route.getSwaths(i);
                for (size_t j = 0; j < rs.size(); ++j)
                    eval_swaths.push_back(rs.at(j));
            }

            EvalParams eval_params;
            eval_params.max_diff_curv = max_diff_curv_;
            eval_params.coverage_width = coverage_width_;
            eval_params.swath_overlap_ratio = swath_overlap_ratio_;
            eval_params.turn_planner_type = turn_planner_type_.c_str();
            eval_params.grid_resolution = eval_grid_resolution_;
            eval_params.coverage_threshold = eval_coverage_threshold_;
            eval_params.turn_angle_threshold = 30.0;
            eval_params.use_grid_method = eval_use_grid_method_;

            if (eval_use_grid_method_) {
                double est_area = (req.polygon.getExteriorRing().size() > 0)
                    ? req.polygon.area() : 0.0;
                int est_points = static_cast<int>(est_area /
                    (eval_grid_resolution_ * eval_grid_resolution_));
                RCLCPP_INFO(this->get_logger(),
                    "Core eval: area=%.1f m², resolution=%.2f m, "
                    "est_grid_points=%d, path_points=%zu",
                    est_area, eval_grid_resolution_, est_points,
                    result.path_points.size());
            }

            EvalResult eval_result = evaluatePlan(
                result.path, eval_swaths, full_polygon_cells, eval_hole_rings,
                result.planning_time_ms, eval_params);

            std::string scenario_label = "polygon_" + std::to_string(polygon_id);
            std::string report = formatEvalReport(eval_result, scenario_label.c_str());
            RCLCPP_INFO(this->get_logger(), "Core %s", report.c_str());

            // 写网格 JSON
            if (eval_use_grid_method_ && eval_result.grid_resolution > 0.0) {
                std::string grid_path = output_dir_ + "/f2c_grid_core_"
                    + scenario_label + ".json";
                writeGridJson(eval_result, grid_path);
            }
        }
#endif

        auto planning_end = std::chrono::steady_clock::now();
        double total_ms = std::chrono::duration<double, std::milli>(
            planning_end - planning_start).count();
        RCLCPP_INFO(this->get_logger(),
            "PlannerCore done: %zu path waypoints, %.1f ms total (plan=%.1f ms)",
            path_waypoints.size(), total_ms, result.planning_time_ms);
    }

    // ========== 主规划函数 ==========
    void planCoveragePath(const geometry_msgs::msg::Polygon& polygon, int polygon_id)
    {
        // P2 迁移开关：PlannerCore 已接入时走新管线
        if (use_planner_core_) {
            planWithCore(polygon, polygon_id);
            return;
        }

        int index = polygon_id - 1;  // 转换为数组索引（需在 try 外供 catch 访问）
        try {
            RCLCPP_INFO(this->get_logger(), "Starting coverage path planning for polygon_%d...", polygon_id);

            // ── 重置 Vis JSON 状态 ──
            // 用 cell 计数器替代文件存在性检测，避免多次 polygon 处理叠加旧数据
            vis_json_cell_count_ = 0;
            std::string vis_path_init = output_dir_ + "/f2c_vis_polygon_" + std::to_string(polygon_id) + ".json";
            std::remove(vis_path_init.c_str());

            // ── 计时开始 ──
            auto planning_start = std::chrono::steady_clock::now();
            
            f2c::types::Robot robot(robot_width_);
            // 使用覆盖宽度构造 Robot，因为实际使用的是覆盖宽度而不是机器人物理宽度
            // f2c::types::Robot robot(coverage_width_);
            robot.setMinTurningRadius(min_turning_radius_);
            robot.setMaxDiffCurv(max_diff_curv_);
            robot.setCovWidth(coverage_width_);

            f2c::types::Cell cell;
            f2c::types::LinearRing ring;
            
            // 创建外环
            for (const auto & p : polygon.points) {
                ring.addPoint(f2c::types::Point(p.x, p.y));
            }
            if (polygon.points.size() > 0 && 
                (polygon.points.front().x != polygon.points.back().x ||
                 polygon.points.front().y != polygon.points.back().y)) {
                ring.addPoint(f2c::types::Point(
                    polygon.points.front().x, 
                    polygon.points.front().y));
            }

            cell.setGeometry(0, ring);
            
            // 添加holes作为内环
            const auto& holes = last_holes_[index];
            if (!holes.empty()) {
                RCLCPP_INFO(this->get_logger(), "Adding %zu holes to polygon_%d (after area filtering)", 
                           holes.size(), polygon_id);
                
                double total_holes_area = 0.0;
                for (size_t hole_idx = 0; hole_idx < holes.size(); ++hole_idx) {
                    const auto& hole = holes[hole_idx];
                    if (hole.points.size() < 3) {
                        RCLCPP_WARN(this->get_logger(), 
                                   "Hole %zu has less than 3 points, skipping", hole_idx);
                        continue;
                    }
                    
                    // 计算hole的面积
                    double hole_area = calculatePolygonArea(hole);
                    total_holes_area += hole_area;
                    
                    const auto hole_ring = makeClosedF2CRing(hole);
                    
                    // 添加为内环（使用addRing方法）
                    cell.addRing(hole_ring);
                    RCLCPP_INFO(this->get_logger(), 
                               "Added hole %zu: %zu points, area=%.3f m²", 
                               hole_idx, hole.points.size(), hole_area);
                }
                RCLCPP_INFO(this->get_logger(), 
                           "Total area of added holes: %.3f m²", total_holes_area);
            } else {
                RCLCPP_INFO(this->get_logger(), "No holes for polygon_%d", polygon_id);
            }
            
            f2c::types::Cells cells;
            cells.addGeometry(cell);

            // ========== Headland 生成和分解流程 ==========
            // 根据 Fields2Cover 教程，使用分解时，需要确保 headland 环在分解后仍然连接
            // 流程：1) 生成 mid_hl → 2) 分解 mid_hl → 3) 在分解后的区域生成 no_hl → 4) 在 no_hl 上生成 swaths
            
            // 有效覆盖间距 = 覆盖宽度 × (1 - 重叠率)，重叠率>0 时相邻swath有重叠防漏缝
            double r_w = robot.getCovWidth() * (1.0 - swath_overlap_ratio_);
            if (swath_overlap_ratio_ > 0.0) {
                RCLCPP_INFO(this->get_logger(),
                           "Swath overlap: %.1f%%, effective spacing=%.3f m (raw coverage_width=%.3f m)",
                           swath_overlap_ratio_ * 100.0, r_w, coverage_width_);
            }
            
            // ── 分解角度选择 ──
            f2c::hg::ConstHL const_hl;

            // ── 自适应 Headland：检测窄通道，自动降低侵蚀避免堵死 ──
            // 适用场景：门洞连通（S5/S6）、狭长走廊（S4）等
            // 原理：headland 从两侧均匀侵蚀，窄通道可能被完全掐断
            //       找到多边形最窄处，如果 headland 侵蚀会堵死，则自动收窄侵蚀量
            double effective_mid_hl_ratio = mid_hl_width_ratio_;
            double effective_no_hl_ratio = no_hl_width_ratio_;
            {
                // 估算多边形最小通道宽度：遍历非相邻边对，找最小间距
                const auto& ring = cell.getExteriorRing();
                double min_passage = std::numeric_limits<double>::max();
                size_t n = ring.size();
                for (size_t i = 0; i < n; ++i) {
                    size_t i_next = (i + 1) % n;
                    for (size_t j = i + 2; j < n; ++j) {
                        // 跳过相邻边和同一边
                        size_t j_next = (j + 1) % n;
                        if (j == i || j == i_next || j_next == i) continue;
                        // 计算边 i→i_next 与边 j→j_next 之间所有 4 个顶点对的最小距离
                        double di = std::hypot(
                            ring.getGeometry(i).getX() - ring.getGeometry(j).getX(),
                            ring.getGeometry(i).getY() - ring.getGeometry(j).getY());
                        double d2 = std::hypot(
                            ring.getGeometry(i).getX() - ring.getGeometry(j_next).getX(),
                            ring.getGeometry(i).getY() - ring.getGeometry(j_next).getY());
                        double d3 = std::hypot(
                            ring.getGeometry(i_next).getX() - ring.getGeometry(j).getX(),
                            ring.getGeometry(i_next).getY() - ring.getGeometry(j).getY());
                        double d4 = std::hypot(
                            ring.getGeometry(i_next).getX() - ring.getGeometry(j_next).getX(),
                            ring.getGeometry(i_next).getY() - ring.getGeometry(j_next).getY());
                        di = std::min({di, d2, d3, d4});
                        min_passage = std::min(min_passage, di);
                    }
                }
                // 仅当检测到有意义的最小通道时做自适应（排除退化多边形）
                if (min_passage < 10.0 && min_passage > 0.01) {
                    double total_hl_erosion = (mid_hl_width_ratio_ + no_hl_width_ratio_) * robot_width_;
                    // 侵蚀后通道宽度 = min_passage - 2 × total_hl_erosion
                    // 需要保证至少一条 swath（有效间距 r_w）能通过
                    double eroded_passage = min_passage - 2.0 * total_hl_erosion;
                    if (eroded_passage < r_w) {
                        // 反算允许的最大侵蚀量
                        double max_erosion = std::max(0.0, (min_passage - r_w) / 2.0);
                        double max_ratio = max_erosion / robot_width_;
                        // 按比例分配到 mid 和 no
                        double total_ratio = mid_hl_width_ratio_ + no_hl_width_ratio_;
                        if (total_ratio > 1e-9) {
                            double scale = max_ratio / total_ratio;
                            effective_mid_hl_ratio = mid_hl_width_ratio_ * scale;
                            effective_no_hl_ratio = no_hl_width_ratio_ * scale;
                            RCLCPP_INFO(this->get_logger(),
                                       "Adaptive headland: min passage=%.2f m, "
                                       "original erosion=%.3f m would leave %.2f m (< effective spacing=%.2f m). "
                                       "Auto-reducing: mid_hl %.3f→%.3f, no_hl %.3f→%.3f",
                                       min_passage, total_hl_erosion, eroded_passage, r_w,
                                       mid_hl_width_ratio_, effective_mid_hl_ratio,
                                       no_hl_width_ratio_, effective_no_hl_ratio);
                        }
                    }
                }
            }

            // 第一步：生成 mid_hl（使用自适应后的比例）
            // mid_hl 用于 RoutePlannerBase 的连接路径生成
            f2c::types::Cells mid_hl = const_hl.generateHeadlands(cells, effective_mid_hl_ratio * robot_width_);
            mid_hl = simplifyCells(mid_hl, 5.0, 0.5);
            // 诊断：检查孔洞是否在 headland 后仍然存在
            for (size_t mci = 0; mci < mid_hl.size(); ++mci) {
                auto mc = mid_hl.getGeometry(mci);
                RCLCPP_INFO(this->get_logger(),
                           "mid_hl cell %zu: %zu rings (1 ext + %zu holes), area=%.2f m²",
                           mci, mc.size(), mc.size() - 1, mc.area());
            }  // 外环激进简化(5°), 孔洞保守(0.5°)

            // 第二步：生成 no_hl（分解 or 直接）
            f2c::types::Cells no_hl;
            if (decomposition_enabled_) {
                // —— 有分解模式（网格分解，轴对齐多边形 → 纯矩形 cell）——
                RCLCPP_INFO(this->get_logger(),
                           "Step 2: Decomposing mid_hl using rectilinear grid decomposition...");
                f2c::types::Cells decomp_mid_hl;
                // 用原始 field 顶点建网格（干净轴对齐），用侵蚀后 mid_hl 做求交（保留安全间距）
                // ── Phase 4A: 倾斜 sweep — 对齐多边形主边方向 ──
                // 若 sweep 启用了且多边形有明显斜边，旋转多边形使斜边变水平再分解，
                // 分解后旋转回来。避免水平 sweep 在斜边界产生三角末端 cell。
                double sweep_align_angle = 0.0;
                if (use_sweep_decomp_ && cell.size() > 0) {
                    // 检测多边形最长边的方向作为 sweep 对齐方向
                    const auto& poly_ext = cell.getExteriorRing();
                    double longest_edge_len = 0.0;
                    for (size_t pi = 0; pi + 1 < poly_ext.size(); ++pi) {
                        double dx = poly_ext.getGeometry(pi+1).getX() - poly_ext.getGeometry(pi).getX();
                        double dy = poly_ext.getGeometry(pi+1).getY() - poly_ext.getGeometry(pi).getY();
                        double el = std::hypot(dx, dy);
                        if (el > longest_edge_len) {
                            longest_edge_len = el;
                            sweep_align_angle = std::atan2(dy, dx);
                        }
                    }
                    // 归一化到 (-π/2, π/2]，因为 swath 方向模 π 等价
                    while (sweep_align_angle > M_PI/2) sweep_align_angle -= M_PI;
                    while (sweep_align_angle <= -M_PI/2) sweep_align_angle += M_PI;

                    // 若主边已接近水平（< 10°），不旋转
                    if (std::abs(sweep_align_angle) < 10.0 * M_PI / 180.0)
                        sweep_align_angle = 0.0;
                }

                for (size_t ci = 0; ci < mid_hl.size(); ++ci) {
                    f2c::types::Cell work_cell = mid_hl.getGeometry(ci);
                    f2c::types::Cell grid_cell = (ci < cells.size()) ? cells.getGeometry(ci) : work_cell;
                    yingshi::DecomposerParams decomp_params;
                    decomp_params.use_sweep = use_sweep_decomp_;

                    if (std::abs(sweep_align_angle) > 0.001) {
                        // 旋转 -angle → sweep 分解 → 旋转 +angle 回来
                        RCLCPP_INFO(this->get_logger(),
                            "Phase 4A: Aligning sweep to polygon edge at %.1f°",
                            sweep_align_angle * 180.0 / M_PI);
                        auto rotated = rotateCell(work_cell, -sweep_align_angle);
                        auto rotated_grid = rotateCell(grid_cell, -sweep_align_angle);
                        decomp_params.use_sweep = true;
                        auto sub = yingshi::rectilinearDecompose(
                            rotated, rotated_grid, decomp_params);
                        for (size_t si = 0; si < sub.size(); ++si) {
                            auto back = rotateCell(sub.getGeometry(si), sweep_align_angle);
                            decomp_mid_hl.addGeometry(back);
                        }
                    } else {
                        auto sub = yingshi::rectilinearDecompose(
                            work_cell, grid_cell, decomp_params);
                        for (size_t si = 0; si < sub.size(); ++si)
                            decomp_mid_hl.addGeometry(sub.getGeometry(si));
                    }
                }
                size_t raw_decomp_count = decomp_mid_hl.size();
                RCLCPP_INFO(this->get_logger(),
                           "Decomposed mid_hl into %zu raw sub-cells", raw_decomp_count);

                // ── 逐 cell 几何诊断 ──
                {
                    RCLCPP_INFO(this->get_logger(),
                               "========== Cell Geometry Diagnostics (%zu cells) ==========",
                               decomp_mid_hl.size());
                    const auto& orig_cell = cells.getGeometry(0);  // 原始多边形（含孔洞）
                    for (size_t si = 0; si < decomp_mid_hl.size(); ++si) {
                        const auto& sc = decomp_mid_hl.getGeometry(si);
                        const auto& ring = sc.getExteriorRing();
                        double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
                        for (size_t pi = 0; pi + 1 < ring.size(); ++pi) {
                            double px = ring.getGeometry(pi).getX();
                            double py = ring.getGeometry(pi).getY();
                            if (px < min_x) min_x = px;
                            if (py < min_y) min_y = py;
                            if (px > max_x) max_x = px;
                            if (py > max_y) max_y = py;
                        }
                        double w = max_x - min_x;
                        double h = max_y - min_y;
                        double area = sc.area();
                        double min_dim = std::min(w, h);
                        double max_dim = std::max(w, h);
                        double ratio = (min_dim > 0.01) ? (max_dim / min_dim) : 999.0;
                        bool is_narrow = (min_dim < robot_width_);

                        // 检查是否与原始孔洞有交集
                        bool touches_hole = false;
                        for (size_t hi = 0; hi + 1 < orig_cell.size() && !touches_hole; ++hi) {
                            f2c::types::Cell hc;
                            hc.addRing(orig_cell.getInteriorRing(hi));
                            auto overlap = f2c::types::Cells::intersection(sc, hc);
                            if (overlap.size() > 0) {
                                double oa = overlap.getGeometry(0).area();
                                if (oa > 0.0001) touches_hole = true;
                            }
                        }

                        size_t n_pts = ring.size() - 1;  // 最后一点=第一点，4=矩形
                        bool is_rect = (n_pts == 4);  // 网格交集矩形恰好4个唯一点
                        RCLCPP_INFO(this->get_logger(),
                                   "  [%2zu] area=%7.2f m²  bbox=[%.1f,%.1f]-[%.1f,%.1f]  "
                                   "%4.1f×%4.1f m  ratio=%5.1f  pts=%zu  rings=%zu  %s%s%s",
                                   si, area,
                                   min_x, min_y, max_x, max_y,
                                   w, h, ratio, n_pts, sc.size(),
                                   is_narrow ? "⚠NARROW " : "",
                                   touches_hole ? "⚠TOUCHES_HOLE " : "",
                                   is_rect ? "" : "⚠NON-RECT");
                        // 非矩形 cell：打印全部顶点坐标
                        if (!is_rect) {
                            std::ostringstream vtx;
                            vtx << "    Vertices: ";
                            for (size_t pi = 0; pi + 1 < ring.size(); ++pi) {
                                if (pi > 0) vtx << " → ";
                                vtx << "(" << std::fixed << std::setprecision(1)
                                    << ring.getGeometry(pi).getX() << ","
                                    << ring.getGeometry(pi).getY() << ")";
                            }
                            RCLCPP_INFO(this->get_logger(), "%s", vtx.str().c_str());
                        }
                    }
                    RCLCPP_INFO(this->get_logger(),
                               "========== End Cell Diagnostics ==========");
                }

                // 立即发布分解轮廓，让 RViz 看到 cell 切分结果
                {
                    nav_msgs::msg::Path d_outline = createDecompositionOutlinePath(decomp_mid_hl);
                    RCLCPP_INFO(this->get_logger(),
                               "  Total sub-cells: %zu, Total points in path: %zu",
                               decomp_mid_hl.size(), d_outline.poses.size());
                    last_decomposition_outlines_[index] = d_outline;
                    decomposition_outline_pubs_[index]->publish(d_outline);
                }

                // 逐 cell 生成 no_hl — 窄 cell 用更小的 headland
                {
                    size_t adapted_count = 0;
                    for (size_t ci = 0; ci < decomp_mid_hl.size(); ++ci) {
                        const auto& mid_cell = decomp_mid_hl.getGeometry(ci);
                        double cell_perimeter = mid_cell.getExteriorRing().length();
                        double cell_area = mid_cell.area();
                        double est_width = (cell_perimeter > 1e-9) ? (2.0 * cell_area / cell_perimeter) : 0.0;
                        double cell_no_hl_ratio = no_hl_width_ratio_;
                        double required = r_w + 0.05;
                        double eroded_width = est_width - 2.0 * cell_no_hl_ratio * robot_width_;
                        if (eroded_width < required && est_width > 0.01) {
                            double max_erosion = std::max(0.0, (est_width - required) / 2.0);
                            cell_no_hl_ratio = std::max(0.0, max_erosion / robot_width_);
                            ++adapted_count;
                        }
                        f2c::types::Cells single;
                        single.addGeometry(mid_cell);
                        f2c::types::Cells eroded = const_hl.generateHeadlands(single, cell_no_hl_ratio * robot_width_);
                        for (size_t ei = 0; ei < eroded.size(); ++ei) {
                            no_hl.addGeometry(eroded.getGeometry(ei));
                        }
                    }
                    if (adapted_count > 0) {
                        RCLCPP_INFO(this->get_logger(),
                                   "Per-cell adaptive no_hl: %zu/%zu cells reduced",
                                   adapted_count, decomp_mid_hl.size());
                    }
                }
            } else {
                // —— W1 不分解模式 ——
                RCLCPP_INFO(this->get_logger(),
                           "Step 2: Decomposition disabled (W1), generating swaths directly on headland");
                no_hl = const_hl.generateHeadlands(mid_hl, effective_no_hl_ratio * robot_width_);
                no_hl = simplifyCells(no_hl, 5.0, 0.5);
            }

            // ── 优化步骤：过滤微小子区域 ──
            if (use_optimized_planner_ && filter_tiny_cells_) {
                double min_cell_area = min_cell_area_ratio_ * coverage_width_ * robot_width_;
                RCLCPP_INFO(this->get_logger(),
                           "Opt: Filtering tiny cells (min_area=%.3f m² = %.1f × %.2f × %.2f)...",
                           min_cell_area, min_cell_area_ratio_, coverage_width_, robot_width_);
                no_hl = filterTinyCells(no_hl, min_cell_area);
                RCLCPP_INFO(this->get_logger(),
                           "After tiny cell filter: %zu sub-cells remain", no_hl.size());
            }

            // ── Cell 合并：相邻且 swath 方向相近的 cell 合并为一个大 cell ──
            // 依据：Agarwal & Akella (2022) — 同向 cell 独立走增加无效掉头
            //       WUR 论文 (Peijnenburg 2024) — 分解碎片化降低覆盖率
            if (use_optimized_planner_ && no_hl.size() > 1) {
                const size_t nc = no_hl.size();

                // 1. 计算每个 cell 的 swath 主方向
                // sweep模式: 放宽合并角度阈值(60°)，斜边cell也不会被错分
                std::vector<double> cell_dirs(nc);
                for (size_t i = 0; i < nc; ++i) {
                    cell_dirs[i] = computeCellMainDirection(no_hl.getGeometry(i));
                }
                double merge_angle_threshold = use_sweep_decomp_ ? (60.0 * M_PI / 180.0) : (merge_angle_threshold_ * M_PI / 180.0);

                // 2. 构建邻接矩阵：两 cell 边界最近距离 < coverage_width 视为相邻
                std::vector<std::vector<bool>> adj(nc, std::vector<bool>(nc, false));
                for (size_t i = 0; i < nc; ++i) {
                    f2c::types::Cell cell_i = no_hl.getGeometry(i);
                    const auto& ring_i = cell_i.getExteriorRing();
                    for (size_t j = i + 1; j < nc; ++j) {
                        f2c::types::Cell cell_j = no_hl.getGeometry(j);
                        const auto& ring_j = cell_j.getExteriorRing();
                        double min_d = std::numeric_limits<double>::max();
                        for (size_t pi = 0; pi < ring_i.size(); ++pi) {
                            double ix = ring_i.getGeometry(pi).getX();
                            double iy = ring_i.getGeometry(pi).getY();
                            for (size_t pj = 0; pj < ring_j.size(); ++pj) {
                                double d = std::hypot(
                                    ix - ring_j.getGeometry(pj).getX(),
                                    iy - ring_j.getGeometry(pj).getY());
                                if (d < min_d) min_d = d;
                            }
                        }
                        adj[i][j] = adj[j][i] = (min_d < 2.0 * coverage_width_);
                    }
                }

                // 3. 贪心分组：邻接 + 方向差 < 30° → 同组
                std::vector<int> group(nc, -1);
                int next_gid = 0;
                for (size_t i = 0; i < nc; ++i) {
                    if (group[i] >= 0) continue;
                    group[i] = next_gid++;
                    for (size_t j = i + 1; j < nc; ++j) {
                        if (group[j] >= 0) continue;
                        if (!adj[i][j]) continue;
                        // 角度归一化到 [0, π)，比较方向差
                        double a = cell_dirs[i];
                        while (a < 0) a += M_PI;
                        while (a >= M_PI) a -= M_PI;
                        double b = cell_dirs[j];
                        while (b < 0) b += M_PI;
                        while (b >= M_PI) b -= M_PI;
                        double diff = std::abs(a - b);
                        if (diff > M_PI / 2) diff = M_PI - diff;
                        if (diff < merge_angle_threshold) {
                            // 孔洞隔离检查：孔洞两侧的 cell 不应合并
                            // 检查两 cell 质心连线是否穿越孔洞
                            if (use_sweep_decomp_ && cell.size() > 1) {
                                // 计算 cell i 质心
                                const auto& ring_i_cent = no_hl.getGeometry(i).getExteriorRing();
                                double ci_x = 0, ci_y = 0;
                                size_t ci_n = ring_i_cent.size() - 1;
                                for (size_t pi = 0; pi < ci_n; ++pi) {
                                    ci_x += ring_i_cent.getGeometry(pi).getX();
                                    ci_y += ring_i_cent.getGeometry(pi).getY();
                                }
                                ci_x /= ci_n; ci_y /= ci_n;

                                // 计算 cell j 质心
                                const auto& ring_j_cent = no_hl.getGeometry(j).getExteriorRing();
                                double cj_x = 0, cj_y = 0;
                                size_t cj_n = ring_j_cent.size() - 1;
                                for (size_t pj = 0; pj < cj_n; ++pj) {
                                    cj_x += ring_j_cent.getGeometry(pj).getX();
                                    cj_y += ring_j_cent.getGeometry(pj).getY();
                                }
                                cj_x /= cj_n; cj_y /= cj_n;

                                // 检查质心连线是否穿过任意孔洞
                                bool crosses_hole = false;
                                for (size_t hi = 0; hi + 1 < cell.size() && !crosses_hole; ++hi) {
                                    const auto& hr = cell.getInteriorRing(hi);
                                    // 线段-孔洞相交：检查质心线段是否与孔洞边相交
                                    for (size_t vi = 0; vi + 1 < hr.size() && !crosses_hole; ++vi) {
                                        double ax = hr.getGeometry(vi).getX();
                                        double ay = hr.getGeometry(vi).getY();
                                        double bx = hr.getGeometry(vi+1).getX();
                                        double by = hr.getGeometry(vi+1).getY();
                                        double dx = cj_x - ci_x, dy = cj_y - ci_y;
                                        double d = dx*(by-ay) - dy*(bx-ax);
                                        if (std::abs(d) < 1e-18) continue;
                                        double t = ((ax-ci_x)*(by-ay) - (ay-ci_y)*(bx-ax)) / d;
                                        double u = ((ax-ci_x)*dy - (ay-ci_y)*dx) / d;
                                        if (t > 0.001 && t < 0.999 && u > 0.0 && u < 1.0) {
                                            crosses_hole = true;
                                        }
                                    }
                                }
                                if (crosses_hole) {
                                    // ── Phase 2D: 斜边感知 — 区分真孔洞 vs 斜边界 ──
                                    // 两 cell 质心连线穿过孔洞边，但若 cell 边界实际贴近
                                    // （距离 < cov_width * 0.25），说明是斜边或共享边界误判，
                                    // 应允许合并（避免 sweep 分解的斜边末端三角 cell 被孤立）。
                                    // 孔洞顶点双向切割产生的小 cell 用更严阈值（0.25 vs 0.50），
                                    // 防止 X 向切割被吞并回全宽条带。
                                    double touch_dist = std::numeric_limits<double>::max();
                                    for (size_t pi2 = 0; pi2 + 1 < ring_i_cent.size(); ++pi2) {
                                        double ix = ring_i_cent.getGeometry(pi2).getX();
                                        double iy = ring_i_cent.getGeometry(pi2).getY();
                                        for (size_t pj2 = 0; pj2 + 1 < ring_j_cent.size(); ++pj2) {
                                            double d = std::hypot(
                                                ix - ring_j_cent.getGeometry(pj2).getX(),
                                                iy - ring_j_cent.getGeometry(pj2).getY());
                                            if (d < touch_dist) touch_dist = d;
                                        }
                                    }
                                    if (touch_dist < coverage_width_ * 0.25) {
                                        // 物理贴近（共享边界）→ 不是真孔洞分隔，允许合并
                                        RCLCPP_DEBUG(this->get_logger(),
                                            "  Cells %zu+%zu: centroid line crosses hole but "
                                            "cells touch (%.2f m < %.2f m) — allowing merge",
                                            i, j, touch_dist, coverage_width_ * 0.25);
                                    } else {
                                        continue;  // 真孔洞分隔，不合并
                                    }
                                }
                            }

                            group[j] = group[i];
                        }
                    }
                }

                // 4. 合并同组 cell（使用 GDAL Union）
                if (next_gid < static_cast<int>(nc)) {
                    f2c::types::Cells merged;
                    size_t merge_count = 0;
                    for (int g = 0; g < next_gid; ++g) {
                        f2c::types::Cell merged_cell;
                        bool first = true;
                        int group_size = 0;
                        for (size_t i = 0; i < nc; ++i) {
                            if (group[i] != g) continue;
                            ++group_size;
                            if (first) {
                                merged_cell = no_hl.getGeometry(i);
                                first = false;
                            } else {
                                f2c::types::Cells tmp(merged_cell);
                                auto result = tmp.unionOp(no_hl.getGeometry(i));
                                if (result.size() >= 1) {
                                    merged_cell = result.getGeometry(0);
                                    // MultiPolygon → 多余的作为独立 cell 加入
                                    for (size_t ri = 1; ri < result.size(); ++ri) {
                                        merged.addGeometry(result.getGeometry(ri));
                                    }
                                } else {
                                    RCLCPP_WARN(this->get_logger(),
                                               "UnionOp failed for cell %zu (group %d),"
                                               " keeping previous shape", i, g);
                                }
                            }
                        }
                        if (group_size > 1) merge_count += (group_size - 1);
                        merged.addGeometry(merged_cell);
                    }
                    RCLCPP_INFO(this->get_logger(),
                               "Cell merging: %zu cells → %zu cells "
                               "(%zu merged, angle threshold=%.0f°)",
                               nc, merged.size(), merge_count,
                               merge_angle_threshold * 180.0 / M_PI);
                    no_hl = merged;
                }
            }

            // ── 孔洞几何挖除：仅 W1 需要（网格分解 W2 已通过网格线自然避开）──
            if (!decomposition_enabled_) {
                for (size_t hi = 0; hi + 1 < cell.size(); ++hi) {
                    f2c::types::Cell hole_cell;
                    hole_cell.addRing(cell.getInteriorRing(hi));
                    double hm = mid_hl_width_ratio_ * robot_width_;
                    f2c::types::Cells hole_cells;
                    hole_cells.addGeometry(hole_cell);
                    hole_cells = hole_cells.buffer(hm);
                    no_hl = no_hl.difference(hole_cells);
                    RCLCPP_INFO(this->get_logger(),
                               "Hole %zu subtracted from no_hl (margin=%.2f m, %zu cells remain)",
                               hi, hm, no_hl.size());
                }
            }

            // 第四步：在 no_hl 上生成 swaths（全局优化 + 统一角度）
            f2c::sg::BruteForce swath_generator;
            swath_generator.setAllowOverlap(true);  // 边界缝隙处允许多铺一条swath
            f2c::types::SwathsByCells swaths_by_cells;
            {
                if (use_optimized_planner_ && swath_angle_optimization_) {
                    // ★ 全局优化：对所有 cell 测试同一角度，选总 swath 数最少的
                    std::vector<double> cands = swath_angle_list_;
                    // 收集所有 cell 的边缘角度（全局候选）
                    for (size_t ci = 0; ci < no_hl.size(); ++ci) {
                        auto edge_cands = extractAllEdgeAngles(no_hl.getGeometry(ci));
                        cands.insert(cands.end(), edge_cands.begin(), edge_cands.end());
                    }
                    // 去重
                    std::sort(cands.begin(), cands.end());
                    cands.erase(std::unique(cands.begin(), cands.end(),
                        [](double a, double b) { return std::abs(a-b) < 1e-4; }), cands.end());

                    size_t best_total = std::numeric_limits<size_t>::max();
                    double best_ang = 0.0;
                    std::vector<f2c::types::Swaths> best_cell_swaths;  // 缓存最优角度的swaths
                    RCLCPP_INFO(this->get_logger(),
                               "Global swath angle optimization: testing %zu candidates...",
                               cands.size());
                    for (double ang : cands) {
                        size_t total = 0;
                        std::vector<f2c::types::Swaths> cell_swaths;
                        cell_swaths.reserve(no_hl.size());
                        for (size_t ci = 0; ci < no_hl.size(); ++ci) {
                            auto cs = swath_generator.generateSwaths(ang, r_w,
                                no_hl.getGeometry(ci));
                            total += cs.size();
                            cell_swaths.push_back(std::move(cs));
                        }
                        RCLCPP_INFO(this->get_logger(),
                                   "  Angle %.1f deg: %zu swaths total",
                                   ang * 180.0 / M_PI, total);
                        if (total < best_total && total > 0) {
                            best_total = total;
                            best_ang = ang;
                            best_cell_swaths = std::move(cell_swaths);
                        }
                    }
                    RCLCPP_INFO(this->get_logger(),
                               "  Best global angle: %.1f deg with %zu total swaths",
                               best_ang * 180.0 / M_PI, best_total);

                    // 复用缓存的 swaths + 边界间隙补填
                    //   闭合硬边界：补的 swath 端点内缩，不撞墙
                    for (size_t ci = 0; ci < best_cell_swaths.size(); ++ci) {
                        auto& cell_swaths = best_cell_swaths[ci];
                        size_t rm = 0;
                        f2c::types::Swaths cs = filterShortSwaths(cell_swaths,
                            min_swath_length_, rm);
                        // 边界间隙填补：filterShortSwaths 之后补，避免填缝被 min_swath_length 误杀
                        size_t sz_before = cs.size();
                        fillBoundaryGaps(cs, no_hl.getGeometry(ci), cell, best_ang,
                            coverage_width_, swath_endpoint_shrink_distance_);
                        if (cs.size() > sz_before) {
                            RCLCPP_INFO(this->get_logger(),
                                "  Boundary fill: cell[%zu] +%zu swaths (%zu→%zu)",
                                ci, cs.size() - sz_before, sz_before, cs.size());
                        }
                        if (cs.size() > 0) swaths_by_cells.push_back(cs);
                    }
                } else {
                    for (size_t ci = 0; ci < no_hl.size(); ++ci) {
                        const auto& sub = no_hl.getGeometry(ci);
                        double ang = computeCellMainDirection(sub);

                        // ── Phase 2A: 斜边感知 swath 角度 ──
                        // sweep 分解产生水平条带，若 cell 贴着斜边界，
                        // 水平 swath 会在斜边末端形成三角缝隙。
                        // 此处检测斜边并覆盖 swath 角度，使 swath 平行于斜边。
                        if (use_sweep_decomp_ && cell.size() > 0) {
                            double slant_ang = detectSlantedBoundaryAngle(
                                sub, cell, ang, coverage_width_);
                            if (std::abs(slant_ang - ang) > 0.05) {  // 角度差异>3°才覆盖
                                ang = slant_ang;
                            }
                        }

                        f2c::types::Swaths cs;
                        if (swath_angle_optimization_) {
                            auto cands = extractAllEdgeAngles(sub);
                            for (double a : swath_angle_list_) cands.push_back(a);
                            cands.push_back(ang);
                            cs = optimizeSwathAngle(sub, swath_generator, r_w, cands);
                        } else {
                            cs = swath_generator.generateSwaths(ang, r_w, sub);
                        }
                        size_t rm = 0;
                        cs = filterShortSwaths(cs, min_swath_length_, rm);
                        size_t sz_before = cs.size();
                        fillBoundaryGaps(
                            cs, sub, cell, ang, coverage_width_,
                            swath_endpoint_shrink_distance_);
                        if (cs.size() > sz_before) {
                            RCLCPP_INFO(this->get_logger(),
                                "  Boundary fill: cell[%zu] +%zu swaths (%zu→%zu)",
                                ci, cs.size() - sz_before, sz_before, cs.size());
                        }
                        if (cs.size() > 0) swaths_by_cells.push_back(cs);
                    }
                }
                if (swaths_by_cells.sizeTotal() == 0) {
                    RCLCPP_WARN(this->get_logger(), "No swaths; skip polygon_%d", polygon_id);
                    this->clearPlanningCacheForPolygon(index, true);
                    return;
                }

                const size_t pruned_seam_fills =
                    yingshi::pruneRedundantCellSeamFills(
                        swaths_by_cells, no_hl, cell, coverage_width_);
                if (pruned_seam_fills > 0) {
                    RCLCPP_INFO(this->get_logger(),
                        "Redundant Cell seam fills removed: %zu",
                        pruned_seam_fills);
                }
            }

            // ── 构建孔洞环（供裁剪和贪心排序共用）──
            std::vector<f2c::types::LinearRing> hole_rings;
            if (!last_holes_[index].empty()) {
                for (const auto& hole_msg : last_holes_[index]) {
                    auto hr = makeClosedF2CRing(hole_msg);
                    if (hr.size() >= 4) {
                        hole_rings.push_back(hr);
                    }
                }
            }

            // ── 孔洞裁剪：切割穿越孔洞的 swath ──
            // sweep 分解产生的 cell 可能因 GDAL 精度问题或合并逻辑
            // 仍包含横穿孔洞的 swath。此处对每个 swath 做线段-孔洞相交检测，
            // 若穿越则沿孔洞边界切割为两段，从源头消除 genRoute 跨孔洞连接。
            if (!hole_rings.empty()) {
                // 点是否在孔洞内（射线法 → yingshi::pointInAnyHole）
                auto pointInAnyHole = [&](double px, double py) -> bool {
                    return yingshi::pointInAnyHole(px, py, hole_rings);
                };

                // 线段是否穿越孔洞（采样检测 → yingshi::segmentCrossesHole）
                auto segmentCrossesHole = [&](double x0, double y0,
                                              double x1, double y1) -> bool {
                    return yingshi::segmentCrossesHole(x0, y0, x1, y1, hole_rings, 50);
                };

                // 线段与孔洞边界的所有交点（按距起点的距离排序）
                auto findHoleIntersections = [&](double sx, double sy,
                                                 double ex, double ey)
                    -> std::vector<double>
                {
                    std::vector<double> t_params;  // 0~1 参数
                    double seg_len = std::hypot(ex - sx, ey - sy);
                    if (seg_len < 1e-12) return t_params;

                    for (const auto& hr : hole_rings) {
                        size_t n = hr.size();
                        for (size_t vi = 0; vi + 1 < n; ++vi) {
                            double ax = hr.getGeometry(vi).getX();
                            double ay = hr.getGeometry(vi).getY();
                            double bx = hr.getGeometry(vi + 1).getX();
                            double by = hr.getGeometry(vi + 1).getY();

                            // 两线段交点参数 (2D)
                            double d = (ex - sx)*(by - ay) - (ey - sy)*(bx - ax);
                            if (std::abs(d) < 1e-18) continue;  // 平行
                            double t = ((ax - sx)*(by - ay) - (ay - sy)*(bx - ax)) / d;
                            double u = ((ax - sx)*(ey - sy) - (ay - sy)*(ex - sx)) / d;
                            if (t > 0.0001 && t < 0.9999 && u > 0.0 && u < 1.0) {
                                t_params.push_back(t);
                            }
                        }
                    }
                    std::sort(t_params.begin(), t_params.end());
                    // 去重（容差 0.001）
                    std::vector<double> deduped;
                    for (double t : t_params) {
                        if (deduped.empty() || t - deduped.back() > 0.001)
                            deduped.push_back(t);
                    }
                    return deduped;
                };

                size_t clipped_count = 0;
                for (size_t ci = 0; ci < swaths_by_cells.size(); ++ci) {
                    f2c::types::Swaths cell_swaths = swaths_by_cells.at(ci);
                    f2c::types::Swaths clipped;
                    for (size_t si = 0; si < cell_swaths.size(); ++si) {
                        const auto& sw = cell_swaths.at(si);
                        double sx = sw.startPoint().getX();
                        double sy = sw.startPoint().getY();
                        double ex = sw.endPoint().getX();
                        double ey = sw.endPoint().getY();

                        if (!segmentCrossesHole(sx, sy, ex, ey)) {
                            clipped.push_back(sw);  // 未穿洞，保留
                            continue;
                        }

                        // 穿洞：找交点并切割
                        auto ts = findHoleIntersections(sx, sy, ex, ey);
                        if (ts.empty()) {
                            // 采样检测到但精确求交失败 → 保守处理，保留
                            clipped.push_back(sw);
                            continue;
                        }

                        // 生成非孔洞子段：从 t=0 开始，每对交点 (奇进偶出) 之间跳过
                        double prev_t = 0.0;
                        // 在每对交点之间取中点判断内外
                        for (size_t ti = 0; ti < ts.size(); ++ti) {
                            double t_mid = (prev_t + ts[ti]) * 0.5;
                            double mx = sx + t_mid * (ex - sx);
                            double my = sy + t_mid * (ey - sy);
                            bool mid_outside = !pointInAnyHole(mx, my);

                            if (mid_outside && prev_t < ts[ti] - 0.0001) {
                                // 这段在孔洞外，保留
                                f2c::types::Swath new_sw = sw.clone();
                                double nsx = sx + prev_t * (ex - sx);
                                double nsy = sy + prev_t * (ey - sy);
                                double nex = sx + ts[ti] * (ex - sx);
                                double ney = sy + ts[ti] * (ey - sy);
                                // 只保留长度足够的段
                                if (std::hypot(nex - nsx, ney - nsy) >=
                                    min_swath_length_ * 0.5) {
                                    f2c::types::LineString new_path;
                                    new_path.addPoint(
                                        f2c::types::Point(nsx, nsy));
                                    new_path.addPoint(
                                        f2c::types::Point(nex, ney));
                                    new_sw.setPath(new_path);
                                    clipped.push_back(new_sw);
                                }
                            }
                            prev_t = ts[ti];
                        }
                        // 最后一段 (最后一个交点 → end)
                        if (prev_t < 0.9999) {
                            double t_mid = (prev_t + 1.0) * 0.5;
                            double mx = sx + t_mid * (ex - sx);
                            double my = sy + t_mid * (ey - sy);
                            if (!pointInAnyHole(mx, my)) {
                                f2c::types::Swath new_sw = sw.clone();
                                double nsx = sx + prev_t * (ex - sx);
                                double nsy = sy + prev_t * (ey - sy);
                                f2c::types::LineString new_path2;
                                new_path2.addPoint(
                                    f2c::types::Point(nsx, nsy));
                                new_path2.addPoint(
                                    f2c::types::Point(ex, ey));
                                new_sw.setPath(new_path2);
                                if (std::hypot(ex - nsx, ey - nsy) >=
                                    min_swath_length_ * 0.5) {
                                    clipped.push_back(new_sw);
                                }
                            }
                        }
                        ++clipped_count;
                    }
                    swaths_by_cells[ci] = clipped;
                }
                if (clipped_count > 0) {
                    RCLCPP_INFO(this->get_logger(),
                               "Hole clipping: %zu swaths split to avoid crossing holes",
                               clipped_count);
                }
            }

            // cell_order: 遍历顺序 → 原始 no_hl 索引（贪心排序会更新）
            std::vector<size_t> cell_order(swaths_by_cells.size());
            for (size_t i = 0; i < swaths_by_cells.size(); ++i) cell_order[i] = i;

            // 更新 decomposition 可视化
            {
                nav_msgs::msg::Path d_outline = createDecompositionOutlinePath(no_hl);
                last_decomposition_outlines_[index] = d_outline;
                decomposition_outline_pubs_[index]->publish(d_outline);
            }

            // ── Swath 排序 + Cell/入口联合优化 ──
            // 无孔洞时按真实出口动态贪心；有孔洞时保持安全极角顺序并
            // 联合优化整条 Cell 链，同时保留 F2C 的规则内部覆盖顺序。
            if (swath_order_type_ != "none") {
                yingshi::greedyCellOrder(swaths_by_cells, cell_order, hole_rings,
                                         swath_order_type_);
                {
                    std::ostringstream order_str;
                    for (size_t i = 0; i < cell_order.size(); ++i) {
                        if (i > 0) order_str << " → ";
                        order_str << "c" << cell_order[i];
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "Cell traversal order (greedy): %s", order_str.str().c_str());
                }
            }

            // 第五步：生成路由
            // redirect_swaths=false 保留贪心排序选定的 swath 方向，
            // 避免 OR-Tools 翻转已精心选定的 Cell 内路径。
            f2c::types::Route route;

            // ── Phase 2E: Route 生成 ──
            if (swath_order_type_ == "snake") {
                // Snake 模式：TSP-free 直连，避免跨孔洞连接
                RCLCPP_INFO(this->get_logger(),
                    "Step 5: Snake safe path — greedy cell order, direct connections");

                route.addConnection(f2c::types::MultiPoint());

                for (size_t ci = 0; ci < swaths_by_cells.size(); ++ci) {
                    if (swaths_by_cells.at(ci).size() == 0) {
                        RCLCPP_WARN(this->get_logger(),
                            "  Snake: skipping empty cell %zu", ci);
                        continue;
                    }

                    route.addSwaths(swaths_by_cells.at(ci));

                    f2c::types::MultiPoint conn;
                    size_t next_ci = ci + 1;
                    while (next_ci < swaths_by_cells.size() &&
                           swaths_by_cells.at(next_ci).size() == 0) {
                        ++next_ci;
                    }
                    if (next_ci < swaths_by_cells.size()) {
                        const auto& last_sw = swaths_by_cells.at(ci).back();
                        const auto& next_sw = swaths_by_cells.at(next_ci).at(0);
                        double lx = last_sw.endPoint().getX();
                        double ly = last_sw.endPoint().getY();
                        double nx = next_sw.startPoint().getX();
                        double ny = next_sw.startPoint().getY();

                        if (mid_hl.size() > 0) {
                            conn.addPoint(lx, ly);
                            const auto& hl_ring = mid_hl.getGeometry(0).getExteriorRing();
                            if (hl_ring.size() > 2) {
                                double hlx = 0, hly = 0;
                                for (size_t hi = 0; hi + 1 < hl_ring.size(); ++hi) {
                                    hlx += hl_ring.getGeometry(hi).getX();
                                    hly += hl_ring.getGeometry(hi).getY();
                                }
                                hlx /= (hl_ring.size() - 1);
                                hly /= (hl_ring.size() - 1);
                                double mx = (lx + nx) * 0.5, my = (ly + ny) * 0.5;
                                double dx2 = mx - hlx, dy2 = my - hly;
                                double dlen = std::hypot(dx2, dy2);
                                if (dlen > 0.01) {
                                    conn.addPoint(hlx + dx2 * 0.3, hly + dy2 * 0.3);
                                }
                            }
                            conn.addPoint(nx, ny);
                        } else {
                            conn.addPoint(lx, ly);
                            conn.addPoint(nx, ny);
                        }
                    }
                    route.addConnection(conn);
                }

                RCLCPP_INFO(this->get_logger(),
                    "Snake route: %zu swath groups, %zu connections (TSP-free)",
                    route.sizeVectorSwaths(), route.sizeConnections());
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "Step 5: Planning route (headland-routed)...");
                bool show_solver_log = use_optimized_planner_ && ortools_exact_solve_;
                f2c::rp::RoutePlannerBase route_planner;
                route = route_planner.genRoute(
                    mid_hl, swaths_by_cells, show_solver_log);
                RCLCPP_INFO(this->get_logger(),
                    "Generated route with %zu swath groups",
                    route.sizeVectorSwaths());
            }

            // ── 孔洞感知：修复 genRoute 连接穿洞 ──
            // genRoute 的 headland 最短路径中，简化逻辑可能产生穿洞连接。
            // 此处检测穿洞的 connection 段，沿孔洞边界插入强制绕行路点。
            if (!last_holes_[index].empty()) {
                // 构建孔洞环（LinearRing，用于射线法 + 交点计算）
                std::vector<f2c::types::LinearRing> hole_rings;
                for (const auto& hole_msg : last_holes_[index]) {
                    auto hr = makeClosedF2CRing(hole_msg);
                    if (hr.size() >= 4) {
                        hole_rings.push_back(hr);
                    }
                }

                if (!hole_rings.empty()) {
                    // 采样法：线段是否穿越孔洞 (→ yingshi::segmentCrossesHole)
                    auto segXHole = [&](double x0, double y0,
                                        double x1, double y1) -> bool {
                        return yingshi::segmentCrossesHole(x0, y0, x1, y1, hole_rings, 50);
                    };

                    // 交点信息：线段参数 + 孔洞索引 + 边索引 + 实际坐标
                    struct HoleX {
                        double t;
                        size_t hole_idx;
                        size_t edge_idx;  // 孔洞环中该边的起点顶点索引
                        f2c::types::Point pt;
                    };

                    // 线段与所有孔洞边界的交点（按 t 排序）
                    auto findX = [&](double sx, double sy, double ex, double ey)
                        -> std::vector<HoleX> {
                        std::vector<HoleX> result;
                        double seg_len = std::hypot(ex - sx, ey - sy);
                        if (seg_len < 1e-12) return result;
                        for (size_t hi = 0; hi < hole_rings.size(); ++hi) {
                            const auto& hr = hole_rings[hi];
                            size_t n = hr.size();
                            for (size_t vi = 0; vi + 1 < n; ++vi) {
                                double ax = hr.getGeometry(vi).getX();
                                double ay = hr.getGeometry(vi).getY();
                                double bx = hr.getGeometry(vi + 1).getX();
                                double by = hr.getGeometry(vi + 1).getY();
                                double d = (ex - sx)*(by - ay) - (ey - sy)*(bx - ax);
                                if (std::abs(d) < 1e-18) continue;
                                double t = ((ax - sx)*(by - ay) - (ay - sy)*(bx - ax)) / d;
                                double u = ((ax - sx)*(ey - sy) - (ay - sy)*(ex - sx)) / d;
                                if (t > 0.0001 && t < 0.9999 && u >= 0.0 && u <= 1.0) {
                                    result.push_back({t, hi, vi,
                                        f2c::types::Point(sx + t*(ex - sx),
                                                          sy + t*(ey - sy))});
                                }
                            }
                        }
                        std::sort(result.begin(), result.end(),
                            [](const HoleX& a, const HoleX& b) { return a.t < b.t; });
                        return result;
                    };

                    // 沿孔洞边界从 entry 走到 exit，选较短方向，返回边界顶点序列
                    auto walkBoundary = [&](const HoleX& entry, const HoleX& exit)
                        -> std::vector<f2c::types::Point> {
                        // 同一边上 → 直接连接
                        if (entry.hole_idx != exit.hole_idx ||
                            entry.edge_idx == exit.edge_idx) {
                            return {entry.pt, exit.pt};
                        }

                        const auto& hr = hole_rings[entry.hole_idx];
                        size_t n_edges = hr.size() - 1;  // 闭合环，最后顶点=第一顶点
                        auto addVtx = [&](std::vector<f2c::types::Point>& path, size_t idx) {
                            path.push_back(f2c::types::Point(
                                hr.getGeometry(idx).getX(),
                                hr.getGeometry(idx).getY()));
                        };

                        // 前进方向：entry → v[entry.edge_idx+1] → ... → v[exit.edge_idx] → exit
                        std::vector<f2c::types::Point> fwd;
                        fwd.push_back(entry.pt);
                        size_t cur = (entry.edge_idx + 1) % n_edges;
                        while (true) {
                            addVtx(fwd, cur);
                            if (cur == exit.edge_idx) break;
                            cur = (cur + 1) % n_edges;
                        }
                        fwd.push_back(exit.pt);

                        // 后退方向：entry → v[entry.edge_idx] → ... → v[exit.edge_idx+1] → exit
                        std::vector<f2c::types::Point> bwd;
                        bwd.push_back(entry.pt);
                        cur = entry.edge_idx;
                        while (true) {
                            addVtx(bwd, cur);
                            if (cur == ((exit.edge_idx + 1) % n_edges)) break;
                            cur = (cur + n_edges - 1) % n_edges;
                        }
                        bwd.push_back(exit.pt);

                        auto pathLen = [](const std::vector<f2c::types::Point>& pts) {
                            double len = 0;
                            for (size_t i = 1; i < pts.size(); ++i)
                                len += std::hypot(pts[i].getX() - pts[i-1].getX(),
                                                  pts[i].getY() - pts[i-1].getY());
                            return len;
                        };
                        return pathLen(fwd) <= pathLen(bwd) ? fwd : bwd;
                    };

                    size_t fixed_conns = 0;
                    for (size_t ci = 0; ci < route.sizeConnections(); ++ci) {
                        auto& conn = route.getConnection(ci);
                        if (conn.size() < 2) continue;

                        // 检测是否有穿洞段
                        bool has_crossing = false;
                        for (size_t pi = 0; pi + 1 < conn.size(); ++pi) {
                            double x0 = conn.getGeometry(pi).getX();
                            double y0 = conn.getGeometry(pi).getY();
                            double x1 = conn.getGeometry(pi + 1).getX();
                            double y1 = conn.getGeometry(pi + 1).getY();
                            if (segXHole(x0, y0, x1, y1)) {
                                has_crossing = true;
                                break;
                            }
                        }
                        if (!has_crossing) continue;

                        // 重建连接：逐段检测，穿洞段插入孔洞边界绕行路点
                        f2c::types::MultiPoint new_conn;
                        for (size_t pi = 0; pi < conn.size(); ++pi) {
                            double cx = conn.getGeometry(pi).getX();
                            double cy = conn.getGeometry(pi).getY();
                            if (new_conn.size() == 0) {
                                new_conn.addPoint(cx, cy);
                                continue;
                            }
                            size_t li = new_conn.size() - 1;
                            double px = new_conn.getGeometry(li).getX();
                            double py = new_conn.getGeometry(li).getY();

                            if (segXHole(px, py, cx, cy)) {
                                auto hits = findX(px, py, cx, cy);
                                // 逐对 (入口, 出口) 插入绕行路点
                                for (size_t hi = 0; hi + 1 < hits.size(); hi += 2) {
                                    auto detour = walkBoundary(hits[hi], hits[hi + 1]);
                                    for (const auto& dp : detour) {
                                        new_conn.addPoint(dp.getX(), dp.getY());
                                    }
                                }
                                // 奇数交点兜底：线段起/终点落入孔洞，用中点偏移
                                if (hits.size() < 2 || hits.size() % 2 != 0) {
                                    double mx = (px + cx) * 0.5, my = (py + cy) * 0.5;
                                    // 找最近孔洞中心
                                    double best_hcx = 0, best_hcy = 0, best_d = 1e9;
                                    for (size_t hi2 = 0; hi2 < hole_rings.size(); ++hi2) {
                                        const auto& hr = hole_rings[hi2];
                                        double sx = 0, sy = 0;
                                        for (size_t vi = 0; vi + 1 < hr.size(); ++vi) {
                                            sx += hr.getGeometry(vi).getX();
                                            sy += hr.getGeometry(vi).getY();
                                        }
                                        sx /= (hr.size() - 1); sy /= (hr.size() - 1);
                                        double d = std::hypot(mx - sx, my - sy);
                                        if (d < best_d) { best_d = d; best_hcx = sx; best_hcy = sy; }
                                    }
                                    double odx = mx - best_hcx, ody = my - best_hcy;
                                    double olen = std::hypot(odx, ody);
                                    if (olen < 1e-9) { odx = 0; ody = 1.0; olen = 1.0; }
                                    odx /= olen; ody /= olen;
                                    double detour_margin = robot_width_ * 0.8;
                                    new_conn.addPoint(mx + odx * detour_margin, my + ody * detour_margin);
                                }
                            }
                            new_conn.addPoint(cx, cy);
                        }
                        route.setConnection(ci, new_conn);
                        ++fixed_conns;
                    }

                    if (fixed_conns > 0) {
                        RCLCPP_INFO(this->get_logger(),
                            "Hole-aware route fix: %zu connections patched",
                            fixed_conns);
                    }
                }
            }

            f2c::types::Swaths swaths;
            f2c::types::Path path;
            std::string used_planner;
            geometry_msgs::msg::PoseArray swath_path_points;
            geometry_msgs::msg::PoseArray dubins_path_points;

            // ── 边界覆盖策略：统一闭合/开放边界处理 ──
            // 根据 boundary_type 确定 swath 端点调整量
            //   "closed" → 内缩（安全距离，防撞墙）
            //   "open"   → 外伸（侵入headland，换覆盖率）
            //   "custom" → 使用用户指定的 boundary_coverage_margin
            double effective_margin = boundary_coverage_margin_;
            if (boundary_type_ == "closed") {
                // 闭合硬边界：使用 swath_endpoint_shrink_distance 的传统安全值
                effective_margin = swath_endpoint_shrink_distance_;
                if (effective_margin <= 0.0) effective_margin = 0.3;  // 保底值
            } else if (boundary_type_ == "open") {
                // 开放软边界：向外延伸，值为负（adjustSwathEndpoints 负值=延伸）
                effective_margin = boundary_coverage_margin_;
                if (effective_margin >= 0.0) effective_margin = boundary_open_default_margin_;  // 保底值（ROS 参数可调）
            }
            // "custom": 直接使用 boundary_coverage_margin_，不做改写

            if (effective_margin != 0.0) {
                const char* direction = effective_margin > 0.0 ? "shrink inward (closed boundary safety)" :
                                        (effective_margin < 0.0 ? "extend outward (open boundary coverage)" :
                                         "no adjustment");
                RCLCPP_INFO(this->get_logger(),
                           "Boundary strategy [%s]: adjusting swath endpoints by %.3f m (%s)...",
                           boundary_type_.c_str(), effective_margin, direction);

                // 每个端点独立判断外环/孔洞净空，避免 swath 中点靠近孔洞时
                // 把远离孔洞、原本可贴近外边界的两个端点一起缩短。
                const auto& outer_ring = cell.getExteriorRing();
                std::vector<f2c::types::LinearRing> hole_rings;
                for (size_t hi = 0; hi + 1 < cell.size(); ++hi) {
                    hole_rings.push_back(cell.getInteriorRing(hi));
                }
                for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
                    f2c::types::Swaths& route_swaths = route.getSwaths(i);
                    f2c::types::Swaths adjusted_swaths;
                    for (size_t j = 0; j < route_swaths.size(); ++j) {
                        const auto& swath = route_swaths.at(j);
                        adjusted_swaths.push_back(
                            yingshi::adjustSwathEndpointsForBoundaryClearance(
                                swath, outer_ring, hole_rings,
                                coverage_width_, effective_margin));
                    }

                    route.setSwaths(i, adjusted_swaths);
                }

                RCLCPP_INFO(this->get_logger(),
                           "Swath endpoints adjusted: %zu swath groups processed",
                           route.sizeVectorSwaths());
            } else {
                RCLCPP_INFO(this->get_logger(),
                           "Boundary adjustment disabled (margin=%.3f m)",
                           effective_margin);
            }

            // genRoute 早于端点缩进，connection 仍指向旧 swath 端点。
            // 先同步首尾点，避免路径探到旧边界后立即折返形成短毛刺。
            const size_t synchronized_connections =
                yingshi::synchronizeRouteConnectionEndpoints(
                    route, 2.0 * std::abs(effective_margin));
            if (synchronized_connections > 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Route endpoint synchronization: %zu connections updated",
                    synchronized_connections);
            }

            // 所有 Route 变更完成后统一修复孔洞连接。
            // 1 mm 仅用于让中心线离开几何边界，机器人外形净空由上游 headland 保证。
            const size_t final_repaired_connections =
                yingshi::repairRouteConnectionsAroundHoles(
                    route, hole_rings, 0.001);
            if (final_repaired_connections > 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Final hole-aware route repair: %zu connections patched",
                    final_repaired_connections);
            }

            // 从路由中提取所有 swaths 用于可视化（按路由顺序）
            swaths = f2c::types::Swaths();
            for (size_t i = 0; i < route.sizeVectorSwaths(); ++i) {
                const auto& route_swaths = route.getSwaths(i);
                swaths.append(route_swaths);
            }
            RCLCPP_INFO(this->get_logger(), "Total swaths in route: %zu", swaths.size());

            // 创建并发布航向线端点
            geometry_msgs::msg::PoseArray swath_points;
            swath_points.header.frame_id = "map";
            swath_points.header.stamp = this->now();
            
            for (const auto& swath : swaths) {
                geometry_msgs::msg::Pose start_pose, end_pose;
                
                // 起点
                start_pose.position.x = swath.startPoint().getX();
                start_pose.position.y = swath.startPoint().getY();
                start_pose.position.z = 0.0;
                
                // 终点
                end_pose.position.x = swath.endPoint().getX();
                end_pose.position.y = swath.endPoint().getY();
                end_pose.position.z = 0.0;
                
                // 设置朝向（使用航向线方向）
                double dx = end_pose.position.x - start_pose.position.x;
                double dy = end_pose.position.y - start_pose.position.y;
                double yaw = std::atan2(dy, dx);
                
                start_pose.orientation.w = std::cos(yaw / 2);
                start_pose.orientation.z = std::sin(yaw / 2);
                end_pose.orientation = start_pose.orientation;
                
                swath_points.poses.push_back(start_pose);
                swath_points.poses.push_back(end_pose);
            }

            // 保存并发布航向线端点
            last_swath_points_[index] = swath_points;
            swath_points_pubs_[index]->publish(swath_points);

            // 创建采样路径点消息（航向线上的点）
            swath_path_points = geometry_msgs::msg::PoseArray();
            swath_path_points.header.frame_id = "map";
            swath_path_points.header.stamp = this->now();

            // 创建采样路径点消息（Dubins连接部分的点）
            dubins_path_points = geometry_msgs::msg::PoseArray();
            dubins_path_points.header.frame_id = "map";
            dubins_path_points.header.stamp = this->now();

            // 首先处理航向线上的点
            for (const auto& swath : swaths) {
                double dx = swath.endPoint().getX() - swath.startPoint().getX();
                double dy = swath.endPoint().getY() - swath.startPoint().getY();
                double length = std::sqrt(dx * dx + dy * dy);
                double yaw = std::atan2(dy, dx);

                // 在航向线上生成均匀分布的路径点
                int num_points = std::max(2, static_cast<int>(length / path_resolution_));
                for (int i = 0; i < num_points; ++i) {
                    geometry_msgs::msg::Pose pose;
                    double ratio = static_cast<double>(i) / (num_points - 1);
                    
                    // 线性插值计算位置
                    pose.position.x = swath.startPoint().getX() + ratio * dx;
                    pose.position.y = swath.startPoint().getY() + ratio * dy;
                    pose.position.z = 0.0;
                    
                    // 设置朝向
                    pose.orientation.w = std::cos(yaw / 2);
                    pose.orientation.x = 0.0;
                    pose.orientation.y = 0.0;
                    pose.orientation.z = std::sin(yaw / 2);
                    
                    swath_path_points.poses.push_back(pose);
                }
            }

            // 使用掉头曲线进行路径规划（基于路由）
            // "direct"：严格保留 Route 折线控制点，不做运动学曲线拟合
            // "dubins"：标准 Dubins（仅前进）
            // "dubins_cc"：连续曲率 Dubins + 环路检测（JFR 2025, F2C #150）
            // "reeds_shepp"：Reeds-Shepp（前进+后退）
            path = f2c::types::Path();
            used_planner.clear();

            if (turn_planner_type_ == "dubins_cc") {
                f2c::pp::PathPlanning pp;
                f2c::pp::DubinsCurvesCC dcc;
                path = pp.planPath(robot, route, dcc);
                used_planner = "Dubins CC (continuous curv + loop detect)";
            } else if (turn_planner_type_ == "direct") {
                path = yingshi::planDirectPath(route, robot.getCruiseVel());
                used_planner = "Direct (route polyline preserved)";

            } else if (turn_planner_type_ == "reeds_shepp") {
                f2c::pp::PathPlanning pp;
                f2c::pp::ReedsSheppCurves rs;
                path = pp.planPath(robot, route, rs);
                used_planner = "Reeds-Shepp (forward + reverse)";
            } else {  // "dubins"
                f2c::pp::PathPlanning pp;
                f2c::pp::DubinsCurves dc;
                path = pp.planPath(robot, route, dc);
                used_planner = "Dubins (forward only)";
            }

            RCLCPP_INFO(this->get_logger(), "Turn planner: %s", used_planner.c_str());

            // ── 优化步骤：RDP 路径简化 ──
            //   保存原始路径用于覆盖率计算（简化版会漏判覆盖）
            f2c::types::Path path_pre_rdp = path;
            // direct 的 Route 控制点承担绕障约束，禁止再用 RDP 拉成跨区弦线。
            if (use_optimized_planner_ && path_simplify_enabled_ &&
                turn_planner_type_ != "direct") {
                auto simplified_states = simplifyPathRDP(path, path_simplify_tolerance_);
                path.setStates(simplified_states);
            }

            // ── 方案 A：后置孔洞感知路径修复（中点偏移绕行） ──
            if (!last_holes_[index].empty()) {
                std::vector<f2c::types::LinearRing> hole_rings;
                std::vector<std::pair<double, double>> hole_centers;  // 孔洞中心
                for (const auto& hole_msg : last_holes_[index]) {
                    double sum_x = 0, sum_y = 0;
                    for (const auto& p : hole_msg.points) {
                        sum_x += p.x; sum_y += p.y;
                    }
                    auto hr = makeClosedF2CRing(hole_msg);
                    if (hr.size() >= 4 && !hole_msg.points.empty()) {
                        hole_rings.push_back(hr);
                        hole_centers.push_back({sum_x / hole_msg.points.size(),
                                                sum_y / hole_msg.points.size()});
                    }
                }

                if (!hole_rings.empty()) {
                    // 采样法：线段是否穿越孔洞 (→ yingshi::segmentCrossesHole)
                    auto segXHole = [&](double x0, double y0,
                                        double x1, double y1) -> bool {
                        return yingshi::segmentCrossesHole(x0, y0, x1, y1, hole_rings, 50);
                    };

                    // 检测简化后路径是否有穿洞段
                    bool path_has_crossing = false;
                    for (size_t pi = 0; pi + 1 < path.size(); ++pi) {
                        if (segXHole(path[pi].point.getX(), path[pi].point.getY(),
                                     path[pi+1].point.getX(), path[pi+1].point.getY())) {
                            path_has_crossing = true;
                            break;
                        }
                    }

                    if (path_has_crossing) {
                        // ── 方案 A：中点偏移绕行 ──
                        // 对每段穿洞路径，在线段中点插入一个向远离孔洞方向偏移的路点
                        std::vector<f2c::types::PathState> new_states;
                        for (size_t pi = 0; pi < path.size(); ++pi) {
                            if (new_states.empty()) {
                                new_states.push_back(path[pi]);
                                continue;
                            }
                            size_t li = new_states.size() - 1;
                            double px = new_states[li].point.getX();
                            double py = new_states[li].point.getY();
                            double cx = path[pi].point.getX();
                            double cy = path[pi].point.getY();

                            if (segXHole(px, py, cx, cy)) {
                                // 在中点插入向远离孔洞方向的偏移路点
                                double mx = (px + cx) * 0.5, my = (py + cy) * 0.5;
                                double best_hcx = 0, best_hcy = 0, best_d = 1e9;
                                for (auto& hc : hole_centers) {
                                    double d = std::hypot(mx - hc.first, my - hc.second);
                                    if (d < best_d) { best_d = d; best_hcx = hc.first; best_hcy = hc.second; }
                                }
                                double odx = mx - best_hcx, ody = my - best_hcy;
                                double olen = std::hypot(odx, ody);
                                if (olen < 1e-9) { odx = 0; ody = 1.0; olen = 1.0; }
                                odx /= olen; ody /= olen;
                                double detour_margin = robot_width_ * 0.8;
                                f2c::types::PathState ps;
                                ps.point = f2c::types::Point(mx + odx * detour_margin, my + ody * detour_margin);
                                new_states.push_back(ps);
                                geometry_msgs::msg::Pose dpp;
                                dpp.position.x = mx + odx * detour_margin;
                                dpp.position.y = my + ody * detour_margin;
                                dpp.position.z = 0.0; dpp.orientation.w = 1.0;
                                swath_path_points.poses.push_back(dpp);
                            }
                            new_states.push_back(path[pi]);
                        }

                        path.setStates(new_states);
                        RCLCPP_INFO(this->get_logger(),
                            "Post-planPath hole fix: simplified path %zu pts (midpoint offset)",
                            path.size());
                    }
                }
            }

            // ── 掉头可行性检查（仅 dubins/reeds_shepp 模式） ──
            // direct 模式下无转弯曲线，跳过
            struct TurnFeasibilityResult {
                int total_turns = 0;
                int infeasible_turns = 0;
                double worst_curvature = 0.0;
                std::vector<std::pair<int, double>> bad_segments;
            };
            TurnFeasibilityResult turn_fb;  // direct 模式保持全零

            if (turn_planner_type_ != "direct") {
            {
                double max_allowed_curv = (min_turning_radius_ > 1e-9)
                    ? (1.0 / min_turning_radius_) : std::numeric_limits<double>::max();
                bool in_turn = false;
                int turn_idx = 0;
                double turn_max_curv = 0.0;

                for (size_t i = 1; i < path.size(); ++i) {
                    // F2C 的 PathState 自带 type 字段：SWATH / TURN / HL_SWATH
                    bool is_turn = (path[i].type == f2c::types::PathSectionType::TURN);

                    // 曲率 = |dθ| / ds（使用 PathState.len 更精确）
                    double da = std::abs(path[i].angle - path[i-1].angle);
                    if (da > M_PI) da = 2.0 * M_PI - da;
                    double ds = path[i].len;
                    double curv = (ds > 1e-9) ? (da / ds) : 0.0;

                    if (is_turn && !in_turn) {
                        // 进入 TURN 段
                        in_turn = true;
                        ++turn_idx;
                        turn_max_curv = curv;
                    } else if (is_turn && in_turn) {
                        turn_max_curv = std::max(turn_max_curv, curv);
                    } else if (!is_turn && in_turn) {
                        // 离开 TURN 段，结算
                        in_turn = false;
                        turn_fb.worst_curvature = std::max(turn_fb.worst_curvature, turn_max_curv);
                        if (turn_max_curv > max_allowed_curv) {
                            ++turn_fb.infeasible_turns;
                            turn_fb.bad_segments.push_back({turn_idx, turn_max_curv});
                        }
                        turn_max_curv = 0.0;
                    }
                }
                // 路径以 TURN 结束
                if (in_turn) {
                    turn_fb.worst_curvature = std::max(turn_fb.worst_curvature, turn_max_curv);
                    if (turn_max_curv > max_allowed_curv) {
                        ++turn_fb.infeasible_turns;
                        turn_fb.bad_segments.push_back({turn_idx, turn_max_curv});
                    }
                }
                turn_fb.total_turns = turn_idx;
            }

            if (turn_fb.total_turns > 0) {
                double max_allowed_curv = (min_turning_radius_ > 1e-9)
                    ? (1.0 / min_turning_radius_) : std::numeric_limits<double>::max();
                RCLCPP_INFO(this->get_logger(),
                           "Turn feasibility: %d turns detected, %d infeasible "
                           "(max allowed curv=%.3f [r_min=%.2f m], worst=%.3f)",
                           turn_fb.total_turns, turn_fb.infeasible_turns,
                           max_allowed_curv, min_turning_radius_, turn_fb.worst_curvature);

                for (const auto& bad : turn_fb.bad_segments) {
                    RCLCPP_WARN(this->get_logger(),
                               "  ⚠ Turn #%d infeasible: curvature=%.3f > allowed=%.3f "
                               "(requires min turning radius=%.2f m, robot has %.2f m)",
                               bad.first, bad.second, max_allowed_curv,
                               1.0 / bad.second, min_turning_radius_);
                }

                if (turn_fb.infeasible_turns > 0) {
                    double fail_rate = static_cast<double>(turn_fb.infeasible_turns) / turn_fb.total_turns;
                    if (fail_rate > 0.5) {
                        RCLCPP_ERROR(this->get_logger(),
                                    "  🚫 %.0f%% of turns infeasible! "
                                    "Consider: ① increase headland width "
                                    "(mid_hl_width_ratio/no_hl_width_ratio) "
                                    "② increase swath_endpoint_shrink_distance "
                                    "③ use larger min_turning_radius in robot config",
                                    fail_rate * 100.0);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                                   "  ⚡ %.0f%% of turns may be hard to execute. "
                                   "Check headland width or swath endpoint shrink distance.",
                                   fail_rate * 100.0);
                    }
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Turn feasibility: no TURN segments detected (straight or HL path)");
            }

            // ── 发布不可执行掉头段标记（RViz 红色粗线） ──
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->now();
                marker.ns = "infeasible_turns";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.action = (turn_fb.infeasible_turns > 0)
                    ? visualization_msgs::msg::Marker::ADD
                    : visualization_msgs::msg::Marker::DELETE;
                marker.scale.x = 0.08;  // 线宽 8cm，比正常路径粗
                marker.pose.orientation.w = 1.0;

                double max_allowed_curv = (min_turning_radius_ > 1e-9)
                    ? (1.0 / min_turning_radius_) : std::numeric_limits<double>::max();

                if (turn_fb.infeasible_turns > 0) {
                    marker.color.r = 1.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 0.0f;
                    marker.color.a = 0.9f;  // 半透明红

                    // 第二遍扫描：收集不可执行 TURN 段的路径点
                    bool in_infeasible = false;
                    int current_turn = 0;
                    double turn_max_c = 0.0;
                    for (size_t i = 1; i < path.size(); ++i) {
                        bool is_turn = (path[i].type == f2c::types::PathSectionType::TURN);
                        double da = std::abs(path[i].angle - path[i-1].angle);
                        if (da > M_PI) da = 2.0 * M_PI - da;
                        double curv = (path[i].len > 1e-9) ? (da / path[i].len) : 0.0;

                        if (is_turn && !in_infeasible) {
                            in_infeasible = true;
                            ++current_turn;
                            turn_max_c = curv;
                            // 加入前一个点（TURN 段起点）
                            geometry_msgs::msg::Point pt;
                            pt.x = path[i-1].point.getX();
                            pt.y = path[i-1].point.getY();
                            pt.z = 0.02;
                            marker.points.push_back(pt);
                        }
                        if (is_turn) {
                            turn_max_c = std::max(turn_max_c, curv);
                            geometry_msgs::msg::Point pt;
                            pt.x = path[i].point.getX();
                            pt.y = path[i].point.getY();
                            pt.z = 0.02;
                            marker.points.push_back(pt);
                        }
                        if (!is_turn && in_infeasible) {
                            // 离开 TURN 段，判断是否不可执行
                            if (turn_max_c <= max_allowed_curv) {
                                // 可执行，清除收集的点
                                marker.points.clear();
                            }
                            // 不可执行的段保留点（下一次进入 infeasible 时会先 clear）
                            in_infeasible = false;
                            turn_max_c = 0.0;
                        }
                    }
                    // 最后一段
                    if (in_infeasible && turn_max_c <= max_allowed_curv) {
                        marker.points.clear();
                    }
                }

                last_infeasible_turn_markers_[index] = marker;
                infeasible_turn_pubs_[index]->publish(marker);
            }

            } else {
                // direct 模式：无转弯曲线，掉头检测不适用
                RCLCPP_INFO(this->get_logger(),
                           "Turn feasibility: N/A (direct Euclidean mode, no turning curves)");
                // 清除可能残留的红色标记
                visualization_msgs::msg::Marker clear;
                clear.header.frame_id = "map";
                clear.header.stamp = this->now();
                clear.ns = "infeasible_turns";
                clear.id = 0;
                clear.action = visualization_msgs::msg::Marker::DELETE;
                infeasible_turn_pubs_[index]->publish(clear);
                last_infeasible_turn_markers_[index] = visualization_msgs::msg::Marker();
            }

            // 原始路径消息（包含所有点）
            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = this->now();

            // 处理Dubins连接部分的点
            bool is_dubins_section = false;  // 标记是否在Dubins连接部分
            geometry_msgs::msg::Pose last_pose;
            bool has_last_pose = false;

            for (const auto& state : path) {
                geometry_msgs::msg::Pose pose;

                // 设置位置和方向
                pose.position.x = state.point.getX();
                pose.position.y = state.point.getY();
                pose.position.z = 0.0;
                
                double yaw = state.angle;
                pose.orientation.w = std::cos(yaw / 2);
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = std::sin(yaw / 2);

                // 检测是否在Dubins连接部分
                if (has_last_pose) {
                    double dx = pose.position.x - last_pose.position.x;
                    double dy = pose.position.y - last_pose.position.y;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    
                    // 如果点之间距离很小，说明是Dubins连接部分的密集点
                    if (dist < 0.05) {  // 5cm阈值
                        is_dubins_section = true;
                    } else {
                        is_dubins_section = false;
                    }
                }

                // 根据点的类型添加到不同的数组
                if (is_dubins_section) {
                    dubins_path_points.poses.push_back(pose);
                }

                last_pose = pose;
                has_last_pose = true;
            }

            // ── 边界补线 v7 校验：基于最终路径段检查 route/path 一致性 ──
            // v6 用点距离+跨度判断，在窄走廊会被转弯误判（转弯全线经过填充位置）。
            // v7 改为段级检测：找 path 中与 swath 同向的长连续段。
            // 仅当存在足够长的连续对齐段时才判定为"已覆盖"。
            {
                const auto path_pre_rdp_points = yingshi::materializePath(path_pre_rdp);
                if (swaths.size() > 0 && path_pre_rdp_points.size() >= 2) {
                    double half_w = coverage_width_ * 0.5;

                    for (size_t si = 0; si < swaths.size(); ++si) {
                        const auto& sw = swaths.at(si);
                        double sx = sw.startPoint().getX(), sy = sw.startPoint().getY();
                        double ex = sw.endPoint().getX(), ey = sw.endPoint().getY();
                        double slen = std::hypot(ex - sx, ey - sy);
                        if (slen < coverage_width_ * 0.5) continue;

                        // swath 方向单位向量
                        double sw_dx = (ex - sx) / slen;
                        double sw_dy = (ey - sy) / slen;

                        // ── 段级检测：扫描 path 中所有相邻点对（段），
                        //     找与 swath 同向且在覆盖范围内的最长连续段序列 ──
                        double best_along_min = 1e9, best_along_max = -1e9;
                        double cur_along_min = 1e9, cur_along_max = -1e9;

                        for (size_t pi = 0; pi + 1 < path_pre_rdp_points.size(); ++pi) {
                            const auto& p0 = path_pre_rdp_points[pi];
                            const auto& p1 = path_pre_rdp_points[pi + 1];
                            double p0x = p0.getX(), p0y = p0.getY();
                            double p1x = p1.getX(), p1y = p1.getY();

                            // 段中点
                            double mx = (p0x + p1x) * 0.5, my = (p0y + p1y) * 0.5;
                            // 段方向
                            double seg_dx = p1x - p0x, seg_dy = p1y - p0y;
                            double seg_len = std::hypot(seg_dx, seg_dy);

                            // 条件 1：段与 swath 同向（cos > 0.866 ≈ 30°）
                            bool aligned = false;
                            if (seg_len > 0.01) {
                                double cos_angle = std::abs(seg_dx * sw_dx + seg_dy * sw_dy) / seg_len;
                                aligned = (cos_angle > 0.866);
                            }

                            // 条件 2：段中点在 swath 覆盖范围内
                            double t = ((mx - sx) * sw_dx + (my - sy) * sw_dy) / slen;
                            t = std::max(0.0, std::min(1.0, t));
                            double cx = sx + t * sw_dx * slen;  // closest point on swath
                            double cy = sy + t * sw_dy * slen;  // (this is the projection)
                            double dist = std::hypot(mx - cx, my - cy);
                            bool in_range = (dist < half_w);

                            if (aligned && in_range) {
                                // 该段是 swath 覆盖的一部分
                                double along0 = ((p0x - sx) * sw_dx + (p0y - sy) * sw_dy);
                                double along1 = ((p1x - sx) * sw_dx + (p1y - sy) * sw_dy);
                                double seg_min = std::min(along0, along1);
                                double seg_max = std::max(along0, along1);

                                if (cur_along_max < cur_along_min) {
                                    // 开始新序列
                                    cur_along_min = seg_min;
                                    cur_along_max = seg_max;
                                } else {
                                    cur_along_min = std::min(cur_along_min, seg_min);
                                    cur_along_max = std::max(cur_along_max, seg_max);
                                }
                            } else {
                                // 段不满足条件 → 结束当前序列
                                if (cur_along_max - cur_along_min > best_along_max - best_along_min) {
                                    best_along_min = cur_along_min;
                                    best_along_max = cur_along_max;
                                }
                                cur_along_min = 1e9;
                                cur_along_max = -1e9;
                            }
                        }
                        // 处理最后一段序列
                        if (cur_along_max - cur_along_min > best_along_max - best_along_min) {
                            best_along_min = cur_along_min;
                            best_along_max = cur_along_max;
                        }

                        double best_span = best_along_max - best_along_min;
                        bool covered = (best_span >= slen * 0.5);  // 50% 即可，段级检测已过滤转弯

                        if (!covered) {
                            // 规划完成后禁止直接追加补线：否则会生成未经掉头规划的隐式连接。
                            // 边界线必须在 route 阶段恢复；这里仅报告规划不变量被破坏。
                            RCLCPP_ERROR(this->get_logger(),
                                "  Border fill v7: swath[%zu] uncovered (best_span=%.2f/%.2f), "
                                "route/path mismatch [(%.2f,%.2f)->(%.2f,%.2f)]",
                                si, best_span, slen, sx, sy, ex, ey);
                        }
                    }
                }
            }

            // F2C PathState 表示“段起点 + 航向 + 长度”；先展开完整折线，
            // 再统一用于发布、诊断、评估和 JSON，避免漏掉最后一段或使用不同版本。
            const auto published_path_points = yingshi::materializePath(path);
            const auto published_path_waypoints = yingshi::materializePathWaypoints(path);
            for (const auto& waypoint : published_path_waypoints) {
                geometry_msgs::msg::PoseStamped path_pose;
                path_pose.header = path_msg.header;
                path_pose.pose.position.x = waypoint.point.getX();
                path_pose.pose.position.y = waypoint.point.getY();
                path_pose.pose.position.z = 0.0;
                path_pose.pose.orientation.w = std::cos(waypoint.angle / 2.0);
                path_pose.pose.orientation.z = std::sin(waypoint.angle / 2.0);
                path_msg.poses.push_back(path_pose);
            }

            // ── 路径自洽性检查（publish 前最后一个 seam）──
            {
                std::vector<f2c::types::LinearRing> sanity_holes;
                for (const auto& hole : last_holes_[index]) {
                    auto hr = makeClosedF2CRing(hole);
                    if (hr.size() >= 4) sanity_holes.push_back(hr);
                }
                auto sanity = yingshi::checkPathSanity(
                    path, published_path_points, cell,
                    sanity_holes, swaths.size());
                if (!sanity.passed) {
                    for (const auto& iss : sanity.issues) {
                        if (iss.severity == yingshi::SanityIssue::Severity::ERROR)
                            RCLCPP_ERROR(this->get_logger(), "Sanity: %s", iss.message.c_str());
                        else
                            RCLCPP_WARN(this->get_logger(), "Sanity: %s", iss.message.c_str());
                    }
                }
            }

            // 保存并发布路径
            last_paths_[index] = path_msg;
            path_pubs_[index]->publish(path_msg);

            // 合并所有路径点
            geometry_msgs::msg::PoseArray sampled_points;
            sampled_points.header.frame_id = "map";
            sampled_points.header.stamp = this->now();
            sampled_points.poses = swath_path_points.poses;
            sampled_points.poses.insert(
                sampled_points.poses.end(),
                dubins_path_points.poses.begin(),
                dubins_path_points.poses.end()
            );

            // 保存并发布采样点
            last_sampled_points_[index] = sampled_points;
            sampled_path_pubs_[index]->publish(sampled_points);

            RCLCPP_INFO(this->get_logger(), "Published path details for polygon_%d:", polygon_id);
            RCLCPP_INFO(this->get_logger(), "  - Original path points: %zu", path_msg.poses.size());
            RCLCPP_INFO(this->get_logger(), "  - Swath path points: %zu", swath_path_points.poses.size());
            RCLCPP_INFO(this->get_logger(), "  - Dubins path points: %zu", dubins_path_points.poses.size());
            RCLCPP_INFO(this->get_logger(), "  - Total sampled points: %zu", sampled_points.poses.size());

            // ── 计时截止（纯规划，不含诊断/评估）──
            auto planning_end = std::chrono::steady_clock::now();
            double planning_time_ms = std::chrono::duration<double, std::milli>(
                planning_end - planning_start).count();

            // ── 路径-孔洞交叉诊断（点级 + 线段级）──
            {
                std::vector<f2c::types::LinearRing> hole_check_rings;
                for (const auto& hole : last_holes_[index]) {
                    auto hr = makeClosedF2CRing(hole);
                    if (hr.size() >= 4) hole_check_rings.push_back(hr);
                }
                if (!hole_check_rings.empty()) {
                    // lambda: 射线法点-in-孔洞 (→ yingshi::pointInAnyHole)
                    auto pointInHole = [&](double px, double py) -> bool {
                        return yingshi::pointInAnyHole(px, py, hole_check_rings);
                    };

                    const int SEG_SAMPLES = 50;  // 高密度采样，确保窄孔洞不被漏掉
                    size_t pts_in_hole = 0, segs_crossing = 0;

                    // 辅助：对单个segment做采样检测 (→ yingshi::segmentCrossesHole)
                    auto checkSegment = [&](double x0, double y0, double x1, double y1) -> bool {
                        return yingshi::segmentCrossesHole(x0, y0, x1, y1, hole_check_rings, SEG_SAMPLES);
                    };

                    // ── 1. 简化路径检测（F2C Path, RDP后 ~161点）──
                    for (const auto& point : published_path_points) {
                        if (pointInHole(point.getX(), point.getY()))
                            ++pts_in_hole;
                    }
                    for (size_t si = 0; si + 1 < published_path_points.size(); ++si) {
                        if (checkSegment(published_path_points[si].getX(), published_path_points[si].getY(),
                                        published_path_points[si + 1].getX(), published_path_points[si + 1].getY()))
                            ++segs_crossing;
                    }
                    RCLCPP_INFO(this->get_logger(),
                               "  Simplified path (%zu pts, %zu segs): %zu pts in hole, %zu segs crossing",
                               published_path_points.size(),
                               published_path_points.empty() ? 0 : published_path_points.size() - 1,
                               pts_in_hole, segs_crossing);

                    // ── 2. 已发布路径复核（与 RViz nav_msgs::Path 完全同源）──
                    {
                        size_t d_pts = 0, d_segs = 0;
                        const auto& poses = path_msg.poses;
                        for (size_t pi = 0; pi < poses.size(); ++pi) {
                            if (pointInHole(poses[pi].pose.position.x, poses[pi].pose.position.y))
                                ++d_pts;
                        }
                        const int MAX_REPORT = 5;  // 最多打印5个穿越线段
                        int reported = 0;
                        for (size_t si = 0; si + 1 < poses.size(); ++si) {
                            if (checkSegment(poses[si].pose.position.x, poses[si].pose.position.y,
                                            poses[si+1].pose.position.x, poses[si+1].pose.position.y)) {
                                ++d_segs;
                                if (reported < MAX_REPORT) {
                                    RCLCPP_WARN(this->get_logger(),
                                               "  ⚠ seg[%zu]: (%.2f,%.2f) → (%.2f,%.2f)  len=%.2f m",
                                               si,
                                               poses[si].pose.position.x, poses[si].pose.position.y,
                                               poses[si+1].pose.position.x, poses[si+1].pose.position.y,
                                               std::hypot(poses[si+1].pose.position.x - poses[si].pose.position.x,
                                                         poses[si+1].pose.position.y - poses[si].pose.position.y));
                                    ++reported;
                                }
                            }
                        }
                        RCLCPP_INFO(this->get_logger(),
                                   "  Published path (%zu pts, %zu segs): %zu pts in hole, %zu segs crossing",
                                   poses.size(), poses.size()>0 ? poses.size()-1 : 0,
                                   d_pts, d_segs);
                        if (d_pts > 0 || d_segs > 0) {
                            RCLCPP_WARN(this->get_logger(),
                                       "⚠ PUBLISHED PATH CROSSES HOLE!");
                        }
                    }

                    if (pts_in_hole == 0 && segs_crossing == 0) {
                        RCLCPP_INFO(this->get_logger(),
                                   "✓ Simplified path: clean (0 pts, 0 segs in hole)");
                    }
                }
            }

            // ── 评估输出（实车构建 OFF 时整块编译剔除）──
#if YINGSHI_EVAL_ENABLED
            if (eval_enable_report_) {
                // 构建孔洞环列表（f2c::types::LinearRing 格式）
                std::vector<f2c::types::LinearRing> hole_rings;
                for (const auto& hole : last_holes_[index]) {
                    auto hole_ring = makeClosedF2CRing(hole);
                    if (hole_ring.size() >= 4) {
                        hole_rings.push_back(hole_ring);
                    }
                }

                // 配置评估参数
                EvalParams eval_params;
                eval_params.max_diff_curv = max_diff_curv_;
                eval_params.coverage_width = coverage_width_;
                eval_params.swath_overlap_ratio = swath_overlap_ratio_;
                eval_params.turn_planner_type = turn_planner_type_.c_str();
                eval_params.grid_resolution = eval_grid_resolution_;
                eval_params.coverage_threshold = eval_coverage_threshold_;
                eval_params.turn_angle_threshold = 30.0;
                eval_params.use_grid_method = eval_use_grid_method_;

                // 运行评估（与已发布路径同源；评估器会按 PathState 语义展开完整折线）
                f2c::types::Cells full_polygon_cells; full_polygon_cells.addGeometry(cell);
                if (eval_use_grid_method_) {
                    double est_area = (cell.getExteriorRing().size() > 0) ? cell.area() : 0.0;
                    int est_points = static_cast<int>(est_area / (eval_grid_resolution_ * eval_grid_resolution_));
                    RCLCPP_INFO(this->get_logger(),
                        "Grid evaluation starting: area=%.1f m², resolution=%.2f m, "
                        "est_grid_points=%d, path_points=%zu. May take a moment...",
                        est_area, eval_grid_resolution_, est_points, published_path_points.size());
                }
                EvalResult eval_result = evaluatePlan(
                    path, swaths, full_polygon_cells, hole_rings,
                    planning_time_ms, eval_params);

                // 注入掉头可行性检查结果
                eval_result.turn_total_count = turn_fb.total_turns;
                eval_result.turn_infeasible_count = turn_fb.infeasible_turns;
                eval_result.turn_feasibility_pass = (turn_fb.infeasible_turns == 0);
                eval_result.turn_worst_curvature = turn_fb.worst_curvature;
                eval_result.turn_planner_used = used_planner.c_str();

                // 输出报告
                std::string scenario_label = "polygon_" + std::to_string(polygon_id);
                std::string report = formatEvalReport(eval_result, scenario_label.c_str());
                RCLCPP_INFO(this->get_logger(), "%s", report.c_str());

                // 将网格覆盖数据写入 JSON（供渲染脚本复用，避免 Python 重算）
                if (eval_use_grid_method_ && eval_result.grid_resolution > 0.0) {
                    std::string grid_path = output_dir_ + "/f2c_grid_" + scenario_label + ".json";
                    writeGridJson(eval_result, grid_path);
                }

                // ── 路径+评估可视化 JSON（供 render_coverage.py 直接出图）──
                // 支持多 cell 多边形：首个 cell 创建文件，后续 cell 追加路径点。
                // 使用计数器而非文件存在性检测，避免多次 polygon 处理叠加旧数据。
                {
                    std::string vis_path = output_dir_ + "/f2c_vis_" + scenario_label + ".json";
                    bool is_first_cell = (vis_json_cell_count_ == 0);
                    ++vis_json_cell_count_;

                    if (is_first_cell) {
                        std::ofstream vf(vis_path);
                        vf << "{\n  \"path\": [\n";
                        for (size_t pi = 0; pi < published_path_points.size(); ++pi) {
                            vf << "    {\"x\":" << published_path_points[pi].getX()
                               << ",\"y\":" << published_path_points[pi].getY() << "}";
                            if (pi + 1 < published_path_points.size()) vf << ",";
                            vf << "\n";
                        }
                        vf << "  ],\n  \"swaths\": [\n";
                        for (size_t si = 0; si < swaths.size(); ++si) {
                            const auto& swath = swaths.at(si);
                            vf << "    {\"points\":[{\"x\":"
                               << swath.startPoint().getX() << ",\"y\":"
                               << swath.startPoint().getY() << "},{\"x\":"
                               << swath.endPoint().getX() << ",\"y\":"
                               << swath.endPoint().getY() << "}]}";
                            if (si + 1 < swaths.size()) vf << ",";
                            vf << "\n";
                        }
                        vf << "  ],\n  \"eval\": {\n";
                        vf << "    \"coverage_rate\":" << eval_result.coverage_rate << ",\n";
                        vf << "    \"single_score\":" << eval_result.single_score << "\n";
                        vf << "  }";

                        // ── 追加 cells 数组（cell 边界 + 起止点）──
                        // 使用 cell_order 映射：遍历顺序 → 原始 no_hl 索引
                        size_t num_cells = swaths_by_cells.size();
                        vf << ",\n  \"cells\": [\n";
                        for (size_t ci = 0; ci < num_cells; ++ci) {
                            size_t orig_idx = (ci < cell_order.size()) ? cell_order[ci] : ci;
                            const auto& cell_geom = no_hl.getGeometry(orig_idx);
                            const auto& ring = cell_geom.getExteriorRing();

                            // Cell 边界顶点
                            vf << "    {\"id\":" << ci
                               << ",\"boundary\":[";
                            for (size_t pi = 0; pi < ring.size(); ++pi) {
                                vf << "[" << ring.getGeometry(pi).getX()
                                   << "," << ring.getGeometry(pi).getY() << "]";
                                if (pi < ring.size() - 1) vf << ",";
                            }
                            vf << "]";

                            // 起止点：统一使用 swaths_by_cells 数据
                            if (swaths_by_cells.at(ci).size() > 0) {
                                const auto& swaths_cell = swaths_by_cells.at(ci);
                                const auto& e = swaths_cell.at(0).startPoint();
                                vf << ",\"entry\":{\"x\":" << e.getX()
                                   << ",\"y\":" << e.getY() << "}";
                                const auto& x = swaths_cell.at(swaths_cell.size() - 1).endPoint();
                                vf << ",\"exit\":{\"x\":" << x.getX()
                                   << ",\"y\":" << x.getY() << "}";
                                vf << ",\"swath_count\":" << swaths_cell.size();
                                double cell_area = cell_geom.area();
                                vf << ",\"area\":" << cell_area;
                            } else {
                                vf << ",\"swath_count\":0,\"area\":0";
                            }

                            vf << "}";
                            if (ci < num_cells - 1) vf << ",";
                            vf << "\n";
                        }
                        vf << "  ]";

                        // ── 追加 connections 数组（route 连接控制点，不伪装成最终曲线）──
                        vf << ",\n  \"connections\": [\n";
                        bool first_conn = true;
                        const size_t num_route_groups = route.sizeVectorSwaths();
                        const size_t num_route_connections = route.sizeConnections();
                        for (size_t ci = 1;
                             ci < num_route_groups && ci < num_route_connections;
                             ++ci) {
                            const auto& prev_swaths = route.getSwaths(ci - 1);
                            const auto& next_swaths = route.getSwaths(ci);
                            if (prev_swaths.size() == 0 || next_swaths.size() == 0) continue;
                            const auto& from_pt = prev_swaths.at(prev_swaths.size() - 1).endPoint();
                            const auto& to_pt = next_swaths.at(0).startPoint();
                            const auto& connection = route.getConnection(ci);

                            if (!first_conn) vf << ",\n";
                            first_conn = false;
                            vf << "    {\"from_cell\":" << (ci - 1)
                               << ",\"to_cell\":" << ci
                               << ",\"source\":\"route_waypoints\",\"path\":[["
                               << from_pt.getX() << "," << from_pt.getY() << "]";
                            for (size_t pi = 0; pi < connection.size(); ++pi) {
                                const auto& point = connection.getGeometry(pi);
                                vf << ",[" << point.getX() << "," << point.getY() << "]";
                            }
                            vf << ",[" << to_pt.getX() << "," << to_pt.getY() << "]]}";
                        }
                        vf << "\n  ]\n";

                        vf << "}\n";
                        vf.close();
                    } else {
                        // Multi-cell: append path points before the closing of path array
                        // Read existing JSON, find the "path" array closing position
                        std::ifstream rf(vis_path);
                        std::string content((std::istreambuf_iterator<char>(rf)),
                                            std::istreambuf_iterator<char>());
                        rf.close();

                        // swaths 紧跟 path；在 swaths 字段之前反查 path 的闭合括号。
                        auto swaths_pos = content.find("\"swaths\"");
                        if (swaths_pos != std::string::npos) {
                            auto path_end = content.rfind("]", swaths_pos);
                            if (path_end != std::string::npos) {
                                std::ofstream vf(vis_path);
                                // Write everything before the path end bracket
                                vf << content.substr(0, path_end);
                                // Append new path points
                                vf << ",\n";
                                for (size_t pi = 0; pi < published_path_points.size(); ++pi) {
                                    vf << "    {\"x\":" << published_path_points[pi].getX()
                                       << ",\"y\":" << published_path_points[pi].getY() << "}";
                                    if (pi + 1 < published_path_points.size()) vf << ",";
                                    vf << "\n";
                                }
                                // Write the rest (closing bracket + eval + cells + connections)
                                vf << content.substr(path_end);
                                vf.close();
                            }
                        }
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "Vis JSON saved: %s (%zu path points, %s cell)",
                        vis_path.c_str(), published_path_points.size(),
                        is_first_cell ? "first" : "append");
                }

            }
#endif  // YINGSHI_EVAL_ENABLED

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in path planning: %s", e.what());
            this->clearPlanningCacheForPolygon(index, true);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolygonPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
