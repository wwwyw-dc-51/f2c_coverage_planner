#pragma once
// ========== 路径自洽性验证 ==========
// 只检查算法自身承诺是否兑现（路径在区域内、swath 数 > 0、不穿洞）。
// 不做碰撞判定——那是实车避障层的职责。
// 失败 = 算法 bug，不是安全事件。

#include <string>
#include <vector>
#include <fields2cover.h>
#include "yingshi_robot/boundary_filler.hpp"  // pointInPolygon, segmentCrossesHole

namespace yingshi {

struct SanityIssue {
    enum class Severity { WARN, ERROR };
    Severity severity = Severity::WARN;
    std::string message;
};

struct PathSanityResult {
    bool passed = true;
    std::vector<SanityIssue> issues;
};

// 验证规划结果的内部自洽性。
// 此函数不依赖 ROS、不写文件、不访问网络。
inline PathSanityResult checkPathSanity(
    const f2c::types::Path& path,
    const std::vector<f2c::types::Point>& path_points,
    const f2c::types::Cell& polygon,
    const std::vector<f2c::types::LinearRing>& hole_rings,
    size_t total_swaths)
{
    PathSanityResult result;

    // 1. 空路径
    if (path_points.empty()) {
        result.passed = false;
        result.issues.push_back({SanityIssue::Severity::ERROR,
            "Path is empty — no swaths were generated or routing failed"});
        return result;
    }

    // 2. Swath 数为零
    if (total_swaths == 0) {
        result.passed = false;
        result.issues.push_back({SanityIssue::Severity::ERROR,
            "Zero swaths generated — decomposition or swath generation failed"});
    }

    // 3. 路径点是否全在外环内
    //    容差设计：pointInPolygon 射线法对恰好在边界上的点返回 false，
    //    此类浮点精度边界点不应阻断规划。区分"真正越界"和"贴在边界上"。
    {
        const auto& outer = polygon.getExteriorRing();
        // 计算外环包围盒
        double ob_min_x = 1e30, ob_max_x = -1e30, ob_min_y = 1e30, ob_max_y = -1e30;
        for (const auto& p : outer) {
            ob_min_x = std::min(ob_min_x, p.getX());
            ob_max_x = std::max(ob_max_x, p.getX());
            ob_min_y = std::min(ob_min_y, p.getY());
            ob_max_y = std::max(ob_max_y, p.getY());
        }
        constexpr double kBboxEpsilon = 1e-9;
        size_t hard_oob = 0;   // 真正越界
        size_t soft_oob = 0;   // 贴在边界上（浮点精度）
        for (const auto& pt : path_points) {
            if (!pointInPolygon(pt.getX(), pt.getY(), outer)) {
                // 计算点到包围盒的超出距离
                double dx = 0, dy = 0;
                if (pt.getX() < ob_min_x) dx = ob_min_x - pt.getX();
                else if (pt.getX() > ob_max_x) dx = pt.getX() - ob_max_x;
                if (pt.getY() < ob_min_y) dy = ob_min_y - pt.getY();
                else if (pt.getY() > ob_max_y) dy = pt.getY() - ob_max_y;
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist <= kBboxEpsilon) {
                    ++soft_oob;
                } else {
                    ++hard_oob;
                }
            }
        }
        if (hard_oob > 0) {
            result.passed = false;
            result.issues.push_back({SanityIssue::Severity::ERROR,
                std::to_string(hard_oob) + " path points outside polygon exterior"});
        }
        if (soft_oob > 0) {
            result.issues.push_back({SanityIssue::Severity::WARN,
                std::to_string(soft_oob) + " path points on polygon boundary (floating-point tolerance)"});
        }
    }

    // 4. 路径点是否在孔洞内
    if (!hole_rings.empty()) {
        size_t in_hole = 0;
        for (const auto& pt : path_points) {
            if (pointInAnyHole(pt.getX(), pt.getY(), hole_rings)) {
                ++in_hole;
            }
        }
        if (in_hole > 0) {
            result.passed = false;
            result.issues.push_back({SanityIssue::Severity::ERROR,
                std::to_string(in_hole) + " path points inside holes"});
        }
    }

    // 5. 连接段穿洞
    if (!hole_rings.empty() && path_points.size() >= 2) {
        size_t crossings = 0;
        for (size_t i = 0; i + 1 < path_points.size(); ++i) {
            if (segmentCrossesHole(
                    path_points[i].getX(), path_points[i].getY(),
                    path_points[i+1].getX(), path_points[i+1].getY(),
                    hole_rings, 50)) {
                ++crossings;
            }
        }
        if (crossings > 0) {
            result.passed = false;
            result.issues.push_back({SanityIssue::Severity::ERROR,
                std::to_string(crossings) + " path segments cross holes"});
        }
    }

    return result;
}

}  // namespace yingshi
