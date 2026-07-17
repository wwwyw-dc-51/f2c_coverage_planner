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
    {
        const auto& outer = polygon.getExteriorRing();
        size_t out_of_bounds = 0;
        for (const auto& pt : path_points) {
            if (!pointInPolygon(pt.getX(), pt.getY(), outer)) {
                ++out_of_bounds;
            }
        }
        if (out_of_bounds > 0) {
            result.passed = false;
            result.issues.push_back({SanityIssue::Severity::ERROR,
                std::to_string(out_of_bounds) + " path points outside polygon exterior"});
        }
    }

    // 4. 路径点是否在孔洞内
    if (!hole_rings.empty()) {
        size_t in_hole = 0;
        for (const auto& pt : path_points) {
            for (const auto& hole : hole_rings) {
                if (pointInPolygon(pt.getX(), pt.getY(), hole)) {
                    ++in_hole;
                    break;
                }
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
            result.issues.push_back({SanityIssue::Severity::WARN,
                std::to_string(crossings) + " path segments cross holes"});
        }
    }

    return result;
}

}  // namespace yingshi
