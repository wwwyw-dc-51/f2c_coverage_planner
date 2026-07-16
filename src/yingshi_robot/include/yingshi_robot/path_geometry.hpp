#pragma once

#include <vector>

#include "fields2cover.h"

namespace yingshi {

struct PathWaypoint {
    f2c::types::Point point;
    double angle;
};

/// 展开发布用路点；保留每个状态的真实线段和倒车段车体朝向。
inline std::vector<PathWaypoint> materializePathWaypoints(
    const f2c::types::Path& path)
{
    std::vector<PathWaypoint> waypoints;
    for (size_t i = 0; i < path.size(); ++i) {
        const auto& state = path[i];
        waypoints.push_back({state.point, state.angle});
        const auto end = state.atEnd();
        const bool is_last = (i + 1 == path.size());
        const bool is_discontinuous = !is_last && end.distance(path[i + 1].point) > 1e-9;
        if ((is_last || is_discontinuous) && state.point.distance(end) > 1e-9) {
            waypoints.push_back({end, state.angle});
        }
    }
    return waypoints;
}

/// 将 F2C 的“起点 + 航向 + 长度”状态展开为完整折线。
/// 相邻状态不连续时，同时保留状态线段和显式连接，避免静默漏算。
inline std::vector<f2c::types::Point> materializePath(
    const f2c::types::Path& path)
{
    std::vector<f2c::types::Point> points;

    auto append_distinct = [&points](const f2c::types::Point& point) {
        if (!points.empty() && points.back().distance(point) <= 1e-9) {
            return;
        }
        points.push_back(point);
    };

    for (const auto& waypoint : materializePathWaypoints(path)) {
        append_distinct(waypoint.point);
    }

    return points;
}

inline double polylineLength(const std::vector<f2c::types::Point>& points) {
    double total = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        total += points[i - 1].distance(points[i]);
    }
    return total;
}

}  // namespace yingshi
