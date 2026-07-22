#include "yingshi_robot/physical_footprint.hpp"

#include <algorithm>
#include <cmath>

namespace yingshi {

namespace {

constexpr double kGeometryTolerance = 1e-9;

bool finitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

bool validParams(const PhysicalFootprintParams& params)
{
    return finitePositive(params.body_length) &&
        finitePositive(params.body_width) &&
        finitePositive(params.cleaner_diameter) &&
        std::isfinite(params.cleaner_center_forward) &&
        std::isfinite(params.cleaner_center_lateral) &&
        std::isfinite(params.safety_margin) &&
        params.safety_margin >= 0.0 &&
        finitePositive(params.sample_step);
}

f2c::types::Point transformPoint(
    double local_x,
    double local_y,
    const f2c::types::Point& center,
    double heading)
{
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    return f2c::types::Point(
        center.getX() + c * local_x - s * local_y,
        center.getY() + s * local_x + c * local_y);
}

f2c::types::Cell makeBody(
    const f2c::types::Point& center,
    double heading,
    const PhysicalFootprintParams& params)
{
    const double half_length = 0.5 * params.body_length;
    const double half_width = 0.5 * params.body_width;
    const std::vector<f2c::types::Point> local_corners {
        {-half_length, -half_width},
        { half_length, -half_width},
        { half_length,  half_width},
        {-half_length,  half_width},
    };

    f2c::types::LinearRing ring;
    for (const auto& local : local_corners) {
        ring.addPoint(transformPoint(
            local.getX(), local.getY(), center, heading));
    }
    ring.addPoint(transformPoint(
        local_corners.front().getX(), local_corners.front().getY(),
        center, heading));

    const auto body = f2c::types::Cell(ring);
    return params.safety_margin > kGeometryTolerance
        ? f2c::types::Cell::buffer(body, params.safety_margin)
        : body;
}

f2c::types::Cell makeCleaner(
    const f2c::types::Point& center,
    double heading,
    double local_y,
    const PhysicalFootprintParams& params)
{
    const auto wheel_center = transformPoint(
        params.cleaner_center_forward, local_y, center, heading);
    const double radius = 0.5 * params.cleaner_diameter + params.safety_margin;
    return f2c::types::Cell::buffer(wheel_center, radius);
}

std::vector<f2c::types::Cell> makeFootprintParts(
    const f2c::types::Point& center,
    double heading,
    const PhysicalFootprintParams& params)
{
    return {
        makeBody(center, heading, params),
        makeCleaner(
            center, heading, params.cleaner_center_lateral, params),
        makeCleaner(
            center, heading, -params.cleaner_center_lateral, params),
    };
}

FootprintCollisionResult invalidParamsResult()
{
    FootprintCollisionResult result;
    result.collision = true;
    result.message = "Invalid physical footprint parameters";
    return result;
}

}  // namespace

double physicalFootprintClearanceRadius(
    const PhysicalFootprintParams& params)
{
    if (!validParams(params)) return 0.0;

    const double body_radius = std::hypot(
        0.5 * params.body_length, 0.5 * params.body_width);
    const double cleaner_radius = std::hypot(
        params.cleaner_center_forward,
        std::abs(params.cleaner_center_lateral) +
            0.5 * params.cleaner_diameter);
    return std::max(body_radius, cleaner_radius) + params.safety_margin;
}

std::size_t repairRouteSwathEndpointsForCollision(
    f2c::types::Route& route,
    const f2c::types::Cell& free_space,
    const std::vector<f2c::types::LinearRing>& obstacle_rings,
    const PhysicalFootprintParams& params)
{
    if (!validParams(params) || route.sizeVectorSwaths() == 0) return 0;

    const double clearance_radius =
        physicalFootprintClearanceRadius(params);
    const double step = std::max(
        0.01, std::min(0.05, 0.25 * params.sample_step));
    std::size_t adjusted_swaths = 0;

    for (std::size_t group_idx = 0;
         group_idx < route.sizeVectorSwaths(); ++group_idx) {
        const auto& source_swaths = route.getSwaths(group_idx);
        f2c::types::Swaths repaired_swaths;
        bool group_changed = false;

        for (const auto& swath : source_swaths) {
            const auto start = swath.startPoint();
            const auto end = swath.endPoint();
            const double dx = end.getX() - start.getX();
            const double dy = end.getY() - start.getY();
            const double length = std::hypot(dx, dy);
            if (length <= 1e-9) {
                repaired_swaths.push_back(swath);
                continue;
            }

            const double heading = std::atan2(dy, dx);
            const auto endpoint_safe = [&](const f2c::types::Point& point) {
                return !checkFootprintAtPose(
                    point, heading, free_space, obstacle_rings,
                    params).collision &&
                    !checkFootprintAtPose(
                        point, heading + std::acos(-1.0), free_space,
                        obstacle_rings, params).collision;
            };

            const double max_trim = std::min(
                0.45 * length,
                std::max(2.0 * clearance_radius, step));
            const auto find_trim = [&](bool from_start) {
                for (double trim = 0.0;
                     trim <= max_trim + 1e-9;
                     trim += step) {
                    const double offset = from_start ? trim : -trim;
                    const auto candidate = f2c::types::Point(
                        (from_start ? start.getX() : end.getX()) +
                            offset * dx / length,
                        (from_start ? start.getY() : end.getY()) +
                            offset * dy / length);
                    if (endpoint_safe(candidate)) return trim;
                }
                return 0.0;
            };

            const double start_trim = find_trim(true);
            const double end_trim = find_trim(false);
            if (start_trim + end_trim >= 0.9 * length) {
                repaired_swaths.push_back(swath);
                continue;
            }

            const auto repaired_start = f2c::types::Point(
                start.getX() + start_trim * dx / length,
                start.getY() + start_trim * dy / length);
            const auto repaired_end = f2c::types::Point(
                end.getX() - end_trim * dx / length,
                end.getY() - end_trim * dy / length);
            if (start_trim <= 1e-9 && end_trim <= 1e-9) {
                repaired_swaths.push_back(swath);
                continue;
            }

            f2c::types::LineString repaired_path;
            repaired_path.addPoint(repaired_start);
            repaired_path.addPoint(repaired_end);
            f2c::types::Swath repaired(
                repaired_path, swath.getWidth(), swath.getId(),
                swath.getType());
            repaired.setCreationDir(swath.getCreationDir());
            repaired_swaths.push_back(repaired);
            group_changed = true;
            ++adjusted_swaths;
        }

        if (group_changed) route.setSwaths(group_idx, repaired_swaths);
    }
    return adjusted_swaths;
}

FootprintCollisionResult checkFootprintAtPose(
    const f2c::types::Point& center,
    double heading,
    const f2c::types::Cell& free_space,
    const std::vector<f2c::types::LinearRing>& obstacle_rings,
    const PhysicalFootprintParams& params)
{
    if (!validParams(params) || free_space.getExteriorRing().size() < 4) {
        return invalidParamsResult();
    }

    FootprintCollisionResult result;
    const auto footprint_parts = makeFootprintParts(center, heading, params);

    std::vector<f2c::types::Cell> obstacles;
    obstacles.reserve(obstacle_rings.size());
    for (const auto& ring : obstacle_rings) {
        if (ring.size() >= 4) obstacles.emplace_back(ring);
    }

    for (const auto& part : footprint_parts) {
        if (!part.within(free_space)) {
            result.outside_boundary = true;
        }
        for (const auto& obstacle : obstacles) {
            if (part.intersects(obstacle)) {
                result.intersects_obstacle = true;
            }
        }
    }

    result.collision = result.outside_boundary || result.intersects_obstacle;
    if (result.outside_boundary && result.intersects_obstacle) {
        result.message = "Physical footprint leaves boundary and intersects obstacle";
    } else if (result.outside_boundary) {
        result.message = "Physical footprint leaves free-space boundary";
    } else if (result.intersects_obstacle) {
        result.message = "Physical footprint intersects obstacle";
    }
    return result;
}

FootprintCollisionResult checkPathFootprintCollision(
    const f2c::types::Path& path,
    const f2c::types::Cell& free_space,
    const std::vector<f2c::types::LinearRing>& obstacle_rings,
    const PhysicalFootprintParams& params)
{
    if (!validParams(params)) return invalidParamsResult();

    FootprintCollisionResult result;
    const auto check_segment = [&](
        const f2c::types::Point& start,
        double robot_heading,
        double travel_heading,
        double length,
        std::size_t state_index) {
        const double segment_length = std::max(0.0, length);
        const std::size_t samples = std::max<std::size_t>(
            1, static_cast<std::size_t>(
                std::ceil(segment_length / params.sample_step)));

        for (std::size_t sample = 0; sample <= samples; ++sample) {
            const double distance = segment_length *
                static_cast<double>(sample) / static_cast<double>(samples);
            const auto point = f2c::types::Point(
                start.getX() + distance * std::cos(travel_heading),
                start.getY() + distance * std::sin(travel_heading));
            ++result.checked_poses;

            const auto pose_result = checkFootprintAtPose(
                point, robot_heading, free_space, obstacle_rings, params);
            if (pose_result.collision) {
                const std::size_t checked_poses = result.checked_poses;
                result = pose_result;
                result.checked_poses = checked_poses;
                result.collision_state_index = state_index;
                return true;
            }
        }
        return false;
    };

    for (std::size_t state_index = 0; state_index < path.size(); ++state_index) {
        const auto& state = path[state_index];
        const bool backward =
            state.dir == f2c::types::PathDirection::BACKWARD;
        const double travel_heading = state.angle +
            (backward ? std::acos(-1.0) : 0.0);
        if (check_segment(
                state.point, state.angle, travel_heading,
                state.len, state_index)) {
            return result;
        }

        if (state_index + 1 < path.size()) {
            const double state_length = std::max(0.0, state.len);
            const auto state_end = f2c::types::Point(
                state.point.getX() + state_length * std::cos(travel_heading),
                state.point.getY() + state_length * std::sin(travel_heading));
            const auto& next_start = path[state_index + 1].point;
            const double dx = next_start.getX() - state_end.getX();
            const double dy = next_start.getY() - state_end.getY();
            const double connection_length = std::hypot(dx, dy);
            if (connection_length > 1e-9 && check_segment(
                    state_end, std::atan2(dy, dx), std::atan2(dy, dx),
                    connection_length, state_index)) {
                return result;
            }
        }
    }
    return result;
}

}  // namespace yingshi
