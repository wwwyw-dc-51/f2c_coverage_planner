#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <fields2cover.h>

namespace yingshi {

// Collision-only physical model. It must not feed headland, sweep, or
// coverage-rate calculations.
struct PhysicalFootprintParams {
    double body_length = 0.90;
    double body_width = 0.80;
    double cleaner_diameter = 0.45;
    double cleaner_center_forward = 0.45;
    double cleaner_center_lateral = 0.225;
    double safety_margin = 0.03;
    double sample_step = 0.20;
};

struct FootprintCollisionResult {
    bool collision = false;
    bool outside_boundary = false;
    bool intersects_obstacle = false;
    std::size_t checked_poses = 0;
    std::size_t collision_state_index = 0;
    std::string message;
};

// Conservative circular envelope for obstacle-corner connection repair.
double physicalFootprintClearanceRadius(
    const PhysicalFootprintParams& params = {});

// Trim only colliding swath endpoints and keep route connections aligned.
// This is a collision repair; it must not be used for coverage accounting.
std::size_t repairRouteSwathEndpointsForCollision(
    f2c::types::Route& route,
    const f2c::types::Cell& free_space,
    const std::vector<f2c::types::LinearRing>& obstacle_rings,
    const PhysicalFootprintParams& params = {});

FootprintCollisionResult checkFootprintAtPose(
    const f2c::types::Point& center,
    double heading,
    const f2c::types::Cell& free_space,
    const std::vector<f2c::types::LinearRing>& obstacle_rings,
    const PhysicalFootprintParams& params = {});

// Samples every path segment, including discontinuous connections, and
// checks only collision; rotation-coverage accounting is intentionally absent.
FootprintCollisionResult checkPathFootprintCollision(
    const f2c::types::Path& path,
    const f2c::types::Cell& free_space,
    const std::vector<f2c::types::LinearRing>& obstacle_rings,
    const PhysicalFootprintParams& params = {});

}  // namespace yingshi
