#include "yingshi_robot/traversability.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

namespace yingshi {

TraversabilityResult analyzeTraversability(
    const f2c::types::Cell& free_space,
    const TraversabilityParams& params)
{
    TraversabilityResult result;

    if (!std::isfinite(params.robot_width) || params.robot_width <= 0.0) {
        result.error_message = "robot_width must be finite and positive";
        return result;
    }
    if (!std::isfinite(params.clearance_margin) ||
        params.clearance_margin < 0.0) {
        result.error_message =
            "clearance_margin must be finite and non-negative";
        return result;
    }
    if (!std::isfinite(params.max_excluded_area_ratio) ||
        params.max_excluded_area_ratio < 0.0 ||
        params.max_excluded_area_ratio > 1.0) {
        result.error_message =
            "max_excluded_area_ratio must be between zero and one";
        return result;
    }
    if (!std::isfinite(params.area_tolerance) ||
        params.area_tolerance <= 0.0) {
        result.error_message =
            "area_tolerance must be finite and positive";
        return result;
    }
    if (!std::isfinite(params.ratio_tolerance) ||
        params.ratio_tolerance <= 0.0 ||
        params.ratio_tolerance > 1.0) {
        result.error_message =
            "ratio_tolerance must be finite and between zero and one";
        return result;
    }

    f2c::types::Cells original;
    original.addGeometry(free_space);
    result.original_area = original.area();
    if (!std::isfinite(result.original_area) ||
        result.original_area <= params.area_tolerance) {
        result.error_message = "free_space must have positive finite area";
        return result;
    }

    const double footprint_radius = 0.5 * params.robot_width;
    const double center_clearance =
        footprint_radius + params.clearance_margin;
    if (!std::isfinite(center_clearance)) {
        result.error_message = "derived center clearance is not finite";
        return result;
    }
    try {
        result.center_space = original.buffer(-center_clearance);
    } catch (const std::exception& e) {
        result.error_message =
            std::string("failed to build center space: ") + e.what();
        return result;
    }
    result.component_count = result.center_space.size();
    result.requires_repositioning = result.component_count > 1;

    if (result.component_count == 0) {
        result.excluded_regions = original;
        result.excluded_area = result.original_area;
        result.excluded_area_ratio = 1.0;
        result.has_excluded_regions = true;
        result.exclusion_limit_exceeded =
            result.excluded_area_ratio >
            params.max_excluded_area_ratio + params.ratio_tolerance;
        result.analysis_valid = true;
        return result;
    }

    try {
        f2c::types::Cells recovered_components;
        result.coverage_components.reserve(result.component_count);
        for (std::size_t i = 0; i < result.component_count; ++i) {
            f2c::types::Cells center_component;
            center_component.addGeometry(result.center_space.getGeometry(i));
            auto coverage_component = center_component
                .buffer(footprint_radius)
                .intersection(original);
            result.coverage_components.push_back(coverage_component);
            for (std::size_t j = 0; j < coverage_component.size(); ++j) {
                recovered_components.addGeometry(
                    coverage_component.getGeometry(j));
            }
        }
        result.reachable_coverage = recovered_components
            .unionCascaded()
            .intersection(original);
        result.excluded_regions = original.difference(result.reachable_coverage);
    } catch (const std::exception& e) {
        result.error_message =
            std::string("failed to recover coverage target: ") + e.what();
        return result;
    }

    result.reachable_area = std::clamp(
        result.reachable_coverage.area(), 0.0, result.original_area);
    const double geometry_excluded_area = std::clamp(
        result.excluded_regions.area(), 0.0, result.original_area);
    result.excluded_area = result.original_area - result.reachable_area;
    const double area_tolerance = params.area_tolerance +
        params.ratio_tolerance * result.original_area;
    if (!std::isfinite(area_tolerance)) {
        result.error_message = "derived area tolerance is not finite";
        return result;
    }
    if (!std::isfinite(result.reachable_area) ||
        !std::isfinite(geometry_excluded_area) ||
        std::abs(result.excluded_area - geometry_excluded_area) >
            area_tolerance) {
        result.error_message =
            "C-space result violates the coverage area invariant";
        return result;
    }
    result.excluded_area_ratio = std::clamp(
        result.excluded_area / result.original_area, 0.0, 1.0);
    result.has_excluded_regions =
        result.excluded_area > area_tolerance;
    result.exclusion_limit_exceeded =
        result.excluded_area_ratio >
        params.max_excluded_area_ratio + params.ratio_tolerance;
    result.analysis_valid = true;
    return result;
}

}  // namespace yingshi
