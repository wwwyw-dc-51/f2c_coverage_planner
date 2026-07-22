#pragma once

#include <cstddef>
#include <vector>

#include <fields2cover.h>

namespace yingshi {

// Normalize polygon rings without changing the planning model:
// snap tiny coordinate noise, remove consecutive duplicates, close rings,
// and use CCW exterior/CW interior orientation.
f2c::types::LinearRing normalizeRing(
    const f2c::types::LinearRing& ring,
    bool counter_clockwise,
    double snap_tolerance = 1e-7);

f2c::types::Cell normalizeCell(
    const f2c::types::Cell& cell,
    double snap_tolerance = 1e-7);

f2c::types::Cells normalizeCells(
    const f2c::types::Cells& cells,
    double snap_tolerance = 1e-7);

std::vector<f2c::types::LinearRing> normalizeRings(
    const std::vector<f2c::types::LinearRing>& rings,
    double snap_tolerance = 1e-7);

}  // namespace yingshi
