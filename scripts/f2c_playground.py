#!/usr/bin/env python3
"""Fields2Cover Python Playground — 秒级验证算法参数，无需C++编译
用法: python3 f2c_playground.py [--snake|--boustro|--spiral] [yaml文件]
"""
import sys, os
sys.path.insert(0, os.path.expanduser("~/f2c_coverage_planner/install/lib/python3.10/site-packages"))
os.environ.setdefault("LD_LIBRARY_PATH", 
    os.path.expanduser("~/f2c_coverage_planner/install/lib") + ":" + 
    os.path.expanduser("~/f2c_coverage_planner/src/Fields2Cover/build/_deps/steering_functions-build") + ":" +
    os.path.expanduser("~/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/lib"))

import fields2cover as f2c

# ═══════ 参数区（改这里秒级试）═══════
ROBOT_W = 0.95
COV_W = 0.45
MID_HL = 0.20
NO_HL = 0.0
ANGLE = 0.0        # rad, 0°=水平
OVERLAP = 0.03
SORTER = "snake"   # boustrophedon | snake | spiral
# ═══════════════════════════════════

def load_field(yaml_path):
    import yaml
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    outer = f2c.LinearRing()
    for p in data["polygon"]:
        outer.addPoint(float(p[0]), float(p[1]))
    outer.addPoint(float(data["polygon"][0][0]), float(data["polygon"][0][1]))
    cell = f2c.Cell(outer)
    for hole in data.get("holes", []):
        ring = f2c.LinearRing()
        for p in hole:
            ring.addPoint(float(p[0]), float(p[1]))
        ring.addPoint(float(hole[0][0]), float(hole[0][1]))
        cell.addRing(ring)
    cells = f2c.Cells()
    cells.addGeometry(cell)
    return cells

def main():
    yaml_path = sys.argv[-1] if sys.argv[-1].endswith(".yaml") else \
        os.path.expanduser("~/f2c_coverage_planner/src/yingshi_robot/config/f2c_areas/notched_10m_with_center_hole.yaml")
    
    # Parse flags
    global SORTER
    for arg in sys.argv:
        if arg == "--snake": SORTER = "snake"
        if arg == "--boustro": SORTER = "boustrophedon"
        if arg == "--spiral": SORTER = "spiral"
    
    print(f"=== F2C Playground: {os.path.basename(yaml_path)} ===")
    print(f"Robot={ROBOT_W}m Cov={COV_W}m Mid={MID_HL} No={NO_HL} Overlap={OVERLAP}")
    print(f"Sorter: {SORTER}\n")

    robot = f2c.Robot(ROBOT_W, COV_W)
    field = load_field(yaml_path)
    print(f"Field area: {field.area():.1f} m²")

    # Headlands
    const_hl = f2c.HG_Const_gen()
    mid = const_hl.generateHeadlands(field, MID_HL * ROBOT_W)
    no = const_hl.generateHeadlands(mid, NO_HL * ROBOT_W)

    # Swaths (per-cell)
    sg = f2c.SG_BruteForce()
    swaths_by_cells = sg.generateSwaths(ANGLE, COV_W * (1 - OVERLAP), no)
    
    total_swaths = 0
    total_len = 0.0
    for ci in range(swaths_by_cells.size()):
        cell_sw = swaths_by_cells.at(ci)
        total_swaths += cell_sw.size()
        for si in range(cell_sw.size()):
            total_len += cell_sw.at(si).getLength()

    print(f"Raw swaths: {total_swaths} in {swaths_by_cells.size()} cells, {total_len:.0f}m")

    # Apply sorter & measure turns
    sorters = {
        "boustrophedon": f2c.RP_Boustrophedon(),
        "snake": f2c.RP_Snake(),
        "spiral": f2c.RP_Spiral(6),
    }
    s = sorters.get(SORTER, sorters["boustrophedon"])
    
    sorted_total = 0
    for ci in range(swaths_by_cells.size()):
        sorted_sw = s.genSortedSwaths(swaths_by_cells.at(ci), 0)
        sorted_total += sorted_sw.size()

    est_coverage = total_len * COV_W / field.area() * 100
    print(f"Sorted swaths: {sorted_total}")
    print(f"Est. turns: ~{sorted_total} (1 per swath)")
    print(f"Est. coverage: {total_len*COV_W:.0f}/{field.area():.0f} m² = {est_coverage:.1f}%")

if __name__ == "__main__":
    main()
