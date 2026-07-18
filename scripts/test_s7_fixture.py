import re
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
S7_PATH = (
    REPO_ROOT
    / "src"
    / "yingshi_robot"
    / "test_polygons"
    / "S7_S7_factory_workshop.yaml"
)
POINT_PATTERN = re.compile(
    r"\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\]"
)


def _point(line):
    match = POINT_PATTERN.search(line)
    if match is None:
        raise AssertionError(f"无法解析 S7 坐标行: {line!r}")
    return float(match.group(1)), float(match.group(2))


def _load_s7_geometry():
    """解析固定格式的 S7 夹具，避免测试额外依赖 PyYAML。"""
    polygon = []
    holes = []
    current_hole = None
    section = None

    for line in S7_PATH.read_text(encoding="utf-8").splitlines():
        if line == "polygon:":
            section = "polygon"
            continue
        if line == "holes:":
            section = "holes"
            continue
        if line.startswith("boundary_type:"):
            section = None
            continue

        if section == "polygon" and line.startswith("  - ["):
            polygon.append(_point(line))
        elif section == "holes" and line.startswith("  - - ["):
            current_hole = [_point(line)]
            holes.append(current_hole)
        elif section == "holes" and line.startswith("    - ["):
            if current_hole is None:
                raise AssertionError("S7 障碍顶点出现在首顶点之前")
            current_hole.append(_point(line))

    return polygon, holes


def _area(points):
    return abs(sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
    )) / 2.0


class S7FixtureTest(unittest.TestCase):
    def test_factory_scene_keeps_all_ten_obstacles(self):
        polygon, holes = _load_s7_geometry()

        self.assertEqual(len(polygon), 4)
        self.assertAlmostEqual(_area(polygon), 600.0)
        self.assertEqual(len(holes), 10)
        self.assertEqual([len(hole) for hole in holes], [4] * 9 + [5])
        self.assertAlmostEqual(sum(_area(hole) for hole in holes), 98.55)

    def test_five_small_pillars_cannot_be_silently_removed(self):
        _, holes = _load_s7_geometry()

        for pillar in holes[:5]:
            self.assertAlmostEqual(_area(pillar), 0.36)

    def test_production_batch_preserves_physical_holes_and_enables_cspace(self):
        batch_script = (REPO_ROOT / "scripts" / "batch_test_v2.sh").read_text(
            encoding="utf-8"
        )
        node_source = (
            REPO_ROOT
            / "src"
            / "yingshi_robot"
            / "src"
            / "polygon_planner_node.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("min_hole_area:=0.0", batch_script)
        self.assertIn("traversability_enabled:=true", batch_script)
        self.assertIn("filtered_holes = holes", node_source)
        self.assertNotIn("filterHolesByArea", node_source)


if __name__ == "__main__":
    unittest.main()
