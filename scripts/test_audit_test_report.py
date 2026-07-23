from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

from audit_test_report import (
    SCENARIO_POLYGON_FILES,
    _find_polygon_file,
    audit_report,
    main,
)


class AuditReportTest(unittest.TestCase):
    def test_clean_report_has_no_findings(self):
        report = {
            "path": [{"x": 1.0, "y": 5.0}, {"x": 9.0, "y": 5.0}],
            "swaths": [{"points": [{"x": 1.0, "y": 5.0}, {"x": 9.0, "y": 5.0}]}],
            "connections": [],
            "eval": {"turn_count": 0},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertEqual(findings, [])

    def test_reports_missing_final_path(self):
        report = {
            "path": [],
            "swaths": [{"points": [{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertIn("PATH_MISSING", {finding.code for finding in findings})

    def test_reports_missing_swath_evidence(self):
        report = {
            "path": [{"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 1.0}],
            "swaths": [],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertIn("EMPTY_SWATHS", {finding.code for finding in findings})

    def test_reports_turn_count_that_disagrees_with_final_path(self):
        report = {
            "path": [
                {"x": 1.0, "y": 1.0},
                {"x": 9.0, "y": 1.0},
                {"x": 9.0, "y": 9.0},
                {"x": 1.0, "y": 9.0},
            ],
            "swaths": [{"points": [{"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 1.0}]}],
            "eval": {"turn_count": 0},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertIn("TURN_COUNT_MISMATCH", {finding.code for finding in findings})

    def test_reports_path_length_outside_the_work_area(self):
        report = {
            "path": [{"x": -1.0, "y": 5.0}, {"x": 1.0, "y": 5.0}],
            "swaths": [{"points": [{"x": 0.0, "y": 5.0}, {"x": 1.0, "y": 5.0}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertIn("CENTERLINE_OUTSIDE", {finding.code for finding in findings})

    def test_treats_short_centerline_excursion_as_warning(self):
        report = {
            "path": [{"x": -0.2, "y": 5.0}, {"x": 0.2, "y": 5.0}],
            "swaths": [{"points": [{"x": 0.0, "y": 5.0}, {"x": 0.2, "y": 5.0}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)
        finding = next(item for item in findings if item.code == "CENTERLINE_OUTSIDE")

        self.assertEqual(finding.severity, "warning")

    def test_reports_robot_footprint_clearance_violation(self):
        report = {
            "path": [{"x": 1.0, "y": 0.2}, {"x": 9.0, "y": 0.2}],
            "swaths": [{"points": [{"x": 1.0, "y": 0.2}, {"x": 9.0, "y": 0.2}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon, robot_width=1.0)

        self.assertIn("FOOTPRINT_CLEARANCE", {finding.code for finding in findings})

    def test_default_robot_width_matches_planner_baseline(self):
        report = {
            "path": [{"x": 1.0, "y": 0.4}, {"x": 9.0, "y": 0.4}],
            "swaths": [{"points": [{"x": 1.0, "y": 0.4}, {"x": 9.0, "y": 0.4}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertNotIn("FOOTPRINT_CLEARANCE", {finding.code for finding in findings})

    def test_clearance_rounding_error_is_inside_tolerance(self):
        report = {
            "path": [
                {"x": 1.0, "y": 0.3749999711989731},
                {"x": 9.0, "y": 0.3749999711989731},
            ],
            "swaths": [{"points": [{"x": 1.0, "y": 0.375}, {"x": 9.0, "y": 0.375}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertNotIn("FOOTPRINT_CLEARANCE", {finding.code for finding in findings})

    def test_clearance_real_shortfall_is_still_reported(self):
        report = {
            "path": [
                {"x": 1.0, "y": 0.374},
                {"x": 9.0, "y": 0.374},
            ],
            "swaths": [{"points": [{"x": 1.0, "y": 0.374}, {"x": 9.0, "y": 0.374}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertIn("FOOTPRINT_CLEARANCE", {finding.code for finding in findings})

    def test_turn_merge_distance_is_configurable(self):
        report = {
            "path": [
                {"x": 1.0, "y": 1.0},
                {"x": 2.0, "y": 1.0},
                {"x": 2.0, "y": 1.6},
                {"x": 3.0, "y": 1.6},
            ],
            "swaths": [{"points": [{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}]}],
            "eval": {"turn_count": 1},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        merged_findings = audit_report(report, polygon, merge_distance=1.0)
        split_findings = audit_report(report, polygon, merge_distance=0.5)

        self.assertNotIn("TURN_COUNT_MISMATCH", {finding.code for finding in merged_findings})
        self.assertIn("TURN_COUNT_MISMATCH", {finding.code for finding in split_findings})

    def test_default_turn_merge_distance_matches_evaluator_baseline(self):
        report = {
            "path": [
                {"x": 1.0, "y": 1.0},
                {"x": 2.0, "y": 1.0},
                {"x": 2.0, "y": 1.6},
                {"x": 3.0, "y": 1.6},
            ],
            "swaths": [{"points": [{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}]}],
            "eval": {"turn_count": 1},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertNotIn("TURN_COUNT_MISMATCH", {finding.code for finding in findings})

    def test_report_turn_merge_distance_overrides_default(self):
        report = {
            "path": [
                {"x": 1.0, "y": 1.0},
                {"x": 2.0, "y": 1.0},
                {"x": 2.0, "y": 1.6},
                {"x": 3.0, "y": 1.6},
            ],
            "swaths": [{"points": [{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}]}],
            "eval": {"turn_count": 1, "turn_merge_distance": 1.0},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertNotIn("TURN_COUNT_MISMATCH", {finding.code for finding in findings})

    def test_reports_excessive_sub_centimeter_segments(self):
        report = {
            "path": [
                {"x": 1.0, "y": 1.0},
                {"x": 1.005, "y": 1.0},
                {"x": 2.005, "y": 1.0},
                {"x": 2.010, "y": 1.0},
            ],
            "swaths": [{"points": [{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertIn("TINY_SEGMENTS", {finding.code for finding in findings})

    def test_reports_retraced_path_length(self):
        report = {
            "path": [
                {"x": 1.0, "y": 5.0},
                {"x": 3.0, "y": 5.0},
                {"x": 1.0, "y": 5.0},
                {"x": 3.0, "y": 5.0},
            ],
            "swaths": [{"points": [{"x": 1.0, "y": 5.0}, {"x": 3.0, "y": 5.0}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertIn("RETRACE_LENGTH", {finding.code for finding in findings})

    def test_reports_retrace_with_different_segment_sampling(self):
        report = {
            "path": [
                {"x": 1.0, "y": 5.0},
                {"x": 3.0, "y": 5.0},
                {"x": 2.27, "y": 5.0},
                {"x": 1.0, "y": 5.0},
            ],
            "swaths": [{"points": [{"x": 1.0, "y": 5.0}, {"x": 3.0, "y": 5.0}]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertIn("RETRACE_LENGTH", {finding.code for finding in findings})

    def test_reports_connections_without_provenance(self):
        report = {
            "path": [{"x": 1.0, "y": 5.0}, {"x": 3.0, "y": 5.0}],
            "swaths": [{"points": [{"x": 1.0, "y": 5.0}, {"x": 3.0, "y": 5.0}]}],
            "connections": [{"from_cell": 0, "to_cell": 1, "path": [[1.0, 5.0], [3.0, 5.0]]}],
            "eval": {},
        }
        polygon = {"polygon": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}

        findings = audit_report(report, polygon)

        self.assertIn("CONNECTION_PROVENANCE", {finding.code for finding in findings})

    def test_cli_writes_markdown_for_every_scenario(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            batch = root / "test_results" / "batch"
            polygons = root / "src" / "yingshi_robot" / "test_polygons"
            batch.mkdir(parents=True)
            polygons.mkdir(parents=True)
            (batch / "S1_data.json").write_text(json.dumps({
                "path": [{"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}],
                "swaths": [],
                "eval": {},
            }), encoding="utf-8")
            (polygons / "S1_square.yaml").write_text(
                "polygon:\n  - [0, 0]\n  - [10, 0]\n  - [10, 10]\n  - [0, 10]\nholes: []\n",
                encoding="utf-8",
            )
            markdown = root / "audit.md"

            with redirect_stdout(io.StringIO()):
                exit_code = main([
                    str(batch), "--repo-root", str(root), "--markdown", str(markdown),
                ])

            self.assertEqual(exit_code, 1)
            self.assertIn("EMPTY_SWATHS", markdown.read_text(encoding="utf-8"))

    def test_cli_keeps_auditing_when_one_report_is_invalid_json(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            batch = root / "test_results" / "batch"
            polygons = root / "src" / "yingshi_robot" / "test_polygons"
            batch.mkdir(parents=True)
            polygons.mkdir(parents=True)
            (batch / "S1_data.json").write_text("{broken", encoding="utf-8")
            (polygons / "S1_square.yaml").write_text(
                "polygon:\n  - [0, 0]\n  - [10, 0]\n  - [10, 10]\n  - [0, 10]\nholes: []\n",
                encoding="utf-8",
            )
            markdown = root / "audit.md"

            with redirect_stdout(io.StringIO()):
                exit_code = main([
                    str(batch), "--repo-root", str(root), "--markdown", str(markdown),
                ])

            self.assertEqual(exit_code, 1)
            self.assertIn("REPORT_READ_ERROR", markdown.read_text(encoding="utf-8"))

    def test_explicit_mapping_covers_extended_scenarios(self):
        expected = {
            "S8",
            "N1_ring", "N2_oblique", "N3_dense", "N4_wmixed",
            "N5_ucorr", "N6_llarge", "N7_gate", "N8_ushape",
            "N9_wshelv", "N10_whoriz", "N11_whole", "N12_lware",
            "N13_robs",
        }
        self.assertTrue(expected.issubset(SCENARIO_POLYGON_FILES))

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for relative_path in SCENARIO_POLYGON_FILES.values():
                polygon_path = root / relative_path
                polygon_path.parent.mkdir(parents=True, exist_ok=True)
                polygon_path.write_text("polygon: []\nholes: []\n", encoding="utf-8")

            self.assertEqual(
                _find_polygon_file(root, "N3_dense"),
                root / SCENARIO_POLYGON_FILES["N3_dense"],
            )


if __name__ == "__main__":
    unittest.main()
